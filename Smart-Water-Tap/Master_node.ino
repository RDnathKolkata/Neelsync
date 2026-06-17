#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>       // or PubSubClient for MQTT
#include "tap_protocol.h"

//  Configgies
const char* WIFI_SSID     = "your_ssid"; // change
const char* WIFI_PASS     = "your_password"; // CHANGE
const char* SERVER_URL    = "https://yourserver.com/api/tap"; // CHANGEEE

uint8_t tapMAC[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66}; // Tap Node MAC

//ESP-NOW receive callback 
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  TapPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  // Forward to server
  forwardToServer(pkt);
}

// Send shutdown to Tap
void sendShutdown(uint8_t nodeID) {
  MasterPacket cmd;
  cmd.command      = 0x01;
  cmd.targetNodeID = nodeID;
  esp_now_send(tapMAC, (uint8_t*)&cmd, sizeof(cmd));
  Serial.println("[MASTER] Shutdown command sent");
}

// HTTP POST server
void forwardToServer(TapPacket &pkt) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");

char body[150];
snprintf(body, sizeof(body), 
    "{\"nodeID\":%d,\"valveOpen\":%d,\"state\":%d,\"sessionRuntime\":%lu,\"totalRuntime\":%lu,\"errorFlags\":%d,\"timestamp\":%lu}",
    pkt.nodeID, pkt.valveOpen, pkt.state, pkt.sessionRuntime, pkt.totalRuntime, pkt.errorFlags, pkt.timestamp
);
http.POST(body);
http.end();

}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);     // AP+STA allows ESP-NOW + WiFi simultaneously
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("[WiFi] Connected");

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);

  // Register Tap Node as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, tapMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.reconnect();
    delay(5000);
  }
  // Server can POST back a shutdown command here
  // Poll or use webhook → call sendShutdown(1)
}