//  unified_tap.ino
//  Single firmware for both Tap Node and Master Node.

//  ROLE DETECTION:
//    GPIO ROLE_PIN pulled LOW via jumper then MASTER NODE
//    GPIO ROLE_PIN floating (INPUT_PULLUP) then TAP NODE

//  RDnathKolkata

#include <FastLED.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
#include "tap_protocol.h"

//  ROLE DETECTION PIN

#define ROLE_PIN        33    // Jumper this pin to GND → becomes Master Node
                              // Leave floating             → becomes Tap Node

bool isMaster = false;        // Set at boot, never changes


//  Shared configgies  (CHANGE these before flashing)

#define ESPNOW_CHANNEL  1

// Master Node network config
const char* WIFI_SSID  = "your_ssid";       // CHANGE
const char* WIFI_PASS  = "your_password";   // CHANGE
const char* SERVER_URL = "https://yourserver.com/api/tap";  // CHANGE

// These MACs must match the actual hardware
// Tap Node needs to know Master's MAC; Master needs to know Tap's MAC.
// Both are registered as peers at boot — the role decides which one is used.
uint8_t masterMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};  // CHANGE
uint8_t tapMAC[]    = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};   // CHANGE

//  TAP NODE — Pin definitions

#define RELAY_PIN       26    // Relay IN           (LOW = valve OPEN, active-low)
#define LED_DATA_PIN    27    // WS2812B data line
#define BUTTON_PIN      25    // Override button    (LOW = pressed, INPUT_PULLUP)

//  TAP NODE — LED config


#define NUM_LEDS        3
#define LED_BRIGHTNESS  100

CRGB leds[NUM_LEDS];

//  TAP NODE — Timing (remote-config-able, loaded from NVS on boot)


unsigned long VALVE_OPEN_DURATION = 45000UL;   // ms
unsigned long WARNING_DURATION    = 10000UL;   // ms
unsigned long COOLDOWN_DURATION   = 5000UL;    // ms
unsigned long BLINK_INTERVAL      = 300UL;     // ms
unsigned long GRACE_PERIOD        = 2000UL;    // ms

// Safe limits for remote config validation
#define MIN_VALVE_DURATION   5000UL
#define MAX_VALVE_DURATION   300000UL
#define MIN_WARNING          1000UL
#define MAX_WARNING          30000UL
#define MIN_COOLDOWN         2000UL
#define MAX_COOLDOWN         60000UL
#define MIN_BLINK            100UL
#define MAX_BLINK            1000UL
#define MIN_GRACE            500UL
#define MAX_GRACE            10000UL
#define DEBOUNCE_DELAY       50UL

//  TAP NODE — VL53L0X

VL53L0X tof;
#define HAND_THRESHOLD_MM   200

//  TAP NODE — State machine

enum State { IDLE, DISPENSING, WARNING, COOLDOWN };
State currentState = IDLE;

//  TAP NODE — Runtime variables [Everytime I add one, it feels like 2 more are needed to be added...]

uint32_t      sessionRuntime    = 0;
uint32_t      totalRuntime      = 0;
uint8_t       errorFlags        = 0;
unsigned long lastHandSeenTime  = 0;
unsigned long stateStartTime    = 0;
unsigned long lastBlinkTime     = 0;
unsigned long lastDebounceTime  = 0;
bool          blinkToggle       = false;
bool          lastRawButtonState = HIGH;
bool          stableButtonState  = HIGH;
bool          isManualOverride   = false;

// Periodic telemetry heartbeat during DISPENSING
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000UL;








//  MASTER NODE — Runtime variables


bool      serverPostPending = false;
TapPacket pendingPacket;
unsigned long lastPollTime  = 0;
const unsigned long POLL_INTERVAL = 5000UL;

//  SHARED — Forward declarations
//  (needed because onDataRecv is registered before role-specific functions are defined, and the compiler needs to know they exist)

void tap_onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
void master_onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
void enterIdle();
void enterDispensing();
void enterWarning();
void enterCooldown();
void sendTapPacket();

//  SHARED — Single ESP-NOW receive callback (dispatches by role)

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (isMaster) {
    master_onDataRecv(mac, data, len);
  } else {
    tap_onDataRecv(mac, data, len);
  }
}

//  TAP NODE — ESP-NOW receive

void tap_onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(MasterPacket)) {
    MasterPacket cmd;
    memcpy(&cmd, data, sizeof(cmd));
    if (cmd.command == 0x01 && cmd.targetNodeID == 1) {
      Serial.println("[ESP-NOW] REMOTE SHUTDOWN received");
      enterCooldown();
    }

  } else if (len == sizeof(TapConfig)) {
    TapConfig cfg;
    memcpy(&cfg, data, sizeof(cfg));

    TapConfigACK ack;
    ack.nodeID = 1;

    if (cfg.command == 0x03) {
      // Reset to firmware defaults
      VALVE_OPEN_DURATION = 45000UL;
      WARNING_DURATION    = 10000UL;
      COOLDOWN_DURATION   = 5000UL;
      BLINK_INTERVAL      = 300UL;
      GRACE_PERIOD        = 2000UL;
      ack.accepted = 1;
      Serial.println("[CONFIG] Defaults restored");

    } else if (cfg.command == 0x02 && cfg.targetNodeID == 1) {
      ack.accepted = applyConfig(cfg) ? 1 : 0;

    } else {
      ack.accepted = 0;
    }

    // Echo back current live values
    ack.valveOpenDuration = VALVE_OPEN_DURATION;
    ack.warningDuration   = WARNING_DURATION;
    ack.cooldownDuration  = COOLDOWN_DURATION;
    ack.blinkInterval     = BLINK_INTERVAL;
    ack.gracePeriod       = GRACE_PERIOD;

    esp_now_send(masterMAC, (uint8_t*)&ack, sizeof(ack));
  }
}

//  TAP NODE — Config validation and apply

bool applyConfig(const TapConfig &cfg) {
  if (cfg.valveOpenDuration < MIN_VALVE_DURATION || cfg.valveOpenDuration > MAX_VALVE_DURATION) return false;
  if (cfg.warningDuration   < MIN_WARNING        || cfg.warningDuration   > MAX_WARNING)        return false;
  if (cfg.cooldownDuration  < MIN_COOLDOWN       || cfg.cooldownDuration  > MAX_COOLDOWN)       return false;
  if (cfg.blinkInterval     < MIN_BLINK          || cfg.blinkInterval     > MAX_BLINK)          return false;
  if (cfg.gracePeriod       < MIN_GRACE          || cfg.gracePeriod       > MAX_GRACE)          return false;
  if (cfg.warningDuration   >= cfg.valveOpenDuration)                                           return false;

  VALVE_OPEN_DURATION = cfg.valveOpenDuration;
  WARNING_DURATION    = cfg.warningDuration;
  COOLDOWN_DURATION   = cfg.cooldownDuration;
  BLINK_INTERVAL      = cfg.blinkInterval;
  GRACE_PERIOD        = cfg.gracePeriod;

  Serial.printf("[CONFIG] Applied — valve:%lums warn:%lums cool:%lums blink:%lums grace:%lums\n",
    VALVE_OPEN_DURATION, WARNING_DURATION, COOLDOWN_DURATION, BLINK_INTERVAL, GRACE_PERIOD);
  return true;
}

//  TAP NODE — Valve helpers

void openValve()  { digitalWrite(RELAY_PIN, LOW);  Serial.println("[VALVE] OPEN");   }
void closeValve() { digitalWrite(RELAY_PIN, HIGH); Serial.println("[VALVE] CLOSED"); }

//  TAP NODE — LED helper

void setLEDs(CRGB colour) {
  fill_solid(leds, NUM_LEDS, colour);
  FastLED.show();
}

//  TAP NODE — VL53L0X read

bool readHandDetected() {
  uint16_t distance = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) {
    errorFlags |= 0x01;   // Bit 0 — ToF timeout
    return false;
  }
  errorFlags &= ~0x01;    // Clear bit 0 on successful read
  return distance < HAND_THRESHOLD_MM;
}

//  TAP NODE — State transitions

void enterIdle() {
  currentState = IDLE;
  setLEDs(CRGB::Green);
  Serial.println("[STATE] IDLE — ready");
  sendTapPacket();
}

void enterDispensing() {
  currentState     = DISPENSING;
  stateStartTime   = millis();
  lastHandSeenTime = millis();
  openValve();
  setLEDs(CRGB::Blue);
  Serial.println("[STATE] DISPENSING — clock started");
  sendTapPacket();
}

void enterWarning() {
  currentState  = WARNING;
  lastBlinkTime = millis();
  blinkToggle   = false;
  Serial.println("[STATE] WARNING — closing soon");
  sendTapPacket();
}

void enterCooldown() {
  sessionRuntime = millis() - stateStartTime;
  totalRuntime  += sessionRuntime;
  Serial.printf("[RUNTIME] Session: %lu ms  Total: %lu ms\n", sessionRuntime, totalRuntime);

  currentState     = COOLDOWN;
  stateStartTime   = millis();
  isManualOverride = false;
  closeValve();
  setLEDs(CRGB::Red);
  Serial.println("[STATE] COOLDOWN — lockout");
  sendTapPacket();
}

//  TAP NODE — Telemetry packet

void sendTapPacket() {
  TapPacket pkt;
  pkt.nodeID         = 1;
  pkt.valveOpen      = (currentState == DISPENSING || currentState == WARNING) ? 1 : 0;
  pkt.state          = (uint8_t)currentState;
  pkt.sessionRuntime = sessionRuntime;
  pkt.totalRuntime   = totalRuntime;
  pkt.errorFlags     = errorFlags;
  pkt.timestamp      = millis();
  esp_now_send(masterMAC, (uint8_t*)&pkt, sizeof(pkt));
}

//  TAP NODE — Button debounce

bool buttonJustPressed() {
  bool rawState     = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (rawState != lastRawButtonState) {
    lastDebounceTime = now;
  }
  lastRawButtonState = rawState;

  if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (rawState == LOW && stableButtonState == HIGH) {
      stableButtonState = LOW;
      return true;
    }
    if (rawState == HIGH) {
      stableButtonState = HIGH;
    }
  }
  return false;
}

//  MASTER NODE — ESP-NOW receive

void master_onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(TapPacket)) {
    memcpy(&pendingPacket, data, sizeof(pendingPacket));
    serverPostPending = true;   // Defer HTTP call to loop()

  } else if (len == sizeof(TapConfigACK)) {
    TapConfigACK ack;
    memcpy(&ack, data, sizeof(ack));
    Serial.printf("[MASTER] Config ACK from node %d — accepted:%d\n", ack.nodeID, ack.accepted);
    // TODO: forward ACK to server when server polling is implemented
  }
}

//  MASTER NODE — Send commands to Tap Node

void sendShutdown(uint8_t nodeID) {
  MasterPacket cmd;
  cmd.command      = 0x01;
  cmd.targetNodeID = nodeID;
  esp_now_send(tapMAC, (uint8_t*)&cmd, sizeof(cmd));
  Serial.println("[MASTER] Shutdown command sent");
}

void sendConfig(uint8_t nodeID, uint32_t valveDur, uint32_t warnDur,
                uint32_t coolDur, uint32_t blinkInt, uint32_t grace) {
  TapConfig cfg;
  cfg.command           = 0x02;
  cfg.targetNodeID      = nodeID;
  cfg.valveOpenDuration = valveDur;
  cfg.warningDuration   = warnDur;
  cfg.cooldownDuration  = coolDur;
  cfg.blinkInterval     = blinkInt;
  cfg.gracePeriod       = grace;
  esp_now_send(tapMAC, (uint8_t*)&cfg, sizeof(cfg));
  Serial.println("[MASTER] Config update sent");
}

void sendResetDefaults(uint8_t nodeID) {
  TapConfig cfg = {};
  cfg.command      = 0x03;
  cfg.targetNodeID = nodeID;
  esp_now_send(tapMAC, (uint8_t*)&cfg, sizeof(cfg));
  Serial.println("[MASTER] Reset defaults sent");
}

//  MASTER NODE — HTTP forward to server

void forwardToServer(TapPacket &pkt) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/tap/data");
  http.addHeader("Content-Type", "application/json");

  char body[160];
  snprintf(body, sizeof(body),
    "{\"nodeID\":%d,\"valveOpen\":%d,\"state\":%d,"
    "\"sessionRuntime\":%lu,\"totalRuntime\":%lu,"
    "\"errorFlags\":%d,\"timestamp\":%lu}",
    pkt.nodeID, pkt.valveOpen, pkt.state,
    pkt.sessionRuntime, pkt.totalRuntime,
    pkt.errorFlags, pkt.timestamp);

  int code = http.POST(body);
  Serial.printf("[MASTER] POST → server: HTTP %d\n", code);
  http.end();
}

//  MASTER NODE — Poll server for pending commands

void pollServerCommands() {
  if (millis() - lastPollTime < POLL_INTERVAL) return;
  lastPollTime = millis();

  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(String(SERVER_URL) + "/api/tap/commands/1");
  int code = http.GET();

  if (code == 200) {
    // Minimal JSON parse — no ArduinoJson dependency
    // Server returns: {"commands":[{"id":1,"command":"SHUTDOWN","payload":null}]}
    // Full ArduinoJson parsing can replace this block when library is available
    String body = http.getString();
    Serial.printf("[MASTER] Poll response: %s\n", body.c_str());

    if (body.indexOf("SHUTDOWN") >= 0)       sendShutdown(1);
    if (body.indexOf("RESET_DEFAULTS") >= 0) sendResetDefaults(1);
    // CONFIG_UPDATE parsing requires ArduinoJson — add when library included
  }
  http.end();
}

//  MASTER NODE — WiFi reconnect

void masterHandleWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MASTER] WiFi lost — reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(500);
    if (WiFi.status() == WL_CONNECTED) Serial.println("[MASTER] WiFi restored");
    else Serial.println("[MASTER] WiFi reconnect failed — will retry");
  }
}

//  SETUP — shared ESP-NOW init helper

void initESPNow(uint8_t *peerMAC, uint8_t channel) {
  WiFi.mode(isMaster ? WIFI_AP_STA : WIFI_STA);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed — halting");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = channel;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("[ESP-NOW] Initialised");
}

//  SETUP

void setup() {
  Serial.begin(115200);
  delay(100);   // Let serial settle before printing

  // --- Role detection ---
  pinMode(ROLE_PIN, INPUT_PULLUP);
  delay(10);    // Let pin settle
  isMaster = (digitalRead(ROLE_PIN) == LOW);

  Serial.printf("\n=== Tap Unified Firmware Boot ===\n");
  Serial.printf("Role: %s\n", isMaster ? "MASTER NODE" : "TAP NODE");

  if (isMaster) {
    //  Master Node setup 
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WiFi] Connecting");
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) Serial.println("[WiFi] Connected");
    else                                Serial.println("[WiFi] Failed — will retry in loop");

    initESPNow(tapMAC, ESPNOW_CHANNEL);

  } else {
    //  Tap Node setup 
    initESPNow(masterMAC, ESPNOW_CHANNEL);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    closeValve();   // Always start safe

    Wire.begin(21, 22);
    tof.setTimeout(500);
    if (!tof.init()) {
      Serial.println("[ERROR] VL53L0X not found — check wiring!");
      // Blink red 5 times as visual error indicator before halting
      FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
      FastLED.setBrightness(LED_BRIGHTNESS);
      for (int i = 0; i < 5; i++) {
        setLEDs(CRGB::Red); delay(300);
        setLEDs(CRGB::Black); delay(300);
      }
      while (true) delay(1000);
    }
    tof.startContinuous();
    Serial.println("[ToF] VL53L0X ready");

    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);

    enterIdle();
  }
}

//  LOOP

void loop() {
  if (isMaster) {
    //  Master Node loop 
    masterHandleWiFi();

    if (serverPostPending) {
      forwardToServer(pendingPacket);
      serverPostPending = false;
    }

    pollServerCommands();

  } else {
    // Tap Node loop 
    unsigned long now          = millis();
    bool          handDetected = readHandDetected();
    bool          buttonPressed = buttonJustPressed();

    if (handDetected) lastHandSeenTime = now;

    // Heartbeat telemetry during active dispensing
    if ((currentState == DISPENSING || currentState == WARNING) &&
        (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL)) {
      lastHeartbeatTime = now;
      sessionRuntime    = now - stateStartTime;
      sendTapPacket();
    }

    // Manual override button
    if (buttonPressed) {
      if (currentState == IDLE) {
        isManualOverride = true;
        Serial.println("[BTN] Force-open");
        enterDispensing();
        return;
      } else if (currentState == DISPENSING || currentState == WARNING) {
        Serial.println("[BTN] Force-close");
        enterCooldown();
        return;
      }
      // Button does nothing during COOLDOWN — lockout is intentional, DO NOT TOUCH
    }

    // State machine
    switch (currentState) {

      case IDLE:
        if (handDetected) enterDispensing();
        break;

      case DISPENSING: {
        if (!isManualOverride && (now - lastHandSeenTime > GRACE_PERIOD)) {
          Serial.println("[IR] Hand gone — closing early");
          enterCooldown();
          break;
        }
        if ((now - stateStartTime) >= VALVE_OPEN_DURATION - WARNING_DURATION) {
          enterWarning();
        }
        break;
      }

      case WARNING: {
        if (!isManualOverride && (now - lastHandSeenTime > GRACE_PERIOD)) {
          Serial.println("[IR] Hand gone during warning — closing early");
          enterCooldown();
          break;
        }
        if (now - lastBlinkTime >= BLINK_INTERVAL) {
          blinkToggle = !blinkToggle;
          setLEDs(blinkToggle ? CRGB::Orange : CRGB::Black);
          lastBlinkTime = now;
        }
        if ((now - stateStartTime) >= VALVE_OPEN_DURATION) {
          enterCooldown();
        }
        break;
      }

      case COOLDOWN: {
        if ((now - stateStartTime) >= COOLDOWN_DURATION) {
          enterIdle();
        }
        break;
      }
    }
  }
}

// RDnathKolkata
