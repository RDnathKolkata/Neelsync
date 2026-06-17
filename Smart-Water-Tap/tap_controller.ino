#include <FastLED.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <esp_now.h>
#include <WiFi.h>
#include "tap_protocol.h"

uint8_t masterMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // Replace with real MAC

VL53L0X tof;
#define HAND_THRESHOLD_MM   200   // VL53L0X XSHUT detection in mm

// Pin definitions

#define RELAY_PIN       26    // Relay IN           (LOW = valve OPEN for active-low modules)
#define LED_DATA_PIN    27    // WS2812B data line
#define BUTTON_PIN      25    // Override button    (LOW = pressed, uses INPUT_PULLUP)

// LED config

#define NUM_LEDS        3     // How many WS2812B LEDs
#define LED_BRIGHTNESS  100   // 0–255

CRGB leds[NUM_LEDS];

//  Timing constants (in ms)

const unsigned long VALVE_OPEN_DURATION = 60000UL;  // 60 seconds total open time
const unsigned long WARNING_DURATION    = 10000UL;   // Last 2 s blinking warning
const unsigned long COOLDOWN_DURATION   = 5000UL;   // 5-second lockout after closing
const unsigned long BLINK_INTERVAL      = 300UL;    // LED blink rate during warning
const unsigned long DEBOUNCE_DELAY      = 50UL;     // Button debounce window (ms)
const unsigned long GRACE_PERIOD = 2000UL; // 2 seconds of "extra" water after hand leaves


//State machine

enum State {
  IDLE,       // Waiting for hand
  DISPENSING, // Valve open, countdown running
  WARNING,    // Last 2 s of dispense — LEDs blinking
  COOLDOWN    // Valve closed, locked out for 5 s
};

State currentState = IDLE;

// Runtime variables 

uint32_t sessionRuntime = 0;
uint32_t totalRuntime   = 0;
uint8_t  errorFlags     = 0;
unsigned long lastHandSeenTime = 0;        // Timestamp of the last successful IR hit
unsigned long stateStartTime  = 0;   // When the current DISPENSING cycle began
unsigned long lastBlinkTime   = 0;   // Tracks blink toggling during WARNING
unsigned long lastDebounceTime = 0;  // For button debounce
bool blinkToggle              = false;
bool lastRawButtonState       = HIGH;
bool stableButtonState        = HIGH;
bool isManualOverride = false;


// Helper: valve control

void openValve() {
  digitalWrite(RELAY_PIN, LOW);   // Active-low relay: LOW energises the coil
  Serial.println("[VALVE] OPEN");
}

void closeValve() {
  digitalWrite(RELAY_PIN, HIGH);  // HIGH de-energises — solenoid spring closes
  Serial.println("[VALVE] CLOSED");
}

//Helper: LED shortcuts

void setLEDs(CRGB colour) {
  fill_solid(leds, NUM_LEDS, colour);
  FastLED.show();
}

bool readHandDetected() {
  uint16_t distance = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return false; // Treat timeouts as no hand detected
  return distance < HAND_THRESHOLD_MM;
  
}

// State transition functions


void enterDispensing() {
  currentState   = DISPENSING;
  stateStartTime = millis();  // Anchor: all timing is relative to this
  lastHandSeenTime = millis(); 
  openValve();
  setLEDs(CRGB::Blue);
  Serial.println("[STATE] DISPENSING — 60 s clock started");
  sendTapPacket();
}

void enterWarning() {
  currentState  = WARNING;
  lastBlinkTime = millis();
  blinkToggle   = false;
  Serial.println("[STATE] WARNING — closing in 2 s");
  sendTapPacket();
}

void enterCooldown() {
  currentState   = COOLDOWN;
  stateStartTime = millis();  // Reset anchor for cooldown timer
  closeValve();
  sessionRuntime = millis() - stateStartTime;
  totalRuntime += sessionRuntime;
  setLEDs(CRGB::Red);
  Serial.println("[STATE] COOLDOWN — 5 s lockout");
}

// Button debounce 
// Returns true on the falling edge (button just pressed, after debounce).

bool buttonJustPressed() {
  bool rawState = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  if (rawState != lastRawButtonState) {
    lastDebounceTime = now;           // State changed — reset debounce timer
  }
  lastRawButtonState = rawState;

  if ((now - lastDebounceTime) > DEBOUNCE_DELAY) {
    // Signal has been stable for long enough
    if (rawState == LOW && stableButtonState == HIGH) {
      // It just settled to LOW (pressed)
      stableButtonState = LOW;
      return true;                    // ← rising edge confirmed
    }
    if (rawState == HIGH) {
      stableButtonState = HIGH;       // Button released, reset
    }
  }
  return false;
}

//  Setup 

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Tap Controller Boot ===");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Internal pull-up; button connects pin to GND

  closeValve();  // Always start safe — valve shut

  Wire.begin(21, 22);
tof.setTimeout(500);
if (!tof.init()) {
  Serial.println("[ERROR] VL53L0X not found — check wiring!");
  while (true) { delay(1000); }
}
tof.startContinuous();
Serial.println("[ToF] VL53L0X ready");

  FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);

  enterIdle();
}


void enterIdle() {
  currentState = IDLE;
  setLEDs(CRGB::Green);
  Serial.println("[STATE] IDLE — ready");
  sendTapPacket();
}

//send packet on every state change + every N seconds during DISPENSING
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

// Receive callback and listen for shutdown
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  MasterPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.command == 0x01 && cmd.targetNodeID == 1) {
    Serial.println("[ESP-NOW] REMOTE SHUTDOWN received");
    enterCooldown();
  }
}


// Main loop

void loop() {
  unsigned long now = millis();
  bool handDetected = readHandDetected(); 
  bool buttonPressed = buttonJustPressed();

  // Update the "last seen" timer whenever a hand is detected
  if (handDetected) {
    lastHandSeenTime = now;
  }

  //  Manual override button 
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
    // Button does nothing during COOLDOWN, lockout is intentional, DO NOT TOUCH
  }

  //  State machine logic
  switch (currentState) {

    case IDLE:
      if (handDetected) {
        enterDispensing();
      }
      break;

    case DISPENSING: {
        // Logic: Only close if the hand has been gone for LONGER than the grace period
        if (!isManualOverride && (now - lastHandSeenTime > GRACE_PERIOD)) {
            Serial.println("[IR] Hand gone too long — closing early");
            enterCooldown();
            break;
        }

        unsigned long elapsed = now - stateStartTime;
        if (elapsed >= VALVE_OPEN_DURATION - WARNING_DURATION) {
            enterWarning();
        }
        break;

       }

    case WARNING: {
        if (!isManualOverride && (now - lastHandSeenTime > GRACE_PERIOD)) {
            Serial.println("[IR] Hand gone too long during warning — closing early");
            enterCooldown();
            break;
        }
      unsigned long elapsed = now - stateStartTime; // relative to dispense start

      // Blink the LEDs (non-blocking)
      if (now - lastBlinkTime >= BLINK_INTERVAL) {
        blinkToggle = !blinkToggle;
        setLEDs(blinkToggle ? CRGB::Orange : CRGB::Black);
        lastBlinkTime = now;
      }

      // Full 60 s elapsed, so closed
      if (elapsed >= VALVE_OPEN_DURATION) {
        enterCooldown();
      }
      break;
    }

    case COOLDOWN: {
      unsigned long elapsed = now - stateStartTime; // Relative to cooldown start
      if (elapsed >= COOLDOWN_DURATION) {
        enterIdle();
      }
      break;
    }
  }
}

// RDnathKolkata