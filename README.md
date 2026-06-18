# Smart Water Tap

A hand-sensing automatic tap built on ESP32. A Time-of-Flight sensor detects a hand under the spout, opens a solenoid valve for a fixed duration, gives a visual warning before closing, and then locks out for a short cooldown. The system is designed to scale from a single standalone unit to a network of tap nodes reporting to a central master node over ESP-NOW, with the master bridging that data to a server and Telegram bot.

## Status

Currently in a single-unit testing phase (6-month trial). The local sensing/valve/LED logic is complete. The Master/Daughter networking layer is implemented but unpaired — MAC addresses and WiFi channel still need to be configured per the **Pairing** section below before ESP-NOW will work.

## How it works

A VL53L0X distance sensor continuously measures the gap under the spout. The firmware runs a non-blocking state machine:

```
IDLE → DISPENSING → WARNING → COOLDOWN → IDLE
```

- **IDLE** — valve closed, LEDs green, waiting for a hand.
- **DISPENSING** — hand detected, valve opens, LEDs blue. Runs for up to 60 seconds.
- **WARNING** — last 2 seconds of the dispense window, LEDs blink orange to signal the valve is about to close.
- **COOLDOWN** — valve closed, LEDs red, 5-second lockout before the cycle can restart.

If the hand is removed mid-cycle, a short grace period (2 s) is allowed before the valve closes early — this avoids closing the valve on a brief, normal hand movement. A physical button provides manual override: force-open from IDLE, or force-close from DISPENSING/WARNING. The button does nothing during COOLDOWN; that lockout is intentional.

## Hardware

One firmware image runs on every board. At boot, each board reads a single GPIO jumper to decide whether it's acting as the **Master** node (WiFi + Telegram/server bridge) or a **Daughter** node (the physical tap — sensor, valve, LEDs).

| Role | Jumper state |
|---|---|
| Daughter (tap) | GPIO13 floating (default, no jumper) |
| Master (gateway) | GPIO13 wired to GND |

### Bill of materials (per Daughter/tap unit)

| Component | Spec | Notes |
|---|---|---|
| ESP32-WROOM-32 DevKit | 38-pin | |
| VL53L0X | I2C ToF sensor | Replaces earlier IR sensor — distance-based, tunable threshold |
| Solenoid valve | 12V DC, Normally Closed, ½" BSP | NC is mandatory — fails safe (shut) on power loss |
| IRLZ44N MOSFET | Logic-level gate | Switches at 3.3V GPIO; IRF540N will not work here |
| WS2812B LED strip | 5V, 3 LEDs | Addressable, single data pin |
| LM7805 (TO-220) | Linear regulator | Steps 12V → 5V for the ESP32; avoids switching-converter EMI |
| Heatsink for LM7805 | — | Mandatory at this load (~2W dissipation) |
| 12V 2A DC adapter | 230V AC input | Powers the valve and feeds the LM7805 |
| Push button | Momentary, NO | Manual override |
| 1N4007 diode | — | Flyback protection across the solenoid coil |
| 100Ω resistor | — | MOSFET gate |
| 10kΩ resistor | — | MOSFET gate pull-down (keeps gate low during boot) |
| 100µF 25V electrolytic ×2 | — | LM7805 input and output filtering |
| 0.33µF ceramic cap | — | LM7805 input |
| 0.1µF ceramic cap | — | LM7805 output |
| IP65 enclosure | — | Non-negotiable near a water source |
| ½" BSP fittings ×2 | — | Valve inlet/outlet |

3.3V is taken directly from the ESP32 board's onboard regulator — no separate component needed.

### Pin map (Daughter)

| Function | GPIO |
|---|---|
| Role select jumper | 13 |
| Valve (MOSFET gate) | 26 |
| WS2812B data | 27 |
| Override button | 25 (INPUT_PULLUP) |
| ToF SDA | 21 |
| ToF SCL | 22 |

### Power chain

```
12V 2A adapter
       │
       ├──────────────── 12V rail → MOSFET drain → Solenoid → GND
       │
    LM7805 (+ filter caps)
       │
       └── 5V → ESP32 VIN → onboard LDO → 3.3V (internal, used by ToF/LEDs)
```

## Firmware

`Smart_Water_Tap.ino` is the single source flashed to every board.

### Required libraries (Arduino IDE → Library Manager)

- **FastLED**
- **VL53L0X** by Pololu (not the Adafruit one — different API)

### Key constants

| Constant | Value | Meaning |
|---|---|---|
| `VALVE_OPEN_DURATION` | 60000 ms | Total dispense time |
| `WARNING_DURATION` | 2000 ms | Blink window before close |
| `COOLDOWN_DURATION` | 5000 ms | Lockout after close |
| `GRACE_PERIOD` | 2000 ms | Allowed hand-absence before early close |
| `HAND_THRESHOLD_MM` | 200 mm | Distance under which a hand is considered present |

Tune `HAND_THRESHOLD_MM` on the bench: print raw ToF readings over Serial with and without a hand present, then set the threshold roughly halfway between the two.

## Networking: Master/Daughter pairing

ESP-NOW requires each board's actual hardware MAC address, which can't be chosen in advance. Pairing is a one-time, two-step process per pair of boards:

1. Flash `Smart_Water_Tap.ino` to both boards with the placeholder MAC values left as-is.
2. Open Serial Monitor on each board and note the line `[INFO] My MAC: ...`.
3. Paste the Master's MAC into `MASTER_MAC[]` and the Daughter's MAC into `DAUGHTER_MAC[]` (both constants live in the same file, used by whichever role a given board ends up playing).
4. Reflash both boards.
5. Note the WiFi channel the Master prints after connecting (`[WiFi] Channel: ...`), set `ESPNOW_CHANNEL` to that value, and reflash the Daughter — it never joins your router directly, so it has no other way of knowing which channel to listen on.

### `tap_protocol.h`

Both roles share this header, which must define:

```cpp
struct TapPacket {
  uint8_t  nodeID;
  uint8_t  valveOpen;
  uint8_t  state;
  uint32_t sessionRuntime;
  uint32_t totalRuntime;
  uint8_t  errorFlags;
  uint32_t timestamp;
};

struct MasterPacket {
  uint8_t command;
  uint8_t targetNodeID;
};
```

Since both roles now compile from the same file, there's no risk of the struct layout drifting out of sync between Master and Daughter builds.

## Safety notes

- The solenoid valve must be **Normally Closed (NC)**. If the ESP32 crashes or power is lost, the spring-return keeps the line shut rather than flooding.
- `closeValve()` is called unconditionally at the very start of `setup()`, before anything else runs, so the valve starts in a known-safe state on every boot.
- The button does nothing during COOLDOWN — this lockout is intentional and should not be removed.

## Known gaps / next steps

- No watchdog timer — if the ESP32 hangs, the valve stays in whatever state it was in when it froze.
- No fault state for ToF sensor failure beyond a boot-time halt; a runtime sensor dropout isn't currently detected.
- Master's loop has a stub comment for receiving remote shutdown commands from the server/Telegram bot — the polling/webhook logic isn't implemented yet.
- Tested with a single Daughter node so far; multi-daughter behavior (multiple `nodeID`s, peer list management on the Master) is unverified.