#ifndef TAP_PROTOCOL_H
#define TAP_PROTOCOL_H

#include <Arduino.h>   // for uint8_t / uint32_t types

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

struct TapConfig {
  uint8_t  command;            // 0x02 = apply config, 0x03 = reset to defaults
  uint8_t  targetNodeID;
  uint32_t valveOpenDuration;
  uint32_t warningDuration;
  uint32_t cooldownDuration;
  uint32_t blinkInterval;
  uint32_t gracePeriod;
};

struct TapConfigACK {
  uint8_t  nodeID;
  uint8_t  accepted;
  uint32_t valveOpenDuration;
  uint32_t warningDuration;
  uint32_t cooldownDuration;
  uint32_t blinkInterval;
  uint32_t gracePeriod;
};

#endif


//RDnathKolkata