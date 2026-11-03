#pragma once
#include "Arduino.h"
class TwoWire {
public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(bool = true) { return 2; }  // NACK: no device on the desktop
  uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
  int read() { return 0; }
};
extern TwoWire Wire;
