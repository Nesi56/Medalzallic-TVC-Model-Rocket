// pyro.h - igniter / MOSFET channels
//
// Hardware per channel: Teensy pin -> 100R -> logic-level N-MOSFET gate,
// 10k gate-to-GND pull-down (keeps it OFF while the Teensy boots),
// igniter between the ARMED pyro rail (+) and the MOSFET drain.
// In SIM_MODE the pin is never driven high.
#pragma once
#include "config.h"

static bool g_pyroArmed = false;   // software arm; cleared whenever we disarm

class PyroChannel {
public:
  int pin = -1;
  const char* name = "";
  bool needsArm = true;
  bool firing = false;
  bool hasFired = false;
  uint32_t firedAtMs = 0;
  uint32_t onMs = PYRO_FIRE_MS;

  void begin(int p, const char* n, bool requireArm = true) {
    pin = p;
    name = n;
    needsArm = requireArm;
    if (pin >= 0) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }

  bool fitted() const { return pin >= 0; }

  // returns true if the channel was actually energised
  bool fire(uint32_t nowMs, uint32_t durationMs = PYRO_FIRE_MS) {
    if (pin < 0 || hasFired) return false;
    if (needsArm && !g_pyroArmed) return false;
    firing = true;
    hasFired = true;
    firedAtMs = nowMs;
    onMs = durationMs;
#if SIM_MODE
    digitalWrite(pin, LOW);  // simulation: never energise a real igniter
#else
    digitalWrite(pin, HIGH);
#endif
    return true;
  }

  void update(uint32_t nowMs) {
    if (firing && (uint32_t)(nowMs - firedAtMs) >= onMs) off();
  }

  void off() {
    firing = false;
    if (pin >= 0) digitalWrite(pin, LOW);
  }

  void rearm() {  // allow the channel to be used again (new flight)
    off();
    hasFired = false;
  }
};

inline float readDividedVolts(int pin, float ratio) {
  if (pin < 0) return 0.0f;
  return analogRead(pin) * (3.3f / 4095.0f) * ratio;
}
