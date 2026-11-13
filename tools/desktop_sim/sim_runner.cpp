// sim_runner.cpp - runs the real flight code on a PC
//
// The sketch is compiled with the stand-in Arduino/Teensy headers in mock/,
// time is faked (50 us per loop() call), and the commands ARM and LAUNCH
// are typed automatically. With SIM_MODE 1 the built-in simulator in
// sim.h flies the rocket.
//
// Usage: sim_runner [-q] [seed igniter_delay_s]
//   -q                 hide telemetry and messages, print only the result
//   seed, delay        random seed and "true" igniter delay for the sim
#include "Arduino.h"
#include "Wire.h"
#include "SD.h"

static uint64_t g_us = 0;
uint32_t millis() { return (uint32_t)(g_us / 1000); }
uint32_t micros() { return (uint32_t)g_us; }
void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
int g_pinState[64];
int g_pwm[64];
usb_serial_class Serial;
HardwareSerial Serial2;
TwoWire Wire;
SDClass SD;

#include "TVC_Flight_Computer.ino"

#if SIM_MODE
int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "-q")) Serial.quiet = true;
  setup();
  if (argc > 3) {
    sim.seed = atoi(argv[2]);
    sim.igniterDelayS = atof(argv[3]);
    sim.begin();
  }
  bool armed = false, launched = false, reported = false;
  uint32_t doneAt = 0;
  while (g_us < 120ull * 1000000ull) {
    loop();
    g_us += 50;
    // the igniter pins must never go high in SIM_MODE
    if (g_pinState[PIN_PYRO_ASCENT] || g_pinState[PIN_PYRO_LANDING]) {
      printf("FAIL: pyro pin went HIGH in SIM_MODE\n");
      return 2;
    }
    if (!armed && millis() > 3000) { Serial.inject("ARM\n"); armed = true; }
    if (!launched && state == ST_ARMED && millis() > 6000) { Serial.inject("LAUNCH\n"); launched = true; }
    if (!reported && state == ST_LANDING_IGNITION) {
      reported = true;
      printf("FIRE true h=%.2f v=%.2f | estimate h=%.2f v=%.2f | error h=%+.2f v=%+.2f\n", sim.pos.z,
             sim.vel.z, nav.h, nav.v, nav.h - sim.pos.z, nav.v - sim.vel.z);
    }
    if ((state == ST_LANDED || state == ST_ABORT) && flog.saved && !doneAt) doneAt = millis();
    if (doneAt && millis() > doneAt + 500) break;
  }
  printf("RESULT seed=%u delay=%.3f state=%s td=%d v=%.2f tilt=%.1f apogee=%.1f\n", (unsigned)sim.seed,
         sim.igniterDelayS, stateName(state), sim.touchedDown, sim.touchdownVel, sim.touchdownTilt, sim.maxAlt);
  return 0;
}
#else
// SIM_MODE 0: no sensors exist on a PC, so this only proves the hardware
// code paths compile and that the computer refuses to arm without sensors.
int main() {
  setup();
  Serial.inject("STATUS\nARM\n");
  for (int i = 0; i < 200000; i++) { loop(); g_us += 50; }
  printf("RESULT hardware-build state=%s\n", stateName(state));
  return state == ST_ERROR ? 0 : 1;
}
#endif
