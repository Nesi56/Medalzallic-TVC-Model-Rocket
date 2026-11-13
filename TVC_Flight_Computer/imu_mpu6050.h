// imu_mpu6050.h - minimal MPU6050 driver (no library needed)
// +/-16 g accel, +/-1000 deg/s gyro, 1 kHz sample rate, 98 Hz low-pass
#pragma once
#include <Wire.h>
#include "flight_types.h"

class MPU6050 {
public:
  uint8_t whoAmI = 0;
  bool saturated = false;

  bool begin(TwoWire& w, uint8_t addr) {
    wire = &w;
    address = addr;
    if (!writeReg(0x6B, 0x80)) return false;   // PWR_MGMT_1: reset
    delay(100);
    writeReg(0x6B, 0x01);                      // clock = PLL on gyro X
    delay(10);
    writeReg(0x19, 0x00);                      // SMPLRT_DIV: 1 kHz
    writeReg(0x1A, 0x02);                      // CONFIG: DLPF 94/98 Hz (~3 ms delay)
    writeReg(0x1B, 0x10);                      // GYRO_CONFIG: +/-1000 dps
    writeReg(0x1C, 0x18);                      // ACCEL_CONFIG: +/-16 g
    delay(10);
    uint8_t id;
    if (!readRegs(0x75, &id, 1)) return false;
    whoAmI = id;
    // genuine MPU6050 = 0x68; many clones report 0x70/0x72/0x98 but work
    float a[3], g[3];
    return readRaw(a, g);
  }

  // accel in g, gyro in deg/s, SENSOR axes
  bool readRaw(float accelG[3], float gyroDps[3]) {
    uint8_t b[14];
    if (!readRegs(0x3B, b, 14)) return false;
    int16_t r[7];
    saturated = false;
    for (int i = 0; i < 7; i++) {
      r[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
      if (i != 3 && (r[i] == 32767 || r[i] == -32768)) saturated = true;
    }
    for (int i = 0; i < 3; i++) {
      accelG[i] = r[i] / 2048.0f;     // +/-16 g
      gyroDps[i] = r[i + 4] / 32.8f;  // +/-1000 dps
    }
    return true;
  }

private:
  TwoWire* wire = 0;
  uint8_t address = 0x68;

  bool writeReg(uint8_t reg, uint8_t val) {
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(val);
    return wire->endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    wire->beginTransmission(address);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) return false;
    if (wire->requestFrom(address, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = wire->read();
    return true;
  }
};
