// baro_bmp3xx.h - minimal non-blocking BMP388 / BMP390 driver
// Runs the sensor in NORMAL mode at 50 Hz (x8 pressure, x1 temperature,
// IIR filter OFF so it adds no lag - the altitude Kalman filter smooths it).
// Compensation formulas are the floating-point ones from the Bosch datasheet.
#pragma once
#include <Wire.h>
#include <math.h>

class BMP3XX {
public:
  uint8_t chipId = 0;
  float pressurePa = 0;
  float temperatureC = 0;

  bool begin(TwoWire& w, uint8_t addr) {
    wire = &w;
    address = addr;
    if (!readRegs(0x00, &chipId, 1)) return false;
    if (chipId != 0x50 && chipId != 0x60) return false;  // 0x50 BMP388, 0x60 BMP390
    writeReg(0x7E, 0xB6);                                // soft reset
    delay(10);
    uint8_t c[21];
    if (!readRegs(0x31, c, 21)) return false;
    parT1 = (double)(uint16_t)(c[1] << 8 | c[0]) * 256.0;              // / 2^-8
    parT2 = (double)(uint16_t)(c[3] << 8 | c[2]) / 1073741824.0;       // / 2^30
    parT3 = (double)(int8_t)c[4] / 281474976710656.0;                  // / 2^48
    parP1 = ((double)(int16_t)(c[6] << 8 | c[5]) - 16384.0) / 1048576.0;    // 2^14, 2^20
    parP2 = ((double)(int16_t)(c[8] << 8 | c[7]) - 16384.0) / 536870912.0;  // 2^14, 2^29
    parP3 = (double)(int8_t)c[9] / 4294967296.0;                       // / 2^32
    parP4 = (double)(int8_t)c[10] / 137438953472.0;                    // / 2^37
    parP5 = (double)(uint16_t)(c[12] << 8 | c[11]) * 8.0;              // / 2^-3
    parP6 = (double)(uint16_t)(c[14] << 8 | c[13]) / 64.0;             // / 2^6
    parP7 = (double)(int8_t)c[15] / 256.0;                             // / 2^8
    parP8 = (double)(int8_t)c[16] / 32768.0;                           // / 2^15
    parP9 = (double)(int16_t)(c[18] << 8 | c[17]) / 281474976710656.0; // / 2^48
    parP10 = (double)(int8_t)c[19] / 281474976710656.0;                // / 2^48
    parP11 = (double)(int8_t)c[20] / 36893488147419103232.0;           // / 2^65

    writeReg(0x1C, 0x03);  // OSR: pressure x8, temperature x1
    writeReg(0x1D, 0x02);  // ODR: 50 Hz
    writeReg(0x1F, 0x00);  // IIR filter off
    writeReg(0x1B, 0x33);  // PWR_CTRL: pressure + temp enabled, NORMAL mode
    delay(50);
    uint8_t err = 0;
    readRegs(0x02, &err, 1);
    if (err & 0x07) return false;  // fatal / command / config error
    for (int i = 0; i < 20 && !poll(); i++) delay(10);
    return pressurePa > 30000.0f && pressurePa < 110000.0;
  }

  // returns true when a NEW pressure sample has been read
  bool poll() {
    uint8_t status;
    if (!readRegs(0x03, &status, 1)) return false;
    if (!(status & 0x20)) return false;  // drdy_press
    uint8_t d[6];
    if (!readRegs(0x04, d, 6)) return false;
    // double precision: Teensy 4.1 has a hardware double FPU
    double up = (double)((uint32_t)d[2] << 16 | (uint32_t)d[1] << 8 | d[0]);
    double ut = (double)((uint32_t)d[5] << 16 | (uint32_t)d[4] << 8 | d[3]);
    // temperature
    double pd1 = ut - parT1;
    double pd2 = pd1 * parT2;
    double tLin = pd2 + pd1 * pd1 * parT3;
    temperatureC = (float)tLin;
    // pressure
    double t2 = tLin * tLin, t3 = t2 * tLin;
    double out1 = parP5 + parP6 * tLin + parP7 * t2 + parP8 * t3;
    double out2 = up * (parP1 + parP2 * tLin + parP3 * t2 + parP4 * t3);
    double up2 = up * up;
    double out3 = up2 * (parP9 + parP10 * tLin) + up2 * up * parP11;
    pressurePa = (float)(out1 + out2 + out3);
    return true;
  }

private:
  TwoWire* wire = 0;
  uint8_t address = 0x77;
  double parT1, parT2, parT3;
  double parP1, parP2, parP3, parP4, parP5, parP6, parP7, parP8, parP9, parP10, parP11;

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

// barometric altitude (m) of pressure p relative to reference pressure p0
inline float pressureToAltitude(float p, float p0) {
  return 44330.0f * (1.0f - powf(p / p0, 0.190295f));
}
