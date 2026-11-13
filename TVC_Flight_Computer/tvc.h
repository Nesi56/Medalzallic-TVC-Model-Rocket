// tvc.h - thrust vector control: PID per axis + gimbal servo output
//
// Sign convention: a POSITIVE gimbal angle about an axis rotates the motor
// positively (right-hand rule) about that body axis. With that convention
//   gimbal = +KP*tilt + KI*integral + KD*rate
// pushes the tail toward the side the nose is leaning, which rights the
// rocket. Physical servo direction is fixed with SERVO_X_DIR / SERVO_Y_DIR.
#pragma once
#include "config.h"
#include "flight_types.h"

// write a servo pulse using Teensy hardware PWM (analogWriteResolution 15)
inline void servoWriteUs(int pin, float us, float hz) {
  if (pin < 0) return;
  float counts = us * hz * 1e-6f * 32768.0f;
  analogWrite(pin, (int)(counts + 0.5f));
}

class TVCController {
public:
  float gimbalX = 0, gimbalY = 0;  // commanded gimbal angles after limits (deg)
  float iX = 0, iY = 0;            // integrals of tilt (deg*s)

  void begin() {
    analogWriteResolution(15);
    if (PIN_SERVO_X >= 0) analogWriteFrequency(PIN_SERVO_X, SERVO_PWM_HZ);
    if (PIN_SERVO_Y >= 0) analogWriteFrequency(PIN_SERVO_Y, SERVO_PWM_HZ);
    center();
  }

  void resetIntegral() { iX = 0; iY = 0; }

  // tilt in deg, rate in deg/s (body X / body Y)
  void update(float tiltX, float tiltY, float rateX, float rateY, float dt, bool integrate) {
    if (integrate && TVC_KI > 0.0f) {
      float lim = TVC_I_LIMIT_DEG / TVC_KI;
      iX = clampf(iX + tiltX * dt, -lim, lim);
      iY = clampf(iY + tiltY * dt, -lim, lim);
    }
    float cx = TVC_KP * tiltX + TVC_KI * iX + TVC_KD * rateX;
    float cy = TVC_KP * tiltY + TVC_KI * iY + TVC_KD * rateY;
    setGimbal(cx, cy);
  }

  void center() { setGimbal(0, 0); }

  void setGimbal(float gx, float gy) {
    // limit the TOTAL deflection so diagonal commands can't exceed the mount
    float mag = sqrtf(gx * gx + gy * gy);
    if (mag > TVC_MAX_GIMBAL_DEG) {
      float k = TVC_MAX_GIMBAL_DEG / mag;
      gx *= k;
      gy *= k;
    }
    gimbalX = gx;
    gimbalY = gy;
    float usX = SERVO_X_CENTER_US + SERVO_X_DIR * gx * GIMBAL_TO_SERVO_RATIO * SERVO_US_PER_DEG;
    float usY = SERVO_Y_CENTER_US + SERVO_Y_DIR * gy * GIMBAL_TO_SERVO_RATIO * SERVO_US_PER_DEG;
    servoWriteUs(PIN_SERVO_X, clampf(usX, SERVO_MIN_US, SERVO_MAX_US), SERVO_PWM_HZ);
    servoWriteUs(PIN_SERVO_Y, clampf(usY, SERVO_MIN_US, SERVO_MAX_US), SERVO_PWM_HZ);
  }
};
