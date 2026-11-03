// attitude.h - quaternion attitude estimator
//
// On the pad the accelerometer (gravity) keeps the estimate level-true.
// In flight the accelerometer only sees thrust/drag, so attitude comes from
// integrating the gyro alone - which is why the gyro bias calibration done
// at ARM time matters so much.
#pragma once
#include "flight_types.h"

class AttitudeEstimator {
public:
  Quat q = {1, 0, 0, 0};          // body -> world
  Vec3 gyroBiasDps = {0, 0, 0};
  float accelScale = 1.0f;        // makes |accel| read exactly 1 g on the pad

  void initFromAccel(Vec3 accelG) { q = qFromUpVector(accelG); }

  // gyro in deg/s (bias NOT yet removed), accel in g, body axes
  void update(Vec3 gyroDps, Vec3 accelG, float dt, bool useAccel) {
    Vec3 w = (gyroDps - gyroBiasDps) * DEG2RAD;
    if (useAccel) {
      float a = norm(accelG);
      if (a > 0.85f && a < 1.15f) {
        // rotate the estimate so that its "up" matches the measured "up"
        Vec3 err = cross(normalized(accelG), upInBody());
        w = w + err * 2.0f;
      }
    }
    q = qintegrate(q, w, dt);
  }

  // world "up" expressed in body axes. (0,0,1) = perfectly vertical
  Vec3 upInBody() const { return qrotateInv(q, v3(0, 0, 1)); }

  float tiltDeg() const {
    return acosf(clampf(upInBody().z, -1.0f, 1.0f)) * RAD2DEG;
  }
  // tilt about body X / body Y - these map straight onto the two servos and
  // stay correct when the rocket rolls
  float tiltXDeg() const { Vec3 u = upInBody(); return atan2f(u.y, u.z) * RAD2DEG; }
  float tiltYDeg() const { Vec3 u = upInBody(); return atan2f(-u.x, u.z) * RAD2DEG; }

  // vertical (world up) acceleration in m/s^2 with gravity removed
  float verticalAccel(Vec3 accelG) const {
    Vec3 aw = qrotate(q, accelG * accelScale);
    return (aw.z - 1.0f) * G0;
  }
};
