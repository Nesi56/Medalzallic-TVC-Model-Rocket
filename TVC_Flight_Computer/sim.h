// sim.h - bench physics simulation (only compiled when SIM_MODE is 1)
//
// Replaces the MPU6050 and barometer with a simulated rocket: two motors on
// a gimbal with servo lag, gravity, drag, thrust misalignment, gusts,
// uncontrolled roll, sensor noise/bias, igniter delays and the E16-4
// ejection charge. Lets you run complete flights at your desk.
#pragma once
#include "config.h"
#include "flight_types.h"
#include "motor_curves.h"

class RocketSim {
public:
  // truth state (world: z up)
  Vec3 pos = {0, 0, 0}, vel = {0, 0, 0};
  Quat q = {1, 0, 0, 0};
  Vec3 omega = {0, 0, 0};  // body rad/s
  float t = 0;
  bool onPad = true, touchedDown = false;
  float touchdownVel = 0, touchdownTilt = 0, maxAlt = 0, maxAltAfterLanding = 0;
  float gimX = 0, gimY = 0;  // actual gimbal (deg)
  float igniterDelayS = SIM_IGNITER_DELAY_S;
  uint32_t seed = SIM_SEED;

  RocketSim() : ascent(ASCENT_CURVE, CURVE_POINTS(ASCENT_CURVE)),
                landing(LANDING_CURVE, CURVE_POINTS(LANDING_CURVE)) {}

  void begin() {
    rng = seed ? seed : 1;
    Vec3 axis = v3(0.6f, 0.8f, 0);
    float a = SIM_INITIAL_TILT_DEG * DEG2RAD;
    Quat tq = {cosf(a / 2), axis.x * sinf(a / 2), axis.y * sinf(a / 2), 0};
    q = tq;
    gyroBias = v3(SIM_GYRO_BIAS_DPS, -0.6f * SIM_GYRO_BIAS_DPS, 0.4f * SIM_GYRO_BIAS_DPS);
    accelBias = v3(0.015f, -0.01f, 0.02f);
  }

  // stand-in for pressing a launch controller button (ONBOARD_ASCENT_IGNITION 0)
  void externalIgnite() { if (ascentIgnite < 0) ascentIgnite = t + igniterDelay(); }

  // call once per control loop. pyro flags tell the sim when igniters got power
  void step(float dt, float cmdGimX, float cmdGimY, bool ascentFired, bool landingFired) {
    if (ascentFired && ascentIgnite < 0) ascentIgnite = t + igniterDelay();
    if (landingFired && landingIgnite < 0) landingIgnite = t + igniterDelay();
    const int sub = 4;
    float h = dt / sub;
    for (int i = 0; i < sub; i++) substep(h, cmdGimX, cmdGimY);
  }

  void readImu(Vec3& accelG, Vec3& gyroDps) {
    accelG = lastSpecificForce * (1.0f / G0) + accelBias +
             v3(gauss() * 0.01f, gauss() * 0.01f, gauss() * 0.01f);
    if (ejectionKick > 0) accelG = accelG + v3(0, 0, ejectionKick);
    gyroDps = omega * RAD2DEG + gyroBias + v3(gauss() * 0.05f, gauss() * 0.05f, gauss() * 0.05f);
  }

  // 50 Hz barometer
  bool readBaro(float& pressurePa) {
    if (t - lastBaroT < 0.02f) return false;
    lastBaroT = t;
    float alt = pos.z;
    if (ejectionKick > 0) alt -= 6.0f;  // pressure spike inside the airframe
    pressurePa = 101325.0f * powf(1.0f - alt / 44330.0f, 5.255f) + gauss() * 2.0f;
    return true;
  }

  float tiltDeg() const {
    Vec3 nose = qrotate(q, v3(0, 0, 1));
    return acosf(clampf(nose.z, -1, 1)) * RAD2DEG;
  }

private:
  ThrustCurve ascent, landing;
  float ascentIgnite = -1, landingIgnite = -1;
  float ascentImpulse = 0, landingImpulse = 0;
  float burnoutT = -1, lastBaroT = -1;
  float ejectionKick = 0;
  bool ejected = false;
  Vec3 lastSpecificForce = {0, 0, G0};
  Vec3 gyroBias, accelBias;
  Vec3 dist = {0, 0, 0};
  uint32_t rng = 1;
  float pwmTimer = 0, latchedX = 0, latchedY = 0;

  float rnd() {  // 0..1
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return (rng & 0xFFFFFF) / 16777216.0f;
  }
  float gauss() {
    float u = rnd() + 1e-7f, v = rnd();
    return sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * v);
  }
  float igniterDelay() { return igniterDelayS + (rnd() * 2 - 1) * SIM_IGNITER_JITTER_S; }

  float mass() const {
    return LIFTOFF_MASS_KG - ASCENT_PROP_MASS_KG * fminf(1.0f, ascentImpulse / ascent.totalImpulse) -
           LANDING_PROP_MASS_KG * fminf(1.0f, landingImpulse / landing.totalImpulse);
  }

  void substep(float dt, float cmdX, float cmdY) {
    t += dt;
    // ---- servos: PWM frame latency + speed limit ----
    pwmTimer += dt;
    if (pwmTimer >= 1.0f / SERVO_PWM_HZ) { pwmTimer = 0; latchedX = cmdX; latchedY = cmdY; }
    float maxStep = SIM_SERVO_DPS / GIMBAL_TO_SERVO_RATIO * dt;
    gimX += clampf(latchedX - gimX, -maxStep, maxStep);
    gimY += clampf(latchedY - gimY, -maxStep, maxStep);

    // ---- motors ----
    float T = 0;
    if (ascentIgnite >= 0 && t >= ascentIgnite) {
      float ta = t - ascentIgnite;
      float f = ascent.thrustAt(ta);
      ascentImpulse += f * dt;
      T += f;
      if (ta > ascent.burnTime && burnoutT < 0) burnoutT = t;
    }
    if (landingIgnite >= 0 && t >= landingIgnite) {
      float f = landing.thrustAt(t - landingIgnite);
      landingImpulse += f * dt;
      T += f;
    }
    // E16-4 ejection charge: brief kick + baro spike
    ejectionKick = 0;
    if (SIM_EJECTION_DELAY_S >= 0 && burnoutT >= 0) {
      float te = t - burnoutT - SIM_EJECTION_DELAY_S;
      if (te >= 0 && te < 0.03f) ejectionKick = 4.0f;
    }

    float m = mass();
    float gx = (gimX + SIM_THRUST_MISALIGN_DEG) * DEG2RAD;
    float gy = (gimY - 0.5f * SIM_THRUST_MISALIGN_DEG) * DEG2RAD;
    // thrust direction in body frame for gimbal angles about X then Y
    Vec3 dir = v3(cosf(gx) * sinf(gy), -sinf(gx), cosf(gx) * cosf(gy));
    Vec3 Fb = dir * T;
    Vec3 r = v3(0, 0, -SIM_GIMBAL_ARM_M);
    Vec3 torque = cross(r, Fb);
    // gusts: slowly varying random torque (only once flying)
    if (!onPad) {
      dist = dist * 0.995f + v3(gauss(), gauss(), 0) * (SIM_DIST_TORQUE_NM * 0.1f);
      torque = torque + dist;
    }
    // aerodynamic damping of pitch/yaw rates
    float speed = norm(vel);
    torque = torque - v3(omega.x, omega.y, 0) * (0.002f * speed);

    Vec3 Fw = qrotate(q, Fb);
    Vec3 drag = vel * (-0.5f * 1.225f * SIM_CDA_M2 * speed);
    Vec3 acc = (Fw + drag) * (1.0f / m) + v3(0, 0, -G0);

    if (onPad) {
      if (Fw.z > m * G0) onPad = false;  // lift-off
      else acc = v3(0, 0, 0);
    }
    if (touchedDown) acc = v3(0, 0, 0);

    if (!onPad && !touchedDown) {
      omega.x += torque.x / SIM_MOI_KGM2 * dt;
      omega.y += torque.y / SIM_MOI_KGM2 * dt;
      omega.z = SIM_ROLL_RATE_DPS * DEG2RAD;
      q = qintegrate(q, omega, dt);
      vel = vel + acc * dt;
      pos = pos + vel * dt;
      if (pos.z > maxAlt) maxAlt = pos.z;
      if (landingIgnite >= 0 && pos.z > maxAltAfterLanding) maxAltAfterLanding = pos.z;
      if (pos.z <= LANDING_GROUND_ALT_M && vel.z < 0) {
        touchedDown = true;
        touchdownVel = vel.z;
        touchdownTilt = tiltDeg();
        pos.z = LANDING_GROUND_ALT_M;
        vel = v3(0, 0, 0);
        omega = v3(0, 0, 0);
        Serial.printf("SIM: TOUCHDOWN %.2f m/s  tilt %.1f deg  t=%.2f s  drift %.1f m  true apogee %.1f m\n",
                      touchdownVel, touchdownTilt, t, sqrtf(pos.x * pos.x + pos.y * pos.y), maxAlt);
      }
    } else {
      omega = v3(0, 0, 0);
    }
    // what an accelerometer feels (specific force) in body axes
    Vec3 sf = (onPad || touchedDown) ? v3(0, 0, G0) : acc + v3(0, 0, G0);
    lastSpecificForce = qrotateInv(q, sf);
  }
};
