// landing.h - landing-burn ignition predictor
//
// A solid motor cannot be throttled, so the only control is WHEN to light
// it. Every loop after apogee the computer simulates "what happens if the
// igniter gets power right now?" (igniter delay, the motor's thrust curve,
// touchdown, any hop on leftover thrust) and also "...one loop later?".
// It fires at the moment waiting any longer would make the touchdown harder.
#pragma once
#include "config.h"
#include "flight_types.h"
#include "motor_curves.h"

struct LandingPrediction {
  float touchdownSpeed;  // worst ground impact speed if fired now (m/s, positive)
  bool stops;            // reaches zero velocity in the air before touching down
  float stopAlt;         // height where it first stops (m above aim point)
  float hop;             // highest point after the first stop (m)
};

inline float dragAccel(float v, float mass) {
  if (LANDING_DRAG_CDA_M2 <= 0.0f) return 0.0f;
  return -0.5f * 1.225f * LANDING_DRAG_CDA_M2 * v * fabsf(v) / mass;
}

// Simulates the WHOLE outcome of powering the igniter right now: igniter
// delay, burn, touchdown, any hop back up on leftover thrust, and the fall
// after burnout. h: height above landing surface (m), v: vertical velocity
// (m/s, + up).
inline LandingPrediction predictIgnitionNow(const ThrustCurve& motor, float h, float v,
                                            float mass, float cosTilt, float igniterDelay) {
  LandingPrediction p = {0, false, 0, 0};
  const float dt = 0.002f;
  h -= LANDING_AIM_ALT_M;
  // 1) igniter delay: free fall
  for (float t = 0; t < igniterDelay; t += dt) {
    v += (-G0 + dragAccel(v, mass)) * dt;
    h += v * dt;
    if (h <= 0) { p.touchdownSpeed = -v; return p; }
  }
  // 2) burn, including touchdowns and hops on leftover thrust
  int hint = 0;
  float impulse = 0;
  bool onGround = false;
  for (float t = 0; t < motor.burnTime; t += dt) {
    float T = motor.thrustAt(t, &hint);
    impulse += T * dt;
    float m = mass - LANDING_PROP_MASS_KG * fminf(1.0f, impulse / motor.totalImpulse);
    if (onGround) {
      if (T * cosTilt <= m * G0) continue;  // sitting on its legs
      onGround = false;                     // leftover thrust lifts it off again
    }
    v += (T * cosTilt / m - G0 + dragAccel(v, m)) * dt;
    h += v * dt;
    if (!p.stops && v >= 0) { p.stops = true; p.stopAlt = h; }
    if (p.stops && h > p.hop) p.hop = h;
    if (h <= 0 && v < 0) {
      if (-v > p.touchdownSpeed) p.touchdownSpeed = -v;
      h = 0;
      v = 0;
      onGround = true;
    }
  }
  // 3) burned out in the air: free fall to the ground
  if (!onGround) {
    float s = sqrtf(fmaxf(0.0f, v * v + 2.0f * G0 * fmaxf(0.0f, h)));
    if (s > p.touchdownSpeed) p.touchdownSpeed = s;
  }
  return p;
}

// Fire when lighting now gives a softer worst-case touchdown than lighting
// a moment later. 'now' receives the fire-now prediction for logging.
// Touchdown speed changes by several m/s per 20 ms of timing error, so the
// look-ahead step is kept short.
inline bool landingFireDecision(const ThrustCurve& motor, float h, float v, float mass,
                                float cosTilt, LandingPrediction* now) {
  const float d = 1.0f / LOOP_HZ;  // one control loop
  LandingPrediction a = predictIgnitionNow(motor, h, v, mass, cosTilt, IGNITER_DELAY_S);
  if (now) *now = a;
  if (v > -LANDING_MIN_DESCENT_MPS) return false;
  if (a.stops && a.stopAlt > LANDING_MAX_STOP_ALT_M) return false;  // far too early
  LandingPrediction b = predictIgnitionNow(motor, h + v * d - 0.5f * G0 * d * d, v - G0 * d, mass,
                                           cosTilt, IGNITER_DELAY_S);
  return a.touchdownSpeed <= b.touchdownSpeed;
}

// Drop from 'apogee' metres using the same rule the flight code uses.
struct LandingOutcome { float igniteAlt, igniteVel; LandingPrediction result; };

inline LandingOutcome simulateDrop(const ThrustCurve& motor, float apogee, float mass) {
  LandingOutcome o;
  const float dt = 0.002f;
  float h = apogee, v = 0;
  while (h > 0) {
    v += (-G0 + dragAccel(v, mass)) * dt;
    h += v * dt;
    if (landingFireDecision(motor, h, v, mass, 1.0f, 0)) break;
  }
  o.igniteAlt = h;
  o.igniteVel = v;
  o.result = predictIgnitionNow(motor, h, v, mass, 1.0f, IGNITER_DELAY_S);
  return o;
}

// Prints a design table: which apogee gives the softest touchdown for this
// mass, motor and igniter delay. Aim your rocket's apogee at the best row.
template <class P>
void printLandingPlan(P& out, const ThrustCurve& motor) {
  float mass = LIFTOFF_MASS_KG - ASCENT_PROP_MASS_KG;
  out.printf("\nLANDING PLAN  mass at landing %.3f kg, igniter delay %.2f s, motor %.1f Ns / %.2f s\n",
             mass, IGNITER_DELAY_S, motor.totalImpulse, motor.burnTime);
  float avgT = motor.totalImpulse / motor.burnTime;
  if (avgT < mass * G0 * 1.2f)
    out.printf("WARNING: average thrust %.1f N is under 1.2x weight %.1f N - cannot land reliably\n",
               avgT, mass * G0);
  out.printf(" apogee_m  fire_at_m  fall_mps  touchdown_mps  hop_m\n");
  float bestApogee = 0, bestSpeed = 1e9f;
  for (float ap = 6.0f; ap <= 72.0f; ap += 1.0f) {
    LandingOutcome o = simulateDrop(motor, ap, mass);
    if (((int)ap) % 6 == 0)
      out.printf("  %6.1f    %6.2f    %6.2f      %6.2f       %5.2f\n", ap, o.igniteAlt, o.igniteVel,
                 o.result.touchdownSpeed, o.result.hop);
    if (o.result.touchdownSpeed < bestSpeed) { bestSpeed = o.result.touchdownSpeed; bestApogee = ap; }
  }
  out.printf("Softest landing: apogee ~%.0f m (%.0f ft) -> %.2f m/s touchdown.\n"
             "Tune mass / ascent motor so the rocket peaks there.\n\n",
             bestApogee, bestApogee * 3.281f, bestSpeed);
}
