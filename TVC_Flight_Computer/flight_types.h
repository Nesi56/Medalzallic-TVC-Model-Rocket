// flight_types.h - shared types and small vector / quaternion math
#pragma once
#include <math.h>
#include <stdint.h>

static const float G0 = 9.80665f;
static const float DEG2RAD = 0.01745329252f;
static const float RAD2DEG = 57.2957795131f;

enum FlightState : uint8_t {
  ST_IDLE = 0,
  ST_CALIBRATING,
  ST_ARMED,
  ST_COUNTDOWN,
  ST_IGNITION_WAIT,
  ST_ASCENT,
  ST_COAST,
  ST_DESCENT,
  ST_LANDING_IGNITION,
  ST_LANDING_BURN,
  ST_LANDED,
  ST_ABORT,
  ST_TVC_TEST,
  ST_ERROR
};

inline const char* stateName(uint8_t s) {
  switch (s) {
    case ST_IDLE:             return "IDLE";
    case ST_CALIBRATING:      return "CALIBRATING";
    case ST_ARMED:            return "ARMED";
    case ST_COUNTDOWN:        return "COUNTDOWN";
    case ST_IGNITION_WAIT:    return "IGNITION_WAIT";
    case ST_ASCENT:           return "ASCENT";
    case ST_COAST:            return "COAST";
    case ST_DESCENT:          return "DESCENT";
    case ST_LANDING_IGNITION: return "LANDING_IGNITION";
    case ST_LANDING_BURN:     return "LANDING_BURN";
    case ST_LANDED:           return "LANDED";
    case ST_ABORT:            return "ABORT";
    case ST_TVC_TEST:         return "TVC_TEST";
    case ST_ERROR:            return "ERROR";
  }
  return "?";
}

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------- 3-vector ----------
struct Vec3 { float x, y, z; };
inline Vec3 v3(float x, float y, float z) { Vec3 v = {x, y, z}; return v; }
inline Vec3 operator+(Vec3 a, Vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator-(Vec3 a, Vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator*(Vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float norm(Vec3 a) { return sqrtf(dot(a, a)); }
inline Vec3 normalized(Vec3 a) {
  float n = norm(a);
  return n > 1e-9f ? a * (1.0f / n) : v3(0, 0, 1);
}

// ---------- quaternion (rotation body -> world) ----------
struct Quat { float w, x, y, z; };
inline Quat qmul(Quat a, Quat b) {
  Quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}
inline Quat qnormalize(Quat q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-9f) { Quat i = {1, 0, 0, 0}; return i; }
  float k = 1.0f / n;
  Quat r = {q.w * k, q.x * k, q.y * k, q.z * k};
  return r;
}
// rotate a body-frame vector into the world frame
inline Vec3 qrotate(Quat q, Vec3 v) {
  Vec3 u = v3(q.x, q.y, q.z);
  Vec3 t = cross(u, v) * 2.0f;
  return v + t * q.w + cross(u, t);
}
// rotate a world-frame vector into the body frame
inline Vec3 qrotateInv(Quat q, Vec3 v) {
  Quat c = {q.w, -q.x, -q.y, -q.z};
  return qrotate(c, v);
}
// integrate body rates (rad/s) for dt seconds, exact for constant rate
inline Quat qintegrate(Quat q, Vec3 w, float dt) {
  float rate = norm(w);
  float angle = rate * dt;
  if (angle < 1e-9f) return q;
  float s = sinf(angle * 0.5f) / rate;
  Quat dq = {cosf(angle * 0.5f), w.x * s, w.y * s, w.z * s};
  return qnormalize(qmul(q, dq));
}
// shortest rotation that takes body vector 'from' onto world +Z
inline Quat qFromUpVector(Vec3 from) {
  Vec3 f = normalized(from);
  Vec3 z = v3(0, 0, 1);
  float d = dot(f, z);
  if (d < -0.9999f) { Quat q = {0, 1, 0, 0}; return q; }  // upside down
  Vec3 c = cross(f, z);
  Quat q = {1.0f + d, c.x, c.y, c.z};
  return qnormalize(q);
}
