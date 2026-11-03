// nav_filter.h - altitude / vertical-velocity Kalman filter
// State: [altitude m, vertical velocity m/s, accelerometer bias m/s^2]
// Predict with the IMU's vertical acceleration (fast, smooth, drifts),
// correct with the barometer (slow, noisy, no drift).
#pragma once
#include <math.h>

class AltitudeFilter {
public:
  float h = 0, v = 0, b = 0;
  float P[3][3];
  int baroRejects = 0;     // consecutive rejected baro samples
  uint32_t totalRejects = 0;

  // tuning
  float accelNoise = 0.6f;     // (m/s^2)^2 / Hz  - IMU noise + model error
  float biasNoise = 0.002f;    // (m/s^2)^2 / s   - bias random walk
  float baroVar = 0.35f * 0.35f;  // m^2         - barometer noise
  float gateM = 4.0f;          // reject baro jumps larger than this (ejection charge, gusts)

  AltitudeFilter() { reset(0); }

  void reset(float h0) {
    h = h0; v = 0; b = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = 0;
    P[0][0] = 0.1f; P[1][1] = 0.01f; P[2][2] = 0.05f;
    baroRejects = 0;
  }

  void predict(float accUp, float dt) {
    float a = accUp - b;
    h += v * dt + 0.5f * a * dt * dt;
    v += a * dt;
    // P = F P F' + Q,  F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1]
    float F[3][3] = {{1, dt, -0.5f * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
    float FP[3][3], N[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        FP[i][j] = 0;
        for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        N[i][j] = 0;
        for (int k = 0; k < 3; k++) N[i][j] += FP[i][k] * F[j][k];
      }
    float q = accelNoise;
    N[0][0] += q * dt * dt * dt / 3.0f;
    N[0][1] += q * dt * dt / 2.0f;
    N[1][0] += q * dt * dt / 2.0f;
    N[1][1] += q * dt;
    N[2][2] += biasNoise * dt;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = N[i][j];
  }

  // returns false if the sample was rejected as an outlier
  bool updateBaro(float z) {
    float y = z - h;
    if (fabsf(y) > gateM && baroRejects < 25) {  // after 0.5 s of rejects, trust baro again
      baroRejects++;
      totalRejects++;
      return false;
    }
    baroRejects = 0;
    float S = P[0][0] + baroVar;
    float K[3] = {P[0][0] / S, P[1][0] / S, P[2][0] / S};
    h += K[0] * y;
    v += K[1] * y;
    b += K[2] * y;
    float row0[3] = {P[0][0], P[0][1], P[0][2]};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] -= K[i] * row0[j];
    return true;
  }
};
