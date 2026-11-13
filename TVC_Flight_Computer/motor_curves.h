// motor_curves.h - thrust curves {time s, thrust N}
//
// E16 below is an APPROXIMATION built to match the 2013 NAR certification
// numbers for the Estes E16 (33.4 N*s total, 26.44 N peak, 2.09 s burn,
// 40 g propellant). For best landing accuracy replace it with the real
// RASP (.eng) points from https://www.thrustcurve.org (search "Estes E16"):
// each line of a .eng file is "time thrust" -> paste as {time, thrust},
// and make sure the first point is {0, 0} and the last thrust is 0.
#pragma once

static const float E16_CURVE[][2] = {
  {0.00f, 0.00f},
  {0.04f, 4.00f},
  {0.08f, 11.00f},
  {0.12f, 18.00f},
  {0.16f, 23.50f},
  {0.20f, 26.44f},
  {0.24f, 25.50f},
  {0.28f, 22.50f},
  {0.34f, 19.00f},
  {0.40f, 17.99f},
  {0.50f, 17.35f},
  {0.60f, 17.04f},
  {0.80f, 16.83f},
  {1.00f, 16.72f},
  {1.20f, 16.61f},
  {1.40f, 16.51f},
  {1.60f, 16.40f},
  {1.80f, 16.19f},
  {1.88f, 15.45f},
  {1.94f, 11.00f},
  {1.99f, 7.00f},
  {2.04f, 3.00f},
  {2.09f, 0.00f}
};

// Which curve each motor uses
#define ASCENT_CURVE   E16_CURVE
#define LANDING_CURVE  E16_CURVE

class ThrustCurve {
public:
  const float (*pts)[2];
  int n;
  float totalImpulse;
  float burnTime;

  ThrustCurve(const float (*p)[2], int count) : pts(p), n(count), totalImpulse(0), burnTime(0) {
    for (int i = 0; i + 1 < n; i++)
      totalImpulse += (pts[i + 1][0] - pts[i][0]) * 0.5f * (pts[i + 1][1] + pts[i][1]);
    burnTime = pts[n - 1][0];
  }

  // linear interpolation; 'hint' speeds up calls with increasing t
  float thrustAt(float t, int* hint = 0) const {
    if (t <= 0.0f || t >= burnTime) return 0.0f;
    int i = hint ? *hint : 0;
    if (i < 0 || i >= n - 1 || pts[i][0] > t) i = 0;
    while (i < n - 2 && pts[i + 1][0] < t) i++;
    if (hint) *hint = i;
    float t0 = pts[i][0], t1 = pts[i + 1][0];
    float f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0f;
    return pts[i][1] + f * (pts[i + 1][1] - pts[i][1]);
  }
};

#define CURVE_POINTS(c) ((int)(sizeof(c) / sizeof(c[0])))
