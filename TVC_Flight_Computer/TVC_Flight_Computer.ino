// =====================================================================
//  TVC_Flight_Computer.ino
//  Teensy 4.1 thrust-vector-controlled, propulsively landed rocket
//
//  Hardware: Teensy 4.1, MPU6050 (gyro/accel), BMP388 (barometer),
//            2x B13DLM gimbal servos, MOSFET pyro channels for the ascent
//            and landing igniters, leg release (servo or MOSFET),
//            optional ESP32 Wi-Fi ground link on Serial2.
//
//  Flight sequence:
//    IDLE -> ARM -> CALIBRATING (2 s, keep still) -> ARMED
//    -> LAUNCH -> COUNTDOWN -> ascent igniter -> ASCENT (TVC on)
//    -> burnout -> COAST -> apogee -> DESCENT (landing predictor running)
//    -> landing igniter -> LANDING_BURN (TVC on) -> LANDED (log saved to SD)
//
//  START WITH SIM_MODE 1 IN config.h. Commands (USB Serial or ESP32):
//    HELP STATUS ARM DISARM LAUNCH ABORT TVCTEST STOP SERVOTEST
//    LEGS LEGLOCK PLAN SAVE RESET
// =====================================================================
#include "config.h"
#include "flight_types.h"
#include "motor_curves.h"
#include "imu_mpu6050.h"
#include "baro_bmp3xx.h"
#include "attitude.h"
#include "nav_filter.h"
#include "tvc.h"
#include "pyro.h"
#include "landing.h"
#include "flight_log.h"
#if SIM_MODE
#include "sim.h"
#endif
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

// ---------------------------------------------------------------------
// objects
// ---------------------------------------------------------------------
MPU6050 imu;
BMP3XX baro;
AttitudeEstimator att;
AltitudeFilter nav;
TVCController tvc;
PyroChannel pyroAscent, pyroLanding, pyroBackup, legPyro;
ThrustCurve landingMotor(LANDING_CURVE, CURVE_POINTS(LANDING_CURVE));
FlightLog flog;
#if SIM_MODE
RocketSim sim;
#endif

static uint8_t groundTxBuf[2048];

// ---------------------------------------------------------------------
// state
// ---------------------------------------------------------------------
uint8_t state = ST_IDLE;
uint32_t stateStartMs = 0;
uint32_t lastLoopUs = 0;
uint32_t loopCount = 0;

Vec3 accelG = {0, 0, 1};     // body axes, g
Vec3 gyroDps = {0, 0, 0};    // body axes, deg/s (raw, bias not removed)
float accBodyZ = 1.0f;       // specific force along the nose, g
float pressurePa = 101325.0f, groundPressurePa = 101325.0f, baroAlt = 0;
bool sensorsOk = false;
uint32_t imuFailCount = 0;
float railV = 0, contAscentV = 0, contLandingV = 0;

uint32_t launchMs = 0, burnoutMs = 0, apogeeMs = 0, landingFireMs = 0, landingThrustMs = 0;
float maxAlt = 0;
bool legsDeployed = false;
bool spikeReported = false;
bool launchFromGround = false;
uint32_t lastGroundRxMs = 0;
int countdownLast = -1;
LandingPrediction lastPred = {0, false, 0, 0};

// "condition held since" timers
uint32_t holdLaunch = 0, holdBurnout = 0, holdApogee = 0, holdThrust = 0, holdStill = 0;

// calibration accumulators
uint32_t calN = 0, calPressN = 0;
Vec3 calGyroSum, calGyroSq, calAccSum;
double calPressSum = 0;

const int AXIS_MAP[3] = {IMU_AXIS_FOR_BODY_X, IMU_AXIS_FOR_BODY_Y, IMU_AXIS_FOR_BODY_Z};
const float AXIS_SIGN[3] = {IMU_SIGN_FOR_BODY_X, IMU_SIGN_FOR_BODY_Y, IMU_SIGN_FOR_BODY_Z};

// ---------------------------------------------------------------------
// output helpers
// ---------------------------------------------------------------------
bool inFlight() { return state >= ST_IGNITION_WAIT && state <= ST_LANDING_BURN; }

void sendLine(const char* line, bool toUsb) {
  int n = (int)strlen(line) + 2;
  // never let a slow USB host stall the control loop in flight
  if (toUsb && (!inFlight() || Serial.availableForWrite() >= n)) Serial.println(line);
  if (GROUND_SERIAL.availableForWrite() >= n) GROUND_SERIAL.println(line);
}

void event(const char* fmt, ...) {
  char msg[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  char line[192];
  snprintf(line, sizeof(line), "M,%lu,%s", (unsigned long)millis(), msg);
  sendLine(line, true);
  flog.addEvent(line + 2);
}

void setState(uint8_t s) {
  if (s == state) return;
  event("STATE %s -> %s", stateName(state), stateName(s));
  state = s;
  stateStartMs = millis();
  holdLaunch = holdBurnout = holdApogee = holdThrust = holdStill = 0;
}

float stateSeconds() { return (millis() - stateStartMs) * 0.001f; }

// true once 'cond' has been continuously true for 'ms'
bool heldFor(bool cond, uint32_t ms, uint32_t& since) {
  uint32_t now = millis();
  if (!cond) { since = 0; return false; }
  if (since == 0) since = now ? now : 1;
  return (now - since) >= ms;
}

// ---------------------------------------------------------------------
// sensors
// ---------------------------------------------------------------------
bool axisMapIsRightHanded() {
  float M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for (int i = 0; i < 3; i++) {
    if (AXIS_MAP[i] < 0 || AXIS_MAP[i] > 2) return false;
    M[i][AXIS_MAP[i]] = AXIS_SIGN[i];
  }
  float det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
              M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
              M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  return fabsf(det - 1.0f) < 0.01f;
}

bool readImu() {
#if SIM_MODE
  sim.readImu(accelG, gyroDps);
  return true;
#else
  float a[3], g[3];
  if (!imu.readRaw(a, g)) return false;
  accelG = v3(AXIS_SIGN[0] * a[AXIS_MAP[0]], AXIS_SIGN[1] * a[AXIS_MAP[1]], AXIS_SIGN[2] * a[AXIS_MAP[2]]);
  gyroDps = v3(AXIS_SIGN[0] * g[AXIS_MAP[0]], AXIS_SIGN[1] * g[AXIS_MAP[1]], AXIS_SIGN[2] * g[AXIS_MAP[2]]);
  return true;
#endif
}

bool pollBaro(float& p) {
#if SIM_MODE
  return sim.readBaro(p);
#else
  if (!baro.poll()) return false;
  p = baro.pressurePa;
  return true;
#endif
}

void readVoltages() {
#if SIM_MODE
  railV = 8.0f;
  contAscentV = contLandingV = 8.0f;
#else
  railV = readDividedVolts(PIN_PYRO_RAIL_SENSE, RAIL_DIVIDER_RATIO);
  contAscentV = readDividedVolts(PIN_CONT_ASCENT, CONT_DIVIDER_RATIO);
  contLandingV = readDividedVolts(PIN_CONT_LANDING, CONT_DIVIDER_RATIO);
#endif
}

bool continuityOk(int pin, float v) { return pin < 0 || v > CONT_MIN_FRACTION * railV; }

Vec3 gyroUnbiased() { return gyroDps - att.gyroBiasDps; }

bool isStill() {
  float a = norm(accelG) * att.accelScale;
  return fabsf(a - 1.0f) < 0.15f && norm(gyroUnbiased()) < 20.0f;
}

// ---------------------------------------------------------------------
// actuators
// ---------------------------------------------------------------------
void legsBegin() {
#if LEG_RELEASE_MODE == LEG_MODE_SERVO
  if (PIN_LEG_RELEASE >= 0) {
    analogWriteFrequency(PIN_LEG_RELEASE, 50);  // must not share a PWM timer with the TVC servos
    servoWriteUs(PIN_LEG_RELEASE, LEG_SERVO_LOCKED_US, 50);
  }
#elif LEG_RELEASE_MODE == LEG_MODE_MOSFET
  legPyro.begin(PIN_LEG_RELEASE, "legs", false);
#endif
}

void deployLegs() {
  if (legsDeployed) return;
  legsDeployed = true;
#if LEG_RELEASE_MODE == LEG_MODE_SERVO
  servoWriteUs(PIN_LEG_RELEASE, LEG_SERVO_RELEASED_US, 50);
#elif LEG_RELEASE_MODE == LEG_MODE_MOSFET
  legPyro.fire(millis(), LEG_MOSFET_ON_MS);
#endif
  event("LEGS DEPLOYED at %.2f m", nav.h);
}

void lockLegs() {
  legsDeployed = false;
#if LEG_RELEASE_MODE == LEG_MODE_SERVO
  servoWriteUs(PIN_LEG_RELEASE, LEG_SERVO_LOCKED_US, 50);
#elif LEG_RELEASE_MODE == LEG_MODE_MOSFET
  legPyro.rearm();
#endif
}

void servoSweep() {
  for (int i = 0; i <= 720; i += 8) {
    float a = i * DEG2RAD;
    tvc.setGimbal(TVC_MAX_GIMBAL_DEG * cosf(a), TVC_MAX_GIMBAL_DEG * sinf(a));
    delay(12);
  }
  tvc.center();
}

void disarm(const char* why) {
  g_pyroArmed = false;
  pyroAscent.off();
  pyroLanding.off();
  pyroBackup.off();
  tvc.center();
  event("DISARMED (%s)", why);
}

void saveLog() {
  flog.recording = false;
  event("Saving flight log to SD...");
  if (flog.saveToSD()) event("Log saved: %s (%lu records)", flog.lastFile, (unsigned long)flog.count);
  else event("SD SAVE FAILED - is a card in the Teensy slot?");
}

// ---------------------------------------------------------------------
// flight events
// ---------------------------------------------------------------------
void abortFlight(const char* why) {
  event("ABORT: %s", why);
  tvc.center();
  pyroLanding.off();
  if (pyroBackup.fitted() && state != ST_IDLE && pyroBackup.fire(millis())) event("BACKUP PARACHUTE FIRED");
  setState(ST_ABORT);
}

void onLiftoff() {
  launchMs = millis() - LAUNCH_DETECT_MS;
  maxAlt = 0;
  spikeReported = false;
  tvc.resetIntegral();
  setState(ST_ASCENT);
  event("LIFTOFF");
}

void tryArm() {
  if (state != ST_IDLE) { event("ARM refused: must be IDLE (now %s)", stateName(state)); return; }
  if (!sensorsOk) { event("ARM refused: sensor fault"); return; }
  readVoltages();
  if (railV < ARM_MIN_VOLTS) {
    event("ARM refused: pyro rail %.2f V (< %.1f). Is the ARM switch on?", railV, ARM_MIN_VOLTS);
    return;
  }
  if (REQUIRE_CONTINUITY) {
    if (ONBOARD_ASCENT_IGNITION && !continuityOk(PIN_CONT_ASCENT, contAscentV)) {
      event("ARM refused: ascent igniter open circuit (%.2f V)", contAscentV);
      return;
    }
    if (!continuityOk(PIN_CONT_LANDING, contLandingV)) {
      event("ARM refused: landing igniter open circuit (%.2f V)", contLandingV);
      return;
    }
  }
  if (att.tiltDeg() > PAD_MAX_TILT_DEG) {
    event("ARM refused: rocket tilted %.1f deg (max %.0f)", att.tiltDeg(), PAD_MAX_TILT_DEG);
    return;
  }
  calN = calPressN = 0;
  calGyroSum = calGyroSq = calAccSum = v3(0, 0, 0);
  calPressSum = 0;
  event("Calibrating - DO NOT TOUCH THE ROCKET");
  setState(ST_CALIBRATING);
}

void finishCalibration() {
  float n = (float)calN;
  Vec3 mg = calGyroSum * (1.0f / n);
  Vec3 ma = calAccSum * (1.0f / n);
  Vec3 var = v3(calGyroSq.x / n - mg.x * mg.x, calGyroSq.y / n - mg.y * mg.y, calGyroSq.z / n - mg.z * mg.z);
  float gyroStd = sqrtf(fmaxf(0.0f, fmaxf(var.x, fmaxf(var.y, var.z))));
  float aMag = norm(ma);
  if (gyroStd > 1.0f || aMag < 0.9f || aMag > 1.1f || calPressN < 10) {
    event("CAL FAILED (gyro noise %.2f dps, |a| %.3f g, baro samples %lu) - rocket moved? Try ARM again",
          gyroStd, aMag, (unsigned long)calPressN);
    setState(ST_IDLE);
    return;
  }
  att.gyroBiasDps = mg;
  att.accelScale = 1.0f / aMag;
  att.initFromAccel(ma);
  groundPressurePa = (float)(calPressSum / calPressN);
  nav.reset(0);
  pyroAscent.rearm();
  pyroLanding.rearm();
  pyroBackup.rearm();
  lockLegs();
  flog.clear();
  flog.recording = true;
  g_pyroArmed = true;
  event("ARMED. gyro bias %.2f %.2f %.2f dps, tilt %.1f deg, P0 %.1f Pa, rail %.2f V",
        mg.x, mg.y, mg.z, att.tiltDeg(), groundPressurePa, railV);
  if (railV < LOW_BATT_WARN_VOLTS) event("WARNING: battery low (%.2f V)", railV);
#if ONBOARD_ASCENT_IGNITION
  event("Send LAUNCH to start the %d s countdown. DISARM to stand down.", COUNTDOWN_S);
#else
  event("Waiting for lift-off from your launch controller.");
#endif
  setState(ST_ARMED);
}

// ---------------------------------------------------------------------
// state machine
// ---------------------------------------------------------------------
void runStateMachine(float dt, uint32_t now) {
  float tiltX = att.tiltXDeg(), tiltY = att.tiltYDeg(), tilt = att.tiltDeg();
  Vec3 rate = gyroUnbiased();

  // in-flight safety nets
  if (inFlight()) {
    if (imuFailCount > LOOP_HZ / 20) { abortFlight("IMU lost"); return; }
    if (launchMs && (now - launchMs) > (uint32_t)(MAX_FLIGHT_S * 1000)) {
      event("Max flight time reached");
      setState(ST_LANDED);
      return;
    }
  }

  switch (state) {
    case ST_IDLE:
    case ST_ERROR:
      tvc.center();
      break;

    case ST_CALIBRATING:
      calN++;
      calGyroSum = calGyroSum + gyroDps;
      calGyroSq = calGyroSq + v3(gyroDps.x * gyroDps.x, gyroDps.y * gyroDps.y, gyroDps.z * gyroDps.z);
      calAccSum = calAccSum + accelG;
      if (stateSeconds() >= CAL_SECONDS) finishCalibration();
      break;

    case ST_ARMED:
      tvc.center();
      // slowly track gyro bias while sitting still on the pad
      if (norm(rate) < 2.0f && isStill()) att.gyroBiasDps = att.gyroBiasDps + rate * 0.0005f;
      if (heldFor(accBodyZ > LAUNCH_DETECT_G, LAUNCH_DETECT_MS, holdLaunch)) { onLiftoff(); break; }
      if (railV < ARM_MIN_VOLTS) { disarm("ARM switch off / battery"); setState(ST_IDLE); break; }
      if (tilt > PAD_MAX_TILT_DEG) { disarm("rocket tipped on pad"); setState(ST_IDLE); break; }
      if (stateSeconds() > ARMED_TIMEOUT_S) { disarm("armed timeout"); setState(ST_IDLE); break; }
      break;

    case ST_COUNTDOWN: {
      tvc.center();
      int left = COUNTDOWN_S - (int)stateSeconds();
      if (left != countdownLast) { countdownLast = left; if (left > 0) event("T-%d", left); }
      if (launchFromGround && (now - lastGroundRxMs) > GROUND_LINK_TIMEOUT_MS) {
        disarm("ground link lost during countdown");
        setState(ST_IDLE);
        break;
      }
      if (railV < ARM_MIN_VOLTS) { disarm("ARM switch off / battery"); setState(ST_IDLE); break; }
      if (tilt > PAD_MAX_TILT_DEG) { disarm("rocket tipped on pad"); setState(ST_IDLE); break; }
      if (left <= 0) {
        if (pyroAscent.fire(now)) {
          event("IGNITION");
          setState(ST_IGNITION_WAIT);
        } else {
          disarm("ascent pyro channel not available");
          setState(ST_IDLE);
        }
      }
      break;
    }

    case ST_IGNITION_WAIT:
      tvc.center();
      if (heldFor(accBodyZ > LAUNCH_DETECT_G, LAUNCH_DETECT_MS, holdLaunch)) { onLiftoff(); break; }
      if (stateSeconds() > IGNITION_WAIT_S) {
        disarm("MISFIRE - no lift-off. WAIT 60 s BEFORE APPROACHING THE ROCKET");
        setState(ST_IDLE);
      }
      break;

    case ST_ASCENT: {
      tvc.update(tiltX, tiltY, rate.x, rate.y, dt, true);
      if (tilt > ABORT_TILT_DEG) { abortFlight("tilt exceeded on ascent"); break; }
      float t = (now - launchMs) * 0.001f;
      bool burnout = heldFor(accBodyZ < BURNOUT_DETECT_G, BURNOUT_DETECT_MS, holdBurnout) && t > 0.5f;
      if (burnout || t > ASCENT_MAX_BURN_S) {
        burnoutMs = now;
        tvc.center();
        event("BURNOUT at %.2f s, alt %.1f m, vel %.1f m/s", t, nav.h, nav.v);
        setState(ST_COAST);
      }
      break;
    }

    case ST_COAST: {
      tvc.center();
      if (tilt > ABORT_TILT_DEG) { abortFlight("tilt exceeded in coast"); break; }
      float sinceBurnout = (now - burnoutMs) * 0.001f;
      bool velApogee = heldFor(nav.v < 0.0f, APOGEE_CONFIRM_MS, holdApogee) && sinceBurnout > 0.3f;
      bool altApogee = nav.h < maxAlt - 2.0f;
      if (velApogee || altApogee || sinceBurnout > APOGEE_TIMEOUT_S) {
        apogeeMs = now;
        event("APOGEE %.2f m (%s)", maxAlt, velApogee ? "velocity" : (altApogee ? "altitude" : "timeout"));
        setState(ST_DESCENT);
      }
      break;
    }

    case ST_DESCENT: {
      tvc.center();
      float hAbove = nav.h - LANDING_GROUND_ALT_M;
      if (hAbove < LEG_DEPLOY_ALT_M) deployLegs();
      if (tilt > ABORT_TILT_DEG) { abortFlight("tumbling in descent"); break; }
      float mass = LIFTOFF_MASS_KG - ASCENT_PROP_MASS_KG;
      float cosTilt = clampf(att.upInBody().z, 0.5f, 1.0f);
      if (landingFireDecision(landingMotor, hAbove, nav.v, mass, cosTilt, &lastPred)) {
        if (tilt > LANDING_MAX_TILT_DEG) {
          char why[64];
          snprintf(why, sizeof(why), "tilted %.1f deg at landing-burn time", tilt);
          abortFlight(why);
          break;
        }
        if (pyroLanding.fire(now)) {
          landingFireMs = now;
          event("LANDING IGNITION alt %.2f m vel %.2f m/s tilt %.1f, predicted touchdown %.2f m/s hop %.2f m",
                hAbove, nav.v, tilt, lastPred.touchdownSpeed, lastPred.hop);
          if (LEG_DEPLOY_AT_IGNITION) deployLegs();
          setState(ST_LANDING_IGNITION);
        } else {
          abortFlight("landing pyro channel not available");
        }
        break;
      }
      // it reached the ground without ever needing to fire (very low hop)
      if (heldFor(isStill(), 1000, holdStill)) { event("Landed without landing burn"); setState(ST_LANDED); }
      break;
    }

    case ST_LANDING_IGNITION:
      tvc.center();
      if (heldFor(accBodyZ > LANDING_THRUST_DETECT_G, 60, holdThrust)) {
        landingThrustMs = now - 60;
        event("LANDING MOTOR LIT %.3f s after fire (config delay %.3f; includes ~0.05 s thrust rise)",
              (landingThrustMs - landingFireMs) * 0.001f, IGNITER_DELAY_S);
        tvc.resetIntegral();
        setState(ST_LANDING_BURN);
        break;
      }
      if (stateSeconds() > LANDING_IGNITION_TIMEOUT_S) abortFlight("landing motor did not light");
      break;

    case ST_LANDING_BURN: {
      if (TVC_DURING_LANDING) tvc.update(tiltX, tiltY, rate.x, rate.y, dt, true);
      else tvc.center();
      float sinceThrust = (now - landingThrustMs) * 0.001f;
      if (sinceThrust > 0.5f && heldFor(isStill(), 500, holdStill)) {
        event("TOUCHDOWN detected");
        setState(ST_LANDED);
      } else if (sinceThrust > 0.5f && tilt > 60.0f) {
        event("Tipped over after touchdown");
        setState(ST_LANDED);
      } else if (sinceThrust > landingMotor.burnTime + 6.0f) {
        setState(ST_LANDED);
      }
      break;
    }

    case ST_LANDED:
      tvc.center();
      if (g_pyroArmed && stateSeconds() > 1.0f) disarm("flight over");
      if (!flog.saved && flog.recording && stateSeconds() > 1.5f) saveLog();
      break;

    case ST_ABORT:
      tvc.center();
      if (!flog.saved && flog.recording && (heldFor(isStill(), 1000, holdStill) || stateSeconds() > 20.0f)) {
        if (g_pyroArmed) disarm("abort complete");
        saveLog();
      }
      break;

    case ST_TVC_TEST:
      // live controller, no thrust: tilt the rocket by hand and watch the motor.
      // The NOZZLE should swing to the OPPOSITE side of the way the nose leans.
      tvc.update(tiltX, tiltY, rate.x, rate.y, dt, false);
      if (loopCount % (LOOP_HZ / 5) == 0)
        Serial.printf("TVCTEST tiltX %6.2f tiltY %6.2f | gimbalX %6.2f gimbalY %6.2f\n", tiltX, tiltY,
                      tvc.gimbalX, tvc.gimbalY);
      break;
  }
}

// ---------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------
void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  STATUS     sensor / battery / continuity readout"));
  Serial.println(F("  ARM        calibrate (keep still 2 s) and arm pyros"));
  Serial.println(F("  DISARM     stand down"));
  Serial.println(F("  LAUNCH     start countdown (ARMED only, ONBOARD_ASCENT_IGNITION 1)"));
  Serial.println(F("  ABORT      stop countdown, or in flight: never light the landing motor"));
  Serial.println(F("  TVCTEST    live gimbal response to hand tilting (STOP to end)"));
  Serial.println(F("  SERVOTEST  sweep the gimbal"));
  Serial.println(F("  LEGS / LEGLOCK  test the leg release"));
  Serial.println(F("  PLAN       landing design table (best apogee for your mass)"));
  Serial.println(F("  SAVE       write the RAM log to SD now"));
  Serial.println(F("  RESET      back to IDLE after a flight (sim: new flight)"));
}

// sent as events so the ESP32 ground page shows it too
void printStatus() {
  readVoltages();
  event("STATUS %s | sensors %s | SIM_MODE %d", stateName(state), sensorsOk ? "OK" : "FAULT", SIM_MODE);
  event("STATUS rail %.2f V | continuity ascent %.2f V %s, landing %.2f V %s", railV, contAscentV,
        continuityOk(PIN_CONT_ASCENT, contAscentV) ? "ok" : "OPEN", contLandingV,
        continuityOk(PIN_CONT_LANDING, contLandingV) ? "ok" : "OPEN");
  event("STATUS tilt %.1f (X %.2f Y %.2f) | alt %.2f m vel %.2f | baro alt %.2f m P %.1f Pa", att.tiltDeg(),
        att.tiltXDeg(), att.tiltYDeg(), nav.h, nav.v, baroAlt, pressurePa);
  event("STATUS accel %.3f %.3f %.3f g | gyro %.2f %.2f %.2f dps | bias %.2f %.2f %.2f", accelG.x, accelG.y,
        accelG.z, gyroDps.x, gyroDps.y, gyroDps.z, att.gyroBiasDps.x, att.gyroBiasDps.y, att.gyroBiasDps.z);
#if !SIM_MODE
  event("STATUS MPU WHO_AM_I 0x%02X | BMP chip 0x%02X (0x50 = BMP388) %.1f C", imu.whoAmI, baro.chipId,
        baro.temperatureC);
#endif
}

void handleCommand(char* cmd, bool fromGround) {
  for (char* c = cmd; *c; c++) *c = toupper(*c);
  while (*cmd == ' ') cmd++;
  if (fromGround) lastGroundRxMs = millis();
  if (!strcmp(cmd, "HB")) return;  // ESP32 heartbeat
  if (strcmp(cmd, "STATUS")) event("CMD %s%s", cmd, fromGround ? " (ground)" : "");

  if (!strcmp(cmd, "HELP")) printHelp();
  else if (!strcmp(cmd, "STATUS")) printStatus();
  else if (!strcmp(cmd, "ARM")) tryArm();
  else if (!strcmp(cmd, "DISARM")) {
    if (inFlight()) { event("DISARM ignored in flight (use ABORT)"); return; }
    disarm("command");
    setState(ST_IDLE);
  } else if (!strcmp(cmd, "LAUNCH")) {
    if (state != ST_ARMED) { event("LAUNCH refused: not ARMED"); return; }
#if ONBOARD_ASCENT_IGNITION
    launchFromGround = fromGround;
    countdownLast = -1;
    setState(ST_COUNTDOWN);
#elif SIM_MODE
    event("SIM: pressing the launch controller button");
    sim.externalIgnite();
#else
    event("ONBOARD_ASCENT_IGNITION is 0 - use your launch controller");
#endif
  } else if (!strcmp(cmd, "ABORT")) {
    if (state == ST_COUNTDOWN || state == ST_ARMED || state == ST_CALIBRATING) {
      disarm("countdown aborted");
      setState(ST_IDLE);
    } else if (state >= ST_IGNITION_WAIT && state <= ST_DESCENT) {
      abortFlight("ground command");
    } else {
      event("ABORT: nothing to abort in %s", stateName(state));
    }
  } else if (!strcmp(cmd, "TVCTEST")) {
    if (state != ST_IDLE) { event("TVCTEST only from IDLE"); return; }
    att.initFromAccel(accelG);
    event("TVCTEST: tilt the rocket - the nozzle must swing AWAY from the side the nose leans");
    setState(ST_TVC_TEST);
  } else if (!strcmp(cmd, "STOP")) {
    if (state == ST_TVC_TEST) { tvc.center(); setState(ST_IDLE); }
  } else if (!strcmp(cmd, "SERVOTEST")) {
    if (state == ST_IDLE) servoSweep();
  } else if (!strcmp(cmd, "LEGS")) {
    if (state == ST_IDLE) { legsDeployed = false; deployLegs(); }
  } else if (!strcmp(cmd, "LEGLOCK")) {
    if (state == ST_IDLE) { lockLegs(); event("legs locked"); }
  } else if (!strcmp(cmd, "PLAN")) {
    printLandingPlan(Serial, landingMotor);
  } else if (!strcmp(cmd, "SAVE")) {
    if (!inFlight()) saveLog();
  } else if (!strcmp(cmd, "RESET")) {
    if (state == ST_LANDED || state == ST_ABORT || state == ST_ERROR) {
      disarm("reset");
      pyroAscent.rearm();  // clear "already fired" (pyros stay software-disarmed)
      pyroLanding.rearm();
      pyroBackup.rearm();
#if SIM_MODE
      sim = RocketSim();
      sim.begin();
#endif
      lockLegs();
      launchMs = 0;
      setState(sensorsOk ? ST_IDLE : ST_ERROR);
    }
  } else {
    event("Unknown command '%s' - type HELP", cmd);
  }
}

char usbLine[64], gndLine[64];
uint8_t usbLen = 0, gndLen = 0;

void pollStream(Stream& s, char* buf, uint8_t& len, bool fromGround) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
        handleCommand(buf, fromGround);
        len = 0;
      }
    } else if (len < 63) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------------
// telemetry, logging, indicators
// ---------------------------------------------------------------------
uint8_t statusFlags() {
  return (g_pyroArmed ? 1 : 0) | (legsDeployed ? 2 : 0) | (pyroAscent.hasFired ? 4 : 0) |
         (pyroLanding.hasFired ? 8 : 0) | (continuityOk(PIN_CONT_ASCENT, contAscentV) ? 16 : 0) |
         (continuityOk(PIN_CONT_LANDING, contLandingV) ? 32 : 0) | (sensorsOk ? 64 : 0);
}

void sendTelemetry(uint32_t now) {
  // T,ms,state,alt,vel,tilt,tiltX,tiltY,gimbalX,gimbalY,railV,predTouchdown,flags
  char line[160];
  snprintf(line, sizeof(line), "T,%lu,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u", (unsigned long)now,
           stateName(state), nav.h, nav.v, att.tiltDeg(), att.tiltXDeg(), att.tiltYDeg(), tvc.gimbalX,
           tvc.gimbalY, railV, state == ST_DESCENT ? lastPred.touchdownSpeed : 0.0f, statusFlags());
  // USB gets telemetry only when something is happening, so you can type in IDLE
  sendLine(line, state != ST_IDLE && state != ST_ERROR && state != ST_TVC_TEST);
}

void logRecord(uint32_t now) {
  Vec3 r = gyroUnbiased();
  LogRecord rec;
  rec.tMs = now;
  rec.state = state;
  rec.flags = statusFlags();
  rec.baroRejects = (uint16_t)nav.totalRejects;
  rec.alt = nav.h;
  rec.vel = nav.v;
  rec.baroAlt = baroAlt;
  rec.accUp = att.verticalAccel(accelG);
  rec.accBodyZ = accBodyZ;
  rec.tiltX = att.tiltXDeg();
  rec.tiltY = att.tiltYDeg();
  rec.rateX = r.x;
  rec.rateY = r.y;
  rec.rateZ = r.z;
  rec.gimbalX = tvc.gimbalX;
  rec.gimbalY = tvc.gimbalY;
  rec.railV = railV;
  rec.predTouchdown = state == ST_DESCENT ? lastPred.touchdownSpeed : 0.0f;
  flog.add(rec);
}

void updateIndicators(uint32_t now) {
  bool led = false, buzz = false;
  uint32_t ph;
  switch (state) {
    case ST_IDLE:        led = (now % 2000) < 100; break;
    case ST_CALIBRATING: led = (now % 200) < 100; break;
    case ST_ARMED:
      ph = now % 2000;
      buzz = ph < 80 || (ph > 200 && ph < 280);
      led = (now % 500) < 250;
      break;
    case ST_COUNTDOWN:
      ph = (now - stateStartMs) % 1000;
      buzz = ph < (stateSeconds() > COUNTDOWN_S - 3 ? 400u : 100u);
      led = buzz;
      break;
    case ST_LANDED:
    case ST_ABORT:
      buzz = (now % 3000) < 500;  // locator beeps
      led = (now % 1000) < 500;
      break;
    case ST_ERROR:
      buzz = (now % 400) < 200;
      led = buzz;
      break;
    case ST_TVC_TEST: led = (now % 300) < 150; break;
    default:             led = true; break;  // flying
  }
  if (PIN_LED >= 0) digitalWrite(PIN_LED, led);
  if (PIN_BUZZER >= 0) digitalWrite(PIN_BUZZER, buzz);
}

// ---------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------
void setup() {
  // pyro gates LOW before anything else
  pyroAscent.begin(PIN_PYRO_ASCENT, "ascent");
  pyroLanding.begin(PIN_PYRO_LANDING, "landing");
  pyroBackup.begin(PIN_PYRO_BACKUP, "backup");
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  if (PIN_BUZZER >= 0) pinMode(PIN_BUZZER, OUTPUT);

  Serial.begin(USB_BAUD);
  GROUND_SERIAL.begin(GROUND_BAUD);
  GROUND_SERIAL.addMemoryForWrite(groundTxBuf, sizeof(groundTxBuf));
  analogReadResolution(12);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}  // give the Serial Monitor a moment

  flog.clear();
  tvc.begin();
  legsBegin();
  event("TVC flight computer boot%s", SIM_MODE ? " - SIMULATION MODE, pyros disabled" : "");

  bool axesOk = axisMapIsRightHanded();
  if (!axesOk) event("CONFIG ERROR: IMU axis mapping is not right-handed - fix IMU_AXIS/IMU_SIGN");

#if SIM_MODE
  sim.begin();
  sensorsOk = axesOk;
#else
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  bool imuOk = imu.begin(Wire, MPU6050_ADDR);
  bool baroOk = baro.begin(Wire, BMP3XX_ADDR);
  event("MPU6050 %s (WHO_AM_I 0x%02X)", imuOk ? "OK" : "NOT FOUND", imu.whoAmI);
  event("BMP388 %s (chip id 0x%02X)", baroOk ? "OK" : "NOT FOUND", baro.chipId);
  sensorsOk = imuOk && baroOk && axesOk;
  if (baroOk) groundPressurePa = baro.pressurePa;
#endif

  // initial attitude from gravity
  Vec3 sum = v3(0, 0, 0);
  int n = 0;
  for (int i = 0; i < 50; i++) {
#if SIM_MODE
    sim.step(0.002f, 0, 0, false, false);
#endif
    if (readImu()) { sum = sum + accelG; n++; }
    delay(2);
  }
  if (n) att.initFromAccel(sum * (1.0f / n));
  float p = 0;
  for (int i = 0; i < 30; i++) {
#if SIM_MODE
    sim.step(0.01f, 0, 0, false, false);
#endif
    if (pollBaro(p)) { groundPressurePa = p; break; }
    delay(10);
  }

  if (BOOT_SERVO_SWEEP) servoSweep();
  printLandingPlan(Serial, landingMotor);
  printHelp();
  if (!sensorsOk) setState(ST_ERROR);
  else event("READY - IDLE");
  lastLoopUs = micros();
}

void loop() {
  pollStream(Serial, usbLine, usbLen, false);
  pollStream(GROUND_SERIAL, gndLine, gndLen, true);

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < (1000000UL / LOOP_HZ)) return;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt > 0.02f) dt = 0.02f;
  uint32_t now = millis();
  loopCount++;

#if SIM_MODE
  sim.step(dt, tvc.gimbalX, tvc.gimbalY, pyroAscent.hasFired, pyroLanding.hasFired);
#endif

  // ---- sensors + estimation ----
  bool imuOk = readImu();
  imuFailCount = imuOk ? 0 : imuFailCount + 1;
  accBodyZ = accelG.z * att.accelScale;
  bool onPad = state == ST_ARMED || state == ST_COUNTDOWN || state == ST_IGNITION_WAIT;
  bool useAccel = state == ST_IDLE || state == ST_ARMED || state == ST_COUNTDOWN || state == ST_TVC_TEST ||
                  state == ST_LANDED || state == ST_ERROR;
  if (imuOk && state != ST_CALIBRATING) att.update(gyroDps, accelG, dt, useAccel);
  float accUp = att.verticalAccel(accelG);
  // With no motor burning the rocket can only feel drag (well under 1 g).
  // Anything bigger is a shock - e.g. the E16-4 ejection charge - and
  // integrating it would wreck the velocity the landing burn depends on.
  if ((state == ST_COAST || state == ST_DESCENT) && norm(accelG) * att.accelScale > 1.0f) {
    accUp = -G0;
    if (!spikeReported) { spikeReported = true; event("Accel shock ignored (ejection charge?)"); }
  }
  nav.predict(accUp, dt);

  float p;
  if (pollBaro(p)) {
    pressurePa = p;
    if (state == ST_CALIBRATING) { calPressSum += p; calPressN++; }
    if (state == ST_IDLE || ((state == ST_ARMED || state == ST_COUNTDOWN) && isStill()))
      groundPressurePa += 0.01f * (p - groundPressurePa);  // follow slow weather drift
    baroAlt = pressureToAltitude(p, groundPressurePa);
    nav.updateBaro(baroAlt);
  }
  // sitting on the pad: altitude and velocity are zero by definition
  if ((onPad && accBodyZ < 1.2f) || state == ST_IDLE || state == ST_CALIBRATING) nav.reset(0);
  if (inFlight() && nav.h > maxAlt) maxAlt = nav.h;

  if (loopCount % (LOOP_HZ / 10) == 0) readVoltages();

  runStateMachine(dt, now);

  pyroAscent.update(now);
  pyroLanding.update(now);
  pyroBackup.update(now);
  legPyro.update(now);

  if (loopCount % (LOOP_HZ / LOG_HZ) == 0) logRecord(now);
  if (loopCount % (LOOP_HZ / TELEMETRY_HZ) == 0) sendTelemetry(now);
  updateIndicators(now);
}
