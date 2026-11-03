// =====================================================================
//  config.h  -  every setting you should need to change is in this file
// =====================================================================
//  Teensy 4.1 TVC / propulsive-landing flight computer
//  Units: metres, seconds, kilograms, newtons, degrees (unless noted)
// =====================================================================
#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// 1. MODE
// ---------------------------------------------------------------------
// SIM_MODE 1 : bench simulation. Sensors are replaced by a physics model
//              of the rocket and the pyro pins are NEVER switched on.
//              Open the Serial Monitor (115200), type ARM, wait for
//              "ARMED", then type LAUNCH and watch a whole flight play out.
//              The servos DO move, so you can see the TVC working.
// SIM_MODE 0 : real flight hardware.
#define SIM_MODE 1

// 1 = the flight computer lights the ascent motor itself
//     (ARM -> LAUNCH command -> countdown -> ascent pyro channel).
// 0 = you light the ascent motor with a normal launch controller and the
//     computer simply detects lift-off with the accelerometer.
#define ONBOARD_ASCENT_IGNITION 1

// ---------------------------------------------------------------------
// 2. PINS (Teensy 4.1)
// ---------------------------------------------------------------------
// I2C (Wire): SDA = pin 18, SCL = pin 19  -> MPU6050 + BMP388 barometer
// ESP32 link: Serial2, RX2 = pin 7, TX2 = pin 8 (3.3 V logic both sides)
// Use -1 for anything you have not fitted.
#define PIN_SERVO_X          2    // gimbal servo that tilts the motor about body X
#define PIN_SERVO_Y          3    // gimbal servo that tilts the motor about body Y
#define PIN_LEG_RELEASE      4    // leg-release servo signal OR MOSFET gate
#define PIN_PYRO_ASCENT      5    // MOSFET gate -> ascent motor igniter
#define PIN_PYRO_LANDING     6    // MOSFET gate -> landing motor igniter
#define PIN_PYRO_BACKUP     -1    // MOSFET gate -> backup parachute (strongly recommended)
#define PIN_BUZZER           9    // active buzzer (+ to pin, - to GND)
#define PIN_LED             13    // on-board LED
#define PIN_PYRO_RAIL_SENSE 14    // A0: pyro rail (after ARM switch) via divider
#define PIN_CONT_ASCENT     15    // A1: ascent igniter continuity via divider, -1 = none
#define PIN_CONT_LANDING    16    // A2: landing igniter continuity via divider, -1 = none

#define GROUND_SERIAL       Serial2
#define GROUND_BAUD         460800
#define USB_BAUD            115200

// ---------------------------------------------------------------------
// 3. BATTERY / ARMING / CONTINUITY
// ---------------------------------------------------------------------
// Rail sense:  rail -> 10k -> pin -> 4.7k -> GND. Ratio = (10k + 4.7k) / 4.7k
// Continuity:  MOSFET drain -> 100k -> pin -> 47k -> GND (same ratio; only
//              ~57 uA flows through the igniter, far below what heats it)
// A full 2S LiPo (8.4 V) gives 2.69 V at the pin. NEVER exceed 3.3 V there.
#define RAIL_DIVIDER_RATIO   3.128f
#define CONT_DIVIDER_RATIO   3.128f
#define ARM_MIN_VOLTS        6.4f    // rail must be at least this to ARM (arm switch ON)
#define LOW_BATT_WARN_VOLTS  7.2f
#define CONT_MIN_FRACTION    0.5f    // igniter OK if its sense voltage > 50 % of rail
#define REQUIRE_CONTINUITY   1       // 1 = refuse to ARM if a fitted channel shows open

// ---------------------------------------------------------------------
// 4. SENSORS
// ---------------------------------------------------------------------
#define I2C_CLOCK_HZ   400000
#define MPU6050_ADDR   0x68          // 0x69 if AD0 is pulled high
#define BMP3XX_ADDR    0x77          // BMP388 (BMP390 also works). Adafruit default 0x77, 0x76 if SDO -> GND

// IMU mounting. The code works in "body" axes:
//   body Z = along the rocket, pointing at the NOSE
//   body X = the pivot axis of the X gimbal servo
//   body Y = the pivot axis of the Y gimbal servo
// body[i] = SIGN * sensor[AXIS]   (sensor axes: 0 = X, 1 = Y, 2 = Z as printed
// on the MPU6050 board). The result must be right-handed (X cross Y = Z);
// the code checks this at boot and refuses to fly if it is not.
// Default = board mounted across the tube, chip Z toward the nose, chip X
// parallel to the X servo's pivot.
#define IMU_AXIS_FOR_BODY_X  0
#define IMU_SIGN_FOR_BODY_X  1
#define IMU_AXIS_FOR_BODY_Y  1
#define IMU_SIGN_FOR_BODY_Y  1
#define IMU_AXIS_FOR_BODY_Z  2
#define IMU_SIGN_FOR_BODY_Z  1

// ---------------------------------------------------------------------
// 5. ROCKET
// ---------------------------------------------------------------------
// WEIGH IT: whole rocket, both motors loaded. With an E16 up and an E16 down
// the ascent apogee only matches what the landing burn needs at ~1.0-1.15 kg
// (type PLAN to check). 1.10 kg peaks around 19-20 m (~65 ft).
#define LIFTOFF_MASS_KG       1.100f
#define ASCENT_PROP_MASS_KG   0.040f   // E16 = 40 g propellant (NAR cert)
#define LANDING_PROP_MASS_KG  0.040f   // set for your landing motor

// ---------------------------------------------------------------------
// 6. THRUST VECTOR CONTROL
// ---------------------------------------------------------------------
// Controller output is GIMBAL angle (how far the motor tilts), in degrees:
//   gimbal = KP * tilt + KI * integral(tilt) + KD * tilt_rate
// These defaults are a sane starting point for a ~1.1 kg, ~0.8 m rocket
// with the gimbal ~0.3 m below the CG. They are NOT tuned for your rocket.
// Tune them in simulation (SIM_MODE, section 10) and with a tethered/static
// test before free flight.
#define TVC_KP                0.40f    // deg gimbal per deg tilt
#define TVC_KI                0.10f    // deg gimbal per (deg * s)
#define TVC_KD                0.12f    // deg gimbal per (deg/s)
#define TVC_I_LIMIT_DEG       2.0f     // max gimbal the integral term may add
#define TVC_MAX_GIMBAL_DEG    5.0f     // mechanical limit of your mount
#define TVC_DURING_LANDING    1        // 1 = landing motor is in the gimbal too

// Servo calibration
// B13DLM V2 is rated for 333 Hz refresh. Leave 50 until you have confirmed
// your servos are the V2; higher refresh = less lag = easier TVC.
#define SERVO_PWM_HZ          50
#define SERVO_US_PER_DEG      10.0f    // servo pulse change per servo-horn degree
#define SERVO_X_CENTER_US     1500.0f  // trim so the motor is perfectly straight
#define SERVO_Y_CENTER_US     1500.0f
#define SERVO_X_DIR           1        // flip to -1 if TVCTEST moves the wrong way
#define SERVO_Y_DIR           1
// Servo-horn degrees per gimbal degree. Measure: command the servo 10 deg
// and measure how far the motor mount tilts. ratio = 10 / mount_tilt.
#define GIMBAL_TO_SERVO_RATIO 3.0f
#define SERVO_MIN_US          900.0f
#define SERVO_MAX_US          2100.0f
#define BOOT_SERVO_SWEEP      1        // wiggle the gimbal at power-up

// ---------------------------------------------------------------------
// 7. FLIGHT LOGIC
// ---------------------------------------------------------------------
#define LOOP_HZ               500
#define CAL_SECONDS           2.0f     // pad calibration time after ARM
#define ARMED_TIMEOUT_S       600.0f   // auto-disarm if not launched
#define COUNTDOWN_S           10
#define PAD_MAX_TILT_DEG      10.0f    // refuse to ARM/launch if leaning more
#define LAUNCH_DETECT_G       1.5f     // body-Z specific force for lift-off
#define LAUNCH_DETECT_MS      50
#define IGNITION_WAIT_S       3.0f     // no lift-off this long after firing = misfire
#define BURNOUT_DETECT_G      0.4f
#define BURNOUT_DETECT_MS     60
#define ASCENT_MAX_BURN_S     2.6f     // E16 burns 2.09 s
#define APOGEE_CONFIRM_MS     100      // velocity must stay negative this long
#define APOGEE_TIMEOUT_S      8.0f     // force apogee this long after burnout
#define ABORT_TILT_DEG        30.0f    // during ascent/coast -> ABORT (no landing burn)
#define PYRO_FIRE_MS          1000     // how long a pyro channel stays on
#define MAX_FLIGHT_S          45.0f    // end flight and save log after this

// ---------------------------------------------------------------------
// 8. LANDING BURN
// ---------------------------------------------------------------------
// A solid motor cannot be throttled, so WHEN you light it is everything.
// Every loop after apogee the computer simulates "what if I fire right now?"
// (igniter delay + thrust curve + touchdown + any hop) and fires at the
// moment that gives the softest touchdown. Type PLAN to see the apogee your
// rocket needs for a soft landing.
//
// IGNITER_DELAY_S is the biggest source of error. MEASURE it: same battery,
// same MOSFET, same igniter type, motor in a test stand, film at 240 fps
// and count frames from pyro ON to first flame. Do it several times.
#define IGNITER_DELAY_S          0.35f
#define LANDING_AIM_ALT_M        0.10f  // aim this far above the surface (safety margin)
#define LANDING_MAX_STOP_ALT_M   3.0f   // never fire if it would stop higher than this
#define LANDING_GROUND_ALT_M     0.0f   // landing surface vs. where it sat when ARMED
                                        // (e.g. -0.5 if launched from a 0.5 m stand)
#define LANDING_MAX_TILT_DEG     15.0f  // never light the landing motor tilted more than this
#define LANDING_MIN_DESCENT_MPS  1.0f   // only consider firing once falling this fast
#define LANDING_DRAG_CDA_M2      0.003f // drag coefficient * frontal area (m^2), fit from flight logs
#define LANDING_THRUST_DETECT_G  1.3f   // body-Z specific force that means "it lit"
#define LANDING_IGNITION_TIMEOUT_S 0.7f // no thrust this long after firing -> ABORT

// Landing legs
#define LEG_MODE_NONE    0
#define LEG_MODE_SERVO   1   // servo pulls a pin / rotates a latch
#define LEG_MODE_MOSFET  2   // nichrome burn-wire or solenoid on a MOSFET
#define LEG_RELEASE_MODE        LEG_MODE_SERVO
#define LEG_SERVO_LOCKED_US     1000.0f
#define LEG_SERVO_RELEASED_US   2000.0f
#define LEG_MOSFET_ON_MS        1500
#define LEG_DEPLOY_ALT_M        8.0f   // deploy below this altitude while descending...
#define LEG_DEPLOY_AT_IGNITION  1      // ...or when the landing motor is fired

// ---------------------------------------------------------------------
// 9. LOGGING / TELEMETRY
// ---------------------------------------------------------------------
#define TELEMETRY_HZ   10
#define LOG_HZ         100
#define LOG_SECONDS    40     // RAM ring buffer, written to the SD card after landing
#define GROUND_LINK_TIMEOUT_MS 2000  // countdown started from ESP32 aborts if link drops

// ---------------------------------------------------------------------
// 10. SIMULATION (only used when SIM_MODE is 1)
// ---------------------------------------------------------------------
#define SIM_MOI_KGM2            0.060f  // pitch/yaw inertia (~ m*L^2/12)
#define SIM_GIMBAL_ARM_M        0.30f   // CG to gimbal pivot distance
#define SIM_CDA_M2              0.0035f // "true" drag (deliberately not = LANDING_DRAG_CDA_M2)
#define SIM_THRUST_MISALIGN_DEG 0.5f    // build error the integral term must fix
#define SIM_INITIAL_TILT_DEG    2.0f
#define SIM_ROLL_RATE_DPS       15.0f   // uncontrolled roll (gimbal cannot stop roll)
#define SIM_IGNITER_DELAY_S     0.35f   // what the "real" igniter does in the sim
#define SIM_IGNITER_JITTER_S    0.04f   // +/- random variation
#define SIM_EJECTION_DELAY_S    4.0f    // E16-4 ejection charge after burnout (-1 = E16-0, none)
#define SIM_SERVO_DPS           600.0f  // B13DLM V2: ~0.10 s / 60 deg at 8.4 V
#define SIM_GYRO_BIAS_DPS       0.8f
#define SIM_DIST_TORQUE_NM      0.003f  // random gusts
#define SIM_SEED                12345
