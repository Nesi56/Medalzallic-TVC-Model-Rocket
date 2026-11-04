# Medalzallic TVC Model Rocket

A Teensy 4.1 flight computer for a thrust-vector-controlled, propulsively landed model rocket.

Thrust-vector-controlled ascent on an Estes E16, then a computer-timed landing burn on a second motor. Hardware:

- Teensy 4.1
- MPU6050 (gyro/accelerometer)
- BMP388 barometer
- 2× AGFRC B13DLM gimbal servos
- MOSFET pyro channels
- Leg release (servo or MOSFET)
- Optional ESP32 Wi-Fi ground station

```
Medalzallic-TVC-Model-Rocket/
├── TVC_Flight_Computer/        <- open this .ino in the Arduino IDE (Teensy 4.1)
│   ├── TVC_Flight_Computer.ino    state machine, commands, telemetry
│   ├── config.h                   *** every setting you edit is here ***
│   ├── flight_types.h             state names + vector/quaternion math
│   ├── imu_mpu6050.h              MPU6050 driver (no library needed)
│   ├── baro_bmp3xx.h              BMP388/BMP390 driver, non-blocking (no library needed)
│   ├── attitude.h                 quaternion attitude from the gyro
│   ├── nav_filter.h               altitude/velocity Kalman filter (baro + accel)
│   ├── tvc.h                      PID -> gimbal servos
│   ├── pyro.h                     igniter MOSFET channels
│   ├── landing.h                  landing-burn timing predictor + PLAN table
│   ├── motor_curves.h             E16 thrust curve
│   ├── flight_log.h               RAM log -> SD card after landing
│   └── sim.h                      built-in flight simulator (SIM_MODE)
├── ESP32_Ground_Link/
│   └── ESP32_Ground_Link.ino      Wi-Fi access point + phone web page
├── docs/
│   └── CODE_GUIDE.html            visual guide to the code: open in any web browser
├── tools/desktop_sim/             runs the flight code + simulator on a PC (needs only g++)
├── .github/workflows/build.yml    GitHub check: simulated flights + Teensy and ESP32 compiles
└── LICENSE                        MIT
```

## Read this first: four things that change your build

1. **The MPU6050 cannot measure altitude or distance.** It measures rotation and acceleration. Altitude comes from your **BMP388**, fused with the accelerometer in a Kalman filter.

2. **E16-4 vs E16-0.** The "-4" means a 4-second delay after burnout, then an ejection charge blows out of the top of the motor. In this rocket that happens during or just before the landing burn. It blasts hot gas into the airframe and can kick the sensors. The code ignores the shock, and the simulator includes it, but:
   - **Use an E16-0 for the landing motor.** Estes makes it. A "-0" has no delay and no ejection charge.
   - Consider an E16-0 for the ascent motor too.
   - Either motor vents forward at the end of its burn. Give that gas a vent path away from the electronics, the wiring and the landing motor's nozzle.

3. **Solid-motor landings live or die on timing.** You can't throttle a solid motor, so *when* you light it is the only control. In simulation, every 10 ms of igniter-delay error adds about **2.5–3.5 m/s** to touchdown speed.
   - Measure your igniter delay. See "Measure your igniter delay" below.
   - Use the most consistent igniters you can get.
   - Build legs that absorb about 3 m/s (crush foam or springs).
   - Fit a backup parachute (`PIN_PYRO_BACKUP`).

4. **Mass decides whether it can land at all.** With an E16 up and an E16 down, the height the ascent reaches only matches the drop the landing burn needs when the rocket weighs **about 1.0–1.15 kg** at liftoff (default 1.10 kg). That gives an apogee of about 17–25 m (55–80 ft).
   - A 0.8 kg rocket overshoots to about 49 m and lands much harder.
   - **Weigh your rocket**, put the number in `LIFTOFF_MASS_KG`, and type `PLAN`. It prints the apogee you need.
   - 1.1 kg is heavy for an E16. Check Estes' max liftoff weight for the motor.

## Step 1: run the simulator (no hardware needed beyond the Teensy)

1. Install Arduino IDE 2. In *File → Preferences → Additional boards manager URLs* add `https://www.pjrc.com/teensy/package_teensy_index.json`, then install "Teensy" from the Boards Manager.
2. Get the files (clone this repository, or on GitHub use **Code → Download ZIP** and unzip it). Move the `TVC_Flight_Computer` folder into your sketchbook (`Documents/Arduino`). Keep the folder name; Arduino requires it to match the `.ino` name. Don't use *Sketch → Include Library → Add .ZIP Library*: that menu is for libraries, and this is a sketch.
3. *File → Open* → `TVC_Flight_Computer.ino`. All 13 files open as tabs. Select Tools → Board → **Teensy 4.1** and USB Type **Serial**.
4. Leave `#define SIM_MODE 1` in `config.h`. Upload, then open the Serial Monitor at 115200 baud with the line ending set to **Newline**. Commands only run when a line ends.
5. Type `ARM`. Wait for `ARMED` (2 s calibration), then type `LAUNCH`. You'll see:
   - the countdown
   - lift-off, burnout and apogee
   - `LANDING IGNITION ... predicted touchdown x m/s`
   - `SIM: TOUCHDOWN -1.7 m/s tilt 0.3 deg`
   - `Log saved: FLT000.CSV` (if an SD card is in the Teensy)
6. Type `RESET` to fly again. Change things in section 10 of `config.h`:
   - `SIM_IGNITER_DELAY_S`, wind (`SIM_DIST_TORQUE_NM`), drag, mass
   - set `SIM_EJECTION_DELAY_S -1` to simulate an E16-0

In sim mode the pyro pins are never switched on, but the servos really move.

## Step 2: wiring

**Teensy 4.1 pins are 3.3 V only. Nothing above 3.3 V may touch a signal pin.**

| What | Teensy pin | Notes |
|---|---|---|
| MPU6050 SDA / SCL | 18 / 19 | VCC → Teensy **3.3 V**, GND |
| BMP388 SDA / SCL | 18 / 19 | same bus, VIN → 3.3 V. Address 0x77 (0x76 if SDO → GND) |
| Servo X signal (gimbal "pitch") | 2 | |
| Servo Y signal (gimbal "yaw") | 3 | |
| Leg release | 4 | servo signal, or MOSFET gate for a burn-wire/solenoid |
| Ascent igniter MOSFET gate | 5 | |
| Landing igniter MOSFET gate | 6 | |
| ESP32 link | 7 (RX2) ← ESP32 GPIO17, 8 (TX2) → ESP32 GPIO16 | plus GND |
| Buzzer (active) | 9 | |
| Pyro-rail sense | 14 (A0) | rail → 10 kΩ → pin → 4.7 kΩ → GND |
| Ascent continuity | 15 (A1) | ascent MOSFET drain → 100 kΩ → pin → 47 kΩ → GND |
| Landing continuity | 16 (A2) | landing MOSFET drain → 100 kΩ → pin → 47 kΩ → GND |
| Backup chute MOSFET (optional) | set `PIN_PYRO_BACKUP` | strongly recommended |

### Power: 2S LiPo

```
2S LiPo + ──[MAIN switch]──┬── 5 V BEC (2-3 A) ──┬── Teensy VIN (3.6-5.5 V)
                           │                     └── ESP32 5V/VIN
                           ├── Servos + (B13DLM V2 is rated 4.8-8.4 V, can run on 2S directly)
                           │      (1000 uF capacitor across servo + and -)
                           └──[ARM switch / key]── PYRO RAIL ──┬── igniter ── MOSFET drain (ascent)
                                                               ├── igniter ── MOSFET drain (landing)
                                                               └── divider → pin 14 (arm sense)
MOSFET source → GND.  Gate: Teensy pin → 100 Ω → gate, plus 10 kΩ gate→GND pull-down.
ALL grounds common.
```

- **MOSFETs** must turn fully on at 3.3 V. Examples: AO3400A (SOT-23) or IRLB8721 (TO-220). Avoid IRLZ44N/IRF540, which barely turn on at 3.3 V.
- **The 10 kΩ pull-down** holds the gate low while the Teensy boots. Without it an igniter can fire at power-up.
- **Servos:** if yours are not the V2 (8.4 V rated) version, run them from a 6 V BEC instead.
- **Battery sag:** a LiPo dips when an igniter fires. If the Teensy resets at ignition during bench tests, use a separate small pyro battery (with a common ground).
- **The ARM switch** is the physical safety interlock. With it off, the pyro rail is dead no matter what the code does.

## Step 3: bench bring-up (SIM_MODE 0, NO motors installed)

1. **Set up and check the sensors.** Set `SIM_MODE 0` and upload. Type `STATUS`:
   - you should see `BMP chip 0x50`, the BMP388
   - stand the rocket nose-up: accel Z should be about +1.00 g
   - tilt the nose toward each servo's side: tilt X / tilt Y should change sign as you'd expect
   - if the axes are wrong, fix `IMU_AXIS_FOR_BODY_*` / `IMU_SIGN_FOR_BODY_*`. The code refuses a left-handed mapping.
2. **Calibrate the gimbal.**
   - `SERVOTEST` sweeps the gimbal.
   - Trim `SERVO_X/Y_CENTER_US` until the motor mount is perfectly straight.
   - Measure `GIMBAL_TO_SERVO_RATIO`: servo-horn degrees per motor-mount degree.
3. **`TVCTEST`: the most important test.** Tilt the rocket by hand. **The nozzle must swing to the OPPOSITE side from the way the nose leans** (that pushes the tail back under the nose).
   - If an axis moves the wrong way, flip `SERVO_X_DIR` or `SERVO_Y_DIR`.
   - Getting this wrong means the rocket flips over within half a second of lift-off.
4. **Test the legs.** `LEGS` / `LEGLOCK` test the release.
5. **Check continuity.** Wire igniters (not in motors), turn the ARM switch on, and type `STATUS`. Both should read "ok".
6. **Measure your igniter delay.** Put a motor in a test stand, outdoors and from a safe distance, using this same battery, MOSFET and igniter type. Film at 240 fps and count frames from power-on to first flame. Repeat 5+ times.
   - Put the average in `IGNITER_DELAY_S`.
   - The spread tells you how soft your landings can be.
   - After each flight, the log's `LANDING MOTOR LIT x s after fire` line shows the real value. It includes about 0.05 s of thrust rise.
7. **Test-stand the TVC.** Tether the rocket in a gimbaled test stand with the ascent motor and confirm it fights disturbances before any free flight.

## Step 4: flight

1. Put the rocket on the pad and power on with the ARM switch **off**. Clear the area.
2. Switch ARM on, then retreat at least **30 ft** (NAR minimum for this total impulse; check your club's rules).
3. Join Wi-Fi `TVC-ROCKET` (password `launchsafe`; change it in the ESP32 sketch) and open `http://192.168.4.1`.
4. `ARM` (keep it still for 2 s), then `LAUNCH` and confirm. During the 10 s countdown:
   - `ABORT` stops it
   - if the phone loses the link, the rocket aborts the countdown by itself
5. In flight, `ABORT` means "never light the landing motor" (backup chute fires if fitted).
6. Misfire: the code disarms and prints a warning. **Wait 60 s before approaching**, then switch ARM off first.
7. After landing, the log is written to the SD card as `FLTnnn.CSV` (100 Hz data) and `FLTnnn.TXT` (events).

If your club requires a standard launch controller, set `ONBOARD_ASCENT_IGNITION 0`. The computer then just detects lift-off.

## Tuning after your first flights

- **TVC gains** (`TVC_KP/KI/KD`). Plot `tilt_x_deg`, `tilt_y_deg` and `gimbal_*` from the log.
  - oscillating: lower KP and KD
  - slow or drifting: raise KP
  - a steady lean: raise KI
- **Drag** (`LANDING_DRAG_CDA_M2`). The predictor uses drag. Fit it from the descent part of the log: compare measured vertical acceleration (`acc_up_mps2`) with −9.81.
- **Igniter delay**: update it from the logged value.
- **Mass**: re-weigh and re-run `PLAN` every time you change the rocket.

## Commands (USB Serial Monitor or ESP32 page)

`HELP STATUS ARM DISARM LAUNCH ABORT TVCTEST STOP SERVOTEST LEGS LEGLOCK PLAN SAVE RESET`

## How it works (short version)

- **500 Hz loop.** Each cycle: read the IMU, integrate the attitude quaternion, run the altitude Kalman filter (barometer at 50 Hz), then the state machine, then the servos.
- **States:** IDLE → CALIBRATING → ARMED → COUNTDOWN → IGNITION_WAIT → ASCENT → COAST → DESCENT → LANDING_IGNITION → LANDING_BURN → LANDED. ABORT is reachable from any flight state.
- **TVC:**
  - Tilt is measured about the two gimbal axes, so it stays correct when the rocket rolls. A 2-axis gimbal cannot stop roll.
  - A PID drives the gimbal angle, with a total-deflection limit and integral clamp.
  - TVC is active during the ascent burn and the landing burn. The gimbal is centred otherwise, since there's no thrust to steer with.
- **Landing timing:** every loop after apogee the code simulates two cases from its current altitude and velocity: "light the igniter now" and "light it one loop later". Each simulation covers the igniter delay, the thrust curve, mass loss, drag, touchdown, and any hop back up on leftover thrust. It fires at the moment waiting longer would make the touchdown harder. It refuses to fire if tilted more than 15°.
- **Safety:**
  - pyro gates are driven low before anything else at boot
  - pyros need both the physical ARM switch and software ARM
  - auto-disarm if the rocket tips on the pad, the ARM switch goes off, or 10 minutes pass
  - abort if tilt exceeds 30° in flight
  - abort if the IMU fails or the landing motor doesn't light within 0.7 s

## Run the simulator on your PC

`tools/desktop_sim/run.sh` compiles the real flight code for your computer, using stand-in versions of the Arduino and Teensy libraries, then flies simulated flights with no Teensy attached:

```
bash tools/desktop_sim/run.sh          # one flight with every message, then 10 quick flights
bash tools/desktop_sim/run.sh --quick  # just the 10 quick flights
```

It also builds the `SIM_MODE 0` hardware version to check it compiles. Each quick flight prints a line like `RESULT seed=4 ... state=LANDED td=1 v=-3.00 tilt=0.3`. Here `v` is touchdown speed in m/s and `tilt` is degrees from vertical. The script fails if any flight doesn't land upright. It needs `g++`: built in on Linux, from Xcode command-line tools on a Mac, or WSL on Windows.

## What was and wasn't tested

- **Tested:** the flight code was compiled with desktop GCC against mock Teensy/Arduino libraries. It ran hundreds of complete simulated flights, plus these cases:
  - misfire
  - landing motor fails to light
  - tumbling
  - external launch controller
  - ESP32 link loss during countdown
  - reset and re-fly
  - a deliberately reversed TVC sign, which the sim catches as an instant flip
- **Not tested yet:** it hasn't been compiled with the real Teensyduino toolchain or run on real hardware. The ESP32 sketch was only syntax-checked. Once this repository is on GitHub, the `build` check (`.github/workflows/build.yml`) compiles both sketches with the real Teensy and ESP32 tools on every push. Its first run is the first real compile, so check the **Actions** tab after you push.
- **The E16 thrust curve** is reconstructed from the NAR certification numbers (33.4 N·s, 26.4 N peak, 2.09 s, 40 g propellant). For best landing accuracy, paste the real curve from thrustcurve.org into `motor_curves.h`.

## License

MIT. See [LICENSE](LICENSE).
