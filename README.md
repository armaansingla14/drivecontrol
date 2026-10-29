# drivecontrol

This is a C++17 library of vehicle actuation controllers covering steering angle, throttle and braking:

- PID speed control with feedforward and anti-windup
- Pure-pursuit and Stanley lateral control
- Actuator saturation and rate limits
- Emergency-stop and link-loss state machine
- CAN command interface with E2E checks

A Python kinematic and dynamic bicycle-model simulator drives the C++ code through a `ctypes` bridge. With it I measure path-tracking error on six tracks, stopping distance under several stop triggers, anti-windup behaviour and actuator-constraint compliance, each compared against a naive baseline controller.

Everything here runs in simulation. ROS 2 node source and a CARLA client script are included but were **not built or run** in this environment, because neither ROS 2 nor CARLA is available here.

## Layout

```
include/drivecontrol/   public headers
  types.hpp             VehicleState, ActuatorCommand, VehicleParams, ActuatorLimits
  pid.hpp               PID: feedforward, filtered derivative on measurement, None/Clamp/BackCalculation anti-windup, track()
  limits.hpp            rate limiter, ActuatorShaper (saturation + slew limits, throttle/brake exclusivity)
  path.hpp              reference path: arc length, heading, curvature, speed profile, windowed projection
  lateral.hpp           PurePursuit (speed-scheduled look-ahead), Stanley (yaw-rate damping, understeer feedforward)
  longitudinal.hpp      SpeedController: PID -> desired accel -> inverse model -> throttle/brake with coast band
  safety.hpp            SafetyManager: Disabled / Active / ControlledStop / EmergencyStop / Stopped
  can_interface.hpp     CAN frames (DC_EStop 0x080, DC_Command 0x100, DC_Status 0x300, DC_ActuatorCmd 0x310), CRC-8, RX supervision
  controller.hpp        DriveController (full) and NaiveController (baseline)
  capi.h                C ABI for ctypes / other hosts
src/                    implementations
tools/dc_replay.cpp     HIL-style harness: replays candump logs into the controller with a plant in the loop
tests/                  C++ unit and closed-loop tests (66 cases, 5 executables)
python/drivecontrol_sim vehicle.py (plants), tracks.py, bridge.py (ctypes), sim.py (closed loop + metrics),
                        reference.py (pure-Python PID / pure pursuit / Stanley for parity), experiments.py
python/tests            pytest (27 cases): parity, plant sanity, CAN round trips, closed-loop checks
ros2/drivecontrol_ros   rclcpp node, launch file, parameters      (not built here)
carla/                  CARLA client driving a vehicle with the library (not run here)
results/                CSVs, summary.md, HIL input logs
```

### Controller details

- **Lateral control.** Pure pursuit works on the rear axle with a look-ahead of `clamp(3 m + 0.5 s * v, 3, 25) m`. Stanley works on the front axle: `psi_e + atan(-k e / (1 + v))`, with `k = 1.5`. To this it adds yaw-rate damping, `-0.05 (r - v kappa)`, and an understeer feedforward of `0.002 rad per m/s^2` of lateral acceleration.
- **Longitudinal control.** A PI controller (kp 0.8, ki 0.25) acts on speed, with the profile acceleration as feedforward. It outputs a desired acceleration limited to -8..4 m/s^2. An inverse model (mass, drag, rolling resistance) turns that into a force. A ±150 N coast band prevents throttle/brake chatter. Pedal saturation is fed back to the PID through back-calculation (`Pid::track`).
- **Actuator limits.** These are applied to every output:
  - steering: ±0.52 rad, 0.6 rad/s
  - throttle: rises at most 1.5 /s
  - brake: rises at most 4 /s, or 10 /s in an emergency
  - throttle and brake are never applied together
- **Safety.** The state machine reacts to each condition as follows:

  | Condition | Reaction |
  |---|---|
  | E-stop bit | emergency stop |
  | Loss of the 20 ms safety heartbeat (100 ms timeout) | emergency stop |
  | Invalid or NaN state | emergency stop |
  | Loss of the 50 ms planner command (200 ms timeout) | controlled stop at 3 m/s^2 |
  | Lateral error > 3 m | controlled stop at 3 m/s^2 |
  | Disable request | controlled stop at 3 m/s^2 |

  Lateral control stays active while braking. After a stop, the controller only re-arms through an explicit reset followed by an enable edge. Frames with a bad CRC-8, a repeated alive counter, or a counter jump larger than 2 are rejected, so they do not refresh the timeouts.
- **Baseline (`NaiveController`).** A PI speed loop drives the pedals directly, with no feedforward and no anti-windup. Steering is a proportional law on cross-track and heading error, `0.1 e + 1.0 psi_e`, with no look-ahead. The baseline gains were picked by a 4x4 grid search on the same evaluation tracks, which favours the baseline. Steering is clamped but not rate-limited. The e-stop bit produces a full brake step, and there is no heartbeat or command-timeout supervision.

## Build and run

```bash
# C++ (library, shared C library for ctypes, replay tool, tests)
cmake -S . -B build -G Ninja && cmake --build build
cd build && ctest                       # 5/5 executables, 66 test cases

# Python (numpy, pytest)
cd python
python -m pytest -q tests               # 27 passed
python -m drivecontrol_sim.experiments --out ../results
```

The C++ code was built with g++ 13.3 inside a Linux container. `libdrivecontrol_c.so` links libstdc++ statically, which lets it load from the host Python 3.12 environment where the simulator and pytest ran.

## Plant model (`python/drivecontrol_sim/vehicle.py`)

- **Vehicle:** 1500 kg, Iz 2250 kg m^2, lf 1.2 m, lr 1.5 m.
- **Tyres:** linear, 80 kN/rad per axle, saturating at mu = 1. Below 3 m/s the model switches to kinematic.
- **Forces:** drive 6 kN at full throttle, brake 12 kN (8 m/s^2), aerodynamic drag (CdA 0.7) and rolling resistance.
- **Actuators:** steering has an 80 ms lag, 0.8 rad/s rate and ±0.6 rad range; drive lag is 150 ms; brake lag is 80 ms.
- **Integration:** semi-implicit Euler at 1 kHz; the controller runs at 100 Hz.

Lateral error is measured at the centre of gravity against the reference path by an independent projection, so it is not the controller's own estimate.

## Measured results (simulated)

All numbers come from `python -m drivecontrol_sim.experiments` (written to `results/summary.md` and `results/*.csv`). Runs are deterministic.

### Path tracking on six tracks (dynamic plant)

The tracks are:

- a 1.6 km straight with speed steps (15 → 28 → 12 m/s)
- a circle of R = 40 m
- a sinusoidal S-curve
- a 3.5 m double lane change at 16 m/s
- a R = 15 m hairpin
- a figure-eight

Speed profiles are limited to 4-4.5 m/s^2 of lateral acceleration (8 m/s^2 for the lane change).

| Controller | Mean lateral RMS (m) | Worst lateral max (m) | Mean speed RMS (m/s) | Commanded-constraint violations |
|---|---:|---:|---:|---:|
| Stanley | 0.134 | 0.346 | 0.394 | 0 |
| Pure pursuit | 0.235 | 0.650 | 0.476 | 0 |
| Naive (grid-tuned) | 0.655 | 2.001 | 0.805 | 2105 |

- On the kinematic plant the mean lateral RMS is 0.069 m (Stanley), 0.054 m (pure pursuit) and 0.451 m (naive). Pure pursuit loses the most when tyre slip is added, because its geometric law has no sideslip compensation.
- The constraint violations are 10 ms steps in which the baseline commanded the steering faster than the 0.6 rad/s limit. All 2105 were steering-rate violations; there were no pedal-rate or overlap violations. The full controller produced none, because every output passes through `ActuatorShaper`.
- An earlier version of Stanley added `atan(L kappa)` as curvature feedforward. Because the heading error at the front-axle projection already contains that term, it was counted twice, and the steady-state error was 0.59 m on the R = 40 m circle (kinematic plant). Replacing it with an understeer term brought the mean lateral RMS from 0.195 to 0.134 m on the dynamic plant.

Per-track numbers are in `results/summary.md`.

### Anti-windup (speed step 0 → 30 m/s, unmodelled 10 % grade from 50-450 m)

| Controller | Anti-windup | Overshoot | Settling after first reaching 29.5 m/s | IAE after reaching |
|---|---|---:|---:|---:|
| Full | back-calculation | 0.88 m/s | 13.3 s | 5.0 m |
| Full | conditional integration (clamp) | 0.88 m/s | 14.1 s | 6.0 m |
| Full | none | 18.82 m/s | 15.2 s | 139.5 m |
| Naive | none | 14.13 m/s | 21.9 s | 114.2 m |

The remaining 0.88 m/s overshoot with anti-windup comes from the end of the grade. That is a disturbance the speed loop only corrects through its integral term, so it settles slowly.

### Emergency stop and link loss (trigger at t = 3 s)

| Case | Full controller | Naive |
|---|---|---|
| E-stop bit, 20 m/s straight | reacts in 0.02 s, stops in 27.2 m, peak jerk 58 m/s^3 | reacts in 0.02 s, stops in 26.2 m, peak jerk 96 m/s^3 |
| E-stop bit, 30 m/s straight | 58.8 m (ideal instantaneous braking: 56.2 m) | 57.1 m |
| Safety heartbeat lost, 20 m/s | detected after 0.11 s, stops in 29.0 m | does not stop |
| Corrupted (bad-CRC) frames, 20 m/s | treated as heartbeat loss, 29.0 m | does not stop |
| Planner lost, 20 m/s | controlled stop at about 3.2 m/s^2: 70.9 m, 6.85 s | does not stop |
| E-stop bit on R = 80 m curve at 17 m/s | 19.8 m, max lateral deviation 0.36 m | 19.0 m, 0.67 m |

When the e-stop bit is the trigger, the naive controller stops 0.5-1.7 m shorter. It steps the brake instead of ramping it at the 10 /s jerk limit, and pays for it with about 65% higher peak jerk. The full controller's advantage is in the link-loss cases: the baseline has no supervision and keeps driving in all of them.

### C++ / Python parity

On 200 random PID sequences and 300 random pure-pursuit/Stanley states, the independent Python re-implementation (`reference.py`) agrees with the C++ library to a maximum absolute difference of 0.0. Both use the same IEEE double operations in the same order.

### HIL-style CAN replay

`build/dc_replay` reads candump-format logs of planner and safety traffic (`results/hil/*.log`) and feeds them to the controller at the recorded timestamps, with a kinematic plant in the loop. It writes every transmitted frame to an output log. For all four logs (nominal, e-stop at 6 s, heartbeat lost at 6 s, CRC-error burst 6.0-6.2 s), two runs produced bit-identical outputs. Results for the full controller:

- **E-stop log:** the first brake frame is at 6.00 s.
- **Heartbeat and CRC-burst logs:** the first brake frame is at 6.09 s.

The baseline ignored the heartbeat and CRC cases.

## Limitations / not built here

- The ROS 2 package (`ros2/drivecontrol_ros`) and the CARLA script (`carla/run_carla_tracking.py`) were written against the rclcpp, nav_msgs, ackermann_msgs and CARLA 0.9.x Python APIs. They were **not compiled or executed**; no ROS 2 distribution or CARLA server was available.
- The plant is a single-track model with linear tyres. It has no load transfer, combined-slip tyre model, suspension, powertrain or ABS. Brake force is applied without wheel-lock modelling, so the e-stop distances are optimistic compared with a real vehicle.
- Sensor noise, localisation delay and CAN bus timing (arbitration and latency) are not modelled. CAN frames are delivered at the instant they are sent.
- Controller gains are hand-set defaults and were not optimised per track. The baseline gains were tuned on the evaluation tracks themselves.

## License

MIT, see `LICENSE`.
