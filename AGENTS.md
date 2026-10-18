# AGENTS.md — Wheel-Legged Balancing Robot

## Project at a glance

- **Title:** Design and Control of a Wheel-Legged Balancing Robot (PID / LQR / MPC comparison)
- **Type:** Mechatronics graduation project, solo, ~12 months
- **Phase:** V0 — 2-wheel PID baseline only. No legs, no LiDAR, no ROS2, no BLDC/FOC.
- **Repo state:** Not a git repo yet. No build / lint / test / typecheck commands. No source code yet.
- **Core rule:** Control comparison is the core. **Perception is cut first if scope or budget gets tight, never the control comparison.**

## Architecture (hard rule)

Two layers, strictly separated:

| Layer | Hardware | Rate | Role |
|---|---|---|---|
| Low level | MCU (ESP32 in V0, STM32 candidate) | 200–500 Hz | IMU + state estimation + PID/LQR/MPC + motor drive + safety |
| High level | SBC running ROS2 (added later) | 10–50 Hz | LiDAR / camera / SLAM / nav → velocity + leg-height commands |

**Invariants the low-level firmware must enforce:**
- Perception must never block the balance loop.
- MCU has final authority — clamp or reject any unsafe command.
- Heartbeat timeout from SBC / remote → immediate motor disable.

## Hardware (confirm model before coding)

| Item | V0 choice | Notes |
|---|---|---|
| MCU | ESP32 (DevKit variant TBD) | Arduino-core OK for V0. STM32 reserved for MPC if needed. |
| Wheel motors | N20 gearmotor + encoder | Ratio / RPM / KV **TBD — ask before tuning**. |
| Motor driver | DRV8833 | Dual H-bridge, VM max 10.8 V. Voltage-mode in V0. |
| IMU | MPU6050 / MPU9250 / ICM-20948 / BNO085 (TBD) | Pitch + pitch rate are the critical signals. **Ask before writing filter.** |
| Battery | 2 × 3.7 V Li-ion in series | 7.4 V nominal, 8.4 V full. Buck to 5 V / 3.3 V for MCU. |
| Frame | 3D-printed | < 500 g upper mass, vibration-isolated IMU, tether/handle mounts. |

**Do not assume a specific MCU / IMU / N20 model without asking.** Filter and safety code depend on exact hardware.

## Electronics rules that prevent mistakes

- Star ground — do not route motor return through MCU ground plane.
- 100–470 µF bulk cap on DRV8833 VM rail; 0.1 µF decoupling per IC; motor suppression caps.
- IMU I2C on 3.3 V, short wires, mechanically away from motors.
- ESP32 PWM ×2 → DRV8833 IN1/2 + IN3/4; GPIO interrupts ×4 for two N20 quadrature encoders.
- **Always include:** e-stop switch, fuse, and a tether / test stand for the first run of any new controller.

## Firmware structure (do NOT put everything in `loop()`)

Separate files from day one:

- `imu.*` — raw IMU read + filter (complementary first, Kalman later)
- `estimator.*` — filtered state: pitch, pitch_rate, speed, position, yaw_rate
- `controller.*` — PID in V0; same interface for LQR / MPC later
- `motors.*` — DRV8833 PWM + encoder read
- `safety.*` — hard limits, watchdog, e-stop
- `telemetry.*` — WiFi serial / UDP log of state + commands every cycle

All constants, pin numbers, and model parameters live in one `config.h`. Do not hard-code elsewhere.

## Safety cutoffs (non-negotiable — check before any code change)

- `|pitch| > ~30°` → immediate motor disable.
- Heartbeat lost → motor disable.
- Any motor command saturated or current unknown → log warning.

## Simulation rule

**MATLAB first.** Reproduce the plant (inverted pendulum on cart, matching mass / inertia / wheel radius) and tune PID there before porting gains to ESP32. Save every sim run with its parameters. Bad gains destroy hardware in seconds.

## Testing (carried into LQR / MPC)

- Minimum 10 runs per condition; report mean ± std, not a single run.
- Same Q, R, state estimation, control rate across controllers — only the method differs.
- Tune on one set of conditions, evaluate on a different set.
- No unlogged test is a valid test.

## What NOT to do

- Add ROS2 / LiDAR / camera only after V0 PID balances unassisted for 30 s.
- Swap to BLDC / FOC only after the controller-comparison decision and MCU choice.
- Use hobby servos for leg actuators (not torque-controllable).
- Trade control-comparison risk for a cool perception demo.
- Write code comments, emojis, or cute names in firmware. Keep it minimal.

## Agent workflow

- Before changing code, verify the change does not break safety invariants above.
- Before adding a feature, ask whether it belongs to V0. If not, propose — do not build.
- When unsure about hardware (IMU model, N20 ratio, ESP32 variant, battery), **ask before coding.**
- MATLAB scripts and firmware `config.h` must stay in sync per sim run.
- Commit messages describe the physical / algorithmic effect, not the file edited.

## Roadmap (for context only)

1. **V0 — 2-wheel PID baseline** (current)
2. **LQR** on same hardware / same model
3. **MPC** with explicit constraints
4. **Legs** (height-only first; gain-scheduled)
5. **Perception** (SBC + ROS2 + LiDAR + camera)
6. **Thesis** — written in parallel from V0 onward

Full technical spec: `Wheel-Legged Balancing Robot Project Documentation from claude.md`
