# Wheel-Legged Balancing Robot — WLB-1

Al-Balqa' Applied University · Mechatronics Engineering graduation project (solo, ≈ 12 months).

A two-wheeled balancing robot built as an inverted pendulum — unstable at every instant, upright only
because a controller reacts faster than a human can — used to answer one question on real hardware:

> **Does model-based control actually pay off — and at what price?**

**PID vs LQR vs MPC.** Three controllers, one robot, one plant model, one state estimator, one test
protocol, equal tuning effort. Only the mathematics changes, so any difference in performance belongs
to the algorithm, not to the hardware or the tuning.

## Status

**V0 — two-wheel PID baseline.** No legs, no LiDAR, no ROS2, no BLDC/FOC yet.
Gate for V0: 30 s unassisted balancing, logged, ≥ 10 runs.

## Platform (V0)

| Item | Choice |
|---|---|
| MCU | ESP32 (200–500 Hz balance loop) |
| Drive | 2 × N20 gearmotor + quadrature encoder, DRV8833 voltage mode |
| IMU | MPU-class (pitch + pitch rate are the critical signals) |
| Power | 2S Li-ion (7.4 V nominal), buck to 5 V / 3.3 V |
| Structure | 3D-printed frame, vibration-isolated IMU, e-stop + fuse + test tether |

Two layers, strictly separated: a fast low-level MCU layer (estimation, control, motor drive, safety)
and, later, a slow high-level layer (SBC running ROS2 for perception → velocity and leg-height
commands). The balance loop is never blocked by perception, and the MCU keeps final authority.

## Roadmap

1. **V0 · PID baseline** — MATLAB model first, then port the gains to the MCU; 30 s unassisted.
2. **LQR** — same frame, same model, gains from the same Q and R.
3. **MPC** — finite horizon with explicit torque/tilt constraints; solver must fit the cycle.
4. **Legs** (height-only first), then **perception** (SBC + ROS2 + LiDAR + camera).
5. **Thesis** — written in parallel from V0 onward.

The control comparison is the core of the project. If scope or budget gets tight, perception is cut
first — never the comparison.

## The page

**Live:** https://joengineer656.github.io/wheel-legged-balancing-robot/

- `index.html` — the project summary (one page, what and why)
- `brief.html` — the one-page project brief
- `docs/project-documentation.md` — full technical specification
- `docs/AGENTS.md` — engineering rules for the build (safety cutoffs, architecture, testing protocol)

## Repo layout

```
index.html                        summary page (GitHub Pages root)
brief.html                        one-page brief
assets/                           university logo
docs/project-documentation.md     full technical documentation
docs/AGENTS.md                    build rules: safety, architecture, testing
```

## Safety (non-negotiable)

- `|pitch| > ~30°` → immediate motor disable.
- Heartbeat lost → immediate motor disable.
- Saturated or unknown motor command → logged warning.
- Every new controller gets its first run on a test stand / tether.

## Testing protocol

No unlogged test is a valid test. Minimum 10 runs per condition, reported as mean ± std; tune on one
set of conditions, evaluate on a different set; identical Q, R, estimation and control rate across
controllers — only the method differs.
