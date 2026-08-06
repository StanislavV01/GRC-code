# Link-Loss Backtrack — UGV mock data + return-home demo

A self-contained, **pure-Python (stdlib only)** simulation for the *Link-Loss
Backtrack* system (TZ `tz_backtrack_system_2.pdf`): when the UGV loses **all**
control channels (Starlink / LTE / radio), it returns home along its own
recorded route using **GNSS-denied dead reckoning** — wheel odometry from the
VESC/CAN bus fused with the gyro and compass.

It does two things at once:

1. **Generates realistic mock blackbox data** for one ~300 m offroad route,
   as it would be logged on the hardware (CSV time-series).
2. **Implements and proves the backtrack algorithm** — the code that actually
   drives the vehicle home — and ships tests that assert it lands within the
   TZ acceptance budget.

No numpy, no matplotlib, no ROS2, no SITL. Runs on a bare Python 3 install.

## Quick start

```bash
cd backtrack_demo
python3 run_demo.py                 # fusion estimator (the real algorithm)
python3 run_demo.py --heading odometry   # naive baseline, for contrast
python3 -m unittest discover -s tests -v  # run the test suite
```

Outputs land in `data/`:

| file | what |
|------|------|
| `session_timeseries.csv` | 50 Hz blackbox log: sensors + estimate + ground truth |
| `breadcrumbs.csv` | the recorded trail (x, y, heading, cumulative distance) |
| `route.svg` | true path vs. dead-reckoned estimate vs. home/final markers |

Render the plot with any browser, or
`qlmanage -t -s 900 -o . data/route.svg` on macOS.

## Result (seed 7)

```
outbound distance   : 300 m
final return error  : 3.5 m  (1.17% of route)   <- fusion
final return error  : 75 m   (25%)              <- odometry-only heading
```

The TZ acceptance criterion is ≤ 15 m (~3%) over 500 m. The gyro-fused
estimator clears it; the odometry-only baseline shows why heading must not rely
on wheel slip — exactly the risk called out in the TZ.

## How it maps to the TZ

| TZ requirement | Module |
|----------------|--------|
| FR-2 breadcrumb recording (x, y, heading, t) | `estimator.DeadReckoning` |
| FR-3 GNSS-denied state estimation (IMU + VESC odometry + mag/gyro) | `estimator.DeadReckoning` (`fusion`) |
| FR-4 backtrack execution (pure-pursuit path following) | `controller.BacktrackController` |
| FR-7 blackbox log | `logio.write_timeseries_csv` |
| Skid-steer / VESC ERPM model | `kinematics.DiffDriveModel` |
| Magnetometer disturbed by 84 V power stage; skid slip | `sensors.SensorModel` |
| Link loss → failsafe → return | `simulation.run_simulation` |

## Architecture

```
route.py        outbound 300 m route -> per-tick (v, omega) commands
kinematics.py   skid-steer model: twist <-> wheels <-> VESC ERPM, pose integration
sensors.py      synth ERPM / current / gyro / compass from true motion (noise, slip, mag disturbance)
estimator.py    dead reckoning: fuse odometry + gyro + compass -> pose + breadcrumbs
controller.py   pure-pursuit follower over the reversed breadcrumb trail
simulation.py   ground-truth loop: outbound -> link loss -> backtrack; writes records
logio.py        CSV + hand-rolled SVG plot
run_demo.py     CLI entry point
```

The simulation keeps a hidden **ground-truth** pose (with skid slip) that the
estimator never sees — the estimator navigates on sensors alone, just like the
real vehicle. Final error is measured between the true return position and true
home `(0, 0)`.

## What is intentionally out of scope

Matches TZ §10: no obstacle avoidance (backtrack reuses a known-traversable
path), no visual odometry / SLAM, no real MAVLink / ArduPilot / VESC CAN
transport. This is the **offline algorithm sandbox** — the proven logic is what
ports to the C++/companion stack in the PoC stage.

### Modelling assumptions (open TZ questions, defaulted here)

- **Differential / skid-steer** kinematics (TZ open question #1 — undecided).
- Single unified **50 Hz** log rate (real hardware logs sensors at native rates).
- Slip and noise levels are plausible defaults, tunable in `simulation.SimConfig`
  and `sensors.SensorParams`; they are not calibrated to a specific platform.
```
