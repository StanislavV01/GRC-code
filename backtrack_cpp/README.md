# Link-Loss Backtrack — C++ core (compass-free, IMU-only)

C++17 core of the *Link-Loss Backtrack* system (TZ `tz_backtrack_system_2.pdf`)
for a **tracked (skid-steer) UGV** that has **no compass and no GNSS**: GNSS is
jammed, and the magnetometer is useless next to the 84 V FSESC power stage. When
the vehicle loses **all** control channels it returns home along its own
recorded route using only:

- **two track motors** (left/right, forward/back) → ERPM/current over **CAN**
- **gyroscope** → yaw rate
- **accelerometer** → stationarity detection (ZUPT) + gyro-bias calibration

**Dependency-free**: standard library only. No ROS2, no Eigen, no GoogleTest —
matching the TZ's offline / on-board constraint.

📖 **Документация:** [`docs/STAND_TEST.md`](docs/STAND_TEST.md) — стендовая
инструкция запуска (что нужно, как собрать/прогнать/принять). ·
[`docs/REFERENCE.md`](docs/REFERENCE.md) — техсправочник (файлы, CLI, формат CSV,
параметры под реальное железо, привязка к ТЗ).

## Real drivetrain (FlipSky FSESC 75450 + QS138 70H V3)

`DiffDriveModel` defaults are the verified hardware figures:

| Parameter | Value | Source |
|---|---|---|
| pole pairs | **5** | QS138 70H V3 |
| internal reduction | **1 : 2.35** | QS138 built-in gearbox |
| speed sensor | Hall (built-in) or **AS5047P encoder** (FSESC-supported) | datasheets |
| external reduction, sprocket radius | **TBD — measure on platform** | — |

The QS138 is geared for ~100 km/h; the autonomous backtrack crawl is ~1.5 m/s.
That mismatch is modelled: the demo reports the **mean backtrack ERPM ≈ 1200**,
right at the VESC sensorless floor (~1000–2000), so the motor must run
**sensored** (Hall, ideally + encoder) at return speed.

### Hall vs encoder (`--speed-sensor hall|encoder|ideal`)

At crawl speed the speed-sensor resolution dominates the **odometry distance**:

| Sensor | odometry distance error | final return error |
|---|---|---|
| Hall only (30 counts/rev) | **~65 m** | ~5 m |
| AS5047P encoder (16384/rev) | **~12 m** (≈ track-slip floor) | ~6 m |

Key nuance: **return-home is robust either way** (quantisation noise cancels on
the retrace), but Hall corrupts the *absolute travelled distance* badly. Since
distance drives the BACKTRACK_PARTIAL stop spacing and the FR-2 rolling-buffer
truncation, **the AS5047P encoder is strongly recommended.** The residual ~12 m
with a perfect encoder is the 2 % track-slip bias, not a sensor limit — heading
is gyro-based for exactly this reason.

## The compass-free problem

With a compass gone, **heading comes only from integrating the gyro**, and a
gyro's bias makes heading drift without bound. The accelerometer **cannot** fix
this — gravity is vertical, so yaw is unobservable from it. Two inertial
techniques keep the drift usable:

1. **Boot gyro-bias calibration** — the vehicle sits still ~15 s at startup; the
   mean gyro reading *is* the bias, so we subtract it (this is what ArduPilot
   does on boot).
2. **ZUPT + BACKTRACK_PARTIAL** — the accelerometer/odometry detect when the
   vehicle is still; velocity is zeroed (no position drift at halts) and the
   bias is re-estimated. During the return the vehicle halts every ~75 m
   (FR-4 BACKTRACK_PARTIAL) to re-calibrate, bounding accumulated drift.

The breadcrumb trail is still self-consistent, so **driving it in reverse brings
the true vehicle home** even though the absolute map drifts.

## Build & run

```bash
cmake -S . -B build && cmake --build build      # preferred
ctest --test-dir build --output-on-failure
# or, on a bare toolchain:
./build.sh && ./build/unit_tests
./build/run_demo
```

Options: `run_demo [--source gyro|odometry] [--no-calib] [--no-zupt] [--reverse] [--speed-sensor hall|encoder|ideal] [--seed N] [--outdir DIR]`

### Backtrack tactics: forward vs reverse-only (FR-4)

`--reverse` selects **reverse-only** return: the hull does **not** turn around at
the failsafe point — it keeps its outbound orientation and backs up along the
trail (for narrow lanes / preserving frontal orientation). Same pure-pursuit
geometry, measured from the rear; velocity is negated. Both modes return home
within budget (seed 7: forward 6.1 m, reverse 4.0 m); the difference is hull
orientation, verified by the mean motion-vs-heading metric (forward +1.0 =
nose-first, reverse −1.0 = backing up), not visible in an x-y plot.

Outputs in `data/`: `session_timeseries.csv` (blackbox log incl. `accel_*`,
`gyro_bias_est`, `stationary`), `breadcrumbs.csv`, `route.svg`.

## Run on a Raspberry Pi (CM4)

The code is pure stdlib C++17 — build natively on the Pi, no cross-toolchain:

```bash
# on the Pi (Raspberry Pi OS / Debian, ARM64):
sudo apt update && sudo apt install -y g++        # or: build-essential cmake
cd backtrack_cpp && ./build.sh
./build/run_demo --log-file data/backtrack.log
```

One-command deploy from your laptop (syncs sources, builds on the Pi, runs):

```bash
PI=pi@raspberrypi.local ./deploy/deploy.sh
PI=pi@raspberrypi.local ./deploy/deploy.sh --service   # + install systemd unit
```

`deploy/deploy.sh` uses `rsync` + `ssh`; `deploy/backtrack.service` is the
systemd unit (`Restart=always` → the TZ watchdog NFR; journald captures logs).
Cross-compiling from a host instead: `CXX=aarch64-linux-gnu-g++ ./build.sh`.

## Logs — understanding what went wrong

Two complementary streams:

1. **Event log** (`backtrack/log.hpp`) — lifecycle + problems, to stderr and/or
   a file. Opt-in (silent in unit tests). Levels: `debug|info|warn|error`.

   ```bash
   ./build/run_demo --log-level info --log-file data/backtrack.log
   ```
   ```
   [    0.000] INFO  simulation.cpp:78  CALIBRATING: holding still 15 s ...
   [    0.000] INFO  simulation.cpp:85  calibration done: gyro bias est 0.0038 (true 0.004)
   [    0.003] WARN  simulation.cpp:99  LINK LOST -> FAILSAFE_BACKTRACK; trail 577 crumbs ...
   [    0.008] INFO  simulation.cpp:180 ARRIVED: final return error 6.07 m (2.0%)
   ```
   Failure cases self-announce, e.g. `ERROR backtrack DID NOT COMPLETE ...` or
   `WARN mean backtrack ERPM ... below sensorless floor`.

2. **Blackbox CSV** (`data/session_timeseries.csv`, FR-7) — every 50 Hz tick:
   sensors, `gyro_bias_est`, `stationary`, estimate vs ground truth. This is the
   post-mortem record for tuning the EKF / diagnosing drift.

Under systemd: `journalctl -u backtrack -f` tails the event log live.

## Result (seed 7, good gyro)

```
heading drift @ end : 1.35 deg
final return error  : 4.6 m  (1.53% of route)
```

Inside the TZ acceptance budget (≤ 15 m / ~3% over 500 m).

### The hardware lesson: gyro quality dominates

With no compass, accuracy rides almost entirely on gyro noise × calibration
time. Worst-case final error over 6 seeds, 15 s boot calibration:

| `gyro_noise` | realistic class | mean | worst |
|---|---|---|---|
| 0.010 rad/s | cheap / heavy vibration | 15 m | 30 m ✗ |
| 0.005 rad/s | budget MEMS | 7 m | 16 m |
| **0.003 rad/s** | **good MEMS (ICM-42688-class)** ← default | 4 m | 10 m ✓ |
| 0.002 rad/s | tactical-ish | 3 m | 7 m |

**Takeaway for the build:** compass-free backtrack is feasible but imposes a
real requirement — a decent gyro (~0.003 rad/s) and a proper boot calibration.
A cheap/vibrating gyro is marginal and high-variance.

Run the deliberately-degraded baselines to see why each piece matters:

```
./build/run_demo --no-calib        # raw bias -> ~35 deg drift, ~150 m
./build/run_demo --source odometry # track-diff heading (slip) -> ~66 m
```

## Layout

```
include/backtrack/   public headers (the API)
  geometry.hpp       angle/vector helpers (header-only)
  types.hpp          Pose / Point / Twist / WheelSpeeds
  random.hpp         seedable Gaussian source
  kinematics.hpp     skid-steer model: twist <-> tracks <-> VESC ERPM
  sensors.hpp        synthetic ERPM / current / gyro / 3-axis accel
  estimator.hpp      IMU-only dead reckoning + bias cal + ZUPT  (FR-2, FR-3)
  controller.hpp     pure-pursuit backtrack follower            (FR-4)
  route.hpp          outbound 300 m route definition
  simulation.hpp     ground-truth loop: calibrate -> drive -> link loss -> return
  logio.hpp          CSV + SVG writers                          (FR-7)
src/                 implementations
apps/run_demo.cpp    CLI entry point
tests/               zero-dependency harness + 27 unit/integration tests
```

## Real hardware (implemented: `hal/` + `run_vehicle`)

`estimator.*` and `controller.*` run **unchanged** on the vehicle. The edges
are implemented in `include/backtrack/hal/` + `src/hal/`:

| In the sandbox | On the real UGV (CM4, no flight controller) |
|---|---|
| `sensors.*` (synthetic IMU) | `Mpu9250I2c` — GY-9250 on `/dev/i2c-1` |
| `sensors.*` (synthetic ERPM/current) | `VescSocketCan` — 2× FSESC 75450 on `can0` (Waveshare RS485 CAN HAT / MCP2515) |
| scripted link loss | `Rs485Link` + `LinkSupervisor` — operator frames on RS485, silence > timeout = LINK LOST (FR-1) |
| `simulation.*` loop | `apps/run_vehicle.cpp` — 50 Hz loop + state machine (CALIBRATING → IDLE → MANUAL → FAILSAFE_BACKTRACK → ARRIVED) |

The CM4 is the single control bridge: operator commands arrive over RS485 and
are relayed to the VESCs over CAN, so link monitoring and breadcrumb recording
see the same data. Wire-protocol codecs (`hal/frame_codec.*`) are pure logic,
unit-tested off-target; `apps/op_console.cpp` fakes the operator console from a
laptop + USB-RS485 dongle for bench tests.

📖 **First-run guide:** [`docs/HARDWARE_BRINGUP.md`](docs/HARDWARE_BRINGUP.md) —
wiring, `deploy/setup_cm4.sh` (overlays + `can0` at 500k), the VESC Tool
checklist (CAN IDs 1/2, status broadcast, app timeout), and the on-hardware
link-loss acceptance scenario. Future BNO055 = a second `IImu` implementation.

## Notes / out of scope

- **RNG differs by design** from any other language port; tests assert
  *properties and bounds*, not exact values.
- **Tilt / slope projection** is modelled flat-ground only. On non-level terrain
  the accelerometer's roll/pitch would project track motion onto the horizontal
  plane — a future extension (needs a 3-axis attitude filter).
- Out of scope (TZ §10): obstacle avoidance, SLAM/visual odometry, real
  MAVLink/CAN transport. This is the offline algorithm sandbox.
```
