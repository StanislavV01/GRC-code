"""End-to-end simulation: outbound drive -> link loss -> autonomous backtrack.

The loop maintains a hidden *ground truth* pose (what the vehicle really does,
including skid-steer slip) and feeds only synthetic sensor measurements to the
dead-reckoning estimator. During the outbound leg a recorded route drives the
vehicle; when all comms drop, the BacktrackController takes over and steers the
true vehicle home using nothing but the estimated pose and the breadcrumb trail.
"""

from dataclasses import dataclass, field

from .controller import BacktrackController, PursuitConfig
from .estimator import DeadReckoning
from .geometry import distance
from .kinematics import DiffDriveModel
from .route import DEFAULT_ROUTE, outbound_commands, total_length
from .sensors import SensorModel, SensorParams

MODE_RECORDING = "RECORDING"
MODE_BACKTRACK = "FAILSAFE_BACKTRACK"
MODE_ARRIVED = "ARRIVED"


@dataclass
class SimConfig:
    dt: float = 0.02
    cruise_speed: float = 2.0
    slip_long: float = 0.02          # fractional under-travel from wheel slip
    slip_turn: float = 0.10          # fractional under-rotation in skid turns
    slip_noise: float = 0.01         # per-tick random slip jitter
    heading_source: str = "fusion"
    seed: int = 7
    max_backtrack_steps: int = 40000
    sensor_params: SensorParams = field(default_factory=SensorParams)
    pursuit: PursuitConfig = field(default_factory=PursuitConfig)


@dataclass
class SimResult:
    records: list                    # per-tick dict rows (the CSV time-series)
    breadcrumbs: list                # Breadcrumb objects from the estimator
    return_path: list                # reversed trail the controller followed
    summary: dict


def run_simulation(config=None, route=DEFAULT_ROUTE):
    cfg = config or SimConfig()
    model = DiffDriveModel()
    sensors = SensorModel(model, cfg.sensor_params, seed=cfg.seed)
    estimator = DeadReckoning(model, heading_source=cfg.heading_source)

    import random
    slip_rng = random.Random(cfg.seed + 1000)

    gt = (0.0, 0.0, 0.0)             # ground-truth pose
    records = []
    t = 0.0

    def drive(v_des, omega_des, mode, link_ok):
        """Advance one tick: apply slip, synth sensors, update estimate, log."""
        nonlocal gt, t
        v_left_cmd, v_right_cmd = model.wheel_speeds_from_twist(v_des, omega_des)

        # Ground truth: wheels spin as commanded but the chassis slips.
        jitter = slip_rng.gauss(0.0, cfg.slip_noise)
        true_v = v_des * (1.0 - cfg.slip_long + jitter)
        true_omega = omega_des * (1.0 - cfg.slip_turn)
        gt = model.step_pose(gt, true_v, true_omega, cfg.dt)

        meas = sensors.sample(v_left_cmd, v_right_cmd, omega_des,
                              true_omega, gt[2])
        est = estimator.update(cfg.dt, meas["erpm_left"], meas["erpm_right"],
                               meas["gyro_z"], meas["compass_heading"],
                               current_load=meas["current_load"])

        records.append({
            "t": round(t, 3),
            "mode": mode,
            "link_ok": int(link_ok),
            "erpm_left": round(meas["erpm_left"], 2),
            "erpm_right": round(meas["erpm_right"], 2),
            "motor_current": round(meas["motor_current"], 3),
            "gyro_z": round(meas["gyro_z"], 5),
            "compass_heading": round(meas["compass_heading"], 5),
            "est_x": round(est[0], 4),
            "est_y": round(est[1], 4),
            "est_heading": round(est[2], 5),
            "gt_x": round(gt[0], 4),
            "gt_y": round(gt[1], 4),
            "gt_heading": round(gt[2], 5),
        })
        t += cfg.dt

    # --- Outbound leg: follow the recorded route, link healthy -----------
    for v_des, omega_des in outbound_commands(cfg.dt, cfg.cruise_speed, route):
        drive(v_des, omega_des, MODE_RECORDING, link_ok=True)

    gt_outbound_end = gt
    est_outbound_end = estimator.pose

    # --- Link loss -> backtrack along the reversed breadcrumb trail ------
    return_path = list(reversed(estimator.trail_xy()))
    controller = BacktrackController(return_path, cfg.pursuit)

    steps = 0
    while steps < cfg.max_backtrack_steps:
        v_des, omega_des, done = controller.compute(estimator.pose)
        if done:
            break
        drive(v_des, omega_des, MODE_BACKTRACK, link_ok=False)
        steps += 1

    if records:
        records[-1]["mode"] = MODE_ARRIVED

    outbound_dist = total_length(route)
    final_error = distance(gt[0], gt[1], 0.0, 0.0)
    summary = {
        "heading_source": cfg.heading_source,
        "outbound_distance_m": round(outbound_dist, 2),
        "ticks": len(records),
        "breadcrumbs": len(estimator.breadcrumbs),
        "gt_outbound_end": tuple(round(c, 3) for c in gt_outbound_end),
        "est_outbound_end": tuple(round(c, 3) for c in est_outbound_end),
        "est_outbound_error_m": round(
            distance(gt_outbound_end[0], gt_outbound_end[1],
                     est_outbound_end[0], est_outbound_end[1]), 3),
        "gt_final": tuple(round(c, 3) for c in gt),
        "final_return_error_m": round(final_error, 3),
        "final_error_pct": round(100.0 * final_error / outbound_dist, 2),
        "backtrack_completed": controller.done,
    }

    return SimResult(records=records,
                     breadcrumbs=estimator.breadcrumbs,
                     return_path=return_path,
                     summary=summary)
