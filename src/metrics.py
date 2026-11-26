import csv
import json
from collections import deque
from math import dist, isfinite
from pathlib import Path

import numpy as np


TIMING_FIELDS = (
    "tick_ms", "sensor_wait_ms", "preprocess_ms", "perception_ms",
    "control_ms", "telemetry_ms", "view_ms", "video_ms", "work_ms", "sleep_ms", "loop_ms",
)
LIVE_FIELDS = (
    "tick_frame", "sensor_frame", "timestamp_sim", "frame_gap", "speed_kmh",
    "is_junction", "collisions", "lane_invasions", "detected", "fresh",
    "missed_frames", "offset_px", "sim_elapsed_s", "target_speed_kmh",
    "steer", "throttle", "brake", "control_valid", "pursuit_forward_m", "pursuit_lateral_m",
    "lane_error_m", "heading_error_rad", "lane_width_m", "distance_m", "ttc_s", *TIMING_FIELDS,
)
PHASE4_FIELDS = (
    "phase4_state", "phase4_target_speed_kmh", "phase4_obstacle_count", "phase4_lead_kind", "phase4_gap_m",
    "phase4_left_clear", "phase4_right_clear", "phase4_change_started", "phase4_change_completed",
    "phase4_target_blocker", "scenario_gap_m", "closing_mps", "visibility_m", "speed_limit_kmh",
    "lane_reference", "lateral_accel_mps2",
)


class RunTelemetry:
    """Odometry from consecutive poses and TTC from a short lead-range history."""

    def __init__(self) -> None:
        self.distance_m = 0.
        self.position = None
        self.lead_key = None
        self.ranges = deque(maxlen=5)

    def update(self, timestamp: float, position: tuple[float, float, float] | None,
               lead_key: tuple | None = None, gap_m: float | None = None,
               *, frame_gap: bool = False) -> dict:
        if position is not None and self.position is not None and not frame_gap:
            self.distance_m += dist(self.position, position)
        self.position = position
        valid_lead = position is not None and lead_key is not None and gap_m is not None and isfinite(gap_m)
        if (not valid_lead or frame_gap or lead_key != self.lead_key or
                self.ranges and (timestamp <= self.ranges[-1][0] or
                                 abs(gap_m - self.ranges[-1][1]) > 30. * (timestamp - self.ranges[-1][0]))):
            self.ranges.clear()
        self.lead_key = lead_key if valid_lead else None
        ttc_s = None
        if valid_lead:
            self.ranges.append((timestamp, gap_m))
            if len(self.ranges) == self.ranges.maxlen:
                first_t, first_gap = self.ranges[0]
                closing = (first_gap - gap_m) / (timestamp - first_t)
                if closing > .2:
                    ttc_s = gap_m / closing
        return {"distance_m": self.distance_m, "ttc_s": ttc_s}


def summarize_live_run(rows: list[dict], elapsed_s: float, fixed_delta: float) -> dict:
    valid = [row for row in rows if row["sensor_frame"] is not None]
    detected = sum(bool(row["detected"]) for row in valid)
    fresh = sum(bool(row["fresh"]) for row in valid)
    non_junction = [row for row in valid if row["is_junction"] is False]
    moving = [row for row in valid if row.get("speed_kmh", 0) >= 1.0]
    budget_ms = fixed_delta * 1000.0
    ttc = [row["ttc_s"] for row in valid if row.get("ttc_s") is not None]
    timing = {}
    for key in TIMING_FIELDS:
        values = [row[key] for row in rows]
        timing[key] = {
            "p50": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        } if values else None
    return {
        "loop_attempts": len(rows),
        "frames": len(valid),
        "sensor_timeouts": len(rows) - len(valid),
        "frame_gaps": sum(bool(row["frame_gap"]) for row in rows),
        "elapsed_s": elapsed_s,
        "avg_fps_wall": len(valid) / elapsed_s if elapsed_s > 0 else 0.,
        "target_fps": 1.0 / fixed_delta,
        "tick_budget_ms": budget_ms,
        "deadline_overruns": sum(row["work_ms"] > budget_ms for row in rows),
        "fresh_frames": fresh,
        "held_frames": detected - fresh,
        "lost_frames": len(valid) - detected,
        "detector_coverage": detected / len(valid) if valid else None,
        "lane_loss_rate": (len(valid) - detected) / len(valid) if valid else None,
        "non_junction_frames": len(non_junction),
        "non_junction_loss_rate": (
            sum(not row["detected"] for row in non_junction) / len(non_junction)
        ) if non_junction else None,
        "moving_frames": len(moving),
        "moving_loss_rate": (
            sum(not row["detected"] for row in moving) / len(moving)
        ) if moving else None,
        "collisions": max((row.get("collisions", 0) for row in valid), default=0),
        "lane_invasions": max((row.get("lane_invasions", 0) for row in valid), default=0),
        "distance_m": rows[-1].get("distance_m", 0.) if rows else 0.,
        "min_ttc_s": min(ttc, default=None),
        "ttc_samples": len(ttc),
        "timing_ms": timing,
    }


def write_live_report(directory: Path, rows: list[dict], report: dict) -> None:
    fields = (*LIVE_FIELDS, *PHASE4_FIELDS) if report.get("driver") == "avoid" else LIVE_FIELDS
    with (directory / "frames.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (directory / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


def summarize_lane_control(rows: list[dict], fixed_delta: float, settle_s: float = 5.) -> dict:
    """Phase 3 acceptance uses simulation time, with startup reported separately."""
    controlled = [r for r in rows if r.get("steer") is not None]
    measured = [r for r in controlled if r["sensor_frame"] is not None]
    steady = [r for r in measured if r["sim_elapsed_s"] > settle_s]
    errors = [abs(r["target_speed_kmh"] - r["speed_kmh"]) for r in measured]
    steady_errors = [abs(r["target_speed_kmh"] - r["speed_kmh"]) for r in steady]
    steer = np.asarray([r["steer"] for r in controlled])
    steps = np.abs(np.diff(steer))
    # Count alternating steering beyond a +/-0.02 deadband in each 2-second window.
    last_sign = 0
    reversals = []
    max_reversals = 0
    for row in controlled:
        sign = 1 if row["steer"] > .02 else -1 if row["steer"] < -.02 else 0
        if sign and last_sign and sign != last_sign:
            reversals.append(row["sim_elapsed_s"])
        if sign:
            last_sign = sign
        reversals = [t for t in reversals if row["sim_elapsed_s"] - t <= 2.]
        max_reversals = max(max_reversals, len(reversals))
    simulation_s = len(rows) * fixed_delta
    invasion = max((r.get("lane_invasions", 0) or 0 for r in rows), default=0)
    collisions = max((r.get("collisions", 0) or 0 for r in rows), default=0)
    max_step = float(steps.max()) if len(steps) else None
    checks = {
        "full_120_sim_seconds": simulation_s >= 120. - 1e-6,
        "lane_invasion_zero": invasion == 0,
        "collision_zero": collisions == 0,
        "no_sensor_timeouts_or_gaps": bool(rows) and all(
            r["sensor_frame"] is not None and not r["frame_gap"] for r in rows),
        "speed_error_below_3_after_settle": bool(steady_errors) and max(steady_errors) < 3.,
        "steer_slew_within_limit": max_step is not None and max_step <= .5 * fixed_delta + 1e-6,
        "no_repeated_steer_sign_reversals": max_reversals < 3,
    }
    return {
        "simulation_s": simulation_s, "settle_s": settle_s,
        "speed_max_abs_error_all_kmh": max(errors) if errors else None,
        "speed_max_abs_error_steady_kmh": max(steady_errors) if steady_errors else None,
        "speed_mae_steady_kmh": float(np.mean(steady_errors)) if steady_errors else None,
        "steer_max_step": max_step,
        "steer_step_p95": float(np.percentile(steps, 95)) if len(steps) else None,
        "max_steer_reversals_per_2s": max_reversals,
        "lane_error_abs_max_m": max((abs(r["lane_error_m"]) for r in measured
                                      if r.get("lane_error_m") is not None), default=None),
        "invalid_control_frames": sum(not r["control_valid"] for r in controlled),
        "checks": checks,
        "verdict": ("PASS" if all(checks.values()) else "FAIL") if controlled else "N/A",
    }
