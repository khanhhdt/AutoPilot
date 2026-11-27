"""Run one CARLA obstacle scenario or the 20-run Phase 4 acceptance suite."""

import argparse
import csv
import json
from datetime import datetime
from pathlib import Path

import carla

from src.main import main as drive
from src.scenarios import NAMES


SUITE = (
    *(("static", seed) for seed in range(10)),
    *(("cut-in", seed) for seed in range(4)),
    *(("pedestrian", seed) for seed in range(3)),
    ("avoid", 0), ("avoid", 1), ("avoid-left", 0),
)
CLEAR_ROAD_TARGET_KMH = 50.
CLEAR_ROAD_TOLERANCE_KMH = 1.5
CLEAR_ROAD_HOLD_S = 1.


def measure_clear_road(frames_path: Path, fixed_delta_s: float) -> dict:
    peak, streak, longest = 0., 0, 0
    with frames_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            measured_kmh = float(row["speed_kmh"] or 0.)
            peak = max(peak, measured_kmh)
            within_target = abs(measured_kmh - CLEAR_ROAD_TARGET_KMH) <= CLEAR_ROAD_TOLERANCE_KMH
            cruising = (row["phase4_state"] == "CRUISE" and row["phase4_lead_kind"] == ""
                        and row["lane_reference"] == "camera" and row["control_valid"] == "True"
                        and row["is_junction"] == "False"
                        and float(row["phase4_target_speed_kmh"] or 0.) == CLEAR_ROAD_TARGET_KMH)
            streak = streak + 1 if cruising and within_target else 0
            longest = max(longest, streak)
    return {"target_kmh": CLEAR_ROAD_TARGET_KMH, "peak_kmh": peak,
            "tolerance_kmh": CLEAR_ROAD_TOLERANCE_KMH, "required_s": CLEAR_ROAD_HOLD_S,
            "longest_s": longest * fixed_delta_s}


def checks(report: dict, name: str, clear_road: dict | None = None) -> dict[str, bool]:
    phase = report["phase4"]
    scenario = phase["scenario"]
    result = {
        "valid_run": report["error"] is None and not report["cleanup_errors"]
        and report["sensor_timeouts"] == 0 and report["frame_gaps"] == 0
        and report["stop_reason"] in ("scenario_complete", "sim_duration"),
        "collision_free": report["collisions"] == 0,
        "started_only_when_clear": phase["unsafe_starts"] == 0,
    }
    if name in ("clear-road", "curve-road"):
        result["completed_route_segment"] = report["stop_reason"] == "sim_duration" and report["distance_m"] >= 10.
        result["stayed_in_lane"] = report["lane_invasions"] == 0
        if name == "clear-road":
            result["maintained_50_kmh"] = (report["target_speed_kmh"] == CLEAR_ROAD_TARGET_KMH
                                           and clear_road is not None
                                           and clear_road["longest_s"] >= CLEAR_ROAD_HOLD_S)
        else:
            result["lane_coverage"] = report["detector_coverage"] >= .8
    elif scenario is None:
        result["valid_run"] = False
    elif name == "static":
        result["stopped_before_obstacle"] = scenario["stopped_before_obstacle"]
        result["stayed_in_lane"] = sum(phase["starts"].values()) == 0 and report["lane_invasions"] == 0
    elif name in ("cut-in", "pedestrian"):
        result["actor_crossed"] = scenario["crossed"]
        result["responded"] = scenario["responded"]
    elif name == "avoid-rear":
        result["lidar_detected_rear_approach"] = scenario["rear_approach_frames"] >= 5
        result["held_lane_during_approach"] = scenario["rear_unsafe_starts"] == 0
    else:
        side = "left" if name == "avoid-left" else "right"
        result["completed_expected_change"] = (sum(phase["starts"].values()) == phase["starts"][side]
                                               == phase["completed_changes"] == 1)
        result["passed_obstacle"] = scenario["passed_obstacle"]
        result["stayed_in_corridor"] = not scenario["left_corridor"]
    return result


def traffic_ids(world: carla.World) -> set[int]:
    return {a.id for a in world.get_actors() if a.type_id.startswith(
        ("vehicle.", "sensor.", "walker.", "static.prop."))}


def run_trials(planned: tuple, root: Path, host: str, port: int, save_video: bool, *,
               video_hd: int | None = None, weather: str = "clear", lane_input: str = "semantic") -> dict:
    client = carla.Client(host, port)
    client.set_timeout(120.)
    world = client.get_world()
    original_map = world.get_map().name.rsplit("/", 1)[-1]
    original_weather = world.get_weather()
    original_settings = world.get_settings()
    original_spectator = world.get_spectator().get_transform()
    if original_settings.synchronous_mode or traffic_ids(world):
        raise RuntimeError("use an idle CARLA world in async mode for reproducible scenarios")

    root.mkdir(parents=True, exist_ok=False)
    trials = []
    restore_error = None
    try:
        if original_map != "Town10HD_Opt":
            world = client.load_world("Town10HD_Opt")
        client.set_timeout(15.)
        for name, seed in planned:
            print(f"SCENARIO {name} seed={seed}", flush=True)
            trial = {"name": name, "seed": seed, "weather": weather, "passed": False}
            trials.append(trial)
            argv = ["--host", host, "--port", str(port), "--driver", "avoid", "--weather", weather,
                    "--lane-input", lane_input,
                    "--no-display", "--out", str(root / f"{name}_{seed}")]
            if name == "clear-road":
                argv.extend(["--spawn-index", "69", "--sim-duration", "8", "--target-speed", str(CLEAR_ROAD_TARGET_KMH),
                             "--speed-kp", ".65", "--speed-ki", ".1"])
            elif name == "curve-road":
                argv.extend(["--spawn-index", "9", "--sim-duration", "8"])
            else:
                argv.extend(["--scenario", name, "--scenario-seed", str(seed)])
            if save_video:
                argv.append("--save-video")
            if video_hd:
                argv.extend(["--video-hd", str(video_hd)])
            try:
                path = drive(argv) / "report.json"
                report = json.loads(path.read_text())
                if name == "clear-road":
                    trial["clear_road_speed"] = measure_clear_road(path.with_name("frames.csv"),
                                                                  report["tick_budget_ms"] / 1000.)
                trial.update(report=str(path), checks=checks(report, name, trial.get("clear_road_speed")),
                             collisions=report["collisions"],
                             phase4=report["phase4"], metrics={key: report[key] for key in (
                                 "distance_m", "lane_invasions", "min_ttc_s", "ttc_samples", "frames",
                                 "elapsed_s", "avg_fps_wall")})
                trial["passed"] = all(trial["checks"].values())
            except (RuntimeError, OSError) as exc:
                trial["error"] = str(exc)
            try:
                world = client.get_world()
                if not world.get_settings().synchronous_mode and traffic_ids(world):
                    world.wait_for_tick(10.)
                trial["clean_async"] = not world.get_settings().synchronous_mode and not traffic_ids(world)
            except RuntimeError as exc:
                trial.update(clean_async=False, cleanup_error=str(exc))
            trial["passed"] &= trial["clean_async"]
            print(f"{name} seed={seed}: {'PASS' if trial['passed'] else 'FAIL'}", flush=True)
            if not trial["clean_async"] or (trial.get("report") and report["stop_reason"] == "interrupted"):
                break
    finally:
        if original_map != "Town10HD_Opt":
            try:
                client.set_timeout(120.)
                world = client.load_world(original_map)
                world.apply_settings(original_settings)
                world.set_weather(original_weather)
                world.get_spectator().set_transform(original_spectator)
            except RuntimeError as exc:
                restore_error = str(exc)
        passed = len(trials) == len(planned) and all(trial["passed"] for trial in trials) and restore_error is None
        summary = {
            "host": host, "port": port, "weather": weather, "lane_input": lane_input,
            "trials_planned": len(planned), "trials_attempted": len(trials),
            "trials_completed": sum("report" in trial for trial in trials),
            "static_stops": sum(bool(trial.get("checks", {}).get("stopped_before_obstacle")) for trial in trials),
            "collision_free_trials": sum(trial.get("collisions") == 0 for trial in trials),
            "lane_changes_started": sum(sum(trial.get("phase4", {}).get("starts", {}).values()) for trial in trials),
            "unsafe_starts": sum(trial.get("phase4", {}).get("unsafe_starts", 0) for trial in trials),
            "restoration_error": restore_error,
            "verdict": "PASS" if passed else "FAIL",
            "trials": trials,
        }
        path = root / "summary.json"
        path.write_text(json.dumps(summary, indent=2) + "\n")
        print(f"{summary['verdict']} {path}", flush=True)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--scenario", choices=NAMES)
    mode.add_argument("--suite", action="store_true")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=Path("output/phase4"))
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--weather", choices=("clear", "rain"), default="clear")
    parser.add_argument("--lane-input", choices=("semantic", "rgb"), default="semantic")
    args = parser.parse_args()
    if args.seed < 0:
        parser.error("--seed must be nonnegative")
    root = args.out / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
    planned = SUITE if args.suite else ((args.scenario, args.seed),)
    try:
        summary = run_trials(planned, root, args.host, args.port, args.save_video,
                             weather=args.weather, lane_input=args.lane_input)
    except RuntimeError as exc:
        parser.error(str(exc))
    return 0 if summary["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
