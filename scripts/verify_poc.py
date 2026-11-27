"""Run the headless AutoPilot PoC acceptance suite on localhost:3000."""

import argparse
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


HOST = "localhost"
PORT = 3000
MAP = "Town10HD_Opt"
CARLA_VERSION = "0.9.16"
CONNECTION = ("--host", HOST, "--port", str(PORT))


def require(condition, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def find_artifact(root: Path, pattern: str) -> Path:
    matches = list(root.glob(pattern))
    require(len(matches) == 1, f"Expected one {pattern} under {root}, found {len(matches)}")
    return matches[0]


def checked_output(command: list[str]) -> str:
    return subprocess.check_output(command, stderr=subprocess.STDOUT, text=True, timeout=30)


def python_module(name: str, *arguments) -> list[str]:
    return [sys.executable, "-u", "-m", name, *map(str, arguments)]


def source_commit() -> str | None:
    if shutil.which("git"):
        result = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()
    return None


class Acceptance:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.root.mkdir(parents=True)
        self.summary = {"verdict": "RUNNING", "commit": source_commit(), "host": HOST, "port": PORT,
                        "map": MAP, "stages": []}

    def save(self) -> None:
        temporary = self.root / "summary.json.tmp"
        temporary.write_text(json.dumps(self.summary, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.root / "summary.json")

    def step(self, name, *commands, check=None) -> None:
        log_path = self.root / f"{name}.log"
        entry = {"name": name, "status": "RUNNING", "commands": commands, "log": str(log_path)}
        self.summary["stages"].append(entry)
        self.save()
        print(f"{name}: {log_path}", flush=True)
        with log_path.open("w", encoding="utf-8") as log:
            try:
                for command in commands:
                    subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
                if check is not None:
                    entry.update(check())
                entry["status"] = "PASS"
            except (Exception, KeyboardInterrupt) as error:
                entry.update(status="FAIL", error=str(error) or type(error).__name__)
                print(entry["error"], file=log)
                if isinstance(error, subprocess.CalledProcessError) and error.output:
                    print(error.output, file=log)
                raise
            finally:
                self.save()


def check_idle_world(client) -> None:
    from scripts.scenarios import traffic_ids

    world = client.get_world()
    require(world.get_map().name.rsplit("/", 1)[-1] == MAP, f"Start CARLA directly on {MAP}")
    require(not world.get_settings().synchronous_mode, "CARLA world must be in async mode")
    if traffic_ids(world):
        world.wait_for_tick(5.)
    require(not traffic_ids(world), "CARLA world must have no trial vehicles, sensors, walkers or props")


def preflight(ffmpeg: str, ffprobe: str):
    import carla
    import cv2
    import numpy

    from configs.config import CFG

    client = carla.Client(HOST, PORT)
    client.set_timeout(10.)
    server = client.get_server_version()
    require(server == client.get_client_version() == CARLA_VERSION, f"Client and server must be CARLA {CARLA_VERSION}")
    require(CFG.fixed_delta == .05, "The acceptance suite requires a 0.05 s simulation timestep")
    check_idle_world(client)
    ffmpeg_version = checked_output([ffmpeg, "-version"]).splitlines()[0]
    ffprobe_version = checked_output([ffprobe, "-version"]).splitlines()[0]
    require("libx264" in checked_output([ffmpeg, "-hide_banner", "-encoders"]), "FFmpeg lacks libx264")
    require("drawtext" in checked_output([ffmpeg, "-hide_banner", "-filters"]), "FFmpeg lacks drawtext")
    return client, {"platform": platform.platform(), "python": platform.python_version(), "carla": server,
                    "numpy": numpy.__version__, "opencv": cv2.__version__, "fixed_delta_s": CFG.fixed_delta,
                    "ffmpeg": ffmpeg_version, "ffprobe": ffprobe_version}


def record_lanes(run: Acceptance, client) -> list[Path]:
    sessions = []
    for route, spawn in (("straight", 22), ("curve", 62)):
        for weather in ("clear", "rain"):
            stage = f"record_{route}_{weather}"
            directory = run.root / "lane_data" / f"{route}_{weather}"

            def check_recording():
                path = find_artifact(directory, "session_*/meta.json")
                record = read_json(path)
                require(record["complete"] and record["frames"] == 150 and record["sensor_timeouts"] == 0,
                        f"Incomplete or unsynchronized recording in {path}")
                check_idle_world(client)
                sessions.append(path.parent)
                return {"report": str(path), "frames": record["frames"]}

            run.step(stage, python_module("scripts.record_dataset", *CONNECTION, "--frames", 150,
                                          "--driver", "tm", "--target-speed", 10, "--spawn-index", spawn,
                                          "--route", route, "--weather", weather, "--out", directory),
                     check=check_recording)
    return sessions


def evaluate_lanes(run: Acceptance, sessions: list[Path]) -> None:
    for mode in ("semantic", "rgb"):
        path = run.root / f"lane_{mode}.json"

        def check_evaluation():
            report = read_json(path)
            require([Path(session["session"]) for session in report["sessions"]] == sessions,
                    f"Expected the same four recordings in {path}")
            result = report["normalized"]
            if mode == "rgb":
                require(result["eligible_sessions"] == 4 and result["verdict"] == "PASS",
                        f"All four RGB sessions must meet M1-v2, see {path}")
            return {"report": str(path), "m1_v2": result}

        run.step(f"evaluate_{mode}", python_module("scripts.eval_lane", "--session", *sessions,
                                                   "--lane-input", mode, "--report", path), check=check_evaluation)


def run_suite(run, client, name, module, count, *arguments, phase4=False, no_lane_invasions=False) -> Path:
    directory = run.root / name

    def check_suite():
        path = find_artifact(directory, "run_*/summary.json")
        report = read_json(path)
        require(report["verdict"] == "PASS" and report["trials_completed"] == count
                and report["collision_free_trials"] == count and report["unsafe_starts"] == 0
                and all(trial["clean_async"] for trial in report["trials"]), f"Suite did not pass, see {path}")
        if phase4:
            require(report["static_stops"] == 10, f"Expected 10 controlled stops, see {path}")
        if no_lane_invasions:
            require(all(trial["metrics"]["lane_invasions"] == 0 for trial in report["trials"]),
                    f"Unexpected lane marking events, see {path}")
        check_idle_world(client)
        checks = {"report": str(path), "trials": count}
        for trial in report["trials"]:
            if "clear_road_speed" in trial:
                checks["clear_road_speed"] = trial["clear_road_speed"]
        return checks

    run.step(name, python_module(module, *CONNECTION, *arguments, "--out", directory), check=check_suite)
    return find_artifact(directory, "run_*/summary.json")


def check_demo(summary: Path, video: Path, ffprobe: str) -> dict:
    for trial in read_json(summary)["trials"]:
        report = read_json(Path(trial["report"]))
        require(report["rgb_size"] == [1920, 1080] and report["video_layout"] == "rgb-overlay"
                and report["video_fps_sim"] == 20., f"Unexpected source video profile in {trial['report']}")
    probe = json.loads(checked_output([ffprobe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                                       "stream=codec_name,width,height,avg_frame_rate,nb_frames,duration",
                                       "-of", "json", str(video)]))["streams"][0]
    require(probe["codec_name"] == "h264" and (probe["width"], probe["height"]) == (3840, 1184)
            and probe["avg_frame_rate"] == "30/1" and int(probe["nb_frames"]) == 1800
            and abs(float(probe["duration"]) - 60.) < .001, f"Unexpected demo profile in {video}")
    return {"video": str(video), "probe": probe}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("output/poc"), help="parent directory for each acceptance run")
    parser.add_argument("--ffmpeg", default="ffmpeg", help="FFmpeg with libx264 and drawtext")
    parser.add_argument("--ffprobe", default="ffprobe")
    args = parser.parse_args()
    run = Acceptance(args.out / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f"))

    def check_environment():
        nonlocal client
        client, environment = preflight(args.ffmpeg, args.ffprobe)
        run.summary["environment"] = environment
        return {}

    client = None
    exit_code = 0
    try:
        run.step("preflight", check=check_environment)
        run_suite(run, client, "clear_road_50", "scripts.run_suite", 1, "--case", "clear-road")
        evaluate_lanes(run, record_lanes(run, client))
        run_suite(run, client, "phase4", "scripts.scenarios", 20, "--suite", phase4=True)
        run_suite(run, client, "avoid_rear", "scripts.run_suite", 1, "--case", "avoid-rear")
        for weather in ("clear", "rain"):
            for case, mode in (("curve-road", "rgb"), ("cut-in", "semantic")):
                run_suite(run, client, f"{case}_{weather}", "scripts.run_suite", 1, "--case", case,
                          "--lane-input", mode, "--weather", weather, no_lane_invasions=True)
        summary = run_suite(run, client, "demo_suite", "scripts.run_suite", 6, "--video-hd", 1920)
        video = run.root / "demo_hd_60s.mp4"
        run.step("demo", python_module("scripts.make_demo", summary, "--hd-60", "--ffmpeg", args.ffmpeg,
                                       "--out", video),
                 [args.ffmpeg, "-nostdin", "-v", "error", "-xerror", "-threads", "4", "-i", str(video),
                  "-f", "null", "-"], check=lambda: check_demo(summary, video, args.ffprobe))
        run.summary["verdict"] = "PASS"
    except KeyboardInterrupt:
        run.summary["verdict"] = "INTERRUPTED"
        exit_code = 130
    except Exception as error:
        run.summary.update(verdict="FAIL", error=str(error))
        print(str(error), file=sys.stderr)
        exit_code = 1
    finally:
        run.save()
        print(f"{run.summary['verdict']} {run.root / 'summary.json'}", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
