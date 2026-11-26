import argparse
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import carla
import cv2
import numpy as np

from configs.config import CFG
from src.control.pid import SpeedPID
from src.control.pure_pursuit import PurePursuit
from src.metrics import RunTelemetry, TIMING_FIELDS, summarize_lane_control, summarize_live_run, write_live_report
from src.perception.lane_detection import ROAD_LINE_TAG, ROAD_TAG, LaneDetector
from src.perception.segmentation import colorize
from src.planning.avoidance import AvoidanceController
from src.planning.decision import State
from src.planning.traffic import relative
from src.scenarios import NAMES, SPAWNS, Scenario
from src.sensors import SensorManager
from src.vehicle import EgoVehicle


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CARLA live lane perception")
    parser.add_argument("--host", default=CFG.host)
    parser.add_argument("--port", type=int, default=CFG.port)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = chạy đến khi Ctrl+C")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--frames", type=int, default=0, help="0 = không giới hạn số tick")
    parser.add_argument("--sim-duration", type=float, default=0., help="stop after this many simulation seconds")
    parser.add_argument("--map", default=None, help="load the requested CARLA map (resets the world)")
    parser.add_argument("--driver", choices=("none", "tm", "lane", "avoid"), default="none")
    parser.add_argument("--scenario", choices=NAMES, help="spawn a Phase 4 test on Town10HD_Opt")
    parser.add_argument("--scenario-seed", type=int, default=0)
    parser.add_argument("--target-speed", type=float, default=10.0, help="target speed, km/h")
    parser.add_argument("--lookahead", type=float, default=8., help="pure pursuit lookahead from rear axle, m")
    parser.add_argument("--speed-kp", type=float, default=.18)
    parser.add_argument("--speed-ki", type=float, default=.06)
    parser.add_argument("--speed-kd", type=float, default=.01)
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--weather", choices=("clear", "rain"), default=None)
    parser.add_argument("--out", type=Path, default=Path("output/live"), help="parent directory for a new run")
    parser.add_argument("--save-video", action="store_true", help="save the live RGB/semantic/lane view")
    parser.add_argument("--video-hd", nargs="?", const=1024, type=int, choices=(1024, 1920),
                        help="record the two-panel view at this native RGB width (default 1024)")
    args = parser.parse_args(argv)
    if args.video_hd and not args.save_video:
        parser.error("--video-hd requires --save-video")
    if args.duration < 0 or args.frames < 0 or args.sim_duration < 0 or args.target_speed <= 0:
        parser.error("duration/frames must be >=0 and target-speed must be >0")
    if args.lookahead <= 4 or min(args.speed_kp, args.speed_ki, args.speed_kd) < 0:
        parser.error("lookahead must exceed 4 m and PID gains must be nonnegative")
    if args.scenario:
        if args.driver != "avoid" or args.map not in (None, "Town10HD_Opt") or args.scenario_seed < 0:
            parser.error("scenarios require --driver avoid, Town10HD_Opt, and a nonnegative seed")
        args.map = "Town10HD_Opt"
        if args.spawn_index is None:
            args.spawn_index = SPAWNS[args.scenario]
        if not (args.frames or args.sim_duration or args.duration):
            args.sim_duration = 13.
    return args


def build_view(frame, lane, fps: float, control: dict | None = None,
               *, tracking_rgb: np.ndarray | None = None) -> np.ndarray:
    height, width = frame.rgb.shape[:2]
    if tracking_rgb is None:
        tags = colorize(frame.seg_tags) if control and control.get("phase4_state") else frame.seg
        semantic = cv2.resize(tags, (width, height), interpolation=cv2.INTER_NEAREST)
        body = np.hstack([frame.rgb, semantic, lane.overlay])
        overlay_x = 2 * width
    else:
        # Scale the annotations only. Both panes keep the original HD scene pixels.
        marked = np.any(lane.overlay != tracking_rgb, axis=2).astype(np.uint8)
        marked = cv2.resize(marked, (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
        annotations = cv2.resize(lane.overlay, (width, height), interpolation=cv2.INTER_NEAREST)
        overlay = frame.rgb.copy()
        overlay[marked] = annotations[marked]
        body = np.hstack([frame.rgb, overlay])
        overlay_x = width
    view = cv2.copyMakeBorder(body, 48, 56 if control else 32, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
    status = "FRESH" if lane.fresh else f"HELD {lane.missed_frames}" if lane.detected else "LOST"
    offset = f"{lane.offset_px:+.1f}px" if lane.detected else "n/a"
    cv2.putText(view, f"CARLA LIVE | frame {frame.frame} | {status} | offset {offset}",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(view, "RGB CAMERA", (10, 41), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1, cv2.LINE_AA)
    if tracking_rgb is None:
        cv2.putText(view, "SEMANTIC SEGMENTATION (SIMULATOR)", (width + 10, 41),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1, cv2.LINE_AA)
    cv2.putText(view, "LIVE LANE DETECTOR", (overlay_x + 10, 41),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1, cv2.LINE_AA)
    cv2.putText(view, f"wall FPS {fps:.1f} | speed {frame.speed_kmh:.1f} km/h | "
                f"collisions {frame.collisions} | lane invasions {frame.lane_invasions}",
                (10, body.shape[0] + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if control:
        cv2.putText(view, f"PURE PURSUIT + SPEED PID | steer {control['steer']:+.3f} | "
                    f"throttle {control['throttle']:.2f} | brake {control['brake']:.2f} | "
                    f"{control.get('phase4_state') or ('TRACKING' if control['control_valid'] else 'LANE LOST - BRAKING')}",
                    (10, view.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1, cv2.LINE_AA)
    return view


def main(argv: list[str] | None = None) -> Path:
    args = parse_args(argv)
    sensor_cfg = (replace(CFG, rgb_width=args.video_hd, rgb_height=args.video_hd * 9 // 16)
                  if args.video_hd else CFG)
    adaptive_speed = args.driver == "avoid" and args.target_speed > 20.
    if adaptive_speed:
        sensor_cfg = replace(sensor_cfg, aux_width=384, aux_height=216)
    cv2.setNumThreads(1)
    client = carla.Client(args.host, args.port)
    client.set_timeout(CFG.timeout)
    world = client.get_world()
    if args.map is not None and world.get_map().name.rsplit("/", 1)[-1] != args.map:
        client.set_timeout(120.)
        world = client.load_world(args.map)
        client.set_timeout(CFG.timeout)
    carla_map = world.get_map()
    print(f"connected {args.host}:{args.port} | map {carla_map.name}")

    original = world.get_settings()
    original_weather = world.get_weather()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = CFG.fixed_delta
    output = args.out / datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)

    ego = None
    sensors = None
    traffic_manager = None
    video = None
    detector = LaneDetector()
    telemetry = RunTelemetry()
    pursuit = None
    avoidance = None
    scenario = None
    speed_pid = SpeedPID(args.speed_kp, args.speed_ki, args.speed_kd)
    rows = []
    previous_tick = None
    frames = 0
    start = None
    stop_reason = "error"
    failure = None
    cleanup_errors = []

    try:
        world.apply_settings(settings)
        if args.weather is not None:
            world.set_weather({"clear": carla.WeatherParameters.ClearNoon,
                               "rain": carla.WeatherParameters.HardRainNoon}[args.weather])
        ego = EgoVehicle(world, CFG.vehicle_filter, spawn_index=args.spawn_index)
        if args.scenario:
            # A newly spawned actor has no location in the client snapshot until a tick.
            world.tick()
            scenario = Scenario(args.scenario, args.scenario_seed, world, ego.actor)
            scenario.setup(client)
        sensors = SensorManager(world, ego.actor, sensor_cfg)
        if args.driver == "tm":
            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            ego.actor.set_autopilot(True, traffic_manager.get_port())
            traffic_manager.auto_lane_change(ego.actor, False)
            traffic_manager.set_desired_speed(ego.actor, args.target_speed)
        elif args.driver in ("lane", "avoid"):
            physics = ego.actor.get_physics_control()
            front = [wheel for wheel in physics.wheels if wheel.max_steer_angle > 0]
            rear = [wheel for wheel in physics.wheels if wheel.max_steer_angle == 0]
            wheelbase = 2.875
            if front and rear:
                front_pos = np.mean([[w.position.x, w.position.y, w.position.z] for w in front], axis=0)
                rear_pos = np.mean([[w.position.x, w.position.y, w.position.z] for w in rear], axis=0)
                wheelbase = float(np.linalg.norm(front_pos - rear_pos) / 100.)
            pursuit = PurePursuit(
                lookahead_m=args.lookahead, wheelbase_m=wheelbase, fov_deg=CFG.fov,
                max_steer_angle_deg=max((w.max_steer_angle for w in front), default=70.),
                steering_curve=[(point.x, point.y) for point in physics.steering_curve],
            )
            if args.driver == "avoid":
                box = ego.actor.bounding_box
                avoidance = AvoidanceController(CFG.fov, box.location.x + box.extent.x, adaptive=adaptive_speed)
                ego.apply(brake=1.)
                for _ in range(5):
                    if sensors.wait_for_frame(world.tick()) is not None:
                        break
                else:
                    raise RuntimeError("Phase 4 cameras did not initialize")
        print(f"ego {ego.actor.type_id} | driver {args.driver} | target {1.0 / CFG.fixed_delta:.0f} Hz | "
              f"rgb {sensor_cfg.rgb_width}x{sensor_cfg.rgb_height} | "
              f"aux {sensor_cfg.aux_width}x{sensor_cfg.aux_height} | output {output}", flush=True)
        start = time.monotonic()
        window_start = start
        window_frames = 0

        while True:
            if args.duration and time.monotonic() - start >= args.duration:
                stop_reason = "duration"
                break
            if args.frames and len(rows) >= args.frames:
                stop_reason = "frame_limit"
                break
            if args.sim_duration and len(rows) * CFG.fixed_delta >= args.sim_duration:
                stop_reason = "sim_duration"
                break

            loop_start = time.perf_counter()
            row = {key: 0.0 for key in TIMING_FIELDS}
            row.update(sensor_frame=None, timestamp_sim=None, speed_kmh=None,
                       is_junction=None, collisions=None, lane_invasions=None,
                       detected=False, fresh=False, missed_frames=None, offset_px=None)
            row.update(sim_elapsed_s=(len(rows) + 1) * CFG.fixed_delta, target_speed_kmh=args.target_speed,
                       steer=None, throttle=None, brake=None, control_valid=None,
                       pursuit_forward_m=None, pursuit_lateral_m=None, lane_error_m=None,
                       heading_error_rad=None, lane_width_m=None)
            target_frame = world.tick()
            row["tick_ms"] = (time.perf_counter() - loop_start) * 1000.0
            row["tick_frame"] = target_frame
            row["frame_gap"] = previous_tick is not None and target_frame != previous_tick + 1
            previous_tick = target_frame
            if row["frame_gap"]:
                detector.reset()
                if avoidance:
                    avoidance.sensor_lost()
            wait_start = time.perf_counter()
            frame = sensors.wait_for_frame(target_frame, timeout=5. if scenario else 2.)
            row["sensor_wait_ms"] = (time.perf_counter() - wait_start) * 1000.0
            stop_requested = False

            if frame is None:
                row.update(telemetry.update(row["sim_elapsed_s"], None))
                detector.reset()
                if avoidance:
                    avoidance.sensor_lost()
                    row.update(phase4_state=State.STOP.value, phase4_target_speed_kmh=0.)
                if pursuit is not None:
                    ego.apply(steer=pursuit.steer, brake=1.)
                    speed_pid.reset()
                    row.update(steer=pursuit.steer, throttle=0., brake=1., control_valid=False)
                print("WARN: sensor timeout, bỏ qua frame", flush=True)
            else:
                frames += 1
                window_frames += 1
                preprocess_start = time.perf_counter()
                road = ((frame.seg_tags == ROAD_TAG) | (frame.seg_tags == ROAD_LINE_TAG)).astype(np.uint8) * 255
                tracking_rgb = (cv2.resize(frame.rgb, (CFG.rgb_width, CFG.rgb_height), interpolation=cv2.INTER_AREA)
                                if args.video_hd else frame.rgb)
                road = cv2.resize(road, (tracking_rgb.shape[1], tracking_rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                road = cv2.dilate(road, np.ones((5, 5), np.uint8))
                row["preprocess_ms"] = (time.perf_counter() - preprocess_start) * 1000.0
                perception_start = time.perf_counter()
                lane = detector.detect(tracking_rgb, road_mask=road)
                row["perception_ms"] = (time.perf_counter() - perception_start) * 1000.0
                if avoidance:
                    transform = ego.actor.get_transform()
                    waypoint = carla_map.get_waypoint(transform.location)
                if pursuit is not None:
                    control_start = time.perf_counter()
                    speed_target = args.target_speed
                    plan_stop = False
                    target = None
                    if avoidance:
                        assessment = avoidance.step(frame, world, ego.actor, transform, waypoint,
                                                    lane.fresh, lane.detected and not row["frame_gap"],
                                                    speed_target, CFG.fixed_delta)
                        decision = assessment.decision
                        target = assessment.target
                        speed_target = decision.speed_kmh
                        plan_stop = decision.state == State.STOP or row["frame_gap"]
                        row.update(phase4_state=State.STOP.value if plan_stop else decision.state.value,
                                   phase4_target_speed_kmh=0. if plan_stop else speed_target,
                                   phase4_obstacle_count=len(assessment.obstacles),
                                   phase4_lead_kind=assessment.lead.kind if assessment.lead else None,
                                   phase4_gap_m=assessment.lead.gap_m if assessment.lead else None,
                                   phase4_left_clear=assessment.clear.get("left", False),
                                   phase4_right_clear=assessment.clear.get("right", False),
                                   phase4_change_started=decision.start_side,
                                   phase4_change_completed=assessment.completed,
                                   phase4_target_blocker=assessment.target_blocker,
                                   closing_mps=assessment.lead.closing_mps if assessment.lead else None,
                                   visibility_m=assessment.visible_m, speed_limit_kmh=assessment.speed_limit_kmh,
                                   lane_reference=assessment.reference)
                        if adaptive_speed:
                            acceleration = ego.actor.get_acceleration()
                            right = transform.get_right_vector()
                            row["lateral_accel_mps2"] = (
                                acceleration.x * right.x + acceleration.y * right.y + acceleration.z * right.z)
                        aux_height, aux_width = frame.seg_tags.shape
                        height, width = lane.overlay.shape[:2]
                        for obstacle in assessment.obstacles:
                            x, y, w, h = obstacle.box
                            corner = (x * width // aux_width, y * height // aux_height)
                            cv2.rectangle(lane.overlay, corner, ((x+w) * width // aux_width,
                                          (y+h) * height // aux_height), (0, 180, 255), 1)
                            cv2.putText(lane.overlay, f"{obstacle.kind} {obstacle.forward_m:.1f}m", corner,
                                        cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 180, 255), 1)
                    command = None
                    if not plan_stop and target:
                        forward, lateral, _ = relative(transform, target.transform.location)
                        forward += pursuit.camera_offset_m - 1.6
                        if forward > 0.:
                            command = pursuit.step_target(forward, lateral, frame.speed_kmh, CFG.fixed_delta)
                    elif not plan_stop:
                        command = pursuit.step(lane, frame.speed_kmh, CFG.fixed_delta)
                    if command is None:
                        speed_pid.reset()
                        steer, throttle, brake = pursuit.steer, 0., 1.
                    else:
                        steer = command.steer
                        throttle, brake = speed_pid.step(speed_target, frame.speed_kmh, CFG.fixed_delta)
                        row.update(pursuit_forward_m=command.lookahead_m, pursuit_lateral_m=command.lateral_m)
                        if not target:
                            cv2.circle(lane.overlay, (round(command.target_u), round(command.target_v)),
                                       4, (0, 255, 255), -1)
                    ego.apply(steer=steer, throttle=throttle, brake=brake)
                    row.update(steer=steer, throttle=throttle, brake=brake,
                               control_valid=command is not None or plan_stop)
                    row["control_ms"] = (time.perf_counter() - control_start) * 1000.0
                telemetry_start = time.perf_counter()
                if not avoidance:
                    transform = ego.actor.get_transform()
                    waypoint = carla_map.get_waypoint(transform.location)
                if pursuit is not None and waypoint is not None:
                    right = waypoint.transform.get_right_vector()
                    delta = transform.location - waypoint.transform.location
                    yaw_delta = transform.rotation.yaw - waypoint.transform.rotation.yaw
                    row.update(lane_error_m=delta.x * right.x + delta.y * right.y,
                               lane_width_m=waypoint.lane_width,
                               heading_error_rad=float(np.deg2rad((yaw_delta + 180.) % 360. - 180.)))
                lead_kind = row.get("phase4_lead_kind")
                lead_key = (waypoint.road_id, waypoint.lane_id, lead_kind) if waypoint and lead_kind else None
                pos = transform.location
                row.update(telemetry.update(frame.timestamp, (pos.x, pos.y, pos.z), lead_key,
                                            row.get("phase4_gap_m"), frame_gap=row["frame_gap"]))
                row.update(sensor_frame=frame.frame, timestamp_sim=frame.timestamp,
                           speed_kmh=frame.speed_kmh, is_junction=bool(waypoint.is_junction) if waypoint else None,
                           collisions=frame.collisions, lane_invasions=frame.lane_invasions,
                           detected=lane.detected, fresh=lane.fresh, missed_frames=lane.missed_frames,
                           offset_px=lane.offset_px if lane.detected else None)
                row["telemetry_ms"] = (time.perf_counter() - telemetry_start) * 1000.0
                if scenario and scenario.step(row):
                    stop_reason = "scenario_complete"
                    stop_requested = True

                if not args.no_display or args.save_video:
                    view_start = time.perf_counter()
                    fps = window_frames / max(time.monotonic() - window_start, 1e-6)
                    view = build_view(frame, lane, fps, row if pursuit is not None else None,
                                      tracking_rgb=tracking_rgb if args.video_hd else None)
                    if not args.no_display:
                        cv2.imshow("CARLA", view)
                        stop_requested = (cv2.waitKey(1) & 0xFF) in (27, ord("q"))
                    row["view_ms"] = (time.perf_counter() - view_start) * 1000.0
                    if args.save_video:
                        video_start = time.perf_counter()
                        if video is None:
                            video = cv2.VideoWriter(str(output / "live.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                                    1.0 / CFG.fixed_delta, (view.shape[1], view.shape[0]))
                            if not video.isOpened():
                                raise RuntimeError("could not open live video writer")
                        video.write(view)
                        row["video_ms"] = (time.perf_counter() - video_start) * 1000.0
                now = time.monotonic()
                if now - window_start >= 1.0:
                    fps = window_frames / (now - window_start)
                    control_text = (f" steer={row['steer']:+.3f} "
                                    f"speed_err={args.target_speed-frame.speed_kmh:+.2f} "
                                    f"lane_inv={frame.lane_invasions}") if pursuit is not None else ""
                    if avoidance:
                        control_text += f" state={row['phase4_state']}"
                        if row["phase4_gap_m"] is not None:
                            control_text += f" {row['phase4_lead_kind']} gap={row['phase4_gap_m']:.1f}m"
                    print(f"fps={fps:4.1f} speed={frame.speed_kmh:5.1f} km/h | lane="
                          f"{'fresh' if lane.fresh else 'held' if lane.detected else 'lost'} | "
                          f"perception={row['perception_ms']:.1f}ms collisions={frame.collisions}"
                          f"{control_text}", flush=True)
                    window_start, window_frames = now, 0
                if frame.collisions:
                    stop_reason = "collision"
                    stop_requested = True

            elapsed = time.perf_counter() - loop_start
            row["work_ms"] = elapsed * 1000.0
            sleep_start = time.perf_counter()
            if elapsed < CFG.fixed_delta:
                time.sleep(CFG.fixed_delta - elapsed)
            row["sleep_ms"] = (time.perf_counter() - sleep_start) * 1000.0
            row["loop_ms"] = (time.perf_counter() - loop_start) * 1000.0
            rows.append(row)
            if stop_requested:
                if stop_reason not in ("collision", "scenario_complete"):
                    stop_reason = "user_exit"
                break
    except KeyboardInterrupt:
        stop_reason = "interrupted"
        print("interrupted")
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        elapsed = time.monotonic() - start if start is not None else 0.0
        cleanup = []
        if video is not None:
            cleanup.append(("video", video.release))
        if sensors is not None:
            cleanup.append(("sensors", sensors.close))
        if scenario is not None:
            cleanup.append(("scenario", scenario.close))
        if ego is not None:
            if traffic_manager is not None and ego.actor is not None and ego.actor.is_alive:
                cleanup.append(("autopilot", lambda: ego.actor.set_autopilot(False, traffic_manager.get_port())))
            cleanup.append(("ego", ego.destroy))
        cleanup.extend([("weather", lambda: world.set_weather(original_weather)),
                        ("world", lambda: world.apply_settings(original))])
        if traffic_manager is not None:
            cleanup.append(("traffic_manager", lambda: traffic_manager.set_synchronous_mode(original.synchronous_mode)))
        for name, action in cleanup:
            try:
                action()
            except RuntimeError as exc:
                cleanup_errors.append(f"{name}: {exc}")
                print(f"WARN: cleanup {name}: {exc}", flush=True)
        if not args.no_display:
            cv2.destroyAllWindows()
        report = summarize_live_run(rows, elapsed, CFG.fixed_delta)
        report.update(schema_version="live-v1", stop_reason=stop_reason, error=failure, cleanup_errors=cleanup_errors,
                      host=args.host, port=args.port, map=carla_map.name, driver=args.driver,
                      target_speed_kmh=args.target_speed, spawn_index=args.spawn_index,
                      weather=args.weather or "unchanged", rgb_size=[sensor_cfg.rgb_width, sensor_cfg.rgb_height],
                      perception_rgb_size=[CFG.rgb_width, CFG.rgb_height],
                      video_layout=("rgb-overlay" if args.video_hd else "rgb-semantic-overlay") if args.save_video else None,
                      aux_size=[sensor_cfg.aux_width, sensor_cfg.aux_height], adaptive_speed=adaptive_speed,
                      sensor_sync="exact_tick_frame_id",
                      input_mode=("RGB + semantic tags + depth + CARLA lane and actor geometry"
                                  if avoidance else "RGB + simulator road mask"),
                      perception_mode="live_in_tick_loop",
                      video_fps_sim=1.0 / CFG.fixed_delta if args.save_video else None,
                      opencv_version=cv2.__version__, opencv_threads=cv2.getNumThreads())
        if pursuit is not None:
            report["control_parameters"] = {
                "lookahead_m": pursuit.lookahead_m, "wheelbase_m": pursuit.wheelbase_m,
                "max_steer_angle_deg": float(np.rad2deg(pursuit.max_angle)),
                "max_steer_rate": pursuit.max_steer_rate,
                "camera_height_m": pursuit.camera_height_m, "camera_to_rear_axle_m": pursuit.camera_offset_m,
                "speed_kp": speed_pid.kp, "speed_ki": speed_pid.ki, "speed_kd": speed_pid.kd,
            }
            if avoidance is None:
                report["phase3"] = summarize_lane_control(rows, CFG.fixed_delta)
        if args.driver == "avoid":
            report["phase4"] = {
                "states": {state.value: sum(row.get("phase4_state") == state.value for row in rows) for state in State},
                "starts": {side: sum(row.get("phase4_change_started") == side for row in rows)
                           for side in ("left", "right")},
                "completed_changes": sum(bool(row.get("phase4_change_completed")) for row in rows),
                "unsafe_starts": sum(bool(row.get("phase4_change_started") and
                                          not row.get(f"phase4_{row['phase4_change_started']}_clear")) for row in rows),
                "scenario": scenario.result() if scenario else None,
            }
        write_live_report(output, rows, report)
        print(f"DONE frames={frames} elapsed={elapsed:.1f}s avg_fps={report['avg_fps_wall']:.1f} | "
              f"timeouts={report['sensor_timeouts']} lost={report['lost_frames']} | report {output / 'report.json'}", flush=True)
    return output


if __name__ == "__main__":
    main()
