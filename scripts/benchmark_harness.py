"""Isolate CARLA tick, camera delivery and production callback decoding.

No perception, control, display or image/video writes in the measured loop.
"""

import argparse
import csv
import json
import time
from collections import Counter
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import carla
import numpy as np

from configs.config import CFG
from src.sensors import SensorManager
from src.vehicle import EgoVehicle


class ProfiledSensors(SensorManager):
    def __init__(self, world, vehicle, cfg, enabled, decode, postprocess=True):
        self.enabled = enabled
        self.decode = decode
        self.postprocess = postprocess
        self.profiles = {}
        try:
            super().__init__(world, vehicle, cfg)
        except Exception:
            self.close()
            raise

    def _camera(self, kind, width, height):
        if kind != "rgb" or self.postprocess:
            return super()._camera(kind, width, height)
        blueprint = self._world.get_blueprint_library().find("sensor.camera.rgb")
        for name, value in {"role_name": "ego_project_rgb", "image_size_x": str(width),
                            "image_size_y": str(height), "fov": str(self._cfg.fov),
                            "sensor_tick": str(self._cfg.sensor_tick),
                            "enable_postprocess_effects": "false"}.items():
            blueprint.set_attribute(name, value)
        return self._world.spawn_actor(blueprint, carla.Transform(carla.Location(x=1.6, z=1.7)),
                                       attach_to=self._vehicle)

    def _spawn(self):
        # Inactive camera buffers are sentinels; wait_for_frame still checks every active ID.
        self._rgb = self._seg = self._tags = self._depth = np.empty(0)
        self._ids = {name: -1 for name in self.enabled}
        for name in self.enabled:
            rgb = name == "rgb"
            sensor = self._camera("semantic_segmentation" if name == "seg" else name,
                                  self._cfg.rgb_width if rgb else self._cfg.aux_width,
                                  self._cfg.rgb_height if rgb else self._cfg.aux_height)
            self._actors.append(sensor)
            sensor.listen(lambda image, kind=name: self._capture(kind, image))
        for kind, callback in (("collision", self._on_collision), ("lane_invasion", self._on_lane_invasion)):
            blueprint = self._world.get_blueprint_library().find(f"sensor.other.{kind}")
            blueprint.set_attribute("role_name", f"ego_project_bench_{kind}")
            sensor = self._world.spawn_actor(blueprint, carla.Transform(), attach_to=self._vehicle)
            self._actors.append(sensor)
            sensor.listen(callback)

    def _capture(self, name, image):
        entered = time.perf_counter()
        size = len(image.raw_data)
        if self.decode:
            getattr(SensorManager, f"_on_{name}")(self, image)
        else:
            with self._lock:
                self._ids[name] = image.frame
                self._timestamp = image.timestamp
        ready = time.perf_counter()
        with self._lock:
            self.profiles.setdefault(image.frame, {})[name] = (entered, ready, size)

    def wait_legacy(self, previous, timeout=2.):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            with self._lock:
                if all(self._ids[name] != previous[name] for name in self.enabled):
                    return dict(self._ids)
            time.sleep(.001)
        return None


def stats(values):
    values = [value for value in values if value is not None]
    return {"mean": float(np.mean(values)), "p50": float(np.median(values)),
            "p95": float(np.percentile(values, 95))} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=CFG.host)
    parser.add_argument("--port", type=int, default=CFG.port)
    parser.add_argument("--duration", type=float, default=60.)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--sensors", choices=("none", "rgb", "seg", "depth", "rgb_seg", "all"), default="all")
    parser.add_argument("--raw", action="store_true", help="receive same camera payload without Python decoding")
    parser.add_argument("--unpaced", action="store_true", help="measure capacity without the 20 Hz sleep")
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--no-postprocess", action="store_true")
    parser.add_argument("--reverse-order", action="store_true")
    parser.add_argument("--workers", type=int, default=0, help="LibCarla streaming worker threads; 0 = default")
    parser.add_argument("--sync-policy", choices=("exact", "legacy"), default="exact")
    parser.add_argument("--production", action="store_true", help="use unmodified production SensorManager, without callback profiling")
    parser.add_argument("--rgb-width", type=int, default=CFG.rgb_width)
    parser.add_argument("--aux-width", type=int, default=CFG.aux_width)
    parser.add_argument("--spawn-index", type=int, default=22)
    parser.add_argument("--label", required=True)
    parser.add_argument("--out", type=Path, default=Path("output/phase1_diagnosis"))
    args = parser.parse_args()
    if args.duration <= 0 or min(args.rgb_width, args.aux_width) < 16 or args.warmup < 0:
        parser.error("invalid duration, camera size or warmup")
    if args.production and (args.sensors != "all" or args.raw or args.no_postprocess
                            or args.reverse_order or args.sync_policy != "exact"):
        parser.error("production mode requires all cameras, decoding, and exact synchronization")
    enabled = {"none": [], "all": ["rgb", "seg", "depth"]}.get(args.sensors, args.sensors.split("_"))
    if args.reverse_order:
        enabled.reverse()
    cfg = replace(CFG, rgb_width=args.rgb_width, rgb_height=round(args.rgb_width * 9 / 16),
                  aux_width=args.aux_width, aux_height=round(args.aux_width * 9 / 16))
    client = carla.Client(args.host, args.port, args.workers)
    client.set_timeout(cfg.timeout)
    world = client.get_world()
    original = world.get_settings()
    weather = world.get_weather()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = cfg.fixed_delta
    settings.no_rendering_mode = args.no_rendering
    ego = sensors = None
    rows = []
    cleanup_errors = []
    failure = None
    started = None
    elapsed = 0.
    last_ready = Counter()
    print(f"START {args.label} | {args.host}:{args.port} | sensors={enabled} decode={not args.raw}", flush=True)
    try:
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        ego = EgoVehicle(world, cfg.vehicle_filter, args.spawn_index)
        sensors = (SensorManager(world, ego.actor, cfg) if args.production else
                   ProfiledSensors(world, ego.actor, cfg, enabled, not args.raw, not args.no_postprocess))
        for _ in range(args.warmup):
            target = world.tick()
            if sensors.wait_for_frame(target) is None:
                raise RuntimeError("sensor timeout during warmup")
            with sensors._lock:
                getattr(sensors, "profiles", {}).pop(target, None)
        started = time.perf_counter()
        while time.perf_counter() - started < args.duration:
            begin = time.perf_counter()
            with sensors._lock:
                previous = dict(sensors._ids)
            target = world.tick()
            tick_done = time.perf_counter()
            frame = (sensors.wait_for_frame(target) if args.sync_policy == "exact"
                     else sensors.wait_legacy(previous))
            wait_done = time.perf_counter()
            with sensors._lock:
                ids = dict(sensors._ids)
                profiles = getattr(sensors, "profiles", {})
                profile = {name: profiles.get(ids[name], {}).get(name) for name in enabled}
                profile = {name: value for name, value in profile.items() if value is not None}
                if args.sync_policy == "exact":
                    profiles.pop(target, None)
            row = {"frame": target, "valid": frame is not None,
                   "mixed_frame_ids": len(set(ids.values())) > 1,
                   "stale_to_tick": any(value != target for value in ids.values()),
                   "tick_ms": (tick_done-begin)*1000., "wait_ms": (wait_done-tick_done)*1000.}
            for name in ("rgb", "seg", "depth"):
                observation = profile.get(name)
                row[f"{name}_enter_ms"] = (observation[0]-begin)*1000. if observation else None
                row[f"{name}_decode_ms"] = (observation[1]-observation[0])*1000. if observation else None
                row[f"{name}_bytes"] = observation[2] if observation else None
                row[f"{name}_frame"] = ids.get(name)
            if profile:
                last_ready[max(profile, key=lambda name: profile[name][1])] += 1
            row["work_ms"] = (time.perf_counter()-begin)*1000.
            if not args.unpaced:
                time.sleep(max(0., cfg.fixed_delta-row["work_ms"]/1000.))
            row["loop_ms"] = (time.perf_counter()-begin)*1000.
            rows.append(row)
    except KeyboardInterrupt:
        failure = "interrupted"
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        elapsed = time.perf_counter()-started if started is not None else 0.
        for name, action in [
            ("sensors", sensors.close if sensors else lambda: None),
            ("ego", ego.destroy if ego else lambda: None),
            ("weather", lambda: world.set_weather(weather)),
            ("world", lambda: world.apply_settings(original)),
        ]:
            try:
                action()
            except RuntimeError as exc:
                cleanup_errors.append(f"{name}: {exc}")
        report = {"label": args.label, "host": args.host, "map": world.get_map().name,
                  "sensors": enabled, "decode": not args.raw, "paced": not args.unpaced,
                  "no_rendering": args.no_rendering, "warmup_ticks": args.warmup,
                  "postprocess": not args.no_postprocess, "workers": args.workers,
                  "production_sensor_manager": args.production,
                  "sync_policy": args.sync_policy,
                  "mixed_frame_ids": sum(row["mixed_frame_ids"] for row in rows),
                  "stale_to_tick": sum(row["stale_to_tick"] for row in rows),
                  "rgb_size": [cfg.rgb_width, cfg.rgb_height], "aux_size": [cfg.aux_width, cfg.aux_height],
                  "frames": len(rows), "elapsed_s": elapsed,
                  "fps": sum(row["valid"] for row in rows)/elapsed if elapsed else 0.,
                  "timeouts": sum(not row["valid"] for row in rows),
                  "last_ready_sensor": dict(last_ready), "error": failure, "cleanup_errors": cleanup_errors,
                  "timing_ms": {key: stats([row[key] for row in rows]) for key in
                                ["tick_ms", "wait_ms", "work_ms", "loop_ms"] +
                                [f"{sensor}_{stage}_ms" for sensor in ("rgb", "seg", "depth") for stage in ("enter", "decode")]},
                  "payload_bytes_per_tick": {name: stats([row[f"{name}_bytes"] for row in rows])
                                             for name in enabled}}
        args.out.mkdir(parents=True, exist_ok=True)
        stem = args.out / f"{args.label}_{datetime.now():%Y%m%d_%H%M%S}"
        stem.with_suffix(".json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        if rows:
            with stem.with_suffix(".csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
        print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
