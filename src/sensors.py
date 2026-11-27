import math
import threading
import time
from dataclasses import dataclass
import carla
import numpy as np

LIDAR_HEIGHT_M = 2.


@dataclass(frozen=True)
class LidarScan:
    frame: int
    transform: carla.Transform
    points: np.ndarray


@dataclass
class SensorFrame:
    rgb: np.ndarray
    seg: np.ndarray
    seg_tags: np.ndarray
    depth: np.ndarray
    speed_kmh: float
    collisions: int
    lane_invasions: int
    frame: int
    timestamp: float
    lidar: LidarScan | None = None


def _decode_rgb(image: carla.Image) -> np.ndarray:
    raw = np.frombuffer(image.raw_data, dtype=np.uint8)
    return raw.reshape(image.height, image.width, 4)[:, :, :3].copy()


def _decode_depth(image: carla.Image) -> np.ndarray:
    raw = np.frombuffer(image.raw_data, dtype=np.uint8)
    bgra = raw.reshape(image.height, image.width, 4).astype(np.float32)
    normalized = (bgra[:, :, 2] + bgra[:, :, 1] * 256.0 + bgra[:, :, 0] * 65536.0) / (256.0 ** 3 - 1.0)
    return (normalized * 1000.0).astype(np.float32)


class SensorManager:
    """Owns cameras, event sensors and the optional avoidance LiDAR.
    Assembles exactly one SensorFrame per world.tick() using the shared frame id.
    """

    def __init__(self, world: carla.World, vehicle: carla.Actor, cfg, *, lidar: bool = False) -> None:
        self._world = world
        self._vehicle = vehicle
        self._cfg = cfg
        self._lock = threading.Lock()
        self._ids = {"rgb": -1, "seg": -1, "depth": -1}
        if lidar:
            self._ids["lidar"] = -1
            box = vehicle.bounding_box
            self._lidar_self_center = np.array([box.location.x, box.location.y, box.location.z - LIDAR_HEIGHT_M])
            self._lidar_self_extent = np.array([box.extent.x, box.extent.y, box.extent.z]) + .08
        self._lidar = None
        self._rgb = None
        self._timestamp = 0.0
        self._seg = None
        self._tags = None
        self._depth = None
        self._collisions = 0
        self._lane_invasions = 0
        self._actors: list[carla.Actor] = []
        try:
            self._spawn()
        except (Exception, KeyboardInterrupt):
            self.close()
            raise

    def _camera(self, kind: str, width: int, height: int) -> carla.Actor:
        blueprint = self._world.get_blueprint_library().find(f"sensor.camera.{kind}")
        blueprint.set_attribute("role_name", f"ego_project_{kind}")
        blueprint.set_attribute("image_size_x", str(width))
        blueprint.set_attribute("image_size_y", str(height))
        blueprint.set_attribute("fov", str(self._cfg.fov))
        blueprint.set_attribute("sensor_tick", str(self._cfg.sensor_tick))
        transform = carla.Transform(carla.Location(x=1.6, z=1.7))
        return self._world.spawn_actor(blueprint, transform, attach_to=self._vehicle)

    def _spawn(self) -> None:
        rgb_cam = self._camera("rgb", self._cfg.rgb_width, self._cfg.rgb_height)
        self._actors.append(rgb_cam)
        seg_cam = self._camera("semantic_segmentation", self._cfg.aux_width, self._cfg.aux_height)
        self._actors.append(seg_cam)
        depth_cam = self._camera("depth", self._cfg.aux_width, self._cfg.aux_height)
        self._actors.append(depth_cam)

        blueprint_library = self._world.get_blueprint_library()
        collision_bp = blueprint_library.find("sensor.other.collision")
        collision_bp.set_attribute("role_name", "ego_project_collision")
        lane_invasion_bp = blueprint_library.find("sensor.other.lane_invasion")
        lane_invasion_bp.set_attribute("role_name", "ego_project_lane_invasion")
        collision = self._world.spawn_actor(collision_bp, carla.Transform(), attach_to=self._vehicle)
        self._actors.append(collision)
        lane_invasion = self._world.spawn_actor(lane_invasion_bp, carla.Transform(), attach_to=self._vehicle)
        self._actors.append(lane_invasion)
        if "lidar" in self._ids:
            blueprint = blueprint_library.find("sensor.lidar.ray_cast")
            for name, value in {
                "role_name": "ego_project_lidar", "channels": 32, "range": 80, "horizontal_fov": 360,
                "upper_fov": 5, "lower_fov": -24, "points_per_second": 320000,
                "rotation_frequency": 1. / self._cfg.fixed_delta, "sensor_tick": 0,
                "dropoff_general_rate": 0,
            }.items():
                blueprint.set_attribute(name, str(value))
            lidar = self._world.spawn_actor(blueprint, carla.Transform(carla.Location(z=LIDAR_HEIGHT_M)),
                                             attach_to=self._vehicle)
            self._actors.append(lidar)
            lidar.listen(self._on_lidar)

        rgb_cam.listen(self._on_rgb)
        seg_cam.listen(self._on_seg)
        depth_cam.listen(self._on_depth)
        collision.listen(self._on_collision)
        lane_invasion.listen(self._on_lane_invasion)

    def _on_rgb(self, image: carla.Image) -> None:
        rgb = _decode_rgb(image)
        with self._lock:
            self._rgb = rgb
            self._timestamp = image.timestamp
            self._ids["rgb"] = image.frame

    def _on_seg(self, image: carla.Image) -> None:
        raw = np.frombuffer(image.raw_data, dtype=np.uint8)
        tags = raw.reshape(image.height, image.width, 4)[:, :, 2].copy()
        image.convert(carla.ColorConverter.CityScapesPalette)
        pal = np.frombuffer(image.raw_data, dtype=np.uint8)
        palette = pal.reshape(image.height, image.width, 4)[:, :, :3].copy()
        with self._lock:
            self._tags = tags
            self._seg = palette
            self._ids["seg"] = image.frame

    def _on_depth(self, image: carla.Image) -> None:
        depth = _decode_depth(image)
        with self._lock:
            self._depth = depth
            self._ids["depth"] = image.frame

    def _on_collision(self, _event) -> None:
        with self._lock:
            self._collisions += 1

    def _on_lidar(self, measurement: carla.LidarMeasurement) -> None:
        points = np.frombuffer(measurement.raw_data, dtype=np.float32).reshape(-1, 4)[:, :3]
        points = points[(abs(points - self._lidar_self_center) > self._lidar_self_extent).any(axis=1)]
        scan = LidarScan(measurement.frame, measurement.transform, points)
        with self._lock:
            self._lidar = scan
            self._ids["lidar"] = scan.frame

    def _on_lane_invasion(self, _event) -> None:
        with self._lock:
            self._lane_invasions += 1

    def wait_for_frame(self, target_frame: int, timeout: float = 2.0) -> SensorFrame | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if any(frame_id > target_frame for frame_id in self._ids.values()):
                    return None
                matched = all(frame_id == target_frame for frame_id in self._ids.values())
                if matched and self._rgb is not None and self._seg is not None and self._depth is not None:
                    velocity = self._vehicle.get_velocity()
                    speed_kmh = 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                    return SensorFrame(
                        rgb=self._rgb,
                        seg=self._seg,
                        seg_tags=self._tags,
                        depth=self._depth,
                        speed_kmh=speed_kmh,
                        collisions=self._collisions,
                        lane_invasions=self._lane_invasions,
                        frame=target_frame,
                        timestamp=self._timestamp,
                        lidar=self._lidar,
                    )
            time.sleep(0.001)
        return None

    def close(self) -> None:
        for actor in self._actors:
            try:
                if actor.is_listening:
                    actor.stop()
            except RuntimeError:
                pass
            try:
                actor.destroy()
            except RuntimeError:
                pass
        self._actors = []
