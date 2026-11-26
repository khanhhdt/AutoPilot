"""Image lane fits to rear-axle pure pursuit; CARLA positive steer is right.

Uses a pinhole camera and a locally flat road. No map waypoints enter steering.
"""

import math
from dataclasses import dataclass

import numpy as np

from src.perception.lane_detection import LaneState


@dataclass(frozen=True)
class PursuitCommand:
    steer: float
    lookahead_m: float
    lateral_m: float
    target_u: float
    target_v: float


class PurePursuit:
    def __init__(self, lookahead_m: float = 8., wheelbase_m: float = 2.875,
                 max_steer_angle_deg: float = 70., max_steer_rate: float = 0.5,
                 camera_x_m: float = 1.6, camera_height_m: float = 1.7,
                 fov_deg: float = 90., steering_curve: list[tuple[float, float]] | None = None) -> None:
        if lookahead_m <= camera_x_m + wheelbase_m / 2 or max_steer_angle_deg <= 0:
            raise ValueError("lookahead must be ahead of the camera; steering angle must be positive")
        self.lookahead_m = lookahead_m
        self.wheelbase_m = wheelbase_m
        self.max_angle = math.radians(max_steer_angle_deg)
        self.max_steer_rate = max_steer_rate
        self.camera_offset_m = camera_x_m + wheelbase_m / 2
        self.camera_height_m = camera_height_m
        self.fov_deg = fov_deg
        self.steering_curve = sorted(steering_curve or [(0., 1.)])
        self.steer = 0.

    def reset(self) -> None:
        self.steer = 0.

    def step(self, lane: LaneState, speed_kmh: float, dt: float) -> PursuitCommand | None:
        if dt <= 0:
            raise ValueError("pure pursuit dt must be positive")
        if not lane.detected or lane.left_fit is None or lane.right_fit is None:
            return None
        width, height = lane.image_width, lane.image_height
        if width <= 0 or height <= 0:
            raise ValueError("LaneState needs camera dimensions for metric projection")
        focal = width / (2. * math.tan(math.radians(self.fov_deg) / 2.))
        lookahead = max(self.lookahead_m, speed_kmh / 3.6 * 1.5)
        camera_forward = lookahead - self.camera_offset_m
        y0 = int(height * lane.roi_top)
        v = float(np.clip(height / 2. + focal * self.camera_height_m / camera_forward,
                          y0 + 1, height - 1))
        left = float(np.polyval(lane.left_fit, v - y0))
        right = float(np.polyval(lane.right_fit, v - y0))
        u = 0.5 * (left + right)
        if not 0 <= left < right < width:
            return None
        camera_forward = focal * self.camera_height_m / (v - height / 2.)
        forward = camera_forward + self.camera_offset_m
        lateral = (u - width / 2.) * camera_forward / focal
        return self.step_target(forward, lateral, speed_kmh, dt, u, v)

    def step_target(self, forward: float, lateral: float, speed_kmh: float, dt: float,
                    u: float = 0., v: float = 0.) -> PursuitCommand:
        if dt <= 0 or forward <= 0:
            raise ValueError("pursuit target must be ahead and dt must be positive")
        angle = math.atan2(2. * self.wheelbase_m * lateral, forward * forward + lateral * lateral)
        curve_speed, curve_scale = zip(*self.steering_curve)
        # CARLA/PhysX SteeringCurve abscissa is forward speed in km/h.
        scale = max(0.1, float(np.interp(speed_kmh, curve_speed, curve_scale)))
        requested = float(np.clip(angle / (self.max_angle * scale), -1., 1.))
        delta = self.max_steer_rate * dt
        self.steer = float(np.clip(requested, self.steer - delta, self.steer + delta))
        return PursuitCommand(self.steer, forward, lateral, u, v)
