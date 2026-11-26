"""Front-camera obstacle detection from aligned CARLA tags and depth."""

from dataclasses import dataclass
from math import radians, tan

import cv2
import numpy as np

from src.perception.segmentation import SemanticMasks


@dataclass(frozen=True)
class Obstacle:
    kind: str
    forward_m: float
    left_m: float
    right_m: float
    box: tuple[int, int, int, int]


class ObstacleDetector:
    def __init__(self, fov_deg: float = 90., camera_x_m: float = 1.6,
                 camera_height_m: float = 1.7, max_range_m: float = 45.) -> None:
        self.fov_scale = 2 * tan(radians(fov_deg) / 2)
        self.camera_x_m = camera_x_m
        self.camera_height_m = camera_height_m
        self.max_range_m = max_range_m

    def detect(self, semantic: SemanticMasks, depth: np.ndarray) -> list[Obstacle]:
        height, width = depth.shape
        focal = width / self.fov_scale
        obstacles = []
        for kind, mask in (("vehicle", semantic.vehicles), ("pedestrian", semantic.pedestrians),
                           ("prop", semantic.props)):
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
            for label in range(1, count):
                x, y, w, h, area = map(int, stats[label])
                if area < 4:
                    continue
                ys, xs = np.where(labels[y:y+h, x:x+w] == label)
                ranges = depth[ys + y, xs + x]
                sideways = ((xs + x + 0.5 - width / 2) / focal) * ranges
                elevation = self.camera_height_m + ((height / 2 - ys - y - 0.5) / focal) * ranges
                valid = np.isfinite(ranges) & (ranges > .25) & (ranges < self.max_range_m)
                valid &= (elevation > -.55) & (elevation < 3.5)
                if np.count_nonzero(valid) < 4:
                    continue
                near = float(np.percentile(ranges[valid], 15))
                # Vehicle silhouettes often include both front and side surfaces.
                # Reject background depth outliers while retaining the side profile.
                in_object = valid & (ranges <= near + (6. if kind == "vehicle" else 2.))
                left, right = np.percentile(sideways[in_object], [5, 95])
                obstacles.append(Obstacle(kind, near + self.camera_x_m, float(left), float(right), (x, y, w, h)))
        return sorted(obstacles, key=lambda obstacle: obstacle.forward_m)

    def road_visible(self, road: np.ndarray, points: list[tuple[float, float, float]]) -> bool:
        """Require visible road in the ROI projected from the target lane."""
        height, width = road.shape
        focal = width / self.fov_scale
        seen = 0
        for forward, lateral, up in points:
            forward -= self.camera_x_m
            if forward <= 0:
                continue
            x = round(width / 2 + focal * lateral / forward)
            y = round(height / 2 + focal * (self.camera_height_m - up) / forward)
            if 1 <= x < width - 1 and 1 <= y < height - 1:
                seen += np.count_nonzero(road[y-1:y+2, x-1:x+2]) >= 5
        return bool(points) and seen >= len(points) * 2 / 3
