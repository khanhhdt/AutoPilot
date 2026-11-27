"""Track depth detections in the world plane using only ego pose and sensor data."""

from dataclasses import dataclass
from math import cos, radians, sin

import numpy as np

from src.perception.obstacles import Obstacle


@dataclass(frozen=True)
class TrackedObstacle:
    measurement: Obstacle
    track_id: int
    forward_m: float
    closing_mps: float | None


@dataclass
class _Track:
    id: int
    kind: str
    state: np.ndarray  # Rows are position and velocity, columns are world x and y.
    covariance: np.ndarray
    last_seen: float
    observations: int = 1

    def predict(self, transition: np.ndarray, process_noise: np.ndarray) -> None:
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise

    def correct(self, position: np.ndarray, timestamp: float) -> None:
        gain = self.covariance[:, 0] / (self.covariance[0, 0] + .36)
        self.state += gain[:, None] * (position - self.state[0])
        self.covariance -= np.outer(gain, self.covariance[0])
        self.last_seen = timestamp
        self.observations += 1


class ObstacleTracker:
    """Associate detections by predicted longitudinal and lateral position."""

    def __init__(self) -> None:
        self._next_id = 1
        self.reset()

    def reset(self) -> None:
        self._tracks: dict[int, _Track] = {}
        self._timestamp: float | None = None

    def update(self, obstacles: list[Obstacle], timestamp: float, ego_xy: tuple[float, float],
               yaw_deg: float, ego_speed_mps: float) -> list[TrackedObstacle]:
        dt = timestamp - self._timestamp if self._timestamp is not None else 0.
        if not 0. < dt <= .25:
            self.reset()
            dt = 0.
        self._timestamp = timestamp
        yaw = radians(yaw_deg)
        axes = np.array([[cos(yaw), sin(yaw)], [-sin(yaw), cos(yaw)]])
        ego = np.asarray(ego_xy)
        measurements = np.asarray([(o.forward_m, (o.left_m + o.right_m) * .5)
                                   for o in obstacles]).reshape(-1, 2) @ axes + ego
        self._tracks = {key: track for key, track in self._tracks.items() if timestamp - track.last_seen <= .25}
        transition = np.array([[1., dt], [0., 1.]])
        process_noise = 2. * np.array([[dt**3 / 3., dt**2 / 2.], [dt**2 / 2., dt]])
        for track in self._tracks.values():
            track.predict(transition, process_noise)

        candidates = []
        for key, track in self._tracks.items():
            errors = (measurements - track.state[0]) @ axes.T
            age = timestamp - track.last_seen
            for index, (forward, lateral) in enumerate(errors):
                if (obstacles[index].kind == track.kind and abs(forward) <= 2. + 6. * age
                        and abs(lateral) <= 1. + 2. * age):
                    candidates.append((forward**2 + 4. * lateral**2, key, index))
        matched: dict[int, _Track] = {}
        used = set()
        for _, key, index in sorted(candidates):
            if key in used or index in matched:
                continue
            track = self._tracks[key]
            track.correct(measurements[index], timestamp)
            matched[index] = track
            used.add(key)

        tracked = []
        for index, obstacle in enumerate(obstacles):
            track = matched.get(index)
            if track is None:
                track = _Track(self._next_id, obstacle.kind, np.stack((measurements[index], np.zeros(2))),
                               np.diag([.36, 16.]), timestamp)
                self._tracks[track.id] = track
                self._next_id += 1
            forward_m = float((track.state[0] - ego) @ axes[0])
            closing = float(ego_speed_mps - track.state[1] @ axes[0]) if track.observations >= 5 else None
            tracked.append(TrackedObstacle(obstacle, track.id, forward_m, closing))
        return tracked
