"""Map geometry and sensor-based clearance for lane changes."""

from dataclasses import dataclass
from math import cos, radians

import carla
import numpy as np

from src.perception.obstacles import Obstacle, ObstacleDetector
from src.perception.tracking import TrackedObstacle
from src.planning.decision import Lead
from src.planning.speed import MAX_CRUISE_KMH, curve_approach_speed_kmh
from src.sensors import LidarScan


def relative(transform: carla.Transform, point: carla.Location) -> tuple[float, float, float]:
    delta = point - transform.location
    return tuple(delta.x * axis.x + delta.y * axis.y + delta.z * axis.z for axis in (
        transform.get_forward_vector(), transform.get_right_vector(), transform.get_up_vector()))


def ahead(waypoint: carla.Waypoint, distance: float) -> carla.Waypoint | None:
    if distance <= .1:
        return waypoint
    options = waypoint.next(distance)
    return options[0] if len(options) == 1 else None


def same_direction(a: carla.Waypoint, b: carla.Waypoint, limit: float = 25.) -> bool:
    return cos(radians(a.transform.rotation.yaw - b.transform.rotation.yaw)) >= cos(radians(limit))


def lane_key(waypoint: carla.Waypoint) -> tuple[int, int]:
    return waypoint.road_id, waypoint.lane_id


def nearest_lead(obstacles: list[TrackedObstacle], waypoint: carla.Waypoint | None,
                 pose: carla.Transform, bumper_m: float) -> Lead | None:
    for track in obstacles:
        obstacle = track.measurement
        center = ahead(waypoint, max(0., obstacle.forward_m - relative(pose, waypoint.transform.location)[0])) \
            if waypoint else None
        lateral = relative(pose, center.transform.location)[1] if center else 0.
        half_width = min(1.55, (waypoint.lane_width * .44) if waypoint else 1.35)
        if obstacle.left_m <= lateral + half_width and obstacle.right_m >= lateral - half_width:
            closing = track.closing_mps
            ttc = max(0., track.forward_m - bumper_m) / closing if closing is not None and closing > .2 else None
            return Lead(obstacle.kind, max(0., obstacle.forward_m - bumper_m),
                        (obstacle.left_m + obstacle.right_m) / 2 - lateral, closing, track.track_id, ttc)
    return None


@dataclass(frozen=True)
class LaneChange:
    side: str
    target: tuple[int, int]
    start: carla.Location
    minimum_travel_m: float

    def waypoint(self, current: carla.Waypoint | None) -> carla.Waypoint | None:
        if current is None:
            return None
        for lane in (current, current.get_left_lane(), current.get_right_lane()):
            if lane and lane_key(lane) == self.target:
                return lane
        return None

    def centered(self, lane: carla.Waypoint, pose: carla.Transform) -> bool:
        yaw_difference = lane.transform.rotation.yaw - pose.rotation.yaw
        return (abs(relative(lane.transform, pose.location)[1]) < .35
                and cos(radians(yaw_difference)) > cos(radians(8.)))


def adjacent_lanes(origin: carla.Waypoint) -> dict[str, carla.Waypoint]:
    if origin.is_junction:
        return {}
    result = {}
    for side, marking, adjacent in (("left", origin.left_lane_marking, origin.get_left_lane()),
                                    ("right", origin.right_lane_marking, origin.get_right_lane())):
        if not adjacent or marking is None or adjacent.lane_type != carla.LaneType.Driving:
            continue
        allowed = carla.LaneChange.Left if side == "left" else carla.LaneChange.Right
        if not (marking.lane_change & allowed) or not same_direction(origin, adjacent):
            continue
        ends = [ahead(lane, distance) for lane in (origin, adjacent) for distance in (7., 20., 27.)]
        if all(end and not end.is_junction and same_direction(origin, end, 15.) for end in ends):
            result[side] = adjacent
    return result


def road_horizon(lane: carla.Waypoint, pose: carla.Transform, road: np.ndarray,
                 detector: ObstacleDetector) -> tuple[float, float]:
    """Return visible distance and speed available before upcoming bends."""
    visible, curve_limit = 0., MAX_CRUISE_KMH
    previous = lane
    step_m = 4
    for distance in range(step_m, 57, step_m):
        point = ahead(lane, distance)
        if point is None or point.is_junction:
            break
        turn = (point.transform.rotation.yaw - previous.transform.rotation.yaw + 180.) % 360. - 180.
        curvature = abs(radians(turn)) / step_m
        curve_limit = min(curve_limit, curve_approach_speed_kmh(curvature, distance - step_m))
        right = point.transform.get_right_vector()
        offset = min(1.15, point.lane_width * .33)
        probes = [relative(pose, point.transform.location + carla.Location(
            x=lateral * right.x, y=lateral * right.y, z=lateral * right.z))
            for lateral in (-offset, 0., offset)]
        if detector.road_visible(road, probes):
            visible = distance
        previous = point
    return visible, curve_limit


def lidar_lane_blocker(lane: carla.Waypoint, scan: LidarScan | None, frame_id: int,
                       forward_limit: float, rear_limit: float) -> str | None:
    if scan is None or scan.frame != frame_id:
        return "lidar_unavailable"
    points = scan.points[np.isfinite(scan.points).all(axis=1)]
    if len(points) < 100:
        return "lidar_unavailable"

    # Transform the measured cloud into the target lane's local frame.
    transform = np.asarray(lane.transform.get_inverse_matrix()) @ np.asarray(scan.transform.get_matrix())
    x, y, z = (points @ transform[:3, :3].T + transform[:3, 3]).T
    in_range = (x >= -rear_limit) & (x <= forward_limit)
    x, y, z = x[in_range], y[in_range], z[in_range]

    # Sample the corridor center and elevation to handle connected road sections.
    samples = [(0., 0., 0.)]
    for reverse, limit in ((True, rear_limit), (False, forward_limit)):
        for distance in range(6, int(limit) + 7, 6):
            options = lane.previous(distance) if reverse else lane.next(distance)
            if len(options) != 1 or not same_direction(lane, options[0]):
                break
            samples.append(relative(lane.transform, options[0].transform.location))
    xs, ys, zs = np.asarray(sorted(samples)).T
    lateral = y - np.interp(x, xs, ys)
    height = z - np.interp(x, xs, zs)
    supported = (x >= xs[0] - 1.) & (x <= xs[-1] + 1.)
    occupied = supported & (abs(lateral) <= lane.lane_width / 2 + .15) & (height >= .30) & (height <= 3.)
    return "lidar_occupied" if np.count_nonzero(occupied) >= 3 else None


def lane_blocker(lane: carla.Waypoint, pose: carla.Transform, ego_speed_kmh: float,
                 scan: LidarScan | None, frame_id: int, obstacles: list[Obstacle],
                 road: np.ndarray, detector: ObstacleDetector) -> str | None:
    if lane.is_junction:
        return "junction"
    forward_limit = max(24., ego_speed_kmh / 3.6 * 4.)
    half_width = lane.lane_width * .5
    # Bound closure over a four-second maneuver with rear traffic at 50 km/h.
    maneuver_kmh = min(ego_speed_kmh, 10.)
    rear_limit = 5. + (50. - maneuver_kmh) / 3.6 * 4.
    blocker = lidar_lane_blocker(lane, scan, frame_id, forward_limit, rear_limit)
    if blocker:
        return blocker

    probes = []
    for distance in (9., 13., 17., 21.):
        point = ahead(lane, distance)
        if point is None or point.is_junction:
            return "lane_ends"
        right = point.transform.get_right_vector()
        for offset in (-.8, 0., .8):
            probes.append(relative(pose, point.transform.location + carla.Location(
                x=offset * right.x, y=offset * right.y, z=offset * right.z)))
    if not detector.road_visible(road, probes):
        return "road_unobserved"
    for obstacle in obstacles:
        if obstacle.forward_m > forward_limit:
            break
        point = ahead(lane, max(0., obstacle.forward_m - relative(pose, lane.transform.location)[0]))
        if point:
            lateral = relative(pose, point.transform.location)[1]
            if obstacle.left_m < lateral + half_width and obstacle.right_m > lateral - half_width:
                return "semantic_obstacle"
    return None
