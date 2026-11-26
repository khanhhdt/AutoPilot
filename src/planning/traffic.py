"""Map geometry and simulator-side clearance for Phase 4 lane changes."""

from dataclasses import dataclass
from math import cos, radians

import carla
import numpy as np

from src.perception.obstacles import Obstacle, ObstacleDetector
from src.planning.decision import Lead


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


def nearest_lead(obstacles: list[Obstacle], waypoint: carla.Waypoint | None,
                 pose: carla.Transform, bumper_m: float) -> Lead | None:
    for obstacle in obstacles:
        center = ahead(waypoint, max(0., obstacle.forward_m - relative(pose, waypoint.transform.location)[0])) \
            if waypoint else None
        lateral = relative(pose, center.transform.location)[1] if center else 0.
        half_width = min(1.55, (waypoint.lane_width * .44) if waypoint else 1.35)
        if obstacle.left_m <= lateral + half_width and obstacle.right_m >= lateral - half_width:
            return Lead(obstacle.kind, max(0., obstacle.forward_m - bumper_m))
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


def lane_blocker(lane: carla.Waypoint, pose: carla.Transform, ego_speed_kmh: float,
                 actors: list[carla.Actor], obstacles: list[Obstacle], road: np.ndarray,
                 detector: ObstacleDetector) -> str | None:
    if lane.is_junction:
        return "junction"
    forward_limit = max(24., ego_speed_kmh / 3.6 * 4.)
    half_width = lane.lane_width * .5

    # An adjacent lane in the front-camera blind zone cannot be cleared by
    # pixels alone. Check collision-box overlap ahead, alongside and behind.
    for actor in actors:
        forward, lateral, _ = relative(lane.transform, actor.get_location())
        if abs(lateral) > 12. or not -65. <= forward <= forward_limit + 10.:
            continue
        box = getattr(actor, "bounding_box", None)
        if box is not None:
            points = [relative(lane.transform, p) for p in box.get_world_vertices(actor.get_transform())]
            xs, ys, _ = zip(*points)
        else:
            xs, ys = (forward - 1., forward + 1.), (lateral - 1., lateral + 1.)
        velocity = actor.get_velocity()
        axis = lane.transform.get_forward_vector()
        closing = max(0., velocity.x * axis.x + velocity.y * axis.y - ego_speed_kmh / 3.6)
        rear_buffer = max(10., 5. + 3. * closing)
        if min(xs) <= forward_limit and max(xs) >= -rear_buffer and min(ys) <= half_width and max(ys) >= -half_width:
            return "actor"

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
