"""Join sensor observations, the decision FSM and lane-change geometry."""

from dataclasses import dataclass

import carla

from src.perception.obstacles import Obstacle, ObstacleDetector
from src.perception.segmentation import masks
from src.perception.tracking import ObstacleTracker
from src.planning.decision import (
    PASS_ENTRY_SPEED_KMH, PASS_PREPARE_RANGE_M, PASS_PREPARE_SPEED_KMH, Decision, DecisionPlanner, Lead, State,
)
from src.planning.speed import curve_speed_kmh, preparation_range, visible_speed_kmh
from src.planning.traffic import LaneChange, adjacent_lanes, ahead, lane_blocker, lane_key, nearest_lead, relative, road_horizon


@dataclass
class Assessment:
    decision: Decision
    obstacles: list[Obstacle]
    lead: Lead | None
    clear: dict[str, bool]
    blockers: dict[str, str | None]
    target: carla.Waypoint | None
    target_blocker: str | None
    completed: bool
    speed_limit_kmh: float | None
    visible_m: float | None
    reference: str


class AvoidanceController:
    def __init__(self, fov: float, bumper_m: float, *, adaptive: bool = False) -> None:
        self.detector = ObstacleDetector(fov, max_range_m=65. if adaptive else 45.)
        self.adaptive = adaptive
        self.tracker = ObstacleTracker()
        self.trusted_lane = None
        self.last_fresh_s = None
        self.planner = DecisionPlanner()
        self.bumper_m = bumper_m
        self.change: LaneChange | None = None
        self.target_centered = False
        self.centered_fresh_frames = 0

    def step(self, frame, pose: carla.Transform,
             waypoint: carla.Waypoint | None, lane_fresh: bool, lane_valid: bool,
             cruise_kmh: float, dt: float) -> Assessment:
        semantic = masks(frame.seg_tags)
        obstacles = self.detector.detect(semantic, frame.depth)
        tracked = self.tracker.update(obstacles, frame.timestamp, (pose.location.x, pose.location.y),
                                      pose.rotation.yaw, frame.speed_kmh / 3.6)
        lead = nearest_lead(tracked, waypoint, pose, self.bumper_m)
        active = self.change
        completed = False
        target_lane = active.waypoint(waypoint) if active else None
        target = ahead(target_lane, 7.) if target_lane else None
        candidates = {}
        candidate_range = PASS_PREPARE_RANGE_M if cruise_kmh > PASS_ENTRY_SPEED_KMH else 26.
        if self.adaptive:
            candidate_range = preparation_range(max(cruise_kmh, frame.speed_kmh), PASS_PREPARE_SPEED_KMH, None)
        if not active and waypoint and lane_fresh and lead and lead.kind != "pedestrian" and lead.gap_m <= candidate_range:
            candidates = adjacent_lanes(waypoint)

        road = semantic.road | semantic.road_line
        limit = visible = None
        reference = "camera"
        if self.adaptive:
            reference_lane = target_lane or waypoint
            visible, curvature = road_horizon(reference_lane, pose, road, self.detector) if reference_lane else (0., 0.)
            # Reserve a few meters at the edge of the verified road projection.
            limit = min(curve_speed_kmh(curvature), visible_speed_kmh(max(0., visible - 4.)))
            if waypoint and lane_fresh and lane_valid:
                self.trusted_lane = lane_key(waypoint)
                self.last_fresh_s = frame.timestamp
            if not active and not lane_valid and waypoint and self.last_fresh_s is not None:
                yaw_error = (pose.rotation.yaw - waypoint.transform.rotation.yaw + 180.) % 360. - 180.
                if (lane_key(waypoint) == self.trusted_lane and not waypoint.is_junction
                        and frame.timestamp - self.last_fresh_s <= .85 and visible >= 16.
                        and abs(yaw_error) <= 8. and abs(relative(waypoint.transform, pose.location)[1]) <= .6):
                    target = ahead(waypoint, 8.)
                    if target is not None:
                        lane_valid = True
                        limit = min(limit, 10.)
                        reference = "map_fallback"
        blockers = {side: lane_blocker(lane, pose, frame.speed_kmh, frame.lidar, frame.frame,
                                       obstacles, road, self.detector)
                    for side, lane in candidates.items()}
        clear = {side: reason is None for side, reason in blockers.items()}
        blocker = None
        if active:
            blocker = (lane_blocker(target_lane, pose, frame.speed_kmh, frame.lidar, frame.frame,
                                    obstacles, road, self.detector)
                       if target else "target_unavailable")
        if active and not blocker and active.centered(target_lane, pose):
            completed = not self.target_centered
            self.target_centered = True
            self.centered_fresh_frames = self.centered_fresh_frames + 1 if lane_fresh else 0
            # Keep a stable lateral reference until the ego has passed the lead
            # and the camera has consistently reacquired the destination lane.
            if (active.start.distance(pose.location) >= active.minimum_travel_m
                    and self.centered_fresh_frames >= 4):
                self.change = active = None
                target = None
        else:
            self.centered_fresh_frames = 0
        decision = self.planner.step(lead, frame.speed_kmh, cruise_kmh, clear,
                                     lane_valid=lane_valid, dt=dt, adjacent_available=bool(candidates),
                                     adaptive=self.adaptive, speed_limit_kmh=limit,
                                     active_side=active.side if active else None,
                                     active_clear=blocker is None)
        if decision.start_side:
            target_lane = candidates[decision.start_side]
            # The observed gap reaches the rear of the obstacle. Allow another
            # 9 m for the obstacle body and clearance before leaving the target.
            minimum_travel = lead.gap_m + self.bumper_m + 9.
            self.change = LaneChange(decision.start_side, lane_key(target_lane), pose.location, minimum_travel)
            self.target_centered = False
            target = ahead(target_lane, 7.)
        if active or decision.start_side:
            reference = "target_lane"
        return Assessment(decision, obstacles, lead, clear, blockers, target, blocker, completed, limit, visible,
                          reference)

    def sensor_lost(self) -> None:
        self.planner.clear_frames = {"left": 0, "right": 0}
        self.tracker.reset()
        self.trusted_lane = self.last_fresh_s = None
