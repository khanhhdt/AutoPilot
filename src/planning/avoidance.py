"""Join sensor observations, the decision FSM and lane-change geometry."""

from dataclasses import dataclass

import carla

from src.perception.obstacles import Obstacle, ObstacleDetector
from src.perception.segmentation import masks
from src.planning.decision import Decision, DecisionPlanner, Lead, State
from src.planning.traffic import LaneChange, adjacent_lanes, ahead, lane_blocker, lane_key, nearest_lead


@dataclass
class Assessment:
    decision: Decision
    obstacles: list[Obstacle]
    lead: Lead | None
    clear: dict[str, bool]
    target: carla.Waypoint | None
    target_blocker: str | None
    completed: bool


class AvoidanceController:
    def __init__(self, fov: float, bumper_m: float) -> None:
        self.detector = ObstacleDetector(fov)
        self.planner = DecisionPlanner()
        self.bumper_m = bumper_m
        self.change: LaneChange | None = None
        self.target_centered = False
        self.centered_fresh_frames = 0

    def step(self, frame, world: carla.World, ego: carla.Vehicle, pose: carla.Transform,
             waypoint: carla.Waypoint | None, lane_fresh: bool, lane_valid: bool,
             cruise_kmh: float, dt: float) -> Assessment:
        semantic = masks(frame.seg_tags)
        obstacles = self.detector.detect(semantic, frame.depth)
        lead = nearest_lead(obstacles, waypoint, pose, self.bumper_m)
        active = self.change
        completed = False
        target_lane = active.waypoint(waypoint) if active else None
        target = ahead(target_lane, 7.) if target_lane else None
        candidates = {}
        if not active and waypoint and lane_fresh and lead and lead.kind != "pedestrian" and lead.gap_m <= 26.:
            candidates = adjacent_lanes(waypoint)

        road = semantic.road | semantic.road_line
        actors = []
        if candidates or target_lane:
            actors = [actor for actor in world.get_actors() if actor.id != ego.id and
                      actor.type_id.startswith(("vehicle.", "walker.pedestrian.", "static.prop."))]
        clear = {side: lane_blocker(lane, pose, frame.speed_kmh, actors, obstacles, road, self.detector) is None
                 for side, lane in candidates.items()}
        blocker = None
        if active:
            blocker = (lane_blocker(target_lane, pose, frame.speed_kmh, actors, obstacles, road, self.detector)
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
                                     lane_valid=lane_valid, dt=dt, active_side=active.side if active else None,
                                     active_clear=blocker is None)
        if decision.start_side:
            target_lane = candidates[decision.start_side]
            # The observed gap reaches the rear of the obstacle. Allow another
            # 9 m for the obstacle body and clearance before leaving the target.
            minimum_travel = lead.gap_m + self.bumper_m + 9.
            self.change = LaneChange(decision.start_side, lane_key(target_lane), pose.location, minimum_travel)
            self.target_centered = False
            target = ahead(target_lane, 7.)
        return Assessment(decision, obstacles, lead, clear, target, blocker, completed)

    def sensor_lost(self) -> None:
        self.planner.clear_frames = {"left": 0, "right": 0}
