"""Controlled traffic events for the Town10HD_Opt Phase 4 trials."""

import random

import carla

from src.planning.traffic import ahead, lane_key, relative


NAMES = ("static", "cut-in", "pedestrian", "avoid", "avoid-left")
SPAWNS = dict.fromkeys(NAMES, 151) | {"avoid": 22, "avoid-left": 152}


class Scenario:
    def __init__(self, name: str, seed: int, world: carla.World, ego: carla.Vehicle) -> None:
        self.name = name
        self.seed = seed
        self.world = world
        self.map = world.get_map()
        self.ego = ego
        self.origin = self.map.get_waypoint(ego.get_location())
        self.actors = []
        self.actor = None
        self.traffic_manager = None
        self.triggered = False
        self.crossed = False
        self.approached = False
        self.responded = False
        self.stopped_frames = 0
        self.passed = False
        self.left_corridor = False
        self.keep_going = True
        self.next_cut_in_request_s = 0.
        self.min_gap_m = None
        self.front_m = ego.bounding_box.location.x + ego.bounding_box.extent.x

    def setup(self, client: carla.Client) -> None:
        rng = random.Random(self.seed)
        right = self.name != "avoid-left"
        adjacent = self.origin.get_right_lane() if right else self.origin.get_left_lane()
        if not adjacent or adjacent.lane_type != carla.LaneType.Driving:
            raise RuntimeError("scenario needs the designated adjacent driving lane")
        distance = rng.uniform(15., 20.) if self.name == "static" else rng.uniform(18.5, 20.5)
        if self.name == "pedestrian":
            crossing = self._point(self.origin, rng.uniform(14.5, 16.))
            side = crossing.get_right_vector()
            location = crossing.location + carla.Location(x=2.7 * side.x, y=2.7 * side.y, z=.35)
            bp = self.world.get_blueprint_library().find("walker.pedestrian.0001")
            self.actor = self._spawn(bp, carla.Transform(location))
            self.walk_direction = carla.Vector3D(-side.x, -side.y, 0.)
            self.triggered = False
        elif self.name == "cut-in":
            self.actor = self._park(adjacent, rng.uniform(14.5, 16.5))
            self.cut_in_lane = lane_key(adjacent)
            self.traffic_manager = client.get_trafficmanager()
            tm = self.traffic_manager
            tm.set_synchronous_mode(True)
            tm.set_random_device_seed(self.seed)
            self.actor.set_autopilot(True, tm.get_port())
            tm.auto_lane_change(self.actor, False)
            tm.set_desired_speed(self.actor, 5.)
        elif self.name == "static":
            barrier = self.world.get_blueprint_library().find("static.prop.streetbarrier")
            transform = self._point(self.origin, distance)
            transform.location.z += .04
            self.actor = self._spawn(barrier, transform)
            self._park(adjacent, 6. if self.seed % 2 == 0 else -5.)
        else:
            self.actor = self._park(self.origin, distance)
            self.target_lane = lane_key(adjacent)

    def _point(self, lane: carla.Waypoint, distance: float) -> carla.Transform:
        if distance < 0.:
            options = lane.previous(-distance)
            point = options[0] if len(options) == 1 else None
        else:
            point = ahead(lane, distance)
        if point is None or point.is_junction:
            raise RuntimeError("scenario waypoint must be on a continuous non-junction lane")
        return point.transform

    def _spawn(self, blueprint, transform: carla.Transform):
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "phase4_scenario")
        actor = self.world.spawn_actor(blueprint, transform)
        self.actors.append(actor)
        return actor

    def _park(self, lane: carla.Waypoint, distance: float) -> carla.Vehicle:
        bp = self.world.get_blueprint_library().find("vehicle.tesla.model3")
        transform = self._point(lane, distance)
        transform.location.z += .25
        actor = self._spawn(bp, transform)
        actor.apply_control(carla.VehicleControl(hand_brake=True))
        return actor

    def step(self, row: dict) -> bool:
        pose = self.ego.get_transform()
        position = self.actor.get_location()
        longitudinal, lateral, _ = relative(self.origin.transform, position)
        traveled = relative(self.origin.transform, pose.location)[0]
        forward, _, _ = relative(pose, position)
        obstacle_extent = getattr(self.actor, "bounding_box", None)
        rear_m = obstacle_extent.extent.x if obstacle_extent else .65
        gap = forward - rear_m - self.front_m
        row["scenario_gap_m"] = gap
        if gap >= 0.:
            self.min_gap_m = gap if self.min_gap_m is None else min(self.min_gap_m, gap)
        self.approached |= traveled >= 4.

        if self.name == "cut-in":
            lane = self.map.get_waypoint(position)
            actor_lane = lane_key(lane) if lane else None
            self.crossed |= self.triggered and actor_lane == lane_key(self.origin)
            # TM can discard a force request while initializing its lane buffer.
            # Retry in the starting lane only, and let any ongoing merge finish.
            if (not self.crossed and actor_lane == self.cut_in_lane and traveled >= 1.
                    and lateral >= self.origin.lane_width * .85
                    and row["sim_elapsed_s"] >= self.next_cut_in_request_s):
                self.traffic_manager.force_lane_change(self.actor, False)
                self.triggered = True
                self.next_cut_in_request_s = row["sim_elapsed_s"] + 1.
            self.responded |= self.triggered and row["phase4_lead_kind"] == "vehicle" and row["phase4_state"] in (
                "FOLLOW", "STOP", "AVOID_L", "AVOID_R")
        elif self.name == "pedestrian":
            if not self.triggered and row["sim_elapsed_s"] >= 1.:
                self.actor.apply_control(carla.WalkerControl(direction=self.walk_direction, speed=1.25))
                self.triggered = True
            if self.triggered and lateral < -2.8 and self.keep_going:
                self.actor.apply_control(carla.WalkerControl())
                self.keep_going = False
            self.crossed |= self.triggered and lateral < -.8
            self.responded |= self.triggered and row["phase4_lead_kind"] == "pedestrian" and row["phase4_state"] == "STOP"
        elif self.name == "static":
            stopped = (self.approached and .5 < gap <= 9. and row["phase4_state"] == "STOP"
                       and row["phase4_lead_kind"] == "prop"
                       and row["speed_kmh"] < .2)
            self.stopped_frames = self.stopped_frames + 1 if stopped else 0
            if self.stopped_frames >= 20:
                return True
        else:
            current = self.map.get_waypoint(pose.location, project_to_road=False)
            current_lane = lane_key(current) if current else None
            self.left_corridor |= current_lane not in (lane_key(self.origin), self.target_lane)
            self.passed |= current_lane == self.target_lane and traveled >= longitudinal + 5.5
            if self.passed:
                return True
        return False

    def result(self) -> dict:
        return {"name": self.name, "seed": self.seed, "triggered": self.triggered, "crossed": self.crossed,
                "approached": self.approached, "responded": self.responded,
                "stopped_before_obstacle": self.stopped_frames >= 20,
                "passed_obstacle": self.passed, "left_corridor": self.left_corridor,
                "min_positive_gap_m": self.min_gap_m}

    def close(self) -> None:
        errors = []
        if self.traffic_manager and self.actor and self.actor.is_alive:
            try:
                self.actor.set_autopilot(False, self.traffic_manager.get_port())
            except RuntimeError as exc:
                errors.append(str(exc))
        for actor in reversed(self.actors):
            try:
                actor.destroy()
            except RuntimeError as exc:
                errors.append(str(exc))
        if self.traffic_manager:
            try:
                self.traffic_manager.set_synchronous_mode(False)
            except RuntimeError as exc:
                errors.append(str(exc))
        if errors:
            raise RuntimeError(" | ".join(errors))
