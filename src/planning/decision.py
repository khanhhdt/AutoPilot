"""Longitudinal control and lane-change admission for the CARLA simulation."""

from dataclasses import dataclass
from enum import Enum

from src.planning.speed import preparation_range, stopping_gap


PASS_ENTRY_SPEED_KMH = 15.
PASS_PREPARE_SPEED_KMH = 12.
PASS_PREPARE_RANGE_M = 30.


class State(str, Enum):
    CRUISE = "CRUISE"
    FOLLOW = "FOLLOW"
    AVOID_L = "AVOID_L"
    AVOID_R = "AVOID_R"
    STOP = "STOP"


@dataclass(frozen=True)
class Lead:
    kind: str
    gap_m: float
    center_m: float = 0.
    closing_mps: float | None = None


@dataclass(frozen=True)
class Decision:
    state: State
    speed_kmh: float
    start_side: str | None = None


class DecisionPlanner:
    def __init__(self) -> None:
        self.state = State.CRUISE
        self.clear_frames = {"left": 0, "right": 0}
        self.missing_lead_frames = 0
        self.stopped_for_lead = False
        self.preparing = False
        self.last_lead: Lead | None = None

    def step(self, lead: Lead | None, speed_kmh: float, cruise_kmh: float,
             clear: dict[str, bool], *, lane_valid: bool, dt: float,
             adjacent_available: bool = False,
             adaptive: bool = False, speed_limit_kmh: float | None = None,
             active_side: str | None = None, active_clear: bool = False) -> Decision:
        for side in self.clear_frames:
            self.clear_frames[side] = self.clear_frames[side] + 1 if clear.get(side) else 0

        speed = max(0., speed_kmh / 3.6)
        limited_cruise = min(cruise_kmh, speed_limit_kmh) if speed_limit_kmh is not None else cruise_kmh
        closing = lead.closing_mps if adaptive and lead and lead.kind == "vehicle" else None
        front = min(speed, max(0., speed - closing)) if closing is not None else 0.
        emergency_gap = stopping_gap(speed_kmh)
        follow_gap = stopping_gap(speed_kmh, front)
        if active_side:
            self.preparing = False
            if limited_cruise <= 0. or not active_clear or (lead and lead.gap_m <= emergency_gap):
                return self._set(State.STOP, 0.)
            state = State.AVOID_L if active_side == "left" else State.AVOID_R
            return self._set(state, min(limited_cruise, 10.))
        if not lane_valid or limited_cruise <= 0.:
            return self._set(State.STOP, 0.)
        observed = lead is not None
        if lead is None:
            self.missing_lead_frames += 1
            if self.stopped_for_lead and self.missing_lead_frames * dt <= .5:
                return self._set(State.STOP, 0.)
            if (self.last_lead and (self.last_lead.gap_m < 12. or self.preparing)
                    and self.missing_lead_frames * dt <= .25):
                lead = Lead(self.last_lead.kind, max(0., self.last_lead.gap_m - speed * dt))
            else:
                self.stopped_for_lead = False
                self.preparing = False
                self.last_lead = None
                return self._set(State.CRUISE, limited_cruise)
        else:
            self.missing_lead_frames = 0
        self.last_lead = lead

        passable_lead = observed and lead.kind != "pedestrian"
        entry_gap = max(10., emergency_gap + 4.)
        if passable_lead and speed_kmh <= PASS_ENTRY_SPEED_KMH and entry_gap <= lead.gap_m <= 24.:
            for side, frames in self.clear_frames.items():
                if frames >= 3:
                    state = State.AVOID_L if side == "left" else State.AVOID_R
                    self.stopped_for_lead = False
                    self.preparing = False
                    return self._set(state, min(limited_cruise, 10.), side)

        hold_gap = 7. if self.stopped_for_lead else 0.
        object_gap = {"prop": 6., "pedestrian": 10.}.get(lead.kind, 0.)
        if lead.gap_m <= max(emergency_gap, hold_gap, object_gap):
            self.stopped_for_lead = True
            return self._set(State.STOP, 0.)
        self.stopped_for_lead = False
        follow_kmh = (lead.gap_m - 4.) * 1.55
        if adaptive and closing is not None:
            follow_kmh = min(follow_kmh, max(0., (front + .5 * (lead.gap_m - follow_gap)) * 3.6))
        # Leave enough room for the PID to reach the actual entry speed.
        # An adjacent lane can be legal but occupied. The clearance checks
        # above still govern whether the maneuver may begin.
        prepare_gap = preparation_range(speed_kmh, PASS_PREPARE_SPEED_KMH, closing) if adaptive else PASS_PREPARE_RANGE_M
        if adaptive and observed:
            self.preparing = (lead.kind != "pedestrian" and adjacent_available
                              and (self.preparing or lead.gap_m <= prepare_gap))
        if ((adaptive and self.preparing) or
                (not adaptive and passable_lead and adjacent_available and cruise_kmh > PASS_ENTRY_SPEED_KMH
                 and lead.gap_m <= prepare_gap)):
            follow_kmh = min(follow_kmh, PASS_PREPARE_SPEED_KMH)
        if follow_kmh < limited_cruise:
            return self._set(State.FOLLOW, follow_kmh)
        return self._set(State.CRUISE, limited_cruise)

    def _set(self, state: State, speed_kmh: float, start_side: str | None = None) -> Decision:
        self.state = state
        return Decision(state, speed_kmh, start_side)
