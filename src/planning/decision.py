"""Longitudinal control and lane-change admission for the CARLA simulation."""

from dataclasses import dataclass
from enum import Enum


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
        self.last_lead: Lead | None = None

    def step(self, lead: Lead | None, speed_kmh: float, cruise_kmh: float,
             clear: dict[str, bool], *, lane_valid: bool, dt: float,
             active_side: str | None = None, active_clear: bool = False) -> Decision:
        for side in self.clear_frames:
            self.clear_frames[side] = self.clear_frames[side] + 1 if clear.get(side) else 0

        speed = max(0., speed_kmh / 3.6)
        stop_gap = 4. + .6 * speed + speed ** 2 / 6.
        if active_side:
            if not active_clear or (lead and lead.gap_m <= stop_gap):
                return self._set(State.STOP, 0.)
            state = State.AVOID_L if active_side == "left" else State.AVOID_R
            return self._set(state, min(cruise_kmh, 10.))
        if not lane_valid:
            return self._set(State.STOP, 0.)
        observed = lead is not None
        if lead is None:
            self.missing_lead_frames += 1
            if self.stopped_for_lead and self.missing_lead_frames * dt <= .5:
                return self._set(State.STOP, 0.)
            if self.last_lead and self.last_lead.gap_m < 12. and self.missing_lead_frames * dt <= .25:
                lead = Lead(self.last_lead.kind, max(0., self.last_lead.gap_m - speed * dt))
            else:
                self.stopped_for_lead = False
                self.last_lead = None
                return self._set(State.CRUISE, cruise_kmh)
        else:
            self.missing_lead_frames = 0
        self.last_lead = lead

        if observed and lead.kind != "pedestrian" and speed_kmh <= 15. and max(10., stop_gap + 4.) <= lead.gap_m <= 24.:
            for side, frames in self.clear_frames.items():
                if frames >= 3:
                    state = State.AVOID_L if side == "left" else State.AVOID_R
                    self.stopped_for_lead = False
                    return self._set(state, min(cruise_kmh, 10.), side)

        hold_gap = 7. if self.stopped_for_lead else 0.
        object_gap = {"prop": 6., "pedestrian": 10.}.get(lead.kind, 0.)
        if lead.gap_m <= max(stop_gap, hold_gap, object_gap):
            self.stopped_for_lead = True
            return self._set(State.STOP, 0.)
        self.stopped_for_lead = False
        follow_kmh = (lead.gap_m - 4.) * 1.55
        if follow_kmh < cruise_kmh:
            return self._set(State.FOLLOW, follow_kmh)
        return self._set(State.CRUISE, cruise_kmh)

    def _set(self, state: State, speed_kmh: float, start_side: str | None = None) -> Decision:
        self.state = state
        return Decision(state, speed_kmh, start_side)
