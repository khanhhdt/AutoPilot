"""Speed bounds and short-horizon prediction for the CARLA planner."""

from collections import deque
from math import sqrt
from statistics import median


REACTION_S = .6
BRAKE_MPS2 = 3.
COMFORT_BRAKE_MPS2 = 2.5
STANDSTILL_GAP_M = 4.
MAX_CRUISE_KMH = 40.
LATERAL_ACCEL_LIMIT_MPS2 = 1.5


def stopping_gap(speed_kmh: float, front_mps: float = 0.) -> float:
    speed = max(0., speed_kmh / 3.6)
    # Reserve ego response and stopping distance. A tracked moving lead can
    # contribute only its shorter stopping distance under stronger braking.
    return STANDSTILL_GAP_M + max(0., speed * REACTION_S
                                  + speed ** 2 / (2 * BRAKE_MPS2) - max(0., front_mps) ** 2 / 12.)


def visible_speed_kmh(distance_m: float) -> float:
    available = max(0., distance_m - STANDSTILL_GAP_M)
    offset = BRAKE_MPS2 * REACTION_S
    return 3.6 * (sqrt(offset ** 2 + 2 * BRAKE_MPS2 * available) - offset)


def curve_speed_kmh(curvature: float) -> float:
    return min(MAX_CRUISE_KMH, 3.6 * sqrt(LATERAL_ACCEL_LIMIT_MPS2 / curvature)) if curvature > 0. else MAX_CRUISE_KMH


def preparation_range(speed_kmh: float, prepare_kmh: float, closing_mps: float | None) -> float:
    speed = max(0., speed_kmh / 3.6)
    target = prepare_kmh / 3.6
    closing = speed if closing_mps is None else min(speed, max(0., closing_mps))
    front = speed - closing
    braking_time = max(0., speed - target) / COMFORT_BRAKE_MPS2
    braking_distance = max(0., (speed + target) / 2 - front) * braking_time
    return max(30., 24. + REACTION_S * closing + braking_distance)


class LeadHistory:
    """Estimate closing speed for a short, continuously associated detection."""

    def __init__(self) -> None:
        self.samples = deque(maxlen=7)
        self.key = None

    def reset(self) -> None:
        self.samples.clear()
        self.key = None

    def observe(self, key: tuple, gap_m: float, timestamp: float) -> float | None:
        if self.key != key or (self.samples and (
            not 0. < timestamp - self.samples[-1][0] <= .12
            or abs(gap_m - self.samples[-1][1]) > 2.
        )):
            self.reset()
        self.key = key
        self.samples.append((timestamp, gap_m))
        if len(self.samples) < 5:
            return None
        samples = list(self.samples)
        return median((a[1] - b[1]) / (b[0] - a[0]) for a, b in zip(samples, samples[3:]))


def swept_overlap(bounds: tuple, velocity: tuple, region: tuple, duration_s: float) -> bool:
    """Continuous intersection of a translating rectangle and a fixed region."""
    first, last = 0., duration_s
    for (low, high), rate, (near, far) in zip(bounds, velocity, region):
        if abs(rate) < 1e-6:
            if low > far or high < near:
                return False
            continue
        entry, exit = sorted(((near - high) / rate, (far - low) / rate))
        first, last = max(first, entry), min(last, exit)
    return first <= last
