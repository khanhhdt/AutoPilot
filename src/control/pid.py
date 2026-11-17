"""Longitudinal PID: speed in km/h, dt in simulation seconds."""


class SpeedPID:
    def __init__(self, kp: float = 0.18, ki: float = 0.06, kd: float = 0.01,
                 max_throttle: float = 0.6, max_brake: float = 0.6) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_throttle, self.max_brake = max_throttle, max_brake
        self.reset()

    def reset(self) -> None:
        self._integral = 0.
        self._previous_speed = None
        self._derivative = 0.

    def step(self, target_kmh: float, speed_kmh: float, dt: float) -> tuple[float, float]:
        if dt <= 0:
            raise ValueError("PID dt must be positive")
        if target_kmh <= 0:
            self.reset()
            return 0., 1.
        error = target_kmh - speed_kmh
        derivative = 0. if self._previous_speed is None else -(speed_kmh - self._previous_speed) / dt
        self._derivative += 0.2 * (derivative - self._derivative)
        self._previous_speed = speed_kmh
        integral = self._integral + error * dt
        effort = self.kp * error + self.ki * integral + self.kd * self._derivative
        # Integrate only when unsaturated, or when error unwinds saturation.
        if (-self.max_brake <= effort <= self.max_throttle
                or effort > self.max_throttle and error < 0
                or effort < -self.max_brake and error > 0):
            self._integral = integral
        effort = self.kp * error + self.ki * self._integral + self.kd * self._derivative
        return min(self.max_throttle, max(0., effort)), min(self.max_brake, max(0., -effort))
