class PIDController:
    """Controlador PID con anti-windup por saturación integral."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -float('inf'),
        output_max: float = float('inf'),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt < 1e-6:
            return 0.0

        self._integral += error * dt

        # Anti-windup: limita el acumulador integral al rango de salida
        if self.ki != 0.0:
            half_range = (self.output_max - self.output_min) / (2.0 * abs(self.ki))
            self._integral = max(-half_range, min(half_range, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
