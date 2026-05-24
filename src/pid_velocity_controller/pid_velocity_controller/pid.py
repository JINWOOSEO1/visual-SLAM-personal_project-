class PID:
    """
    Discrete PID controller with anti-windup and derivative low-pass filter.

    Control law (velocity form):
        error_k  = setpoint - measurement
        P        = Kp * error_k
        I       += Ki * error_k * dt          (clamped to [-integral_limit, integral_limit])
        D_raw    = Kd * (error_k - error_k-1) / dt
        D        = alpha * D_prev + (1 - alpha) * D_raw  (low-pass, alpha = tau/(tau+dt))
        output   = clamp(P + I + D, output_min, output_max)

    Resetting:
        Call reset() when setpoint changes sign (direction reversal) to clear
        accumulated integral and derivative state.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float = 50.0,
        output_min: float = 0.0,
        output_max: float = 100.0,
        derivative_tau: float = 0.05,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_min = output_min
        self.output_max = output_max
        self.derivative_tau = derivative_tau  # low-pass time constant [s]

        self._integral   = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0

    # ------------------------------------------------------------------
    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID output for the current timestep.

        Args:
            error: setpoint - measurement  (signed)
            dt:    elapsed time since last call [s]  (must be > 0)

        Returns:
            output clamped to [output_min, output_max]
        """
        if dt <= 0.0:
            return self.output_min

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += self.ki * error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))

        # Derivative with low-pass filter (avoids amplifying high-freq noise)
        d_raw = self.kd * (error - self._prev_error) / dt
        alpha = self.derivative_tau / (self.derivative_tau + dt)
        self._d_filtered = alpha * self._d_filtered + (1.0 - alpha) * d_raw

        self._prev_error = error

        output = p_term + self._integral + self._d_filtered
        return max(self.output_min, min(self.output_max, output))

    # ------------------------------------------------------------------
    def reset(self):
        """Clear integral and derivative state (call on direction reversal)."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._d_filtered = 0.0
