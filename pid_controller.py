"""
Self-contained discrete PID controller implementations using only the Python standard library.

Provides:
    - SimplePID: Textbook discrete PID (useful for illustrating baseline behavior and pitfalls).
    - PIDController: Production-ready PID with actuator clamping, anti-windup,
      derivative-on-measurement (no setpoint kick), and low-pass derivative filtering.
"""


class SimplePID:
    """Basic discrete PID controller without real-world safeguards."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None
        self.last_terms = (0.0, 0.0, 0.0)

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        error = setpoint - measurement
        p_term = self.kp * error

        self.integral += error * dt
        i_term = self.ki * self.integral

        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / dt
        d_term = self.kd * d_error
        self.prev_error = error

        self.last_terms = (p_term, i_term, d_term)
        return p_term + i_term + d_term


class PIDController:
    """
    Reusable discrete PID controller with practical real-world safeguards:
      1. Output limits (u_min, u_max) to match physical actuator bounds.
      2. Conditional-integration anti-windup to prevent integral buildup during saturation.
      3. Derivative-on-measurement (optional, enabled by default) to eliminate setpoint kick.
      4. First-order low-pass filter on the derivative term to suppress sensor noise.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        u_min: float = -float("inf"),
        u_max: float = float("inf"),
        anti_windup: bool = True,
        derivative_on_measurement: bool = True,
        d_filter_tau: float = 0.0,
    ):
        if u_min >= u_max:
            raise ValueError("u_min must be strictly less than u_max.")
        if d_filter_tau < 0.0:
            raise ValueError("d_filter_tau must be non-negative.")

        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.u_min = float(u_min)
        self.u_max = float(u_max)
        self.anti_windup = bool(anti_windup)
        self.derivative_on_measurement = bool(derivative_on_measurement)
        self.d_filter_tau = float(d_filter_tau)
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None
        self.prev_measurement = None
        self.d_filtered = 0.0
        self.last_terms = (0.0, 0.0, 0.0)
        self.last_raw_u = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        error = setpoint - measurement

        # P term: reacts to present error
        p_term = self.kp * error

        # D term: reacts to rate of change (without setpoint kick if derivative_on_measurement=True)
        if self.derivative_on_measurement:
            if self.prev_measurement is None:
                raw_derivative = 0.0
            else:
                raw_derivative = -(measurement - self.prev_measurement) / dt
        else:
            if self.prev_error is None:
                raw_derivative = 0.0
            else:
                raw_derivative = (error - self.prev_error) / dt

        # Low-pass filter on derivative: D_k = (tau * D_{k-1} + dt * raw_d) / (tau + dt)
        if self.d_filter_tau > 0.0:
            alpha = dt / (self.d_filter_tau + dt)
            self.d_filtered = (1.0 - alpha) * self.d_filtered + alpha * raw_derivative
        else:
            self.d_filtered = raw_derivative

        d_term = self.kd * self.d_filtered

        # Candidate I term using trapezoidal/Euler step
        candidate_integral = self.integral + error * dt
        candidate_i_term = self.ki * candidate_integral
        u_candidate = p_term + candidate_i_term + d_term

        # Anti-windup via conditional integration (clamping):
        # Only update the integrator if we are not saturated, OR if the current error
        # pushes the controller back out of saturation.
        if self.anti_windup and self.ki != 0.0:
            saturated_high = u_candidate > self.u_max and error > 0.0
            saturated_low = u_candidate < self.u_min and error < 0.0
            if not (saturated_high or saturated_low):
                self.integral = candidate_integral
        else:
            self.integral = candidate_integral

        i_term = self.ki * self.integral
        u_unclamped = p_term + i_term + d_term
        u_clamped = max(self.u_min, min(self.u_max, u_unclamped))

        self.prev_error = error
        self.prev_measurement = measurement
        self.last_raw_u = u_unclamped
        self.last_terms = (p_term, i_term, d_term)

        return u_clamped

