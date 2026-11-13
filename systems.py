"""
Plant models, pure-NumPy RK4 numerical simulator, and step-response metrics.
Uses only NumPy (no scipy or external control libraries).
"""

import numpy as np


class MassSpringDamper:
    """
    1D mass-spring-damper plant:
        m * x'' + c * x' + k * x = u
    Default parameters:
        m = 1.0 kg, c = 10.0 N*s/m, k = 20.0 N/m
    State vector: [position x (m), velocity v (m/s)]
    """

    def __init__(self, mass: float = 1.0, damping: float = 10.0, stiffness: float = 20.0):
        self.m = float(mass)
        self.c = float(damping)
        self.k = float(stiffness)

    def derivatives(self, state: np.ndarray, u: float) -> np.ndarray:
        x, v = state
        a = (u - self.c * v - self.k * x) / self.m
        return np.array([v, a], dtype=float)


class ThirdOrderPlant:
    """
    Third-order plant with transfer function G(s) = 1 / (s + 1)^3:
        y''' + 3 y'' + 3 y' + y = u
    State vector: [y, y', y'']
    """

    def derivatives(self, state: np.ndarray, u: float) -> np.ndarray:
        y, y1, y2 = state
        y3 = u - 3.0 * y2 - 3.0 * y1 - y
        return np.array([y1, y2, y3], dtype=float)


def rk4_step(plant, state: np.ndarray, u: float, dt: float) -> np.ndarray:
    """Advance continuous plant dynamics by dt using 4th-order Runge-Kutta."""
    k1 = plant.derivatives(state, u)
    k2 = plant.derivatives(state + 0.5 * dt * k1, u)
    k3 = plant.derivatives(state + 0.5 * dt * k2, u)
    k4 = plant.derivatives(state + dt * k3, u)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def simulate_closed_loop(
    plant,
    controller,
    setpoint_fn,
    t_end: float,
    dt: float = 0.001,
    substeps: int = 10,
    x0=None,
    u_min: float = -float("inf"),
    u_max: float = float("inf"),
    noise_std: float = 0.0,
    noise_clip: float = float("inf"),
    seed: int = 42,
):
    """
    Simulate a discrete-time controller driving a continuous-time plant.

    The controller samples the plant output and updates its force command u every `dt`
    seconds (Zero-Order Hold). Between controller updates, the plant advances via
    `substeps` RK4 steps of size dt / substeps.
    """
    if controller is not None:
        controller.reset()

    n_steps = int(round(t_end / dt))
    t = np.linspace(0.0, n_steps * dt, n_steps + 1)

    if x0 is None:
        if isinstance(plant, ThirdOrderPlant):
            state = np.zeros(3, dtype=float)
        else:
            state = np.zeros(2, dtype=float)
    else:
        state = np.array(x0, dtype=float)

    rng = np.random.default_rng(seed)

    y_true = np.zeros(n_steps + 1, dtype=float)
    y_meas = np.zeros(n_steps + 1, dtype=float)
    r_arr = np.zeros(n_steps + 1, dtype=float)
    u_arr = np.zeros(n_steps + 1, dtype=float)
    p_arr = np.zeros(n_steps + 1, dtype=float)
    i_arr = np.zeros(n_steps + 1, dtype=float)
    d_arr = np.zeros(n_steps + 1, dtype=float)

    sub_dt = dt / substeps

    for idx in range(n_steps + 1):
        tk = t[idx]
        rk = float(setpoint_fn(tk)) if callable(setpoint_fn) else float(setpoint_fn)
        yk = float(state[0])

        if noise_std > 0.0:
            noise = float(np.clip(rng.normal(0.0, noise_std), -noise_clip, noise_clip))
        else:
            noise = 0.0
        yk_meas = yk + noise

        if controller is None:
            uk = rk  # open-loop mode: setpoint_fn provides input force directly
            p_term, i_term, d_term = 0.0, 0.0, 0.0
        else:
            uk = controller.update(rk, yk_meas, dt)
            p_term, i_term, d_term = controller.last_terms

        uk_applied = float(np.clip(uk, u_min, u_max))

        y_true[idx] = yk
        y_meas[idx] = yk_meas
        r_arr[idx] = rk
        u_arr[idx] = uk_applied
        p_arr[idx] = p_term
        i_arr[idx] = i_term
        d_arr[idx] = d_term

        if idx < n_steps:
            for _ in range(substeps):
                state = rk4_step(plant, state, uk_applied, sub_dt)

    return {
        "t": t,
        "y": y_true,
        "y_meas": y_meas,
        "r": r_arr,
        "u": u_arr,
        "p": p_arr,
        "i": i_arr,
        "d": d_arr,
    }


def _first_crossing_time(t: np.ndarray, y: np.ndarray, target: float) -> float:
    """Linearly interpolate the first time y crosses upward through target."""
    for idx in range(1, len(y)):
        if y[idx - 1] < target <= y[idx]:
            frac = (target - y[idx - 1]) / (y[idx] - y[idx - 1])
            return float(t[idx - 1] + frac * (t[idx] - t[idx - 1]))
    return float("nan")


def compute_step_metrics(t: np.ndarray, y: np.ndarray, setpoint: float = 1.0, band: float = 0.02) -> dict:
    """
    Compute standard step-response metrics for a step from 0 toward `setpoint`:
        - y_ss: final steady-state value (final sample)
        - ess: steady-state error |setpoint - y_ss|
        - rise_time: 10% to 90% of y_ss (s)
        - overshoot_pct: max(0, (peak - y_ss) / y_ss * 100) (%)
        - peak: maximum y value
        - peak_time: time of maximum y
        - settling_time: time after which |y - y_ss| <= band * |y_ss| forever (s)
    """
    y_ss = float(y[-1])
    ess = float(abs(setpoint - y_ss))
    peak_idx = int(np.argmax(y))
    peak = float(y[peak_idx])
    peak_time = float(t[peak_idx])

    if abs(y_ss) < 1e-9:
        rise_time = float("nan")
        overshoot_pct = 0.0
        settling_time = float("nan")
    else:
        t10 = _first_crossing_time(t, y, 0.1 * y_ss)
        t90 = _first_crossing_time(t, y, 0.9 * y_ss)
        rise_time = float(t90 - t10) if not (np.isnan(t10) or np.isnan(t90)) else float("nan")
        overshoot_pct = float(max(0.0, (peak - y_ss) / abs(y_ss) * 100.0))

        tol = band * abs(y_ss)
        outside = np.where(np.abs(y - y_ss) > tol)[0]
        if len(outside) == 0:
            settling_time = 0.0
        elif outside[-1] == len(t) - 1:
            settling_time = float("nan")
        else:
            settling_time = float(t[outside[-1] + 1])

    return {
        "y_ss": y_ss,
        "ess": ess,
        "rise_time": rise_time,
        "t10": _first_crossing_time(t, y, 0.1 * y_ss) if abs(y_ss) >= 1e-9 else float("nan"),
        "t90": _first_crossing_time(t, y, 0.9 * y_ss) if abs(y_ss) >= 1e-9 else float("nan"),
        "overshoot_pct": overshoot_pct,
        "peak": peak,
        "peak_time": peak_time,
        "settling_time": settling_time,
    }

