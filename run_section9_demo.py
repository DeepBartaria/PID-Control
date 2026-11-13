"""
Standalone Section 9 demonstration: runs the reusable PIDController
(with clamping, anti-windup, derivative-on-measurement, and D-filter)
on the 1 kg mass-spring-damper plant under realistic limits and noise.

Usage:
    python3 run_section9_demo.py
"""

import numpy as np
from pid_controller import PIDController
from systems import MassSpringDamper, simulate_closed_loop, compute_step_metrics


def main() -> None:
    plant = MassSpringDamper(mass=1.0, damping=10.0, stiffness=20.0)
    controller = PIDController(
        kp=120.0,
        ki=240.0,
        kd=12.0,
        u_min=-30.0,                 # +/- 30 N motor limit
        u_max=30.0,
        anti_windup=True,            # conditional integration clamping
        derivative_on_measurement=True,  # avoids setpoint kick
        d_filter_tau=0.025,          # 25 ms low-pass filter on D
    )

    step_target = lambda t: 1.0 if t >= 0.3 else 0.0
    sim = simulate_closed_loop(
        plant=plant,
        controller=controller,
        setpoint_fn=step_target,
        t_end=3.0,
        dt=0.005,                    # 200 Hz control loop
        substeps=10,
        u_min=-30.0,
        u_max=30.0,
        noise_std=0.0004,
        noise_clip=0.001,            # +/- 1 mm sensor noise
        seed=42,
    )

    mask = sim["t"] >= 0.3
    metrics = compute_step_metrics(sim["t"][mask] - 0.3, sim["y"][mask], setpoint=1.0)
    steady_force_p2p = float(np.ptp(sim["u"][sim["t"] >= 1.8]))

    print("=== Reusable PIDController Run (Test System) ===")
    print(f"Gains          : Kp={controller.kp}, Ki={controller.ki}, Kd={controller.kd}")
    print(f"Limits / Filter: u in [{controller.u_min}, {controller.u_max}] N, tau_d={controller.d_filter_tau} s")
    print(f"Rise time      : {metrics['rise_time']:.3f} s")
    print(f"Overshoot      : {metrics['overshoot_pct']:.2f} %")
    print(f"Settling time  : {metrics['settling_time']:.3f} s")
    print(f"Steady error   : {metrics['ess']:.4f} m")
    print(f"Peak force     : {np.max(sim['u']):.2f} N")
    print(f"Steady u ripple: {steady_force_p2p:.2f} N (peak-to-peak)")


if __name__ == "__main__":
    main()

