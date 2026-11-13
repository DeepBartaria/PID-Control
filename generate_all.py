"""
Generate all figures and numerical tables for the PID Control Guide.

Usage:
    python3 generate_all.py

Creates figures/fig01_...png through figures/fig13_...png deterministically
using only NumPy and Matplotlib.
"""

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle, Polygon

from pid_controller import SimplePID, PIDController
from systems import (
    MassSpringDamper,
    ThirdOrderPlant,
    simulate_closed_loop,
    compute_step_metrics,
)

FIG_DIR = Path(__file__).resolve().parent / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Consistent visual styling
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.titlesize": 12,
    "axes.titleweight": "bold",
    "axes.labelsize": 10.5,
    "legend.fontsize": 9,
    "xtick.labelsize": 9.5,
    "ytick.labelsize": 9.5,
    "axes.grid": True,
    "grid.alpha": 0.28,
    "grid.linestyle": "--",
    "axes.spines.top": False,
    "axes.spines.right": False,
})

COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


def add_takeaway(fig, text: str, y: float = 0.015) -> None:
    """Place a clean one-line summary statement at the bottom of a figure."""
    fig.text(
        0.5, y, f"Takeaway: {text}",
        ha="center", va="bottom", fontsize=8.8, style="italic",
        color="#222222",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="#f4f6f8", edgecolor="#d0d7de", lw=0.8),
    )


def save_fig(fig, filename: str) -> None:
    out = FIG_DIR / filename
    fig.savefig(out, dpi=240)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 1: Section 1 - Feedback Loop Diagram (+ Shower Example mapping)
# ---------------------------------------------------------------------------
def make_fig01_loop_diagram() -> None:
    fig, ax = plt.subplots(figsize=(9.5, 4.3))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 4.8)
    ax.axis("off")

    ax.set_title("Figure 1: Closed-Loop Feedback Control Diagram (with Shower Temperature Analogy)", pad=10)

    def draw_box(cx, cy, w, h, title, sub, fc="#eaf2f8", ec="#1f77b4"):
        box = FancyBboxPatch(
            (cx - w / 2, cy - h / 2), w, h,
            boxstyle="round,pad=0.08,rounding_size=0.14",
            facecolor=fc, edgecolor=ec, linewidth=1.8,
        )
        ax.add_patch(box)
        ax.text(cx, cy + 0.16, title, ha="center", va="center", fontsize=10.5, fontweight="bold", color="#111111")
        ax.text(cx, cy - 0.22, sub, ha="center", va="center", fontsize=8.5, color="#444444")

    # Summing junction
    sum_x, sum_y = 2.05, 3.15
    circ = Circle((sum_x, sum_y), 0.28, facecolor="#ffffff", edgecolor="#333333", linewidth=1.8)
    ax.add_patch(circ)
    ax.text(sum_x, sum_y, "e = r - y", ha="center", va="center", fontsize=7.5, fontweight="bold")
    ax.text(sum_x - 0.42, sum_y + 0.16, "+", fontsize=11, fontweight="bold", color="#2ca02c")
    ax.text(sum_x + 0.12, sum_y - 0.46, "-", fontsize=13, fontweight="bold", color="#d62728")

    # Main boxes
    draw_box(4.2, 3.15, 2.0, 1.05, "Controller (PID)", "Brain: your hand\n(decides valve turn)", fc="#eaf2f8", ec="#1f77b4")
    draw_box(7.1, 3.15, 2.2, 1.05, "Plant (Machine)", "Shower pipes & water\n(reacts to valve)", fc="#fef5e7", ec="#e67e22")
    draw_box(5.65, 1.25, 2.3, 0.95, "Sensor", "Your skin / thermometer\n(measures actual temp y)", fc="#eafaf1", ec="#2ca02c")

    # Arrows
    arrow_kw = dict(arrowstyle="-|>,head_width=0.28,head_length=0.45", lw=1.8, color="#333333")
    # Target -> Sum
    ax.annotate("", xy=(sum_x - 0.28, sum_y), xytext=(0.35, sum_y), arrowprops=arrow_kw)
    ax.text(0.95, sum_y + 0.34, "Target r\n(e.g. 38 C)", ha="center", va="bottom", fontsize=9, fontweight="bold", color="#111111")

    # Sum -> Controller
    ax.annotate("", xy=(3.12, sum_y), xytext=(sum_x + 0.28, sum_y), arrowprops=arrow_kw)
    ax.text(2.72, sum_y + 0.18, "Error e", ha="center", va="bottom", fontsize=8.8, color="#555555")

    # Controller -> Plant
    ax.annotate("", xy=(5.92, sum_y), xytext=(5.28, sum_y), arrowprops=arrow_kw)
    ax.text(5.60, sum_y + 0.18, "Push u", ha="center", va="bottom", fontsize=8.8, color="#555555")

    # Plant -> Output
    ax.annotate("", xy=(9.65, sum_y), xytext=(8.28, sum_y), arrowprops=arrow_kw)
    ax.text(9.05, sum_y + 0.34, "Actual output y\n(water temp)", ha="center", va="bottom", fontsize=9, fontweight="bold", color="#111111")

    # Feedback tap down -> Sensor -> back to Sum
    ax.plot([8.75, 8.75], [sum_y, 1.25], color="#333333", lw=1.8)
    ax.annotate("", xy=(6.88, 1.25), xytext=(8.75, 1.25), arrowprops=arrow_kw)
    ax.plot([4.42, sum_x], [1.25, 1.25], color="#333333", lw=1.8)
    ax.annotate("", xy=(sum_x, sum_y - 0.28), xytext=(sum_x, 1.25), arrowprops=arrow_kw)
    ax.text(3.2, 1.43, "Measured value y", ha="center", va="bottom", fontsize=8.8, color="#555555")

    add_takeaway(fig, "A feedback controller continuously measures the output y, compares it to target r, and adjusts effort u until error e = 0.")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.90, bottom=0.14)
    save_fig(fig, "fig01_feedback_loop.png")


# ---------------------------------------------------------------------------
# Figure 2: Section 3 - Mass-Spring-Damper Schematic + Open-Loop 20 N Push
# ---------------------------------------------------------------------------
def make_fig02_test_system(plant: MassSpringDamper) -> None:
    fig, (ax_sch, ax_sim) = plt.subplots(1, 2, figsize=(10.5, 4.2), gridspec_kw={"width_ratios": [1.0, 1.25]})

    # Left: Mechanical schematic
    ax_sch.set_xlim(0, 10)
    ax_sch.set_ylim(0, 6)
    ax_sch.axis("off")
    ax_sch.set_title("Test System: 1 kg Mass-Spring-Damper")

    # Wall
    ax_sch.plot([1.0, 1.0], [1.0, 4.8], color="#222222", lw=3)
    for y_hatch in np.linspace(1.1, 4.6, 10):
        ax_sch.plot([0.65, 1.0], [y_hatch - 0.2, y_hatch], color="#666666", lw=1.1)

    # Ground
    ax_sch.plot([1.0, 9.5], [1.2, 1.2], color="#444444", lw=2)

    # Spring (top path y=3.6)
    sx = np.linspace(1.0, 4.6, 15)
    sy = 3.6 + np.array([0, 0, 0.35, -0.35, 0.35, -0.35, 0.35, -0.35, 0.35, -0.35, 0.35, -0.35, 0, 0, 0])
    sx[0], sx[1] = 1.0, 1.6
    sx[-2], sx[-1] = 4.0, 4.6
    ax_sch.plot(sx, sy, color="#1f77b4", lw=2.0)
    ax_sch.text(2.8, 4.25, "Spring k = 20 N/m", ha="center", fontsize=9.5, color="#1f77b4", fontweight="bold")

    # Damper (bottom path y=2.1)
    ax_sch.plot([1.0, 2.5], [2.1, 2.1], color="#2ca02c", lw=2.0)
    ax_sch.plot([2.5, 2.5, 3.4], [2.45, 1.75, 1.75], color="#2ca02c", lw=2.0)
    ax_sch.plot([2.5, 3.4], [2.45, 2.45], color="#2ca02c", lw=2.0)
    ax_sch.plot([3.0, 3.0], [1.85, 2.35], color="#2ca02c", lw=3.0)
    ax_sch.plot([3.0, 4.6], [2.1, 2.1], color="#2ca02c", lw=2.0)
    ax_sch.text(2.85, 0.72, "Damper c = 10 N·s/m", ha="center", fontsize=9.2, color="#2ca02c", fontweight="bold")

    # Block m = 1 kg
    block = FancyBboxPatch((4.6, 1.3), 2.1, 2.9, boxstyle="round,pad=0.05,rounding_size=0.1",
                           facecolor="#f0f4f8", edgecolor="#222222", lw=2.0)
    ax_sch.add_patch(block)
    ax_sch.text(5.65, 2.75, "Mass\nm = 1 kg", ha="center", va="center", fontsize=10.5, fontweight="bold")

    # Force arrow u(t)
    ax_sch.annotate("", xy=(9.3, 2.75), xytext=(6.78, 2.75),
                    arrowprops=dict(arrowstyle="-|>,head_width=0.3,head_length=0.45", lw=2.4, color="#d62728"))
    ax_sch.text(8.15, 3.12, "Push u(t) (N)", ha="center", fontsize=9.8, fontweight="bold", color="#d62728")

    # Position coordinate x
    ax_sch.plot([5.65, 5.65], [4.2, 5.0], color="#333333", ls=":", lw=1.2)
    ax_sch.annotate("", xy=(7.4, 4.9), xytext=(5.65, 4.9),
                    arrowprops=dict(arrowstyle="-|>,head_width=0.22,head_length=0.35", lw=1.5, color="#333333"))
    ax_sch.text(6.55, 5.15, "Position x(t) (m)\nTarget = 1.0 m", ha="center", fontsize=9, color="#222222")

    # Right: Open-loop constant 20 N vs 10 N push
    r20 = simulate_closed_loop(plant, None, 20.0, t_end=3.0, dt=0.001)
    r10 = simulate_closed_loop(plant, None, 10.0, t_end=3.0, dt=0.001)
    ax_sim.plot(r20["t"], r20["y"], color="#1f77b4", lw=2.2, label="Constant u = 20 N (reaches 1.00 m)")
    ax_sim.plot(r10["t"], r10["y"], color="#ff7f0e", lw=2.0, ls="-.", label="Constant u = 10 N (stops at 0.50 m)")
    ax_sim.axhline(1.0, color="#333333", ls="--", lw=1.2, label="Target x = 1.0 m")
    ax_sim.set_title("Open-Loop Test: Holding Against the Spring")
    ax_sim.set_xlabel("Time t (s)")
    ax_sim.set_ylabel("Position x (m)")
    ax_sim.set_ylim(-0.05, 1.2)
    ax_sim.legend(loc="lower right", frameon=True)

    add_takeaway(fig, "At x = 1.0 m, the spring pulls back with k·x = 20 N, so a constant 20 N push is required just to hold still.")
    fig.subplots_adjust(left=0.03, right=0.97, top=0.87, bottom=0.22, wspace=0.24)
    save_fig(fig, "fig02_test_system.png")


# ---------------------------------------------------------------------------
# Figure 3: Section 4 - Four Step-Response Measures Annotated on One Graph
# ---------------------------------------------------------------------------
def make_fig03_step_metrics(plant: MassSpringDamper) -> dict:
    r = simulate_closed_loop(plant, SimplePID(180.0, 0.0, 0.0), 1.0, t_end=1.8, dt=0.001)
    t, y = r["t"], r["y"]
    m = compute_step_metrics(t, y, setpoint=1.0)

    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    ax.plot(t, y, color="#1f77b4", lw=2.4, label="Position x(t) with Kp = 180")
    ax.axhline(1.0, color="#222222", ls="--", lw=1.5, label="Target (Setpoint r = 1.00 m)")
    ax.axhline(m["y_ss"], color="#2ca02c", ls=":", lw=1.6, label=f"Steady-state value y_ss = {m['y_ss']:.2f} m")

    # 2% settling band around y_ss
    band_lo, band_hi = 0.98 * m["y_ss"], 1.02 * m["y_ss"]
    ax.axhspan(band_lo, band_hi, color="#2ca02c", alpha=0.14, label="±2% settling band around y_ss")

    # 1. Rise time (t10 to t90)
    y10, y90 = 0.1 * m["y_ss"], 0.9 * m["y_ss"]
    ax.plot([m["t10"], m["t90"]], [y10, y90], "o", color="#ff7f0e", ms=6, zorder=5)
    ax.vlines([m["t10"], m["t90"]], 0, [y10, y90], color="#ff7f0e", ls=":", lw=1.2)
    ax.annotate("", xy=(m["t90"], 0.18), xytext=(m["t10"], 0.18),
                arrowprops=dict(arrowstyle="<->", color="#ff7f0e", lw=1.8))
    ax.text((m["t10"] + m["t90"]) / 2 + 0.08, 0.22,
            f"1. Rise time\nt_r = {m['rise_time']:.3f} s\n(10% to 90% of y_ss)",
            color="#c25900", fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc="#fff8f0", ec="#ff7f0e", lw=0.8))

    # 2. Overshoot
    ax.plot(m["peak_time"], m["peak"], "o", color="#d62728", ms=7, zorder=6)
    ax.annotate("", xy=(m["peak_time"], m["peak"]), xytext=(m["peak_time"], m["y_ss"]),
                arrowprops=dict(arrowstyle="<->", color="#d62728", lw=1.8))
    ax.text(m["peak_time"] + 0.06, 1.08,
            f"2. Overshoot\n{m['overshoot_pct']:.1f}% above y_ss\n(peak = {m['peak']:.3f} m)",
            color="#a61b1b", fontsize=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.25", fc="#fff5f5", ec="#d62728", lw=0.8))

    # 3. Settling time
    ax.axvline(m["settling_time"], color="#2ca02c", ls="--", lw=1.4)
    ax.plot(m["settling_time"], y[np.argmin(np.abs(t - m["settling_time"]))], "s", color="#2ca02c", ms=6, zorder=5)
    ax.annotate(f"3. Settling time\nt_s = {m['settling_time']:.3f} s\n(stays within ±2%)",
                xy=(m["settling_time"], 0.88), xytext=(m["settling_time"] + 0.08, 0.56),
                color="#196f3d", fontsize=9, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=1.5),
                bbox=dict(boxstyle="round,pad=0.25", fc="#f2fbf5", ec="#2ca02c", lw=0.8))

    # 4. Steady-state error
    ax.annotate("", xy=(1.68, 1.0), xytext=(1.68, m["y_ss"]),
                arrowprops=dict(arrowstyle="<->", color="#9467bd", lw=2.0))
    ax.annotate(f"4. Steady-state error\ne_ss = {m['ess']:.2f} m",
                xy=(1.68, 0.95), xytext=(1.32, 0.72),
                ha="center", va="center", color="#6c3483", fontsize=9, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color="#9467bd", lw=1.4),
                bbox=dict(boxstyle="round,pad=0.25", fc="#f9f5fc", ec="#9467bd", lw=0.8))

    ax.set_title("Figure 3: The Four Step-Response Measures (Example: P-Only Kp = 180)")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 1.8)
    ax.set_ylim(0, 1.32)
    ax.legend(loc="upper right", frameon=True, fontsize=8.5)

    add_takeaway(fig, f"One graph captures all four quality metrics: fast rise ({m['rise_time']:.3f} s), {m['overshoot_pct']:.1f}% overshoot, {m['settling_time']:.3f} s settling, and {m['ess']:.2f} m gap.")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig03_step_metrics.png")
    return m


# ---------------------------------------------------------------------------
# Figure 4, 5, 6: Section 5 - P only, PI, PD
# ---------------------------------------------------------------------------
def make_sec5_plots(plant: MassSpringDamper) -> dict:
    summary = {"p": [], "pi": [], "pd": []}

    # --- Fig 4: P-only (4 strengths) ---
    fig, ax = plt.subplots(figsize=(9.0, 4.3))
    kps = [10, 40, 100, 200]
    for idx, kp in enumerate(kps):
        r = simulate_closed_loop(plant, SimplePID(kp, 0.0, 0.0), 1.0, t_end=3.5, dt=0.001)
        m = compute_step_metrics(r["t"], r["y"], 1.0)
        theory_yss = kp / (kp + plant.k)
        summary["p"].append((kp, m, theory_yss))
        ax.plot(r["t"], r["y"], color=COLORS[idx], lw=2.1,
                label=f"Kp = {kp} (sim y_ss={m['y_ss']:.3f} m, theory={theory_yss:.3f} m)")
        ax.axhline(theory_yss, color=COLORS[idx], ls=":", lw=1.0, alpha=0.6)

    ax.axhline(1.0, color="#222222", ls="--", lw=1.4, label="Target r = 1.00 m")
    ax.set_title("Figure 4: Proportional-Only (P) Control Across Four Gains")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 3.5)
    ax.set_ylim(0, 1.30)
    ax.legend(loc="lower right", frameon=True, fontsize=8.5)
    add_takeaway(fig, "Higher Kp moves faster and shrinks the final gap, yet P-only always stops short (0.333 to 0.909 m) and starts bouncing.")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig04_p_only.png")

    # --- Fig 5: Adding I (PI control, Kp=60) ---
    fig, ax = plt.subplots(figsize=(9.0, 4.3))
    kis = [0, 60, 150, 350]
    for idx, ki in enumerate(kis):
        r = simulate_closed_loop(plant, SimplePID(60.0, ki, 0.0), 1.0, t_end=4.5, dt=0.001)
        m = compute_step_metrics(r["t"], r["y"], 1.0)
        summary["pi"].append((ki, m))
        ax.plot(r["t"], r["y"], color=COLORS[idx], lw=2.1,
                label=f"Kp=60, Ki={ki} (y_ss={m['y_ss']:.3f} m, OS={m['overshoot_pct']:.1f}%)")

    ax.axhline(1.0, color="#222222", ls="--", lw=1.4, label="Target r = 1.00 m")
    ax.set_title("Figure 5: Adding Integral Action (PI Control with Kp = 60)")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 4.5)
    ax.set_ylim(0, 1.50)
    ax.legend(loc="upper right", frameon=True, fontsize=8.5)
    add_takeaway(fig, "Integral action eliminates the 0.250 m steady-state gap, but pushing Ki too high (350) causes 47% overshoot.")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig05_pi_control.png")

    # --- Fig 6: Adding D (PD control, Kp=200) ---
    fig, ax = plt.subplots(figsize=(9.0, 4.3))
    kds = [0, 5, 15, 25]
    for idx, kd in enumerate(kds):
        r = simulate_closed_loop(plant, SimplePID(200.0, 0.0, kd), 1.0, t_end=2.0, dt=0.001)
        m = compute_step_metrics(r["t"], r["y"], 1.0)
        zeta = (plant.c + kd) / (2.0 * np.sqrt(plant.m * (plant.k + 200.0)))
        os_th = np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta**2)) * 100.0 if zeta < 1.0 else 0.0
        summary["pd"].append((kd, m, zeta, os_th))
        ax.plot(r["t"], r["y"], color=COLORS[idx], lw=2.1,
                label=f"Kp=200, Kd={kd} (zeta={zeta:.2f}, OS={m['overshoot_pct']:.1f}%)")

    ax.axhline(1.0, color="#222222", ls="--", lw=1.4, label="Target r = 1.00 m")
    ax.axhline(200.0 / 220.0, color="#666666", ls=":", lw=1.2, label="PD steady state (0.909 m)")
    ax.set_title("Figure 6: Adding Derivative Action (PD Control with Kp = 200)")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 2.0)
    ax.set_ylim(0, 1.32)
    ax.legend(loc="lower right", frameon=True, fontsize=8.5)
    add_takeaway(fig, "Derivative gain adds virtual damping (c_eff = 10 + Kd), cutting overshoot from 32.9% (Kd=0) to 0.7% (Kd=15).")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig06_pd_control.png")

    return summary


# ---------------------------------------------------------------------------
# Figure 7: Section 6 - P vs PI vs PD vs PID
# ---------------------------------------------------------------------------
def make_fig07_comparison(plant: MassSpringDamper) -> list:
    configs = [
        ("P   (Kp=100, Ki=0, Kd=0)", SimplePID(100.0, 0.0, 0.0)),
        ("PI  (Kp=100, Ki=120, Kd=0)", SimplePID(100.0, 120.0, 0.0)),
        ("PD  (Kp=100, Ki=0, Kd=8)", SimplePID(100.0, 0.0, 8.0)),
        ("PID (Kp=100, Ki=120, Kd=8)", SimplePID(100.0, 120.0, 8.0)),
    ]

    fig, ax = plt.subplots(figsize=(9.2, 4.4))
    rows = []
    for idx, (label, ctrl) in enumerate(configs):
        r = simulate_closed_loop(plant, ctrl, 1.0, t_end=2.5, dt=0.001)
        m = compute_step_metrics(r["t"], r["y"], 1.0)
        umax = float(np.max(np.abs(r["u"])))
        rows.append((label, m, umax))
        lw = 2.6 if idx == 3 else 2.0
        ls = "-" if idx != 2 else "-."
        ax.plot(r["t"], r["y"], color=COLORS[idx], lw=lw, ls=ls,
                label=f"{label} | OS={m['overshoot_pct']:.1f}%, ts={m['settling_time']:.2f}s, ess={m['ess']:.3f}m")

    ax.axhline(1.0, color="#222222", ls="--", lw=1.4, label="Target r = 1.00 m")
    ax.set_title("Figure 7: Side-by-Side Comparison of P, PI, PD, and Full PID")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 2.5)
    ax.set_ylim(0, 1.25)
    ax.legend(loc="lower right", frameon=True, fontsize=8.3)
    add_takeaway(fig, "Only full PID achieves fast rise (0.261 s), near-zero overshoot (0.86%), rapid settling (0.408 s), and zero steady error.")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig07_p_pi_pd_pid.png")
    return rows


# ---------------------------------------------------------------------------
# Figures 8-11: Section 7 - Real-World Problems & Fixes
# ---------------------------------------------------------------------------
def make_sec7_plots(plant: MassSpringDamper) -> dict:
    stats = {}

    # (a) Motor limit +-30 N -> Integrator Windup vs Anti-Windup
    naive = PIDController(120.0, 240.0, 12.0, u_min=-30.0, u_max=30.0,
                          anti_windup=False, derivative_on_measurement=True)
    fixed = PIDController(120.0, 240.0, 12.0, u_min=-30.0, u_max=30.0,
                          anti_windup=True, derivative_on_measurement=True)
    rn = simulate_closed_loop(plant, naive, 1.0, t_end=3.5, dt=0.001, u_min=-30.0, u_max=30.0)
    rf = simulate_closed_loop(plant, fixed, 1.0, t_end=3.5, dt=0.001, u_min=-30.0, u_max=30.0)
    mn = compute_step_metrics(rn["t"], rn["y"], 1.0)
    mf = compute_step_metrics(rf["t"], rf["y"], 1.0)
    stats["windup"] = (mn, mf)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 5.2), sharex=True)
    ax1.plot(rn["t"], rn["y"], color="#d62728", lw=2.1, label=f"No anti-windup (OS={mn['overshoot_pct']:.1f}%, ts={mn['settling_time']:.2f}s)")
    ax1.plot(rf["t"], rf["y"], color="#2ca02c", lw=2.3, label=f"With anti-windup clamping (OS={mf['overshoot_pct']:.1f}%, ts={mf['settling_time']:.2f}s)")
    ax1.axhline(1.0, color="#222222", ls="--", lw=1.2, label="Target r = 1.00 m")
    ax1.set_title("Figure 8: Problem (a) — Integrator Windup Under ±30 N Motor Limit & Its Fix")
    ax1.set_ylabel("Position x (m)")
    ax1.set_ylim(0, 1.38)
    ax1.legend(loc="lower right", frameon=True, fontsize=8.5)

    ax2.plot(rn["t"], rn["u"], color="#d62728", lw=1.9, label="Applied force (no anti-windup)")
    ax2.plot(rn["t"], rn["i"], color="#d62728", lw=1.4, ls=":",
             label=f"Integral term I (wound up to {np.max(rn['i']):.1f} N!)")
    ax2.plot(rf["t"], rf["u"], color="#2ca02c", lw=2.0, label="Applied force (with anti-windup)")
    ax2.axhline(30.0, color="#555555", ls="--", lw=1.0, label="Motor limit (±30 N)")
    ax2.axhline(-30.0, color="#555555", ls="--", lw=1.0)
    ax2.set_xlabel("Time t (s)")
    ax2.set_ylabel("Force (N)")
    ax2.set_xlim(0, 3.5)
    ax2.set_ylim(-38, 82)
    ax2.legend(loc="upper right", frameon=True, fontsize=8.3)
    add_takeaway(fig, f"Anti-windup stops integral buildup while pinned at 30 N, slashing overshoot from {mn['overshoot_pct']:.1f}% to {mf['overshoot_pct']:.1f}%.")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.17, hspace=0.14)
    save_fig(fig, "fig08_windup.png")

    # (b) Sensor noise (+-1 mm) -> Jumpy D vs Low-pass Filtered D
    c_raw = PIDController(120.0, 240.0, 12.0, derivative_on_measurement=True, d_filter_tau=0.0)
    c_flt = PIDController(120.0, 240.0, 12.0, derivative_on_measurement=True, d_filter_tau=0.025)
    r_raw = simulate_closed_loop(plant, c_raw, 1.0, t_end=2.5, dt=0.001, noise_std=0.0004, noise_clip=0.001, seed=42)
    r_flt = simulate_closed_loop(plant, c_flt, 1.0, t_end=2.5, dt=0.001, noise_std=0.0004, noise_clip=0.001, seed=42)
    mask_ss = r_raw["t"] >= 1.4
    p2p_raw = float(np.ptp(r_raw["u"][mask_ss]))
    p2p_flt = float(np.ptp(r_flt["u"][mask_ss]))
    std_raw = float(np.std(r_raw["u"][mask_ss]))
    std_flt = float(np.std(r_flt["u"][mask_ss]))
    stats["noise"] = (p2p_raw, p2p_flt, std_raw, std_flt)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 5.2), sharex=True)
    ax1.plot(r_raw["t"], r_raw["y"], color="#d62728", lw=1.9, label="Position x(t) — raw D")
    ax1.plot(r_flt["t"], r_flt["y"], color="#1f77b4", lw=2.0, label="Position x(t) — filtered D (tau = 0.025 s)")
    ax1.axhline(1.0, color="#222222", ls="--", lw=1.2, label="Target r = 1.00 m")
    ax1.set_title("Figure 9: Problem (b) — ±1 mm Sensor Noise Causes Derivative Chatter & Filter Fix")
    ax1.set_ylabel("Position x (m)")
    ax1.set_ylim(0, 1.25)
    ax1.legend(loc="lower right", frameon=True, fontsize=8.5)

    ax2.plot(r_raw["t"], r_raw["u"], color="#d62728", lw=0.9, alpha=0.85,
             label=f"Raw D force (steady p2p = {p2p_raw:.1f} N, std = {std_raw:.2f} N)")
    ax2.plot(r_flt["t"], r_flt["u"], color="#1f77b4", lw=1.8,
             label=f"Filtered D force (steady p2p = {p2p_flt:.1f} N, std = {std_flt:.2f} N)")
    ax2.set_xlabel("Time t (s)")
    ax2.set_ylabel("Control Force u (N)")
    ax2.set_xlim(0, 2.5)
    ax2.set_ylim(-20, 145)
    ax2.legend(loc="upper right", frameon=True, fontsize=8.5)
    add_takeaway(fig, f"A 25 ms low-pass filter on D cuts steady-state motor force chatter by {(1 - p2p_flt/p2p_raw)*100:.1f}% ({p2p_raw:.1f} N down to {p2p_flt:.1f} N).")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.17, hspace=0.14)
    save_fig(fig, "fig09_noise.png")

    # (c) Setpoint step at t=0.5s -> Derivative kick vs Derivative-on-measurement
    step_fn = lambda tk: 1.0 if tk >= 0.5 else 0.0
    c_kick = PIDController(120.0, 240.0, 12.0, derivative_on_measurement=False)
    c_safe = PIDController(120.0, 240.0, 12.0, derivative_on_measurement=True)
    rk = simulate_closed_loop(plant, c_kick, step_fn, t_end=2.2, dt=0.001)
    rs = simulate_closed_loop(plant, c_safe, step_fn, t_end=2.2, dt=0.001)
    umax_kick = float(np.max(np.abs(rk["u"])))
    umax_safe = float(np.max(np.abs(rs["u"])))
    stats["kick"] = (umax_kick, umax_safe)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 5.2), sharex=True)
    ax1.plot(rk["t"], rk["y"], color="#d62728", lw=2.0, label="Position — D on error")
    ax1.plot(rs["t"], rs["y"], color="#2ca02c", lw=2.0, ls="--", label="Position — D on measurement")
    ax1.plot(rs["t"], rs["r"], color="#222222", ls=":", lw=1.4, label="Target step r(t) at t = 0.5 s")
    ax1.set_title("Figure 10: Problem (c) — Derivative Kick on Sudden Target Step & Its Fix")
    ax1.set_ylabel("Position x (m)")
    ax1.set_ylim(-0.05, 1.25)
    ax1.legend(loc="lower right", frameon=True, fontsize=8.5)

    # Plot force on zoomed reasonable scale with annotation for the off-chart spike
    ax2.plot(rk["t"], rk["u"], color="#d62728", lw=1.8, label=f"D on error (peaks at {umax_kick:,.0f} N!)")
    ax2.plot(rs["t"], rs["u"], color="#2ca02c", lw=2.2, label=f"D on measurement (peaks at {umax_safe:.1f} N)")
    ax2.annotate(f"Huge spike: {umax_kick:,.0f} N\n(100x motor force!)",
                 xy=(0.502, 240), xytext=(0.68, 190),
                 fontsize=9, fontweight="bold", color="#a61b1b",
                 arrowprops=dict(arrowstyle="->", color="#d62728", lw=1.6),
                 bbox=dict(boxstyle="round,pad=0.25", fc="#fff5f5", ec="#d62728", lw=0.8))
    ax2.set_xlabel("Time t (s)")
    ax2.set_ylabel("Control Force u (N)")
    ax2.set_xlim(0.2, 2.2)
    ax2.set_ylim(-25, 260)
    ax2.legend(loc="upper right", frameon=True, fontsize=8.5)
    add_takeaway(fig, f"D on measurement (-dx/dt) removes the {umax_kick:,.0f} N spike, capping peak force at {umax_safe:.1f} N.")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.17, hspace=0.14)
    save_fig(fig, "fig10_derivative_kick.png")

    # (d) Slow sample rate
    dts = [0.001, 0.06, 0.10, 0.165]
    stats["sample_rate"] = []
    fig, ax = plt.subplots(figsize=(9.2, 4.4))
    for idx, dt_val in enumerate(dts):
        c = PIDController(120.0, 240.0, 12.0, derivative_on_measurement=True)
        r = simulate_closed_loop(plant, c, 1.0, t_end=3.0, dt=dt_val, substeps=20)
        m = compute_step_metrics(r["t"], r["y"], 1.0)
        stats["sample_rate"].append((dt_val, m))
        hz = 1.0 / dt_val
        if m["peak"] > 1.45 and np.max(np.abs(r["y"][-5:] - 1.0)) > 0.25:
            status = f"UNSTABLE (peak={m['peak']:.2f} m)"
        else:
            status = f"OS={m['overshoot_pct']:.1f}%"
        ax.plot(r["t"], r["y"], color=COLORS[idx], lw=2.0,
                label=f"dt = {dt_val:.3f} s ({hz:.0f} Hz) — {status}")

    ax.axhline(1.0, color="#222222", ls="--", lw=1.3, label="Target r = 1.00 m")
    ax.set_title("Figure 11: Problem (d) — Effect of Slow Computer Update Period (dt)")
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Position x (m)")
    ax.set_xlim(0, 3.0)
    ax.set_ylim(-0.1, 2.05)
    ax.legend(loc="upper left", frameon=True, fontsize=8.5)
    add_takeaway(fig, "Slow updates add delay: at dt = 0.10 s (10 Hz) overshoot rises to 18.1%, and at dt = 0.165 s (6 Hz) it becomes unstable.")
    fig.subplots_adjust(left=0.08, right=0.97, top=0.90, bottom=0.20)
    save_fig(fig, "fig11_sample_rate.png")

    return stats


# ---------------------------------------------------------------------------
# Figure 12: Section 8 - Ziegler-Nichols on G(s) = 1/(s+1)^3
# ---------------------------------------------------------------------------
def make_fig12_ziegler_nichols() -> dict:
    p3 = ThirdOrderPlant()

    # Left panel: raising P until steady oscillation at Ku = 8.0
    r_sub = simulate_closed_loop(p3, SimplePID(4.0, 0.0, 0.0), 1.0, t_end=22.0, dt=0.005)
    r_ku = simulate_closed_loop(p3, SimplePID(8.0, 0.0, 0.0), 1.0, t_end=22.0, dt=0.005)

    mask = r_ku["t"] >= 10.0
    t_m, y_m = r_ku["t"][mask], r_ku["y"][mask]
    peaks = [i for i in range(1, len(y_m) - 1) if y_m[i] > y_m[i - 1] and y_m[i] > y_m[i + 1]]
    tu_sim = float(np.mean(np.diff(t_m[peaks])))
    ku = 8.0
    tu_exact = 2.0 * np.pi / np.sqrt(3.0)

    # Right panel: Classic ZN vs Improved PID
    kp_zn = 0.6 * ku
    ki_zn = 2.0 * kp_zn / tu_exact
    kd_zn = kp_zn * tu_exact / 8.0
    r_zn = simulate_closed_loop(p3, SimplePID(kp_zn, ki_zn, kd_zn), 1.0, t_end=16.0, dt=0.005)
    m_zn = compute_step_metrics(r_zn["t"], r_zn["y"], 1.0)

    kp_imp, ki_imp, kd_imp = 3.4, 0.7, 2.6
    r_imp = simulate_closed_loop(p3, SimplePID(kp_imp, ki_imp, kd_imp), 1.0, t_end=16.0, dt=0.005)
    m_imp = compute_step_metrics(r_imp["t"], r_imp["y"], 1.0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.3))

    ax1.plot(r_sub["t"], r_sub["y"], color="#1f77b4", lw=1.8, label="Kp = 4.0 (decays)")
    ax1.plot(r_ku["t"], r_ku["y"], color="#d62728", lw=2.0, label=f"Ku = 8.0 (steady swing, Tu = {tu_sim:.2f} s)")
    ax1.axhline(1.0, color="#222222", ls="--", lw=1.1, label="Target r = 1.0")
    if len(peaks) >= 2:
        t_p1, t_p2 = t_m[peaks[0]], t_m[peaks[1]]
        y_p = max(y_m[peaks[0]], y_m[peaks[1]])
        ax1.annotate("", xy=(t_p2, y_p + 0.08), xytext=(t_p1, y_p + 0.08),
                     arrowprops=dict(arrowstyle="<->", color="#111111", lw=1.5))
        ax1.text((t_p1 + t_p2) / 2, y_p + 0.13, f"Tu = {tu_sim:.2f} s", ha="center", fontsize=9, fontweight="bold")
    ax1.set_title("Step 1: Find Ku & Tu on 1/(s+1)^3")
    ax1.set_xlabel("Time t (s)")
    ax1.set_ylabel("Output y (units)")
    ax1.set_xlim(0, 22)
    ax1.set_ylim(0, 2.05)
    ax1.legend(loc="lower right", frameon=True, fontsize=8.3)

    ax2.plot(r_zn["t"], r_zn["y"], color="#ff7f0e", lw=2.1,
             label=f"ZN PID (4.80, 2.65, 2.18) | OS={m_zn['overshoot_pct']:.1f}%, ts={m_zn['settling_time']:.2f}s")
    ax2.plot(r_imp["t"], r_imp["y"], color="#2ca02c", lw=2.3,
             label=f"Improved (3.40, 0.70, 2.60) | OS={m_imp['overshoot_pct']:.1f}%, ts={m_imp['settling_time']:.2f}s")
    ax2.axhline(1.0, color="#222222", ls="--", lw=1.2, label="Target r = 1.0")
    ax2.set_title("Step 2: Ziegler-Nichols vs Improved Tuning")
    ax2.set_xlabel("Time t (s)")
    ax2.set_ylabel("Output y (units)")
    ax2.set_xlim(0, 16)
    ax2.set_ylim(0, 1.75)
    ax2.legend(loc="upper right", frameon=True, fontsize=8.0)

    add_takeaway(fig, f"ZN gives a fast start but {m_zn['overshoot_pct']:.1f}% overshoot; trimming Kp & Ki and raising Kd drops overshoot to {m_imp['overshoot_pct']:.1f}%.")
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.21, wspace=0.22)
    save_fig(fig, "fig12_ziegler_nichols.png")

    return {
        "ku": ku,
        "tu_sim": tu_sim,
        "tu_exact": tu_exact,
        "zn_gains": (kp_zn, ki_zn, kd_zn),
        "zn_metrics": m_zn,
        "imp_gains": (kp_imp, ki_imp, kd_imp),
        "imp_metrics": m_imp,
    }


# ---------------------------------------------------------------------------
# Figure 13: Section 9 - Full Production PIDController Demo
# ---------------------------------------------------------------------------
def make_fig13_section9_demo(plant: MassSpringDamper) -> dict:
    ctrl = PIDController(
        kp=120.0,
        ki=240.0,
        kd=12.0,
        u_min=-30.0,
        u_max=30.0,
        anti_windup=True,
        derivative_on_measurement=True,
        d_filter_tau=0.025,
    )
    step_fn = lambda tk: 1.0 if tk >= 0.3 else 0.0
    r = simulate_closed_loop(
        plant, ctrl, step_fn, t_end=3.0, dt=0.005, substeps=10,
        u_min=-30.0, u_max=30.0, noise_std=0.0004, noise_clip=0.001, seed=42,
    )
    # Compute step metrics shifted relative to step at t=0.3 s
    mask = r["t"] >= 0.3
    m = compute_step_metrics(r["t"][mask] - 0.3, r["y"][mask], 1.0)
    u_ss = r["u"][r["t"] >= 1.8]
    u_ss_p2p = float(np.ptp(u_ss))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 5.0), sharex=True)
    ax1.plot(r["t"], r["y_meas"], color="#aec7e8", lw=1.0, label="Noisy sensor (±1 mm)")
    ax1.plot(r["t"], r["y"], color="#1f77b4", lw=2.3,
             label=f"True position x(t) (tr={m['rise_time']:.3f}s, OS={m['overshoot_pct']:.2f}%, ts={m['settling_time']:.3f}s)")
    ax1.plot(r["t"], r["r"], color="#222222", ls="--", lw=1.3, label="Target r(t) (0 -> 1 m at t=0.3 s)")
    ax1.set_title("Figure 13: Reusable PIDController on Test System (Limit ±30 N, Noise ±1 mm, 200 Hz)")
    ax1.set_ylabel("Position x (m)")
    ax1.set_ylim(-0.05, 1.22)
    ax1.legend(loc="lower right", frameon=True, fontsize=8.3)

    ax2.plot(r["t"], r["u"], color="#2ca02c", lw=1.8, label=f"Motor force u(t) (steady p2p = {u_ss_p2p:.2f} N)")
    ax2.axhline(30.0, color="#d62728", ls="--", lw=1.1, label="Actuator limit (±30 N)")
    ax2.axhline(-30.0, color="#d62728", ls="--", lw=1.1)
    ax2.set_xlabel("Time t (s)")
    ax2.set_ylabel("Control Force u (N)")
    ax2.set_xlim(0, 3.0)
    ax2.set_ylim(-10, 36)
    ax2.legend(loc="upper right", frameon=True, fontsize=8.5)

    add_takeaway(fig, f"All 4 fixes work together: no kick at t=0.3 s, smooth motor saturation at 30 N, {m['overshoot_pct']:.2f}% overshoot, and {m['ess']:.4f} m error.")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.17, hspace=0.14)
    save_fig(fig, "fig13_reusable_pid_demo.png")
    return {"metrics": m, "u_ss_p2p": u_ss_p2p, "u_max": float(np.max(r["u"]))}


def main() -> None:
    plant = MassSpringDamper(mass=1.0, damping=10.0, stiffness=20.0)

    make_fig01_loop_diagram()
    make_fig02_test_system(plant)
    m4 = make_fig03_step_metrics(plant)
    s5 = make_sec5_plots(plant)
    s6 = make_fig07_comparison(plant)
    s7 = make_sec7_plots(plant)
    s8 = make_fig12_ziegler_nichols()
    s9 = make_fig13_section9_demo(plant)

    print("=== Generated 13 figures in figures/ ===")
    print(f"Sec 4 (Kp=180): y_ss={m4['y_ss']:.4f} m, tr={m4['rise_time']:.3f} s, OS={m4['overshoot_pct']:.2f}%, peak={m4['peak']:.3f} m, ts={m4['settling_time']:.3f} s, ess={m4['ess']:.4f} m")

    print("\nSec 5 P-only:")
    for kp, m, th in s5["p"]:
        print(f"  Kp={kp:3d}: sim y_ss={m['y_ss']:.4f} m, theory={th:.4f} m, ess={m['ess']:.4f} m, tr={m['rise_time']:.3f} s, OS={m['overshoot_pct']:.2f}%, ts={m['settling_time']:.3f} s")

    print("\nSec 5 PI (Kp=60):")
    for ki, m in s5["pi"]:
        print(f"  Ki={ki:3d}: y_ss={m['y_ss']:.4f} m, ess={m['ess']:.4f} m, tr={m['rise_time']:.3f} s, OS={m['overshoot_pct']:.2f}%, ts={m['settling_time']:.3f} s")

    print("\nSec 5 PD (Kp=200):")
    for kd, m, zeta, os_th in s5["pd"]:
        print(f"  Kd={kd:2d}: zeta={zeta:.4f}, OS_theory={os_th:.2f}%, OS_sim={m['overshoot_pct']:.2f}%, tr={m['rise_time']:.3f} s, ts={m['settling_time']:.3f} s")

    print("\nSec 6 Comparison:")
    for label, m, umax in s6:
        print(f"  {label:28s} | tr={m['rise_time']:.3f} s | OS={m['overshoot_pct']:5.2f}% | ts={m['settling_time']:.3f} s | ess={m['ess']:.4f} m | umax={umax:.1f} N")

    mn, mf = s7["windup"]
    print(f"\nSec 7(a) Windup: Naive OS={mn['overshoot_pct']:.2f}%, ts={mn['settling_time']:.3f} s | Anti-windup OS={mf['overshoot_pct']:.2f}%, ts={mf['settling_time']:.3f} s")
    p2p_r, p2p_f, std_r, std_f = s7["noise"]
    print(f"Sec 7(b) Noise: Raw p2p={p2p_r:.2f} N (std={std_r:.2f} N) | Filtered p2p={p2p_f:.2f} N (std={std_f:.2f} N)")
    uk, us = s7["kick"]
    print(f"Sec 7(c) Kick: D-on-error max={uk:.1f} N | D-on-meas max={us:.1f} N")
    print("Sec 7(d) Sample rate:")
    for dt_v, m in s7["sample_rate"]:
        print(f"  dt={dt_v:.3f} s: y_end={m['y_ss']:.4f} m, OS={m['overshoot_pct']:.2f}%, ts={m['settling_time']:.3f} s")

    print(f"\nSec 8 ZN: Ku={s8['ku']}, Tu_sim={s8['tu_sim']:.4f} s (exact={s8['tu_exact']:.4f} s)")
    print(f"  ZN gains={tuple(round(v, 3) for v in s8['zn_gains'])}: tr={s8['zn_metrics']['rise_time']:.3f} s, OS={s8['zn_metrics']['overshoot_pct']:.2f}%, ts={s8['zn_metrics']['settling_time']:.3f} s")
    print(f"  Imp gains={s8['imp_gains']}: tr={s8['imp_metrics']['rise_time']:.3f} s, OS={s8['imp_metrics']['overshoot_pct']:.2f}%, ts={s8['imp_metrics']['settling_time']:.3f} s")

    print(f"\nSec 9 Demo: tr={s9['metrics']['rise_time']:.3f} s, OS={s9['metrics']['overshoot_pct']:.2f}%, ts={s9['metrics']['settling_time']:.3f} s, ess={s9['metrics']['ess']:.4f} m, u_max={s9['u_max']:.1f} N, u_ss_p2p={s9['u_ss_p2p']:.2f} N")


if __name__ == "__main__":
    main()
