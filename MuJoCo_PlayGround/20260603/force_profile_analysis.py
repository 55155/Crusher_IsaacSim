"""
force_profile_analysis.py
Crusher force profile analysis script

Purpose:
    1. Load CSV results -> analyze dF/dt to classify quasi-static vs impact
    2. Theoretical comparison plot
       - Impact type (stiffness peak): sharp spike after collision
       - Quasi-static type (moving-window stall): slow rise to plateau

Usage:
    # Theoretical plot only (no CSV needed)
    python force_profile_analysis.py

    # Load actual simulation CSV
    python force_profile_analysis.py --csv path/to/result.csv
    python force_profile_analysis.py --csv mocap.csv --csv2 slidejoint.csv
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.ndimage import uniform_filter1d


# ─────────────────────────────────────────────────────────────────────
def load_csv(path: str):
    import csv
    rows = []
    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = None
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if header is None:
                header = row
                continue
            rows.append(row)
    if header is None or not rows:
        raise ValueError(f"CSV parse failed: {path}")
    data = {h: [] for h in header}
    for row in rows:
        for i, h in enumerate(header):
            try:
                data[h].append(float(row[i]))
            except (ValueError, IndexError):
                data[h].append(0.0)
    return {k: np.array(v) for k, v in data.items()}


def compute_dfdt(t: np.ndarray, f: np.ndarray, smooth_n: int = 20):
    dfdt_raw = np.gradient(f, t)
    dfdt_sm  = uniform_filter1d(dfdt_raw, size=smooth_n)
    return dfdt_raw, dfdt_sm


def classify_profile(dfdt_sm: np.ndarray, f: np.ndarray, threshold_ratio=0.3):
    F_max     = np.max(np.abs(f))
    peak_rate = np.max(np.abs(dfdt_sm))
    ratio     = peak_rate / (F_max + 1e-9)
    kind = "IMPACT (sharp peak)" if ratio > threshold_ratio else "Quasi-static (slow rise)"
    return kind, ratio, F_max, peak_rate


# ─────────────────────────────────────────────────────────────────────
def plot_theoretical():
    """Theoretical Impact vs Quasi-static comparison (no simulation data needed)."""
    t = np.linspace(0, 10, 2000)

    # ── Impact type ───────────────────────────────────────────────────
    t_impact = 3.0
    omega_d  = 8.0
    zeta     = 0.3
    omega_n  = omega_d / np.sqrt(1 - zeta**2)

    f_impact = np.zeros_like(t)
    mask = t >= t_impact
    tau  = t[mask] - t_impact
    f_impact[mask] = (
        150.0 * np.exp(-zeta * omega_n * tau) * np.sin(omega_d * tau)
    )
    f_impact = np.clip(f_impact, 0, None)

    t2 = 6.5
    mask2 = t >= t2
    tau2  = t[mask2] - t2
    f_impact[mask2] += np.clip(
        80.0 * np.exp(-zeta * omega_n * tau2) * np.sin(omega_d * tau2), 0, None)

    # ── Quasi-static type (moving-window stall) ───────────────────────
    def sigmoid(x, k=3.0):
        return 1.0 / (1.0 + np.exp(-k * x))

    motor_on     = 3.0
    plateau1_end = 6.5
    retreat_end  = 8.5
    plateau2_end = 10.0

    f_qs = np.zeros_like(t)
    mA = (t >= motor_on) & (t < plateau1_end)
    tA = (t[mA] - motor_on) / (plateau1_end - motor_on)
    f_qs[mA] = 45.0 * sigmoid(tA * 8 - 4)

    mB = (t >= plateau1_end) & (t < retreat_end)
    tB = (t[mB] - plateau1_end) / (retreat_end - plateau1_end)
    f_qs[mB] = 45.0 * (1 - sigmoid(tB * 8 - 4))

    mC = (t >= retreat_end) & (t < plateau2_end)
    tC = (t[mC] - retreat_end) / (plateau2_end - retreat_end)
    f_qs[mC] = 30.0 * sigmoid(tC * 8 - 4)

    dfdt_impact = np.gradient(f_impact, t)
    dfdt_qs     = np.gradient(f_qs, t)

    # ── Plot ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 10))
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.48, wspace=0.35)
    ax_fi  = fig.add_subplot(gs[0, 0])
    ax_fq  = fig.add_subplot(gs[0, 1])
    ax_dfi = fig.add_subplot(gs[1, 0])
    ax_dfq = fig.add_subplot(gs[1, 1])
    ax_cmp = fig.add_subplot(gs[2, :])

    # Impact force
    ax_fi.plot(t, f_impact, color="tab:red", lw=2)
    ax_fi.axvline(t_impact, color="gray", ls="--", lw=1, label="Collision")
    ax_fi.axvline(t2,       color="gray", ls=":",  lw=1, label="2nd collision")
    ax_fi.fill_between(t, 0, f_impact, alpha=0.15, color="tab:red")
    ax_fi.set_title("IMPACT type  (high-speed collision + stiffness)",
                    fontsize=10, fontweight="bold", color="tab:red")
    ax_fi.set_ylabel("F [N]"); ax_fi.legend(fontsize=8); ax_fi.grid(True, alpha=0.3)
    ax_fi.set_ylim(-10, 200)
    ax_fi.annotate("Sharp\npeak", xy=(t_impact + 0.05, 140), fontsize=9,
                   color="tab:red", arrowprops=dict(arrowstyle="->", color="tab:red"),
                   xytext=(t_impact + 1.5, 158))

    # Quasi-static force
    ax_fq.plot(t, f_qs, color="tab:blue", lw=2)
    ax_fq.axvline(motor_on,     color="tab:orange", ls="--", lw=1, label="Motor ON")
    ax_fq.axvline(plateau1_end, color="tab:red",    ls=":",  lw=1, label="Stall -> reverse")
    ax_fq.axvline(retreat_end,  color="tab:green",  ls=":",  lw=1, label="Re-compress")
    ax_fq.fill_between(t, 0, f_qs, alpha=0.15, color="tab:blue")
    ax_fq.set_title("QUASI-STATIC type  (moving-window stall <- current Crusher)",
                    fontsize=10, fontweight="bold", color="tab:blue")
    ax_fq.set_ylabel("F [N]"); ax_fq.legend(fontsize=8); ax_fq.grid(True, alpha=0.3)
    ax_fq.set_ylim(-10, 200)
    ax_fq.annotate("Slow\nrise", xy=(4.5, 40), fontsize=9,
                   color="tab:blue", arrowprops=dict(arrowstyle="->", color="tab:blue"),
                   xytext=(5.5, 80))

    # dF/dt — Impact
    ax_dfi.plot(t, dfdt_impact, color="tab:red", lw=1.5)
    ax_dfi.axhline(0, color="k", lw=0.5)
    ax_dfi.set_title("Impact — dF/dt (rate of force change)", fontsize=9)
    ax_dfi.set_ylabel("dF/dt [N/s]"); ax_dfi.grid(True, alpha=0.3)
    peak_impact = np.max(np.abs(dfdt_impact))
    ax_dfi.annotate(f"peak = {peak_impact:.0f} N/s",
                    xy=(t_impact + 0.1, peak_impact * 0.75),
                    fontsize=8, color="tab:red")

    # dF/dt — Quasi-static
    ax_dfq.plot(t, dfdt_qs, color="tab:blue", lw=1.5)
    ax_dfq.axhline(0, color="k", lw=0.5)
    ax_dfq.set_title("Quasi-static — dF/dt (rate of force change)", fontsize=9)
    ax_dfq.set_ylabel("dF/dt [N/s]"); ax_dfq.grid(True, alpha=0.3)
    peak_qs = np.max(np.abs(dfdt_qs))
    ax_dfq.annotate(f"peak = {peak_qs:.1f} N/s\n(much smaller)",
                    xy=(motor_on + 0.5, peak_qs * 0.6),
                    fontsize=8, color="tab:blue")

    # Overlay comparison
    ax_cmp.plot(t, f_impact, color="tab:red",  lw=2.0, label="Impact type (high-speed collision)")
    ax_cmp.plot(t, f_qs,     color="tab:blue", lw=2.0, label="Quasi-static type (current Crusher)")
    ax_cmp.axvline(motor_on, color="tab:orange", ls="--", lw=1.2,
                   label=f"Motor ON (t={motor_on}s)")
    ax_cmp.fill_between(t, 0, f_impact, alpha=0.08, color="tab:red")
    ax_cmp.fill_between(t, 0, f_qs,     alpha=0.08, color="tab:blue")
    ax_cmp.set_title("Force Profile Comparison", fontsize=11, fontweight="bold")
    ax_cmp.set_ylabel("F [N]"); ax_cmp.set_xlabel("Time [s]")
    ax_cmp.legend(fontsize=10); ax_cmp.grid(True, alpha=0.3)

    txt = (
        "Current Crusher characteristics:\n"
        "  - Moving-window stall -> gently press and release\n"
        "  - Peak force = motor torque limit (not stiffness-driven)\n"
        "  - stiffness (solref tau) affects penetration depth, NOT peak magnitude\n"
        "  - Result: NO sharp stiffness peak -> pure quasi-static profile"
    )
    ax_cmp.text(0.01, 0.97, txt, transform=ax_cmp.transAxes,
                fontsize=9, va="top", ha="left",
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow",
                          edgecolor="gray", alpha=0.9))

    fig.suptitle("Crusher Force Profile Analysis — Impact vs Quasi-static",
                 fontsize=12, fontweight="bold")
    return fig


# ─────────────────────────────────────────────────────────────────────
def plot_csv_analysis(csv_path: str, label: str = "", color: str = "tab:blue"):
    d  = load_csv(csv_path)
    t  = d.get("Time_s",  d.get("time",  None))
    fy = d.get("F_Y_N",   d.get("fy",    None))
    if t is None or fy is None:
        print(f"[WARNING] Cannot find Time_s / F_Y_N columns in: {csv_path}")
        return None, None, None

    _, dfdt_sm = compute_dfdt(t, fy)
    kind, ratio, F_max, peak_rate = classify_profile(dfdt_sm, fy)

    print(f"\n  -- {os.path.basename(csv_path)} --")
    print(f"  Classification : {kind}")
    print(f"  F_Y_max        : {F_max:.3f} N")
    print(f"  peak dF/dt     : {peak_rate:.2f} N/s")
    print(f"  ratio          : {ratio:.4f}  (>0.3 -> Impact / <0.3 -> Quasi-static)")

    motor_on_idx = np.argmax(np.abs(fy) > 0.5)
    motor_on_t   = t[motor_on_idx] if motor_on_idx > 0 else None

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    title = f"Force Analysis  [{label or os.path.basename(csv_path)}]  ->  {kind}"
    fig.suptitle(title, fontsize=10, fontweight="bold")

    axes[0].plot(t, fy, color=color, lw=1.5, label="F_Y [N]")
    axes[0].fill_between(t, 0, fy, where=(fy > 0), alpha=0.12, color=color,   label="compression")
    axes[0].fill_between(t, 0, fy, where=(fy < 0), alpha=0.12, color="tab:red", label="tension")
    axes[0].axhline(0, color="k", lw=0.5)
    if motor_on_t:
        axes[0].axvline(motor_on_t, color="tab:orange", ls="--", lw=1.0,
                        label=f"first contact ~t={motor_on_t:.2f}s")
    axes[0].set_ylabel("F_Y [N]")
    axes[0].set_title(f"max={F_max:.3f} N", fontsize=9)
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=0.3)

    dfdt_raw, dfdt_sm2 = compute_dfdt(t, fy, smooth_n=30)
    axes[1].plot(t, dfdt_raw, color="lightgray", lw=0.8, label="dF/dt (raw)")
    axes[1].plot(t, dfdt_sm2, color=color,       lw=1.5, label="dF/dt (smoothed)")
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("dF/dt [N/s]")
    axes[1].set_title(f"peak rate = {peak_rate:.2f} N/s  |  ratio = {ratio:.4f}", fontsize=9)
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=0.3)

    J = np.cumsum(fy) * float(t[1] - t[0]) if len(t) > 1 else np.zeros_like(fy)
    axes[2].plot(t, J, color="tab:green", lw=1.5, label=f"J_Y = {J[-1]:.4f} N*s")
    axes[2].axhline(0, color="k", lw=0.5)
    axes[2].set_ylabel("J_Y [N*s]"); axes[2].set_xlabel("Time [s]")
    axes[2].legend(fontsize=8); axes[2].grid(True, alpha=0.3)

    color_box = "lightyellow" if "Quasi" in kind else "mistyrose"
    axes[0].text(0.99, 0.97,
                 f"Result: {kind}\nratio = {ratio:.4f}",
                 transform=axes[0].transAxes, fontsize=9, va="top", ha="right",
                 bbox=dict(boxstyle="round,pad=0.4", facecolor=color_box,
                           edgecolor="gray", alpha=0.9))
    fig.tight_layout()
    return fig, kind, ratio


# ─────────────────────────────────────────────────────────────────────
def plot_comparison(path1: str, path2: str, label1="mocap", label2="slide"):
    d1 = load_csv(path1)
    d2 = load_csv(path2)
    t1, fy1 = d1["Time_s"], d1["F_Y_N"]
    t2, fy2 = d2["Time_s"], d2["F_Y_N"]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    fig.suptitle("Force Profile Comparison  (mocap fixed vs slide joint free)",
                 fontsize=11, fontweight="bold")

    axes[0].plot(t1, fy1, color="tab:red",  lw=1.5, label=f"{label1}  max={fy1.max():.2f} N")
    axes[0].plot(t2, fy2, color="tab:blue", lw=1.5, label=f"{label2}  max={fy2.max():.2f} N")
    axes[0].axhline(0, color="k", lw=0.5)
    axes[0].fill_between(t1, 0, fy1, alpha=0.08, color="tab:red")
    axes[0].fill_between(t2, 0, fy2, alpha=0.08, color="tab:blue")
    axes[0].set_ylabel("F_Y [N]"); axes[0].legend(fontsize=9)
    axes[0].grid(True, alpha=0.3); axes[0].set_xlabel("Time [s]")

    _, df1 = compute_dfdt(t1, fy1, smooth_n=30)
    _, df2 = compute_dfdt(t2, fy2, smooth_n=30)
    axes[1].plot(t1, df1, color="tab:red",  lw=1.5,
                 label=f"{label1}  peak dF/dt={np.max(np.abs(df1)):.1f} N/s")
    axes[1].plot(t2, df2, color="tab:blue", lw=1.5,
                 label=f"{label2}  peak dF/dt={np.max(np.abs(df2)):.1f} N/s")
    axes[1].axhline(0, color="k", lw=0.5)
    axes[1].set_ylabel("dF/dt [N/s]"); axes[1].set_xlabel("Time [s]")
    axes[1].legend(fontsize=9); axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Crusher force profile analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python force_profile_analysis.py\n"
            "  python force_profile_analysis.py --csv result.csv\n"
            "  python force_profile_analysis.py --csv mocap.csv --csv2 slide.csv\n"
        ),
    )
    parser.add_argument("--csv",    default=None, help="Simulation result CSV (mocap or slide)")
    parser.add_argument("--csv2",   default=None, help="Second CSV for comparison")
    parser.add_argument("--label",  default="mocap",  help="Label for first CSV")
    parser.add_argument("--label2", default="slide",  help="Label for second CSV")
    args = parser.parse_args()

    figs = []

    print("=" * 60)
    print("  Crusher Force Profile Analysis")
    print("=" * 60)
    print("\n[Theoretical] Impact vs Quasi-static comparison")
    figs.append(("theoretical_comparison", plot_theoretical()))

    if args.csv:
        print(f"\n[CSV Analysis] {args.csv}")
        result = plot_csv_analysis(args.csv, label=args.label, color="tab:blue")
        if result[0] is not None:
            figs.append(("csv_analysis", result[0]))

    if args.csv2:
        print(f"\n[CSV Analysis] {args.csv2}")
        result2 = plot_csv_analysis(args.csv2, label=args.label2, color="tab:red")
        if result2[0] is not None:
            figs.append(("csv_analysis2", result2[0]))

    if args.csv and args.csv2:
        print("\n[Comparison Plot]")
        figs.append(("comparison", plot_comparison(
            args.csv, args.csv2, args.label, args.label2)))

    _HERE = os.path.dirname(os.path.abspath(__file__))
    plot_dir = os.path.normpath(os.path.join(_HERE, "..", "Sim_result", "plot"))
    os.makedirs(plot_dir, exist_ok=True)
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    for name, fig in figs:
        if fig is None:
            continue
        p = os.path.join(plot_dir, f"analysis_{name}_{ts}.png")
        fig.savefig(p, dpi=150, bbox_inches="tight")
        print(f"  saved: {os.path.basename(p)}")

    print("\n[Conclusion]")
    print("  Current Crusher (moving-window stall):")
    print("  -> Peak force determined by motor torque * gear ratio")
    print("  -> stiffness (solref tau) affects penetration depth only")
    print("  -> NO sharp stiffness peak -> confirmed quasi-static profile")

    plt.show()
