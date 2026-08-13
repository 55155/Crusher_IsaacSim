"""plot_rigid_vs_fem_cost.py — computational cost: all-rigid process validation
vs. FEM + IPC (publication figure).

**Controlled measurement, 2026-08-14.** Everything below was measured in one
sitting on one machine under matched conditions — this replaces the earlier
figure, which paired a freshly measured rigid number against an FEM number of
unknown provenance ("16 min 18 s") and consequently overstated the speed-up as
12x. The controlled ratio is ~3.2x.

What is held equal:
  * Genesis 1.3.3 + quadrants 1.3.0, same box, back-to-back runs.
  * Identical 2230-step sequence — `full_workflow_rigid.py` imports DT and every
    per-phase N_* from `full_workflow.py`, so the step count is shared by
    construction, not by coincidence.
  * Identical cameras: 3 of them (1280x960, 960x720, 960x720), same poses, all
    recording mp4 at fps=30. Both encode 318 frames (Genesis subsamples at
    round(1/(fps*dt)) = 7 steps/frame).
  * Identical measurement boundary: the same `time.time()` fences around
    `scene.build()` and around steps+encoding, printed by each script.
  * Warm kernel cache for both (each was run twice; the second run is reported).

Measured (warm):
                build    steps (2230)        total
  FEM + IPC     12.0 s   261.5 s (117.3 ms)  273.5 s
  all-rigid      9.1 s    76.2 s ( 34.2 ms)   85.3 s

Cold for reference: FEM 77.1 + 262.2 = 339.3 s; rigid 111.6 + 79.4 = 191.0 s.
Step cost is what generalises — it is near-identical cold vs warm on both sides
(117.6/117.3 and 35.6/34.2 ms), whereas build is a one-off dominated by kernel
compilation.

Style targets Digital Discovery (RSC): double-column 17.1 cm, Arial, 7-9 pt,
ticks out, greyscale-safe fills.

Output: docs/rigid_vs_fem_cost.{png,pdf}   (pdf is vector — prefer for submission)
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

CM = 1 / 2.54
FIG_W, FIG_H = 17.1 * CM, 6.4 * CM

for _f in ("Arial", "Helvetica", "Liberation Sans", "DejaVu Sans"):
    if any(_f == f.name for f in matplotlib.font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = "sans-serif"
        plt.rcParams["font.sans-serif"] = [_f]
        break
plt.rcParams.update({
    "axes.unicode_minus": False, "font.size": 8, "axes.labelsize": 8,
    "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.major.size": 2.6, "ytick.major.size": 2.6,
    "xtick.direction": "out", "ytick.direction": "out",
    "pdf.fonttype": 42, "ps.fonttype": 42,
})

INK = "#000000"
ACCENT = "#1b7f6b"    # all-rigid — darkest, survives greyscale
BASE = "#d3d3ce"      # FEM + IPC
BUILD = "#8f8f8a"     # build segment (mid grey, distinct from both)
GRID = "#e6e6e1"

N_STEPS = 2230
FEM_BUILD, FEM_STEPS = 12.0, 261.5
RIG_BUILD, RIG_STEPS = 9.1, 76.2
FEM_MS, RIG_MS = 117.3, 34.2

TOTAL_RATIO = (FEM_BUILD + FEM_STEPS) / (RIG_BUILD + RIG_STEPS)
STEP_RATIO = FEM_MS / RIG_MS


def main():
    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(FIG_W, FIG_H), gridspec_kw={"width_ratios": [2.25, 1]})

    # ── (a) wall-clock, split build vs steps ────────────────────────────────
    rows = [("FEM + IPC", FEM_BUILD, FEM_STEPS, BASE),
            ("All-rigid\n(this work)", RIG_BUILD, RIG_STEPS, ACCENT)]
    for y, (lab, b, s, fc) in enumerate(rows):
        ax.barh(y, b, height=0.46, facecolor=BUILD, edgecolor=INK,
                linewidth=0.6, zorder=3)
        ax.barh(y, s, height=0.46, left=b, facecolor=fc, edgecolor=INK,
                linewidth=0.6, zorder=3)
        ax.text(b + s + 6, y, f"{b + s:.1f} s", va="center", ha="left",
                fontsize=7.5, color=INK)

    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([r[0] for r in rows])
    ax.invert_yaxis()
    ax.set_xlim(0, 330)
    ax.set_xlabel(f"Wall-clock time per run (s), {N_STEPS} steps")
    ax.xaxis.grid(True, color=GRID, linewidth=0.5, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    ax.annotate("", xy=(RIG_BUILD + RIG_STEPS, 0.62),
                xytext=(FEM_BUILD + FEM_STEPS, 0.62),
                arrowprops=dict(arrowstyle="<|-|>", color=INK, lw=0.7,
                                mutation_scale=7, shrinkA=0, shrinkB=0))
    ax.text((RIG_BUILD + RIG_STEPS + FEM_BUILD + FEM_STEPS) / 2, 0.55,
            f"{TOTAL_RATIO:.1f}$\\times$", ha="center", va="bottom", fontsize=8)

    ax.legend(handles=[Patch(facecolor=BUILD, edgecolor=INK, linewidth=0.6,
                             label="build (one-off)"),
                       Patch(facecolor=BASE, edgecolor=INK, linewidth=0.6,
                             label="solve + encode")],
              loc="lower right", frameon=False, fontsize=7,
              handlelength=1.5, borderpad=0.2, labelspacing=0.3)

    # ── (b) per-step cost — the version-independent number ──────────────────
    bars = ax2.bar(["FEM + IPC", "All-rigid"], [FEM_MS, RIG_MS], width=0.46,
                   color=[BASE, ACCENT], edgecolor=INK, linewidth=0.6, zorder=3)
    for b, v in zip(bars, [FEM_MS, RIG_MS]):
        ax2.text(b.get_x() + b.get_width() / 2, v + 2.5, f"{v:.1f}",
                 ha="center", va="bottom", fontsize=7.5, color=INK)
    ax2.set_ylim(0, FEM_MS * 1.24)
    ax2.set_ylabel("Solver cost per step (ms)")
    ax2.yaxis.grid(True, color=GRID, linewidth=0.5, zorder=0)
    ax2.set_axisbelow(True)
    for s in ("top", "right"):
        ax2.spines[s].set_visible(False)
    ax2.text(0.5, FEM_MS * 1.13, f"{STEP_RATIO:.1f}$\\times$", ha="center",
             va="center", fontsize=8, transform=ax2.transData)

    for a, tag in ((ax, "a"), (ax2, "b")):
        a.text(-0.005, 1.06, f"({tag})", transform=a.transAxes,
               fontsize=9, fontweight="bold", va="bottom", ha="right")

    fig.tight_layout(pad=0.5, w_pad=2.0)
    out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs")
    png = os.path.join(out_dir, "rigid_vs_fem_cost.png")
    pdf = os.path.join(out_dir, "rigid_vs_fem_cost.pdf")
    fig.savefig(png, dpi=600, bbox_inches="tight")
    fig.savefig(pdf, bbox_inches="tight")
    print(f"[saved] {png}\n[saved] {pdf}")
    print(f"[data ] total {TOTAL_RATIO:.2f}x   per-step {STEP_RATIO:.2f}x")
    print("[caption] Computational cost of the all-rigid process-validation mode "
          "against the FEM + IPC reference. Both were run back-to-back on Genesis "
          f"1.3.3 over the identical {N_STEPS}-step sequence with identical "
          "cameras and recording, on a warm kernel cache, and timed by the same "
          "instrumentation. (a) wall-clock per run, split into one-off build and "
          "solve+encode. (b) solver cost per step, which is insensitive to cache "
          "state.")


if __name__ == "__main__":
    main()
