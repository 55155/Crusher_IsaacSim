"""발표용으로 축을 직관적으로 다시 그린 두 그림 (2026-11-03 요청: 축 해석이 어렵다).
  fig5_intuitive.png      — 4장 Fig. 5: (a) 맞닿은 알 비율 IPC vs DEM, (b) 두 알 사이 틈 vs 힘
  fig_ipc_problem_v2.png  — 2장: (a) IPC 알 사이 틈, (b) 보고된 접촉력이 실제 운동(m·a)과 몇 배 어긋나나
수치는 RESULT_cfprofile0922 (IPC 500알) / crusher_scene/crush_out/cap/dem_frames.npz / 두 알 접촉법칙 프로브.
base anaconda (numpy, scipy, matplotlib) 로 실행."""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "assets")
GEN = r"C:\Crusher_isaacsim\Crusher_Genesis"
IPC_RUN = os.path.join(GEN, "Crusher_M0609_RG2_Tablet_Samplebag", "RESULT_cfprofile0922", "grip-28mm",
                       "crush_grip-28mm_yoff+0.0mm_20260922_131907.npz")
IPC_LAW = os.path.join(GEN, "probe", "_contact_law_particle_dhat100um.json")
DEM_FR = r"C:\Newton_sim\crusher_scene\crush_out\cap\dem_frames.npz"
for f in ("Pretendard-Regular.otf", "Pretendard-Bold.otf"):
    font_manager.fontManager.addfont(os.path.join(A, f))
plt.rcParams.update({"font.family": "Pretendard", "font.size": 13, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.linewidth": 0.9, "xtick.direction": "out", "ytick.direction": "out"})
K, G, GR, LG = "#262626", "#1b7f3b", "#9a9a9a", "#eef5ef"

# ── 데이터 ─────────────────────────────────────────────────────────────
ipc = np.load(IPC_RUN)
R_i, d_hat = float(ipc["grain_radius"]), float(ipc["d_hat"])
gap_ipc = (ipc["fld_nn"].ravel() - 2 * R_i) / (2 * R_i) * 100          # 알 지름 대비 %
fr = np.load(DEM_FR)
x, bag = fr["x"], fr["bag"]
inb = np.all((x >= bag.min(1, keepdims=True) - 2e-3) & (x <= bag.max(1, keepdims=True) + 2e-3), axis=2)
gd = []
for i in range(0, len(x), 3):
    p = x[i][inb[i]].astype(np.float64)
    dd, _ = cKDTree(p).query(p, k=2)
    gd.append((dd[:, 1] - 0.5e-3) / 0.5e-3 * 100)
gap_dem = np.concatenate(gd)
touch = lambda g: np.mean(g < 1) * 100                                  # 틈 < 지름 1 % = 맞닿음
park = lambda g: np.mean((g > 9) & (g < 11)) * 100                      # IPC barrier(지름 10 %)에 멈춤
law = json.load(open(IPC_LAW))
s_ipc = np.array([r["gap_m"] for r in law["rows"]]) * 1e6
F_ipc = np.array([r["F_N"] for r in law["rows"]])
fcf = np.linalg.norm(ipc["fld_fn"] + ipc["fld_ft"], axis=2).ravel()
fm = np.linalg.norm(ipc["fld_f"], axis=2).ravel()
ok = (fcf > 0) & (fm > 0)
ratio = fcf[ok] / fm[ok]
print(f"touch IPC {touch(gap_ipc):.1f} % DEM {touch(gap_dem):.1f} % | park IPC {park(gap_ipc):.1f} % DEM {park(gap_dem):.1f} %")


def spheres(ax, cx, cy, gap, r=0.36, col=K):
    """두 알 그림 (축 좌표계 아님, 데이터 비율 무관한 별도 축)."""
    ax.add_patch(Circle((cx - r - gap / 2, cy), r, fc="white", ec=col, lw=1.6))
    ax.add_patch(Circle((cx + r + gap / 2, cy), r, fc="white", ec=col, lw=1.6))


def force_panel(a):
    """두 알 사이 틈 [µm] vs 서로 미는 힘 [N] — 선형 축, 틈 > 0 은 '안 닿음'."""
    a.axvspan(-35, 0, color=LG, zorder=0)
    a.axvline(0, color=GR, lw=0.9)
    a.text(-17.5, 7.25, "맞닿아 눌림", ha="center", fontsize=12, color=G)
    a.text(60, 7.25, "떨어져 있음 → 힘은 0 이어야 함", ha="center", fontsize=12, color=K)
    m = F_ipc > 0
    a.plot(s_ipc[m], F_ipc[m], "s-", color=K, mfc="white", ms=5, lw=1.4, label="IPC")
    s = np.linspace(-35, 0, 100)
    a.plot(np.r_[s, 0, 110], np.r_[1e5 * np.abs(s) * 1e-6, 0, 0], color=G, lw=2.4, label="DEM")
    a.annotate("닿기도 전에 밀어냄\n(+81 µm 에서 0.22 N)", xy=(81, 0.22), xytext=(64, 1.7), fontsize=12,
               arrowprops=dict(arrowstyle="->", color=K, lw=1), ha="left")
    a.annotate("세게 밀어도\n27 µm 떨어진 채 6.7 N", xy=(27, 6.67), xytext=(40, 4.9), fontsize=12,
               arrowprops=dict(arrowstyle="->", color=K, lw=1), ha="left")
    a.annotate("닿은 뒤에만 힘", xy=(-20, 2.0), xytext=(-33, 4.2), fontsize=12, color=G,
               arrowprops=dict(arrowstyle="->", color=G, lw=1))
    a.set_xlim(-35, 115)
    a.set_ylim(0, 8)
    a.set_xlabel("두 알 표면 사이 틈 [µm]   (음수 = 겹쳐 눌림)")
    a.set_ylabel("서로 미는 힘 [N]")
    a.legend(loc="upper left", frameon=False, fontsize=12, bbox_to_anchor=(0.19, 0.84), handlelength=1.4)


# ── 4장 Fig. 5 ─────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(10.2, 3.5), gridspec_kw=dict(width_ratios=[0.8, 1.25], wspace=0.3,
                                                                   left=0.075, right=0.99, top=0.86, bottom=0.21))
a = ax[0]
cats = ["가장 가까운 알과\n맞닿음", "100 µm 떨어져\n멈춤"]
vi, vd = [touch(gap_ipc), park(gap_ipc)], [touch(gap_dem), park(gap_dem)]
xx = np.arange(2)
a.bar(xx - 0.2, vi, width=0.38, color="white", edgecolor=K, lw=1.4, label="IPC")
a.bar(xx + 0.2, vd, width=0.38, color=G, label="DEM")
for xv, v in zip(xx - 0.2, vi):
    a.text(xv, v + 2, f"{v:.1f} %", ha="center", fontsize=12)
for xv, v in zip(xx + 0.2, vd):
    a.text(xv, v + 2, f"{v:.1f} %", ha="center", fontsize=12, color=G)
a.set_xticks(xx, cats)
a.set_ylim(0, 130)
a.set_yticks([0, 50, 100])
a.set_ylabel("알 비율 [%]")
a.legend(loc="upper right", frameon=False, fontsize=12)
a.set_title("(a) 알끼리 실제로 닿는가", loc="left", fontsize=14, fontweight="bold")
force_panel(ax[1])
ax[1].set_title("(b) 언제 힘이 생기는가", loc="left", fontsize=14, fontweight="bold")
fig.savefig(os.path.join(A, "fig5_intuitive.png"), dpi=220, facecolor="white")
plt.close(fig)

# ── 2장 IPC 문제 ───────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 2, figsize=(10.2, 4.3), gridspec_kw=dict(width_ratios=[1, 1.1], wspace=0.34,
                                                                   left=0.08, right=0.98, top=0.86, bottom=0.17))
a = ax[0]
bins = np.arange(-1, 13.01, 0.25)
h, e = np.histogram(gap_ipc, bins=bins)
a.bar(e[:-1] + 0.125, h / h.sum() * 100, width=0.25, color=K)
a.axvline(0, color=G, lw=1.6)
a.text(0.3, 76, "맞닿음 = 0", color=G, fontsize=12)
a.annotate(f"{park(gap_ipc):.0f} % 가 지름의 10 %\n(100 µm) 떨어져 멈춤", xy=(10, 60), xytext=(1.6, 40), fontsize=12,
           arrowprops=dict(arrowstyle="->", color=K, lw=1))
a.set_xlim(-1, 13)
a.set_ylim(0, 85)
a.set_xlabel("가장 가까운 알과의 틈 [알 지름 대비 %]")
a.set_ylabel("알 비율 [%]")
a.set_title("(a) 알끼리 닿지 않는다", loc="left", fontsize=14, fontweight="bold")
a = ax[1]
edges = [(0, 0.5, "0.5배 미만"), (0.5, 2, "2배 이내\n(맞음)"), (2, 10, "2–10배"), (10, 100, "10–100배"), (100, np.inf, "100배 이상")]
vals = [np.mean((ratio >= lo) & (ratio < hi)) * 100 for lo, hi, _ in edges]
cols = [GR, G, K, K, K]
a.bar(range(5), vals, color=cols, width=0.65)
for i, v in enumerate(vals):
    a.text(i, v + 1, f"{v:.1f} %", ha="center", fontsize=12, color=G if i == 1 else K)
a.set_xticks(range(5), [t for *_, t in edges], fontsize=11.5)
a.set_ylim(0, 42)
a.set_ylabel("알·프레임 비율 [%]")
a.set_xlabel("보고된 접촉력 ÷ 실제 움직임(m·a)으로 역산한 힘")
a.set_title("(b) 힘이 움직임과 맞지 않는다", loc="left", fontsize=14, fontweight="bold")
fig.savefig(os.path.join(A, "fig_ipc_problem_v2.png"), dpi=220, facecolor="white")
plt.close(fig)
print("wrote fig5_intuitive.png, fig_ipc_problem_v2.png")
