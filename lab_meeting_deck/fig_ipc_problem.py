"""IPC 점질량 런(RESULT_cfprofile0922)의 두 문제를 그림으로: 낟알 간격이 배리어에 붙음 / 접촉력이 운동량 수지와 안 맞음.
make_paper_figs.py 와 같은 스타일 (검정+녹색, serif, 안쪽 눈금, 영어)."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RUN = r"C:\Crusher_isaacsim\Crusher_Genesis\Crusher_M0609_RG2_Tablet_Samplebag\RESULT_cfprofile0922\grip-28mm\crush_grip-28mm_yoff+0.0mm_20260922_131907.npz"
K, G, GR = "#111111", "#1b7f3b", "#8a8a8a"
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix", "font.size": 16, "axes.labelsize": 16,
    "legend.fontsize": 15, "xtick.labelsize": 14, "ytick.labelsize": 14,
    "xtick.direction": "in", "ytick.direction": "in", "xtick.top": True, "ytick.right": True,
    "axes.linewidth": 0.8, "lines.linewidth": 1.3, "legend.frameon": False,
})

d = np.load(RUN)
R, dh = float(d["grain_radius"]), float(d["d_hat"])
bar = 2 * R + dh
nn = d["fld_nn"].astype(np.float64)
gap = (nn.ravel() - 2 * R) / (2 * R)
at_bar = np.mean(np.abs(nn / bar - 1) < 0.01)
per_frame = np.mean(np.abs(nn / bar - 1) < 0.01, axis=1)

fcf = np.linalg.norm(d["fld_fn"] + d["fld_ft"], axis=2).ravel()
fm = np.linalg.norm(d["fld_f"], axis=2).ravel()
ok = (fcf > 0) & (fm > 0)
ratio = fcf[ok] / fm[ok]
med = np.median(ratio)
w2 = np.mean((ratio > 0.5) & (ratio < 2))
print(f"at barrier ±1%: pooled {at_bar*100:.1f}%, frame range {per_frame.min()*100:.1f}-{per_frame.max()*100:.1f}%")
print(f"ratio median {med:.2f}, within 2x {w2*100:.1f}%, n={ok.sum()}")

fig, ax = plt.subplots(1, 2, figsize=(10.0, 4.2), gridspec_kw=dict(wspace=0.3, left=0.085, right=0.985, top=0.9, bottom=0.2))
a = ax[0]
bins = np.linspace(-0.02, 0.12, 141)
a.hist(gap, bins=bins, histtype="stepfilled", color=K, alpha=0.85, density=True)
a.axvline(dh / (2 * R), color=G, ls="--", lw=1.4)
a.text(0.042, 0.55, f"IPC barrier $\\hat d$\n{at_bar*100:.0f} % of grains\nwithin ±1 %", color=G,
       fontsize=15, ha="center", va="bottom", transform=a.get_xaxis_transform())
a.axvline(0, color=GR, lw=0.8)
a.text(0.002, 0.95, "contact", color=GR, fontsize=14, transform=a.get_xaxis_transform(), va="top")
a.set_yscale("log")
a.set_xlabel("Nearest-neighbour surface gap / grain diameter")
a.set_ylabel("Probability density")
a.text(-0.14, 1.03, "(a)", transform=a.transAxes, fontsize=18, fontweight="bold")

a = ax[1]
lb = np.logspace(-1, 5, 121)
a.hist(ratio, bins=lb, histtype="stepfilled", color=K, alpha=0.85)
a.axvline(1, color=G, ls="--", lw=1.4)
a.text(0.88, 0.95, "ratio\n= 1", color=G, fontsize=15, ha="right", transform=a.get_xaxis_transform(), va="top")
a.axvline(med, color=K, lw=1.4)
a.text(med * 1.15, 0.95, f"median ×{med:.1f}", color=K, fontsize=15, transform=a.get_xaxis_transform(), va="top")
a.text(0.97, 0.62, f"within 2×: {w2*100:.1f} %", ha="right", fontsize=15, transform=a.transAxes)
a.set_xscale("log")
a.set_xlabel(r"|contact force| / |momentum-balance force|")
a.set_ylabel("Count")
a.text(-0.14, 1.03, "(b)", transform=a.transAxes, fontsize=18, fontweight="bold")

out = sys.argv[1]
fig.savefig(out, dpi=220, facecolor="white")
print("wrote", out)
