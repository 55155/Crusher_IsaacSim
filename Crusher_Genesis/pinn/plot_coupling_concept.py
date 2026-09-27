"""plot_coupling_concept.py — "PBM 은 장부, 결합 지점은 S 하나" 개념도 (docs/PINN.md §2-1)

규칙: 상자 3개 + 화살표 2개, 회색 + 강조색 1개(2→3 화살표와 S 에만), 수식 없음.
출력: docs/pinn_coupling_concept.svg / .png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle

DOCS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs")
ACC = "#0F7C7A"                                   # 강조색 — 짙은 청록
INK, MID, LIGHT, EDGE = "#222222", "#6B6B6B", "#D9D9D9", "#BDBDBD"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})

fig = plt.figure(figsize=(12, 4))
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 12); ax.set_ylim(0, 4); ax.axis("off")

BW, BH, BY = 2.9, 2.35, 1.05                      # 상자 폭·높이·바닥
BX = [0.2, 4.1, 8.0]


def box(x, title, sub, tag):
    ax.add_patch(FancyBboxPatch((x, BY), BW, BH, boxstyle="round,pad=0.02,rounding_size=0.12",
                                fc="white", ec=EDGE, lw=1.0))
    ax.text(x + BW / 2, BY + BH + 0.28, title, ha="center", va="center", fontsize=13,
            color=INK, fontweight="bold")
    ax.text(x + BW / 2, BY - 0.25, sub, ha="center", va="center", fontsize=11, color=INK)
    ax.text(x + BW / 2, BY - 0.62, tag, ha="center", va="center", fontsize=9.5, color=MID)


# 1) Simulator — 에너지를 받은 알 더미 (대부분 연하고 셋만 진하다)
box(BX[0], "How force spreads", "energy per grain", "mechanics")
rng = np.random.default_rng(3)
pts = [(0.62, 0.35), (1.12, 0.33), (1.62, 0.36), (2.12, 0.34), (0.87, 0.78), (1.37, 0.80),
       (1.87, 0.77), (2.30, 0.80), (0.62, 1.22), (1.12, 1.24), (1.62, 1.21), (2.10, 1.25),
       (0.88, 1.66), (1.38, 1.68), (1.88, 1.65)]
hot = {5, 6, 10}                                   # 힘사슬 위의 알
for k, (px, py) in enumerate(pts):
    shade = "#4A4A4A" if k in hot else (LIGHT if rng.random() < 0.7 else "#A8A8A8")
    ax.add_patch(Circle((BX[0] + px + 0.03, BY + 0.2 + py), 0.21, fc=shade, ec="white", lw=1.2))

# 2) PINN — 파쇄확률 S자 곡선
box(BX[1], "How a grain breaks", "P(x, e)", "learned")
x0, y0, w, h = BX[1] + 0.45, BY + 0.35, BW - 0.9, BH - 0.75
ax.plot([x0, x0 + w], [y0, y0], color=MID, lw=0.9)
ax.plot([x0, x0], [y0, y0 + h], color=MID, lw=0.9)
e = np.linspace(0, 1, 100)
ax.plot(x0 + e * w, y0 + h * 0.95 / (1 + np.exp(-11 * (e - 0.55))), color=INK, lw=2.2)
ax.text(x0 + w, y0 - 0.17, "energy", ha="right", va="center", fontsize=9, color=MID)
ax.text(x0 - 0.1, y0 + h, "break", ha="right", va="top", fontsize=9, color=MID, rotation=90)

# 3) PBM — 크기 순 막대, 큰 것에서 작은 것으로
box(BX[2], "Where the mass goes", "mass per size class", "bookkeeping")
hs = [1.35, 1.05, 0.80, 0.58, 0.40]
bx0 = BX[2] + 0.42
for k, hh in enumerate(hs):
    ax.add_patch(Rectangle((bx0 + k * 0.44, BY + 0.3), 0.3, hh, fc=LIGHT if k else "#A8A8A8",
                           ec="none"))
for k in range(len(hs) - 1):
    ax.add_patch(FancyArrowPatch((bx0 + k * 0.44 + 0.15, BY + 0.3 + hs[k] + 0.12),
                                 (bx0 + (k + 1) * 0.44 + 0.15, BY + 0.3 + hs[k + 1] + 0.12),
                                 connectionstyle="arc3,rad=-0.45", arrowstyle="-|>",
                                 mutation_scale=9, lw=0.9, color=MID))

# 화살표 — 1→2 회색, 2→3 강조
ym = BY + BH / 2
ax.add_patch(FancyArrowPatch((BX[0] + BW + 0.12, ym), (BX[1] - 0.12, ym), arrowstyle="-|>",
                             mutation_scale=16, lw=1.4, color=MID))
ax.text((BX[0] + BW + BX[1]) / 2, ym + 0.22, "e_k", ha="center", fontsize=11, color=MID,
        style="italic")
ax.add_patch(FancyArrowPatch((BX[1] + BW + 0.12, ym), (BX[2] - 0.12, ym), arrowstyle="-|>",
                             mutation_scale=18, lw=2.6, color=ACC))
ax.text((BX[1] + BW + BX[2]) / 2, ym + 0.24, "S", ha="center", fontsize=15, color=ACC,
        fontweight="bold")
ax.text((BX[1] + BW + BX[2]) / 2, ym - 0.30, "the only\ncoupling point", ha="center",
        va="top", fontsize=9, color=ACC)

# PSD
ax.add_patch(FancyArrowPatch((BX[2] + BW + 0.08, ym), (BX[2] + BW + 0.45, ym), arrowstyle="-|>",
                             mutation_scale=12, lw=1.2, color=MID))
ax.text(BX[2] + BW + 0.52, ym, "PSD", ha="left", va="center", fontsize=11, color=INK)

for ext in ("svg", "png"):
    p = os.path.abspath(os.path.join(DOCS, f"pinn_coupling_concept.{ext}"))
    fig.savefig(p, dpi=200)
    print(p)
