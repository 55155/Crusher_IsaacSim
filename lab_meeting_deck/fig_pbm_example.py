"""참고자료 PBM 슬라이드: F_{n+1} = (I - S + BS) F_n 을 3칸 예시로 (수치는 설명용).
  fig_pbm_example.png — (a) 한 스트로크의 행렬곱, (b) 스트로크별 크기 칸 질량 분율"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "assets")
for f in ("Pretendard-Regular.otf", "Pretendard-Bold.otf"):
    font_manager.fontManager.addfont(os.path.join(A, f))
plt.rcParams.update({"font.family": "Pretendard", "font.size": 14, "axes.spines.top": False, "axes.spines.right": False,
                     "mathtext.fontset": "stix"})
K, G, GR, LG = "#262626", "#1b7f3b", "#9a9a9a", "#d9d9d9"
S = np.diag([0.4, 0.2, 0.0])
B = np.array([[0, 0, 0], [0.7, 0, 0], [0.3, 1, 0]])
M = np.eye(3) - S + B @ S
F = [np.array([100.0, 0, 0])]
for _ in range(2):
    F.append(M @ F[-1])
print(M, F)
LABEL = ["1000 µm", "500 µm", "250 µm"]


def bracket(a, x0, x1, y0, y1):
    d = 0.012
    a.plot([x0 + d, x0, x0, x0 + d], [y0, y0, y1, y1], color=K, lw=1.4, clip_on=False)
    a.plot([x1 - d, x1, x1, x1 - d], [y0, y0, y1, y1], color=K, lw=1.4, clip_on=False)


fig, ax = plt.subplots(1, 2, figsize=(11.5, 3.5), gridspec_kw=dict(width_ratios=[1.35, 1], wspace=0.14, left=0.01, right=0.99, top=0.86, bottom=0.14))
a = ax[0]
a.axis("off"); a.set_xlim(0, 1); a.set_ylim(0, 1)
a.set_title("(a) 한 스트로크의 행렬곱", loc="left", fontweight="bold", fontsize=15)
ys = [0.74, 0.5, 0.26]
for y, l in zip(ys, LABEL):
    a.text(0.11, y, l, ha="right", va="center", color=GR, fontsize=12)


def column(a, x, vals, fmt, width, head):
    for y, v in zip(ys, vals):
        a.text(x, y, fmt.format(v), ha="center", va="center", color=K)
    bracket(a, x - width, x + width, 0.15, 0.85)
    a.text(x, 0.95, head, ha="center", va="center", color=K, fontsize=13)


column(a, 0.2, F[1], "{:.0f}", 0.05, "$F_1$")
a.text(0.295, 0.5, "=", ha="center", va="center", fontsize=17)
for j, x in enumerate([0.4, 0.51, 0.62]):
    for y, v in zip(ys, M[:, j]):
        col = G if (v > 0 and not np.isclose(v, M[list(ys).index(y), list(ys).index(y)])) else K
        a.text(x, y, f"{v:.2f}".rstrip("0").rstrip(".") if v else "0", ha="center", va="center", color=col)
bracket(a, 0.345, 0.675, 0.15, 0.85)
a.text(0.51, 0.95, "$I - S + BS$", ha="center", va="center", fontsize=13)
column(a, 0.78, F[0], "{:.0f}", 0.05, "$F_0$")
a.text(0.51, 0.04, "대각선 = 안 깨지고 남는 몫   ·   초록 = 더 작은 칸으로 옮겨가는 몫", ha="center", va="center", color=GR, fontsize=11.5)

a = ax[1]
x = np.arange(3); w = 0.26
for k, c in zip(range(3), (LG, GR, G)):
    b = a.bar(x + (k - 1) * w, F[k], w, color=c, label=f"스트로크 {k}")
    for r, v in zip(b, F[k]):
        a.text(r.get_x() + r.get_width() / 2, v + 2, f"{v:.0f}", ha="center", fontsize=10.5, color=K)
a.set_xticks(x); a.set_xticklabels(LABEL)
a.set_ylim(0, 112); a.set_yticks([0, 50, 100]); a.set_ylabel("질량 분율 [%]")
a.legend(frameon=False, fontsize=12, loc="upper right")
a.set_title("(b) 큰 입자의 질량이 작은 칸으로 이동", loc="left", fontweight="bold", fontsize=15)
fig.savefig(os.path.join(A, "fig_pbm_example.png"), dpi=220, facecolor="white")
