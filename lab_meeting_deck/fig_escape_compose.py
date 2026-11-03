"""fig_escape_render.py 가 뽑은 Newton 렌더 4장을 한 장으로 — 샘플백에서 공극률을 못 쓰는 이유.
base anaconda (matplotlib, PIL) 로 실행. 출력 assets/fig_escape.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "assets")
for f in ("Pretendard-Regular.otf", "Pretendard-Bold.otf"):
    font_manager.fontManager.addfont(os.path.join(A, f))
plt.rcParams.update({"font.family": "Pretendard", "font.size": 15})

# 판 투영 기준 수치 (dem_frames.npz, 2026-11-03 계산 — 시작 9 ms / 끝 1290 ms)
N0, N_OUT, N_STAY = 5166, 1404, 3758
crops = {"front": (470, 150, 1390, 960), "side": (700, 220, 1260, 1030)}
im = {(v, i): np.asarray(Image.open(os.path.join(A, f"escape_raw_{v}_{i}.png")).convert("RGB").crop(crops[v]))
      for v in crops for i in (0, 148)}

fig = plt.figure(figsize=(16, 7.4), dpi=150)
gs = fig.add_gridspec(1, 4, width_ratios=[920, 920, 560, 560], wspace=0.04, left=0.005, right=0.995, top=0.86, bottom=0.12)
# 글자는 최소한 — 시작/끝 표지와 색 두 개만 (2026-11-03 요청)
for k, key in enumerate([("front", 0), ("front", 148), ("side", 0), ("side", 148)]):
    ax = fig.add_subplot(gs[0, k])
    ax.imshow(im[key])
    ax.set_axis_off()
    ax.set_title("시작" if key[1] == 0 else "끝", fontsize=18, fontweight="bold")
fig.legend(handles=[Patch(color=(0.11, 0.50, 0.23), label="판 아래에 남은 알"),
                    Patch(color=(0.93, 0.47, 0.05), label="판 밖으로 밀려난 알")],
           loc="lower center", ncol=2, frameon=False, fontsize=16, bbox_to_anchor=(0.5, 0.06))
out = os.path.join(A, "fig_escape.png")
fig.savefig(out, facecolor="white", bbox_inches="tight", pad_inches=0.15)
print("wrote", out)
