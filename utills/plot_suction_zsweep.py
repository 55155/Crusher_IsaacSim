"""흡착 위치별 봉투 입구 개구량 — SuctionV1_only/RESULT/zsweep 5런 실측.

봉투는 높이 90mm 이고 입구 테두리가 위쪽 끝(로컬 z=+45mm)에 있다. 흡착컵이 그
테두리에서 얼마나 아래를 무는지(x축)에 따라 입구가 얼마나 벌어지는지(y축) 본다.

두 계열 모두 단위가 mm 라 축 하나에 같이 올린다(컵 이동 = 입력 상한, 개구 =
달성치). 둘의 간격이 곧 손실이고, 그 비가 follow 다.

출력: docs/suction_zsweep.png
"""
import os

import matplotlib as mpl
import matplotlib.pyplot as plt

# SuctionV1_only/RESULT/zsweep/rim<NN>mm_K1e4.log 에서 뽑은 값 (2026-09-02)
#   rim  = 입구 테두리 아래 흡착 위치(mm)
#   move = 컵 이동량(mm), open_ = 개구량(mm), follow = open_/move
RIM    = [5,    15,   25,   40,   60]
MOVE   = [94.0, 85.0, 92.0, 88.0, 90.0]
OPEN   = [72.6, 66.7, 69.0, 63.0, 47.4]
FOLLOW = [0.769, 0.785, 0.746, 0.717, 0.529]

# dataviz 기본 팔레트 슬롯 1-2 (인접쌍 인증: CVD dE 9.1 / 일반시야 19.6, light)
C_OPEN, C_MOVE = "#2a78d6", "#eb6834"
INK, INK2, INK3 = "#0b0b0b", "#52514e", "#8a8985"
SURFACE = "#fcfcfb"


def main():
    mpl.rcParams.update({
        "font.family": "Malgun Gothic", "font.size": 10, "axes.unicode_minus": False,
        "axes.edgecolor": INK3, "axes.linewidth": 0.8,
        "xtick.color": INK2, "ytick.color": INK2,
        "xtick.direction": "out", "ytick.direction": "out",
    })
    fig, ax = plt.subplots(figsize=(7.6, 4.6), dpi=170)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    ax.grid(axis="y", color=INK3, alpha=0.22, linewidth=0.7)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)

    # 봉투 중간 높이(45mm) — 이 너머는 입구가 아니라 몸통을 무는 셈이다
    ax.axvline(45, color=INK3, linewidth=0.9, linestyle=(0, (4, 3)), alpha=0.7)
    ax.text(45.8, 101, "봉투 중간 45mm", color=INK3, fontsize=8.5, va="top")

    ax.plot(RIM, MOVE, color=C_MOVE, linewidth=2, marker="o", markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2, zorder=3)
    ax.plot(RIM, OPEN, color=C_OPEN, linewidth=2, marker="o", markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=2, zorder=4)

    # 직접 라벨(범례와 중복이지만 색만으로 구분되지 않게 한다)
    ax.text(RIM[-1] + 1.6, MOVE[-1], "컵 이동", color=C_MOVE, fontsize=10,
            va="center", fontweight="bold")
    ax.text(RIM[-1] + 1.6, OPEN[-1], "봉투 개구", color=C_OPEN, fontsize=10,
            va="center", fontweight="bold")

    # 값 라벨은 전부 찍지 않는다 — 최고/최저만
    for i, note in ((1, f"최고  follow {FOLLOW[1]:.2f}"),
                    (4, f"최저  follow {FOLLOW[4]:.2f}")):
        ax.annotate(f"{OPEN[i]:.1f}mm\n{note}", (RIM[i], OPEN[i]),
                    textcoords="offset points", xytext=(0, -34),
                    ha="center", fontsize=8.5, color=INK2, linespacing=1.4)

    ax.set_xlabel("흡착 위치 — 입구 테두리 아래 (mm)", color=INK2, labelpad=8)
    ax.set_ylabel("mm", color=INK2, labelpad=6)
    ax.set_xlim(-2, 74)
    ax.set_ylim(0, 104)
    ax.set_xticks(RIM)
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    fig.text(0.09, 0.965, "입구에 가까이 물수록 잘 벌어진다",
             color=INK, fontsize=13.5, fontweight="bold", va="top")
    _sub = ("흡착컵이 입구 테두리에서 멀어질수록 개구량이 줄고,"
            "\n봉투 중간(45mm)을 넘어가면 급격히 떨어진다.")
    fig.text(0.09, 0.905, _sub, color=INK2, fontsize=9, va="top", linespacing=1.5)
    fig.text(0.99, 0.965, "SuctionV1_only  5런\nstrength_rate = 1e4",
             color=INK3, fontsize=8.5, va="top", ha="right", linespacing=1.5)

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "suction_zsweep.png")
    fig.subplots_adjust(left=0.09, right=0.86, top=0.75, bottom=0.14)
    fig.savefig(out, facecolor=SURFACE)
    print(f"[saved] {out}")

    print(f"\n{'흡착위치':>10}{'컵 이동':>10}{'개구':>9}{'follow':>9}")
    for r, m, o, f in zip(RIM, MOVE, OPEN, FOLLOW):
        print(f"{r:>8}mm{m:>8.0f}mm{o:>7.1f}mm{f:>9.3f}")


if __name__ == "__main__":
    main()
