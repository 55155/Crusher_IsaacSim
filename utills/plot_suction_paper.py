"""흡착 개구 성능 — 논문용 2패널 그림 (docs/Paper.md 도판 후보).

(a) 흡착 위치별 개구량. 맨 위 빗금 막대는 종전 방식(하드 핀)으로, 구속이 uipc
    솔브에 전달되지 않아 개구가 0 이다 — 실패이지 작은 값이 아니라서 빗금으로
    구분한다.
(b) 전달 효율 follow = 개구량 / 컵 이동량. 컵 이동량이 런마다 85~94mm 로 달라
    (a)의 절대값만으로는 비교가 안 되므로 정규화한 값을 따로 둔다.

출력: docs/suction_paper.png
"""
import os

import matplotlib as mpl
import matplotlib.pyplot as plt

# SuctionV1_only/RESULT/zsweep + probe_video2 실측 (2026-09-02)
RIM    = [5,    15,   25,   40,   60]      # 입구 테두리 아래 흡착 위치(mm)
OPEN   = [72.6, 66.7, 69.0, 63.0, 47.4]    # 개구량(mm)
FOLLOW = [0.769, 0.785, 0.746, 0.717, 0.529]
HARD_OPEN, HARD_FOLLOW = 0.0, 0.0          # 종전 하드 핀 — 구속 미작동

TEAL, GRAY, INK = "#1baf7a", "#d6d5d0", "#0b0b0b"
SURFACE = "#ffffff"


def main():
    mpl.rcParams.update({
        "font.family": "Malgun Gothic", "font.size": 9.5,
        "axes.unicode_minus": False, "axes.edgecolor": INK, "axes.linewidth": 0.9,
        "xtick.color": INK, "ytick.color": INK,
        "xtick.major.width": 0.9, "ytick.major.width": 0.9,
    })
    fig, (ax, bx) = plt.subplots(
        1, 2, figsize=(9.4, 3.6), dpi=200, gridspec_kw={"width_ratios": [2.15, 1]})
    fig.patch.set_facecolor(SURFACE)

    # ── (a) 흡착 위치별 개구량 ──────────────────────────────────────────────
    labels = ["종전 방식\n(하드 핀)"] + [f"흡착 {r}mm" for r in RIM]
    vals = [HARD_OPEN] + OPEN
    ypos = list(range(len(vals)))[::-1]

    ax.barh(ypos[0], 1.0, height=0.62, facecolor="none", edgecolor=INK,
            linewidth=0.9, hatch="///")
    for y, v, r in zip(ypos[1:], OPEN, RIM):
        best = (r == 15)
        ax.barh(y, v, height=0.62, facecolor=TEAL if best else GRAY,
                edgecolor=INK, linewidth=0.9)
    ax.text(3.0, ypos[0], "0 mm  (구속이 솔버에 전달되지 않음)", va="center",
            fontsize=8.6, color=INK)
    for y, v, f in zip(ypos[1:], OPEN, FOLLOW):
        ax.text(v + 1.6, y, f"{v:.1f} mm", va="center", fontsize=8.6, color=INK)

    # 개선 폭 화살표
    # 개선 폭은 막대 아래 빈 공간에 둔다(막대를 덮지 않게)
    ax.annotate("", xy=(72.6, -0.75), xytext=(0.6, -0.75),
                arrowprops=dict(arrowstyle="<->", color=INK, linewidth=0.9))
    ax.text(36.6, -1.20, "0 → 72.6 mm", ha="center", fontsize=8.6, color=INK)

    ax.set_yticks(ypos)
    ax.set_yticklabels(labels, fontsize=8.8)
    ax.set_xlabel("봉투 입구 개구량 (mm)")
    ax.set_xlim(0, 92)
    ax.set_ylim(-1.5, len(vals) - 0.3)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    # 범례는 두지 않는다 — 빗금 막대 옆 문구가 이미 그 역할을 한다
    ax.text(-0.235, 1.10, "(a)", transform=ax.transAxes, fontsize=11,
            fontweight="bold", va="top")

    # ── (b) 전달 효율 ───────────────────────────────────────────────────────
    bx.bar(0, HARD_FOLLOW, width=0.55, facecolor=GRAY, edgecolor=INK, linewidth=0.9)
    bx.bar(1, FOLLOW[1], width=0.55, facecolor=TEAL, edgecolor=INK, linewidth=0.9)
    bx.text(0, 0.022, "0.00", ha="center", fontsize=8.8, color=INK)
    bx.text(1, FOLLOW[1] + 0.022, f"{FOLLOW[1]:.2f}", ha="center", fontsize=8.8,
            color=INK)
    bx.set_xticks([0, 1])
    bx.set_xticklabels(["하드 핀", "SPC 배선\n(흡착 15mm)"], fontsize=8.8)
    bx.set_ylabel("전달 효율  개구량 / 컵 이동량")
    bx.set_ylim(0, 1.0)
    bx.set_xlim(-0.65, 1.65)
    for s in ("top", "right"):
        bx.spines[s].set_visible(False)
    bx.text(-0.40, 1.10, "(b)", transform=bx.transAxes, fontsize=11,
            fontweight="bold", va="top")

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "suction_paper.png")
    fig.subplots_adjust(left=0.155, right=0.975, top=0.88, bottom=0.20, wspace=0.44)
    fig.savefig(out, facecolor=SURFACE)
    print(f"[saved] {out}")


if __name__ == "__main__":
    main()
