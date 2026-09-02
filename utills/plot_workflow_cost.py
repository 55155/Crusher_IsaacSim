"""공정 단계별 계산 비용 — full_workflow.py 1회 완주 실측 (2026-09-02).

전 공정이 같은 시간비용을 쓰지 않는다. 단계마다 접촉 쌍 수와 FEM 활성 정도가
달라 스텝당 비용이 2배 넘게 벌어진다. 스윕을 설계할 때 어디를 줄여야 하는지가
여기서 갈린다.

(a) 벽시계 총합 — 그 단계가 전체에서 차지하는 몫
(b) 스텝당 비용 — 그 단계가 본질적으로 비싼지. 총합이 큰 것과 다르다:
    압착은 스텝이 많아서 크고, 흡착은 스텝당이 비싸서 크다.

출처: cost_run.log (`[cost]` 행). _tmark 가 단계 시작에서 불려 로그의 이름이 한
칸 밀려 있어, 여기서는 구간에 맞게 바로잡아 적었다.

출력: docs/workflow_cost.png
"""
import os

import matplotlib as mpl
import matplotlib.pyplot as plt

# (구간명, 벽시계 s, 스텝 수)
ROWS = [
    ("파지 → 슬롯 삽입", 143.4, 1930),
    ("클램프",            43.8,  962),
    ("압착 15초",        129.5, 3000),
    ("언클램프 → 투입",  278.6, 4200),
    ("회수장치 잠금",     22.2,  450),
    ("복귀 + 해제",       30.8,  600),
    ("흡착 전진 + 파지", 282.2, 3000),
]

TEAL, GRAY, INK, INK2 = "#1baf7a", "#d6d5d0", "#0b0b0b", "#52514e"
SURFACE = "#ffffff"


def main():
    mpl.rcParams.update({
        "font.family": "Malgun Gothic", "font.size": 9.5,
        "axes.unicode_minus": False, "axes.edgecolor": INK, "axes.linewidth": 0.9,
        "xtick.color": INK, "ytick.color": INK,
    })
    names = [r[0] for r in ROWS]
    secs = [r[1] for r in ROWS]
    steps = [r[2] for r in ROWS]
    mspp = [t / n * 1e3 for t, n in zip(secs, steps)]
    tot = sum(secs)
    y = list(range(len(ROWS)))[::-1]
    hot = max(range(len(ROWS)), key=lambda i: mspp[i])

    fig, (ax, bx) = plt.subplots(
        1, 2, figsize=(9.8, 3.9), dpi=200, gridspec_kw={"width_ratios": [1.5, 1]})
    fig.patch.set_facecolor(SURFACE)

    # (a) 벽시계
    for i, (yy, v) in enumerate(zip(y, secs)):
        ax.barh(yy, v, height=0.62, facecolor=TEAL if i == hot else GRAY,
                edgecolor=INK, linewidth=0.9)
        ax.text(v + 6, yy, f"{v:.0f}s   {v/tot*100:.0f}%", va="center",
                fontsize=8.6, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=9)
    ax.set_xlabel("벽시계 (s)")
    ax.set_xlim(0, 360)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.text(-0.245, 1.09, "(a)", transform=ax.transAxes, fontsize=11,
            fontweight="bold", va="top")

    # (b) 스텝당 비용
    for i, (yy, v, n) in enumerate(zip(y, mspp, steps)):
        bx.barh(yy, v, height=0.62, facecolor=TEAL if i == hot else GRAY,
                edgecolor=INK, linewidth=0.9)
        bx.text(v + 1.8, yy, f"{v:.0f}", va="center", fontsize=8.6, color=INK)
    _avg = tot / sum(steps) * 1e3
    bx.axvline(_avg, color=INK, linewidth=1.0, linestyle=(0, (5, 4)))
    bx.text(_avg + 1.5, -0.95, f"평균 {_avg:.0f}", fontsize=8.4, color=INK)
    bx.set_yticks(y)
    bx.set_yticklabels([])
    bx.set_xlabel("스텝당 비용 (ms)")
    bx.set_xlim(0, 118)
    bx.set_ylim(-1.4, len(ROWS) - 0.3)
    for sp in ("top", "right"):
        bx.spines[sp].set_visible(False)
    bx.text(-0.13, 1.09, "(b)", transform=bx.transAxes, fontsize=11,
            fontweight="bold", va="top")
    ax.set_ylim(-1.4, len(ROWS) - 0.3)

    fig.text(0.055, 0.975, "흡착 구간이 스텝당 가장 비싸다",
             color=INK, fontsize=13, fontweight="bold", va="top")
    fig.text(0.055, 0.918,
             f"전 공정 {tot:.0f}s / {sum(steps):,}스텝.  총합이 큰 것과 스텝당 "
             f"비싼 것은 다르다 — 압착은 스텝이 많아서, 흡착은 스텝당이 비싸서 크다.",
             color=INK2, fontsize=8.8, va="top")

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "workflow_cost.png")
    fig.subplots_adjust(left=0.175, right=0.985, top=0.80, bottom=0.145, wspace=0.06)
    fig.savefig(out, facecolor=SURFACE)
    print(f"[saved] {out}")
    print(f"\n{'구간':22s}{'벽시계':>9}{'스텝':>8}{'ms/스텝':>9}")
    for nm, t, n in ROWS:
        print(f"{nm:22s}{t:>8.1f}s{n:>8d}{t/n*1e3:>9.1f}")
    print(f"{'합계':22s}{tot:>8.1f}s{sum(steps):>8d}{_avg:>9.1f}")


if __name__ == "__main__":
    main()
