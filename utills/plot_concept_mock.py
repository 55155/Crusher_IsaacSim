"""논문 메인 컨셉 도판 **초안(가상 데이터)** — 강체 티칭 가정이 공정 단계마다 깨진다.

주장: 강체는 잡은 곳을 가르치면 물체 전체를 가르친 것이 되지만, 변형체는 잡은
곳만 가르쳐진다. 그 하나의 원인이 공정 단계마다 다른 얼굴로 나타난다.

단계마다 재는 양이 다르므로(파지=실링 물림, 이송=자세각, 투입=수평오차 ...)
각 단계의 **설계 허용치 대비 %** 로 정규화해 축 하나에 올린다. 100% 가 허용 한계다.

마지막 두 단계는 강체 모델에 **값이 존재하지 않는다** — 입구 개구와 정제 분쇄는
변형 그 자체라 강체 봉투로는 질문을 던질 수조차 없다. 그래서 강체 계열은 거기서
끊기고 빗금 표식만 남는다.

*** 이 스크립트의 수치는 전부 임의값이다. 도판 구조를 보기 위한 초안이며 ***
*** 실측으로 교체하기 전까지 논문/발표에 쓰면 안 된다.                    ***

출력: docs/concept_mock.png
"""
import os

import matplotlib as mpl
import matplotlib.pyplot as plt

STAGES = ["파지", "이송", "투입", "개구", "압착"]
CAUSE = ["잡은 곳은 맞는데\n물릴 곳이 어긋남",
         "가는 동안\n형상이 바뀜",
         "같은 동작이\n같은 결과를 안 냄",
         "목적 자체가\n변형",
         "목적 자체가\n변형"]

# ── 전부 임의값 (구조 확인용) ────────────────────────────────────────────────
RIGID = [12, 16, 21, None, None]      # 강체 가정 — 개구/압착은 값이 없음
DEFORM = [38, 92, 155, 205, 240]      # 변형체 실측(가상)

TEAL, GRAY, INK, INK2 = "#1baf7a", "#9c9b96", "#0b0b0b", "#52514e"
SURFACE = "#ffffff"


def main():
    mpl.rcParams.update({
        "font.family": "Malgun Gothic", "font.size": 9.5,
        "axes.unicode_minus": False, "axes.edgecolor": INK, "axes.linewidth": 0.9,
        "xtick.color": INK, "ytick.color": INK,
    })
    fig, ax = plt.subplots(figsize=(8.2, 4.8), dpi=200)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    x = list(range(len(STAGES)))

    # 허용 한계
    ax.axhline(100, color=INK, linewidth=1.0, linestyle=(0, (5, 4)))
    ax.text(4.42, 106, "설계 허용 한계", fontsize=8.6, color=INK, ha="right")

    # 강체 모델에 값이 없는 구간
    ax.axvspan(2.5, 4.5, facecolor="#f2f1ec", edgecolor="none", zorder=0)
    ax.text(3.5, 250, "강체 모델에는\n이 양이 존재하지 않는다", ha="center",
            fontsize=9, color=INK, linespacing=1.5, fontweight="bold")

    xr = [i for i, v in enumerate(RIGID) if v is not None]
    yr = [v for v in RIGID if v is not None]
    ax.plot(xr, yr, color=GRAY, linewidth=2, marker="s", markersize=8,
            markeredgecolor=SURFACE, markeredgewidth=1.6, zorder=3)
    ax.plot(x, DEFORM, color=TEAL, linewidth=2, marker="o", markersize=9,
            markeredgecolor=SURFACE, markeredgewidth=1.6, zorder=4)

    # 강체 계열이 끊기는 지점
    for i in (3, 4):
        ax.plot(i, 0, marker="x", color=GRAY, markersize=10, markeredgewidth=2,
                zorder=3, clip_on=False)
    ax.text(2.62, 26, "강체 계열은 여기서 끊긴다", fontsize=8.4, color=INK2)

    ax.text(2.08, yr[-1] + 12, "강체 가정", color=INK2, fontsize=10,
            fontweight="bold", ha="right")
    ax.text(4.0, DEFORM[-1] - 34, "변형체 실측", color=TEAL, fontsize=10,
            fontweight="bold", ha="center")

    ax.set_xticks(x)
    ax.set_xticklabels(STAGES, fontsize=11)
    for i, c in enumerate(CAUSE[:3]):
        ax.text(i, -46, c, ha="center", fontsize=8.2, color=INK2, linespacing=1.4)
    ax.text(3.5, -46, "목적 자체가 변형", ha="center", fontsize=8.2, color=INK2)

    ax.set_ylabel("설계 허용치 대비 편차 (%)", color=INK2)
    ax.set_ylim(0, 285)
    ax.set_xlim(-0.5, 4.5)
    ax.grid(axis="y", color="#c9c8c3", alpha=0.5, linewidth=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    fig.text(0.075, 0.975, "한 원인이 공정 단계마다 다른 얼굴로 나타난다",
             color=INK, fontsize=13, fontweight="bold", va="top")
    fig.text(0.075, 0.918,
             "강체는 잡은 곳을 가르치면 물체 전체를 가르친 것이 된다. "
             "변형체는 잡은 곳만 가르쳐진다.",
             color=INK2, fontsize=9.2, va="top")
    fig.text(0.985, 0.975, "초안 — 가상 데이터\n실측으로 교체 필요",
             color="#b4342f", fontsize=8.6, va="top", ha="right", linespacing=1.5)

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "concept_mock.png")
    fig.subplots_adjust(left=0.105, right=0.985, top=0.845, bottom=0.215)
    fig.savefig(out, facecolor=SURFACE)
    print(f"[saved] {out}")
    print("*** 수치는 전부 임의값이다 — 구조 확인용 초안 ***")


if __name__ == "__main__":
    main()
