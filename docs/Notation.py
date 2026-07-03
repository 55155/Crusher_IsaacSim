"""
Notation.py — 정제(tablet) 인장강도 σ_t 공식을 LaTeX(mathtext)로 렌더해 PNG 저장.

두 공식:
  1. Fell–Newton (평면 정제)   : σ_t = 2F / (πDt)
     · Fell & Newton (1970), J. Pharm. Sci. 59(5), 688–691
  2. Pitt (볼록면 정제)         : σ_t = 10F/(πD²)·[2.84 t/D − 0.126 t/W + 3.15 W/D + 0.01]⁻¹
     · Pitt, Newton, Stanley (1988), J. Mater. Sci. 23, 2723–2728

TeX 배포판 없이 matplotlib mathtext 로 렌더 → 투명배경 300 dpi PNG.
출력: 이 스크립트와 같은 폴더에 <name>.png 저장.
"""
import os
import matplotlib.pyplot as plt

_DIR = os.path.dirname(os.path.abspath(__file__))

# 수식 정의 (LaTeX/mathtext 문법)
EQS = {
    "sigma_t_fellnewton": r"$\sigma_t = \dfrac{2F}{\pi D t}$",
    "sigma_t_pitt": (
        r"$\sigma_t = \dfrac{10F}{\pi D^{2}}"
        r"\left(2.84\dfrac{t}{D} - 0.126\dfrac{t}{W}"
        r"+ 3.15\dfrac{W}{D} + 0.01\right)^{-1}$"
    ),
    # 에너지형 손상 / 누적일 (W_acc = W_누적; mathtext 는 한글 첨자 불가)
    "damage_work": (
        r"$D = \dfrac{W_{\mathrm{acc}}}{W^{*}}, \quad "
        r"W_{\mathrm{acc}} = \sum_{i}\int F\,v\,dt$"
    ),
    # Weibull 파괴확률 — 부피적분형 (약한 고리; σ_I 장 전체, σ_I>0 인장부만)
    "weibull_pf": (
        r"$P_f = 1 - \exp\!\left[-\int_V \left("
        r"\dfrac{\sigma_I(x)}{\sigma_0}\right)^{m}\dfrac{dV}{V_0}\right]"
        r"\ \ (\sigma_I>0)$"
    ),
    # Weibull FEM 이산형 (tet 합산; risk of rupture)
    "weibull_fem_sum": (
        r"$P_f = 1 - \exp\!\left[-\sum_{e}\left("
        r"\dfrac{\sigma_{I,e}}{\sigma_0}\right)^{m}\dfrac{V_e}{V_0}\right]"
        r"\ \ (\sigma_{I,e}>0)$"
    ),
    # Weibull 유효부피 분리형 (peak σ_I 와 부피효과 분리)
    "weibull_veff": (
        r"$P_f = 1 - \exp\!\left[-\left(\dfrac{\sigma_{I,\max}}{\sigma_0}"
        r"\right)^{m}\dfrac{V_{\mathrm{eff}}}{V_0}\right],\ \ "
        r"V_{\mathrm{eff}} = \int_V\!\left(\dfrac{\sigma_I(x)}{\sigma_{I,\max}}"
        r"\right)^{m} dV$"
    ),
    # Weibull 선형화 — 30개 파괴하중으로 m·σ0 fit (median rank)
    "weibull_fit": (
        r"$\ln\ln\dfrac{1}{1-P_f} = m\,\ln\sigma - m\,\ln\sigma_0,"
        r"\quad P_{f,i} = \dfrac{i-0.5}{N}$"
    ),
    # Regime II 누적 파괴확률 (손상 반영, 일반형) — p_k 는 매 타격 Weibull CDF
    "pf_cumulative": (
        r"$P_f(N) = 1 - \prod_{k=1}^{N}\left(1 - p_k\right),"
        r"\quad p_k = 1 - \exp\!\left[-\left("
        r"\dfrac{\sigma_{I,k}}{\sigma_0}\right)^{m}\right]$"
    ),
    # Regime II 누적 파괴확률 (p 일정, 독립 시행 단순형)
    "pf_independent": r"$P_f(N) = 1 - (1 - p)^{N}$",
    # 커플러 운동방정식 — Rigid / FEM, 공통항 = 접촉력 λ(외력)
    "coupler_rigid": r"$M_r(q)\,a_r = f_r + J_a^{\top}\lambda$",
    "coupler_fem": r"$M_f\,a_f = f_f + J_b^{\top}\lambda$",
}

# 각 수식에 붙일 캡션 (파일에는 미포함, 참고용)
CAPTIONS = {
    "sigma_t_fellnewton": "Fell-Newton (flat-faced tablet)",
    "sigma_t_pitt": "Pitt (convex-faced tablet)",
    "damage_work": "Energy-based damage / cumulative work",
    "weibull_pf": "Weibull failure probability (volume integral)",
    "weibull_fem_sum": "Weibull FEM discrete (tet sum)",
    "weibull_veff": "Weibull effective-volume form",
    "weibull_fit": "Weibull linearization (30-sample fit)",
    "pf_cumulative": "Regime II cumulative P_f(N) (damage)",
    "pf_independent": "Regime II cumulative P_f(N) (constant p)",
    "coupler_rigid": "Coupler EoM - Rigid",
    "coupler_fem": "Coupler EoM - FEM",
}


def render(name: str, tex: str, width: float = 7.0, height: float = 1.4,
           fontsize: int = 22, dpi: int = 300) -> str:
    fig = plt.figure(figsize=(width, height))
    fig.text(0.5, 0.5, tex, ha="center", va="center", fontsize=fontsize)
    out = os.path.join(_DIR, f"{name}.png")
    fig.savefig(out, dpi=dpi, bbox_inches="tight", transparent=True)
    plt.close(fig)
    return out


def main():
    for name, tex in EQS.items():
        path = render(name, tex)
        print(f"[saved] {CAPTIONS.get(name, name):<32} -> {path}")


if __name__ == "__main__":
    main()
