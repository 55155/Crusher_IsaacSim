"""probe_repeat_spread.py — **같은 설정 N회**의 산포를 표와 그림으로 낸다 (2026-09-14).

사용자 요청 "같은 설정으로 5번 돌려서 분포 내줘". `run_repeat5.sh` 가 만든
`RESULT_repeat5_rep*/grip-28mm/crush_*.npz` 를 모아 지표별 분포를 낸다.

**설정 동일성부터 검사한다.** npz 의 0-차원 메타(27개)를 전부 비교해서, 다른
항목이 있으면 표 위에 그대로 찍는다 — "같은 설정"이라는 전제가 깨지면 산포
해석이 통째로 무의미해지므로 숨기지 않는다. `clamp_wall_final` 처럼 **런 결과**인
항목은 당연히 다르므로 설정 비교에서 뺀다.

지표는 전부 `g_*` 계열(좌표 2계 차분 기반)로 고른다 — 접촉력 채널(`cf_*`,
`fld_fn`)은 2026-09-11 에 배선된 것이라 그 이전 런에는 없어서, 옛 런과 나란히
놓으려면 공통 채널이어야 한다. 있으면 접촉력 지표도 덧붙인다.

사용법:
    python probe_repeat_spread.py                    # repeat5 런 전부
    python probe_repeat_spread.py <npz> <npz> ...    # 직접 지정
"""
import os
import sys
import glob

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_WF = os.path.join(os.path.dirname(_HERE), "Crusher_M0609_RG2_Tablet_Samplebag")

# 설정이 아니라 **결과**인 메타 — 동일성 검사에서 뺀다.
_RESULT_META = {"clamp_wall_final", "wall_hold", "n_grains"}


def _load(paths):
    out = []
    for p in paths:
        d = np.load(p, allow_pickle=True)
        out.append((os.path.basename(os.path.dirname(os.path.dirname(p))), p, d))
    return out


def _check_same(runs):
    """0-차원 메타를 전부 비교 — 설정이 진짜 같은지."""
    keys = [k for k in sorted(runs[0][2].files)
            if runs[0][2][k].ndim == 0 and k not in _RESULT_META]
    bad = {}
    for k in keys:
        v = [float(d[k]) for _, _, d in runs]
        if max(v) - min(v) != 0:
            bad[k] = v
    print(f"[설정 동일성] 비교 항목 {len(keys)}개 (결과성 메타 {sorted(_RESULT_META)} 제외)")
    if bad:
        print("  **다른 항목이 있다 — 아래 산포는 비결정성만의 것이 아니다**")
        for k, v in bad.items():
            print(f"    {k:20s} {v}")
    else:
        print("  전부 동일 — 산포는 비결정성에서만 온다")
    # 접촉력 채널 유무(코드 버전 차이)도 같이 본다.
    has_cf = [("fld_fn" in d.files) for _, _, d in runs]
    if len(set(has_cf)) > 1:
        print("  **주의: 접촉력 채널(fld_fn) 유무가 런마다 다르다** — "
              "계측 배선이 다른 코드 버전이 섞여 있다는 뜻이다")
    return not bad


def _metrics(d):
    cd, wd = int(d["crank_dof"]), int(d["wall_dof"])
    q = d["dof_q"]
    ext = (d["g_hi"] - d["g_lo"]) * 1e3
    tau = d["dof_cf"][:, cd]
    m = {
        "알당 반력 최대 [mN]": float(d["g_fmax"].max() * 1e3),
        "알당 반력 평균 [mN]": float(d["g_fmean"][1:].mean() * 1e3),
        "무리 KE 최대 [uJ]": float(d["g_ke"].max() * 1e6),
        "크랭크 총회전 [deg]": float(np.degrees(q[-1, cd] - q[0, cd])),
        "토크 포화 비율 [%]": float((np.abs(tau) >= 0.99 * float(d["crank_torque_lim"])).mean() * 100),
        "더미 최종 높이 [mm]": float(ext[-1, 2]),
        "더미 높이 변화 [mm]": float(ext[-1, 2] - ext[0, 2]),
        "접촉 알 수 최대 [개]": float(d["g_ncon"].max()),
        "벽 최종 위치 [mm]": float(q[-1, wd] * 1e3),
    }
    if "fld_fn" in d.files:
        fn = np.linalg.norm(d["fld_fn"].astype(float), axis=2)
        m["법선 접촉력 최대 [mN]"] = float(fn.max() * 1e3)
    return m


def main():
    args = [a for a in sys.argv[1:] if a.endswith(".npz")]
    if not args:
        args = sorted(glob.glob(os.path.join(_WF, "RESULT_repeat5_rep*",
                                             "grip*", "crush_*.npz")))
    if len(args) < 2:
        raise SystemExit(f"npz 가 {len(args)}개뿐이다 — 반복 런이 아직 안 끝났다")
    runs = _load(args)
    print("=" * 92)
    print(f"같은 설정 {len(runs)}회 반복 — 비결정성 산포")
    print("=" * 92)
    for nm, p, d in runs:
        print(f"  {nm:24s} {os.path.basename(p)}")
    print()
    _check_same(runs)
    print()

    M = [_metrics(d) for _, _, d in runs]
    keys = list(M[0].keys())
    names = [nm.replace("RESULT_repeat5_", "").replace("RESULT_crush_", "")
             for nm, _, _ in runs]

    hdr = f"  {'지표':<22s}" + "".join(f"{n:>11s}" for n in names) + \
          f"{'평균':>11s}{'표준편차':>11s}{'CV%':>8s}{'최대/최소':>10s}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    stats = {}
    for k in keys:
        v = np.array([m[k] for m in M])
        mu, sd = v.mean(), v.std(ddof=1)
        cv = abs(sd / mu) * 100 if mu != 0 else np.nan
        rat = (v.max() / v.min()) if v.min() > 0 else np.nan
        stats[k] = (v, mu, sd, cv, rat)
        print(f"  {k:<22s}" + "".join(f"{x:11.3f}" for x in v) +
              f"{mu:11.3f}{sd:11.3f}{cv:8.1f}" +
              (f"{rat:10.2f}" if np.isfinite(rat) else f"{'-':>10s}"))
    print()
    print("  CV% = 표준편차/평균. 이 값이 크면 1회 실행으로는 그 지표를 말할 수 없다.")

    _plot(runs, names, keys, stats)


def _plot(runs, names, keys, stats):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matplotlib.rcParams["font.family"] = "Malgun Gothic"
    matplotlib.rcParams["axes.unicode_minus"] = False
    matplotlib.rcParams["mathtext.fontset"] = "dejavusans"

    n = len(keys)
    ncol = 5
    nrow = int(np.ceil(n / ncol)) + 1
    fig = plt.figure(figsize=(3.3 * ncol, 3.5 * nrow))
    fig.suptitle(f"같은 설정 {len(runs)}회 반복 — 비결정성 산포 "
                 f"(낟알 {int(runs[0][2]['n_grains'])}알 R="
                 f"{float(runs[0][2]['grain_radius'])*1e3:.1f}mm, "
                 f"{float(runs[0][2]['crank_rpm']):.0f} RPM x "
                 f"{float(runs[0][2]['crush_seconds']):.0f}s, GS_SEED 동일)",
                 fontsize=13)

    # (1) 지표별 스트립 — 점 하나가 런 하나
    for i, k in enumerate(keys):
        a = fig.add_subplot(nrow, ncol, i + 1)
        v, mu, sd, cv, rat = stats[k]
        x = np.arange(len(v))
        a.axhspan(mu - sd, mu + sd, color="#0072B2", alpha=0.13, lw=0)
        a.axhline(mu, color="#0072B2", lw=1.0, ls="--")
        a.scatter(x, v, s=55, color="#d62728", zorder=3, edgecolor="white", lw=0.8)
        for xi, vi in zip(x, v):
            a.annotate(f"{vi:.3g}", (xi, vi), fontsize=6.5, ha="center",
                       textcoords="offset points", xytext=(0, 7))
        a.set_xticks(x); a.set_xticklabels(names, fontsize=6.5, rotation=45, ha="right")
        a.set_title(f"{k}\nCV {cv:.1f}%" + (f" · 최대/최소 {rat:.2f}배"
                                            if np.isfinite(rat) else ""), fontsize=8.5)
        a.grid(alpha=0.25, axis="y")
        a.margins(y=0.28)

    # (2) 발산 곡선 — 모든 런 쌍의 낟알 무게중심 거리
    a = fig.add_subplot(nrow, 1, nrow)
    t = runs[0][2]["t"]
    for i in range(len(runs)):
        for j in range(i + 1, len(runs)):
            d = np.linalg.norm(runs[i][2]["g_com"] - runs[j][2]["g_com"], axis=1) * 1e3
            a.plot(t, np.maximum(d, 1e-6), lw=0.7, alpha=0.65,
                   label=f"{names[i]}-{names[j]}" if i == 0 and j == 1 else None)
    a.set_yscale("log")
    a.set_xlabel("분쇄 시작 기준 t [s]")
    a.set_ylabel("두 런의 낟알 무게중심 거리 [mm]")
    a.set_title("같은 설정인데 갈라진다 — 모든 런 쌍의 거리 (선 하나가 한 쌍)", fontsize=10)
    a.grid(alpha=0.25)

    out = os.path.join(_HERE, "repeat_spread.png")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, facecolor="white", dpi=115)
    plt.close(fig)
    print(f"\n[saved] {out}")


if __name__ == "__main__":
    main()
