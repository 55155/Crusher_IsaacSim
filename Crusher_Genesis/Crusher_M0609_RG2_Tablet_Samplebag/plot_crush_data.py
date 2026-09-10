"""
plot_crush_data.py — `crush_<tag>.npz`(full_workflow.py Phase 11 전수 계측) 요약·플롯.

STAGE=...:crush 런이 남긴 npz 하나를 읽어 (a) 어떤 채널이 들어 있는지 텍스트로
훑고 (b) 하중 경로를 한 장으로 그린다: 액추에이터(크랭크 토크/벽 힘) ->
기구 링크(IPC 커플링 반력) -> 봉투 -> 낟알(알당 접촉 반력·압밀).

사용법:
    python plot_crush_data.py                       # 가장 최근 RESULT*/**/crush_*.npz
    python plot_crush_data.py <경로.npz>
"""
import os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 축/제목이 한글이다 — 기본 폰트로는 전부 두부가 된다(Crusher_8env.py 와 같은 처방).
matplotlib.rcParams["font.family"] = "Malgun Gothic"
matplotlib.rcParams["axes.unicode_minus"] = False
# 로그축 눈금은 mathtext 로 그려지는데 Malgun Gothic 에 그 글리프가 없어 10^-1 이
# "10¤1" 로 깨진다 — 수식 폰트만 DejaVu 로 돌린다.
matplotlib.rcParams["mathtext.fontset"] = "dejavusans"

HERE = os.path.dirname(os.path.abspath(__file__))


def newest():
    c = glob.glob(os.path.join(HERE, "RESULT*", "**", "crush_*.npz"), recursive=True)
    if not c:
        raise SystemExit("crush_*.npz 를 못 찾았다 — STAGE=...:crush 런을 먼저 돌려라")
    return max(c, key=os.path.getmtime)


def inventory(d):
    """npz 안에 실제로 뭐가 들어 있는지 — 채널명/모양/범위."""
    print("=" * 86)
    print(f"{'채널':16s} {'모양':18s} {'단위':8s} 설명")
    print("-" * 86)
    desc = {
        "t": ("s", "분쇄 시작 기준 시간"),
        "step": ("-", "런 전체 누적 스텝 번호"),
        "dof_q": ("rad/m", "크러셔 전 관절 위치"),
        "dof_v": ("rad/s,m/s", "크러셔 전 관절 속도"),
        "dof_cf": ("N·m,N", "관절 **액추에이터 출력** = 반력(control_force)"),
        "dof_f": ("N·m,N", "관절 총 외력 버퍼(커플링 외력 포함)"),
        "link_F": ("N", "IPC->강체 커플링 반력(링크별 3축)"),
        "link_T": ("N·m", "IPC->강체 커플링 반모멘트"),
        "g_com": ("m", "낟알 무리 무게중심"),
        "g_lo": ("m", "낟알 무리 AABB 하단"),
        "g_hi": ("m", "낟알 무리 AABB 상단"),
        "g_vmean": ("m/s", "낟알 속력 평균"),
        "g_vmax": ("m/s", "낟알 속력 최대"),
        "g_ke": ("J", "낟알 무리 운동에너지"),
        "g_fmean": ("N", "알당 접촉 반력 평균"),
        "g_fmax": ("N", "알당 접촉 반력 최대"),
        "g_fp95": ("N", "알당 접촉 반력 95 분위"),
        "g_fsum": ("N", "무리 합력(3축)"),
        "g_ncon": ("개", "반력이 자중 10% 를 넘는 알 수"),
        "b_com": ("m", "봉투 정점 평균"),
        "b_lo": ("m", "봉투 AABB 하단"),
        "b_hi": ("m", "봉투 AABB 상단"),
        "fld_pos": ("m", "낟알 전량 좌표 (샘플, N, 3)"),
        "fld_vel": ("m/s", "낟알 전량 속도(좌표 차분)"),
        "fld_f": ("N", "낟알 전량 접촉 반력"),
        "fld_nn": ("m", "알별 최근접 이웃 거리"),
        "fld_dbag": ("m", "알별 봉투 표면 최근접 거리"),
        "bagv_pos": ("m", "봉투 정점 좌표 (샘플, V, 3)"),
    }
    for k in sorted(d.files):
        a = d[k]
        if k not in desc:
            continue
        u, t = desc[k]
        print(f"{k:16s} {str(a.shape):18s} {u:8s} {t}")
    print("-" * 86)
    meta = [k for k in sorted(d.files) if k not in desc]
    print("메타: " + "  ".join(
        f"{k}={d[k] if d[k].ndim == 0 else d[k].shape}" for k in meta if k not in
        ("joint_names", "link_names", "link_idx")))
    print("관절: " + ", ".join(f"{i}:{n}" for i, n in enumerate(d["joint_names"])))
    _li = d["link_idx"] if "link_idx" in d.files else np.arange(len(d["link_names"]))
    print("커플링 링크: " + ", ".join(f"{n}#{i}" for n, i in zip(d["link_names"], _li)))
    print("=" * 86)


def main():
    f = sys.argv[1] if len(sys.argv) > 1 else newest()
    d = np.load(f, allow_pickle=True)
    inventory(d)

    t = d["t"]
    cd, wd = int(d["crank_dof"]), int(d["wall_dof"])
    rpm = d["dof_v"][:, cd] * 60 / (2 * np.pi)
    deg = np.degrees(d["dof_q"][:, cd] - d["dof_q"][0, cd])
    m1, r = float(d["grain_mass"]), float(d["grain_radius"])
    w1 = m1 * 9.81

    fig, ax = plt.subplots(4, 2, figsize=(15, 14))
    fig.suptitle(f"Crusher 타격 — 낟알 {int(d['n_grains'])}알(R={r*1e3:.2f}mm, "
                 f"총 {m1*int(d['n_grains'])*1e3:.3f}g) 든 샘플백, "
                 f"{float(d['crank_rpm']):.0f} RPM x {float(d['crush_seconds']):.0f}s",
                 fontsize=13)

    a = ax[0, 0]; a.plot(t, deg, color="#0072B2")
    a.set_ylabel("크랭크 회전 [deg]"); a.set_title("크랭크 각도(스트로크 위상)")
    a2 = a.twinx(); a2.plot(t, rpm, color="#999", lw=0.8); a2.set_ylabel("RPM")

    a = ax[0, 1]; a.plot(t, d["dof_cf"][:, cd], color="#d62728", lw=0.8)
    a.axhline(float(d["crank_torque_lim"]), ls="--", c="k", lw=0.8)
    a.axhline(-float(d["crank_torque_lim"]), ls="--", c="k", lw=0.8)
    a.set_ylabel("크랭크 토크 [N·m]"); a.set_title("크랭크 액추에이터 출력(반력)")

    a = ax[1, 0]; a.plot(t, d["dof_q"][:, wd] * 1e3, color="#0072B2")
    a.set_ylabel("Left_Wall 위치 [mm]"); a.set_title("압착 벽 — 타격 중 파고듦")

    a = ax[1, 1]; a.plot(t, d["dof_cf"][:, wd], color="#d62728", lw=0.8, label="control_force")
    a.plot(t, d["dof_f"][:, wd], color="#555", lw=0.6, label="dofs_force")
    a.axhline(-float(d["wall_force_lim"]), ls="--", c="k", lw=0.8)
    a.legend(fontsize=8); a.set_ylabel("벽 반력 [N]"); a.set_title("압착 벽 반력")

    a = ax[2, 0]
    if "link_F" in d.files and len(d["link_names"]):
        Fn = np.linalg.norm(d["link_F"], axis=2)
        for i in np.argsort(-Fn.max(axis=0))[:3]:
            _ix = d["link_idx"][i] if "link_idx" in d.files else i
            a.plot(t, Fn[:, i], lw=0.8, label=f"{d['link_names'][i]}#{_ix}")
        a.legend(fontsize=7)
    a.set_ylabel("|F| [N]"); a.set_title("IPC->강체 커플링 반력 (상위 3 링크)")

    a = ax[2, 1]
    a.plot(t, d["g_fmax"] * 1e3, lw=0.7, color="#d62728", label="최대")
    a.plot(t, d["g_fp95"] * 1e3, lw=0.7, color="#E69F00", label="95%")
    a.plot(t, d["g_fmean"] * 1e3, lw=0.7, color="#0072B2", label="평균")
    a.axhline(w1 * 1e3, ls="--", c="k", lw=0.8, label=f"자중 {w1*1e3:.4f}mN")
    a.set_yscale("log"); a.legend(fontsize=7)
    a.set_ylabel("알당 접촉 반력 [mN]"); a.set_title("낟알 1알에 걸리는 반력")

    a = ax[3, 0]
    ext = (d["g_hi"] - d["g_lo"]) * 1e3
    for i, lb in enumerate("xyz"):
        a.plot(t, ext[:, i], lw=0.8, label=f"낟알 더미 {lb}")
    bext = (d["b_hi"] - d["b_lo"]) * 1e3
    a.plot(t, bext[:, 1], lw=0.8, ls="--", label="봉투 두께 y")
    a.legend(fontsize=7); a.set_xlabel("t [s]")
    a.set_ylabel("폭 [mm]"); a.set_title("압밀 — 더미/봉투 크기")

    a = ax[3, 1]
    a.plot(t, d["g_ke"] * 1e6, lw=0.7, color="#0072B2")
    a.set_ylabel("운동에너지 [uJ]"); a.set_xlabel("t [s]")
    a.set_title("낟알 무리 운동에너지")
    a2 = a.twinx(); a2.plot(t, d["g_ncon"], lw=0.6, color="#999")
    a2.set_ylabel("접촉 중인 알 수")

    for r_ in ax:
        for a_ in r_:
            a_.grid(alpha=0.25)
    # 하중 경로 요약 — 액추에이터가 넣은 일과 낟알이 받은 것.
    dt = float(d["dt"]) * int(d["data_every"])
    w_crank = float(np.sum(d["dof_cf"][:, cd] * d["dof_v"][:, cd]) * dt)
    w_wall = float(np.sum(d["dof_cf"][:, wd] * d["dof_v"][:, wd]) * dt)
    print(f"[일] 크랭크 {w_crank:+.3f} J   벽 {w_wall:+.3f} J   "
          f"(부호 +는 액추에이터가 계에 넣은 일)")
    # 첫 표본은 v^{-1}=0 가정 때문에 가짜 임펄스가 섞인다 — 버린다.
    gfa, gfm = d["g_fmean"][1:], d["g_fmax"][1:]
    print(f"[낟알] 알당 반력 평균 {gfa.mean()*1e3:.4f} mN / "
          f"자중 {w1*1e3:.4f} mN = {gfa.mean()/w1:.2f} 배,  "
          f"최대 {gfm.max()*1e3:.4f} mN = {gfm.max()/w1:.1f} 배")
    fs = d["g_fsum"]
    print(f"[낟알] 무리 합력 |F| 평균 {np.linalg.norm(fs, axis=1).mean()*1e3:.3f} mN  "
          f"최대 {np.linalg.norm(fs, axis=1).max()*1e3:.3f} mN  "
          f"(전체 자중 {w1*int(d['n_grains'])*1e3:.3f} mN)")

    out = os.path.splitext(f)[0] + "_profiles.png"
    fig.tight_layout()
    fig.savefig(out, facecolor="white", dpi=110)
    print(f"[saved] {out}")

    # 압밀 전후 — 최근접 간격 분포
    if "fld_nn" in d.files:
        nn = d["fld_nn"]
        fig2, a = plt.subplots(1, 2, figsize=(12, 4.2))
        bins = np.linspace(0, max(4 * r, float(nn.max())) * 1e3, 60)
        a[0].hist(nn[0] * 1e3, bins=bins, alpha=0.6, label="시작")
        a[0].hist(nn[-1] * 1e3, bins=bins, alpha=0.6, label="종료")
        a[0].axvline((2 * r + float(d["d_hat"])) * 1e3, ls="--", c="k",
                     label="2R+d_hat (배리어)")
        a[0].legend(fontsize=8); a[0].set_xlabel("최근접 낟알 간격 [mm]")
        a[0].set_title("낟알 패킹 — 배리어에 굳었는가")
        db = d["fld_dbag"]
        a[1].hist(db[0] * 1e3, bins=60, alpha=0.6, label="시작")
        a[1].hist(db[-1] * 1e3, bins=60, alpha=0.6, label="종료")
        a[1].legend(fontsize=8); a[1].set_xlabel("낟알-봉투 최근접 거리 [mm]")
        a[1].set_title("봉투 벽과의 거리 분포")
        for a_ in a:
            a_.grid(alpha=0.25)
        out2 = os.path.splitext(f)[0] + "_packing.png"
        fig2.tight_layout(); fig2.savefig(out2, facecolor="white", dpi=110)
        print(f"[saved] {out2}")


if __name__ == "__main__":
    main()
