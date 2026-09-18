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
        "cf_n_mean": ("N", "알당 **법선 접촉력** 평균 (uipc gradient/dt^2)"),
        "cf_n_max": ("N", "알당 법선 접촉력 최대"),
        "cf_n_p95": ("N", "알당 법선 접촉력 95 분위"),
        "cf_n_sum": ("N", "무리 법선 접촉력 합 (3축)"),
        "cf_n_cnt": ("개", "법선힘이 자중 1% 를 넘는 알 수"),
        "cf_t_mean": ("N", "알당 **마찰력** 평균"),
        "cf_t_max": ("N", "알당 마찰력 최대"),
        "cf_t_p95": ("N", "알당 마찰력 95 분위"),
        "cf_types": ("개", "프리미티브 타입별 접촉 수 (PP=알끼리, PT=알-면)"),
        "fld_fn": ("N", "낟알 전량 법선 접촉력"),
        "fld_ft": ("N", "낟알 전량 마찰력"),
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


def _snap_indices(d, t):
    """스냅샷으로 쓸 fld 표본 인덱스 — 타격마다 힘이 가장 큰 순간을 고른다.

    등간격으로 뽑으면 8 RPM(7.5s 주기)에서 타격을 통째로 놓친다. 크랭크 토크가
    포화한 구간(=타격)마다 그 안에서 알당 법선력이 최대인 표본을 집는다.
    반환: [(라벨, fld 인덱스), ...]
    """
    fs = d["fld_step"]
    tau = d["dof_cf"][:, int(d["crank_dof"])]
    fn_max = d["cf_n_max"]
    sat = np.flatnonzero(tau >= 12.4)
    out = [("t=%.1fs 정지" % t[fs[1]], 1)] if len(fs) > 1 else []
    if len(sat):
        brk = np.flatnonzero(np.diff(sat) > 40)
        for gi, g in enumerate(np.split(sat, brk + 1)):
            if len(g) < 20:
                continue
            k = g[int(np.argmax(fn_max[g]))]            # 그 타격의 피크 스텝
            i = int(np.argmin(np.abs(fs - k)))
            out.append((f"타격 {gi+1}  t={t[fs[i]]:.1f}s", i))
    return out[:9]


def _snap_within_strike(d, t, n=8):
    """한 번의 타격 **안**을 촘촘히 — 접근 -> 피크 -> 이완."""
    fs = d["fld_step"]
    tau = d["dof_cf"][:, int(d["crank_dof"])]
    sat = np.flatnonzero(tau >= 12.4)
    if not len(sat):
        return []
    brk = np.flatnonzero(np.diff(sat) > 40)
    groups = [g for g in np.split(sat, brk + 1) if len(g) >= 20]
    if not groups:
        return []
    g = groups[min(2, len(groups) - 1)]                 # 과도 영향 적은 주기
    a, b = g[0], g[-1]
    pad = int(0.35 * (b - a))
    ks = np.linspace(max(a - pad, 0), min(b + pad, fs[-1]), n).astype(int)
    return [(f"t={t[k]:.2f}s", int(np.argmin(np.abs(fs - k)))) for k in ks]


def _particle_grid(d, path, snaps, which, title, fname):
    """낟알을 3D 스캐터로 뿌리고 **색으로 힘**을 표현. 스냅샷 격자."""
    from matplotlib.colors import LogNorm
    import matplotlib.cm as cm
    if not snaps:
        return
    pos = d["fld_pos"].astype(float)
    val = np.linalg.norm(d[which].astype(float), axis=2)     # (샘플, 알) 힘 크기
    w1 = float(d["grain_mass"]) * 9.81
    R = float(d["grain_radius"])
    # 색 범위는 **전 스냅샷 공통**이어야 비교가 된다. 자중을 하한으로 깔아
    # "자중보다 큰가"가 바로 읽히게 한다.
    vmax = max(float(np.quantile(val[[i for _, i in snaps]], 0.999)), w1 * 10)
    norm = LogNorm(vmin=w1, vmax=vmax)
    cmap = cm.get_cmap("turbo") if hasattr(cm, "get_cmap") else plt.get_cmap("turbo")

    ncol = 4 if len(snaps) > 3 else len(snaps)
    nrow = int(np.ceil(len(snaps) / ncol))
    fig = plt.figure(figsize=(4.3 * ncol, 3.9 * nrow + 0.9))
    fig.suptitle(title, fontsize=14)
    lo = pos[[i for _, i in snaps]].reshape(-1, 3).min(axis=0)
    hi = pos[[i for _, i in snaps]].reshape(-1, 3).max(axis=0)
    pad = 0.06 * (hi - lo).max()
    lo, hi = lo - pad, hi + pad
    ext = (hi - lo) * 1e3                       # mm
    # 낟알층은 y 로 3mm 남짓, x 로 40mm 다 — 정육면체로 맞추면 얇은 층이 공중에
    # 뜬 판처럼 보인다. 실제 종횡비를 그대로 준다(단 y 가 너무 납작해 안 보이지
    # 않게 최소 0.12 로 바닥을 깐다).
    asp = ext / ext.max()
    asp[1] = max(asp[1], 0.12)

    for j, (lb, i) in enumerate(snaps):
        ax = fig.add_subplot(nrow, ncol, j + 1, projection="3d")
        p, v = pos[i], np.maximum(val[i], w1 * 1.001)
        o = np.argsort(v)                                     # 큰 힘을 위에 그린다
        ax.scatter(p[o, 0] * 1e3, p[o, 1] * 1e3, p[o, 2] * 1e3, c=v[o], cmap=cmap,
                   norm=norm, s=26, depthshade=False, edgecolors="none")
        ax.set_xlim(lo[0] * 1e3, hi[0] * 1e3)
        ax.set_ylim(lo[1] * 1e3, hi[1] * 1e3)
        ax.set_zlim(lo[2] * 1e3, hi[2] * 1e3)
        ax.set_box_aspect(tuple(asp))
        ax.view_init(elev=20, azim=-68)
        ax.set_title(lb + chr(10) + "최대 %.1f mN · 자중의 %.0f배"
                     % (v.max() * 1e3, v.max() / w1), fontsize=9)
        ax.tick_params(labelsize=6, pad=-1)
        # y(봉투 두께)는 몇 mm 뿐이라 기본 눈금이 겹쳐 뭉갠다 — 개수를 줄인다.
        from matplotlib.ticker import MaxNLocator
        ax.xaxis.set_major_locator(MaxNLocator(4))
        ax.yaxis.set_major_locator(MaxNLocator(3))
        ax.zaxis.set_major_locator(MaxNLocator(4))
        ax.set_xlabel("x [mm]", fontsize=7, labelpad=-2)
        ax.set_ylabel("y [mm]", fontsize=7, labelpad=-4)
        ax.set_zlabel("z [mm]", fontsize=7, labelpad=-2)
    sm = cm.ScalarMappable(norm=norm, cmap=cmap); sm.set_array([])
    cb = fig.colorbar(sm, ax=fig.axes, fraction=0.02, pad=0.02)
    # mathtext 지수(10^-1)가 한글 폰트에서 두부가 되므로 눈금을 평문 mN 으로 쓴다.
    dec = np.unique(np.clip(np.round(np.logspace(np.log10(w1), np.log10(vmax), 6) * 1e3,
                                     4), 1e-4, None))
    tk = [v * 1e-3 for v in dec]
    cb.set_ticks(tk)
    cb.set_ticklabels([("%.3g" % v) for v in dec])
    cb.set_label("알당 힘 [mN]   (하한 = 1알 자중 %.4f mN)" % (w1 * 1e3), fontsize=9)
    out = os.path.splitext(path)[0] + fname
    fig.savefig(out, facecolor="white", dpi=105, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {out}")


# ── 봉투 면 좌표 — 충격력을 **봉투 규격 격자** 위에 얹는다 (2026-09-14) ─────
# 규격은 full_workflow.py 의 BAG_PANEL_HALF_W(0.032)/BAG_HALF_H(0.045)/
# SEAL_BAND_WIDTH(0.010) 이 정본이다 — **64 x 90mm**(두께 6mm) 이지 50x100 이
# 아니다. 좌우 가장자리 10mm 는 실링이라 파우더가 실제로 담기는 폭은 44mm.
BAG_W_MM, BAG_H_MM, SEAL_MM = 64.0, 90.0, 10.0


def _bag_local(d):
    """낟알 world 좌표를 **봉투 면 좌표**로 옮긴다 — u=폭, v=바닥 기준 높이.

    실배치(BAG_EULER=(90,0,0))에서 봉투 로컬 폭축은 world X, 높이축은 world Z,
    두께는 world Y 로 매핑된다(full_workflow.py §BAG_EULER 주석). 봉투는 분쇄
    내내 벽에 물려 거의 안 움직이지만(폭 중심 드리프트 ~1mm) 그 1mm 가 그대로
    격자 오차가 되므로 **매 스냅샷 봉투 정점군**에서 기준을 다시 잡는다:
        u0 = 정점 x 평균(폭 중심),  v0 = 정점 z 하위 0.5%(바닥, 국소 처짐 배제)
    반환: u, v, y (샘플, 알) [mm] + 봉투 정점의 bu, bv (샘플, V) [mm] + 짝 인덱스
    """
    bv = d["bagv_pos"].astype(float)
    fp = d["fld_pos"].astype(float)
    # fld 는 5스텝, bagv 는 10스텝 간격이라 표본 수가 다르다 — 가장 가까운 걸 붙인다.
    bi = np.abs(d["fld_step"][:, None] - d["bagv_step"][None, :]).argmin(axis=1)
    u0, y0 = bv[:, :, 0].mean(axis=1), bv[:, :, 1].mean(axis=1)
    v0 = np.percentile(bv[:, :, 2], 0.5, axis=1)
    u = (fp[:, :, 0] - u0[bi][:, None]) * 1e3
    v = (fp[:, :, 2] - v0[bi][:, None]) * 1e3
    y = (fp[:, :, 1] - y0[bi][:, None]) * 1e3
    return u, v, y, (bv[:, :, 0] - u0[:, None]) * 1e3, (bv[:, :, 2] - v0[:, None]) * 1e3, bi


def _bag_frame(ax, bu=None, bvv=None):
    """봉투 규격 격자 — 64x90mm 외곽 + 실링 밴드 + 10mm 눈금 + 실측 실루엣."""
    hw = BAG_W_MM / 2
    ax.add_patch(plt.Rectangle((-hw, 0), BAG_W_MM, BAG_H_MM, fill=False,
                               ec="#333333", lw=1.3, zorder=4))
    for x0 in (-hw, hw - SEAL_MM):              # 실링부 — 여긴 내용물이 안 들어간다
        ax.add_patch(plt.Rectangle((x0, 0), SEAL_MM, BAG_H_MM, facecolor="#d62728",
                                   alpha=0.07, lw=0, zorder=0))
    if bu is not None:
        # 사각형은 **원래 치수**다. 분쇄 중 봉투는 눌려 퍼지므로(폭 70mm 까지)
        # 실제 외곽을 같이 얹는다 — 격자가 이상적 규격임을 숨기지 않는다.
        try:
            from scipy.spatial import ConvexHull
            P = np.c_[bu, bvv]
            h = ConvexHull(P).vertices
            h = np.r_[h, h[:1]]
            ax.plot(P[h, 0], P[h, 1], color="#888888", lw=0.9, ls="--", zorder=3)
        except Exception:
            pass
    ax.set_xlim(-hw - 8, hw + 8)
    ax.set_ylim(-6, BAG_H_MM + 6)
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(-30, 31, 10))
    ax.set_yticks(np.arange(0, 91, 10))
    ax.grid(alpha=0.28, lw=0.5)


def _bagface(d, path, snaps, which, title, fname):
    """봉투 면(64x90mm) 위 낟알 스캐터 — **색이 곧 충격력**. 타격마다 한 칸."""
    from matplotlib.colors import LogNorm
    import matplotlib.cm as cm
    if not snaps:
        return
    u, v, y, bu, bvv, bi = _bag_local(d)
    val = np.linalg.norm(d[which].astype(float), axis=2)
    w1 = float(d["grain_mass"]) * 9.81
    ii = [i for _, i in snaps]
    vmax = max(float(np.quantile(val[ii], 0.999)), w1 * 10)
    norm = LogNorm(vmin=w1, vmax=vmax)          # 하한 = 자중, 즉 "안 눌리는 알"
    cmap = plt.get_cmap("turbo")

    ncol = 4 if len(snaps) > 3 else len(snaps)
    nrow = int(np.ceil(len(snaps) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 4.7 * nrow + 0.8),
                             squeeze=False)
    fig.suptitle(title, fontsize=13)
    # 점 크기를 **실제 낟알 지름**에 맞춘다 — 겹쳐 보이면 진짜로 겹쳐 있는 것이다.
    R = float(d["grain_radius"]) * 1e3
    s = (2 * R / (BAG_W_MM + 16) * (3.6 * 0.62) * 72) ** 2

    for j, ax in enumerate(axes.ravel()):
        if j >= len(snaps):
            ax.axis("off")
            continue
        lb, i = snaps[j]
        _bag_frame(ax, bu[bi[i]], bvv[bi[i]])
        f = np.maximum(val[i], w1 * 1.001)
        o = np.argsort(f)                       # 큰 힘을 위에 그린다
        ax.scatter(u[i][o], v[i][o], c=f[o], cmap=cmap, norm=norm, s=s,
                   linewidths=0.15, edgecolors="#00000030", zorder=5)
        ax.set_title(f"{lb}\n최대 {f.max()*1e3:,.0f} mN · 자중의 {f.max()/w1:,.0f}배",
                     fontsize=9)
        if j % ncol == 0:
            ax.set_ylabel("봉투 높이 v [mm]  (바닥=0)", fontsize=8)
        if j // ncol == nrow - 1:
            ax.set_xlabel("봉투 폭 u [mm]", fontsize=8)
        ax.tick_params(labelsize=7)
    sm = cm.ScalarMappable(norm=norm, cmap=cmap); sm.set_array([])
    cb = fig.colorbar(sm, ax=list(axes.ravel()), fraction=0.018, pad=0.02)
    dec = np.unique(np.round(np.logspace(np.log10(w1), np.log10(vmax), 6) * 1e3, 4))
    cb.set_ticks([x * 1e-3 for x in dec])
    cb.set_ticklabels(["%.3g" % x for x in dec])
    cb.set_label(f"알당 힘 [mN]   (하한 = 1알 자중 {w1*1e3:.4f} mN)", fontsize=9)
    fig.text(0.5, 0.005, "실선 사각형 = 봉투 규격 64x90mm(분홍 = 좌우 실링 10mm) · "
             "점선 = 그 순간의 봉투 실측 외곽 · 두께(y, 6mm) 방향은 접어서 투영했다",
             ha="center", fontsize=8.5, color="#555555")
    out = os.path.splitext(path)[0] + fname
    fig.savefig(out, facecolor="white", dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {out}")


def _bagface_map(d, path, which="fld_fn", cell=2.0):
    """봉투 규격 격자를 **셀로 잘라** 충격력을 집계한다 — 어디가 얼마나 맞는가.

    한 순간만 보면 그 순간의 배치일 뿐이라, 런 전체에서 셀마다
      (a) 최대 알당 힘  (b) 누적 충격량 SUM|f|dt  (c) 자중 100배 초과 표본 수
    를 모아 얹는다. (b)(c) 는 **체류 시간이 섞인** 값이다 — 오래 머문 자리가
    당연히 크게 나온다. 셀은 기본 2mm(낟알 지름과 같은 눈금).
    """
    from matplotlib.colors import LogNorm
    u, v, y, bu, bvv, bi = _bag_local(d)
    val = np.linalg.norm(d[which].astype(float), axis=2)
    w1 = float(d["grain_mass"]) * 9.81
    dtf = float(d["dt"]) * int(d["field_every"])        # fld 표본 간격 [s]
    hw = BAG_W_MM / 2
    ue = np.arange(-hw, hw + 1e-9, cell)
    ve = np.arange(0.0, BAG_H_MM + 1e-9, cell)
    nu, nv = len(ue) - 1, len(ve) - 1

    # 첫 표본은 SPC 해제 과도라 버린다(다른 도면과 같은 처방).
    U, V, F = u[1:].ravel(), v[1:].ravel(), val[1:].ravel()
    inb = (U >= ue[0]) & (U < ue[-1]) & (V >= ve[0]) & (V < ve[-1])
    n_out = int((~inb).sum())
    iu = np.clip(np.searchsorted(ue, U[inb], "right") - 1, 0, nu - 1)
    iv = np.clip(np.searchsorted(ve, V[inb], "right") - 1, 0, nv - 1)
    flat = iv * nu + iu
    peak = np.zeros(nu * nv); imp = np.zeros(nu * nv); hit = np.zeros(nu * nv)
    np.maximum.at(peak, flat, F[inb])
    np.add.at(imp, flat, F[inb] * dtf)
    np.add.at(hit, flat, (F[inb] > 100 * w1).astype(float))
    peak = peak.reshape(nv, nu); imp = imp.reshape(nv, nu); hit = hit.reshape(nv, nu)

    fig = plt.figure(figsize=(19, 7.0))
    gs_ = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1, 0.9], wspace=0.30, hspace=0.38)
    fig.suptitle(f"봉투 규격 {BAG_W_MM:.0f} x {BAG_H_MM:.0f}mm 격자 위 충격력 — "
                 f"{int(d['n_grains'])}알 R={float(d['grain_radius'])*1e3:.1f}mm, "
                 f"{float(d['crank_rpm']):.0f} RPM x {float(d['crush_seconds']):.0f}s "
                 f"(셀 {cell:.0f}mm, 1알 자중 {w1*1e3:.4f} mN)", fontsize=12)

    for k, (M, sc, lb, cmp_, logn) in enumerate((
            (peak, 1e3, "셀별 **최대** 알당 힘 [mN]", "turbo", True),
            (imp, 1e3, "셀별 누적 충격량 SUM|f|dt [mN·s]", "magma", True),
            (hit, 1.0, "자중 100배 초과 표본 수 [개]", "viridis", False))):
        a = fig.add_subplot(gs_[:, k])
        Md = np.where(M > 0, M * sc, np.nan)
        kw = dict(cmap=cmp_, zorder=1)
        if logn:
            # 하한을 최솟값으로 두면 6 decade 가 깔려 흥미로운 구간이 전부 같은
            # 색으로 뭉갠다(실측 셀최대 5분위 98mN vs 최대 111,550mN). 5분위로 자른다.
            pos = Md[np.isfinite(Md)]
            kw["norm"] = LogNorm(vmin=max(float(np.quantile(pos, 0.05)), 1e-4),
                                 vmax=float(pos.max()))
        m = a.pcolormesh(ue, ve, Md, **kw)
        _bag_frame(a, bu[-1], bvv[-1])
        cb = fig.colorbar(m, ax=a, fraction=0.045, pad=0.02)
        cb.ax.tick_params(labelsize=7)
        a.set_title(lb, fontsize=10)
        a.set_xlabel("봉투 폭 u [mm]", fontsize=9)
        if k == 0:
            a.set_ylabel("봉투 높이 v [mm]  (바닥=0)", fontsize=9)

    # 주변분포 — 하중이 폭/높이 어느 쪽으로 쏠리는가
    uc, vc = (ue[:-1] + ue[1:]) / 2, (ve[:-1] + ve[1:]) / 2
    for r, (c, M1, Mi, lb) in enumerate((
            (uc, peak.max(axis=0), imp.sum(axis=0), "봉투 폭 u [mm]"),
            (vc, peak.max(axis=1), imp.sum(axis=1), "봉투 높이 v [mm]"))):
        a = fig.add_subplot(gs_[r, 3])
        a.plot(c, np.where(M1 > 0, M1, np.nan) * 1e3, color="#d62728", lw=1.2,
               label="셀 최대 힘 [mN]")
        a.set_yscale("log"); a.set_xlabel(lb, fontsize=9)
        a.set_ylabel("최대 힘 [mN]", fontsize=8, color="#d62728")
        a.grid(alpha=0.28)
        a2 = a.twinx()
        a2.fill_between(c, Mi / max(Mi.sum(), 1e-30) * 100, color="#0072B2", alpha=0.25)
        a2.set_ylabel("충격량 점유 [%]", fontsize=8, color="#0072B2")
        if r == 0:
            for x0 in (-hw + SEAL_MM, hw - SEAL_MM):
                a.axvline(x0, color="#d62728", ls=":", lw=1.0)
            a.set_title("하중 쏠림 — 폭 방향(점선=실링 경계)", fontsize=10)
        else:
            a.set_title("하중 쏠림 — 높이 방향", fontsize=10)

    fig.text(0.5, 0.005, "실선 사각형 = 봉투 규격 64x90mm(분홍 = 좌우 실링 10mm) · "
             "점선 = 런 종료 시점의 봉투 실측 외곽 · 색 하한은 셀 5분위(6 decade 가 "
             "깔리면 전부 같은 색이 된다)", ha="center", fontsize=8.5, color="#555555")
    out = os.path.splitext(path)[0] + "_bagface_map.png"
    fig.savefig(out, facecolor="white", dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {out}")

    # 숫자로도 남긴다 — 그림만 보고 눈대중하지 않도록.
    occ = int((peak > 0).sum())
    print(f"[봉투면] 낟알 점유 {occ}/{nu*nv} 셀 ({occ/(nu*nv)*100:.1f}%) — "
          f"규격 밖 표본 {n_out}개 ({n_out/max(U.size,1)*100:.2f}%)")
    print(f"[봉투면] 폭 u {U.min():+.1f}~{U.max():+.1f}mm (실링 |u|>{hw-SEAL_MM:.0f}mm), "
          f"높이 v {V.min():.1f}~{V.max():.1f}mm — 파우더는 바닥 {V.max():.0f}mm 안에만 있다")
    print(f"[봉투면] 두께 y {y[1:].min():+.2f}~{y[1:].max():+.2f}mm — 면 투영은 이 "
          f"두께를 접어 버린다(겹쳐 보이는 점은 실제로 앞뒤로 떨어져 있을 수 있다)")
    j = int(np.argmax(peak))
    print(f"[봉투면] 최대 하중 셀 u={uc[j % nu]:+.0f}mm v={vc[j // nu]:.0f}mm — "
          f"{peak.ravel()[j]*1e3:,.0f} mN (자중의 {peak.ravel()[j]/w1:,.0f}배)")
    k = int(np.argmax(imp))
    print(f"[봉투면] 최대 충격량 셀 u={uc[k % nu]:+.0f}mm v={vc[k // nu]:.0f}mm — "
          f"{imp.ravel()[k]*1e3:,.1f} mN·s")


def _plot_contact(d, path, t, deg, w1, m1):
    """접촉력(법선)과 마찰이 타격마다 얼마나 나는지 — 한눈에 들어오게."""
    cd = int(d["crank_dof"])
    tau = d["dof_cf"][:, cd]
    strike = tau >= 12.4                      # 토크 포화 = 타격 중
    fn_max, fn_p95, fn_mean = d["cf_n_max"], d["cf_n_p95"], d["cf_n_mean"]
    ft_max, ft_p95, ft_mean = d["cf_t_max"], d["cf_t_p95"], d["cf_t_mean"]
    sl = slice(1, None)                       # 첫 표본은 해제 과도

    fig = plt.figure(figsize=(15, 11))
    gs_ = fig.add_gridspec(3, 2, height_ratios=[1.15, 1, 1], hspace=0.42, wspace=0.26)
    fig.suptitle(f"낟알 접촉력 · 마찰 — {int(d['n_grains'])}알 R={float(d['grain_radius'])*1e3:.1f}mm, "
                 f"{float(d['crank_rpm']):.0f} RPM x {float(d['crush_seconds']):.0f}s   "
                 f"(1알 자중 {w1*1e3:.4f} mN, mu={float(d['grain_friction']):.1f})", fontsize=13)

    # (1) 시계열 — 타격 구간을 음영으로 깔아 "언제 힘이 드는가"를 먼저 보인다
    a = fig.add_subplot(gs_[0, :])
    _shade(a, t, strike)
    a.plot(t[sl], fn_max[sl] * 1e3, lw=0.8, color="#d62728", label="법선 최대")
    a.plot(t[sl], fn_p95[sl] * 1e3, lw=0.8, color="#E69F00", label="법선 95%")
    a.plot(t[sl], fn_mean[sl] * 1e3, lw=0.9, color="#0072B2", label="법선 평균")
    a.plot(t[sl], ft_max[sl] * 1e3, lw=0.8, color="#009E73", ls="--", label="마찰 최대")
    a.plot(t[sl], ft_mean[sl] * 1e3, lw=0.9, color="#56B4E9", ls="--", label="마찰 평균")
    a.axhline(w1 * 1e3, color="k", ls=":", lw=1.0, label=f"1알 자중 {w1*1e3:.3f} mN")
    a.set_yscale("log"); a.set_ylabel("알당 힘 [mN]"); a.set_xlabel("t [s]")
    a.set_title("타격마다 접촉력이 얼마나 오르나 (음영 = 크랭크 토크 포화 구간)")
    a.legend(fontsize=8, ncol=3, loc="upper right")

    # (2) 타격 중 vs 무부하 — 분포로 직접 비교
    a = fig.add_subplot(gs_[1, 0])
    for m, lb, c in ((strike, "타격 중", "#d62728"), (~strike, "무부하", "#999999")):
        m = m.copy(); m[0] = False
        if m.sum() < 5:
            continue
        a.hist(np.log10(np.maximum(fn_max[m], 1e-12) * 1e3), bins=50, alpha=0.6,
               label=f"법선 {lb}", color=c)
    a.axvline(np.log10(w1 * 1e3), color="k", ls=":", lw=1.0)
    a.set_xlabel("log10( 알당 법선 최대 [mN] )"); a.set_ylabel("스텝 수")
    a.set_title("타격 중 vs 무부하 — 법선 접촉력 분포"); a.legend(fontsize=8)

    # (3) 마찰 대 법선 — **합력 비**다. Coulomb 한계(mu)로 묶이지 않는다.
    #     알 하나에 접촉 항목이 평균 9개 붙고 우리는 그걸 벡터합으로 모았다.
    #     마주보는 법선끼리는 상쇄되고 마찰은 안 상쇄되므로 비가 mu 를 넘는다.
    #     mu 는 **접촉 하나하나**에 걸리는 조건이지 합력 조건이 아니다.
    a = fig.add_subplot(gs_[1, 1])
    mu = float(d["grain_friction"])
    r = ft_max[sl] / np.maximum(fn_max[sl], 1e-15)
    a.plot(t[sl], r, lw=0.6, color="#009E73")
    a.axhline(mu, color="k", ls="--", lw=1.0, label=f"mu={mu} (접촉 단위 한계, 합력엔 미적용)")
    a.set_ylim(0, float(np.quantile(r, 0.995)) * 1.2)
    a.set_xlabel("t [s]"); a.set_ylabel("|합 마찰| / |합 법선|")
    a.set_title("마찰 대 법선 — 합력 비(알당 접촉 ~9개의 벡터합)")
    a.legend(fontsize=7)

    # (4) 접촉 상대 — 알끼리(PP) vs 알-면(PT/PE)
    a = fig.add_subplot(gs_[2, 0])
    if "cf_types" in d.files and len(d["cf_type_names"]):
        names = [str(x) for x in d["cf_type_names"]]
        cts = d["cf_types"]
        grp = {"알끼리 PP": [i for i, n in enumerate(names) if n.startswith("PP")],
               "알-면 PT": [i for i, n in enumerate(names) if n.startswith("PT")],
               "알-모서리 PE": [i for i, n in enumerate(names) if n.startswith("PE")],
               "알-바닥 PH": [i for i, n in enumerate(names) if n.startswith("PH")]}
        for lb, ii in grp.items():
            v = cts[:, ii].sum(axis=1)
            if v.max() > 0:
                a.plot(t, v, lw=0.8, label=lb)
        a.legend(fontsize=8)
    a.set_xlabel("t [s]"); a.set_ylabel("접촉 항목 수")
    a.set_title("누가 누구와 닿는가")

    # (5) 접촉 중인 알 수 + 무리 합력
    a = fig.add_subplot(gs_[2, 1])
    a.plot(t[sl], d["cf_n_cnt"][sl], lw=0.8, color="#0072B2", label="접촉 중인 알")
    a.set_ylabel("알 수", color="#0072B2"); a.set_xlabel("t [s]")
    a2 = a.twinx()
    a2.plot(t[sl], np.linalg.norm(d["cf_n_sum"][sl], axis=1), lw=0.7, color="#d62728",
            label="무리 법선 합력")
    a2.set_ylabel("|합력| [N]", color="#d62728")
    a.set_title(f"접촉 참여 알 수 / 무리 합력 (전체 {int(d['n_grains'])}알)")

    for ax_ in fig.axes:
        ax_.grid(alpha=0.25)
    out = os.path.splitext(path)[0] + "_contact.png"
    fig.savefig(out, facecolor="white", dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {out}")

    # ── 입자 스캐터 — 색이 곧 힘 ──────────────────────────────────────
    if "fld_fn" in d.files:
        R = float(d["grain_radius"]) * 1e3
        _particle_grid(d, path, _snap_indices(d, t), "fld_fn",
                       f"낟알별 **법선 접촉력** — 타격마다 (R={R:.1f}mm, {int(d['n_grains'])}알)",
                       "_particles_normal.png")
        _particle_grid(d, path, _snap_indices(d, t), "fld_ft",
                       f"낟알별 **마찰력** — 타격마다 (mu={float(d['grain_friction']):.1f})",
                       "_particles_friction.png")
        _particle_grid(d, path, _snap_within_strike(d, t), "fld_fn",
                       "낟알별 법선 접촉력 — **한 번의 타격 안에서** (접근→피크→이완)",
                       "_particles_onestrike.png")

    # ── 같은 힘을 **봉투 규격 격자**(64x90mm) 위에 얹는다 ─────────────
    # 3D 스캐터는 "어디가 맞는가"를 눈으로 못 읽는다. 봉투 면으로 투영하면
    # 봉투 어느 자리에 하중이 실리는지가 규격 좌표로 바로 나온다.
    if "fld_fn" in d.files and "bagv_pos" in d.files:
        R = float(d["grain_radius"]) * 1e3
        _bagface(d, path, _snap_indices(d, t), "fld_fn",
                 f"봉투 면({BAG_W_MM:.0f}x{BAG_H_MM:.0f}mm) 위 알별 **법선 접촉력** — "
                 f"타격마다 (R={R:.1f}mm, {int(d['n_grains'])}알)",
                 "_bagface_normal.png")
        _bagface(d, path, _snap_within_strike(d, t), "fld_fn",
                 f"봉투 면 위 알별 법선 접촉력 — **한 번의 타격 안에서**(접근→피크→이완)",
                 "_bagface_onestrike.png")
        _bagface_map(d, path, "fld_fn")

    st = strike.copy(); st[0] = False
    nl = ~strike; nl[0] = False
    print(f"[접촉] 법선 최대  타격중 {fn_max[st].max()*1e3:10.3f} mN | "
          f"무부하 {fn_max[nl].max()*1e3:10.3f} mN  "
          f"(비 {fn_max[st].max()/max(fn_max[nl].max(),1e-15):.1f}배)")
    print(f"[접촉] 법선 평균  타격중 {fn_mean[st].mean()*1e3:10.4f} mN | "
          f"무부하 {fn_mean[nl].mean()*1e3:10.4f} mN")
    print(f"[마찰] 최대      타격중 {ft_max[st].max()*1e3:10.3f} mN | "
          f"무부하 {ft_max[nl].max()*1e3:10.3f} mN")
    print(f"[마찰] |합마찰|/|합법선| 중앙 "
          f"{np.median(ft_max[sl]/np.maximum(fn_max[sl],1e-15)):.3f} "
          f"— mu={mu} 로 묶이지 않는다(접촉 ~9개의 벡터합이라 법선이 상쇄된다)")
    ssum = np.linalg.norm(d["cf_n_sum"], axis=1)
    print(f"[무리] 법선 합력 |sum f| 중앙 {np.median(ssum):.3f} N  최대 {ssum.max():.3f} N  "
          f"(전체 자중 {w1*int(d['n_grains']):.4f} N)")


def _shade(ax, t, mask):
    """True 구간에 음영."""
    idx = np.flatnonzero(mask)
    if not len(idx):
        return
    brk = np.flatnonzero(np.diff(idx) > 1)
    for g in np.split(idx, brk + 1):
        if len(g) > 3:
            ax.axvspan(t[g[0]], t[g[-1]], color="#d62728", alpha=0.10, lw=0)


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

    # ── 접촉력 / 마찰 전용 도면 (§27-11) ───────────────────────────────
    if "cf_n_max" in d.files:
        _plot_contact(d, f, t, deg, w1, m1)

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
