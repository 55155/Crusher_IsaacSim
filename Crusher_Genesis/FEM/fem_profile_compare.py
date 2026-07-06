"""
fem_profile_compare.py — real vs sim 반력 프로파일 구동 시 주응력장 오차 비교

동기
----
디지털 트윈은 실기 ForceGage(real) 가 아니라 *시뮬레이터가 만든 반력*(sim,
Crusher_8env.py 의 MuJoCo 벽 반력)으로 계속 돌려서 검증해야 한다. 그래서 같은 FEM
정제를 (a) real F(t), (b) sim F(t) 두 프로파일로 각각 구동해 주응력장을 뽑고,
둘의 오차를 정량화한다.

"3D 응력장의 오차를 어떻게 그리나?" — 3단 접근
---------------------------------------------
FEM 이 선형 탄성·준정적이므로  σ(x,t) = σ_shape(x)·F(t)/k  로, 공간패턴 σ_shape 는
구동원(real/sim)과 무관하게 동일하다. 따라서 필드 오차는 아래로 응축된다:
  (A) 시간영역   — F(t)·σ_I,max(t) 를 real/sim 겹쳐 → "언제·얼마나" (프로파일 오차)
  (B) 공간영역   — Δσ_I(x)=σ_sim−σ_real 컷어웨이 (peak) → "어디서" (패턴이 같으면 스케일 복제)
  (C) 분포/스칼라 — tet별 σ_real vs σ_sim 산점도 + R²·기울기·RMSE → "전체 일치도 한 장"

입력
----
real: Real_result/반력프로파일/60deg.txt (ForceGage)
sim : Sim_result/sim8env/60deg_sim_Ft.csv (extract_sim_profile.py 로 생성)

출력
----
Sim_result/fem_compare_{θ}deg_<ts>_fields.png   real vs sim σ_I 컷어웨이 (peak, 동일 스케일)
Sim_result/fem_compare_{θ}deg_<ts>_error.png    (A)(B)(C) 오차 분석
"""
import os, sys, csv
from datetime import datetime
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Malgun Gothic"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

from fem_stroke_impact import (_npy, _stress_per_tet, _principal,
                               _surface_faces, _cutaway_polys)
from fem_stroke_real_profile import load_profile, extract_cycle

# ── 옵션 ────────────────────────────────────────────────────────
THETA        = "60deg"
DT, SUBSTEPS = 1e-3, 1
N_STEPS      = 2700
SAMPLE_EVERY = 5
# sim strike 앞뒤 zero-padding. 작을수록 strike 가 정규 사이클의 더 큰 비율을 차지 →
# 상승·유지 구간이 길어짐. 0.11 → real 밴드폭(≈0.41)과 매칭 (기존 0.30 은 0.24 로 짧음).
SIM_PAD_S    = 0.11
# 언로딩 컴플라이언스: sim 뒷엣지(접촉 해제) 완만화. 강체 MuJoCo 접촉은 분리가 불연속
# 이라 F 가 계단처럼 급락한다. 실제 프레임/센서 컴플라이언스를 모사해 하강 구간에만
# 1차 지연(직렬 스프링 이완)을 걸어 real 처럼 완만하게. τ(정규위상) 단위 시상수.
# 0 이면 비활성. 0.035 → 해제 Δτ(90→10%)≈0.08 로 real 뒷엣지(0.080)와 매칭.
TAU_UNLOAD   = 0.035
TARGET_FACES = 3000
D_PROBE      = 5e-6
D_RATE_MAX   = 0.5e-6
E_TABLET, NU_TABLET, RHO_TABLET = 2.0e9, 0.25, 1300.0

_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3
REAL_TXT     = os.path.join(_REPO_ROOT, "Crusher_Genesis", "Real_result", "반력프로파일", f"{THETA}.txt")
SIM_CSV      = os.path.join(_REPO_ROOT, "Crusher_Genesis", "Sim_result", "sim8env", f"{THETA}_sim_Ft.csv")

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
PNG_FIELDS = os.path.join(OUT_DIR, f"fem_compare_{THETA}_{_TS}_fields.png")
PNG_ERROR  = os.path.join(OUT_DIR, f"fem_compare_{THETA}_{_TS}_error.png")


def load_sim_csv(path):
    t, F = [], []
    with open(path) as fh:
        r = csv.reader(fh); next(r)
        for row in r:
            t.append(float(row[0])); F.append(float(row[1]))
    t = np.array(t); F = np.array(F)
    fs = len(t) / max(t[-1] - t[0], 1.0)
    return t - t[0], F, fs


def extract_sim_cycle(t, F, fs, f_th=20.0, min_dur=0.05, pad_s=0.30):
    """sim 반력은 stall에서 STRIKE를 끊어 짧은 스파이크(≈254ms). 대표 strike(peak가
    중앙값에 가장 가까운 것) 1개 + 앞뒤 pad. real 의 extract_cycle 과 동일 역할이나
    min-길이 제약만 완화."""
    on = F > f_th
    edges = np.flatnonzero(np.diff(on.astype(int)))
    starts = edges[on[edges + 1]] + 1
    ends = edges[~on[edges + 1]] + 1
    if on[0]: starts = np.r_[0, starts]
    if on[-1]: ends = np.r_[ends, len(F)]
    runs = [(s, e) for s, e in zip(starts, ends) if (e - s) >= int(min_dur * fs)]
    assert runs, "sim 타격 스파이크를 찾지 못함"
    peaks = np.array([F[s:e].max() for s, e in runs])
    k = int(np.argmin(np.abs(peaks - np.median(peaks))))
    s, e = runs[k]; pad = int(pad_s * fs)
    s0, e0 = max(0, s - pad), min(len(F), e + pad)
    print(f"[sim] {len(runs)} strikes, peaks median={np.median(peaks):.0f} N → "
          f"strike #{k} (peak {peaks[k]:.0f} N, {t[e0-1]-t[s0]:.2f} s)")
    return t[s0:e0] - t[s0], F[s0:e0]


def resample_cycle(t_cyc, F_cyc, n_steps):
    """대표 사이클을 n_steps+1 로 리샘플 (시간축은 [0,1] 정규 위상)."""
    T = t_cyc[-1]
    tau = np.linspace(0, 1, n_steps + 1)
    F = np.interp(tau * T, t_cyc, F_cyc)
    return tau, F, T


def apply_unload_compliance(F_step, tau_unload, n_steps=N_STEPS):
    """언로딩(하강) 구간에만 1차 지연(직렬 컴플라이언스)을 적용해 뒷엣지를 완만하게.
    상승/유지는 raw 를 즉시 추종(로딩 매칭 보존), 하강 시에만 시상수 tau_unload(τ 단위)
    로 이완. → 강체 접촉의 계단형 해제를 실제 프레임/센서 컴플라이언스처럼 부드럽게."""
    if tau_unload <= 0:
        return F_step
    dtau = 1.0 / n_steps
    a = dtau / (tau_unload + dtau)          # 1차 IIR 계수 (Δτ/(τ_c+Δτ))
    out = F_step.copy()
    for i in range(1, len(out)):
        out[i] = F_step[i] if F_step[i] >= out[i - 1] else out[i - 1] + a * (F_step[i] - out[i - 1])
    return out


# ── FEM 씬(전역 1회 빌드용) ──────────────────────────────────────
_G = {}   # genesis 씬 캐시


def _build_scene():
    import genesis as gs
    if not _G.get("init"):
        gs.init(backend=gs.gpu, logging_level="warning", precision="64")
        _G["init"] = True
    raw = tm.load(TABLET_STL)
    raw.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    if len(raw.faces) > TARGET_FACES:
        try: raw = raw.simplify_quadric_decimation(face_count=TARGET_FACES)
        except Exception as e: print(f"[decimate][warn] {e}")
    STL_TMP = os.path.join(OUT_DIR, f"_tablet_stress_{TARGET_FACES}.stl")
    raw.export(STL_TMP)
    bb = raw.bounding_box.bounds * TABLET_SCALE
    tablet_pos = (-0.5 * (bb[0][0] + bb[1][0]), -0.5 * (bb[0][1] + bb[1][1]),
                  -0.5 * (bb[0][2] + bb[1][2]))
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, 0)),
        fem_options=gs.options.FEMOptions(use_implicit_solver=True,
                                          enable_vertex_constraints=True, pcg_threshold=1e-10),
        show_viewer=False)
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET,
                                          model="linear_corotated", friction_mu=1.0),
        morph=gs.morphs.Mesh(file=STL_TMP, scale=TABLET_SCALE, pos=tablet_pos))
    scene.build(n_envs=0)
    return scene, tablet


def run_fem(F_step, label):
    """F_step(N,) 프로파일로 FEM 구동 → dict(trace, peak σ 필드, mesh)."""
    scene, tablet = _build_scene()
    el2v = _npy(tablet.get_el2v()).astype(np.int64)
    pos0 = _npy(tablet.get_state().pos).squeeze()
    N = len(pos0)
    if el2v.min() != 0 or el2v.max() >= N:
        el2v = el2v - el2v.min()
    z = pos0[:, 2]; band = (z.max() - z.min()) * 0.10
    strike_idx = np.where(z > z.max() - band)[0]
    wall_idx = np.where(z < z.min() + band)[0]
    strike_pos0 = pos0[strike_idx].copy()
    tablet.set_vertex_constraints(wall_idx.tolist())
    tablet.set_vertex_constraints(strike_idx.tolist())
    H0 = float(pos0[strike_idx, 2].mean() - pos0[wall_idx, 2].mean())

    # 벽 반력 적분자
    surf_faces, surf_tet = _surface_faces(el2v)
    wall_set = set(wall_idx.tolist())
    wsel = np.array([all(v in wall_set for v in f) for f in surf_faces])
    w_faces, w_tets = surf_faces[wsel], surf_tet[wsel]
    nA = 0.5 * np.cross(pos0[w_faces[:, 1]] - pos0[w_faces[:, 0]],
                        pos0[w_faces[:, 2]] - pos0[w_faces[:, 0]])
    out = np.sign(((pos0[w_faces].mean(1) - pos0.mean(0)) * nA).sum(1))
    nA *= out[:, None]

    def wall_F(sig):
        return np.einsum("fij,fj->fi", sig[w_tets], nA)[:, 2].sum()

    def drive(d):
        tg = strike_pos0.copy(); tg[:, 2] -= d
        tablet.update_constraint_targets(strike_idx.tolist(), tg)
        scene.step()

    # 강성 캘리브
    for i in range(60): drive(D_PROBE * (i + 1) / 60)
    for _ in range(20): drive(D_PROBE)
    pos = _npy(tablet.get_state().pos).squeeze()
    k = wall_F(_stress_per_tet(pos0, pos, el2v)) / D_PROBE
    for i in range(30): drive(D_PROBE * (29 - i) / 30)
    for _ in range(10): drive(0.0)

    peak_step = int(np.argmax(F_step))
    trace, d_prev = [], 0.0
    peak = None
    print(f"[{label}] k={k:.3e} N/m  F_pk={F_step.max():.0f} N  "
          f"d_pk={F_step.max()/k*1e6:.1f} μm ...")
    for step in range(N_STEPS + 1):
        d_cmd = np.clip(F_step[step] / k, d_prev - D_RATE_MAX, d_prev + D_RATE_MAX)
        d_prev = d_cmd
        if step > 0: drive(d_cmd)
        if step == peak_step or step % SAMPLE_EVERY == 0:
            pos = _npy(tablet.get_state().pos).squeeze()
            sig = _stress_per_tet(pos0, pos, el2v)
            pr = _principal(sig)
            trace.append((step / N_STEPS, F_step[step], wall_F(sig),
                          pr[:, -1].max(), pr[:, 0].min()))
            if step == peak_step:
                peak = dict(tau=step / N_STEPS, F=F_step[step], pos=pos.copy(),
                            sI=pr[:, -1].copy(), sIII=pr[:, 0].copy())
    return dict(pos0=pos0, el2v=el2v, k=k, H0=H0, trace=np.array(trace),
                peak=peak, label=label)


def main():
    print("=" * 64)
    print(f" FEM Profile Compare — {THETA}: real vs sim 반력 구동 주응력장 오차")
    print("=" * 64)

    # 프로파일 로드 + 대표 사이클
    tr, Fr, fsr = load_profile(REAL_TXT)
    tr, Fr = extract_cycle(tr, Fr, fsr)
    _, Fr_step, Tr = resample_cycle(tr, Fr, N_STEPS)
    ts, Fs, fss = load_sim_csv(SIM_CSV)
    ts, Fs = extract_sim_cycle(ts, Fs, fss, pad_s=SIM_PAD_S)
    _, Fs_step, Ts = resample_cycle(ts, Fs, N_STEPS)
    Fs_step = apply_unload_compliance(Fs_step, TAU_UNLOAD)   # 뒷엣지 컴플라이언스
    print(f"[real] peak={Fr_step.max():.1f} N  cycle={Tr:.2f} s  ({fsr:.0f} Hz)")
    print(f"[sim ] peak={Fs_step.max():.1f} N  cycle={Ts:.2f} s  ({fss:.0f} Hz)")

    R = run_fem(Fr_step, "real")
    S = run_fem(Fs_step, "sim")

    el2v, pos0 = R["el2v"], R["pos0"]
    y_cut = float(np.median(pos0[:, 1]))
    lo, hi = pos0.min(0) * 1e3, pos0.max(0) * 1e3
    ext = hi - lo

    pr_sI, ps_sI = R["peak"]["sI"], S["peak"]["sI"]
    pr_s3, ps_s3 = R["peak"]["sIII"], S["peak"]["sIII"]

    # ── 오차 지표 (peak 순간, tet 단위) ──
    def metrics(a, b):
        rmse = np.sqrt(np.mean((b - a) ** 2))
        rel = np.linalg.norm(b - a) / np.linalg.norm(a)
        slope = np.dot(a, b) / np.dot(a, a)               # 원점 통과 최소자승 기울기
        ss = 1 - np.sum((b - a) ** 2) / np.sum((a - a.mean()) ** 2)
        return rmse, rel, slope, ss
    rmse_I, rel_I, slope_I, r2_I = metrics(pr_sI, ps_sI)
    print(f"\n[error@peak σ_I ] RMSE={rmse_I/1e6:.2f} MPa  rel-L2={rel_I*100:.1f}%  "
          f"slope(sim/real)={slope_I:.3f}  R²(vs y=x)={r2_I:.4f}")
    print(f"[error peak mag ] σ_I,max real={pr_sI.max()/1e6:.1f}  sim={ps_sI.max()/1e6:.1f} MPa "
          f"({(ps_sI.max()/pr_sI.max()-1)*100:+.1f}%)")
    print(f"[error peak mag ] σ_III,min real={pr_s3.min()/1e6:.1f} sim={ps_s3.min()/1e6:.1f} MPa "
          f"({(ps_s3.min()/pr_s3.min()-1)*100:+.1f}%)")

    # 공통 컬러 스케일 (real peak 75%)
    smax = max(np.percentile(np.abs(pr_sI), 75) / 1e6, 1e-3)
    norm = colors.Normalize(-smax, smax); cmap = matplotlib.colormaps["seismic"]

    def _cutaway(ax, pos, vals, norm_, cmap_):
        verts, fvals = _cutaway_polys(pos, el2v, vals, y_cut)
        ax.add_collection3d(Poly3DCollection(verts, facecolors=cmap_(norm_(fvals / 1e6)),
                                             edgecolors="k", linewidths=0.02))
        wq = [[(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
               (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2])]]
        ax.add_collection3d(Poly3DCollection(wq, facecolors="0.7", alpha=0.5))
        ax.quiver(0, y_cut * 1e3, hi[2] + 1.1, 0, 0, -0.9, color="k",
                  arrow_length_ratio=0.35, lw=1.8)
        m = 0.06
        ax.set_xlim(lo[0] - m * ext[0], hi[0] + m * ext[0])
        ax.set_ylim(lo[1] - m * ext[1], hi[1] + m * ext[1])
        ax.set_zlim(lo[2] - m * ext[2] - 0.4, hi[2] + m * ext[2] + 1.2)
        ax.set_box_aspect((ext[0], ext[1], ext[2] + 1.6)); ax.view_init(elev=18, azim=105)
        ax.set_axis_off()

    # ── Fig 1: 필드 side-by-side (σ_I, peak) ──
    fig = plt.figure(figsize=(9, 4.8))
    for j, (D, tag) in enumerate([(R, "real (ForceGage)"), (S, "sim (MuJoCo)")]):
        ax = fig.add_subplot(1, 2, j + 1, projection="3d")
        _cutaway(ax, D["peak"]["pos"], D["peak"]["sI"], norm, cmap)
        ax.set_title(f"{tag}\nF_pk={D['peak']['F']:.0f} N   "
                     f"$\\sigma_I^{{max}}$={D['peak']['sI'].max()/1e6:.0f} MPa", fontsize=10)
    sm = cm.ScalarMappable(norm=norm, cmap=cmap); sm.set_array([])
    fig.colorbar(sm, ax=fig.axes, fraction=0.02, pad=0.02).set_label(
        "max principal $\\sigma_I$ [MPa]")
    fig.suptitle(f"FEM principal stress field $\\sigma_I$ — real vs sim reaction-driven "
                 f"({THETA}, peak instant)", fontsize=11)
    plt.savefig(PNG_FIELDS, dpi=200, bbox_inches="tight"); plt.close()

    # ── Fig 2: 오차 분석 (A)(B)(C) ──
    fig = plt.figure(figsize=(16.5, 5.0))
    gsr = fig.add_gridspec(1, 3, width_ratios=[1.25, 1.0, 1.0], wspace=0.28)

    # (A) 시간영역: F(t) + σ_I,max(t)
    axA = fig.add_subplot(gsr[0, 0])
    axA.plot(R["trace"][:, 0], R["trace"][:, 1], "k--", lw=1.4, label="F real")
    axA.plot(S["trace"][:, 0], S["trace"][:, 1], color="0.45", ls="-", lw=1.4, label="F sim")
    axA.set_xlabel("Normalized phase τ (cycle = 1)", fontsize=9); axA.set_ylabel("F [N]", fontsize=9)
    axB2 = axA.twinx()
    axB2.plot(R["trace"][:, 0], R["trace"][:, 3] / 1e6, "b-", lw=1.2, label="$\\sigma_I^{max}$ real")
    axB2.plot(S["trace"][:, 0], S["trace"][:, 3] / 1e6, "r-", lw=1.2, label="$\\sigma_I^{max}$ sim")
    axB2.set_ylabel("$\\sigma_I^{max}$ [MPa]", fontsize=9)
    h1, l1 = axA.get_legend_handles_labels(); h2, l2 = axB2.get_legend_handles_labels()
    axA.legend(h1 + h2, l1 + l2, fontsize=7.5, loc="upper right", ncol=2)
    axA.set_title("(A) Time domain — profile & peak stress (when & how much)", fontsize=10)
    axA.tick_params(labelsize=8); axB2.tick_params(labelsize=8)

    # (B) 공간 Δ 필드: σ_I,sim − σ_I,real (peak), sim peak 위치로 렌더
    dmax = max(np.percentile(np.abs(ps_sI - pr_sI), 95) / 1e6, 1e-3)
    normd = colors.Normalize(-dmax, dmax); cmapd = matplotlib.colormaps["PuOr_r"]
    axB = fig.add_subplot(gsr[0, 1], projection="3d")
    _cutaway(axB, S["peak"]["pos"], ps_sI - pr_sI, normd, cmapd)
    axB.set_title(f"(B) Spatial error Δ$\\sigma_I$ = sim - real (peak, where)\n"
                  f"rel-L2={rel_I*100:.1f}%  RMSE={rmse_I/1e6:.1f} MPa", fontsize=10)
    smd = cm.ScalarMappable(norm=normd, cmap=cmapd); smd.set_array([])
    fig.colorbar(smd, ax=axB, fraction=0.03, pad=0.04).set_label("Δ$\\sigma_I$ [MPa]", fontsize=9)

    # (C) tet 산점도
    axC = fig.add_subplot(gsr[0, 2])
    axC.scatter(pr_sI / 1e6, ps_sI / 1e6, s=3, alpha=0.25, color="#1f77b4", ec="none")
    lim = np.array([min(pr_sI.min(), ps_sI.min()), max(pr_sI.max(), ps_sI.max())]) / 1e6
    axC.plot(lim, lim, "k--", lw=1, label="y = x (perfect match)")
    axC.plot(lim, slope_I * lim, "r-", lw=1.2, label=f"fit: {slope_I:.3f}·x")
    axC.set_xlabel("$\\sigma_I$ real [MPa]", fontsize=9); axC.set_ylabel("$\\sigma_I$ sim [MPa]", fontsize=9)
    axC.set_aspect("equal"); axC.legend(fontsize=8, loc="upper left")
    axC.set_title(f"(C) Per-tet agreement (all-in-one)\n"
                  f"slope={slope_I:.3f}  R²={r2_I:.4f}", fontsize=10)
    axC.tick_params(labelsize=8); axC.grid(True, alpha=0.3)

    fig.suptitle(f"real vs sim reaction-driven FEM principal-stress-field error ({THETA})  —  "
                 f"linear-elastic quasi-static: field error = reaction-profile error × fixed spatial response",
                 fontsize=11)
    plt.savefig(PNG_ERROR, dpi=200, bbox_inches="tight"); plt.close()

    print(f"\n[saved] fields : {PNG_FIELDS}")
    print(f"[saved] error  : {PNG_ERROR}")


if __name__ == "__main__":
    main()
