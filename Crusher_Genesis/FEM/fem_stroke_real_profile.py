"""
fem_stroke_real_profile.py — 실측 반력 프로파일 F(t) 추종 FEM 주응력장

임의 half-sine 펄스(fem_stroke_impact.py) 대신, 실기 Crusher 가 정제 위치에서
실제로 가한 힘 — ForceGage 실측 반력 프로파일(Real_result/반력프로파일/{θ}deg.txt,
v3 FSM STRIKE→stall→RETRACT 사이클) — 을 FEM 하중으로 추종한다.

하중 구동 방식 (SAP rigid 접촉 불가 → Dirichlet + 힘 추종)
----------------------------------------------------------
SAP soft contact 는 2 GPa 정제에 하중을 싣지 못하므로 (FEM.md, DigitalTwin.md §7-7)
타격면을 kinematic Dirichlet 로 구동하되, *변위가 아니라 힘*을 맞춘다:

  1. 프로브: 작은 변위 d_probe 를 가해 벽면 반력 적분 → 정제 강성 k = F/d 캘리브레이션
  2. 실측 사이클에서 대표 타격 1회를 추출, 시뮬 스텝에 리샘플
  3. 피드포워드  d(t) = F_meas(t) / k  (linear_corotated 준정적 → F ∝ d 성립)
  4. 매 스텝 벽면 경계 σ·n 적분으로 시뮬 반력 F_sim(t) 을 후처리 → 실측과 겹쳐 검증

  t=0        : 타격 직전 (F=0, 무응력)
  t=1..n-1   : STRIKE 상승 → stall plateau (crushing 지속) → RETRACT 해제
  t=n        : 타격 종료 (F=0 복귀)

출력
----
Sim_result/fem_real_profile_{θ}deg_<ts>_sigI_cut.png    σ_I  컷어웨이 + F(t) 추종 검증
Sim_result/fem_real_profile_{θ}deg_<ts>_sigIII_cut.png  σ_III 컷어웨이 + F(t) 추종 검증
Sim_result/fem_real_profile_{θ}deg_<ts>_snaps.npz       스냅샷 (재렌더용)
"""
import os, re, sys
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

from fem_stroke_impact import (_npy, _stress_per_tet, _vm, _principal,
                               _surface_faces, _cutaway_polys)

# ── 옵션 ────────────────────────────────────────────────────────
THETA        = "60deg"          # 사용할 실측 프로파일 (0/30/45/60/75/90/105/120deg)
DT, SUBSTEPS = 1e-3, 1
N_STEPS      = 2700             # 타격 사이클 시간 압축 (준정적 → 유효)
D_RATE_MAX   = 0.5e-6           # m/step — 슬루 제한 (급해제 시 요소 붕괴 방지)
N_PANELS     = 7
SAMPLE_EVERY = 5
TARGET_FACES = 3000
F_TH         = 20.0             # N — 타격 사이클 검출 임계값
PAD_S        = 1.0              # s — 사이클 앞뒤 여유
D_PROBE      = 5e-6             # m — 강성 캘리브레이션 프로브 변위

E_TABLET, NU_TABLET, RHO_TABLET = 2.0e9, 0.25, 1300.0

_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3
PROFILE_TXT  = os.path.join(_REPO_ROOT, "Crusher_Genesis", "Real_result",
                            "반력프로파일", f"{THETA}.txt")

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
BASE       = os.path.join(OUT_DIR, f"fem_real_profile_{THETA}_{_TS}")
PNG_SI_C   = BASE + "_sigI_cut.png"
PNG_SIII_C = BASE + "_sigIII_cut.png"
NPZ        = BASE + "_snaps.npz"


# ── 실측 프로파일 파싱 · 대표 사이클 추출 ────────────────────────
def load_profile(path):
    """ForceGage txt → (t[s], F[N]). Time 열의 h:mm:ss 로 실측 샘플레이트 추정."""
    F, sec = [], []
    with open(path, encoding="cp949", errors="replace") as f:
        for line in f:
            parts = [p.strip() for p in line.split("|")]
            if len(parts) < 4 or not parts[0].isdigit():
                continue
            mv = re.match(r"(-?\d+)\s*N", parts[2])
            mt = re.search(r"(\d+):(\d+):(\d+)", parts[1])
            if not (mv and mt):
                continue
            F.append(float(mv.group(1)))
            h, m, s = int(mt.group(1)), int(mt.group(2)), int(mt.group(3))
            sec.append(h * 3600 + m * 60 + s)
    F = np.array(F); sec = np.array(sec, dtype=float)
    # 12h/자정 롤오버 보정 후 초당 샘플수로 fs 추정
    for i in range(1, len(sec)):
        while sec[i] < sec[i - 1] - 1:
            sec[i] += 12 * 3600
    dur = sec[-1] - sec[0]
    fs = len(F) / max(dur, 1.0)
    t = np.arange(len(F)) / fs
    return t, F, fs


def extract_cycle(t, F, fs):
    """F>F_TH 연속 구간들 중 대표(피크가 중앙값에 가장 가까운) 사이클 1개 + 앞뒤 PAD."""
    on = F > F_TH
    edges = np.flatnonzero(np.diff(on.astype(int)))
    starts = edges[on[edges + 1]] + 1
    ends   = edges[~on[edges + 1]] + 1
    if on[0]:  starts = np.r_[0, starts]
    if on[-1]: ends = np.r_[ends, len(F)]
    runs = [(s, e) for s, e in zip(starts, ends) if e - s >= int(1.0 * fs)]
    assert runs, "타격 사이클을 찾지 못함"
    peaks = np.array([F[s:e].max() for s, e in runs])
    k = int(np.argmin(np.abs(peaks - np.median(peaks))))
    s, e = runs[k]
    pad = int(PAD_S * fs)
    s0, e0 = max(0, s - pad), min(len(F), e + pad)
    print(f"[profile] {len(runs)} cycles, peaks median={np.median(peaks):.0f} N → "
          f"cycle #{k} (peak {peaks[k]:.0f} N, {t[e0-1]-t[s0]:.1f} s)")
    return t[s0:e0] - t[s0], F[s0:e0]


def _phase_steps(F_step):
    """리샘플된 F 에서 7 시점: 직전/상승/stall 시작·중·끝/해제/종료"""
    pk = F_step.max()
    hi = np.flatnonzero(F_step >= 0.95 * pk)
    nz = np.flatnonzero(F_step > F_TH)
    s_pre  = max(nz[0] - 1, 0)
    s_rise = nz[0] + int(np.argmax(F_step[nz[0]:hi[0] + 1] >= 0.5 * pk)) if hi[0] > nz[0] else nz[0]
    s_ps, s_pm, s_pe = hi[0], hi[len(hi) // 2], hi[-1]
    after = np.flatnonzero(F_step[s_pe:] <= 0.5 * pk)
    s_rel  = s_pe + (after[0] if len(after) else 0)
    s_end  = len(F_step) - 1
    steps = sorted(set([s_pre, s_rise, s_ps, s_pm, s_pe, s_rel, s_end]))
    labels = {s_pre: "t=0 (직전)", s_rise: "상승", s_ps: "stall 시작", s_pm: "stall 중",
              s_pe: "stall 끝", s_rel: "해제", s_end: "t=n (종료)"}
    return steps, labels


def main():
    print("=" * 64)
    print(f" FEM Real-Profile Stroke — {THETA} 실측 반력 추종 주응력장")
    print("=" * 64)

    t_cyc, F_cyc, fs = load_profile(PROFILE_TXT)
    print(f"[profile] {PROFILE_TXT}")
    print(f"[profile] {len(F_cyc)} samples @ {fs:.1f} Hz, F_max={F_cyc.max():.0f} N")
    t_cyc, F_cyc = extract_cycle(t_cyc, F_cyc, fs)
    T_CYC = t_cyc[-1]
    # 시뮬 스텝으로 리샘플 (시간 압축; 준정적이라 응력은 순시 F 만의 함수)
    F_step = np.interp(np.linspace(0, T_CYC, N_STEPS + 1), t_cyc, F_cyc)
    t_real = np.linspace(0, T_CYC, N_STEPS + 1)      # 실측 시간축 (라벨용)
    panel_steps, panel_labels = _phase_steps(F_step)

    # ── 메쉬 & 씬 (fem_stroke_impact.py 와 동일 셋업) ──
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

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, 0)),
        fem_options=gs.options.FEMOptions(use_implicit_solver=True,
                                          enable_vertex_constraints=True, pcg_threshold=1e-10),
        show_viewer=False,
    )
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET,
                                          model="linear_corotated", friction_mu=1.0),
        morph=gs.morphs.Mesh(file=STL_TMP, scale=TABLET_SCALE, pos=tablet_pos))
    scene.build(n_envs=0)

    el2v = _npy(tablet.get_el2v()).astype(np.int64)
    pos0 = _npy(tablet.get_state().pos).squeeze()
    N = len(pos0)
    if el2v.min() != 0 or el2v.max() >= N:
        el2v = el2v - el2v.min()
    print(f"[mesh] nodes={N}  tets={len(el2v)}")

    z = pos0[:, 2]; band = (z.max() - z.min()) * 0.10
    strike_idx = np.where(z > z.max() - band)[0]
    wall_idx   = np.where(z < z.min() + band)[0]
    strike_pos0 = pos0[strike_idx].copy()
    tablet.set_vertex_constraints(wall_idx.tolist())
    tablet.set_vertex_constraints(strike_idx.tolist())
    H0 = float(pos0[strike_idx, 2].mean() - pos0[wall_idx, 2].mean())

    # 벽면 경계 삼각형 (반력 적분용): 세 꼭짓점 모두 wall band 에 속한 boundary face
    surf_faces, surf_tet = _surface_faces(el2v)
    wall_set = set(wall_idx.tolist())
    wsel = np.array([all(v in wall_set for v in f) for f in surf_faces])
    w_faces, w_tets = surf_faces[wsel], surf_tet[wsel]
    v1 = pos0[w_faces[:, 1]] - pos0[w_faces[:, 0]]
    v2 = pos0[w_faces[:, 2]] - pos0[w_faces[:, 0]]
    nA = 0.5 * np.cross(v1, v2)                     # 면적 가중 법선
    ctr_face = pos0[w_faces].mean(axis=1)
    out = np.sign(((ctr_face - pos0.mean(0)) * nA).sum(axis=1))
    nA *= out[:, None]                              # 바깥쪽으로 정렬
    print(f"[bc] wall faces={len(w_faces)}  strike nodes={len(strike_idx)}  H0={H0*1e3:.3f} mm")

    def _wall_F(sig):
        """벽면 traction σ·n̂ 적분 → 벽이 받는 수직력 [N] (압축 양수)
        t=σ·n (n: 바깥법선≈-z) 은 벽이 정제에 주는 힘/면적 — 압축이면 t_z>0."""
        tr = np.einsum("fij,fj->fi", sig[w_tets], nA)
        return tr[:, 2].sum()

    def _drive(d):
        tg = strike_pos0.copy(); tg[:, 2] -= d
        tablet.update_constraint_targets(strike_idx.tolist(), tg)
        scene.step()

    # ── 1) 강성 캘리브레이션 프로브 ──
    print(f"\n[probe] d={D_PROBE*1e6:.1f} μm 램프로 k 측정...")
    for i in range(60): _drive(D_PROBE * (i + 1) / 60)
    for _ in range(20): _drive(D_PROBE)
    pos = _npy(tablet.get_state().pos).squeeze()
    F_probe = _wall_F(_stress_per_tet(pos0, pos, el2v))
    k_stiff = F_probe / D_PROBE
    print(f"[probe] F={F_probe:.1f} N → k={k_stiff:.3e} N/m  "
          f"(F_max={F_step.max():.0f} N → d_max={F_step.max()/k_stiff*1e6:.1f} μm, "
          f"ε_nom={F_step.max()/k_stiff/H0*100:.2f}%)")
    for i in range(30): _drive(D_PROBE * (29 - i) / 30)   # 원위치
    for _ in range(10): _drive(0.0)

    # ── 2) 실측 프로파일 추종 ──
    snaps, trace = {}, []
    d_prev = 0.0
    print(f"\n[run] {N_STEPS} steps — {THETA} 사이클 {T_CYC:.1f} s 추종 "
          f"(슬루 {D_RATE_MAX*1e6:.1f} μm/step)...")
    for step in range(N_STEPS + 1):
        d_cmd = np.clip(F_step[step] / k_stiff, d_prev - D_RATE_MAX, d_prev + D_RATE_MAX)
        d_prev = d_cmd
        if step > 0:
            _drive(d_cmd)
        need_snap  = step in panel_steps
        need_trace = (step % SAMPLE_EVERY == 0) or need_snap
        if need_trace:
            pos = _npy(tablet.get_state().pos).squeeze()
            sig = _stress_per_tet(pos0, pos, el2v)
            pr = _principal(sig)
            F_sim = _wall_F(sig)
            trace.append((t_real[step], F_step[step], F_sim, pr[:, -1].max(), pr[:, 0].min()))
            if need_snap:
                snaps[step] = pos.copy()
                print(f"  [panel] {panel_labels[step]:<10s} t={t_real[step]:5.2f}s  "
                      f"F_meas={F_step[step]:6.1f}N  F_sim={F_sim:6.1f}N  "
                      f"σ_I max={pr[:,-1].max()/1e6:+.1f}  σ_III min={pr[:,0].min()/1e6:+.1f} MPa")
    trace = np.array(trace)
    err = trace[:, 2] - trace[:, 1]
    print(f"[track] F RMS err={np.sqrt((err**2).mean()):.1f} N "
          f"(peak {trace[:,1].max():.0f} N 의 {np.sqrt((err**2).mean())/trace[:,1].max()*100:.1f}%)")

    # ── 3) 패널 & 렌더 ──
    y_cut = float(np.median(pos0[:, 1]))
    panels = []
    for s in panel_steps:
        sig = _stress_per_tet(pos0, snaps[s], el2v)
        pr = _principal(sig)
        panels.append(dict(step=s, t=t_real[s], F=F_step[s], label=panel_labels[s],
                           pos=snaps[s], sI=pr[:, -1], sIII=pr[:, 0]))

    peak = max(panels, key=lambda p: p["F"])
    smax = max(np.percentile(np.abs(peak["sI"]), 75) / 1e6, 1e-3)
    s3max = max(np.percentile(np.abs(peak["sIII"]), 75) / 1e6, 1e-3)
    cmap_s = matplotlib.colormaps["seismic"]
    norm_si = colors.Normalize(-smax, smax)
    norm_s3 = colors.Normalize(-s3max, s3max)

    lo, hi = pos0.min(0) * 1e3, pos0.max(0) * 1e3
    ext = hi - lo
    hdr = (f"실측 반력 {THETA} (v3 FSM, ForceGage {fs:.0f} Hz) 추종  |  "
           f"E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  k={k_stiff:.2e} N/m  "
           f"F_pk={F_step.max():.0f} N (d={F_step.max()/k_stiff*1e6:.1f} μm)  nodes={N}")

    def _render(path, key, norm_s, cbar_label, title, stat_fn):
        n = len(panels)
        fig = plt.figure(figsize=(3.1 * n, 5.6))
        gs_ = fig.add_gridspec(2, n, height_ratios=[2.4, 1.0], hspace=0.30, top=0.80)
        for i, pn in enumerate(panels):
            ax = fig.add_subplot(gs_[0, i], projection="3d")
            verts, fvals = _cutaway_polys(pn["pos"], el2v, pn[key], y_cut)
            ax.add_collection3d(Poly3DCollection(verts, facecolors=cmap_s(norm_s(fvals / 1e6)),
                                                 edgecolors="k", linewidths=0.02))
            wall_quad = [[(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
                          (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2])]]
            ax.add_collection3d(Poly3DCollection(wall_quad, facecolors="0.7", alpha=0.5))
            ax.quiver(0, y_cut * 1e3, hi[2] + 1.1, 0, 0, -0.9,
                      color="k", arrow_length_ratio=0.35, lw=1.8)
            m = 0.06
            ax.set_xlim(lo[0] - m * ext[0], hi[0] + m * ext[0])
            ax.set_ylim(lo[1] - m * ext[1], hi[1] + m * ext[1])
            ax.set_zlim(lo[2] - m * ext[2] - 0.4, hi[2] + m * ext[2] + 1.2)
            ax.set_box_aspect((ext[0], ext[1], ext[2] + 1.6))
            ax.view_init(elev=18, azim=105)
            ax.set_axis_off()
            ax.set_title(f"{pn['label']}   t={pn['t']:.2f} s\n"
                         f"F={pn['F']:.0f} N\n{stat_fn(pn)}", fontsize=8.5)
            if i == 0:
                ax.text2D(0.5, 0.86, "stroke ↓", transform=ax.transAxes,
                          ha="center", fontsize=8, color="k")
                ax.text2D(0.5, 0.04, "wall (고정)", transform=ax.transAxes,
                          ha="center", fontsize=8, color="0.35")
        axd = fig.add_subplot(gs_[1, :])
        axd.plot(trace[:, 0], trace[:, 1], "k--", lw=1.6, label=f"F 실측 ({THETA})")
        axd.plot(trace[:, 0], trace[:, 2], "r-", lw=1.1, alpha=0.9, label="F 시뮬 (벽 반력 적분)")
        axd.set_xlabel("t [s] (실측 시간축)", fontsize=9)
        axd.set_ylabel("F [N]", fontsize=9)
        for pn in panels:
            axd.axvline(pn["t"], color="0.6", ls=":", lw=0.8)
        axd.legend(fontsize=8, loc="upper right")
        axd.tick_params(labelsize=8)
        sm = cm.ScalarMappable(norm=norm_s, cmap=cmap_s); sm.set_array([])
        fig.colorbar(sm, ax=fig.axes[:n], fraction=0.02, pad=0.02).set_label(cbar_label)
        fig.suptitle(title + "\n" + hdr, fontsize=10)
        plt.savefig(path, dpi=200, bbox_inches="tight"); plt.close()

    _render(PNG_SI_C, "sI", norm_si,
            "max principal $\\sigma_I$ [MPa]  (+ tension / - compression)",
            f"FEM — 실측 반력 프로파일({THETA}) 추종 시 $\\sigma_I$ 주응력장 (cutaway)",
            lambda pn: f"$\\sigma_I^{{max}}$={pn['sI'].max()/1e6:+.1f} MPa")
    _render(PNG_SIII_C, "sIII", norm_s3,
            "min principal $\\sigma_{III}$ [MPa]  (+ tension / - compression)",
            f"FEM — 실측 반력 프로파일({THETA}) 추종 시 $\\sigma_{{III}}$ 주응력장 (cutaway)",
            lambda pn: f"$\\sigma_{{III}}^{{min}}$={pn['sIII'].min()/1e6:+.1f} MPa")

    np.savez_compressed(NPZ, pos0=pos0, el2v=el2v, trace=trace,
                        panel_steps=np.array([p["step"] for p in panels]),
                        F_step=F_step, t_real=t_real, k_stiff=k_stiff,
                        **{f"pos_{p['step']}": p["pos"] for p in panels})

    print(f"\n[saved] σ_I  : {PNG_SI_C}")
    print(f"[saved] σ_III: {PNG_SIII_C}")
    print(f"[saved] snaps: {NPZ}")


if __name__ == "__main__":
    main()
