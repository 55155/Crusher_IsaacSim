"""
fem_stroke_impact.py — Stroke 타격 펄스 동안 정제 내부 주응력장(σ_I) 시계열

시나리오
--------
정제가 벽에 붙어 있고(벽쪽 표면 고정), 반대쪽에서 스트로크가 타격한다.
fem_stress_field.py 와 동일하게 SAP rigid 접촉 대신 *kinematic Dirichlet*
(타격면 노드 타깃을 직접 이동)로 하중을 구동한다 — SAP soft contact 는
2 GPa 정제에 하중을 못 싣고 평판이 관통하기 때문 (FEM.md §terminology).

타격 프로파일: half-sine 변위 펄스
    d(t) = D_MAX · sin(π t / T_IMPACT),  t ∈ [0, T_IMPACT]
    t=0        : 충격 직전 (d=0, 무응력)
    t=1..n-1   : 타격 중 (압입 상승 → 최대 → 후퇴)
    t=n        : 충격 종료 (d=0 복귀, elastic 이므로 잔류응력 ~0)

출력
----
Sim_result/fem_stroke_impact_<ts>_sigI_cut.png    σ_I  3D 컷어웨이(반쪽 절단) — 메인
Sim_result/fem_stroke_impact_<ts>_sigIII_cut.png  σ_III 3D 컷어웨이 (압축 주응력장)
Sim_result/fem_stroke_impact_<ts>_sigI.png        σ_I  평면 단면(y=0)
Sim_result/fem_stroke_impact_<ts>_sigIII.png      σ_III 평면 단면
Sim_result/fem_stroke_impact_<ts>_vM.png          von Mises 평면 단면 (참고용)
Sim_result/fem_stroke_impact_<ts>_snaps.npz       스냅샷 (재렌더용)
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Malgun Gothic"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 옵션 ────────────────────────────────────────────────────────
DT, SUBSTEPS = 1e-3, 1
T_IMPACT     = 0.40                 # s — 타격 펄스 길이 (준정적: 관성 무시 가능)
N_STEPS      = int(T_IMPACT / DT)   # 400
D_MAX        = 40e-6                # m — 최대 압입 40 μm (ε_nom ≈ 1%)
N_PANELS     = 7                    # t=0, 타격 중 5, t=n
SAMPLE_EVERY = 5                    # σ 트레이스 샘플 간격
TARGET_FACES = 3000

E_TABLET, NU_TABLET, RHO_TABLET = 2.0e9, 0.25, 1300.0
MU  = E_TABLET / (2 * (1 + NU_TABLET))
LAM = E_TABLET * NU_TABLET / ((1 + NU_TABLET) * (1 - 2 * NU_TABLET))

_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
PNG_SI     = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_sigI.png")
PNG_SIII   = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_sigIII.png")
PNG_VM     = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_vM.png")
PNG_SI_C   = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_sigI_cut.png")
PNG_SIII_C = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_sigIII_cut.png")
NPZ        = os.path.join(OUT_DIR, f"fem_stroke_impact_{_TS}_snaps.npz")


def _npy(x):
    if hasattr(x, "to_numpy"): return x.to_numpy()
    if hasattr(x, "cpu"):      return x.cpu().numpy()
    return np.asarray(x)


# ── 후처리: 노드 위치 → tet Cauchy 응력 (corotated 소변형, fem_stress_field.py 동일) ──
def _stress_per_tet(X_ref, x_cur, el2v):
    Xr = X_ref[el2v]; Xc = x_cur[el2v]
    Dm = np.stack([Xr[:, 1] - Xr[:, 0], Xr[:, 2] - Xr[:, 0], Xr[:, 3] - Xr[:, 0]], axis=-1)
    Ds = np.stack([Xc[:, 1] - Xc[:, 0], Xc[:, 2] - Xc[:, 0], Xc[:, 3] - Xc[:, 0]], axis=-1)
    detDm = np.linalg.det(Dm)
    good = np.abs(detDm) > 1e-18
    F = np.zeros_like(Dm)
    F[good] = Ds[good] @ np.linalg.inv(Dm[good])
    F[~good] = np.eye(3)
    U, _S, Vt = np.linalg.svd(F)
    R = U @ Vt
    flip = np.linalg.det(R) < 0
    U[flip, :, -1] *= -1
    R = U @ Vt
    F_hat = np.transpose(R, (0, 2, 1)) @ F
    eps = 0.5 * (F_hat + np.transpose(F_hat, (0, 2, 1))) - np.eye(3)
    tr = np.trace(eps, axis1=1, axis2=2)
    return 2 * MU * eps + LAM * tr[:, None, None] * np.eye(3)


def _vm(sig):
    dev = sig - (np.trace(sig, axis1=1, axis2=2) / 3)[:, None, None] * np.eye(3)
    return np.sqrt(1.5 * (dev * dev).sum(axis=(1, 2)))


def _principal(sig):
    """주응력 (E,3) 오름차순: [:,0]=σ_III(최대 압축) … [:,-1]=σ_I(최대 인장)"""
    return np.linalg.eigvalsh(sig)


def _slice_polys(pos, el2v, vals, y_cut):
    Y = pos[:, 1]
    polys, cvals = [], []
    for e in range(len(el2v)):
        vs = el2v[e]; d = Y[vs] - y_cut
        if d.min() > 0 or d.max() < 0:
            continue
        pts = []
        for a in range(4):
            for b in range(a + 1, 4):
                if d[a] * d[b] < 0:
                    t = d[a] / (d[a] - d[b])
                    p = pos[vs[a]] + t * (pos[vs[b]] - pos[vs[a]])
                    pts.append((p[0], p[2]))
        if len(pts) < 3:
            continue
        pts = np.array(pts)
        c = pts.mean(0)
        pts = pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]
        polys.append(pts * 1e3)
        cvals.append(vals[e])
    return polys, np.array(cvals)


def _surface_faces(el2v):
    """tet 집합 → 경계 삼각형 (M,3) + 각 face 소속 tet idx (M,)"""
    faces = el2v[:, [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]].reshape(-1, 3)
    face_tet = np.repeat(np.arange(len(el2v)), 4)
    key = np.sort(faces, axis=1)
    _u, inv = np.unique(key, axis=0, return_inverse=True)
    cnt = np.bincount(inv)
    boundary = cnt[inv] == 1
    return faces[boundary], face_tet[boundary]


def _cutaway_polys(pos, el2v, vals, y_cut):
    """y>y_cut 반쪽을 제거한 부분메시의 경계면(절단면 포함) + face 별 tet 값"""
    cent = pos[el2v].mean(axis=1)
    keep = np.where(cent[:, 1] <= y_cut)[0]
    faces, ftet = _surface_faces(el2v[keep])
    return pos[faces] * 1e3, vals[keep][ftet]


def _d_of_step(step):
    """half-sine 타격 변위 [m]"""
    return D_MAX * np.sin(np.pi * min(step, N_STEPS) / N_STEPS)


def main():
    print("=" * 60)
    print(" FEM Stroke Impact — principal stress σ_I time series")
    print("=" * 60)

    raw = tm.load(TABLET_STL)
    raw.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    if len(raw.faces) > TARGET_FACES:
        try: raw = raw.simplify_quadric_decimation(face_count=TARGET_FACES)
        except Exception as e: print(f"[decimate][warn] {e}")
    STL_TMP = os.path.join(OUT_DIR, f"_tablet_stress_{TARGET_FACES}.stl")
    raw.export(STL_TMP)
    print(f"[stl] {len(raw.faces)} faces")

    bb = raw.bounding_box.bounds * TABLET_SCALE
    tab_lo, tab_hi = bb[0], bb[1]
    tablet_pos = (-0.5 * (tab_lo[0] + tab_hi[0]), -0.5 * (tab_lo[1] + tab_hi[1]),
                  -0.5 * (tab_lo[2] + tab_hi[2]))

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")

    # 순수 FEM (rigid 평판·SAP 커플러 불필요 — 하중은 Dirichlet 구동이 전담)
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

    # BC: 벽쪽(-z) 표면 고정, 스트로크쪽(+z) 표면을 half-sine 펄스로 압입
    z = pos0[:, 2]; band = (z.max() - z.min()) * 0.10
    strike_idx = np.where(z > z.max() - band)[0]      # 스트로크가 닿는 면
    wall_idx   = np.where(z < z.min() + band)[0]      # 벽에 붙은 면
    strike_pos0 = pos0[strike_idx].copy()
    tablet.set_vertex_constraints(wall_idx.tolist())
    tablet.set_vertex_constraints(strike_idx.tolist())
    H0 = float(pos0[strike_idx, 2].mean() - pos0[wall_idx, 2].mean())
    print(f"[bc] wall nodes={len(wall_idx)}  strike nodes={len(strike_idx)}  H0={H0*1e3:.3f} mm")

    # 패널 시점: t=0(직전) + 타격 중 균등 + t=n(종료)
    panel_steps = np.unique(np.round(np.linspace(0, N_STEPS, N_PANELS)).astype(int))

    # ── 스텝 ──
    snaps = {}          # step → pos
    trace = []          # (t, d, sI_max, sI_min, vm_max)
    print(f"\n[run] {N_STEPS} steps, half-sine pulse D_MAX={D_MAX*1e6:.0f} μm ...")
    if 0 in panel_steps:
        snaps[0] = pos0.copy()
    for step in range(1, N_STEPS + 1):
        d_cmd = _d_of_step(step)
        targets = strike_pos0.copy(); targets[:, 2] -= d_cmd
        tablet.update_constraint_targets(strike_idx.tolist(), targets)
        scene.step()
        need_snap  = step in panel_steps
        need_trace = (step % SAMPLE_EVERY == 0) or need_snap
        if need_trace:
            pos = _npy(tablet.get_state().pos).squeeze()
            sig = _stress_per_tet(pos0, pos, el2v)
            pr = _principal(sig); sI, sIII = pr[:, -1], pr[:, 0]
            trace.append((step * DT, d_cmd, sI.max(), sIII.min(), _vm(sig).max()))
            if need_snap:
                snaps[step] = pos.copy()
                print(f"  [panel] t={step*DT*1e3:5.0f} ms  d={d_cmd*1e6:6.2f} μm  "
                      f"ε_nom={d_cmd/H0*100:5.2f}%  σ_I max={sI.max()/1e6:+.2f}  "
                      f"σ_III min={sIII.min()/1e6:+.2f} MPa")
    trace = np.array(trace)

    # ── 패널 응력 계산 ──
    y_cut = float(np.median(pos0[:, 1]))
    panels = []
    for s in panel_steps:
        pos = snaps[s]
        sig = _stress_per_tet(pos0, pos, el2v)
        pr = _principal(sig)
        panels.append(dict(step=s, t=s * DT, d=_d_of_step(s),
                           pos=pos, sI=pr[:, -1], sIII=pr[:, 0], vm=_vm(sig)))

    # 컬러 스케일: 피크 패널의 75퍼센타일 — 내부 벌크 장이 선명하게 보이도록.
    # (구속밴드 모서리 집중응력(수백 MPa 특이점)은 포화시켜 배제)
    peak = max(panels, key=lambda p: p["d"])
    smax = max(np.percentile(np.abs(peak["sI"]), 75) / 1e6, 1e-3)
    norm_si = colors.Normalize(-smax, smax);   cmap_si = matplotlib.colormaps["seismic"]
    s3max = max(np.percentile(np.abs(peak["sIII"]), 75) / 1e6, 1e-3)
    norm_s3 = colors.Normalize(-s3max, s3max); cmap_s3 = cmap_si
    vmax = max(np.percentile(peak["vm"], 75) / 1e6, 1e-3)
    norm_vm = colors.Normalize(0.0, vmax);     cmap_vm = matplotlib.colormaps["jet"]

    lo, hi = pos0.min(0) * 1e3, pos0.max(0) * 1e3
    hdr = (f"E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  half-sine pulse T={T_IMPACT*1e3:.0f} ms  "
           f"D_max={D_MAX*1e6:.0f} μm (ε~{D_MAX/H0*100:.1f}%)  nodes={N}  tets={len(el2v)}")

    def _phase_label(i, n):
        if i == 0:     return "t=0 (직전)"
        if i == n - 1: return f"t=n (종료)"
        return f"t={i}"

    def _render(path, key, cmap_s, norm_s, cbar_label, title, stat_fn):
        n = len(panels)
        fig = plt.figure(figsize=(3.1 * n, 5.4))
        gs_ = fig.add_gridspec(2, n, height_ratios=[2.4, 1.0], hspace=0.42)
        for i, pn in enumerate(panels):
            ax = fig.add_subplot(gs_[0, i])
            polys, cvals = _slice_polys(pn["pos"], el2v, pn[key], y_cut)
            ax.add_collection(PolyCollection(polys, array=cvals / 1e6, cmap=cmap_s,
                                             norm=norm_s, edgecolors="face", linewidths=0))
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[2], hi[2]); ax.set_aspect("equal")
            ax.set_xticks([]); ax.set_yticks([])
            ax.set_title(f"{_phase_label(i, n)}   {pn['t']*1e3:.0f} ms\n"
                         f"d={pn['d']*1e6:.1f} μm\n{stat_fn(pn)}", fontsize=8.5)
        # 하단: 펄스 프로파일 + σ 트레이스
        axd = fig.add_subplot(gs_[1, :])
        axd.plot(trace[:, 0] * 1e3, trace[:, 1] * 1e6, "k-", lw=1.4, label="d(t) [μm]")
        axd.set_xlabel("t [ms]", fontsize=9); axd.set_ylabel("d [μm]", fontsize=9)
        axs2 = axd.twinx()
        axs2.plot(trace[:, 0] * 1e3, trace[:, 2] / 1e6, "r-", lw=1.0, label="$\\sigma_I^{max}$ (tension)")
        axs2.plot(trace[:, 0] * 1e3, trace[:, 3] / 1e6, "b-", lw=1.0, label="$\\sigma_{III}^{min}$ (compression)")
        axs2.set_ylabel("$\\sigma_I$ [MPa]", fontsize=9)
        for pn in panels:
            axd.axvline(pn["t"] * 1e3, color="0.6", ls=":", lw=0.8)
        h1, l1 = axd.get_legend_handles_labels(); h2, l2 = axs2.get_legend_handles_labels()
        axd.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right", ncol=3)
        axd.tick_params(labelsize=8); axs2.tick_params(labelsize=8)

        sm = cm.ScalarMappable(norm=norm_s, cmap=cmap_s); sm.set_array([])
        fig.colorbar(sm, ax=fig.axes[:n], fraction=0.02, pad=0.02).set_label(cbar_label)
        fig.suptitle(title + "\n" + hdr, fontsize=10)
        plt.savefig(path, dpi=200, bbox_inches="tight"); plt.close()

    def _render_cutaway(path, key, cmap_s, norm_s, cbar_label, title, stat_fn):
        """3D 컷어웨이: y>0 반쪽 제거 → 절단면 응력장 + 바깥 표면. 벽/스트로크 주석 포함."""
        n = len(panels)
        ext = hi - lo
        fig = plt.figure(figsize=(3.1 * n, 5.6))
        gs_ = fig.add_gridspec(2, n, height_ratios=[2.4, 1.0], hspace=0.30, top=0.80)
        for i, pn in enumerate(panels):
            ax = fig.add_subplot(gs_[0, i], projection="3d")
            verts, fvals = _cutaway_polys(pn["pos"], el2v, pn[key], y_cut)
            ax.add_collection3d(Poly3DCollection(
                verts, facecolors=cmap_s(norm_s(fvals / 1e6)),
                edgecolors="k", linewidths=0.02))
            # 벽(고정) — 정제 아래 회색 판
            wall_quad = [[(lo[0], lo[1], lo[2]), (hi[0], lo[1], lo[2]),
                          (hi[0], hi[1], lo[2]), (lo[0], hi[1], lo[2])]]
            ax.add_collection3d(Poly3DCollection(wall_quad, facecolors="0.7", alpha=0.5))
            # 스트로크 — 위에서 내려꽂는 화살표
            ax.quiver(0, y_cut * 1e3, hi[2] + 1.1, 0, 0, -0.9,
                      color="k", arrow_length_ratio=0.35, lw=1.8)
            m = 0.06
            ax.set_xlim(lo[0] - m * ext[0], hi[0] + m * ext[0])
            ax.set_ylim(lo[1] - m * ext[1], hi[1] + m * ext[1])
            ax.set_zlim(lo[2] - m * ext[2] - 0.4, hi[2] + m * ext[2] + 1.2)
            ax.set_box_aspect((ext[0], ext[1], ext[2] + 1.6))
            ax.view_init(elev=18, azim=105)
            ax.set_axis_off()
            ax.set_title(f"{_phase_label(i, n)}   {pn['t']*1e3:.0f} ms\n"
                         f"d={pn['d']*1e6:.1f} μm\n{stat_fn(pn)}", fontsize=8.5)
            if i == 0:
                ax.text2D(0.5, 0.86, "stroke ↓", transform=ax.transAxes,
                          ha="center", fontsize=8, color="k")
                ax.text2D(0.5, 0.04, "wall (고정)", transform=ax.transAxes,
                          ha="center", fontsize=8, color="0.35")
        axd = fig.add_subplot(gs_[1, :])
        axd.plot(trace[:, 0] * 1e3, trace[:, 1] * 1e6, "k-", lw=1.4, label="d(t) [μm]")
        axd.set_xlabel("t [ms]", fontsize=9); axd.set_ylabel("d [μm]", fontsize=9)
        axs2 = axd.twinx()
        axs2.plot(trace[:, 0] * 1e3, trace[:, 2] / 1e6, "r-", lw=1.0, label="$\\sigma_I^{max}$ (tension)")
        axs2.plot(trace[:, 0] * 1e3, trace[:, 3] / 1e6, "b-", lw=1.0, label="$\\sigma_{III}^{min}$ (compression)")
        axs2.set_ylabel("$\\sigma$ [MPa]", fontsize=9)
        for pn in panels:
            axd.axvline(pn["t"] * 1e3, color="0.6", ls=":", lw=0.8)
        h1, l1 = axd.get_legend_handles_labels(); h2, l2 = axs2.get_legend_handles_labels()
        axd.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper right", ncol=3)
        axd.tick_params(labelsize=8); axs2.tick_params(labelsize=8)
        sm = cm.ScalarMappable(norm=norm_s, cmap=cmap_s); sm.set_array([])
        fig.colorbar(sm, ax=fig.axes[:n], fraction=0.02, pad=0.02).set_label(cbar_label)
        fig.suptitle(title + "\n" + hdr, fontsize=10)
        plt.savefig(path, dpi=200, bbox_inches="tight"); plt.close()

    _render_cutaway(PNG_SI_C, "sI", cmap_si, norm_si,
                    "max principal $\\sigma_I$ [MPa]  (+ tension / - compression)",
                    "FEM Stroke Impact — $\\sigma_I$ cutaway (앞 반쪽 제거, 절단면 = 내부 응력장)",
                    lambda pn: f"$\\sigma_I^{{max}}$={pn['sI'].max()/1e6:+.1f} MPa")
    _render_cutaway(PNG_SIII_C, "sIII", cmap_s3, norm_s3,
                    "min principal $\\sigma_{III}$ [MPa]  (+ tension / - compression)",
                    "FEM Stroke Impact — $\\sigma_{III}$ cutaway (앞 반쪽 제거, 절단면 = 내부 응력장)",
                    lambda pn: f"$\\sigma_{{III}}^{{min}}$={pn['sIII'].min()/1e6:+.1f} MPa")

    _render(PNG_SI, "sI", cmap_si, norm_si,
            "max principal $\\sigma_I$ [MPa]  (+ tension / - compression)",
            "FEM Stroke Impact — max principal stress $\\sigma_I$ (cross-section y=0, wall-side fixed)",
            lambda pn: f"$\\sigma_I^{{max}}$={pn['sI'].max()/1e6:+.1f} MPa")
    _render(PNG_SIII, "sIII", cmap_s3, norm_s3,
            "min principal $\\sigma_{III}$ [MPa]  (+ tension / - compression)",
            "FEM Stroke Impact — min principal stress $\\sigma_{III}$ (cross-section y=0, wall-side fixed)",
            lambda pn: f"$\\sigma_{{III}}^{{min}}$={pn['sIII'].min()/1e6:+.1f} MPa")
    _render(PNG_VM, "vm", cmap_vm, norm_vm, "von Mises $\\sigma_{vm}$ [MPa]",
            "FEM Stroke Impact — von Mises (cross-section y=0, reference)",
            lambda pn: f"$\\sigma_{{vm}}^{{max}}$={pn['vm'].max()/1e6:.1f} MPa")

    np.savez_compressed(NPZ, pos0=pos0, el2v=el2v, trace=trace,
                        panel_steps=np.array([p["step"] for p in panels]),
                        **{f"pos_{p['step']}": p["pos"] for p in panels})

    print(f"\n[saved] σ_I  cutaway : {PNG_SI_C}")
    print(f"[saved] σ_III cutaway: {PNG_SIII_C}")
    print(f"[saved] σ_I  : {PNG_SI}")
    print(f"[saved] σ_III: {PNG_SIII}")
    print(f"[saved] vM   : {PNG_VM}")
    print(f"[saved] snaps: {NPZ}")
    print(f"[scale] σI |max|(95%)={smax:.2f}  σIII |max|(95%)={s3max:.2f}  vM(95%)={vmax:.2f} MPa")


if __name__ == "__main__":
    main()
