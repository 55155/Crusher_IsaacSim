"""
fem_stress_field.py — SAP Rigid–FEM 압축 중 von Mises 응력장 3D 컬러맵

목적
----
fem_uniaxial_compression.py 와 *동일한 SAP 환경*(box 가 정제를 누름)에서, 압축이
진행되는 동안 정제 내부의 von Mises 응력장이 어떻게 발달하는지를 3D 컬러 contour
5장으로 시각화한다. (Genesis FEM 은 tet 응력을 외부로 노출하지 않으므로, 노드
위치를 스냅샷으로 기록한 뒤 변형구배 F 로부터 응력을 *후처리* 계산 — 솔버가 쓰는
linear_corotated 모델과 동일한 corotated 소변형 → Cauchy 응력.)

터널링 방지
-----------
PLATE_VEL · DT = step 당 평판 변위. 여기선 3e-4 · 1e-3 = 0.3 μm/step 으로 표면 tet
크기(수십~수백 μm)보다 훨씬 작아 관통이 구조적으로 불가능. tet 도 충분히 촘촘하게.

출력
----
Sim_result/fem_stress_field_<ts>.png   (1×5 패널 + 공유 컬러바)
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 옵션 (quasi-static, 터널링-free) ────────────────────────────
DT, SUBSTEPS = 1e-3, 1
PLATE_VEL    = 1.0e-4            # 0.1 mm/s → 0.1 μm/step (≪ tet 크기). SAP 접촉 안정 위해 quasi-static.
DURATION     = 0.40             # s  → 400 step
N_STEPS      = int(DURATION / DT)
SAMPLE_EVERY = 20               # 노드 위치 스냅샷 간격 (→ 20 snapshot)
TARGET_FACES = 3000             # 표면 decimation (촘촘한 tet)
N_PANELS     = 5
F_MAX_Z      = 1.0e6            # 사실상 uncapped → velocity-driven 압축 (관통 대신 실제 변형)

E_TABLET, NU_TABLET, RHO_TABLET = 2.0e9, 0.25, 1300.0
MU  = E_TABLET / (2 * (1 + NU_TABLET))                       # Lamé μ
LAM = E_TABLET * NU_TABLET / ((1 + NU_TABLET) * (1 - 2 * NU_TABLET))  # Lamé λ

_REPO_ROOT   = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL   = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3

PLATE_SIZE   = (0.025, 0.025, 0.003)
PLATE_MARGIN = 1.0e-6

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
PNG = os.path.join(OUT_DIR, f"fem_stress_field_{_TS}.png")


def _npy(x):
    if hasattr(x, "to_numpy"): return x.to_numpy()
    if hasattr(x, "cpu"):      return x.cpu().numpy()
    return np.asarray(x)


# ── 후처리: 노드 위치 → tet Cauchy 응력 텐서 (corotated 소변형) ─────
def _stress_per_tet(X_ref, x_cur, el2v):
    """X_ref,(N,3) 기준 / x_cur,(N,3) 현재 / el2v,(E,4) → σ,(E,3,3) Cauchy [Pa]"""
    Xr = X_ref[el2v]   # (E,4,3)
    Xc = x_cur[el2v]
    Dm = np.stack([Xr[:, 1] - Xr[:, 0], Xr[:, 2] - Xr[:, 0], Xr[:, 3] - Xr[:, 0]], axis=-1)  # (E,3,3)
    Ds = np.stack([Xc[:, 1] - Xc[:, 0], Xc[:, 2] - Xc[:, 0], Xc[:, 3] - Xc[:, 0]], axis=-1)
    detDm = np.linalg.det(Dm)
    good = np.abs(detDm) > 1e-18
    F = np.zeros_like(Dm)
    F[good] = Ds[good] @ np.linalg.inv(Dm[good])
    F[~good] = np.eye(3)
    U, _S, Vt = np.linalg.svd(F)                      # polar: R = U Vt (reflection 보정)
    R = U @ Vt
    flip = np.linalg.det(R) < 0
    U[flip, :, -1] *= -1
    R = U @ Vt
    F_hat = np.transpose(R, (0, 2, 1)) @ F
    eps = 0.5 * (F_hat + np.transpose(F_hat, (0, 2, 1))) - np.eye(3)   # corotated 소변형
    tr = np.trace(eps, axis1=1, axis2=2)
    return 2 * MU * eps + LAM * tr[:, None, None] * np.eye(3)          # σ (E,3,3)


def _vm(sig):
    """von Mises [Pa] from σ,(E,3,3)"""
    dev = sig - (np.trace(sig, axis1=1, axis2=2) / 3)[:, None, None] * np.eye(3)
    return np.sqrt(1.5 * (dev * dev).sum(axis=(1, 2)))


def _sigma_I(sig):
    """최대 주응력 σ_I [Pa] (가장 인장쪽 고유값) from σ,(E,3,3)"""
    return np.linalg.eigvalsh(sig)[:, -1]   # eigvalsh: 오름차순 → 마지막이 최대


def _slice_polys(pos, el2v, vals, y_cut):
    """평면 y=y_cut 와 교차하는 tet 들의 단면 다각형(x–z, mm) + tet 값"""
    Y = pos[:, 1]
    polys, cvals = [], []
    for e in range(len(el2v)):
        vs = el2v[e]; d = Y[vs] - y_cut
        if d.min() > 0 or d.max() < 0:   # 평면을 안 가로지름
            continue
        pts = []
        for a in range(4):
            for b in range(a + 1, 4):
                if d[a] * d[b] < 0:                       # 모서리가 평면을 가로지름
                    t = d[a] / (d[a] - d[b])
                    p = pos[vs[a]] + t * (pos[vs[b]] - pos[vs[a]])
                    pts.append((p[0], p[2]))              # x, z
        if len(pts) < 3:
            continue
        pts = np.array(pts)
        c = pts.mean(0)                                   # 각도순 정렬 → convex 다각형
        pts = pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]
        polys.append(pts * 1e3)
        cvals.append(vals[e])
    return polys, np.array(cvals)


def _surface_faces(el2v):
    """tet 집합 → 경계 삼각형 (surf_faces,(M,3)), 각 face 의 소속 tet idx (M,)"""
    faces = el2v[:, [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]]].reshape(-1, 3)  # (4E,3)
    face_tet = np.repeat(np.arange(len(el2v)), 4)
    key = np.sort(faces, axis=1)
    _u, inv = np.unique(key, axis=0, return_inverse=True)
    cnt = np.bincount(inv)
    boundary = cnt[inv] == 1
    return faces[boundary], face_tet[boundary]


def main():
    print("=" * 60)
    print(" FEM Stress Field — SAP Rigid–FEM compression (von Mises)")
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
    tab_h = float(tab_hi[2] - tab_lo[2])
    tablet_pos = (-0.5 * (tab_lo[0] + tab_hi[0]), -0.5 * (tab_lo[1] + tab_hi[1]),
                  -0.5 * (tab_lo[2] + tab_hi[2]))
    plate_gap = tab_h + 2 * PLATE_MARGIN
    z_center = plate_gap / 2

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, 0)),
        fem_options=gs.options.FEMOptions(use_implicit_solver=True,
                                          enable_vertex_constraints=True, pcg_threshold=1e-10),
        coupler_options=gs.options.SAPCouplerOptions(
            pcg_threshold=1e-10, sap_convergence_atol=1e-10, sap_convergence_rtol=1e-10,
            linesearch_ftol=1e-10, rigid_rigid_contact_type="none"),
        show_viewer=False,
    )
    plate_bot = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, -PLATE_MARGIN - PLATE_SIZE[2] / 2), fixed=True),
        material=gs.materials.Rigid())
    plate_top_z0 = plate_gap + PLATE_MARGIN + PLATE_SIZE[2] / 2
    plate_top = scene.add_entity(
        morph=gs.morphs.Box(size=PLATE_SIZE, pos=(0, 0, plate_top_z0), fixed=False),
        material=gs.materials.Rigid(rho=1.0e7, coup_friction=1.0, friction=1.0))
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET,
                                          model="linear_corotated", friction_mu=1.0),
        morph=gs.morphs.Mesh(file=STL_TMP, scale=TABLET_SCALE,
                             pos=(tablet_pos[0], tablet_pos[1], tablet_pos[2] + z_center)))
    scene.build(n_envs=0)

    # 연결성 + 참조 위치
    el2v = _npy(tablet.get_el2v()).astype(np.int64)
    pos0 = _npy(tablet.get_state().pos).squeeze()        # (N,3) 참조 config
    N = len(pos0)
    if el2v.min() != 0 or el2v.max() >= N:               # global→local 보정
        el2v = el2v - el2v.min()
    print(f"[mesh] nodes={N}  tets={len(el2v)}")
    surf_faces, surf_tet = _surface_faces(el2v)
    print(f"[mesh] surface triangles={len(surf_faces)}")

    # BC: bot 고정 + top 을 매 스텝 아래로 이동시키는 kinematic Dirichlet.
    # (link= 구속은 bind 시점 world 위치에 *고정*되어 압축을 못 만듦. SAP soft-contact 는
    #  이 stiff(2GPa) 정제에 하중을 못 실어 평판이 관통(F≈0). 따라서 update_constraint_targets
    #  로 top 표면 타깃을 직접 하강시켜 box 가 정제를 누르는 단축 압축을 강제한다.)
    z = pos0[:, 2]; band = (z.max() - z.min()) * 0.10
    top_idx = np.where(z > z.max() - band)[0]
    bot_idx = np.where(z < z.min() + band)[0]
    top_pos0 = pos0[top_idx].copy()                      # top 노드 초기 위치
    tablet.set_vertex_constraints(bot_idx.tolist())      # 현재 위치에 고정
    tablet.set_vertex_constraints(top_idx.tolist())      # 현재 위치에 핀 → 이후 타깃 이동

    # plate velocity control + force cap
    n_dof = plate_top.n_dofs
    plate_top.set_dofs_kp(np.zeros(n_dof))
    plate_top.set_dofs_kv(np.ones(n_dof) * 1.0e7)
    fmin = np.full(n_dof, -np.inf); fmax = np.full(n_dof, np.inf)
    if n_dof >= 3: fmin[2], fmax[2] = -F_MAX_Z, F_MAX_Z
    plate_top.set_dofs_force_range(lower=fmin, upper=fmax)
    v_target = np.zeros(n_dof)
    if n_dof >= 3: v_target[2] = -PLATE_VEL
    plate_top.control_dofs_velocity(velocity=v_target, dofs_idx_local=list(range(n_dof)))

    coupler = scene.sim.coupler
    top_np = np.asarray(top_idx, dtype=np.int64)
    H0 = float(pos0[top_idx, 2].mean() - pos0[bot_idx, 2].mean())   # 초기 정제 두께 [m]

    def _F_top():
        imp = _npy(coupler.fem_state_v.impulse)
        if imp.ndim == 3: imp = imp[0]
        return -float(imp[top_np, 2].sum()) / DT

    # ── 스텝 + 스냅샷 ──
    snaps = []   # (t, d[m], F[N], pos(N,3))
    print("\n[run] stepping...   (d=top 하강량, ε_nom=정제 두께 변형률, σvm=즉석 후처리)")
    for step in range(N_STEPS):
        # top 타깃을 -z 로 이동 (압축 driver). x,y 유지 → 단축 압축.
        d_cmd = PLATE_VEL * (step * DT)
        targets = top_pos0.copy(); targets[:, 2] -= d_cmd
        tablet.update_constraint_targets(top_idx.tolist(), targets)
        scene.step()
        if step % SAMPLE_EVERY == 0:
            pos = _npy(tablet.get_state().pos).squeeze()
            d = d_cmd
            H = float(pos[top_idx, 2].mean() - pos[bot_idx, 2].mean())
            eps_nom = (H0 - H) / H0
            snaps.append((step * DT, d, eps_nom, pos.copy()))
            vm = _vm(_stress_per_tet(pos0, pos, el2v))
            print(f"  t={step*DT*1e3:6.0f}ms  d={d*1e6:7.2f}μm  ε_nom={eps_nom*100:+.3f}%  "
                  f"σvm[max/95%]={vm.max()/1e6:.2f}/{np.percentile(vm,95)/1e6:.2f}MPa")
    print(f"[run] {len(snaps)} snapshots, d_max={snaps[-1][1]*1e6:.1f} μm")

    # ── 5 패널 선택 (변위 균등) ──
    d_all = np.array([s[1] for s in snaps])
    targets = np.linspace(d_all[d_all > 0].min() if (d_all > 0).any() else d_all[1],
                          d_all.max(), N_PANELS)
    sel = [int(np.argmin(np.abs(d_all - t))) for t in targets]
    sel = sorted(set(sel))
    while len(sel) < N_PANELS:  # 중복 제거로 부족하면 채움
        for i in range(len(snaps)):
            if i not in sel: sel.append(i)
            if len(sel) == N_PANELS: break
        sel = sorted(set(sel))
    sel = sel[:N_PANELS]

    # ── 패널별 응력(텐서→vM, σ_I, nodal vM) ──
    from matplotlib.collections import PolyCollection
    y_cut = float(np.median(pos0[:, 1]))           # 단면 평면 y=center
    panels = []
    for i in sel:
        pos = snaps[i][3]
        sig = _stress_per_tet(pos0, pos, el2v)     # (E,3,3)
        vm_t, sI_t = _vm(sig), _sigma_I(sig)
        vn = np.zeros(N); cn = np.zeros(N)
        np.add.at(vn, el2v.ravel(), np.repeat(vm_t, 4)); np.add.at(cn, el2v.ravel(), 1.0)
        panels.append(dict(t=snaps[i][0], d=snaps[i][1], eps=snaps[i][2],
                           pos=pos, vm_t=vm_t, sI_t=sI_t, vm_node=vn / np.maximum(cn, 1)))

    # 컬러 스케일
    vmax = max(np.percentile(np.concatenate([p["vm_node"] for p in panels]), 99) / 1e6, 1e-3)
    norm_vm = colors.Normalize(0.0, vmax);  cmap_vm = matplotlib.colormaps["jet"]
    smax = max(np.percentile(np.abs(np.concatenate([p["sI_t"] for p in panels])), 99) / 1e6, 1e-3)
    norm_si = colors.Normalize(-smax, smax); cmap_si = matplotlib.colormaps["RdBu_r"]  # 적=인장 청=압축

    lo, hi = pos0.min(0) * 1e3, pos0.max(0) * 1e3
    ctr = (lo + hi) / 2; rng = (hi - lo).max() * 0.6

    base = PNG[:-4]
    PNG_SURF, PNG_VM, PNG_SI = PNG, base + "_sliceVM.png", base + "_sliceSigI.png"
    hdr = (f"E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  v={PLATE_VEL*1e6:.0f} μm/s  "
           f"nodes={N}  tets={len(el2v)}  (kinematic top-surface Dirichlet)")

    # ── Fig 1: 표면 von Mises (윗면 + 밑면) ──
    views = [(30, -70, "top view"), (-30, -70, "bottom view")]
    fig = plt.figure(figsize=(4 * N_PANELS, 8.6))
    for p, pn in enumerate(panels):
        vn_mpa = pn["vm_node"] / 1e6
        verts = pn["pos"][surf_faces] * 1e3
        face_c = cmap_vm(norm_vm(vn_mpa[surf_faces].mean(axis=1)))
        for r, (elev, azim, tag) in enumerate(views):
            ax = fig.add_subplot(2, N_PANELS, r * N_PANELS + p + 1, projection="3d")
            ax.add_collection3d(Poly3DCollection(verts, facecolors=face_c, edgecolors="k", linewidths=0.05))
            ax.set_xlim(ctr[0] - rng, ctr[0] + rng); ax.set_ylim(ctr[1] - rng, ctr[1] + rng)
            ax.set_zlim(ctr[2] - rng, ctr[2] + rng); ax.set_box_aspect((1, 1, 1))
            ax.view_init(elev=elev, azim=azim); ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
            if r == 0:
                ax.set_title(f"t={pn['t']*1e3:.0f} ms   ε={pn['eps']*100:.2f}%\n"
                             f"$\\sigma_{{vm}}^{{max}}$={vn_mpa.max():.1f} MPa", fontsize=9)
            ax.text2D(0.5, -0.02, tag, transform=ax.transAxes, ha="center", fontsize=8, color="0.4")
    sm = cm.ScalarMappable(norm=norm_vm, cmap=cmap_vm); sm.set_array([])
    fig.colorbar(sm, ax=fig.axes, fraction=0.012, pad=0.02).set_label("von Mises $\\sigma_{vm}$ [MPa]")
    fig.suptitle("FEM Uniaxial Compression — von Mises (surface, top & bottom)\n" + hdr, fontsize=10)
    plt.savefig(PNG_SURF, dpi=130, bbox_inches="tight"); plt.close()

    # ── Fig 2 & 3: 단면(y=center) — von Mises / 최대 주응력 σ_I ──
    def _render_slice(path, key, cmap_s, norm_s, label, title, signed=False):
        fig, axs = plt.subplots(1, N_PANELS, figsize=(3.6 * N_PANELS, 4.4))
        for p, pn in enumerate(panels):
            polys, cvals = _slice_polys(pn["pos"], el2v, pn[key], y_cut)
            ax = axs[p]
            ax.add_collection(PolyCollection(polys, array=cvals / 1e6, cmap=cmap_s,
                                             norm=norm_s, edgecolors="face", linewidths=0))
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[2], hi[2]); ax.set_aspect("equal")
            ax.set_xticks([]); ax.set_yticks([])
            if signed:
                ttl = (f"t={pn['t']*1e3:.0f} ms  ε={pn['eps']*100:.2f}%\n"
                       f"$\\sigma_I$[max/min]={pn['sI_t'].max()/1e6:.1f}/{pn['sI_t'].min()/1e6:.1f} MPa")
            else:
                ttl = (f"t={pn['t']*1e3:.0f} ms  ε={pn['eps']*100:.2f}%\n"
                       f"$\\sigma_{{vm}}^{{max}}$={pn['vm_t'].max()/1e6:.1f} MPa")
            ax.set_title(ttl, fontsize=9)
        sm = cm.ScalarMappable(norm=norm_s, cmap=cmap_s); sm.set_array([])
        fig.colorbar(sm, ax=axs, fraction=0.02, pad=0.02).set_label(label)
        fig.suptitle(title + "\n" + hdr, fontsize=10)
        plt.savefig(path, dpi=130, bbox_inches="tight"); plt.close()

    _render_slice(PNG_VM, "vm_t", cmap_vm, norm_vm, "von Mises $\\sigma_{vm}$ [MPa]",
                  "FEM Uniaxial Compression — von Mises (cross-section y=0, interior)")
    _render_slice(PNG_SI, "sI_t", cmap_si, norm_si, "max principal $\\sigma_I$ [MPa]  (+ tension / − compression)",
                  "FEM Uniaxial Compression — max principal stress $\\sigma_I$ (cross-section y=0)", signed=True)

    print(f"\n[saved] surface vM : {PNG_SURF}")
    print(f"[saved] slice   vM : {PNG_VM}")
    print(f"[saved] slice   σI : {PNG_SI}")
    print(f"[scale] vM vmax(99%)={vmax:.2f} MPa   σI |max|(99%)={smax:.2f} MPa")


if __name__ == "__main__":
    main()
