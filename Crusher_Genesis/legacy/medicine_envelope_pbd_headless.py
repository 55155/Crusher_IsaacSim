"""
medicine_envelope_pbd_headless.py
약봉투(Medicine Envelope) Genesis PBD 헤드리스 시뮬레이션
particle 공간 분포 그래프 출력

medicine_envelope.py 파라미터 기준:
  BAG_WIDTH  = 8  cm (X)
  BAG_HEIGHT = 12 cm (Y → euler(90,0,0) 후 World-Z 방향)
  BAG_DEPTH  = 1  cm (Z → World-Y 방향)

5개 패널:
  Front_Panel  : XY plane, Z=0
  Back_Panel   : XY plane, Z=D
  Bottom_Seal  : XZ plane, Y=0
  Left_Seal    : YZ plane, X=0
  Right_Seal   : YZ plane, X=W

사용법:
  python medicine_envelope_pbd_headless.py
"""

import os, time, struct
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ─────────────────────────────────────────────────────────────
# 약봉투 파라미터 (medicine_envelope.py 와 동일)
W = 0.08   # 폭    [m]
H = 0.12   # 높이  [m]
D = 0.01   # 깊이  [m]

# 메시 분할 해상도 (셀 수)
NW = 8     # 폭 방향 (X)
NH = 12    # 높이 방향 (Y)
ND = 3     # 깊이 방향 (Z, seal 패널)

# 시뮬레이션
DT        = 5e-4
SUBSTEPS  = 10
SIM_TIME  = 2.0
LOG_EVERY = 40        # step 간격으로 통계 기록

# 스냅샷 시각 [s]
SNAP_TIMES = [0.0, 0.5, 1.0, 1.5, 2.0]

# 출력 경로
_DIR     = os.path.dirname(os.path.abspath(__file__))
OUT_DIR  = os.path.join(_DIR, "Sim_result")
os.makedirs(OUT_DIR, exist_ok=True)

STL_PATH  = os.path.join(OUT_DIR, "medicine_envelope_pbd.stl")
PLOT_PATH = os.path.join(OUT_DIR, "medicine_envelope_pbd_distribution.png")
CSV_PATH  = os.path.join(OUT_DIR, "medicine_envelope_pbd.csv")
# ─────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════
#  메시 생성 유틸리티
# ══════════════════════════════════════════════════════════════

def _panel_triangles(fn, nu, nv):
    """
    fn(u, v) → np.array([x, y, z])  (u,v ∈ [0,1])
    nu×nv 쿼드 그리드 → 삼각형 리스트 반환.
    인접 패널 공유 엣지는 동일 float 값으로 계산되므로
    Genesis가 vertex merge 후 weld constraint 생성.
    """
    tris = []
    for i in range(nu):
        for j in range(nv):
            u0, u1 = i / nu, (i + 1) / nu
            v0, v1 = j / nv, (j + 1) / nv
            p00 = fn(u0, v0).astype(np.float32)
            p10 = fn(u1, v0).astype(np.float32)
            p11 = fn(u1, v1).astype(np.float32)
            p01 = fn(u0, v1).astype(np.float32)
            tris.append(np.array([p00, p10, p11], np.float32))
            tris.append(np.array([p00, p11, p01], np.float32))
    return tris


def create_envelope_stl():
    """
    medicine_envelope.py 와 동일한 5-panel 상단-개방(open-top) 봉투 → STL.

    원본 모델(Fusion surface)에는 Top_Seal 이 없다(상단 개방).
    Genesis 1.1.0 에서는 open(비-watertight) 메시도 PBD.Cloth 로 정상 빌드되므로
    watertight 용 Top_Seal 을 추가하지 않는다.

    trimesh 로 vertex merge(인접 패널 weld) + origin 중심 정렬만 수행.
    """
    import trimesh as _tm

    tris = []
    # Front_Panel  (XY, Z=0)
    tris += _panel_triangles(lambda u, v: np.array([u*W,  v*H,  0.0]), NW, NH)
    # Back_Panel   (XY, Z=D)
    tris += _panel_triangles(lambda u, v: np.array([u*W,  v*H,  D  ]), NW, NH)
    # Bottom_Seal  (XZ, Y=0)
    tris += _panel_triangles(lambda u, v: np.array([u*W,  0.0,  v*D]), NW, ND)
    # Left_Seal    (YZ, X=0)
    tris += _panel_triangles(lambda u, v: np.array([0.0,  u*H,  v*D]), NH, ND)
    # Right_Seal   (YZ, X=W)
    tris += _panel_triangles(lambda u, v: np.array([W,    u*H,  v*D]), NH, ND)
    # → 상단(Y=H) 개방. Top_Seal 없음.

    # trimesh 로 vertex merge + origin 중심 정렬
    verts = np.array([pt for tri in tris for pt in tri])
    faces = np.arange(len(verts)).reshape(-1, 3)
    m = _tm.Trimesh(vertices=verts, faces=faces, process=False)
    m.merge_vertices(digits_vertex=7)

    # bounding box 중심 → origin
    center = m.bounding_box.centroid.copy()
    m.vertices -= center

    print(f"[mesh] panels=5 (open-top)  triangles={len(m.faces)}  "
          f"vertices={len(m.vertices)}  watertight={m.is_watertight}")
    print(f"[mesh] bounds after centering: {m.bounds}")

    m.export(STL_PATH)
    print(f"[mesh] STL saved: {STL_PATH}")

    # 원래 좌표계에서의 중심 오프셋 반환 (pos 보정에 사용)
    return center


# ══════════════════════════════════════════════════════════════
#  파티클 유틸리티
# ══════════════════════════════════════════════════════════════

def _get_pos(entity):
    """Genesis 파티클 위치 → (N, 3) numpy float64."""
    pos = entity.get_particles_pos()
    if hasattr(pos, "cpu"):
        pos = pos.cpu().numpy()
    pos = np.asarray(pos, dtype=np.float64)
    if pos.ndim == 3:        # (n_envs, N, 3)
        pos = pos[0]
    return pos


def find_top_corners(pos0):
    """
    Z 상위 5% 구역에서 X·Y 4방향 조합으로 코너 4점 선택 (상단 개방부).
    euler=(90,0,0) 후 봉투 상단 → world 최대 Z.
    """
    z = pos0[:, 2]
    x = pos0[:, 0]
    y = pos0[:, 1]
    top = np.where(z >= np.quantile(z, 0.95))[0]
    c = [
        top[np.argmin( x[top] + y[top])],   # -X -Y
        top[np.argmax( x[top] - y[top])],   # +X -Y
        top[np.argmin(-x[top] + y[top])],   # -X +Y
        top[np.argmax( x[top] + y[top])],   # +X +Y
    ]
    return list(set(int(i) for i in c))


# ══════════════════════════════════════════════════════════════
#  플롯
# ══════════════════════════════════════════════════════════════

def make_plot(N, snapshots, log_t, log_com, log_std, log_nan):
    cmap = plt.cm.plasma(np.linspace(0.1, 0.95, len(SNAP_TIMES)))

    fig = plt.figure(figsize=(18, 13))
    fig.suptitle(
        f"Medicine Envelope PBD — Particle Distribution\n"
        f"W={W*100:.0f} cm  H={H*100:.0f} cm  D={D*100:.1f} cm  "
        f"N_particles={N}  T_sim={SIM_TIME} s",
        fontsize=13, fontweight="bold",
    )

    # ── Row 1: 3D scatter  +  XZ / YZ 투영 ──────────────────
    ax3d = fig.add_subplot(3, 4, (1, 2), projection="3d")
    for i, st in enumerate(SNAP_TIMES):
        if st not in snapshots:
            continue
        p = snapshots[st]
        v = p[~np.isnan(p).any(axis=1)]
        ax3d.scatter(v[:, 0] * 100, v[:, 1] * 100, v[:, 2] * 100,
                     c=[cmap[i]], s=3, alpha=0.55, label=f"t={st:.1f}s")
    ax3d.set_xlabel("X [cm]"); ax3d.set_ylabel("Y [cm]"); ax3d.set_zlabel("Z [cm]")
    ax3d.set_title("3D Particle Distribution"); ax3d.legend(fontsize=7, markerscale=3)

    for idx, (xi, yi, lx, ly, title) in enumerate([
        (0, 2, "X [cm]", "Z [cm]", "XZ — front view"),
        (1, 2, "Y [cm]", "Z [cm]", "YZ — side view"),
    ]):
        ax = fig.add_subplot(3, 4, 3 + idx)
        for i, st in enumerate(SNAP_TIMES):
            if st not in snapshots:
                continue
            p = snapshots[st]
            v = p[~np.isnan(p).any(axis=1)]
            ax.scatter(v[:, xi] * 100, v[:, yi] * 100,
                       c=[cmap[i]], s=2, alpha=0.45, label=f"t={st:.1f}s")
        ax.set_xlabel(lx); ax.set_ylabel(ly); ax.set_title(title)
        ax.legend(fontsize=6, markerscale=3); ax.grid(alpha=0.25)

    # ── Row 2: X / Y / Z 히스토그램  +  Z ECDF ─────────────
    for ci, lbl in enumerate(["X", "Y", "Z"]):
        ax = fig.add_subplot(3, 4, 5 + ci)
        for i, st in enumerate(SNAP_TIMES):
            if st not in snapshots:
                continue
            p = snapshots[st]
            v = p[~np.isnan(p).any(axis=1)]
            ax.hist(v[:, ci] * 100, bins=30, alpha=0.45,
                    density=True, color=cmap[i], label=f"t={st:.1f}s")
        ax.set_xlabel(f"{lbl} [cm]"); ax.set_ylabel("density")
        ax.set_title(f"{lbl} distribution"); ax.legend(fontsize=6); ax.grid(alpha=0.25)

    ax_ecdf = fig.add_subplot(3, 4, 8)
    for i, st in enumerate(SNAP_TIMES):
        if st not in snapshots:
            continue
        p = snapshots[st]
        v = p[~np.isnan(p).any(axis=1)]
        zv = np.sort(v[:, 2] * 100)
        ecdf = np.arange(1, len(zv) + 1) / len(zv)
        ax_ecdf.plot(zv, ecdf, color=cmap[i], lw=1.5, label=f"t={st:.1f}s")
    ax_ecdf.set_xlabel("Z [cm]"); ax_ecdf.set_ylabel("ECDF")
    ax_ecdf.set_title("Z ECDF (height distribution)")
    ax_ecdf.legend(fontsize=6); ax_ecdf.grid(alpha=0.25)

    # ── Row 3: Center of Mass  +  Spread (std) 시계열 ────────
    ax_com = fig.add_subplot(3, 4, (9, 10))
    for ci, lbl in enumerate(["X", "Y", "Z"]):
        ax_com.plot(log_t, log_com[:, ci] * 100, lw=1.4, label=f"COM {lbl} [cm]")
    ax_com.set_xlabel("t [s]"); ax_com.set_ylabel("[cm]")
    ax_com.set_title("Center of Mass over time")
    ax_com.legend(fontsize=8); ax_com.grid(alpha=0.3)

    ax_std = fig.add_subplot(3, 4, (11, 12))
    for ci, lbl in enumerate(["X", "Y", "Z"]):
        ax_std.plot(log_t, log_std[:, ci] * 100, lw=1.4, label=f"σ {lbl} [cm]")
    ax_std.set_xlabel("t [s]"); ax_std.set_ylabel("[cm]")
    ax_std.set_title("Particle Spread (std) over time")
    ax_std.legend(fontsize=8); ax_std.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(PLOT_PATH, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {PLOT_PATH}")


# ══════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print(" Medicine Envelope PBD — headless simulation")
    print(f"   W={W*100:.0f}cm  H={H*100:.0f}cm  D={D*100:.1f}cm")
    print(f"   NW={NW}  NH={NH}  ND={ND}  DT={DT}  T={SIM_TIME}s")
    print("=" * 60)

    # ── 1. STL 메시 생성 ──────────────────────────────────────
    mesh_center = create_envelope_stl()
    # euler=(90,0,0): 모델 Y(높이) → World Z.  봉투 하단을 Z=5cm 에 맞추려면:
    #   World_Z_bottom = pos_z - H/2  → pos_z = 0.05 + H/2
    pos_z = 0.05 + H / 2   # 봉투 하단(Y=-H/2 → World Z) = 0.05 m

    # ── 2. Genesis 초기화 (Metal GPU 백엔드) ──────────────────
    import genesis as gs
    gs.init(backend=gs.metal, logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=DT,
            substeps=SUBSTEPS,
            gravity=(0, 0, -9.81),
        ),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25,
            max_bending_solver_iterations=8,
            max_volume_solver_iterations=3,
            max_density_solver_iterations=1,
            max_viscosity_solver_iterations=1,
            particle_size=2e-3,
        ),
        coupler_options=gs.options.LegacyCouplerOptions(
            rigid_pbd=True,
            rigid_mpm=False, rigid_sph=False, rigid_fem=False,
            mpm_sph=False,   mpm_pbd=False,  fem_mpm=False, fem_sph=False,
        ),
        show_viewer=False,
    )

    # 바닥
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())

    # 약봉투 PBD Cloth
    #   euler=(90,0,0): 모델 Y축(높이) → World Z(중력 방향과 평행)
    #   mesh 는 origin 중심 → pos 로 원하는 위치로 이동
    #   pos_z = 0.05 + H/2 → 봉투 하단이 Ground+5cm 위에 위치
    envelope = scene.add_entity(
        material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(
            file=STL_PATH,
            scale=1.0,
            pos=(0.0, 0.0, pos_z),
            euler=(90.0, 0.0, 0.0),
        ),
    )

    print("[build] starting ...")
    t_build = time.time()
    scene.build(n_envs=0)
    print(f"[build] OK  ({time.time()-t_build:.1f}s)")

    # ── 3. 초기 파티클 정보 ───────────────────────────────────
    pos0 = _get_pos(envelope)
    N    = pos0.shape[0]
    print(f"[mesh] N_particles = {N}")
    print(f"[mesh] bbox  "
          f"X=[{pos0[:,0].min():.4f}, {pos0[:,0].max():.4f}]  "
          f"Y=[{pos0[:,1].min():.4f}, {pos0[:,1].max():.4f}]  "
          f"Z=[{pos0[:,2].min():.4f}, {pos0[:,2].max():.4f}]  [m]")

    # ── 4. 상단 코너 고정 ──────────────────────────────────────
    fix_idx = find_top_corners(pos0)
    envelope.fix_particles(particles_idx_local=fix_idx)
    print(f"[fix] {len(fix_idx)} top-corner particles fixed")
    for idx in fix_idx:
        p = pos0[idx]
        print(f"      idx={idx:5d}  pos=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})")

    # ── 5. 헤드리스 시뮬 ─────────────────────────────────────
    n_steps    = int(SIM_TIME / DT)
    snapshots  = {0.0: pos0.copy()}
    log_t, log_com, log_std, log_nan = [], [], [], []

    print(f"\n[sim] {n_steps} steps  log_every={LOG_EVERY}")
    print(f"  {'t[s]':>6s} | {'NaN':>5s} | {'COM_Z[cm]':>10s} | {'σZ[cm]':>8s}")
    print("  " + "-" * 40)

    sim_t0 = time.time()
    for k in range(n_steps):
        scene.step()
        t_cur = (k + 1) * DT

        # 스냅샷 저장
        for st in SNAP_TIMES[1:]:
            if st not in snapshots and abs(t_cur - st) < DT * 0.5:
                snapshots[st] = _get_pos(envelope).copy()

        # 로그
        if (k + 1) % LOG_EVERY == 0:
            pos    = _get_pos(envelope)
            n_nan  = int(np.isnan(pos).any(axis=1).sum())
            valid  = pos[~np.isnan(pos).any(axis=1)]
            com    = valid.mean(axis=0)
            std    = valid.std(axis=0)
            log_t.append(t_cur)
            log_com.append(com)
            log_std.append(std)
            log_nan.append(n_nan)
            print(f"  {t_cur:6.3f} | {n_nan:5d} | "
                  f"{com[2]*100:10.3f} | {std[2]*100:8.3f}")

    print(f"\n[sim] wall-time = {time.time()-sim_t0:.1f}s")

    log_t   = np.array(log_t)
    log_com = np.array(log_com)
    log_std = np.array(log_std)
    log_nan = np.array(log_nan)

    # ── 6. 그래프 저장 ────────────────────────────────────────
    make_plot(N, snapshots, log_t, log_com, log_std, log_nan)

    # ── 7. CSV 저장 ───────────────────────────────────────────
    with open(CSV_PATH, "w", encoding="utf-8") as fh:
        fh.write("t_s,com_x_m,com_y_m,com_z_m,"
                 "std_x_m,std_y_m,std_z_m,n_nan\n")
        for i in range(len(log_t)):
            fh.write(
                f"{log_t[i]:.4f},"
                f"{log_com[i,0]:.6f},{log_com[i,1]:.6f},{log_com[i,2]:.6f},"
                f"{log_std[i,0]:.6f},{log_std[i,1]:.6f},{log_std[i,2]:.6f},"
                f"{log_nan[i]}\n"
            )
    print(f"[saved] {CSV_PATH}")
    print("\n완료.")


if __name__ == "__main__":
    main()
