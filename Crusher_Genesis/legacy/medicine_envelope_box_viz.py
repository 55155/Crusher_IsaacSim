"""
medicine_envelope_box_viz.py
열린 약봉투(open-top) 안으로 작은 Box 를 떨어뜨리고
  (2) 박스가 봉투 안에 담겼는지 실시간 위치로 판정 → containment 플롯
  (3) particle + box 3D 애니메이션(GIF) 으로 직접 시각화

봉투: medicine_envelope.py 와 동일한 5-panel 상단-개방 모델.
  W=8cm(X)  H=12cm(Y→World-Z)  D=1cm(Z→World-Y),  euler=(90,0,0)

박스: Genesis 내장 morphs.Box (rigid). 열린 상단으로 낙하.

판정(containment): 매 프레임 cloth particle 들의 Convex Hull 내부에
                   box 중심이 들어왔는지(Delaunay point-in-hull)로 "담김" 여부 결정.

출력(Sim_result/):
  medicine_envelope_box_containment.png   ← box_z / 담김여부 / COM거리 시계열
  medicine_envelope_box_drop.gif          ← 3D 애니메이션
  medicine_envelope_box_viz.csv

사용:
  python medicine_envelope_box_viz.py
"""

import os, time
import numpy as np
from scipy.spatial import ConvexHull, Delaunay
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# ─── 봉투 파라미터 ───────────────────────────────────────
W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 8, 12, 3
PARTICLE_SIZE = 2e-3

# ─── 시뮬 ────────────────────────────────────────────────
DT, SUBSTEPS = 5e-4, 10
SIM_TIME  = 1.2
SNAP_EVERY = 30          # 프레임(애니메이션/판정) 저장 간격 (step)

# ─── 낙하 박스 ───────────────────────────────────────────
BOX_SIZE   = (0.015, 0.006, 0.015)   # 1.5 × 0.6 × 1.5 cm
BOX_RHO    = 400.0
BOX_DROP_Z = 0.235

# ─── 출력 ────────────────────────────────────────────────
_DIR    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result")
os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
PNG_PATH = os.path.join(OUT_DIR, "medicine_envelope_box_containment.png")
GIF_PATH = os.path.join(OUT_DIR, "medicine_envelope_box_drop.gif")
CSV_PATH = os.path.join(OUT_DIR, "medicine_envelope_box_viz.csv")


# ══════════════════════════════════════════════════════════
def _panel(fn, nu, nv):
    tris = []
    for i in range(nu):
        for j in range(nv):
            u0, u1 = i/nu, (i+1)/nu
            v0, v1 = j/nv, (j+1)/nv
            a, b = fn(u0, v0), fn(u1, v0)
            c, d = fn(u1, v1), fn(u0, v1)
            tris += [[a, b, c], [a, c, d]]
    return tris


def create_open_envelope():
    import trimesh as _tm
    tris  = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)  # Front
    tris += _panel(lambda u, v: np.array([u*W, v*H, D  ]), NW, NH)  # Back
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)  # Bottom
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)  # Left
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)  # Right
    verts = np.array([p for t in tris for p in t])
    faces = np.arange(len(verts)).reshape(-1, 3)
    m = _tm.Trimesh(vertices=verts, faces=faces, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)
    print(f"[mesh] open-top  tris={len(m.faces)}  verts={len(m.vertices)}")


def _pos(entity):
    p = entity.get_particles_pos()
    if hasattr(p, "cpu"):
        p = p.cpu().numpy()
    p = np.asarray(p, dtype=np.float64)
    return p[0] if p.ndim == 3 else p


def _vec(x):
    if hasattr(x, "cpu"):
        x = x.cpu().numpy()
    return np.asarray(x, dtype=np.float64).reshape(-1)


def quat_to_R(q):
    """Genesis quat (w,x,y,z) → 3x3 회전행렬."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def box_corners(pos, quat, size):
    """박스 8 꼭짓점 (월드 좌표)."""
    sx, sy, sz = np.array(size)/2
    local = np.array([[ sx, sy, sz], [-sx, sy, sz], [-sx,-sy, sz], [ sx,-sy, sz],
                      [ sx, sy,-sz], [-sx, sy,-sz], [-sx,-sy,-sz], [ sx,-sy,-sz]])
    R = quat_to_R(quat)
    return (local @ R.T) + pos


_BOX_EDGES = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
              (0,4),(1,5),(2,6),(3,7)]


def find_top_corners(pos0):
    z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    top = np.where(z >= np.quantile(z, 0.95))[0]
    c = [top[np.argmin(x[top]+y[top])], top[np.argmax(x[top]-y[top])],
         top[np.argmin(-x[top]+y[top])], top[np.argmax(x[top]+y[top])]]
    return list({int(i) for i in c})


# ══════════════════════════════════════════════════════════
def main():
    print("="*60)
    print(" Open-top envelope + Box drop : containment + 3D viz")
    print("="*60)
    create_open_envelope()

    import genesis as gs
    gs.init(backend=gs.metal, logging_level="warning")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS,
                                          gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25, max_bending_solver_iterations=8,
            max_volume_solver_iterations=3, max_density_solver_iterations=1,
            max_viscosity_solver_iterations=1, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        show_viewer=False,
    )
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())
    pos_z = 0.05 + H/2
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0,
                             pos=(0, 0, pos_z), euler=(90, 0, 0)))
    box = scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=(0.0, 0.0, BOX_DROP_Z), fixed=False))

    t0 = time.time()
    scene.build(n_envs=0)
    print(f"[build] OK ({time.time()-t0:.1f}s)")

    pos0 = _pos(bag)
    N = pos0.shape[0]
    bag.fix_particles(particles_idx_local=find_top_corners(pos0))
    print(f"[N]={N}  fixed top corners")

    n_steps = int(SIM_TIME / DT)
    frames_p, frames_box = [], []     # 애니메이션용
    t_log, box_z, contained, dist_com, depth = [], [], [], [], []

    print(f"\n  {'t[s]':>6} | {'box_z[cm]':>9} | {'contained':>9} | {'dist_COM[cm]':>12}")
    print("  " + "-"*46)
    ts = time.time()
    for k in range(n_steps):
        scene.step()
        t = (k+1)*DT
        if (k+1) % SNAP_EVERY == 0 or k == 0:
            p   = _pos(bag)
            bp  = _vec(box.get_pos())
            bq  = _vec(box.get_quat())
            valid = p[~np.isnan(p).any(axis=1)]
            com = valid.mean(axis=0)

            # containment: cloth convex hull 내부에 box 중심?
            try:
                hull = ConvexHull(valid)
                inside = Delaunay(valid[hull.vertices]).find_simplex(bp) >= 0
            except Exception:
                inside = False
            d = float(np.linalg.norm(bp - com))
            # 침투 깊이: bag 상단 Z 대비 박스가 얼마나 내려갔나
            bag_top = valid[:, 2].max()
            dep = float(bag_top - bp[2])

            frames_p.append(valid.astype(np.float32))
            frames_box.append(box_corners(bp, bq, BOX_SIZE).astype(np.float32))
            t_log.append(t); box_z.append(bp[2]); contained.append(bool(inside))
            dist_com.append(d); depth.append(dep)

            if (k+1) % (SNAP_EVERY*8) == 0 or k == 0:
                print(f"  {t:6.3f} | {bp[2]*100:9.3f} | {str(bool(inside)):>9} | "
                      f"{d*100:12.3f}")
    print(f"[sim] wall={time.time()-ts:.1f}s   frames={len(frames_p)}")

    t_log    = np.array(t_log)
    box_z    = np.array(box_z)
    contained= np.array(contained)
    dist_com = np.array(dist_com)
    depth    = np.array(depth)

    # ── (2) containment 플롯 ─────────────────────────────
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.5))
    fig.suptitle("Box drop into open-top envelope — containment analysis",
                 fontsize=13, fontweight="bold")
    axs[0].plot(t_log, box_z*100, color="tab:red", lw=1.8, label="box center Z")
    axs[0].axhline(17, color="gray", ls="--", lw=1, label="bag top ~17cm")
    axs[0].axhline(5,  color="black", ls=":", lw=1, label="bag bottom ~5cm")
    axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("Z [cm]")
    axs[0].set_title("Box height"); axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3)

    axs[1].fill_between(t_log, 0, contained.astype(int), step="mid",
                        color="tab:green", alpha=0.5)
    axs[1].plot(t_log, contained.astype(int), color="tab:green", lw=1.5, drawstyle="steps-mid")
    axs[1].set_ylim(-0.1, 1.1); axs[1].set_yticks([0, 1]); axs[1].set_yticklabels(["outside", "inside"])
    axs[1].set_xlabel("t [s]"); axs[1].set_title("Contained? (in cloth convex hull)")
    axs[1].grid(alpha=0.3)

    axs[2].plot(t_log, dist_com*100, color="tab:purple", lw=1.8, label="‖box−bagCOM‖")
    axs[2].plot(t_log, depth*100, color="tab:orange", lw=1.5, label="depth below bag top")
    axs[2].set_xlabel("t [s]"); axs[2].set_ylabel("[cm]")
    axs[2].set_title("Distance to bag COM / insertion depth")
    axs[2].legend(fontsize=8); axs[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(PNG_PATH, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"[saved] {PNG_PATH}")

    # ── (3) 3D 애니메이션 GIF ────────────────────────────
    print("[anim] rendering GIF ...")
    figa = plt.figure(figsize=(7, 8))
    ax = figa.add_subplot(111, projection="3d")

    def draw(i):
        ax.clear()
        p = frames_p[i]
        ax.scatter(p[:,0]*100, p[:,1]*100, p[:,2]*100,
                   s=2, alpha=0.25, color="tab:blue")
        bc = frames_box[i] * 100
        segs = [[bc[a], bc[b]] for a, b in _BOX_EDGES]
        lc = Line3DCollection(segs, colors="tab:red", linewidths=2)
        ax.add_collection3d(lc)
        ax.scatter(bc[:,0], bc[:,1], bc[:,2], color="tab:red", s=12)
        ax.set_xlim(-5, 5); ax.set_ylim(-5, 5); ax.set_zlim(0, 24)
        ax.set_xlabel("X[cm]"); ax.set_ylabel("Y[cm]"); ax.set_zlabel("Z[cm]")
        state = "INSIDE" if contained[i] else "outside"
        ax.set_title(f"t={t_log[i]:.2f}s   box_z={box_z[i]*100:.1f}cm   [{state}]")
        ax.view_init(elev=12, azim=-70)
        return ()

    anim = FuncAnimation(figa, draw, frames=len(frames_p), interval=120)
    anim.save(GIF_PATH, writer=PillowWriter(fps=8))
    plt.close(figa)
    print(f"[saved] {GIF_PATH}")

    # ── CSV ──────────────────────────────────────────────
    with open(CSV_PATH, "w") as fh:
        fh.write("t_s,box_z_m,contained,dist_com_m,depth_below_top_m\n")
        for i in range(len(t_log)):
            fh.write(f"{t_log[i]:.4f},{box_z[i]:.6f},{int(contained[i])},"
                     f"{dist_com[i]:.6f},{depth[i]:.6f}\n")
    print(f"[saved] {CSV_PATH}")

    # ── 요약 ─────────────────────────────────────────────
    n_in = int(contained.sum())
    print("\n" + "="*60)
    print(f" 최종 box_z = {box_z[-1]*100:.2f} cm  (bag bottom≈5, top≈17)")
    print(f" containment: {n_in}/{len(contained)} 프레임 inside  "
          f"(최종 {'INSIDE' if contained[-1] else 'OUTSIDE'})")
    print(f" 최종 insertion depth = {depth[-1]*100:.2f} cm")
    print("="*60)
    print("완료.")


if __name__ == "__main__":
    main()
