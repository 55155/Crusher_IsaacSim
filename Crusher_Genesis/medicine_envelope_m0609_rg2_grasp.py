"""
medicine_envelope_m0609_rg2_grasp.py
Doosan M0609 + OnRobot RG2 (실제 mesh, 단순 평행조 prismatic) 로
열린 약봉투(PBD cloth)를 물리적으로 닫아 집고 들어올리는 헤드리스 시뮬.

모델: robots/m0609_rg2.xml  (8-DOF: 팔 6 + RG2 핑거 2)
  · RG2 mesh = AndrejOrsula/ur5_rg2_ign (hand, finger) — 실제 RG2 형상
  · 핑거 = prismatic ±y (0=닫힘, 0.05=열림), link_6 +z(approach) 에 hand +x 정렬
  · ※ GitHub 에 RG2-v2 정확본이 없어 RG2 mesh + 평행조 URDF 를 직접 구성

파지 방식 (하이브리드):
  1) 봉투를 그리퍼 throat 에 두고 코너 월드고정으로 매달아 둠
  2) 핑거를 물리적으로 닫아 봉투 양면에 접촉(실제 RG2 동작)
  3) throat 안 파티클을 rg2_hand 링크에 부착(fix_particles_to_link) + 코너 해제
       → 안정적 리프트 확보 (순수 마찰 파지는 2.1 압착이슈로 다음 단계)
  4) 팔을 들어올림 → 봉투 추종

제어: set_dofs_position 운동학 구동 (8-DOF 모두).

출력(Sim_result/):
  m0609_rg2_grasp.png / .gif / .csv
"""

import os, time
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Line3DCollection

W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3

DT, SUBSTEPS = 5e-4, 10
SNAP_EVERY = 30

# 팔 자세 (FK 확인): grasp 에서 link_6 z≈0.815, 핑거 throat z≈0.60~0.71
Q_ARM_GRASP = np.array([0, -0.50, 1.00, 0, 1.10, 0], float)
Q_ARM_LIFT  = np.array([0, -0.05, 0.25, 0, 0.40, 0], float)
FING_OPEN, FING_CLOSE = 0.05, 0.008     # prismatic (m)

# 봉투 mouth 를 throat 상부에 배치
GRASP_XY = np.array([0.10, 0.006])
BAG_MOUTH_Z = 0.70
BAG_POS = (GRASP_XY[0], GRASP_XY[1], BAG_MOUTH_Z - H/2)

# throat(파지) 영역 박스 — 이 안의 파티클을 그리퍼에 부착
THROAT = dict(x=(0.07, 0.13), y=(-0.012, 0.024), z=(0.62, 0.71))

N_SETTLE = 400     # 0.20s 매달림 안정화
N_CLOSE  = 600     # 0.30s 핑거 닫기
N_GRASP  = 200     # 0.10s 부착 후 안정화
N_LIFT   = 2200    # 1.10s 리프트
N_HOLD   = 400     # 0.20s

_DIR    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
PNG_PATH = os.path.join(OUT_DIR, "m0609_rg2_grasp.png")
GIF_PATH = os.path.join(OUT_DIR, "m0609_rg2_grasp.gif")
CSV_PATH = os.path.join(OUT_DIR, "m0609_rg2_grasp.csv")


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a = fn(i/nu, j/nv); b = fn((i+1)/nu, j/nv)
            c = fn((i+1)/nu, (j+1)/nv); d = fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t


def make_bag():
    tris  = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, v*H, D  ]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)
    v = np.array([p for t in tris for p in t]); f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7); m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)


def npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
def pos_of(bag):
    x = npy(bag.get_particles_pos()); return x[0] if x.ndim == 3 else x
def lerp(a, b, s): return a + (b - a) * s


def main():
    print("="*60); print(" M0609 + RG2 grasp & lift (real gripper mesh)"); print("="*60)
    make_bag()

    import genesis as gs
    from genesis.vis.rasterizer import Rasterizer
    Rasterizer.build = (lambda self: setattr(self, "visualizer", self._context.visualizer)
                        if self._context is not None else None)
    gs.init(backend=gs.metal, logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25, max_bending_solver_iterations=8,
            max_volume_solver_iterations=3, max_density_solver_iterations=2,
            max_viscosity_solver_iterations=1, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        show_viewer=False)
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())
    robot = scene.add_entity(gs.morphs.MJCF(file="robots/m0609_rg2.xml"))
    bag = scene.add_entity(material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)))
    scene.build(n_envs=0)

    names = [l.name for l in robot.links]
    hand = robot.get_link("rg2_hand"); hand_idx = hand.idx
    li = {n: i for i, n in enumerate(names)}

    # 시작: 팔 grasp 자세 + 핑거 열림
    q0 = np.concatenate([Q_ARM_GRASP, [FING_OPEN, FING_OPEN]])
    robot.set_dofs_position(q0)

    pos0 = pos_of(bag); N = pos0.shape[0]
    z = pos0[:, 2]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    x, y = pos0[:, 0], pos0[:, 1]
    corners = list({int(i) for i in [
        band[np.argmin(x[band]+y[band])], band[np.argmax(x[band]-y[band])],
        band[np.argmin(-x[band]+y[band])], band[np.argmax(x[band]+y[band])]]})
    # throat 안 파티클 = 그리퍼 부착 대상 (코너 제외)
    in_throat = ((x>=THROAT['x'][0])&(x<=THROAT['x'][1])&
                 (y>=THROAT['y'][0])&(y<=THROAT['y'][1])&
                 (z>=THROAT['z'][0])&(z<=THROAT['z'][1]))
    grip_idx = np.array([i for i in np.where(in_throat)[0] if i not in corners])
    print(f"[bag] N={N}  band={len(band)}  throat-grip={len(grip_idx)}  corners={len(corners)}")
    print(f"[bag] mouth z≈{z[band].mean():.3f}")
    if len(grip_idx) < 5:
        print("!! throat grip 파티클이 너무 적음 — THROAT/배치 점검 필요")

    bag.fix_particles(particles_idx_local=corners)

    t_log, ee_z, com_z, bot_z, grip_z, fopen_log, phase_log = [], [], [], [], [], [], []
    frames_p, frames_L, frames_R, frames_hand = [], [], [], []
    pbounds = {}

    def record(t, phase, fopen):
        p = pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        lp = npy(robot.get_links_pos());  lp = lp[0] if lp.ndim == 3 else lp
        t_log.append(t); ee_z.append(lp[li['link_6'], 2]); com_z.append(v[:, 2].mean())
        bot_z.append(v[:, 2].min()); grip_z.append(p[grip_idx, 2].mean() if len(grip_idx) else np.nan)
        fopen_log.append(fopen); phase_log.append(phase)
        frames_p.append(v.astype(np.float32))
        frames_L.append(lp[li['rg2_left']].copy()); frames_R.append(lp[li['rg2_right']].copy())
        frames_hand.append(lp[li['rg2_hand']].copy())

    step = 0
    def run(name, qa0, qa1, f0, f1, n, attach=False, release=False):
        nonlocal step
        pbounds[name] = step
        if attach:
            bag.fix_particles_to_link(link_idx=hand_idx, particles_idx_local=grip_idx)
            print(f"[grasp] attached {len(grip_idx)} throat particles → rg2_hand")
        if release:
            bag.release_particle(particles_idx_local=corners)
            print(f"[grasp] released {len(corners)} corners")
        for k in range(n):
            s = (k+1)/n
            qa = lerp(qa0, qa1, s); f = lerp(f0, f1, s)
            robot.set_dofs_position(np.concatenate([qa, [f, f]]))
            scene.step(); step += 1
            if step % SNAP_EVERY == 0:
                record(step*DT, name, f)

    print("\n[phase] SETTLE (open, hanging)"); run("settle", Q_ARM_GRASP, Q_ARM_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    print("[phase] CLOSE fingers");           run("close",  Q_ARM_GRASP, Q_ARM_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE)
    print("[phase] GRASP attach + release");  run("grasp",  Q_ARM_GRASP, Q_ARM_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP, attach=True, release=True)
    print("[phase] LIFT");                    run("lift",   Q_ARM_GRASP, Q_ARM_LIFT,  FING_CLOSE, FING_CLOSE, N_LIFT)
    print("[phase] HOLD");                    run("hold",   Q_ARM_LIFT,  Q_ARM_LIFT,  FING_CLOSE, FING_CLOSE, N_HOLD)

    t_log=np.array(t_log); ee_z=np.array(ee_z); com_z=np.array(com_z)
    bot_z=np.array(bot_z); grip_z=np.array(grip_z)

    # ── 플롯 ──
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_log, ee_z*100, 'k', lw=2, label="EE (link_6) z")
    ax.plot(t_log, grip_z*100, color="tab:red", lw=1.8, label="gripped particles z")
    ax.plot(t_log, com_z*100, color="tab:blue", lw=1.8, label="bag COM z")
    ax.plot(t_log, bot_z*100, color="tab:green", lw=1.6, label="bag bottom z")
    for nm, st in pbounds.items():
        ax.axvline(st*DT, color="gray", ls=":", lw=1)
        ax.text(st*DT, ax.get_ylim()[1], nm, fontsize=8, rotation=90, va="top", color="gray")
    ax.set_xlabel("t [s]"); ax.set_ylabel("height [cm]")
    ax.set_title("M0609 + RG2 — grasp & lift heights (real gripper)")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(PNG_PATH, dpi=140, bbox_inches="tight"); plt.close(fig)
    print(f"[saved] {PNG_PATH}")

    # ── 애니메이션: 파티클 + RG2 핑거(실제 링크 위치) ──
    print("[anim] GIF ...")
    figa = plt.figure(figsize=(7, 8)); ax = figa.add_subplot(111, projection="3d")
    def draw(i):
        ax.clear()
        p = frames_p[i]
        ax.scatter(p[:,0]*100, p[:,1]*100, p[:,2]*100, s=2, alpha=0.3, color="tab:blue")
        L, R, Hd = frames_L[i]*100, frames_R[i]*100, frames_hand[i]*100
        ax.add_collection3d(Line3DCollection([[Hd, L], [Hd, R]], colors="k", linewidths=2.5))
        ax.scatter([L[0],R[0]],[L[1],R[1]],[L[2],R[2]], color="tab:red", s=40, marker="s")
        ax.scatter([Hd[0]],[Hd[1]],[Hd[2]], color="k", s=30)
        ax.set_xlim(0,20); ax.set_ylim(-10,10); ax.set_zlim(45,110)
        ax.set_xlabel("X[cm]"); ax.set_ylabel("Y[cm]"); ax.set_zlabel("Z[cm]")
        ax.set_title(f"t={t_log[i]:.2f}s [{phase_log[i]}] finger={fopen_log[i]*1000:.0f}mm "
                     f"COM_z={com_z[i]*100:.1f}")
        ax.view_init(elev=10, azim=-65); return ()
    FuncAnimation(figa, draw, frames=len(frames_p), interval=120).save(GIF_PATH, writer=PillowWriter(fps=10))
    plt.close(figa); print(f"[saved] {GIF_PATH}")

    with open(CSV_PATH, "w") as fh:
        fh.write("t_s,phase,finger_open_m,ee_z_m,grip_z_m,com_z_m,bottom_z_m\n")
        for i in range(len(t_log)):
            fh.write(f"{t_log[i]:.4f},{phase_log[i]},{fopen_log[i]:.4f},{ee_z[i]:.6f},"
                     f"{grip_z[i]:.6f},{com_z[i]:.6f},{bot_z[i]:.6f}\n")
    print(f"[saved] {CSV_PATH}")

    dz_ee=(ee_z[-1]-ee_z[0])*100; dz_com=(com_z[-1]-com_z[0])*100
    print("\n"+"="*60)
    print(f" EE 상승량 {dz_ee:+.2f} cm | 봉투 COM 상승량 {dz_com:+.2f} cm | 추종율 {dz_com/dz_ee*100 if dz_ee else 0:.0f}%")
    print(f" 봉투 바닥 z: {bot_z[0]*100:.1f} → {bot_z[-1]*100:.1f} cm")
    print("="*60); print("완료.")


if __name__ == "__main__":
    main()
