"""
medicine_envelope_m0609_grasp.py
Doosan M0609 (MJCF) 이 열린 약봉투(PBD cloth)를 잡고 들어올리는 헤드리스 시뮬.

핵심 설계 (이전 논의 반영):
  - Coupler: LegacyCoupler(rigid_pbd=True)  ← Genesis 1.1.0 에서 PBD↔Rigid 는 Legacy 만 지원
  - 잡기 방식: "단순 평행조 + attachment"
      · 마찰-핀치(두 층 압착)로 인한 PBD 충돌제약 위반을 회피하기 위해
        grip 파티클을 fix_particles_to_link() 로 그리퍼 링크(link_6)에 부착해서 잡는다.
  - 들어올리기: fix_particles() 는 월드 고정이라 안 올라감 → fix_particles_to_link 로 해결.
      · 접근 동안엔 봉투 mouth 코너를 월드 고정(fix_particles)으로 매달아 둠.
      · 잡는 순간: mouth-band(코너 제외)를 link_6 에 부착 → 코너 월드고정 해제(release_particle).
        (release_particle 은 link-attach 도 함께 지우므로 두 집합을 분리한다)
  - particle 수: 강성이 커서 절반으로 감축 (particle_size 2e-3 → 2.83e-3, N≈3050)

팔 제어: MJCF position actuator(kp=10)는 너무 약해 자중도 못 버팀 →
         set_dofs_position 으로 관절을 운동학적(kinematic)으로 보간 구동.
         (부착 파티클은 링크 transform 을 따라가므로 운동학 구동으로 충분)

출력(Sim_result/):
  m0609_grasp_lift.png   : EE / 봉투 COM / 봉투 바닥 높이 시계열 (들림 검증)
  m0609_grasp_lift.gif   : 파티클 + 그리퍼(평행조) 3D 애니메이션
  m0609_grasp_lift.csv
"""

import os, time
import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# ─── 봉투 ────────────────────────────────────────────────
W, H, D = 0.08, 0.12, 0.01
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3            # 절반 수준 (N≈3050)

# ─── 시뮬 ────────────────────────────────────────────────
DT, SUBSTEPS = 5e-4, 10
SNAP_EVERY = 30

# ─── 팔 waypoint (FK 로 확인된 관절각) ────────────────────
Q_APPROACH = np.array([0, -0.40, 0.85, 0, 1.05, 0], float)   # link_6 z≈0.853 (mouth 위)
Q_GRASP    = np.array([0, -0.50, 1.00, 0, 1.10, 0], float)   # link_6 z≈0.815 (mouth)
Q_LIFT     = np.array([0, -0.05, 0.25, 0, 0.40, 0], float)   # link_6 z≈1.005 (+19cm)

# grasp point(=mouth center) = FK(Q_GRASP) link_6 위치
GRASP_PT = np.array([0.10, 0.006, 0.815])
BAG_POS  = (GRASP_PT[0], GRASP_PT[1], GRASP_PT[2] - H/2)      # mouth 가 grasp_pt 에 오도록

# phase 별 step 수
#   접근 시 봉투가 낙하→과신장→들어올릴 때 폭발하는 문제를 피하기 위해
#   hanging-approach 를 제거하고 grasp 자세에서 즉시 부착(t=0) 후 천천히 들어올린다.
N_SETTLE   = 800     # 0.40s (부착 후 봉투가 그리퍼에 매달려 안정화)
N_LIFT     = 2400    # 1.20s (천천히)
N_HOLD     = 400     # 0.20s

# ─── 출력 ────────────────────────────────────────────────
_DIR    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result")
os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "medicine_envelope_open.stl")
PNG_PATH = os.path.join(OUT_DIR, "m0609_grasp_lift.png")
GIF_PATH = os.path.join(OUT_DIR, "m0609_grasp_lift.gif")
CSV_PATH = os.path.join(OUT_DIR, "m0609_grasp_lift.csv")


# ══════════════════════════════════════════════════════════
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
    v = np.array([p for t in tris for p in t])
    f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)


def npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def pos_of(bag):
    x = npy(bag.get_particles_pos())
    return x[0] if x.ndim == 3 else x


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def lerp(a, b, s):
    return a + (b - a) * s


# ══════════════════════════════════════════════════════════
def main():
    print("="*60)
    print(" M0609 grasp & lift of open-top medicine envelope (PBD)")
    print("="*60)
    make_bag()

    import genesis as gs
    # ── 헤드리스 가드 ──────────────────────────────────────
    # Genesis 는 scene.build() 시 항상 OffscreenRenderer(GL)를 생성하는데,
    # 디스플레이가 없는(잠금/슬립) 백그라운드에서는 pyglet 이 화면을 못 찾아
    # 'IndexError: list index out of range' 로 죽는다.
    # 카메라를 쓰지 않고 matplotlib 으로만 시각화하므로 GL 생성을 건너뛴다.
    from genesis.vis.rasterizer import Rasterizer
    def _headless_rasterizer_build(self):
        if self._context is not None:
            self.visualizer = self._context.visualizer
    Rasterizer.build = _headless_rasterizer_build

    gs.init(backend=gs.metal, logging_level="warning")
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(
            max_stretch_solver_iterations=25, max_bending_solver_iterations=8,
            max_volume_solver_iterations=3, max_density_solver_iterations=2,
            max_viscosity_solver_iterations=1, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        show_viewer=False,
    )
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid())
    robot = scene.add_entity(gs.morphs.MJCF(file="robots/m0609/m0609.xml"))
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)))

    scene.build(n_envs=0)
    ee = robot.get_link("link_6")
    ee_link_idx = ee.idx          # global link idx for coupler
    ee_i = ee.idx_local

    # 팔을 grasp 자세로 초기화 (link_6 가 봉투 mouth 위치)
    robot.set_dofs_position(Q_GRASP)

    pos0 = pos_of(bag)
    N = pos0.shape[0]
    z = pos0[:, 2]
    # mouth band = 상단 8% → link_6 에 부착할 grip 파티클
    band = np.where(z >= np.quantile(z, 0.92))[0]
    attach_idx = band.copy()
    print(f"[bag] N={N}  attach(mouth-band)={len(attach_idx)}")
    print(f"[bag] mouth z≈{z[band].mean():.3f}  grasp_pt={GRASP_PT}")

    # ── 로깅 ─────────────────────────────────────────────
    t_log, ee_z, com_z, bot_z, band_z, phase_log = [], [], [], [], [], []
    frames_p, frames_ee, frames_q = [], [], []
    phase_bounds = {}

    def record(t, phase):
        p = pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        ep = npy(robot.get_links_pos()); eq = npy(robot.get_links_quat())
        if ep.ndim == 3: ep = ep[0]; eq = eq[0]
        t_log.append(t); ee_z.append(ep[ee_i, 2]); com_z.append(v[:, 2].mean())
        bot_z.append(v[:, 2].min()); band_z.append(p[band, 2].mean()); phase_log.append(phase)
        frames_p.append(v.astype(np.float32)); frames_ee.append(ep[ee_i].copy())
        frames_q.append(eq[ee_i].copy())

    step = 0
    def run_phase(name, q0, q1, n, attach_at_start=False):
        nonlocal step
        phase_bounds[name] = step
        if attach_at_start:
            bag.fix_particles_to_link(link_idx=ee_link_idx, particles_idx_local=attach_idx)
            print(f"[grasp] attached {len(attach_idx)} mouth particles → link_6")
        for k in range(n):
            s = (k + 1) / n
            robot.set_dofs_position(lerp(q0, q1, s))
            scene.step()
            step += 1
            if step % SNAP_EVERY == 0:
                record(step * DT, name)

    print("\n[phase] GRASP+SETTLE ...")
    run_phase("settle", Q_GRASP, Q_GRASP, N_SETTLE, attach_at_start=True)
    print("[phase] LIFT ...")
    run_phase("lift", Q_GRASP, Q_LIFT, N_LIFT)
    print("[phase] HOLD ...")
    run_phase("hold", Q_LIFT, Q_LIFT, N_HOLD)

    t_log = np.array(t_log); ee_z = np.array(ee_z); com_z = np.array(com_z)
    bot_z = np.array(bot_z); band_z = np.array(band_z)

    # ── 플롯 ─────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(t_log, ee_z*100, color="k", lw=2, label="EE (link_6) z")
    ax.plot(t_log, band_z*100, color="tab:red", lw=1.8, label="grasped mouth-band z")
    ax.plot(t_log, com_z*100, color="tab:blue", lw=1.8, label="bag COM z")
    ax.plot(t_log, bot_z*100, color="tab:green", lw=1.6, label="bag bottom z")
    for nm, st in phase_bounds.items():
        ax.axvline(st*DT, color="gray", ls=":", lw=1)
        ax.text(st*DT, ax.get_ylim()[1], nm, fontsize=8, rotation=90, va="top", color="gray")
    ax.set_xlabel("t [s]"); ax.set_ylabel("height [cm]")
    ax.set_title("M0609 grasp & lift — heights over time (lift verification)")
    ax.legend(fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(PNG_PATH, dpi=140, bbox_inches="tight"); plt.close(fig)
    print(f"[saved] {PNG_PATH}")

    # ── 3D 애니메이션 (파티클 + 평행조 그리퍼) ───────────
    print("[anim] rendering GIF ...")
    figa = plt.figure(figsize=(7, 8)); ax = figa.add_subplot(111, projection="3d")
    FL, GAP = 0.05, 0.025      # finger length / half-gap [m]

    def gripper_segs(ep, eq):
        R = quat_to_R(eq)
        segs = []
        for sgn in (+1, -1):
            base = ep + R @ np.array([sgn*GAP, 0, 0])
            tip  = base + R @ np.array([0, 0, FL])
            segs.append([base*100, tip*100])
        # 두 finger 연결(평행조 베이스)
        b1 = ep + R @ np.array([ GAP, 0, 0]); b2 = ep + R @ np.array([-GAP, 0, 0])
        segs.append([b1*100, b2*100])
        return segs

    def draw(i):
        ax.clear()
        p = frames_p[i]
        ax.scatter(p[:,0]*100, p[:,1]*100, p[:,2]*100, s=2, alpha=0.3, color="tab:blue")
        segs = gripper_segs(frames_ee[i], frames_q[i])
        ax.add_collection3d(Line3DCollection(segs, colors="k", linewidths=2.5))
        ax.scatter([frames_ee[i][0]*100],[frames_ee[i][1]*100],[frames_ee[i][2]*100],
                   color="tab:red", s=25)
        ax.set_xlim(-5,25); ax.set_ylim(-15,15); ax.set_zlim(50,115)
        ax.set_xlabel("X[cm]"); ax.set_ylabel("Y[cm]"); ax.set_zlabel("Z[cm]")
        ax.set_title(f"t={t_log[i]:.2f}s [{phase_log[i]}]  EE_z={frames_ee[i][2]*100:.1f}  "
                     f"COM_z={com_z[i]*100:.1f} cm")
        ax.view_init(elev=10, azim=-65)
        return ()

    anim = FuncAnimation(figa, draw, frames=len(frames_p), interval=120)
    anim.save(GIF_PATH, writer=PillowWriter(fps=10)); plt.close(figa)
    print(f"[saved] {GIF_PATH}")

    # ── CSV + 요약 ───────────────────────────────────────
    with open(CSV_PATH, "w") as fh:
        fh.write("t_s,phase,ee_z_m,band_z_m,com_z_m,bottom_z_m\n")
        for i in range(len(t_log)):
            fh.write(f"{t_log[i]:.4f},{phase_log[i]},{ee_z[i]:.6f},"
                     f"{band_z[i]:.6f},{com_z[i]:.6f},{bot_z[i]:.6f}\n")
    print(f"[saved] {CSV_PATH}")

    dz_ee  = (ee_z[-1]  - ee_z[0]) * 100
    dz_com = (com_z[-1] - com_z[0]) * 100
    print("\n" + "="*60)
    print(f" EE 상승량      : {dz_ee:+.2f} cm")
    print(f" 봉투 COM 상승량: {dz_com:+.2f} cm   (EE 따라 올라가면 grasp 성공)")
    print(f" 추종율 COM/EE  : {dz_com/dz_ee*100 if dz_ee else 0:.1f} %")
    print(f" 봉투 바닥 z    : {bot_z[0]*100:.1f} → {bot_z[-1]*100:.1f} cm")
    print("="*60)
    print("완료.")


if __name__ == "__main__":
    main()
