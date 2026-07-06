"""
ipc_grasp_bag_test.py — 최소 격리 테스트 (IPC 커플러 + FEM cloth 샘플백).

목적
----
Crushing.py 전체를 FEM+IPC 로 포팅하기 전, 다음을 *격리*해서 검증한다:
  1. GPU(gs.cuda) + IPCCouplerOptions 에서 M0609+RG2 (articulated rigid) 와
     FEM.Cloth 봉투가 공존하며 crash(cudaErrorInvalidDevice) 없이 build·step 되는가.
  2. IPC 접촉: RG2 핑거를 닫을 때 rigid↔FEM-cloth 접촉으로 천이 눌리는가.
  3. 파지 유지: 참조 예제(examples/IPC_Solver/ipc_robot_grasp_cube.py) 패턴
       robot = Rigid(coup_type='two_way_soft_constraint',
                     coup_links=('rg2_left','rg2_right'), coup_friction=..)
     + FEM set_vertex_constraints(strip, link=finger) 로 봉투를 핑거에 구속 →
     top 구속 해제 후 arm 을 올리면 봉투가 그리퍼를 따라오는가.

PBD 버전(Crushing.py)과의 대응
------------------------------
  bag.fix_particles(idx)              → bag.set_vertex_constraints(idx)                (제자리 고정)
  bag.fix_particles_to_link(link,idx) → bag.set_vertex_constraints(idx, link=finger)   (링크 구속)
  bag.release_particle(idx)           → bag.remove_vertex_constraints(idx)
  bag.get_particles_pos()             → bag.get_state().pos

주의: IPC 는 PBD 를 못 본다(커플러가 fem_solver+rigid_solver 만 등록). 그래서 봉투는
      반드시 FEM.Cloth. contact_d_hat 은 봉투 두께(6mm)보다 작게(1mm) 잡아 초기 barrier
      위반을 피한다.
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm
from PIL import Image

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 봉투 형상 (Crushing.make_bag 와 동일 파라미터, 5-panel open pouch) ──────────
W, H, D = 0.05, 0.08, 0.006          # local: x=폭, y=높이, z=두께
NW, NH, ND = 6, 9, 2

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
IPC_D_HAT = 1.0e-3                    # 1mm < 봉투 두께 6mm → 초기 barrier off
RENDER_EVERY = 4

# ── FEM cloth 강성 (grasp 예제 참고: E=1e5, nu=0.499, thin shell) ─────────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

# ── 로봇 자세 (FK probe 로 확인: 이 자세에서 핑거 중점 ≈ (0.18,0.006,0.52)) ──
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)   # arm 을 올려 봉투 들어올림
FING_OPEN, FING_CLOSE = 0.040, 0.006

# 봉투는 열린 핑거 중점에 맞춰 배치 (FK probe 결과)
FINGER_MID = np.array([0.1823, 0.0062, 0.5223])
BAG_POS    = tuple(FINGER_MID)        # 봉투 중심 = 핑거 중점
BAG_EULER  = (90, 0, 0)               # 두께(6mm)를 y(핑거 닫힘축)로, 높이(8cm)를 z 로

# ── 페이즈 스텝 ──────────────────────────────────────────────────────────────
N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 120, 80, 40, 200, 100

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
_ROBOT_DIR = os.path.join(os.path.dirname(_DIR), "robots")   # Crusher_Genesis/robots
ROBOT_MJCF = os.path.join(_ROBOT_DIR, "m0609_rg2.xml")
OUT_DIR = os.path.join(os.path.dirname(_DIR), "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
STL_PATH = os.path.join(OUT_DIR, "_ipc_test_bag.stl")
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"ipc_grasp_bag_{_TS}.mp4")


def _npy(x): return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _panel(fn, nu, nv):
    t = []
    for i in range(nu):
        for j in range(nv):
            a, b = fn(i/nu, j/nv), fn((i+1)/nu, j/nv)
            c, d = fn((i+1)/nu, (j+1)/nv), fn(i/nu, (j+1)/nv)
            t += [[a, b, c], [a, c, d]]
    return t


def make_bag():
    tris = []
    tris += _panel(lambda u, v: np.array([u*W, v*H, 0.0]), NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, v*H, D]),   NW, NH)
    tris += _panel(lambda u, v: np.array([u*W, 0.0, v*D]), NW, ND)
    tris += _panel(lambda u, v: np.array([0.0, u*H, v*D]), NH, ND)
    tris += _panel(lambda u, v: np.array([W,   u*H, v*D]), NH, ND)
    v = np.array([p for t in tris for p in t])
    f = np.arange(len(v)).reshape(-1, 3)
    m = tm.Trimesh(vertices=v, faces=f, process=False)
    m.merge_vertices(digits_vertex=7)
    m.vertices -= m.bounding_box.centroid
    m.export(STL_PATH)
    print(f"[bag] {len(m.vertices)} verts, {len(m.faces)} faces → {STL_PATH}")


def main(use_viewer: bool = False):
    print("="*60)
    print(f" IPC grasp bag test — M0609+RG2 + FEM cloth  (viewer={use_viewer})")
    print("="*60)
    make_bag()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="info", precision="32")

    scene = gs.Scene(
        # 중력 OFF: 봉투를 핑거 높이에 띄워 두고 순수 IPC 접촉만 관찰
        # (얇은 봉투는 RG2 gap(~46mm) 로 pinch 불가 → vertex-pin 없이 접촉 변형만 검증)
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, 0)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,     # 로봇 self / plate 끼리 불필요
            enable_rigid_ground_contact=False,    # ground↔rigid 불필요 (봉투만 관심)
            constraint_strength_translation=10.0,
            constraint_strength_rotation=10.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    # ground (봉투가 떨어지면 받도록) — plane geom 은 IPC 에서 ipc_only 만 허용
    scene.add_entity(gs.morphs.Plane(),
                     material=gs.materials.Rigid(coup_type="ipc_only"))

    # 로봇: 핑거만 IPC 접촉 (two_way_soft_constraint)
    robot = scene.add_entity(
        gs.morphs.MJCF(file=ROBOT_MJCF, decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=("rg2_left", "rg2_right"),
            coup_friction=CLOTH_FRICTION,
        ),
    )

    # FEM cloth 봉투
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.75,
                                    roughness=0.9, double_sided=True),
    )

    cam = scene.add_camera(res=(960, 720), pos=(0.55, -0.55, 0.62),
                           lookat=(0.18, 0.0, 0.50), fov=42, GUI=False)

    scene.build(n_envs=0)

    # ── 시작 자세 (핑거 열림) ──
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN, FING_OPEN]]))
    left_link = robot.get_link("rg2_left")
    grip_link_idx = left_link.idx

    # ── 봉투 정점 위치로 top edge / grip strip 선정 ──
    vp = _npy(bag.get_state().pos).squeeze()
    print(f"[bag] built verts={vp.shape}  z={vp[:,2].min():.3f}~{vp[:,2].max():.3f}  "
          f"x={vp[:,0].min():.3f}~{vp[:,0].max():.3f}  y={vp[:,1].min():.3f}~{vp[:,1].max():.3f}")
    # grip strip: 핑거 중점 근처 (접촉 관찰용 진단 인덱스)
    d_to_mid = np.linalg.norm(vp - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag] grip_strip verts near finger_mid={len(grip_idx)}")
    # NOTE: set_vertex_constraints 는 IPC 커플러에서 막힘(fem_entity.py:907) →
    #       중력 OFF 로 봉투를 띄워 두고 순수 IPC 접촉만 본다. vertex-pin 없음.

    frames = []
    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out   # (rgb, depth, seg, normal)
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"ipc_grasp_bag_{name}.png"))

    def run(name, q0, q1, f0, f1, n):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f, f]]))
            scene.step()
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        vpn = _npy(bag.get_state().pos).squeeze()
        print(f"[phase] {name:8s} @done  bag_com=({vpn[:,0].mean():.3f},"
              f"{vpn[:,1].mean():.3f},{vpn[:,2].mean():.3f})  "
              f"grip_com_z={vpn[grip_idx,2].mean():.3f}" if len(grip_idx) else f"[phase] {name}")

    com0 = _npy(bag.get_state().pos).squeeze().mean(axis=0)
    # 1) settle: 봉투 띄운 채 안정화 (핑거 열림)
    run("settle", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    # 2) close: 핑거 닫음 → IPC 접촉으로 천이 밀리는가?
    run("close",  Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE)
    # 3) hold: 접촉 유지
    run("hold",   Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP + N_HOLD)

    # 접촉 판정: 닫은 뒤 봉투 정점이 초기 대비 움직였으면 IPC 접촉이 작용한 것
    vpf = _npy(bag.get_state().pos).squeeze()
    comf = vpf.mean(axis=0)
    max_disp = float(np.linalg.norm(vpf - vp, axis=1).max())
    print(f"\n[contact-check] bag com {np.round(com0,4)} → {np.round(comf,4)}  "
          f"(shift {np.linalg.norm(comf-com0)*1e3:.2f} mm)")
    print(f"[contact-check] max vertex displacement = {max_disp*1e3:.2f} mm  "
          f"→ {'CONTACT detected (cloth deformed)' if max_disp>1e-4 else 'NO contact (cloth static)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → Sim_result/ipc_grasp_bag_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=False)
