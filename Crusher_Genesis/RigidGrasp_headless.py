"""
RigidGrasp_headless.py — Franka Panda + PBD 샘플백 grasp 테스트.

변인통제 단계:
  1. Franka + rigid box  → 성공 확인 ✓ (이전 단계)
  2. Franka + PBD 봉투   → 본 테스트 (이 파일)
  3. M0609+RG2 + 봉투    → 향후 (Crushing.py)

성공 시 → 봉투 grasp 자체는 Franka 패턴으로 가능 → M0609+RG2 가 원인.
실패 시 → PBD-Rigid coupling 자체에 추가 이슈.

핑거 충돌 OFF (Franka 핑거 mesh + PBD 봉투 마찰 압착은 불안정 가능).
봉투 파지는 fix_particles_to_link 로 처리 (Franka hand 링크에 weld).

시퀀스 (grasp_bottle.py 패턴 적용):
  1. pre-grasp: EE → (0.65, 0, 0.25), gripper open
  2. reach   : EE → (0.65, 0, 0.10), gripper open (봉투 입구 근처)
  3. grasp   : gripper position 0 (close) + fix_particles_to_link(mid-top strip)
  4. lift    : EE → (0.65, 0, 0.35), gripper force -20 N

카메라:
  · cam_front: 정면 고정뷰
  · cam_track: 봉투 com 추적뷰
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm

# stdout/stderr utf-8 (Windows cp949 깨짐 방지)
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 0.005           # PBD 안정성 위해 grasp_bottle.py 의 0.01 보다 작게
SUBSTEPS = 5

# ── 봉투 (cloth 패널) ────────────────────────────────────────────────────────
W, H, D = 0.05, 0.08, 0.006     # 50 × 80 × 6 mm (D 는 PBD 하한 6mm)
NW, NH, ND = 6, 9, 2
PARTICLE_SIZE = 2.83e-3
STRETCH_COMPLIANCE = 1e-3       # Samplebag 검증된 유연 강성
BENDING_COMPLIANCE = 1e-3

# ── 박스 (봉투 내용물) ──────────────────────────────────────────────────────
BOX_SIZE  = (0.025, 0.004, 0.025)
BOX_RHO   = 300.0

# ── 봉투 spawn (Franka 정면 reach 영역) ─────────────────────────────────────
BAG_XY = (0.65, 0.0)
BAG_MOUTH_Z = 0.12                  # 봉투 입구 z, 바닥 위 4cm
BAG_POS = (BAG_XY[0], BAG_XY[1], BAG_MOUTH_Z - H/2)
BOX_SPAWN = (BAG_XY[0], BAG_XY[1], BAG_MOUTH_Z + 0.05)

# 파지 strip (가로 중앙 + 상단) — 봉투 sag 고려해서 넉넉하게
GRIP_X_WIDTH    = 0.015      # 30mm strip (was 10mm)
GRIP_Z_FROM_TOP = 0.020      # 상단 20mm (was 5mm)

# ── 4-phase 타깃 z (Franka hand link 기준) ──────────────────────────────────
PRE_Z   = 0.30
REACH_Z = 0.13      # 봉투 입구 z=0.12 살짝 위
LIFT_Z  = 0.40

# ── 카메라 ──────────────────────────────────────────────────────────────────
CAM_RES          = (960, 720)
CAM_FRONT_POS    = (1.5, -0.8, 0.6)
CAM_FRONT_LOOK   = (0.5, 0.0, 0.2)
CAM_TRACK_OFFSET = np.array([0.4, -0.4, 0.2])
RENDER_EVERY     = 4

# ── 경로 ────────────────────────────────────────────────────────────────────
_DIR    = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS     = datetime.now().strftime("%Y%m%d_%H%M%S")
STL_PATH  = os.path.join(OUT_DIR, "rigidgrasp_bag.stl")
MP4_FRONT = os.path.join(OUT_DIR, f"RigidGrasp_FrankaBag_{_TS}_front.mp4")
MP4_TRACK = os.path.join(OUT_DIR, f"RigidGrasp_FrankaBag_{_TS}_track.mp4")


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


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _pos_of(b):
    x = _npy(b.get_particles_pos())
    return x[0] if x.ndim == 3 else x


def main():
    print("="*60)
    print(" RigidGrasp — Franka Panda + PBD 봉투 grasp")
    print("="*60)

    make_bag()
    print(f"[bag] STL exported → {STL_PATH}")

    import genesis as gs
    gs.init(backend=gs.cuda, logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        rigid_options=gs.options.RigidOptions(dt=DT),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=False,
    )

    # 바닥
    scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

    # PBD 봉투
    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=STL_PATH, scale=1.0, pos=BAG_POS, euler=(90, 0, 0)),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7,
                                      roughness=0.9, double_sided=True),
    )
    # 내용물 박스
    box_in_bag = scene.add_entity(
        material=gs.materials.Rigid(rho=BOX_RHO),
        morph=gs.morphs.Box(size=BOX_SIZE, pos=BOX_SPAWN, fixed=False),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )

    # Franka Panda (Genesis 공식 번들)
    franka = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))

    # 카메라 2개
    cam_front = scene.add_camera(res=CAM_RES, pos=CAM_FRONT_POS, lookat=CAM_FRONT_LOOK,
                                  fov=42, GUI=False)
    cam_track = scene.add_camera(
        res=CAM_RES,
        pos=tuple(np.array([BAG_XY[0], BAG_XY[1], BAG_MOUTH_Z]) + CAM_TRACK_OFFSET),
        lookat=(BAG_XY[0], BAG_XY[1], BAG_MOUTH_Z),
        fov=42, GUI=False,
    )

    scene.build(n_envs=0)

    # ── 봉투 corners 고정 (4 코너) ──
    pos0 = _pos_of(bag); z, x, y = pos0[:, 2], pos0[:, 0], pos0[:, 1]
    band = np.where(z >= np.quantile(z, 0.92))[0]
    corners = list({int(i) for i in [
        band[np.argmin(x[band] + y[band])], band[np.argmax(x[band] - y[band])],
        band[np.argmax(y[band] - x[band])], band[np.argmax(x[band] + y[band])]]})
    print(f"[bag] N={pos0.shape[0]}  corners={len(corners)}  "
          f"bbox X={x.min():.3f}~{x.max():.3f}  Y={y.min():.3f}~{y.max():.3f}  "
          f"Z={z.min():.3f}~{z.max():.3f}")
    bag.fix_particles(particles_idx_local=corners)

    # ── Franka 설정 (grasp_bottle.py 패턴 그대로) ──
    franka.set_qpos(np.array([1.56, -0.72, -0.02, -2.09, 0.04, 1.33, 2.4, 0.01, 0.01]))
    franka.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]))
    franka.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]))
    franka.set_dofs_force_range(
        np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
        np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
    )

    motors_dof  = np.arange(7)
    fingers_dof = np.arange(7, 9)
    end_effector = franka.get_link("hand")
    hand_link_idx = end_effector.idx

    # ── grip 파티클 선정 함수 (grasp 직전 호출) ──
    def _select_mid_top_grip(K=30):
        """가운데 x strip 안에서 z 가장 높은 K개 선정.
        4 fixed corners 가 봉투 좌우 끝(x 끝단) 에 있어 봉투 mouth 의 중앙 부위는
        reach 후 처짐. 따라서 z_top 절대값 비교 대신 x_strip 안의 z 상위 K개로
        '가운데 가장 위' 파티클 안정적으로 잡음."""
        cur = _pos_of(bag); cx, cz = cur[:, 0], cur[:, 2]
        x_center = float(BAG_XY[0])
        x_match  = np.abs(cx - x_center) < GRIP_X_WIDTH
        # corners 제외
        cand = np.array([i for i in np.where(x_match)[0] if i not in corners])
        if len(cand) == 0:
            print(f"   [grip-select] no candidate in x_strip")
            return np.array([], dtype=int)
        z_cand = cz[cand]
        K = min(K, len(cand))
        top_K_local = np.argpartition(z_cand, -K)[-K:]
        sel = cand[top_K_local]
        print(f"   [grip-select] x_strip=[{x_center-GRIP_X_WIDTH:.4f},{x_center+GRIP_X_WIDTH:.4f}] "
              f"N(x)={len(cand)}  → top-{K} z 선정")
        print(f"   [grip-select] sel z range: {cz[sel].min():.4f}~{cz[sel].max():.4f}")
        return sel

    def bag_com():
        p = _pos_of(bag); v = p[~np.isnan(p).any(axis=1)]
        return v.mean(axis=0)

    cam_front.start_recording()
    cam_track.start_recording()

    def update_track():
        bc = bag_com()
        cam_track.set_pose(pos=tuple(bc + CAM_TRACK_OFFSET), lookat=tuple(bc))

    def step_and_render(n, label=""):
        for i in range(n):
            scene.step()
            if (i + 1) % RENDER_EVERY == 0:
                update_track()
                cam_front.render(); cam_track.render()
        bc = bag_com(); ee = _npy(end_effector.get_pos())
        print(f"[phase] {label}  bag_com={np.round(bc,4)}  ee={np.round(ee,4)}")

    # ── 1. pre-grasp ──
    print("\n[phase] 1/4 pre-grasp")
    qpos = franka.inverse_kinematics(
        link=end_effector,
        pos=np.array([BAG_XY[0], BAG_XY[1], PRE_Z]),
        quat=np.array([0, 1, 0, 0]),
    )
    qpos[-2:] = 0.04
    path = franka.plan_path(qpos)
    for waypoint in path:
        franka.control_dofs_position(waypoint)
        scene.step()
    step_and_render(50, "pre-grasp (done)")

    # ── 2. reach (봉투 입구 위) ──
    print("[phase] 2/4 reach")
    qpos = franka.inverse_kinematics(
        link=end_effector,
        pos=np.array([BAG_XY[0], BAG_XY[1], REACH_Z]),
        quat=np.array([0, 1, 0, 0]),
    )
    franka.control_dofs_position(qpos[:-2], motors_dof)
    step_and_render(200, "reach (done)")

    # ── 3. grasp (close + fix_particles_to_link) ──
    print("[phase] 3/4 grasp")
    franka.control_dofs_position(qpos[:-2], motors_dof)
    franka.control_dofs_position(np.array([0.0, 0.0]), fingers_dof)
    # 봉투 corners release + grip 파티클 fix
    grip_idx = _select_mid_top_grip()
    if len(grip_idx) == 0:
        print("[grasp] ⚠ grip particles 0 → skip attach")
    else:
        bag.release_particle(particles_idx_local=corners)
        bag.fix_particles_to_link(link_idx=hand_link_idx, particles_idx_local=grip_idx)
        print(f"[grasp] release corners({len(corners)}), attach {len(grip_idx)} → hand")
    step_and_render(150, "grasp (done)")

    # ── 4. lift ──
    print("[phase] 4/4 lift (finger force)")
    qpos = franka.inverse_kinematics(
        link=end_effector,
        pos=np.array([BAG_XY[0], BAG_XY[1], LIFT_Z]),
        quat=np.array([0, 1, 0, 0]),
    )
    franka.control_dofs_position(qpos[:-2], motors_dof)
    franka.control_dofs_force(np.array([-20.0, -20.0]), fingers_dof)

    for i in range(800):
        scene.step()
        if (i + 1) % RENDER_EVERY == 0:
            update_track()
            cam_front.render(); cam_track.render()
        if (i + 1) % 100 == 0:
            bc = bag_com(); ee = _npy(end_effector.get_pos())
            print(f"  lift step {i+1}: bag_com_z={bc[2]:+.4f}  ee_z={ee[2]:+.4f}")

    cam_front.stop_recording(save_to_filename=MP4_FRONT, fps=30)
    cam_track.stop_recording(save_to_filename=MP4_TRACK, fps=30)
    print(f"\n[saved] front: {MP4_FRONT}")
    print(f"[saved] track: {MP4_TRACK}")

    bc_final = bag_com()
    print(f"\n[verify] bag_com final = {bc_final}")
    print(f"         lifted (z > 0.20)? = {bc_final[2] > 0.20}")
    print("완료.")


if __name__ == "__main__":
    main()
