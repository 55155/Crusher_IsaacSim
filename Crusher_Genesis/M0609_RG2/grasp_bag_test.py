"""
grasp_bag_test.py — M0609 + RG2(v2) 로 실측 STL 샘플백(PBD Cloth)의 옆면
실링부(side seal)를 마찰 파지로 들어올리는 테스트.

배경
----
· 대상 메시: assets/robots/Samplebag/Samplebag desigin.stl — mm 단위 실측 스캔.
  bbox: 로컬 X∈[-35,35](좌우 실링), Y∈[0,100](Y=0 바닥 실링 ~ Y=100 입구 오픈),
  Z∈[-10,10](두께). X=±35, Y=0 가장자리는 완전 핀치(실링) 선이고, X=±35 에서
  안쪽으로 ~10mm 는 두께가 0→풀두께로 램프되는 구간(="1cm 실링부").
· 1단계 정착 테스트(이 파일의 이전 버전) 결과: 내용물 없는 빈 봉투를 세워서
  드롭하면 자립 못하고 쓰러져 말림 → 세우지 않고 눕혀서(euler 없이) 스폰하면
  실링부는 평평, 오픈 입구만 살짝 말리는 안정적 형태로 정착함을 확인.
· 이번 단계: "초기에는 고정해놓았다가 로봇암이 잡으면 풀자"(사용자 지시) —
  bag.fix_particles 로 전체 파티클을 스폰 직후 월드에 고정해 깨끗한 형태를
  유지한 채 그리퍼가 접근·파지하게 하고, 접촉력이 실측 확인되면
  bag.release_particle 로 고정을 풀어 그 뒤로는 순수 마찰 파지로만 들어올린다
  (grasp_box_test.py 와 동일하게 fix_particles_to_link weld 는 쓰지 않는다 —
  마찰 파지 검증이 목적).
· 팔/그리퍼 제어 인프라는 grasp_box_test.py 에서 검증된 것을 그대로 재사용:
  control_dofs_position(PD) 전체 12DOF 통합 호출, kp/kv/force_range 게인,
  noslip_iterations=20 + constraint_timeconst=0.005(정지 시 접촉 소실 방지,
  box 실험에서 실측 확정), decimate=True + smooth=True.
· PBD 파라미터는 Samplebag.py 검증값: dt=1e-3, substeps=10,
  particle_size=2.83e-3, stretch/bending_compliance=1e-3,
  coupler_options=LegacyCouplerOptions(rigid_pbd=True).
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
from PIL import Image

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag desigin.stl")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"grasp_bag_{_TS}.mp4")

COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 1e-3
SUBSTEPS = 10
RENDER_EVERY = 10

# ── 그리퍼 stroke (grasp_box_test.py 와 동일) ──────────────────────────────
FING_OPEN = 1.00
FING_CLOSE_TARGET = 1.20

# ── 팔 자세 (grasp_box_test.py 와 동일 재사용 — link_6 pose 는 그리퍼/대상과 무관) ──
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)

# ── 봉투 (PBD Cloth, 실측 STL) ──────────────────────────────────────────────
BAG_SCALE = 0.001  # mm → m
# 1차 시도(euler=(0,0,0), 눕혀서 배치): 로컬 Z(두께)가 world Z(수직)로 매핑돼
# 그리퍼의 실제 pinch 축(world X, 팔이 벌리고 오므리는 방향)과 두께 방향이
# 어긋나 버림 — 손가락이 두께를 "집는"게 아니라 위에서 눌러버려 한쪽만 큰 힘
# (contact_f2 최대 85N, 대부분 Z축=누르는 방향)이 걸리고 반대쪽은 접촉 0
# (실측 확인). euler=(0,-90,0) 로 바꾸면 로컬 Z(두께,±10mm)→world X(pinch 축),
# 로컬 X(실링 위치, ±35mm)→world Z(높이), 로컬 Y(바닥실링~입구)→world Y 로 매핑
# 됨(scipy 오프라인 계산으로 검증) — 이러면 그리퍼가 좌우로 벌렸다 오므리는
# 동작이 정확히 봉투 두께를 집게 된다.
BAG_EULER = (0, -90, 0)
PARTICLE_SIZE = 2.83e-3
STRETCH_COMPLIANCE = 1e-3
BENDING_COMPLIANCE = 1e-3

# 이 스크립트와 동일한 robot_xml/게인/Q_GRASP/FING_CLOSE_TARGET 조합으로 별도
# 실측한 FK probe 결과(60스텝 PD 수렴 후 f1/f2 flex_finger AABB 중심의 중점):
# f1=[0.2097,0.0062,0.4307] f2=[0.1989,0.0062,0.4289] mid=[0.2043,0.0062,0.4298]
# gap=10.95mm.
GRIP_TARGET_WORLD = np.array([0.2043, 0.0062, 0.4298])
# 로컬 좌표계 기준 목표점 (euler=(0,-90,0) 적용 전):
#   local_Z = 0 (두께 중앙, → world X = GRIP_TARGET_WORLD[0])
#   local_Y = 30mm (평평한/안 말리는 구간 대표점, → world Y)
#   local_X = -30mm (좌측 실링 테이퍼 구간 중점, → world Z)
SEAL_LOCAL_X = -0.030
SEAL_LOCAL_Y = 0.030
BAG_POS = (
    GRIP_TARGET_WORLD[0],
    GRIP_TARGET_WORLD[1] - SEAL_LOCAL_Y,
    GRIP_TARGET_WORLD[2] - SEAL_LOCAL_X,
)

CAM_POS, CAM_LOOK = (0.75, -0.55, 0.65), (0.20, 0.02, 0.45)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _pos_of(entity):
    x = _npy(entity.get_particles_pos())
    return x[0] if x.ndim == 3 else x


def _prepare_robot_mjcf():
    """grasp_box_test.py::_prepare_robot_mjcf 와 동일 — CoACD 핑거 collision 추가,
    <actuator>/<equality>/damping/frictionloss 제거(게인은 Genesis 네이티브 API로)."""
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_rg2_v2_bag_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))

    dst = os.path.join(tmp_dir, "m0609_rg2_v2_patched.xml")
    tree = ET.parse(ROBOT_MJCF)
    root = tree.getroot()

    asset = root.find("asset")
    for i, hull_file in enumerate(FLEX_FINGER_HULLS):
        mesh_el = ET.SubElement(asset, "mesh")
        mesh_el.set("name", f"flex_finger_hull_{i:03d}")
        mesh_el.set("file", f"{COACD_DIR_REL}/{hull_file}")

    wb = root.find("worldbody")
    for j in wb.iter("joint"):
        j.attrib.pop("damping", None)
        j.attrib.pop("frictionloss", None)

    for body in wb.iter("body"):
        if body.get("name") in ("f1_flex_finger", "f2_flex_finger"):
            for i in range(len(FLEX_FINGER_HULLS)):
                g = ET.SubElement(body, "geom")
                g.set("type", "mesh")
                g.set("mesh", f"flex_finger_hull_{i:03d}")
                g.set("contype", "1")
                g.set("conaffinity", "1")
                g.set("group", "0")
                g.set("friction", "1.5 0.02 0.001")

    for tag in ("actuator", "equality"):
        el = root.find(tag)
        if el is not None:
            root.remove(el)

    tree.write(dst)
    return dst


def main(use_viewer: bool = True):
    print("=" * 60)
    print(f" M0609 + RG2(v2) grasp Bag(PBD) seal-edge test (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            dt=DT, constraint_timeconst=0.005, noslip_iterations=20),
        pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
        coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, decimate=True),
        material=gs.materials.Rigid(friction=1.0),
        surface=gs.surfaces.Default(smooth=True),
    )

    # 테이블 불필요: euler=(0,-90,0) 배치에서는 봉투가 GRIP_TARGET_WORLD 근방
    # 허공에 fix_particles 로 고정된 채 시작한다("초기엔 고정, 그리퍼가 잡으면
    # 풀자" — 사용자 지시). RigidGrasp_headless.py 의 봉투 고정 방식과 동일.

    bag = scene.add_entity(
        material=gs.materials.PBD.Cloth(
            stretch_compliance=STRETCH_COMPLIANCE,
            bending_compliance=BENDING_COMPLIANCE,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.7,
                                     roughness=0.9, double_sided=True, smooth=True),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=42, GUI=False)
    scene.build(n_envs=0)

    nq_arm = 6
    f1 = robot.get_link("f1_flex_finger")
    f2 = robot.get_link("f2_flex_finger")

    arm_dofs = np.arange(0, 6)
    grip_dofs = np.arange(6, 12)
    all_dofs = np.arange(12)

    ARM_KP = np.array([2000., 2000., 2000., 1000., 1000., 500.])
    ARM_KV = np.array([100., 100., 100., 50., 50., 25.])
    ARM_FRC = np.array([400., 300., 300., 200., 220., 50.])
    robot.set_dofs_kp(ARM_KP, dofs_idx_local=arm_dofs)
    robot.set_dofs_kv(ARM_KV, dofs_idx_local=arm_dofs)
    robot.set_dofs_force_range(-ARM_FRC, ARM_FRC, dofs_idx_local=arm_dofs)

    GRIP_KP_SOFT, GRIP_KV_SOFT, GRIP_FRC_SOFT = np.full(6, 30.0), np.full(6, 1.5), np.full(6, 2.0)
    GRIP_KP_STIFF, GRIP_KV_STIFF, GRIP_FRC_STIFF = np.full(6, 80.0), np.full(6, 3.0), np.full(6, 5.0)
    robot.set_dofs_kp(GRIP_KP_SOFT, dofs_idx_local=grip_dofs)
    robot.set_dofs_kv(GRIP_KV_SOFT, dofs_idx_local=grip_dofs)
    robot.set_dofs_force_range(-GRIP_FRC_SOFT, GRIP_FRC_SOFT, dofs_idx_local=grip_dofs)

    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN] * 6]))
    robot.zero_all_dofs_velocity()

    def _drive(q6, g):
        robot.control_dofs_position(np.concatenate([q6, np.full(6, g)]), dofs_idx_local=all_dofs)

    # ── 봉투 전체 파티클을 스폰 위치에 고정 (사용자 지시: "초기엔 고정, 그리퍼가
    # 잡으면 풀자") — 깨끗한 형태를 유지한 채 그리퍼가 접근하게 한다.
    pos0 = _pos_of(bag)
    n_particles = pos0.shape[0]
    all_particles = np.arange(n_particles)
    bag.fix_particles(particles_idx_local=all_particles)
    print(f"[bag] N={n_particles} particles, 전체 fix_particles 로 고정")
    print(f"[bag] bbox X=[{pos0[:,0].min():.4f},{pos0[:,0].max():.4f}] "
          f"Y=[{pos0[:,1].min():.4f},{pos0[:,1].max():.4f}] "
          f"Z=[{pos0[:,2].min():.4f},{pos0[:,2].max():.4f}]")

    # 좌측 실링부 파티클(=파지 타깃 근방) 식별 — 참고/추적용.
    # euler=(0,-90,0) 매핑: 로컬 X(실링 위치, ±35mm) → world Z. 좌측 실링은
    # local_X<0 즉 world_Z < BAG_POS[2] 부근.
    bag_z0 = float(BAG_POS[2])
    seal_idx = np.where(pos0[:, 2] - bag_z0 < -0.020)[0]
    print(f"[seal] 좌측 실링 후보(local_x<-20mm, world_z 기준): {len(seal_idx)} particles, "
          f"world_z range=[{pos0[seal_idx,2].min():.4f},{pos0[seal_idx,2].max():.4f}]"
          if len(seal_idx) else "[seal] 후보 없음 — BAG_POS/타깃 재확인 필요")

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(os.path.join(OUT_DIR, f"grasp_bag_{name}.png"))

    def _seal_center_z():
        p = _pos_of(bag)
        return float(p[seal_idx, 2].mean()) if len(seal_idx) else float("nan")

    cam.start_recording()

    def run(name, q0, q1, f0, f1_, n, trace=False):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1_ - f0) * s
            _drive(q, f)
            scene.step()
            if trace and k % 50 == 0:
                cf = _npy(robot.get_links_net_contact_force()).reshape(-1, 3)
                print(f"    [{name} k={k:4d}] seal_z={_seal_center_z():.4f}  "
                      f"cf1={np.round(cf[f1.idx_local],2)}  cf2={np.round(cf[f2.idx_local],2)}")
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        cf = _npy(robot.get_links_net_contact_force()).reshape(-1, 3)
        print(f"[phase] {name:8s} @done  seal_z={_seal_center_z():.4f}  "
              f"contact_f1={np.round(cf[f1.idx_local],3)}  contact_f2={np.round(cf[f2.idx_local],3)}")
        return cf[f1.idx_local], cf[f2.idx_local]

    # 1) approach: 팔은 이미 Q_GRASP, 그리퍼 open 유지 (봉투는 고정된 채)
    run("approach", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, 150)
    # 2) close: force-controlled, 소프트 스펙으로 부드럽게 접촉
    run("close", Q_GRASP, Q_GRASP, FING_CLOSE_TARGET, FING_CLOSE_TARGET, 400)
    cf1, cf2 = run("grasp", Q_GRASP, Q_GRASP, FING_CLOSE_TARGET, FING_CLOSE_TARGET, 150)

    contact_ok = np.linalg.norm(cf1) > 1.0 and np.linalg.norm(cf2) > 1.0
    print(f"\n[grasp-check] contact |cf1|={np.linalg.norm(cf1):.2f}N |cf2|={np.linalg.norm(cf2):.2f}N "
          f"→ {'접촉 확인, release 진행' if contact_ok else '접촉 부족 — release 보류'}")

    # 3) stiffen (grasp_box_test.py 와 동일 — 급격한 kp 변화는 impulsive 충격 유발하므로 램프)
    N_STIFFEN = 100
    for k in range(N_STIFFEN):
        s = (k + 1) / N_STIFFEN
        robot.set_dofs_kp(GRIP_KP_SOFT + (GRIP_KP_STIFF - GRIP_KP_SOFT) * s, dofs_idx_local=grip_dofs)
        robot.set_dofs_kv(GRIP_KV_SOFT + (GRIP_KV_STIFF - GRIP_KV_SOFT) * s, dofs_idx_local=grip_dofs)
        robot.set_dofs_force_range(
            -(GRIP_FRC_SOFT + (GRIP_FRC_STIFF - GRIP_FRC_SOFT) * s),
            GRIP_FRC_SOFT + (GRIP_FRC_STIFF - GRIP_FRC_SOFT) * s, dofs_idx_local=grip_dofs)
        _drive(Q_GRASP, FING_CLOSE_TARGET)
        scene.step()
        if k % RENDER_EVERY == 0:
            cam.render()
    print("[stiffen] grip kp 30→80, forcerange ±2.0→±5.0 N·m")

    # 4) release: 고정 해제 — 이제부터 순수 마찰 파지만으로 지지된다.
    if contact_ok:
        bag.release_particle(particles_idx_local=all_particles)
        print("[release] 전체 fix 해제 — 마찰 파지만으로 지지")
    else:
        print("[release] 접촉 부족으로 release 생략(그대로 고정 유지) — 파라미터 조정 필요")

    run("settle2", Q_GRASP, Q_GRASP, FING_CLOSE_TARGET, FING_CLOSE_TARGET, 150, trace=True)

    seal_z_pre = _seal_center_z()
    # 5) lift
    run("lift", Q_GRASP, Q_LIFT, FING_CLOSE_TARGET, FING_CLOSE_TARGET, 400, trace=True)
    seal_z_post = run("hold", Q_LIFT, Q_LIFT, FING_CLOSE_TARGET, FING_CLOSE_TARGET, 150, trace=True)
    seal_z_post = _seal_center_z()

    dz = seal_z_post - seal_z_pre
    print(f"\n[lift-check] seal_z Δ = {dz*1e3:+.1f} mm → "
          f"{'LIFT OK' if dz > 0.02 else 'LIFT FAIL (봉투 남겨짐/낙하)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → M0609_RG2/RESULT/grasp_bag_<phase>.png")

    if use_viewer:
        print("\n[viewer] 시퀀스 종료. 창을 닫을 때까지 유지합니다.")
        while True:
            scene.step()


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
