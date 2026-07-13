"""
grasp_bag_ipc_test.py — M0609+RG2 로 실측 STL 샘플백(FEM Cloth + IPC 커플러)의
좌측 실링부(side seal)를 마찰 파지로 들어올리는 테스트.

배경
----
· 이 프로젝트에는 이미 검증된 성공 사례가 있다: FEM/ipc_grasp_bag_test.py
  (커밋 ccaa0a7 "M0609+RG2 IPC 마찰 파지 성공", 봉투 Δz +125.8mm, weld 없이
  순수 contact+friction). 이 스크립트는 그 패턴을 실측 STL(Samplebag
  desigin.stl)의 옆면 실링부 타깃에 맞게 이식한 것이다.
· PBD.Cloth 로는(grasp_bag_test.py) 핑거 mesh 접촉 시 즉시 폭발했다 — IPC
  커플러는 PBD 를 아예 못 보고(fem_solver+rigid_solver 만 등록), FEM.Cloth 를
  써야 한다. IPC 는 barrier-based 접촉이라 침투 자체가 애초에 안 일어난다.
· 그리퍼는 m0609_rg2_v2.xml(6DOF mimic, box 실험에서 검증한 실물 충실 RG2) 사용
  — 처음엔 IPC 성공 사례가 검증한 단순 m0609_rg2.xml(2DOF)로 먼저 성공시켰지만,
  box 파지에서 계속 다듬어온 조합과 통일하기 위해 v2로 교체. gripper_joint +
  5개 mimic 관절을 grasp_box_test.py 와 동일하게 **매 스텝 전부 동일 값**으로
  묶어서 구동(<equality><joint> 를 Genesis 가 못 지키는 문제 우회). collision
  geom 은 CoACD 볼록분해한 f1/f2_flex_finger(box 실험과 동일 패치) — coup_links
  도 이 두 링크로 지정. IPC 세계에선 control_dofs_position(PD) 대신
  set_dofs_position(텔레포트)를 쓴다 — two_way_soft_constraint 가 자체
  soft-constraint 로 반력을 처리하므로 PD 불필요(오히려 actuator 충돌 유발).
· 봉투 로컬 축: X∈[-35,35]mm(실링부, 어느 쪽을 잡을지), Y∈[0,100]mm(바닥실링→
  입구), Z∈[-10,10]mm(두께). euler=(90,0,0) 적용 시 로컬Y→world Z(높이),
  로컬Z(두께)→world Y(그리퍼 닫힘축), 로컬X(실링 위치)→world X 그대로 —
  ipc_grasp_bag_test.py 의 procedural 봉투와 동일한 매핑(스크립트 내
  실측 확인).
· FINGER_MID=(0.2043,0.0062,0.4298) 는 grasp_box_test.py 에서 Q_GRASP+
  FING_CLOSE=1.20 로 FK_PROBE 실측된 f1/f2_flex_finger pad-mid 값 재사용
  (팔 관절은 그리퍼 종류와 무관, v2 그리퍼 자체 pad 위치 기준).
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

ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")  # 6DOF mimic (box 실험과 동일)
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
# 실측 STL(Samplebag desigin.stl)은 실링부(X=±35mm)·바닥실링(Y=0)이 두께 0으로
# 완전히 핀치된 선이라 IPC self-intersection check 에서 계속 실패 — 클리핑/캡
# 유무/데시메이션을 다 시도해도 동일하게 실패했다. → 검증된 IPC 성공 사례
# (ipc_grasp_bag_test.py)의 make_bag() 5-panel 위상(front+back+3 edge, top 만
# open)을 우리 치수로 재현한 프록시로 대체. **진짜 원인**은 그 다음에 로그
# 레벨을 올려서 찾음: "Geometry(0) in cloth_0_0 is too close (distance=1.2mm,
# thickness=2mm) to Geometry(0) in cloth_0_0(0)" — front/back 패널 자기충돌.
# 처음 D(패널 간격)=4mm 로 줄였다가 실패 → 레퍼런스와 동일한 D=6mm 로 복원하니
# 해결(격리 테스트로 bag+ground → +shelf → +robot 단계적으로 확인). 위 XML/캡
# 관련 원인 추정은 틀렸었다 — 실제로는 단순 두께 부족.
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"grasp_bag_ipc_{_TS}.mp4")

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
IPC_D_HAT = 1.0e-3          # 실링부 최소 두께(taper 근방 수mm)보다 작게
RENDER_EVERY = 4

# ── FEM cloth 강성 (ipc_grasp_bag_test.py 검증값) ──────────────────────────
CLOTH_E, CLOTH_NU, CLOTH_RHO = 1.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 50.0
CLOTH_FRICTION = 0.8

FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
# 마지막 관절(joint_6, 손목 롤)에 +90° — v2 그리퍼의 실제 닫힘축은(box 실험 기준)
# world X 인데, 봉투는 BAG_EULER=(90,0,0) 로 두께를 world Y 에 맞춰놔서 축이
# 어긋나 있었다(핑거가 두께를 안 집고 옆으로 스쳐 지나가듯 보인 원인). 손목을
# 90도 돌려 닫힘축을 X→Y 로 바꿔 봉투 두께축과 맞춘다(스크립트 내 격리 프로브로
# 실측 확인: 회전 후 f1/f2 는 Y 방향으로만 벌어짐).
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, np.pi / 2], float)
Q_LIFT = np.array([0, -0.11, 0.60, 0, 2.41, np.pi / 2], float)
# grasp_box_test.py 에서 검증된 v2 그리퍼 stroke — q=1.00(open, gap≈35mm) ~
# q=1.20(close, gap≈11~19mm)의 단조 구간. gripper_joint + 5 mimic 전부 이 값을
# 그대로 받는다.
FING_OPEN, FING_CLOSE = 1.00, 1.20
# 90도 회전 후 재측정한 f1/f2_flex_finger AABB 중점의 중점(팔 관절 자체는
# 손목 롤과 거의 무관해 이전 값과 큰 차이 없음).
FINGER_MID = np.array([0.20365, 0.00618, 0.43297])

# ── 봉투(프록시) 배치 ────────────────────────────────────────────────────────
# Samplebag_seal_pouch3.stl 는 make_bag() 과 동일 네이티브 좌표계(로컬 x=폭,
# y=높이, z=두께)로 생성 후 **bounding_box.centroid 기준 3축 전체 중심맞춤**
# (레퍼런스와 동일 관례) → 로컬 원점이 곧 패널 중심. euler=(90,0,0) 로
# 로컬y(높이)→world Z, 로컬z(두께)→world Y(그리퍼 닫힘축), 로컬x(폭/실링 위치)
# →world X 매핑(스크립트 내 격리 테스트로 실측 확인).
BAG_SCALE = 1.0
BAG_EULER = (90, 0, 0)
# 실측: -20mm 는 패널 폭(±32mm) 중심에서 20mm, 즉 진짜 가장자리(-32mm)에서
# 12mm나 안쪽이라 "실링 가장자리"보다는 패널 중앙에 가까웠다 — 가장자리에
# 훨씬 더 가깝게(±32mm 경계에서 4mm 안쪽, 좌/우 edge 캡 파티클이 확실히 잡히는
# 지점) 당긴다.
SEAL_LOCAL_X = -0.028
# 중심맞춤 후엔 local_y=0(중간 높이), local_z=0(두께 중심) 이 바로 FINGER_MID 에
# 대응 — 레퍼런스의 BAG_POS=tuple(FINGER_MID) 와 동일한 단순한 관계.
BAG_POS = (
    FINGER_MID[0] - SEAL_LOCAL_X,
    FINGER_MID[1],
    FINGER_MID[2],
)

# 봉투 받침 선반: 패널 바닥(로컬 y=-H/2=-45mm, world Z=BAG_POS[2]-0.045) 아래
# 1.5mm 갭 (레퍼런스와 동일).
SHELF_TOP = BAG_POS[2] - 0.045 - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 120, 80, 40, 200, 100

CAM_POS, CAM_LOOK = (0.75, -0.55, 0.65), tuple(FINGER_MID)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    """m0609_rg2_v2.xml 사본 + Genesis/IPC 대응 패치 (grasp_box_test.py 와 동일).

    1) <actuator>/<equality> 제거, joint damping/frictionloss 제거.
    2) f1_flex_finger/f2_flex_finger 에 CoACD hull collision geom 추가
       (contype=1/conaffinity=1) — coup_links 로 지정할 실제 접촉 지오메트리.
    """
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_ipc_bag_v2_")
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


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" IPC grasp bag test — M0609+RG2 + FEM cloth, real STL seal grip (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="info", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))

    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=FINGER_LINKS,
            coup_friction=CLOTH_FRICTION,
        ),
    )

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        surface=gs.surfaces.Default(color=(0.97, 0.97, 0.95), opacity=0.75,
                                     roughness=0.9, double_sided=True),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=42, GUI=False)
    scene.build(n_envs=0)

    q_grasp, q_lift = Q_GRASP, Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
    left_link = robot.get_link(FINGER_LINKS[0])

    vp = _npy(bag.get_state().pos).squeeze()
    print(f"[bag] built verts={vp.shape}  x={vp[:,0].min():.4f}~{vp[:,0].max():.4f}  "
          f"y={vp[:,1].min():.4f}~{vp[:,1].max():.4f}  z={vp[:,2].min():.4f}~{vp[:,2].max():.4f}")
    d_to_mid = np.linalg.norm(vp - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag] grip_strip verts near FINGER_MID: {len(grip_idx)}")
    if len(grip_idx) == 0:
        print("[bag] 경고: grip_strip 비어있음 — BAG_POS/FINGER_MID 재확인 필요")

    frames = []
    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(os.path.join(OUT_DIR, f"grasp_bag_ipc_{name}.png"))

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def run(name, q0, q1, f0, f1, n):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            scene.step()
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        vpn = _npy(bag.get_state().pos).squeeze()
        gz = vpn[grip_idx, 2].mean() if len(grip_idx) else float("nan")
        print(f"[phase] {name:8s} @done  bag_com=({vpn[:,0].mean():.3f},{vpn[:,1].mean():.3f},"
              f"{vpn[:,2].mean():.3f})  grip_com_z={gz:.4f}  finger_z={_finger_z():.4f}")
        return vpn

    run("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE)
    run("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE)
    vp_pre = run("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP)
    fz_pre = _finger_z()
    run("lift", q_grasp, q_lift, FING_CLOSE, FING_CLOSE, N_LIFT)
    vp_post = run("hold", q_lift, q_lift, FING_CLOSE, FING_CLOSE, N_HOLD)
    fz_post = _finger_z()

    dz_f = fz_post - fz_pre
    dz_b = float(vp_post[grip_idx, 2].mean() - vp_pre[grip_idx, 2].mean()) if len(grip_idx) else float("nan")
    slip = dz_f - dz_b
    ok = len(grip_idx) > 0 and dz_f > 0.02 and dz_b > 0.5 * dz_f
    print(f"\n[grasp-check] finger Δz = {dz_f*1e3:+.1f} mm,  bag grip Δz = {dz_b*1e3:+.1f} mm,"
          f"  slip = {slip*1e3:+.1f} mm")
    print(f"[grasp-check] → {'GRASP OK (bag follows gripper)' if ok else 'GRASP FAIL (bag left behind / slipped)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → M0609_RG2/RESULT/grasp_bag_ipc_<phase>.png")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
