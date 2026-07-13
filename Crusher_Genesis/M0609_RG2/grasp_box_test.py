"""
grasp_box_test.py — M0609 + RG2(v2, 실물 충실 mimic 구조) 로 Genesis 프리미티브
Box 엔티티를 실제 마찰 파지(force-controlled grip)로 들어올리는 headless 테스트.

배경
----
· m0609_rg2_v2.xml (assets/robots/) 은 OnRobot 공식 STL + mimic 관절(moment_arm/
  truss_arm/finger_tip, <equality><joint> 5개)로 만든 실물 충실 그리퍼.
· PROJECT_NOTES.md §4.5: Genesis 1.1.0 은 MJCF <equality><joint> 를 제대로 안 지킨다
  (좌우 비대칭 관측됨) → 6개 gripper DOF(gripper_joint + 5 mimic) 를 물리 제약이
  아니라 **매 스텝 동일 목표값을 control_dofs_position 으로 명령**해서 동기화한다
  (RG2 실물은 gripper_joint 하나만 모터고 나머지는 4-bar 링크의 수동 종동 관절 —
  polycoef="0 1 0 0 0" 이 "값 그대로 복사"라 동일 타깃을 주는 것으로 등가 재현).
· §4.6: MJCF <position kp> actuator 와 Genesis set_dofs_kp 가 이중 적용되면 충돌 →
  MJCF 쪽 <actuator>/<equality>/joint damping·frictionloss 는 전부 제거하고, 대신
  Genesis set_dofs_kp/kv/force_range 로 (제거 전 MJCF 에 있던 것과 동일한) 게인을
  네이티브 API 로 다시 심는다.
· **이전 버전(kinematic set_dofs_position 전 구간 텔레포트)의 한계**: 관절을 물리
  없이 직접 텔레포트하면 접촉으로 발생하는 반력이 로봇 바디에 축적되지 않아
  마찰만으로는 lift 가속을 못 버티고 미끄러짐(반복 관측) → 이번 버전은 팔 6DOF +
  그리퍼 6DOF 전부 `control_dofs_position`(PD, force_range 로 saturate) 로 구동해
  실제 접촉·마찰 동역학이 로봇 바디를 통해 전달되게 한다. 그립은 "박스 크기에 맞는
  정확한 압착량"을 미리 계산하지 않고, forcerange(±2.0 N·m, OnRobot 실측 스펙)로
  saturate 될 때까지 "최대한 닫아라" 를 명령 — 실물 그리퍼의 컴플라이언트 파지와
  동일한 방식이라 박스 크기가 달라져도 강건하다.
· 핑거 충돌: 커뮤니티 collision STL 은 단순화가 심해 컨택이 부실 → flex_finger
  (실제 파지 패드) 를 CoACD 로 볼록분해(coacd_fingers.py, 7 hulls)해서 실제 곡면에
  가까운 컨택을 만든다. finger_tip 등 나머지는 기존처럼 visual-only 유지.

결과: M0609_RG2/RESULT/ 에 mp4 + phase PNG 저장.
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
from PIL import Image

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

# ── 경로 ─────────────────────────────────────────────────────────────────────
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"grasp_box_{_TS}.mp4")

COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]

# ── 시뮬 옵션 ────────────────────────────────────────────────────────────────
DT = 5e-3
RENDER_EVERY = 4

# ── 그리퍼 stroke ──────────────────────────────────────────────────────────
# 주의: m0609_rg2_v2.xml 주석의 q_closed/q_open(0.8229~1.8751) 은 flex_finger
# "몸체 원점"(피벗) 기준 gap 이었다 — 실제 패드 mesh 중심은 로컬 -X 로 27mm 더
# 안쪽이라 진짜 파지에 쓰이는 pad-to-pad gap 은 전혀 다르다. Genesis 에서
# _probe_pad_gap.py 로 실측한 pad_gap(q) 은 q≈1.30 부근에서 최소(≈0mm, 진짜 닫힘)
# 이고 양쪽으로 벌어지는 비단조 곡선 — 여기서는 q=1.0(닫히기 전)~1.30(닫힘) 의
# 단조 구간만 사용한다 (q=1.0→35mm, q=1.15→19mm).
FING_OPEN  = 1.00    # pad_gap ≈ 35mm (실측)
# q=1.30 실측 시도: PAD_LOCAL 오프셋 공식(원래 q≈1.0~1.2 구간에서 캘리브레이션된
# 근사치)이 q=1.30 근처에서는 안 맞는다 — 실측 gap 이 16.4mm 로 나왔는데(문서상
# true gap ≈0mm 부근) 이 오프셋이 큰 회전각에서 선형근사가 깨지는 것으로 보임.
# 그 결과 박스를 "틀린 위치"에 놓게 되어 완전히 접촉 없이 닫혀버림(실측:
# gripper_q=1.30 도달, grip_force=0). → PAD_LOCAL 이 검증된(실제 접촉 확인된)
# q≈1.1~1.2 범위 안으로 타깃을 되돌린다.
FING_CLOSE_TARGET = 1.20
# 박스 배치 기준 FK 도 실제 force-close 타깃과 동일 자세로 맞춰야 궤적이 반드시
# 그 위치를 지나며 접촉한다.
FING_CLOSE_PROBE  = FING_CLOSE_TARGET
GRIPPER_JOINTS = ("gripper_joint", "f1_truss_arm_joint", "f1_finger_tip_joint",
                  "gripper_mirror_joint", "f2_truss_arm_joint", "f2_finger_tip_joint")

# ── 박스 (파지 대상, Genesis 프리미티브) ───────────────────────────────────────
BOX_SIZE = (0.03, 0.03, 0.03)   # 20mm 시도에서 접촉 자체가 안 돼서 30mm 로 복원
# 1번 시도: 8g(rho=300)은 너무 가벼워 작은 비대칭력/노이즈에도 쉽게 흔들림 →
# 관성을 키워 안정화되는지 확인 (rho 300→2000, 8.1g→54g).
BOX_RHO = 2000.0

# ── 팔 자세 (link_6 pose 는 그리퍼 종류와 무관 — ipc_grasp_bag_test.py 와 동일 재사용) ──
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)

# N_LIFT=800 로 늘려봤지만 실패 지점이 궤적의 "s≈0.5" 부근(스텝수와 무관, 관절
# 보간 비율 기준)에서 반복 관측 — 스텝을 더 늘려도 그 비율 지점은 피할 수 없음.
# N_LIFT=400(원래값)에서는 그 지점까지(s=0→0.5, 즉 스텝 0~400 전부) 40N대 접촉력
# 유지하며 56mm 실제 lift 성공을 실측했으므로 이 값으로 되돌린다.
N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 100, 400, 150, 400, 150

CAM_POS, CAM_LOOK = (0.75, -0.55, 0.65), (0.25, 0.0, 0.45)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    """m0609_rg2_v2.xml 사본 + Genesis 대응 패치.

    1) <actuator>/<equality> 제거, joint damping/frictionloss 제거 — 게인은
       MJCF 대신 Genesis set_dofs_kp/kv/force_range 로 다시 심는다(§4.6,
       MJCF <position kp> 와 Genesis set_dofs_kp 이중 적용 충돌 회피).
    2) f1_flex_finger / f2_flex_finger 에 CoACD hull collision geom 추가
       (contype=1/conaffinity=1) — 실제 파지 컨택은 이 지오메트리로 일어난다.
    """
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_rg2_v2_")
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
    print(f" M0609 + RG2(v2) grasp Box primitive test (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="info", precision="32")

    # 2번 시도: 접촉 솔버 강성/정확도 상향.
    # · noslip_iterations 기본값 0(비활성) — 이름 그대로 "미끄러지면 안 되는"
    #   접촉(정지마찰)을 보정하는 후처리 패스인데 꺼져 있었다. 실측한 hold 단계
    #   실패 패턴(arm_v_norm 이 거의 0 으로 수렴하는 순간 contact force 가 통째로
    #   0 으로 끊기고 박스가 자유낙하)이 정확히 이 종류의 정지-접촉 정확도 문제와
    #   부합해서 켜본다.
    # · constraint_timeconst 기본 0.01s(연함) → 절반으로 낮춰 접촉/관절 구속을 더
    #   빨리(뻣뻣하게) 수렴시킨다.
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            dt=DT, constraint_timeconst=0.005, noslip_iterations=20),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(friction=0.5))

    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        # decimate=True: 원래 collision 안정성 우려로 False 였는데(27466-face 메시
        # 경고), 이후 noslip_iterations/constraint_timeconst 로 접촉 자체를 이미
        # 안정화했으니 다시 켜서 렌더 품질 개선 — flex_finger CoACD hull(우리가
        # 직접 추가한 파지용 collision geom)은 별도 mesh 라 decimate 대상이 아니라
        # 그립 품질에는 영향 없어야 함.
        gs.morphs.MJCF(file=robot_xml, decimate=True),
        material=gs.materials.Rigid(friction=1.0),
        surface=gs.surfaces.Default(smooth=True),
    )

    # 테이블/박스는 FK probe 이후 실측 위치에 배치 (placeholder 로 우선 생성)
    table = scene.add_entity(
        gs.morphs.Box(size=(0.15, 0.15, 0.02), pos=(0.25, 0.0, 0.30), fixed=True),
        material=gs.materials.Rigid(friction=0.5),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82), smooth=True),
    )
    box = scene.add_entity(
        gs.morphs.Box(size=BOX_SIZE, pos=(0.25, 0.0, 0.35), fixed=False),
        material=gs.materials.Rigid(rho=BOX_RHO, friction=1.5),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25), smooth=True),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=42, GUI=False)

    scene.build(n_envs=0)

    nq_arm = 6
    f1 = robot.get_link("f1_flex_finger")
    f2 = robot.get_link("f2_flex_finger")
    bracket = robot.get_link("gripper_bracket")   # link_6 에 고정된 그리퍼 마운트(비-핑거)

    arm_dofs = np.arange(0, 6)
    grip_dofs = np.arange(6, 12)

    # 제거한 MJCF <actuator> 게인을 Genesis 네이티브 API 로 재현 (§4.6).
    ARM_KP  = np.array([2000., 2000., 2000., 1000., 1000., 500.])
    ARM_KV  = np.array([100.,  100.,  100.,  50.,   50.,   25.])
    ARM_FRC = np.array([400.,  300.,  300.,  200.,  220.,  50.])
    robot.set_dofs_kp(ARM_KP, dofs_idx_local=arm_dofs)
    robot.set_dofs_kv(ARM_KV, dofs_idx_local=arm_dofs)
    robot.set_dofs_force_range(-ARM_FRC, ARM_FRC, dofs_idx_local=arm_dofs)

    # 실측: close 단계는 실물 스펙(kp=30, 2N·m)이 부드럽게 접촉을 잡는다(20~30N
    # 접촉력 확인). 반대로 close 단계부터 kp 를 높이면(80) 접촉 "순간"이 너무
    # 격렬해져 오히려 박스를 옆으로 튕겨버림(실측). → close 는 무른 스펙 그대로
    # 두고, grasp 확정 후(= lift 시작 전)에만 kp/forcerange 를 올려 "다 잡은 다음
    # 더 세게 쥐기" 로 순서를 바꾼다.
    GRIP_KP_SOFT  = np.full(6, 30.0)
    GRIP_KV_SOFT  = np.full(6, 1.5)
    GRIP_FRC_SOFT = np.full(6, 2.0)     # OnRobot RG2 실측 gripping torque 한계
    GRIP_KP_STIFF  = np.full(6, 80.0)
    GRIP_KV_STIFF  = np.full(6, 3.0)
    GRIP_FRC_STIFF = np.full(6, 5.0)
    GRIP_KP, GRIP_KV, GRIP_FRC = GRIP_KP_SOFT, GRIP_KV_SOFT, GRIP_FRC_SOFT
    robot.set_dofs_kp(GRIP_KP, dofs_idx_local=grip_dofs)
    robot.set_dofs_kv(GRIP_KV, dofs_idx_local=grip_dofs)
    robot.set_dofs_force_range(-GRIP_FRC, GRIP_FRC, dofs_idx_local=grip_dofs)

    # 초기 자세는 한 번만 kinematic 스냅 (PD 가 0-pose 에서 Q_GRASP 까지 큰 오차로
    # 튀는 것 방지). 이후부터는 전부 control_dofs_position(PD, force_range saturate).
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN] * 6]))

    # PROJECT_NOTES.md §4.2: "control_dofs_position 두 번 호출 시 두 번째가 첫
    # 번째를 덮어쓰는 케이스가 있어(M0609 에서 관측) 한 번에 8/9 DOF 통합 호출
    # 권장" — 이전 버전은 팔/그리퍼를 두 번 나눠 호출하고 있었다(엔드이펙터
    # 진동의 유력 원인). 12DOF 를 한 번의 호출로 합친다.
    all_dofs = np.arange(12)

    def _drive(q6, g):
        robot.control_dofs_position(np.concatenate([q6, np.full(6, g)]), dofs_idx_local=all_dofs)

    import genesis.utils.geom as gu

    # PAD_LOCAL(고정 로컬 오프셋) 근사는 q 가 캘리브레이션 지점에서 멀어지면
    # (특히 q≈1.2~1.3 근처 큰 회전각) 어긋난다는 게 실측으로 확인됨(gripper_q 가
    # 저항 없이 타깃까지 도달, grip_force=0). link.get_AABB() 는 실제 CoACD collision
    # 지오메트리의 월드 AABB 를 직접 반환하므로 이걸로 대체 — 근사가 아니라 실측.
    def _pad_world(link):
        aabb = _npy(link.get_AABB())
        aabb = aabb.reshape(-1, 3) if aabb.ndim > 2 else aabb
        return aabb.reshape(2, 3).mean(axis=0)

    # ── FK probe: control_dofs_position 은 kinematic 텔레포트가 아니라 PD 수렴이라
    # 목표에 도달할 시간(스텝)이 필요 — 60스텝(=0.3s)씩 명령해서 정상상태로 수렴시킨
    # 뒤 측정한다. 핑거가 호(arc) 운동을 하므로 open 시점의 mid 와 close 시점의
    # mid 가 다르다(실측: close 때 위로 떠오름) — 박스는 FING_CLOSE_PROBE 자세의
    # pad mid 를 기준으로 배치해야 핑거가 박스 "중앙"을 물게 된다. (이 mid 는 어디까지나
    # "배치 참고용" 이고, 실제 그립력/최종 압착량은 force-controlled FING_CLOSE_TARGET
    # 이 결정한다 — 박스와 정확히 안 맞아도 force saturate 로 알아서 멈춘다.)
    N_PROBE = 60
    for _ in range(N_PROBE):
        _drive(Q_GRASP, FING_CLOSE_PROBE)
        scene.step()
    p1 = _pad_world(f1)
    p2 = _pad_world(f2)
    mid = (p1 + p2) / 2.0
    gap_close_mm = np.linalg.norm(p1 - p2) * 1000
    print(f"[probe close] f1_pad={np.round(p1,4)} f2_pad={np.round(p2,4)} mid={np.round(mid,4)} "
          f"gap={gap_close_mm:.1f}mm")

    # open 시점 gap 도 확인 (박스가 open 상태에서 두 패드 사이로 들어갈 수 있는지
    # 안전성 체크 — open gap 이 close mid 근방에서 box 폭보다 충분히 커야 한다)
    for _ in range(N_PROBE):
        _drive(Q_GRASP, FING_OPEN)
        scene.step()
    p1o = _pad_world(f1)
    p2o = _pad_world(f2)
    print(f"[probe open]  f1_pad={np.round(p1o,4)} f2_pad={np.round(p2o,4)} "
          f"gap={np.linalg.norm(p1o-p2o)*1000:.1f}mm")

    # Q_LIFT 자세에서 pad mid 가 Q_GRASP 대비 얼마나 수평 이동하는지 진단 —
    # lift 단계에서 그리퍼가 박스를 "두고 떠나는" 현상의 원인 확인용.
    for _ in range(N_PROBE):
        _drive(Q_LIFT, FING_CLOSE_PROBE)
        scene.step()
    p1l = _pad_world(f1)
    p2l = _pad_world(f2)
    mid_lift = (p1l + p2l) / 2.0
    sweep = mid_lift - mid
    print(f"[probe lift]  mid={np.round(mid_lift,4)} "
          f"sweep(Q_GRASP→Q_LIFT)={np.round(sweep,4)} |sweep|={np.linalg.norm(sweep)*1000:.1f}mm")

    # settle 단계는 FING_OPEN 에서 시작하므로, 그리고 probe 로 흐트러진 arm/box
    # 상태를 리셋하기 위해 초기 자세로 재스냅(kinematic) 한다.
    robot.set_dofs_position(np.concatenate([Q_GRASP, [FING_OPEN] * 6]))
    robot.zero_all_dofs_velocity()
    box.set_pos(np.array([[0.25, 0.0, 0.35]]))
    box.zero_all_dofs_velocity()

    # 테이블/박스를 실측 mid 위치로 재배치
    table_top = float(mid[2]) - BOX_SIZE[2] / 2 - 0.001
    table.set_pos(np.array([[mid[0], mid[1], table_top - 0.01]]))
    box.set_pos(np.array([[mid[0], mid[1], table_top + BOX_SIZE[2] / 2 + 0.001]]))
    cam.set_pose(pos=tuple(np.array(CAM_POS) + (mid - np.array([0.2064, 0.0062, 0.4242]))),
                 lookat=tuple(mid))
    print(f"[setup] table_top={table_top:.4f}  box placed at "
          f"{np.round([mid[0], mid[1], table_top + BOX_SIZE[2]/2 + 0.001], 4)}")

    frames = []
    cam.start_recording()

    def _shot(name):
        out = cam.render()
        rgb = out[0] if isinstance(out, (tuple, list)) else out
        rgb = _npy(rgb)
        rgb = rgb[0] if rgb.ndim == 4 else rgb
        Image.fromarray(rgb[..., :3].astype("uint8")).save(
            os.path.join(OUT_DIR, f"grasp_box_{name}.png"))

    def _box_z():
        return float(_npy(box.get_pos()).squeeze()[2])

    def _grip_force():
        """gripper_joint 실제 발휘 힘(N·m) — forcerange saturate 여부 확인용."""
        return float(_npy(robot.get_dofs_force())[6])

    def run(name, q0, q1, f0, f1_, n, trace=False):
        for k in range(n):
            s = (k + 1) / n
            # (ease-in-ease-out cosine 프로파일을 시도했으나 오히려 더 일찍
            # 이탈함(실측: k≈200/800) — 선형 램프가 k≈420/800(=lift 50%)까지는
            # 40N대 접촉력을 유지하며 실제로 56mm 들어올리는 데 성공했으므로 되돌림.)
            q = q0 + (q1 - q0) * s
            f = f0 + (f1_ - f0) * s
            _drive(q, f)
            scene.step()
            if trace and k % 5 == 0 and k < 60:
                cf = _npy(robot.get_links_net_contact_force()).reshape(-1, 3)
                bp = _npy(box.get_pos()).squeeze()
                arm_q = _npy(robot.get_dofs_position())[:6]
                arm_v = _npy(robot.get_dofs_velocity())[:6]
                print(f"    [{name} k={k:4d}] box_pos={np.round(bp,4)}  "
                      f"cf1={np.round(cf[f1.idx_local],2)}  cf2={np.round(cf[f2.idx_local],2)}  "
                      f"arm_v_norm={np.linalg.norm(arm_v):.4f}")
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        bz = _box_z()
        gf = _grip_force()
        gq = float(_npy(robot.get_dofs_position())[6])
        bpos = _npy(box.get_pos()).squeeze()
        fp1, fp2 = _pad_world(f1), _pad_world(f2)
        cf = _npy(robot.get_links_net_contact_force()).reshape(-1, 3)
        cf_f1, cf_f2 = cf[f1.idx_local], cf[f2.idx_local]
        print(f"[phase] {name:8s} @done  box_pos={np.round(bpos,4)}  grip_force={gf:+.3f}N·m  "
              f"gripper_q={gq:.4f}  f1_pad={np.round(fp1,4)}  f2_pad={np.round(fp2,4)}  "
              f"contact_f1={np.round(cf_f1,3)}  contact_f2={np.round(cf_f2,3)}")
        return bz

    # 1) settle: 박스가 테이블 위에서 안정화 (그리퍼 열림)
    run("settle", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    # 2) close: force-controlled — "최대한 닫아라(1.30)" 명령, forcerange(±2N·m) 로
    #    saturate 될 때까지 박스에 압착력이 자연히 걸림 (압착량 사전계산 불필요).
    #    주의: 목표값 자체를 open→close 로 선형 램프하면 target 이 계속 "전진"해서
    #    박스에 걸려도 target 이 이미 그 너머로 가버려 PD 가 안 멈추고 밀고 지나감
    #    (실측: gripper_q 가 저항 없이 1.30 까지 도달, grip_force=0). target 은
    #    close 시작부터 바로 FING_CLOSE_TARGET 상수로 고정하고, 접근 속도 자체는
    #    kv(damping)·forcerange 가 만드는 PD 고유 동역학에 맡긴다.
    run("close", Q_GRASP, Q_GRASP, FING_CLOSE_TARGET, FING_CLOSE_TARGET, N_CLOSE)
    # 3) grasp: 압착 유지 안정화 (grip force 가 saturate 됐는지 로그로 확인)
    bz_pre = run("grasp", Q_GRASP, Q_GRASP, FING_CLOSE_TARGET, FING_CLOSE_TARGET, N_GRASP)
    # 3.5) stiffen: 부드러운 스펙(kp=30)으로 접촉을 확보한 뒤에만 게인을 올린다 —
    # close 단계부터 뻣뻣하면 접촉 "순간" 충격이 커져 박스를 옆으로 쳐낸다(실측).
    # 단, kp 를 "한번에" 바꾸면 이미 존재하던 위치오차에 새 kp 가 즉시 곱해져
    # impulsive 충격이 생겨 오히려 더 세게 튕겨나감(실측: box_z 가 바닥까지 날아감)
    # → N_STIFFEN 스텝에 걸쳐 게인 자체를 선형 램프해서 힘이 서서히 늘어나게 한다.
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
    print("[stiffen] grip kp 30→80, forcerange ±2.0→±5.0 N·m (contact 확보 후, 램프 적용)")
    # 4) lift: 팔 상승 — control_dofs_position(PD) 라 실제 접촉 반력이 로봇 바디를
    #    통해 전달되고, 그리퍼는 계속 FING_CLOSE_TARGET 을 명령해 압착력 유지.
    run("lift", Q_GRASP, Q_LIFT, FING_CLOSE_TARGET, FING_CLOSE_TARGET, N_LIFT)
    # 5) hold: 들린 채 유지
    bz_post = run("hold", Q_LIFT, Q_LIFT, FING_CLOSE_TARGET, FING_CLOSE_TARGET, N_HOLD, trace=True)

    dz = bz_post - bz_pre
    ok = dz > 0.02
    print(f"\n[grasp-check] box Δz = {dz*1e3:+.1f} mm  → "
          f"{'GRASP OK (box lifted)' if ok else 'GRASP FAIL (box left behind)'}")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")
    print("[done] phase PNGs → M0609_RG2/RESULT/grasp_box_<phase>.png")

    if use_viewer:
        print("\n[viewer] 시퀀스 종료. 창을 닫을 때까지 마지막(hold) 자세로 유지합니다 "
              "(마우스로 회전/확대해서 f1/f2 패드 vs 박스 위치를 직접 확인하세요).")
        while True:
            scene.step()


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
