"""
grasp_box_test.py — M0609 + RG2(v2, 실물 충실 mimic 구조) 로 Genesis 프리미티브
Box 엔티티를 파지하는 headless 테스트.

배경
----
· m0609_rg2_v2.xml (assets/robots/) 은 OnRobot 공식 STL + mimic 관절(moment_arm/
  truss_arm/finger_tip, <equality><joint> 5개)로 만든 실물 충실 그리퍼 (별도 후보
  파일, 아직 프로덕션 미연결).
· PROJECT_NOTES.md §4.5: Genesis 1.1.0 은 MJCF <equality><joint> 를 제대로 안 지킨다
  (좌우 비대칭 관측됨) → 6개 gripper DOF(gripper_joint + 5 mimic) 를 물리 제약이
  아니라 **매 스텝 전부 동일 값으로 set_dofs_position** 해서 동기화한다.
· §4.6: MJCF <position kp> actuator 와 Genesis set_dofs_kp 가 충돌 → 이 스크립트도
  기존 관례대로 <actuator>/<equality>/joint damping·frictionloss 를 전부 제거하고
  set_dofs_position 순수 기구학 구동만 쓴다 (ipc_grasp_bag_test.py 와 동일 패턴).
· 핑거 충돌: 커뮤니티 collision STL 은 단순화가 심해 컨택이 부실 → flex_finger
  (실제 파지 패드) 를 CoACD 로 볼록분해(coacd_fingers.py, 7 hulls)해서 실제 곡면에
  가까운 컨택을 만든다. finger_tip 등 나머지는 기존처럼 visual-only 유지
  (PROJECT_NOTES §4.5: OnRobot 핑거 mesh 가 복잡해 통짜 변환 시 폭주 위험 — 이미
  finger_tip 은 CoACD 로 17-hull 까지 쪼개져 불안정 위험이 있어 배제).

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
FING_CLOSE = 1.20    # pad_gap ≈ 14mm (실측, BOX_SIZE=30mm 대비 약 16mm 압착 — 확실한 그립력 확보)
GRIPPER_JOINTS = ("gripper_joint", "f1_truss_arm_joint", "f1_finger_tip_joint",
                  "gripper_mirror_joint", "f2_truss_arm_joint", "f2_finger_tip_joint")

# ── 박스 (파지 대상, Genesis 프리미티브) ───────────────────────────────────────
BOX_SIZE = (0.03, 0.03, 0.03)
BOX_RHO = 300.0

# ── 팔 자세 (link_6 pose 는 그리퍼 종류와 무관 — ipc_grasp_bag_test.py 와 동일 재사용) ──
Q_GRASP = np.array([0, -0.40, 1.30, 0, 2.00, 0], float)
Q_LIFT  = np.array([0, -0.11, 0.60, 0, 2.41, 0], float)

N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 100, 200, 80, 200, 100

CAM_POS, CAM_LOOK = (0.75, -0.55, 0.65), (0.25, 0.0, 0.45)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def _prepare_robot_mjcf():
    """m0609_rg2_v2.xml 사본 + Genesis 대응 패치.

    1) <actuator>/<equality> 제거, joint damping/frictionloss 제거
       (§4.6 — Genesis set_dofs_position 순수 기구학 구동과 충돌).
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
                g.set("friction", "1.0 0.02 0.001")

    for tag in ("actuator", "equality"):
        el = root.find(tag)
        if el is not None:
            root.remove(el)

    tree.write(dst)
    return dst


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" M0609 + RG2(v2) grasp Box primitive test (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="info", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(friction=0.5))

    robot_xml = _prepare_robot_mjcf()
    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, decimate=False),
        material=gs.materials.Rigid(friction=1.0),
    )

    # 테이블/박스는 FK probe 이후 실측 위치에 배치 (placeholder 로 우선 생성)
    table = scene.add_entity(
        gs.morphs.Box(size=(0.15, 0.15, 0.02), pos=(0.25, 0.0, 0.30), fixed=True),
        material=gs.materials.Rigid(friction=0.5),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )
    box = scene.add_entity(
        gs.morphs.Box(size=BOX_SIZE, pos=(0.25, 0.0, 0.35), fixed=False),
        material=gs.materials.Rigid(rho=BOX_RHO, friction=1.0),
        surface=gs.surfaces.Default(color=(0.85, 0.35, 0.25)),
    )

    cam = scene.add_camera(res=(960, 720), pos=CAM_POS, lookat=CAM_LOOK, fov=42, GUI=False)

    scene.build(n_envs=0)

    nq_arm = 6
    f1 = robot.get_link("f1_flex_finger")
    f2 = robot.get_link("f2_flex_finger")

    def _set(q6, g):
        robot.set_dofs_position(np.concatenate([q6, [g] * 6]))

    # flex_finger.stl bbox 중심 (실측, stl_bbox 로 확인) — body 원점(피벗)이 아니라
    # 이 오프셋만큼 로컬 -X 로 떨어진 곳이 실제 패드 접촉면 중심이다.
    PAD_LOCAL = np.array([-0.02684, 0.0, 0.00425])

    import genesis.utils.geom as gu

    def _pad_world(link):
        p = _npy(link.get_pos()).squeeze()
        q = _npy(link.get_quat()).squeeze()
        T = np.asarray(gu.trans_quat_to_T(p, q))
        return T[:3, :3] @ PAD_LOCAL + T[:3, 3]

    # ── FK probe: Q_GRASP + FING_OPEN 에서 실제 fingertip 패드 중점 실측 ──
    _set(Q_GRASP, FING_OPEN)
    for _ in range(3):
        scene.step()
        _set(Q_GRASP, FING_OPEN)
    p1 = _pad_world(f1)
    p2 = _pad_world(f2)
    mid = (p1 + p2) / 2.0
    print(f"[probe] f1_pad={np.round(p1,4)} f2_pad={np.round(p2,4)} mid={np.round(mid,4)} "
          f"gap={np.linalg.norm(p1-p2)*1000:.1f}mm")

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

    def run(name, q0, q1, f0, f1_, n):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            f = f0 + (f1_ - f0) * s
            _set(q, f)
            scene.step()
            if k % RENDER_EVERY == 0:
                cam.render()
        _shot(name)
        bz = _box_z()
        print(f"[phase] {name:8s} @done  box_z={bz:.4f}")
        return bz

    # 1) settle: 박스가 테이블 위에서 안정화 (그리퍼 열림)
    run("settle", Q_GRASP, Q_GRASP, FING_OPEN, FING_OPEN, N_SETTLE)
    # 2) close: 핑거를 박스 압착 gap 까지 닫기
    run("close", Q_GRASP, Q_GRASP, FING_OPEN, FING_CLOSE, N_CLOSE)
    # 3) grasp: 압착 유지 안정화
    bz_pre = run("grasp", Q_GRASP, Q_GRASP, FING_CLOSE, FING_CLOSE, N_GRASP)
    # 4) lift: 팔 상승
    run("lift", Q_GRASP, Q_LIFT, FING_CLOSE, FING_CLOSE, N_LIFT)
    # 5) hold: 들린 채 유지
    bz_post = run("hold", Q_LIFT, Q_LIFT, FING_CLOSE, FING_CLOSE, N_HOLD)

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
