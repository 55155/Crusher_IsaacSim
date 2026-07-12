"""
rg2_box_grasp_mujoco.py — RG2(v2, 안정화판) 단독 그리퍼가 실제 rigid box 를
쥐고 들어올리는 과정을 MuJoCo 네이티브로 시뮬레이션·녹화한다.

Genesis 대신 네이티브 MuJoCo 를 쓰는 이유: <equality><joint> mimic 을 Genesis
1.1.0 이 제대로 못 지킨다(PROJECT_NOTES §4.5). MuJoCo 는 정상 동작 + 방금
implicitfast/관성 수정으로 수치 안정성도 확보됐다.

구성:
  world -> [slide z, "lift_joint" 로 간이 1-DOF 리프트] -> gripper_bracket -> ...
  flex_finger 만 CoACD 볼록분해 hull(contype=1) 로 실제 컨택 활성화.
  나머지(moment_arm/truss_arm/finger_tip/body/bracket) 는 기존처럼 visual-only.
  박스: 작은 받침대 위에 놓인 free rigid box.

페이즈: open(대기) -> close(파지) -> lift(slide joint 상승) -> hold.
"""
import os
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\m0609_rg2_v2.xml"
OUT_XML = os.path.join(HERE, "_rg2_box_grasp.xml")
RESULT_DIR = os.path.join(HERE, "RESULT")
os.makedirs(RESULT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(RESULT_DIR, f"rg2_box_grasp_{_TS}.mp4")

COACD_DIR = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\rg2\reference_onrobot_ros\meshes\rg2_v1\coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]

BOX_SIZE = 0.04     # 40mm 정육면체
TABLE_TOP_Z = 0.05  # gripper_bracket(=world 고정) 로컬 기준, 아래에서 결정

# ── XML 조립 ──────────────────────────────────────────────────────────────
tree = ET.parse(SRC)
root = tree.getroot()
compiler = root.find("compiler")
compiler.set("meshdir", r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots")

asset = root.find("asset")
for i, hull_file in enumerate(FLEX_FINGER_HULLS):
    me = ET.SubElement(asset, "mesh")
    me.set("name", f"flex_finger_hull_{i:03d}")
    me.set("file", f"rg2/reference_onrobot_ros/meshes/rg2_v1/coacd/{hull_file}")

wb = root.find("worldbody")
gripper_bracket = None
for body in wb.iter("body"):
    if body.get("name") == "gripper_bracket":
        gripper_bracket = body
        break
assert gripper_bracket is not None

# flex_finger 에 CoACD hull collision geom 추가
for body in wb.iter("body"):
    if body.get("name") in ("f1_flex_finger", "f2_flex_finger"):
        for i in range(len(FLEX_FINGER_HULLS)):
            g = ET.SubElement(body, "geom")
            g.set("type", "mesh")
            g.set("mesh", f"flex_finger_hull_{i:03d}")
            g.set("contype", "1")
            g.set("conaffinity", "1")
            g.set("group", "0")
            g.set("friction", "1.2 0.02 0.001")
            g.set("solref", "0.004 1")

# gripper_bracket 을 world 대신 "lift_carriage"(slide z) 밑으로 이동
new_wb = ET.Element("worldbody")
new_wb.append(ET.fromstring('<light pos="0.3 -0.3 0.5" dir="-1 1 -1" diffuse="1 1 1"/>'))
new_wb.append(ET.fromstring(
    '<geom name="floor" type="plane" size="0.5 0.5 0.02" pos="0 0 -0.05" '
    'rgba="0.3 0.32 0.35 1" contype="1" conaffinity="1"/>'))

lift = ET.SubElement(new_wb, "body", name="lift_carriage", pos="0 0 0.35")
ET.SubElement(lift, "joint", name="lift_joint", type="slide", axis="0 0 1",
              range="0 0.30", damping="20")
ET.SubElement(lift, "geom", type="box", size="0.005 0.005 0.005",
              contype="0", conaffinity="0", rgba="1 0 0 0.3")
gripper_bracket.set("pos", "0 0 0")
lift.append(gripper_bracket)

# 받침대 + 박스: lift_carriage pos=(0,0,0.35) + gripper 체인 기준.
# f1/f2_pad FK 실측: q=0.02(열림) mid_z=0.5057, q=1.0(닫힘) mid_z=0.5415 —
# 핑거가 닫히며 39mm 위로 떠오른다(호 운동). 박스를 open 높이에 맞추면 close 때
# 박스 위로 지나가버림(실측 확인) → 열림/닫힘 중간 높이(z≈0.525)에 박스 중심을
# 둬서 닫히는 과정 중간에 확실히 스친다.
TABLE_TOP_Z = 0.504
table = ET.fromstring(
    f'<body name="table" pos="0 0 {TABLE_TOP_Z - 0.01}"> '
    '<geom type="box" size="0.06 0.06 0.01" contype="1" conaffinity="1" '
    'rgba="0.75 0.78 0.82 1"/></body>'
)
new_wb.append(table)

box = ET.fromstring(
    f'<body name="box" pos="0 0 {TABLE_TOP_Z + BOX_SIZE/2 + 0.001}"> '
    '<freejoint/> '
    f'<geom name="box_geom" type="box" size="{BOX_SIZE/2} {BOX_SIZE/2} {BOX_SIZE/2}" '
    'contype="1" conaffinity="1" rgba="0.85 0.35 0.25 1" '
    'friction="1.2 0.02 0.001" mass="0.05"/></body>'
)
new_wb.append(box)

root.remove(wb)
root.insert(list(root).index(asset) + 1, new_wb)

# actuator: 둘 다 position — gripper_joint 를 velocity 로 닫으면 ctrl=0 순간
# "각속도 0 유지"일 뿐 자세를 안 붙잡아서 lift 중 접촉반력에 스르륵 풀린다.
# position 이면 목표각을 계속 붙잡아 lift 중에도 grip force 유지.
act = root.find("actuator")
for a in list(act):
    act.remove(a)
ET.SubElement(act, "position", joint="gripper_joint", kp="15", kv="1.0",
              ctrlrange="0 1.3", forcerange="-3.0 3.0")
ET.SubElement(act, "position", joint="lift_joint", kp="300", ctrlrange="0 0.30")

tree.write(OUT_XML)
print(f"[grasp] wrote {OUT_XML}")

m = mujoco.MjModel.from_xml_path(OUT_XML)
d = mujoco.MjData(m)
print(f"[grasp] nq={m.nq} nu={m.nu} neq={m.neq}")

gj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
lj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
box_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")

d.qpos[m.jnt_qposadr[gj]] = 0.02
for jn in ("f1_truss_arm_joint", "f1_finger_tip_joint", "gripper_mirror_joint",
           "f2_truss_arm_joint", "f2_finger_tip_joint"):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    d.qpos[m.jnt_qposadr[jid]] = 0.0
mujoco.mj_forward(m, d)

r = mujoco.Renderer(m, height=480, width=640)
cam = mujoco.MjvCamera()
cam.lookat = [0, 0, 0.50]
cam.distance = 0.32
cam.azimuth = 60
cam.elevation = -15
frames = []


def _track():
    # 박스 위치를 계속 따라가며 lookat 갱신 (박스가 그리퍼에 들려 올라가도
    # 프레임 밖으로 안 나가게). z 만 부드럽게 추적, x/y 는 0 근방 고정.
    bz = float(d.xpos[box_body][2])
    cam.lookat[0] = 0.0
    cam.lookat[1] = 0.0
    cam.lookat[2] = bz


def _cap(every=3):
    _track()
    r.update_scene(d, camera=cam)
    frames.append(r.render())


def _shot(tag):
    _track()
    r.update_scene(d, camera=cam)
    Image.fromarray(r.render()).save(os.path.join(RESULT_DIR, f"rg2_boxgrasp_{tag}.png"))


def _box_z():
    return float(d.xpos[box_body][2])


Q_OPEN = 0.02
# gap≈37mm @ q=1.0, box_size=40mm 대비 약 3mm 압착 (q=1.05 는 16mm 압착으로 너무 셈)
Q_TARGET_CLOSE = 1.00
LIFT_TARGET = 0.20

d.ctrl[0] = Q_OPEN
d.ctrl[1] = 0.0

# 0) settle: 중력으로 박스가 테이블에 안착 (그리퍼는 open 유지)
for i in range(300):
    mujoco.mj_step(m, d)
    if i % 3 == 0:
        _cap()
_shot("settle")
print(f"[phase] settle  box_z={_box_z():.4f}")

# 1) close: position 목표를 서서히 Q_TARGET_CLOSE 로 램프 (position control 이라
#    ctrl 이 곧 유지할 각도 — 도달 후에도 그 각을 계속 붙잡아 grip force 유지)
N_CLOSE = 400
for i in range(N_CLOSE):
    s = (i + 1) / N_CLOSE
    d.ctrl[0] = Q_OPEN + (Q_TARGET_CLOSE - Q_OPEN) * s
    mujoco.mj_step(m, d)
    if i % 3 == 0:
        _cap()
for i in range(200):   # 닫힘 상태로 안정화 (target 유지)
    mujoco.mj_step(m, d)
    if i % 3 == 0:
        _cap()
_shot("closed")
bz_pre = _box_z()
print(f"[phase] close   q={d.qpos[m.jnt_qposadr[gj]]:.4f}  ctrl={d.ctrl[0]:.4f}  box_z={bz_pre:.4f}")

# 2) lift: gripper_joint 는 Q_TARGET_CLOSE 유지한 채 lift_joint 만 상승
N_LIFT = 500
for i in range(N_LIFT):
    s = (i + 1) / N_LIFT
    d.ctrl[1] = LIFT_TARGET * s
    mujoco.mj_step(m, d)
    if i % 3 == 0:
        _cap()
_shot("lift")
print(f"[phase] lift    lift_q={d.qpos[m.jnt_qposadr[lj]]:.4f}  box_z={_box_z():.4f}")

# 3) hold
for i in range(300):
    mujoco.mj_step(m, d)
    if i % 3 == 0:
        _cap()
_shot("hold")
bz_post = _box_z()
print(f"[phase] hold    box_z={bz_post:.4f}")

dz = bz_post - bz_pre
ok = dz > 0.05
print(f"\n[grasp-check] box Δz = {dz*1e3:+.1f} mm  -> "
      f"{'GRASP OK' if ok else 'GRASP FAIL'}")

imageio.mimsave(MP4, frames, fps=60, quality=8)
print(f"\n[saved video] {MP4}  ({len(frames)} frames)")
print(f"[saved photos] {RESULT_DIR}/rg2_boxgrasp_{{settle,closed,lift,hold}}.png")
