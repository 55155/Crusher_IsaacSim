"""
rg2_velocity_check.py — RG2(v2) 단독 그리퍼를 velocity actuator 로 구동해서
(a) mj_step 를 제대로 밟았을 때 mimic 관절이 실제로 따라오는지,
(b) pad_gap(q) 이 position-control 스윕과 같은 궤적을 그리는지 확인한다.

이전 진단(rg2_only_record.py 로그, q1.30 렌더)에서 이미 position-control +
mj_step 조합으로는 열림/닫힘이 시각적으로 정상 확인됐다. 여기서는 velocity
actuator 로 같은 걸 재현해 두 제어방식 결과가 일치하는지 교차검증한다.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\m0609_rg2_v2.xml"
OUT_XML = os.path.join(HERE, "_rg2_only_vel.xml")
RESULT_DIR = os.path.join(HERE, "RESULT")
os.makedirs(RESULT_DIR, exist_ok=True)

tree = ET.parse(SRC)
root = tree.getroot()
compiler = root.find("compiler")
compiler.set("meshdir", r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots")
wb = root.find("worldbody")

gripper_bracket = None
for body in wb.iter("body"):
    if body.get("name") == "gripper_bracket":
        gripper_bracket = body
        break
assert gripper_bracket is not None

new_wb = ET.Element("worldbody")
new_wb.append(gripper_bracket)
root.remove(wb)
root.insert(list(root).index(root.find("asset")) + 1, new_wb)

# 기존 position actuator 제거하고 velocity actuator 로 교체
act = root.find("actuator")
for a in list(act):
    act.remove(a)
vel = ET.SubElement(act, "velocity")
vel.set("joint", "gripper_joint")
vel.set("kv", "5.0")
vel.set("ctrlrange", "-3.0 3.0")

tree.write(OUT_XML)
print(f"[vel] wrote {OUT_XML}")

m = mujoco.MjModel.from_xml_path(OUT_XML)
d = mujoco.MjData(m)
print(f"[vel] nq={m.nq} nu={m.nu} neq={m.neq}  actuator0_trntype={m.actuator_trntype[0]}")

gj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
PAD_LOCAL = np.array([-0.02684, 0.0, 0.00425])
f1b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "f1_flex_finger")
f2b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "f2_flex_finger")


def pad_gap():
    R1 = d.xmat[f1b].reshape(3, 3); p1 = R1 @ PAD_LOCAL + d.xpos[f1b]
    R2 = d.xmat[f2b].reshape(3, 3); p2 = R2 @ PAD_LOCAL + d.xpos[f2b]
    return float(np.linalg.norm(p1 - p2) * 1000)


mimic_names = ["f1_truss_arm_joint", "f1_finger_tip_joint", "gripper_mirror_joint",
               "f2_truss_arm_joint", "f2_finger_tip_joint"]
mimic_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in mimic_names]

# 시작: qpos 를 명시적으로 q=0 부근에 정렬 (전 관절 동일값, mimic 이 아직 안 풀린
# 초기 상태를 피하기 위해 6개 qpos 모두 직접 세팅) 후 velocity 로 q=0 -> 1.3 구동
d.qpos[m.jnt_qposadr[gj]] = 0.0
for jid in mimic_ids:
    d.qpos[m.jnt_qposadr[jid]] = 0.0
mujoco.mj_forward(m, d)
print(f"[vel] t=0.000  q={d.qpos[m.jnt_qposadr[gj]]:.4f}  pad_gap={pad_gap():7.2f}mm  "
      f"mimic={[round(float(d.qpos[m.jnt_qposadr[i]]),4) for i in mimic_ids]}")

DT = m.opt.timestep
d.ctrl[0] = 1.0   # rad/s, 양의 각속도로 q=0 -> 1.3 쪽으로 구동
t = 0.0
next_print = 0.0
while d.qpos[m.jnt_qposadr[gj]] < 1.3 and t < 5.0:
    mujoco.mj_step(m, d)
    t += DT
    if t >= next_print:
        q = d.qpos[m.jnt_qposadr[gj]]
        mv = [round(float(d.qpos[m.jnt_qposadr[i]]), 4) for i in mimic_ids]
        print(f"[vel] t={t:.3f}  q={q:.4f}  pad_gap={pad_gap():7.2f}mm  mimic={mv}")
        next_print += 0.2
d.ctrl[0] = 0.0
mujoco.mj_step(m, d)
q_final = d.qpos[m.jnt_qposadr[gj]]
print(f"\n[vel] stopped at t={t:.3f}s  q_final={q_final:.4f}  pad_gap={pad_gap():.2f}mm")

r = mujoco.Renderer(m, height=480, width=640)
cam = mujoco.MjvCamera(); cam.lookat = [0, 0, 0.15]; cam.distance = 0.30
cam.azimuth = 60; cam.elevation = -15
r.update_scene(d, camera=cam)
Image.fromarray(r.render()).save(os.path.join(RESULT_DIR, "rg2_velocity_final.png"))
print(f"[saved] {RESULT_DIR}/rg2_velocity_final.png")
