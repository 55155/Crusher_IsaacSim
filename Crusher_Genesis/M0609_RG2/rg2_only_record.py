"""
rg2_only_record.py — RG2(v2) 그리퍼만 단독(월드 고정)으로 열림<->닫힘 구동시켜
영상(mp4)과 phase 사진을 RESULT/ 에 남긴다.

MuJoCo 로 직접 시뮬레이션(Genesis 아님) — <equality><joint> mimic 이 정확히
지켜지므로 실제 RG2 동작(moment_arm 마스터 + truss_arm/finger_tip/finger2 mimic)을
있는 그대로 확인할 수 있다.
"""
import os
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\m0609_rg2_v2.xml"
OUT_XML = os.path.join(HERE, "_rg2_only.xml")
RESULT_DIR = os.path.join(HERE, "RESULT")
os.makedirs(RESULT_DIR, exist_ok=True)

from datetime import datetime
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(RESULT_DIR, f"rg2_only_sweep_{_TS}.mp4")

# ── RG2 서브트리만 떼어내 world 에 직접 부착 (M0609_RG2/rg2_only_view.py 와 동일 패턴) ──
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
light = ET.SubElement(new_wb, "light", pos="0.3 -0.3 0.4", dir="-1 1 -1", diffuse="1 1 1")
new_wb.append(gripper_bracket)
root.remove(wb)
root.insert(list(root).index(root.find("asset")) + 1, new_wb)

act = root.find("actuator")
for a in list(act):
    if a.get("joint") != "gripper_joint":
        act.remove(a)

tree.write(OUT_XML)
print(f"[rg2_only_record] wrote {OUT_XML}")

m = mujoco.MjModel.from_xml_path(OUT_XML)
d = mujoco.MjData(m)
print(f"[rg2_only_record] nq={m.nq} nu={m.nu} neq={m.neq}")

gj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
Q_CLOSED, Q_OPEN = 0.8229, 1.8751   # m0609_rg2_v2.xml 의 검증된 joint range
d.qpos[m.jnt_qposadr[gj]] = Q_OPEN
mujoco.mj_forward(m, d)

r = mujoco.Renderer(m, height=480, width=640)
cam = mujoco.MjvCamera()
cam.lookat = [0, 0, 0.15]
cam.distance = 0.30
cam.azimuth = 60
cam.elevation = -15

frames = []


def _shot(tag):
    r.update_scene(d, camera=cam)
    img = r.render()
    Image.fromarray(img).save(os.path.join(RESULT_DIR, f"rg2_only_{tag}.png"))
    return img


def _settle(target, n=200):
    d.ctrl[0] = target
    for i in range(n):
        mujoco.mj_step(m, d)
        if i % 2 == 0:
            r.update_scene(d, camera=cam)
            frames.append(r.render())


print("[phase] open (start)")
_settle(Q_OPEN, 150)
_shot("open")
print("[phase] close")
_settle(Q_CLOSED, 300)
_shot("closed")
print("[phase] reopen")
_settle(Q_OPEN, 300)
_shot("reopen")
print("[phase] close again (2nd cycle)")
_settle(Q_CLOSED, 300)
_shot("closed2")

imageio.mimsave(MP4, frames, fps=60, quality=8)
print(f"\n[saved video] {MP4}  ({len(frames)} frames)")
print(f"[saved photos] {RESULT_DIR}/rg2_only_{{open,closed,reopen,closed2}}.png")

q_final = d.qpos[m.jnt_qposadr[gj]]
print(f"\n[state] final gripper_joint qpos = {q_final:.4f} rad "
      f"(range {Q_CLOSED:.4f}~{Q_OPEN:.4f})")
for jn in ("f1_truss_arm_joint", "f1_finger_tip_joint", "gripper_mirror_joint",
           "f2_truss_arm_joint", "f2_finger_tip_joint"):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    qv = d.qpos[m.jnt_qposadr[jid]]
    print(f"  mimic {jn:22s} = {qv:.4f}  (should == gripper_joint if mimic OK)")
