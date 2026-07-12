"""
rg2_velocity_record.py — RG2(v2) 단독 그리퍼를 velocity actuator 로 구동해
열림(q=0)<->닫힘(q=1.3) 왕복시키고 영상(mp4)+phase 사진을 RESULT/ 에 남긴다.

m0609_rg2_v2.xml 이 [0,1.3] range 로 수정된 뒤의 정식 재현 영상 —
rg2_velocity_check.py 로 확인한 궤적(q=0→1.3, pad_gap 112mm→1.3mm 단조감소)을
그대로 촬영한다.
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
OUT_XML = os.path.join(HERE, "_rg2_only_vel_record.xml")
RESULT_DIR = os.path.join(HERE, "RESULT")
os.makedirs(RESULT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(RESULT_DIR, f"rg2_velocity_sweep_{_TS}.mp4")

# ── RG2 서브트리만 world 에 직접 부착 + velocity actuator 로 교체 ──
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

act = root.find("actuator")
for a in list(act):
    act.remove(a)
vel = ET.SubElement(act, "velocity")
vel.set("joint", "gripper_joint")
vel.set("kv", "5.0")
vel.set("ctrlrange", "-3.0 3.0")

tree.write(OUT_XML)
print(f"[vel-rec] wrote {OUT_XML}")

m = mujoco.MjModel.from_xml_path(OUT_XML)
d = mujoco.MjData(m)
print(f"[vel-rec] nq={m.nq} nu={m.nu} neq={m.neq}")

gj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint")
PAD_LOCAL = np.array([-0.02684, 0.0, 0.00425])
f1b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "f1_flex_finger")
f2b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "f2_flex_finger")


def pad_gap():
    R1 = d.xmat[f1b].reshape(3, 3); p1 = R1 @ PAD_LOCAL + d.xpos[f1b]
    R2 = d.xmat[f2b].reshape(3, 3); p2 = R2 @ PAD_LOCAL + d.xpos[f2b]
    return float(np.linalg.norm(p1 - p2) * 1000)


r = mujoco.Renderer(m, height=480, width=640)
cam = mujoco.MjvCamera()
cam.lookat = [0, 0, 0.15]
cam.distance = 0.30
cam.azimuth = 60
cam.elevation = -15

frames = []
DT = m.opt.timestep


def _run2(target_ctrl, stop_cond, max_t, tag):
    t = 0.0
    d.ctrl[0] = target_ctrl
    k = 0
    while not stop_cond() and t < max_t:
        mujoco.mj_step(m, d)
        t += DT
        k += 1
        if k % 3 == 0:
            r.update_scene(d, camera=cam)
            frames.append(r.render())
    d.ctrl[0] = 0.0
    for _ in range(30):
        mujoco.mj_step(m, d)
        r.update_scene(d, camera=cam)
        frames.append(r.render())
    print(f"[phase] {tag}  t={t:.3f}s  q={d.qpos[m.jnt_qposadr[gj]]:.4f}  "
          f"pad_gap={pad_gap():.2f}mm")
    r.update_scene(d, camera=cam)
    img = r.render()
    Image.fromarray(img).save(os.path.join(RESULT_DIR, f"rg2_velocity_{tag}.png"))


# 시작: q=0 (열림) 에 정렬
d.qpos[m.jnt_qposadr[gj]] = 0.02
for jn in ("f1_truss_arm_joint", "f1_finger_tip_joint", "gripper_mirror_joint",
           "f2_truss_arm_joint", "f2_finger_tip_joint"):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
    d.qpos[m.jnt_qposadr[jid]] = 0.0
mujoco.mj_forward(m, d)
r.update_scene(d, camera=cam)
Image.fromarray(r.render()).save(os.path.join(RESULT_DIR, "rg2_velocity_start.png"))
print(f"[phase] start  q=0.0000  pad_gap={pad_gap():.2f}mm")

_run2(1.0, lambda: d.qpos[m.jnt_qposadr[gj]] >= 1.3, 3.0, "closed")
_run2(-1.0, lambda: d.qpos[m.jnt_qposadr[gj]] <= 0.0, 3.0, "reopened")
_run2(1.0, lambda: d.qpos[m.jnt_qposadr[gj]] >= 1.3, 3.0, "closed2")

imageio.mimsave(MP4, frames, fps=60, quality=8)
print(f"\n[saved video] {MP4}  ({len(frames)} frames)")
print(f"[saved photos] {RESULT_DIR}/rg2_velocity_{{start,closed,reopened,closed2}}.png")
