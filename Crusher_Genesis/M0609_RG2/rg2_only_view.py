"""
rg2_only_view.py — RG2(v2) 그리퍼만 단독으로 world 에 붙여서 MuJoCo 인터랙티브
뷰어로 조립 상태를 확인한다 (M0609 팔 제외 — 장착 각도 문제와 분리).

MuJoCo 는 <equality><joint> mimic 을 제대로 지키므로(Genesis 와 달리) gripper_joint
슬라이더 하나로 실제 mimic 동작까지 정확히 볼 수 있다.
"""
import os
import xml.etree.ElementTree as ET
import mujoco
import mujoco.viewer

SRC = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\m0609_rg2_v2.xml"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_rg2_only.xml")

tree = ET.parse(SRC)
root = tree.getroot()
compiler = root.find("compiler")
compiler.set("meshdir", r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots")
wb = root.find("worldbody")

# gripper_bracket 서브트리를 link_6 에서 떼어내 worldbody 에 직접 부착
gripper_bracket = None
for body in wb.iter("body"):
    if body.get("name") == "gripper_bracket":
        gripper_bracket = body
        break
assert gripper_bracket is not None, "gripper_bracket not found"

# world 에 지면 삼을 얕은 참조 축 표시용 site 하나 추가 (방향 확인용)
new_wb = ET.Element("worldbody")
axis_body = ET.SubElement(new_wb, "body", name="world_axis")
ET.SubElement(axis_body, "geom", type="cylinder", size="0.003 0.05", pos="0.05 0 0",
              euler="0 90 0", rgba="1 0 0 0.6")   # 빨강 = world +X
ET.SubElement(axis_body, "geom", type="cylinder", size="0.003 0.05", pos="0 0.05 0",
              euler="90 0 0", rgba="0 1 0 0.6")   # 초록 = world +Y
ET.SubElement(axis_body, "geom", type="cylinder", size="0.003 0.05", pos="0 0 0.05",
              rgba="0 0 1 0.6")                    # 파랑 = world +Z
new_wb.append(gripper_bracket)

root.remove(wb)
root.insert(list(root).index(root.find("asset")) + 1, new_wb)

# actuator 는 gripper_joint 만 남기고, equality 는 그대로 유지 (mimic 확인용)
act = root.find("actuator")
if act is not None:
    for a in list(act):
        if a.get("joint") != "gripper_joint":
            act.remove(a)

tree.write(OUT)
print(f"[rg2_only] wrote {OUT}")

m = mujoco.MjModel.from_xml_path(OUT)
d = mujoco.MjData(m)
print(f"[rg2_only] nq={m.nq} nu={m.nu} neq={m.neq}")
print("[rg2_only] 뷰어를 엽니다. gripper_joint 액추에이터(하단 슬라이더 또는 Ctrl+Shift 드래그)로 여닫아 보세요.")
print("           빨강=world +X, 초록=world +Y, 파랑=world +Z 축입니다.")

mujoco.mj_forward(m, d)
mujoco.viewer.launch(m, d)
