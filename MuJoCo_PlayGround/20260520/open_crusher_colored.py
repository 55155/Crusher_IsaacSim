"""
open_crusher_colored.py
Crusher_IsaacSim_colored.xml (Ground.stl 포함) 을 MuJoCo 뷰어로 엽니다.

실행:
    conda activate isaac_sim
    python open_crusher_colored.py
"""
import os
import mujoco
import mujoco.viewer

_HERE     = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.normpath(os.path.join(_HERE, "..", "MJCF", "Crusher_IsaacSim_colored.xml"))

model = mujoco.MjModel.from_xml_path(MJCF_PATH)
mujoco.viewer.launch(model, mujoco.MjData(model))
