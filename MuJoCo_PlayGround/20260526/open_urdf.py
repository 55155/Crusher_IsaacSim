"""
open_urdf.py
원본 URDF 파일을 MuJoCo 뷰어로 열어 초기 형상을 확인합니다.

- Rigid32 (L7→L8) 가 fixed joint 이므로 L8은 L7에 고정된 상태
- 크랭크 메커니즘의 기준 형상(all joints = 0) 확인용

실행:
    conda activate isaaclab
    python open_urdf.py
"""
import os
import mujoco
import mujoco.viewer
import numpy as np

URDF_PATH = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "..", "..", "Crusher_IsaacSim_description", "urdf", "Crusher_IsaacSim.urdf")
)

model = mujoco.MjModel.from_xml_path(URDF_PATH)
data  = mujoco.MjData(model)
mujoco.mj_kinematics(model, data)

print("=" * 55)
print("  원본 URDF 로드 완료")
print(f"  경로: {URDF_PATH}")
print(f"  nbody={model.nbody}  njnt={model.njnt}  ngeom={model.ngeom}")
print("=" * 55)

# 모든 body 의 world 위치 출력
print("\n[Body 월드 위치 (all joints = 0)]")
for i in range(model.nbody):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    pos  = data.xpos[i]
    print(f"  [{i:2d}] {name:<35s}  {np.round(pos, 6)}")

# 모든 joint 정보 출력
print("\n[Joint 목록]")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    jtype = model.jnt_type[i]
    type_str = {0:"free", 1:"ball", 2:"slide", 3:"hinge"}.get(int(jtype), "?")
    axis = model.jnt_axis[i]
    print(f"  [{i}] {name:<45s}  type={type_str:<6s}  axis={np.round(axis,3)}")

print("\n뷰어 실행 중...")
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY   # body 프레임 표시
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
