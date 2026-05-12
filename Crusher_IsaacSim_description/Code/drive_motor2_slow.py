"""
Crushing_CollisionFree.usd — L3_Motor2_1_L4_Motor2_Shaft_1 저속 구동
Isaac Sim Script Editor (Window > Script Editor) 에서 실행

설정값:
  velocity  = 0.05 rad/s  (매우 느린 속도, 튀는 현상 방지)
  damping   = 200.0       (높은 감쇠, 진동 억제)
  stiffness = 0.0         (velocity control 모드)
  max_force = 50.0 N·m
"""

import asyncio
import math
import numpy as np
from pxr import UsdPhysics, Gf
import omni.usd
import omni.kit.app

# ── 설정 ──────────────────────────────────────────────────────────────────────
USD_PATH    = "C:/TEMP/Crusher_IsaacSim_description/Crushing_CollisionFree.usd"
ROBOT_PATH  = "/World/Crusher_IsaacSim"
JOINTS_PATH = f"{ROBOT_PATH}/joints"

TARGET_JOINT    = "L3_Motor2_1_L4_Motor2_Shaft_1"
TARGET_VELOCITY = 0.05   # rad/s — 느릴수록 안정적

DAMPING   = 200.0
STIFFNESS = 0.0
MAX_FORCE = 50.0    # N·m

SIM_STEPS = 1000    # 약 1000 * dt 초 시뮬

# ── Isaac Sim 버전 호환 import ─────────────────────────────────────────────────
try:
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation as Articulation
    from isaacsim.core.utils.types import ArticulationAction
    print("Import: isaacsim.core.api (Isaac Sim 4.x)")
except ImportError:
    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.utils.types import ArticulationAction
    print("Import: omni.isaac.core (Isaac Sim 2023.x)")


async def main():
    # ── Stage 열기 ────────────────────────────────────────────────────────────
    await omni.usd.get_context().open_stage_async(USD_PATH)
    await omni.kit.app.get_app().next_update_async()

    stage = omni.usd.get_context().get_stage()
    print("[1/5] Stage 로드 완료")

    # ── CoM 유효성 보정 (inf / NaN → 0,0,0) ──────────────────────────────────
    fixed_com = 0
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_PATH):
            continue
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        api = UsdPhysics.MassAPI(prim)
        com = api.GetCenterOfMassAttr().Get()
        if com is None:
            continue
        if any(math.isinf(v) or math.isnan(v) or abs(v) > 1e9 for v in com):
            api.GetCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            fixed_com += 1
    print(f"[2/5] CoM 보정 완료 ({fixed_com}개 링크)")

    # ── 중첩 ArticulationRootAPI 제거 후 루트에만 적용 ───────────────────────
    robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if not path_str.startswith(ROBOT_PATH + "/"):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAppliedSchema("PhysicsArticulationRootAPI")

    if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    print("[3/5] ArticulationRootAPI 정리 완료")

    # ── Drive API 설정 (velocity control) ─────────────────────────────────────
    joint_prim = stage.GetPrimAtPath(f"{JOINTS_PATH}/{TARGET_JOINT}")
    if not joint_prim.IsValid():
        print(f"  ERROR: 조인트를 찾을 수 없습니다 → {JOINTS_PATH}/{TARGET_JOINT}")
        print("  stage.Traverse()로 조인트 경로를 확인하세요.")
        return

    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.GetStiffnessAttr().Set(STIFFNESS)
    drive.GetDampingAttr().Set(DAMPING)
    drive.GetMaxForceAttr().Set(MAX_FORCE)
    drive.GetTargetVelocityAttr().Set(0.0)
    print(f"  Drive 적용: {TARGET_JOINT}  vel_target=0 → {TARGET_VELOCITY} rad/s  "
          f"damping={DAMPING}  max_force={MAX_FORCE} N·m")

    # ── World / Articulation 초기화 ───────────────────────────────────────────
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World()

    for _ in range(3):
        await omni.kit.app.get_app().next_update_async()

    robot = world.scene.add(Articulation(prim_path=ROBOT_PATH, name="crusher"))
    await world.reset_async()

    print(f"[4/5] Articulation 초기화 완료")
    print(f"      DOF 수   : {robot.num_dof}")
    print(f"      DOF 이름 : {robot.dof_names}")

    # TARGET_JOINT 이름을 포함하는 DOF 인덱스 탐색
    motor_idx = None
    for i, dof in enumerate(robot.dof_names):
        if TARGET_JOINT in dof or dof in TARGET_JOINT:
            motor_idx = i
            print(f"      → 모터 DOF [{i}] : {dof}")
            break

    if motor_idx is None:
        print(f"  ERROR: '{TARGET_JOINT}' 에 해당하는 DOF를 찾지 못했습니다.")
        print(f"  사용 가능한 DOF: {robot.dof_names}")
        return

    # ── 시뮬레이션 루프 ───────────────────────────────────────────────────────
    print(f"[5/5] 시뮬레이션 시작 ({SIM_STEPS} steps, {TARGET_VELOCITY} rad/s)")

    velocities = np.zeros(robot.num_dof)
    velocities[motor_idx] = TARGET_VELOCITY

    for step in range(SIM_STEPS):
        robot.apply_action(ArticulationAction(joint_velocities=velocities))
        await world.step_async(render=True)

        if step % 200 == 0:
            pos     = robot.get_joint_positions()
            vel_cur = robot.get_joint_velocities()
            print(f"  step={step:4d} | "
                  f"pos={pos[motor_idx]:.4f} rad ({math.degrees(pos[motor_idx]):.2f} deg) | "
                  f"vel={vel_cur[motor_idx]:.4f} rad/s")

    print("시뮬레이션 완료")


asyncio.ensure_future(main())
