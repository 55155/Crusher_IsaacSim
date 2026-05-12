"""
Crushing_merged.usd 모터 조인트 구동 스크립트
Isaac Sim Script Editor (Window > Script Editor) 에서 실행
Isaac Sim 4.x 대응: async/await + isaacsim.core.api import

수정 이력:
  - CoM 보정: fixed_base 한정 → 모든 링크 inf/NaN 검사로 확장
  - 중첩 ArticulationRootAPI 제거: Fusion2URDF 변환 시 하위 링크에
    ArticulationRootAPI가 이미 붙어있어 "Nested articulation roots" 오류 발생
    → 하위 링크에서 전부 제거 후 로봇 루트에만 적용
  - physics context 초기화 대기: World() 생성 직후 frame update 3회 추가
    (미대기 시 get_physics_context() == None → warm_start AttributeError)
  - damping=50.0, max_force=100 N·m (병합 후 관성/질량 증가)
  - USD_PATH → Crushing_merged.usd (42 RB → 7 RB)
"""

import asyncio
import math
import numpy as np
from pxr import UsdPhysics, Gf
import omni.usd
import omni.kit.app

# ── 설정 ──────────────────────────────────────────────────────────────────────
USD_PATH    = "C:/TEMP/Crusher_IsaacSim_description/Crushing_merged.usd"
ROBOT_PATH  = "/World/Crusher_IsaacSim"
JOINTS_PATH = f"{ROBOT_PATH}/joints"

MOTOR_TARGETS = {
    "L3_Bevel_GearBox_1_L4_Shaft_1": 1.0,   # Motor1: base_link → L4_Shaft_1 (rad/s)
    "L3_Motor2_1_L4_Motor2_Shaft_1": 1.0,   # Motor2: base_link → L4_Motor2_Shaft_1 (rad/s)
}

DAMPING   = 50.0
STIFFNESS = 0.0
MAX_FORCE = 100.0   # N·m
SIM_STEPS = 500

# ── Isaac Sim 4.x / 2023.x 모두 대응하는 import ──────────────────────────────
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

    # ── 모든 링크 CoM 유효성 보정 (inf / NaN → 0,0,0) ─────────────────────────
    fixed_com = 0
    for prim in stage.GetPrimAtPath(ROBOT_PATH).GetChildren():
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        api = UsdPhysics.MassAPI(prim)
        com = api.GetCenterOfMassAttr().Get()
        if com is None:
            continue
        bad = any(math.isinf(v) or math.isnan(v) or abs(v) > 1e9 for v in com)
        if bad:
            api.GetCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            print(f"  CoM 보정: {prim.GetName()}  {tuple(com)} → (0,0,0)")
            fixed_com += 1
    print(f"[2/5] CoM 보정 완료 ({fixed_com}개 링크)")

    # ── 중첩 ArticulationRootAPI 제거 후 로봇 루트에만 적용 ───────────────────
    # Fusion2URDF → USD 변환 시 fixed_base 등 하위 링크에 ArticulationRootAPI가
    # 이미 붙어있어 Isaac Sim이 "Nested articulation roots" 오류를 발생시킴
    robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
    removed = 0
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        # 로봇 루트 하위 프림에서만 제거
        if not path_str.startswith(ROBOT_PATH + "/"):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAppliedSchema("PhysicsArticulationRootAPI")
            print(f"  중첩 ArticulationRoot 제거: {prim.GetName()}")
            removed += 1

    if not robot_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
        print("  ArticulationRootAPI → /World/Crusher_IsaacSim 에 적용")
    else:
        print("  ArticulationRootAPI 이미 존재 (로봇 루트)")

    # ── Drive API 설정 (velocity control) ─────────────────────────────────────
    for joint_name in MOTOR_TARGETS:
        prim = stage.GetPrimAtPath(f"{JOINTS_PATH}/{joint_name}")
        if not prim.IsValid():
            print(f"  WARNING: 조인트 없음 → {joint_name}")
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(STIFFNESS)
        drive.GetDampingAttr().Set(DAMPING)
        drive.GetMaxForceAttr().Set(MAX_FORCE)
        drive.GetTargetVelocityAttr().Set(0.0)
        print(f"  Drive 적용: {joint_name}  damping={DAMPING}  max_force={MAX_FORCE} N·m")

    print("[3/5] Drive API 설정 완료")

    # ── World / Articulation 초기화 ───────────────────────────────────────────
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World()

    # World() 생성 직후 physics context가 아직 초기화되지 않은 경우가 있음
    # → frame update 3회로 초기화 완료 대기 (미대기 시 warm_start AttributeError)
    for _ in range(3):
        await omni.kit.app.get_app().next_update_async()

    robot = world.scene.add(Articulation(prim_path=ROBOT_PATH, name="crusher"))
    await world.reset_async()

    print(f"[4/5] Articulation 초기화 완료")
    print(f"      DOF 수   : {robot.num_dof}")
    print(f"      DOF 이름 : {robot.dof_names}")

    motor_idx = {}
    for i, dof in enumerate(robot.dof_names):
        for joint_name, vel in MOTOR_TARGETS.items():
            if joint_name in dof or dof in joint_name:
                motor_idx[i] = vel
                print(f"      → 모터 DOF [{i}] : {dof}  목표={vel} rad/s")

    if not motor_idx:
        print("  WARNING: 모터 DOF를 찾지 못했습니다. DOF 이름을 확인하세요.")

    # ── 시뮬레이션 루프 ───────────────────────────────────────────────────────
    print(f"[5/5] 시뮬레이션 시작 ({SIM_STEPS} steps)")

    for step in range(SIM_STEPS):
        velocities = np.zeros(robot.num_dof)
        for idx, vel in motor_idx.items():
            velocities[idx] = vel

        robot.apply_action(ArticulationAction(joint_velocities=velocities))
        await world.step_async(render=True)

        if step % 100 == 0 and motor_idx:
            pos     = robot.get_joint_positions()
            vel_cur = robot.get_joint_velocities()
            print(f"  step={step:4d} | "
                  f"pos={[f'{pos[i]:.3f}' for i in motor_idx]} rad | "
                  f"vel={[f'{vel_cur[i]:.3f}' for i in motor_idx]} rad/s")

    print("시뮬레이션 완료")


asyncio.ensure_future(main())
