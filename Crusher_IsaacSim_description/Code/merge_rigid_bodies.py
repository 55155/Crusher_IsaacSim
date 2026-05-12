"""
merge_rigid_bodies.py
Fixed joint 체인을 single rigid body로 병합

원본: Crushing.usd          (변경 없음)
결과: Crushing_merged.usd   (신규 생성)

핵심: Flatten 후 편집 (서브레이어 참조 문제 해결)

병합 결과 (42개 rigid body → 7개):
  [0] fixed_base            (world 고정점, 그대로 유지)
  [1] base_link             ← 28개 링크 병합 (bearing holder 2개 포함)
  [2] L4_Shaft_1            ← 3개 링크 병합 (Motor1 회전축)
  [3] L6_Link2_1
  [4] L7_Link3_1            ← 3개 링크 병합 (파쇄판 체인)
  [5] L4_Motor2_Shaft_1     ← 4개 링크 병합 (Motor2 회전축)
  [6] L2_Left_Wall1_1       ← 2개 링크 병합 (슬라이딩 벽)

수정: Fusion2URDF 조립 오류로 bearing holder가 shaft 자식(revolute)으로
     잘못 연결됨 → base_group으로 이동 + 해당 revolute joint 비활성화
"""

import os
import numpy as np
import trimesh
from pxr import Usd, UsdPhysics, UsdGeom, Gf

# ── 설정 ──────────────────────────────────────────────────────────────────────
USD_SRC    = "C:/TEMP/Crusher_IsaacSim_description/Crushing.usd"
USD_DST    = "C:/TEMP/Crusher_IsaacSim_description/Crushing_merged.usd"
MESH_DIR   = "C:/TEMP/Crusher_IsaacSim_description/meshes"
ROBOT_PATH = "/World/Crusher_IsaacSim"
DENSITY    = 7850.0
MIN_I      = 1e-6     # kg·m²  관성 최솟값

# Fusion2URDF 조립 오류: shaft → bearing_holder(revolute) 잘못 연결
# bearing holder는 프레임 고정 부품 → base_group 편입 후 이 joint 비활성화
BEARING_JOINTS = {
    "L6_DcutShaft_1_L7_Holder_Bearing1_1",
    "L6_DcutShaft_1_L7_Holder_Bearing2_1",
}

# ── 병합 그룹 ─────────────────────────────────────────────────────────────────
GROUPS = [
    {
        "name": "base_group", "root": "base_link",
        "links": [
            "base_link",
            "L1_Braket1_1", "L2_Braket2_1",
            "L3_Bevel_GearBox_1", "L4_Bevel_GearBox2_1",
            "L4_Motor1_FrontBearing_1", "L5_Motor1_FrontSnapring_1",
            "L4_Motor1_RearBearing_1",  "L5_Motor1_RearSnapring_1",
            "L4_Reducer1_1", "L5_Reducer2_1",
            "L6_Motor1_Bolt_1", "L6_Motor1_Braket_1", "L7_Motor1_Body_1",
            "L1_Guide1_1", "L1_Guide2_1",
            "L1_Slider_jig_1", "L2_Linear_bush_1",
            "L1_Wall1_1", "L2_GearCase3_1",
            "L2__Motor2_Braket_1", "L3_Motor2_1",
            "L1_Wall2_1", "L2_GearCase2_1", "L3_GearCase1_1", "L2_Wall3_1",
            # Fusion2URDF 오류로 shaft 자식으로 잘못 연결된 bearing holder
            # 물리적으로 프레임에 고정된 부품 → base에 병합
            "L7_Holder_Bearing1_1", "L7_Holder_Bearing2_1",
        ],
    },
    {
        "name": "shaft_group", "root": "L4_Shaft_1",
        "links": ["L4_Shaft_1", "L5_Key_1", "L5_Link1_1"],
    },
    {"name": "link2_group",  "root": "L6_Link2_1",  "links": ["L6_Link2_1"]},
    {
        "name": "plate_group", "root": "L7_Link3_1",
        "links": ["L7_Link3_1", "L8_Link3_Shaft_1", "L9_PLATE_v3_1"],
    },
    {
        "name": "motor2_shaft_group", "root": "L4_Motor2_Shaft_1",
        "links": ["L4_Motor2_Shaft_1", "L5_Coupler_1", "L6_DcutShaft_1", "L7_SpurGear_1"],
    },
    {
        "name": "wall_group", "root": "L2_Left_Wall1_1",
        "links": ["L2_Left_Wall1_1", "L3_RackGear_1"],
    },
]

# ── 유틸리티 ──────────────────────────────────────────────────────────────────
def link_to_stl(name):
    if name.startswith("L") and len(name) > 1 and name[1].isdigit():
        return name[1:] + ".stl"
    return name + ".stl"

def gf_to_np(mat4d):
    arr = np.array([[mat4d[r][c] for c in range(4)] for r in range(4)])
    return arr.T

def rotation_to_quatf(R):
    if np.linalg.det(R) < 0:          # improper rotation 보정
        R = R.copy(); R[:, 2] = -R[:, 2]
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w, x, y, z = 0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        w, x, y, z = (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s
    mag = np.sqrt(w*w + x*x + y*y + z*z)
    if mag > 1e-10:
        w, x, y, z = w/mag, x/mag, y/mag, z/mag
    return Gf.Quatf(float(w), float(x), float(y), float(z))

def quatf_to_matrix(q):
    """Gf.Quatf → 3×3 rotation matrix"""
    w = q.GetReal()
    x, y, z = q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

# ── 그룹 결합 관성 계산 ───────────────────────────────────────────────────────
def compute_group(stage, group):
    meshes = []
    for link_name in group["links"]:
        stl_path = os.path.join(MESH_DIR, link_to_stl(link_name))
        if not os.path.exists(stl_path):
            continue
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{link_name}")
        if not prim.IsValid():
            continue
        mesh = trimesh.load_mesh(stl_path)
        mesh.apply_scale(1e-3)
        T = gf_to_np(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        mesh.apply_transform(T)
        meshes.append(mesh)

    if not meshes:
        return None

    combined       = trimesh.util.concatenate(meshes)
    combined.density = DENSITY
    com_world      = np.array(combined.center_mass)
    I_world        = np.array(combined.moment_inertia)

    root_prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{group['root']}")
    T_root    = gf_to_np(UsdGeom.Xformable(root_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    T_inv     = np.linalg.inv(T_root)
    R_inv     = T_inv[:3, :3]

    com_local = (T_inv @ np.append(com_world, 1.0))[:3]
    I_local   = R_inv @ I_world @ R_inv.T
    eigvals, eigvecs = np.linalg.eigh(I_local)
    eigvals   = np.maximum(eigvals, MIN_I)

    return {
        "mass":      float(combined.mass),
        "com":       Gf.Vec3f(*com_local.tolist()),
        "diagonal":  Gf.Vec3f(*eigvals.tolist()),
        "principal": rotation_to_quatf(eigvecs),
    }

# ── 메인 ─────────────────────────────────────────────────────────────────────
# 1) 원본 스테이지를 Flatten → 단일 레이어 USD 생성 (서브레이어 참조 해소)
print("원본 Flatten 중 (서브레이어 해소)...")
src_stage = Usd.Stage.Open(USD_SRC)
src_stage.Export(USD_DST)   # 모든 레이어를 하나의 파일로 합침
print(f"  저장: {USD_DST}")

stage = Usd.Stage.Open(USD_DST)

# 링크 → 그룹명, 링크 → 그룹 root 매핑
link_to_group_name = {lk: g["name"]  for g in GROUPS for lk in g["links"]}
link_to_group_root = {lk: g["root"]  for g in GROUPS for lk in g["links"]}

# ── 1단계: 결합 관성 계산 및 root 설정 ──────────────────────────────────────
print("\n[1/3] 결합 관성 계산 및 root 링크 업데이트")
for group in GROUPS:
    result = compute_group(stage, group)
    if result is None:
        continue
    root_prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{group['root']}")
    if not root_prim.HasAPI(UsdPhysics.MassAPI):
        UsdPhysics.MassAPI.Apply(root_prim)
    api = UsdPhysics.MassAPI(root_prim)
    api.GetMassAttr().Set(result["mass"])
    api.GetDiagonalInertiaAttr().Set(result["diagonal"])
    api.GetCenterOfMassAttr().Set(result["com"])
    api.GetPrincipalAxesAttr().Set(result["principal"])
    print(f"  [{group['name']}] {len(group['links'])}개 → {group['root']}"
          f"  {result['mass']*1000:.1f}g  I={[f'{v:.2e}' for v in result['diagonal']]}")

# ── 2단계: 비-root 링크 RigidBodyAPI 비활성화 ───────────────────────────────
print("\n[2/3] 비-root 링크 RigidBody 비활성화")
disabled_rb = 0
for group in GROUPS:
    for link_name in group["links"]:
        if link_name == group["root"]:
            continue
        prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{link_name}")
        if not prim.IsValid():
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            # Flatten 후엔 RemoveAPI 또는 속성 override 모두 가능
            rb_api = UsdPhysics.RigidBodyAPI(prim)
            rb_api.GetRigidBodyEnabledAttr().Set(False)
            disabled_rb += 1
print(f"  비활성화된 RigidBody: {disabled_rb}개")

# ── 3단계: 그룹 내 FixedJoint 비활성화 ──────────────────────────────────────
print("\n[3/3] 그룹 내 FixedJoint 비활성화")
deactivated = 0
kept = 0

def process_joints_scope(scope_prim):
    global deactivated, kept
    for joint in scope_prim.GetChildren():
        jname = joint.GetName()

        # Fusion2URDF 오류 bearing joint: shaft→bearing_holder(revolute) → 명시적 비활성화
        if jname in BEARING_JOINTS:
            joint.SetActive(False)
            deactivated += 1
            print(f"  [bearing 오류 보정] {jname} 비활성화")
            continue

        if joint.GetTypeName() != "PhysicsFixedJoint":
            kept += 1
            continue

        b0 = joint.GetRelationship("physics:body0").GetTargets()
        b1 = joint.GetRelationship("physics:body1").GetTargets()
        b0n = str(b0[0]).split("/")[-1] if b0 else None
        b1n = str(b1[0]).split("/")[-1] if b1 else ""

        # root_joint (world→fixed_base) 및 fixed_base→base_link 유지
        if b0n is None or b0n == "fixed_base":
            kept += 1
            continue

        g0 = link_to_group_name.get(b0n)
        g1 = link_to_group_name.get(b1n)
        if g0 and g1 and g0 == g1:
            joint.SetActive(False)    # Flatten 후 SetActive 정상 동작
            deactivated += 1
        else:
            kept += 1

joints_scope = stage.GetPrimAtPath(f"{ROBOT_PATH}/joints")
if joints_scope.IsValid():
    process_joints_scope(joints_scope)

# root_joint 는 joints scope 밖에 있음
root_joint = stage.GetPrimAtPath(f"{ROBOT_PATH}/root_joint")
if root_joint.IsValid():
    kept += 1

print(f"  비활성화된 FixedJoint: {deactivated}개")
print(f"  유지된 joint         : {kept}개")

# ── 4단계: Revolute/Prismatic joint body0 참조 + joint frame 재계산 ───────────
# body0만 바꾸면 localPos0/localRot0이 여전히 구 body0 로컬 기준으로 남아
# 시뮬 시작 시 joint anchor 위치가 틀려져 링크들이 뜯어짐
# → T_rel = inv(T_new_body0_world) @ T_old_body0_world 로 frame 변환 필요
print("\n[4/4] 활성 joint body0 참조 + local frame 재계산")
from pxr import Sdf
updated_refs = 0
for joint in joints_scope.GetChildren():
    if not joint.IsActive():
        continue
    jtype = joint.GetTypeName()
    if jtype not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
        continue
    b0 = joint.GetRelationship("physics:body0").GetTargets()
    if not b0:
        continue
    b0n = str(b0[0]).split("/")[-1]
    new_root = link_to_group_root.get(b0n)
    if not new_root or new_root == b0n:
        continue

    # 구 body0 → 새 body0(group root) 상대 변환
    T_old = gf_to_np(UsdGeom.Xformable(
        stage.GetPrimAtPath(f"{ROBOT_PATH}/{b0n}")
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    T_new = gf_to_np(UsdGeom.Xformable(
        stage.GetPrimAtPath(f"{ROBOT_PATH}/{new_root}")
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    T_rel = np.linalg.inv(T_new) @ T_old   # old_frame → new_body0 frame

    # localPos0 변환: new_pos = T_rel @ [old_pos, 1]
    pos0_attr = joint.GetAttribute("physics:localPos0")
    if pos0_attr and pos0_attr.Get() is not None:
        p = np.array(list(pos0_attr.Get()))
        new_p = (T_rel @ np.append(p, 1.0))[:3]
        pos0_attr.Set(Gf.Vec3f(*new_p.tolist()))

    # localRot0 변환: new_R = R_rel @ old_R
    rot0_attr = joint.GetAttribute("physics:localRot0")
    if rot0_attr and rot0_attr.Get() is not None:
        R_rel = T_rel[:3, :3]
        det = np.linalg.det(R_rel)
        if abs(det) > 1e-10:
            R_rel = R_rel / np.cbrt(abs(det))   # 스케일 제거
        R_new = R_rel @ quatf_to_matrix(rot0_attr.Get())
        rot0_attr.Set(rotation_to_quatf(R_new))

    # body0 참조 업데이트
    joint.GetRelationship("physics:body0").SetTargets(
        [Sdf.Path(f"{ROBOT_PATH}/{new_root}")]
    )
    print(f"  {joint.GetName()}: body0 {b0n} → {new_root}  (frame 재계산)")
    updated_refs += 1
print(f"  업데이트된 참조: {updated_refs}개")

# ── 저장 ─────────────────────────────────────────────────────────────────────
stage.GetRootLayer().Save()

print("\n" + "=" * 65)
print("완료!")
print(f"  원본 (변경 없음): {USD_SRC}")
print(f"  병합 결과       : {USD_DST}")
print(f"  rigid body 수   : 42개 → {len(GROUPS)+1}개 (fixed_base 포함)")
print(f"  활성 joint 수   : 6개 (revolute×4, prismatic×1, fixed×1)")
print(f"  bearing joint   : 2개 비활성화 (Fusion2URDF 조립 오류 보정)")
print(f"  joint body0     : 비활성 링크 → 그룹 root로 리매핑 완료")
print(f"  시각적 재질     : MDL 셰이더 원본 그대로 유지")
