"""
USD 파일의 rigid body 트리를 출력합니다.
python print_tree.py [usd_path]
"""
import sys
from pxr import Usd, UsdPhysics

USD_PATH   = sys.argv[1] if len(sys.argv) > 1 else \
    "C:/TEMP/Crusher_IsaacSim_description/Crushing_merged.usd"
ROBOT_PATH = "/World/Crusher_IsaacSim"

stage = Usd.Stage.Open(USD_PATH)

# ── 활성 RigidBody 집합 및 비-RB 링크의 그룹 root 매핑 ──────────────────────
active_rb  = set()   # 활성 RigidBody 링크명
rb_root_of = {}      # 비활성 링크명 → 해당 그룹의 root 링크명 (병합 후 리매핑용)

for prim in stage.GetPrimAtPath(ROBOT_PATH).GetChildren():
    name = prim.GetName()
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    attr = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
    if attr is None or attr is True:
        active_rb.add(name)

# 비활성 RigidBody → 활성 부모 찾기 (active joint path 역추적)
# 단순화: 모든 active fixed joint 에서 body1이 비활성이면 body0의 활성 root를 따라감
def find_active_root(link_name, visited=None):
    if visited is None:
        visited = set()
    if link_name in active_rb:
        return link_name
    if link_name in visited:
        return link_name
    visited.add(link_name)
    # joints 스코프에서 이 링크가 body1인 fixed joint 찾기
    for joint in stage.GetPrimAtPath(f"{ROBOT_PATH}/joints").GetChildren():
        if not joint.IsActive():
            continue
        if joint.GetTypeName() != "PhysicsFixedJoint":
            continue
        b1 = joint.GetRelationship("physics:body1").GetTargets()
        if b1 and str(b1[0]).split("/")[-1] == link_name:
            b0 = joint.GetRelationship("physics:body0").GetTargets()
            if b0:
                return find_active_root(str(b0[0]).split("/")[-1], visited)
    return link_name

# ── 모든 조인트 수집 ──────────────────────────────────────────────────────────
children_of = {}
root_body   = None

JOINT_TYPES = {"PhysicsFixedJoint", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}

def collect_joints(prim):
    global root_body
    for child in prim.GetChildren():
        if child.GetTypeName() in JOINT_TYPES:
            if not child.IsActive():
                continue
            jtype = child.GetTypeName()
            jname = str(child.GetPath()).split("/")[-1]
            b0 = child.GetRelationship("physics:body0").GetTargets()
            b1 = child.GetRelationship("physics:body1").GetTargets()
            b0n = str(b0[0]).split("/")[-1] if b0 else None
            b1n = str(b1[0]).split("/")[-1] if b1 else ""

            short = {"PhysicsFixedJoint":"FIXED",
                     "PhysicsRevoluteJoint":"REV",
                     "PhysicsPrismaticJoint":"PRISM"}.get(jtype, jtype)

            if b0n is None:
                root_body = b1n
            else:
                # 비활성 body0 → 활성 root로 리매핑
                effective_b0 = find_active_root(b0n) if b0n not in active_rb else b0n
                children_of.setdefault(effective_b0, []).append((b1n, short, jname))
        elif child.GetTypeName() == "Scope":
            collect_joints(child)

robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
collect_joints(robot_prim)

# ── 링크 정보 (RigidBody 활성 여부, 질량) ────────────────────────────────────
def link_info(name):
    prim = stage.GetPrimAtPath(f"{ROBOT_PATH}/{name}")
    if not prim.IsValid():
        return ""
    has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
    rb_on  = False
    mass_s = ""
    if has_rb:
        attr = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        rb_on = (attr is None or attr == True)
    if prim.HasAPI(UsdPhysics.MassAPI):
        m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if m and m > 0:
            mass_s = f"  {m*1000:.1f}g"
    mark = " [RB]" if (has_rb and rb_on) else (" [rb-off]" if has_rb else "")
    return f"{mark}{mass_s}"

# ── 트리 재귀 출력 ────────────────────────────────────────────────────────────
def print_tree(node, prefix="", is_last=True):
    connector = "└── " if is_last else "├── "
    print(f"{prefix}{connector}{node}{link_info(node)}")
    ext = "    " if is_last else "│   "
    kids = children_of.get(node, [])
    for i, (child, jtype, jname) in enumerate(kids):
        last  = (i == len(kids) - 1)
        j_pre = prefix + ext
        j_con = "└── " if last else "├── "
        print(f"{j_pre}{j_con}[{jtype}] {jname}")
        print_tree(child, j_pre + ("    " if last else "│   "), is_last=True)

print(f"USD : {USD_PATH}\n")
print("WORLD")
if root_body:
    print("└── [FIXED] root_joint")
    print_tree(root_body, "    ")
else:
    print("  (root body 없음)")
