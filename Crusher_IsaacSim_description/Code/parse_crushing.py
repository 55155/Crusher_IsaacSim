"""
Crushing.usd 파싱 스크립트
- 링크(Xform) 목록
- 조인트 목록 및 타입 (Revolute / Prismatic / Fixed)
- 머티리얼 목록
- 물리 씬 정보
"""

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf

USD_PATH = "C:/TEMP/Crusher_IsaacSim_description/Crushing.usd"

stage = Usd.Stage.Open(USD_PATH)

# ── 1. 링크 수집 (visuals / collisions 제외한 Xform) ─────────────────────────
print("=" * 60)
print("[ LINKS ]")
print("=" * 60)

links = []
for prim in stage.Traverse():
    if prim.GetTypeName() != "Xform":
        continue
    path = str(prim.GetPath())
    # visuals / collisions / joints 등 하위 노드 제외
    last = path.split("/")[-1]
    if last in ("visuals", "collisions", "joints", "Looks"):
        continue
    links.append(path)
    print(path)

print(f"\n총 링크 수: {len(links)}\n")

# ── 2. 조인트 수집 ────────────────────────────────────────────────────────────
print("=" * 60)
print("[ JOINTS ]")
print("=" * 60)

joint_types = {
    "PhysicsRevoluteJoint":  [],
    "PhysicsPrismaticJoint": [],
    "PhysicsFixedJoint":     [],
}

for prim in stage.Traverse():
    t = prim.GetTypeName()
    if t not in joint_types:
        continue

    path = str(prim.GetPath())
    name = path.split("/")[-1]

    # body0 / body1 (연결된 링크) 읽기
    body0 = prim.GetRelationship("physics:body0").GetTargets()
    body1 = prim.GetRelationship("physics:body1").GetTargets()
    b0 = str(body0[0]) if body0 else "N/A"
    b1 = str(body1[0]) if body1 else "N/A"

    # Revolute / Prismatic → 축 정보
    axis_info = ""
    if t in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
        axis_attr = prim.GetAttribute("physics:axis")
        if axis_attr:
            axis_info = f"  axis={axis_attr.Get()}"

        limit_low  = prim.GetAttribute("physics:lowerLimit")
        limit_high = prim.GetAttribute("physics:upperLimit")
        if limit_low and limit_high:
            axis_info += f"  limits=[{limit_low.Get():.3f}, {limit_high.Get():.3f}]"

    joint_types[t].append((name, b0, b1, axis_info))

for jtype, joints in joint_types.items():
    print(f"\n-- {jtype} ({len(joints)}개) --")
    for name, b0, b1, axis_info in joints:
        print(f"  {name}")
        print(f"    body0: {b0}")
        print(f"    body1: {b1}{axis_info}")

total_joints = sum(len(v) for v in joint_types.values())
print(f"\n총 조인트 수: {total_joints}\n")

# ── 3. 머티리얼 수집 ──────────────────────────────────────────────────────────
print("=" * 60)
print("[ MATERIALS ]")
print("=" * 60)

for prim in stage.Traverse():
    if prim.GetTypeName() != "Material":
        continue
    mat = UsdShade.Material(prim)
    print(f"  {prim.GetPath()}")
    # Surface shader 찾기
    for output in mat.GetOutputs():
        sources, _ = output.GetConnectedSources()
        if sources:
            shader_path = sources[0].source.GetPrim().GetPath()
            print(f"    [{output.GetBaseName()}] shader: {shader_path}")

# ── 4. 물리 씬 정보 ───────────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("[ PHYSICS SCENE ]")
print("=" * 60)

for prim in stage.Traverse():
    if prim.GetTypeName() == "PhysicsScene":
        gravity_dir  = prim.GetAttribute("physics:gravityDirection")
        gravity_mag  = prim.GetAttribute("physics:gravityMagnitude")
        gdir = gravity_dir.Get()  if gravity_dir  else "N/A"
        gmag = gravity_mag.Get()  if gravity_mag  else "N/A"
        print(f"  Path            : {prim.GetPath()}")
        print(f"  Gravity dir     : {gdir}")
        print(f"  Gravity magnitude: {gmag}")
