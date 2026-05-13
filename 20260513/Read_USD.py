"""
Read_USD.py  -  Crushing_CollisionFree.usd 구조 읽기
=====================================================
실행 방법 (둘 다 동작):
  1. Isaac Sim Script Editor : 전체 붙여넣기 후 [Run]
  2. 독립 실행               : python Read_USD.py

출력:
  - 콘솔: 링크 / 조인트 / 가동 조인트 Drive 속성
  - 파일: C:/Crusher_isaacsim/20260513/USD_output.csv
"""

import os
import csv
import math

USD_PATH = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\Crushing_CollisionFree.usd"
CSV_PATH = r"C:\Crusher_isaacsim\20260513\USD_output.csv"

from pxr import Usd, UsdGeom, UsdPhysics

# ─────────────────────────────────────────────────────────────
# Stage 열기
# ─────────────────────────────────────────────────────────────
stage = Usd.Stage.Open(USD_PATH)
assert stage, f"USD 파일을 열 수 없습니다: {USD_PATH}"
xfc = UsdGeom.XformCache()

print("=" * 70)
print(f"  {os.path.basename(USD_PATH)}")
print("=" * 70)

# ─────────────────────────────────────────────────────────────
# 1. 링크 (RigidBodyAPI)
# ─────────────────────────────────────────────────────────────
links = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]

print(f"\n[링크]  총 {len(links)}개")
print(f"  {'#':>3}  {'이름':<45}  {'월드 위치 (x, y, z) [m]'}")
print("  " + "-" * 85)

link_rows = []
for i, p in enumerate(links, 1):
    t = xfc.GetLocalToWorldTransform(p).ExtractTranslation()
    print(f"  {i:>3}  {p.GetName():<45}  ({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f})")
    link_rows.append([i, p.GetName(), f"{t[0]:.6f}", f"{t[1]:.6f}", f"{t[2]:.6f}"])

# ─────────────────────────────────────────────────────────────
# 2. 조인트
# ─────────────────────────────────────────────────────────────
JOINT_TYPES = {
    "Fixed"    : UsdPhysics.FixedJoint,
    "Revolute" : UsdPhysics.RevoluteJoint,
    "Prismatic": UsdPhysics.PrismaticJoint,
    "Spherical": UsdPhysics.SphericalJoint,
}

joints = []
for p in stage.Traverse():
    for tname, tclass in JOINT_TYPES.items():
        if p.IsA(tclass):
            joints.append((p, tname))
            break

print(f"\n[조인트]  총 {len(joints)}개")
print(f"  {'#':>3}  {'이름':<45}  {'타입':<10}  {'Parent':<30}  {'Child'}")
print("  " + "-" * 120)

joint_rows = []
for i, (p, tname) in enumerate(joints, 1):
    api = UsdPhysics.Joint(p)
    b0 = api.GetBody0Rel().GetTargets()
    b1 = api.GetBody1Rel().GetTargets()
    parent = str(b0[0]).split("/")[-1] if b0 else "—"
    child  = str(b1[0]).split("/")[-1] if b1 else "—"
    print(f"  {i:>3}  {p.GetName():<45}  {tname:<10}  {parent:<30}  {child}")
    joint_rows.append([i, p.GetName(), tname, parent, child])

# ─────────────────────────────────────────────────────────────
# 3. 가동 조인트 Drive 속성
# ─────────────────────────────────────────────────────────────
movable = [(p, t) for p, t in joints if t != "Fixed"]

print(f"\n[가동 조인트 Drive 속성]  총 {len(movable)}개")
print(f"  {'이름':<45}  {'타입':<10}  {'축':<6}  "
      f"{'Lower':>10}  {'Upper':>10}  {'Stiffness':>12}  {'Damping':>10}  {'MaxForce':>10}")
print("  " + "-" * 120)

drive_rows = []
for p, tname in movable:
    def _get(attr_name):
        a = p.GetAttribute(attr_name)
        return a.Get() if (a and a.IsValid()) else None

    axis  = _get("physics:axis") or "—"
    lower = _get("physics:lowerLimit")
    upper = _get("physics:upperLimit")

    # angular 또는 linear drive 둘 다 시도
    drive_ns = "angular" if tname == "Revolute" else "linear"
    stiff = _get(f"drive:{drive_ns}:physics:stiffness")
    damp  = _get(f"drive:{drive_ns}:physics:damping")
    mf    = _get(f"drive:{drive_ns}:physics:maxForce")

    def _f(v): return f"{v:.4f}" if v is not None else "—"

    print(f"  {p.GetName():<45}  {tname:<10}  {str(axis):<6}  "
          f"{_f(lower):>10}  {_f(upper):>10}  {_f(stiff):>12}  {_f(damp):>10}  {_f(mf):>10}")
    drive_rows.append([p.GetName(), tname, str(axis),
                       _f(lower), _f(upper), _f(stiff), _f(damp), _f(mf)])

# ─────────────────────────────────────────────────────────────
# CSV 저장
# ─────────────────────────────────────────────────────────────
with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["source", USD_PATH])
    w.writerow([])

    w.writerow(["[LINKS]", f"total={len(link_rows)}"])
    w.writerow(["#", "name", "world_x[m]", "world_y[m]", "world_z[m]"])
    w.writerows(link_rows)
    w.writerow([])

    w.writerow(["[JOINTS]", f"total={len(joint_rows)}"])
    w.writerow(["#", "name", "type", "parent", "child"])
    w.writerows(joint_rows)
    w.writerow([])

    w.writerow(["[DRIVE]", f"total={len(drive_rows)}"])
    w.writerow(["name", "type", "axis", "lower", "upper", "stiffness", "damping", "maxForce"])
    w.writerows(drive_rows)

print(f"\n  → CSV 저장: {CSV_PATH}")
print("=" * 70)
