"""
각 링크의 mass / inertia 속성 출력 스크립트
python check_mass_inertia.py 로 독립 실행 가능 (usd-core 사용)
"""

from pxr import Usd, UsdPhysics, Gf

USD_PATH = "C:/TEMP/Crusher_IsaacSim_description/Crushing.usd"

stage = Usd.Stage.Open(USD_PATH)

# 무시할 경로 키워드
SKIP = ("visuals", "collisions", "joints", "Looks", "Render",
        "Environment", "physicsScene")

print(f"{'LINK':<55} {'MASS':>10}  {'INERTIA (diag)':>38}  {'CoM':>30}")
print("-" * 140)

no_mass  = []
no_rigid = []

for prim in stage.Traverse():
    if prim.GetTypeName() not in ("Xform",):
        continue

    path = str(prim.GetPath())
    name = path.split("/")[-1]

    if any(k in name for k in SKIP):
        continue

    # RigidBodyAPI 확인
    has_rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)

    # MassAPI 확인
    has_mass = prim.HasAPI(UsdPhysics.MassAPI)

    if not has_rigid and not has_mass:
        no_rigid.append(path)
        continue

    mass_val    = "N/A"
    inertia_val = "N/A"
    com_val     = "N/A"

    if has_mass:
        mass_api = UsdPhysics.MassAPI(prim)

        m = mass_api.GetMassAttr().Get()
        if m is not None and m > 0:
            mass_val = f"{m:.6f} kg"
        else:
            d = mass_api.GetDensityAttr().Get()
            mass_val = f"density={d}" if d else "0 / unset"

        inertia = mass_api.GetDiagonalInertiaAttr().Get()
        if inertia:
            inertia_val = f"[{inertia[0]:.4e}, {inertia[1]:.4e}, {inertia[2]:.4e}]"

        com = mass_api.GetCenterOfMassAttr().Get()
        if com:
            com_val = f"({com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f})"
    else:
        no_mass.append(path)

    rigid_mark = "[RB]" if has_rigid else "    "
    print(f"{rigid_mark} {path:<51} {mass_val:>14}  {inertia_val:>38}  {com_val:>30}")

# ── 요약 ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print(f"RigidBody 없는 링크 ({len(no_rigid)}개) - physics 계산 제외됨:")
for p in no_rigid:
    print(f"  {p}")

print(f"\nMassAPI 없는 RigidBody 링크 ({len(no_mass)}개) - 엔진이 자동 계산:")
for p in no_mass:
    print(f"  {p}")
