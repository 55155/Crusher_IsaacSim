"""
Inverse Design: STL 메쉬에서 mass / inertia 역계산 후 USD에 덮어쓰기
  - STL 단위: mm  →  m 변환 후 계산
  - 기준 밀도: 7850 kg/m³ (Steel)
  - 계산값을 USD MassAPI에 기록

python inverse_design.py              → 계산 결과 출력만 (dry-run)
python inverse_design.py --write      → USD 파일에 실제로 덮어쓰기
"""

import sys
import os
import trimesh
import numpy as np
from pxr import Usd, UsdPhysics, Gf

# ── 설정 ──────────────────────────────────────────────────────────────────────
USD_PATH  = "C:/TEMP/Crusher_IsaacSim_description/Crushing.usd"
MESH_DIR  = "C:/TEMP/Crusher_IsaacSim_description/meshes"
DENSITY   = 7850.0      # kg/m³  (Steel 기본값, 필요시 변경)
MM_TO_M   = 1e-3        # STL 단위 mm → m 변환 계수
WRITE_USD = "--write" in sys.argv

# ── 링크명 매핑 (meshes/1_Braket1_1.stl → L1_Braket1_1) ──────────────────────
def stl_to_link(filename):
    base = os.path.splitext(filename)[0]
    parts = base.split("_", 1)
    return ("L" + base) if parts[0].isdigit() else base

# ── USD 로드 ──────────────────────────────────────────────────────────────────
stage = Usd.Stage.Open(USD_PATH)

# ── 현재 USD mass 로드 (비교용) ───────────────────────────────────────────────
usd_mass = {}
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.MassAPI):
        api = UsdPhysics.MassAPI(prim)
        m = api.GetMassAttr().Get()
        if m and m > 0:
            usd_mass[str(prim.GetPath()).split("/")[-1]] = m

# ── 역계산 루프 ───────────────────────────────────────────────────────────────
print(f"밀도 기준: {DENSITY} kg/m³  |  {'[DRY-RUN]' if not WRITE_USD else '[USD 덮어쓰기]'}")
print(f"{'LINK':<35} {'체적(cm³)':>10} {'질량(g)':>9} {'Ixx':>12} {'Iyy':>12} {'Izz':>12}  {'USD질량(g)':>10}  비고")
print("-" * 130)

results = {}
stl_files = sorted(os.listdir(MESH_DIR))

for fname in stl_files:
    if not fname.endswith(".stl"):
        continue

    link_name = stl_to_link(fname)
    stl_path  = os.path.join(MESH_DIR, fname)

    # ── 메쉬 로드 및 단위 변환 (mm → m) ──────────────────────────────────────
    mesh = trimesh.load_mesh(stl_path)
    mesh.apply_scale(MM_TO_M)          # 좌표계를 m 로 변환

    if not mesh.is_watertight:
        mesh = trimesh.convex.convex_hull(mesh)
        note = "(convex hull)"
    else:
        note = ""

    volume_m3  = abs(mesh.volume)
    volume_cm3 = volume_m3 * 1e6

    # ── 질량 계산 ─────────────────────────────────────────────────────────────
    mass_kg = DENSITY * volume_m3

    # ── 관성 텐서 계산 (CoM 기준, 단위 kg·m²) ────────────────────────────────
    # trimesh.moment_inertia = 메쉬 원점 기준 → CoM으로 평행이동 변환 적용
    mesh.density = DENSITY
    inertia_3x3 = mesh.moment_inertia   # kg·m², CoM 기준 3×3 텐서

    ixx = inertia_3x3[0, 0]
    iyy = inertia_3x3[1, 1]
    izz = inertia_3x3[2, 2]

    com_m = mesh.center_mass            # m 단위 CoM

    usd_m = usd_mass.get(link_name, None)
    usd_str = f"{usd_m*1000:.2f}" if usd_m else "N/A"
    diff    = f"(Δ{(mass_kg-usd_m)/usd_m*100:+.0f}%)" if usd_m else ""

    print(f"{link_name:<35} {volume_cm3:>10.3f} {mass_kg*1000:>9.2f} "
          f"{ixx:>12.4e} {iyy:>12.4e} {izz:>12.4e}  {usd_str:>10}  {diff} {note}")

    results[link_name] = {
        "mass_kg":  mass_kg,
        "inertia":  Gf.Vec3f(float(ixx), float(iyy), float(izz)),
        "com":      Gf.Vec3f(*[float(v) for v in com_m]),
    }

# ── 요약 ──────────────────────────────────────────────────────────────────────
total_mass = sum(r["mass_kg"] for r in results.values())
print("-" * 130)
print(f"{'총 계산 질량':<35} {'':>10} {total_mass*1000:>9.2f} g  =  {total_mass:.4f} kg")

# ── USD 쓰기 ──────────────────────────────────────────────────────────────────
if WRITE_USD:
    print("\nUSD 업데이트 중...")
    updated = 0
    for prim in stage.Traverse():
        link_name = str(prim.GetPath()).split("/")[-1]
        if link_name not in results:
            continue

        r = results[link_name]

        # MassAPI가 없으면 Apply
        if not prim.HasAPI(UsdPhysics.MassAPI):
            UsdPhysics.MassAPI.Apply(prim)

        api = UsdPhysics.MassAPI(prim)
        api.GetMassAttr().Set(float(r["mass_kg"]))
        api.GetDiagonalInertiaAttr().Set(r["inertia"])
        api.GetCenterOfMassAttr().Set(r["com"])
        updated += 1
        print(f"  updated: {prim.GetPath()}")

    stage.GetRootLayer().Save()
    print(f"\n완료: {updated}개 링크 업데이트 → {USD_PATH}")
else:
    print(f"\n[DRY-RUN] USD 파일은 변경되지 않았습니다.")
    print(f"실제 적용하려면: python inverse_design.py --write")
