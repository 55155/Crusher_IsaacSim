"""
각 링크의 STL 체적을 계산하고, USD mass와 비교해 사용된 밀도를 역산합니다.
python check_density.py 로 독립 실행
"""

import os
import trimesh
from pxr import Usd, UsdPhysics

USD_PATH  = "C:/TEMP/Crusher_IsaacSim_description/Crushing.usd"
MESH_DIR  = "C:/TEMP/Crusher_IsaacSim_description/meshes"
ROBOT_PATH = "/World/Crusher_IsaacSim"

# 참조 밀도 (kg/m³)
MATERIALS = {
    "Steel (일반강)":      7850,
    "Stainless Steel":    7900,
    "Aluminum 6061":      2700,
    "Aluminum (일반)":    2710,
    "Cast Iron":          7200,
    "Titanium":           4500,
    "PLA (3D프린트)":      1240,
}

# ── USD에서 mass 불러오기 ──────────────────────────────────────────────────────
stage = Usd.Stage.Open(USD_PATH)

usd_mass = {}
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.MassAPI):
        continue
    api = UsdPhysics.MassAPI(prim)
    m = api.GetMassAttr().Get()
    if m and m > 0:
        link_name = str(prim.GetPath()).split("/")[-1]
        usd_mass[link_name] = m

# ── STL 파일명 → 링크명 매핑 ─────────────────────────────────────────────────
# meshes 파일명: "1_Braket1_1.stl" ↔ link: "L1_Braket1_1"
def stl_to_link(filename):
    base = os.path.splitext(filename)[0]          # "1_Braket1_1"
    parts = base.split("_", 1)
    if parts[0].isdigit():
        return "L" + base                         # "L1_Braket1_1"
    return base                                   # "base_link"

# ── 체적 계산 및 밀도 역산 ────────────────────────────────────────────────────
print(f"{'LINK':<35} {'체적(cm³)':>12} {'USD질량(g)':>12} {'역산밀도(kg/m³)':>16}  추정재질")
print("-" * 105)

densities = []
total_volume_cm3 = 0
total_mass_kg    = 0

stl_files = sorted(os.listdir(MESH_DIR))
for fname in stl_files:
    if not fname.endswith(".stl"):
        continue

    link_name = stl_to_link(fname)
    stl_path  = os.path.join(MESH_DIR, fname)

    mesh = trimesh.load_mesh(stl_path)
    if not mesh.is_watertight:
        mesh = trimesh.convex.convex_hull(mesh)   # watertight 아니면 convex hull로 근사

    volume_m3  = abs(mesh.volume)                 # m³ (USD 단위계: m)
    volume_cm3 = volume_m3 * 1e6

    mass_kg = usd_mass.get(link_name, None)

    if mass_kg and volume_m3 > 1e-12:
        density = mass_kg / volume_m3
        densities.append(density)
        total_volume_cm3 += volume_cm3
        total_mass_kg    += mass_kg

        # 가장 가까운 재질 찾기
        closest = min(MATERIALS, key=lambda k: abs(MATERIALS[k] - density))
        diff_pct = abs(MATERIALS[closest] - density) / MATERIALS[closest] * 100

        flag = "  ← 확인필요" if diff_pct > 30 else ""
        print(f"{link_name:<35} {volume_cm3:>12.4f} {mass_kg*1000:>12.2f} {density:>16.1f}  {closest} ({diff_pct:.0f}%){flag}")
    else:
        print(f"{link_name:<35} {volume_cm3:>12.4f} {'N/A':>12} {'N/A':>16}")

# ── 요약 ──────────────────────────────────────────────────────────────────────
print("\n" + "=" * 105)
if densities:
    import statistics
    median_d = statistics.median(densities)
    mean_d   = statistics.mean(densities)
    print(f"  역산 밀도 중앙값 : {median_d:,.1f} kg/m³")
    print(f"  역산 밀도 평균값 : {mean_d:,.1f} kg/m³")
    print(f"  총 메쉬 체적     : {total_volume_cm3:.2f} cm³")
    print(f"  USD 총 질량      : {total_mass_kg*1000:.1f} g  ({total_mass_kg:.4f} kg)")
    print()
    print("  [ 참조 밀도 ]")
    for mat, d in MATERIALS.items():
        diff = abs(d - median_d) / d * 100
        marker = "  ◀ 가장 근접" if diff < 5 else ""
        print(f"    {mat:<22}: {d:>6} kg/m³  (중앙값과 {diff:.1f}% 차이){marker}")
