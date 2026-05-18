"""
make_mesh_csv.py
================
링크별 Material 정보 + 재료 물성값(specular/shininess/reflectance/emission) CSV 생성.
rgba(r,g,b,a) 는 사용자가 직접 채워넣는 칸으로 비워둠.

물성값 참고:
  - MuJoCo Material 문서 (Phong shading 기반)
  - NDT Resource Center, MatWeb 금속 광학 특성
  - Phong model 근사: shininess = (1 - roughness)^2 * scale
"""

import csv
from pathlib import Path
from pxr import Usd, UsdShade

USD_PATH = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\Crushing_CollisionFree.usd"
OUT_CSV  = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260518\mesh_colors.csv"

# ── 재료별 물성 정의 ──────────────────────────────────────────────────────────
# MuJoCo material 속성:
#   specular    : 반사 강도  (Phong specular coefficient, 0~1)
#   shininess   : 광택       (Phong shininess, 0~1 → 내부적으로 128배)
#   reflectance : 반사율     (거울 반사 비율, 0~1)
#   emission    : 자체 발광  (0~1, 대부분 0)
#
# 근거:
#   주철(Iron Cast)       : 거친 표면, 낮은 반사. roughness≈0.8 → shininess≈0.2
#   알루미늄 Scratched    : 스크래치로 반사 분산. roughness≈0.5 → shininess≈0.5
#   알루미늄 Brushed      : 방향성 광택. roughness≈0.4 → shininess≈0.6
#   알루미늄 Clean        : 깨끗한 금속. roughness≈0.2 → shininess≈0.8
#   철 Brushed            : 브러시 처리 철. roughness≈0.5
#   Plastic ABS           : 무광 플라스틱. roughness≈0.6, 반사율 낮음
#   Paper                 : 완전 무광. roughness≈0.9
#   Black Metal           : 검은 금속. 색은 어둡지만 반사는 높음

MATERIAL_PROPS = {
    # ── 주철 계열 ──────────────────────────────────────────────────────────────
    # 참고: cast iron 표면 반사율 약 5~10% (비금속 표면 처리)
    "Iron_Cast":      dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_01":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_02":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_03":   dict(specular=0.25, shininess=0.15, reflectance=0.06, emission=0.0),
    "Iron_Cast_04":   dict(specular=0.25, shininess=0.15, reflectance=0.06, emission=0.0),
    "Iron_Cast_05":   dict(specular=0.25, shininess=0.15, reflectance=0.06, emission=0.0),
    "Iron_Cast_06":   dict(specular=0.25, shininess=0.15, reflectance=0.06, emission=0.0),
    "Iron_Cast_07":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_08":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_09":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_10":   dict(specular=0.3,  shininess=0.2,  reflectance=0.08, emission=0.0),
    "Iron_Cast_11":   dict(specular=0.28, shininess=0.18, reflectance=0.07, emission=0.0),
    "Iron_Cast_12":   dict(specular=0.28, shininess=0.18, reflectance=0.07, emission=0.0),
    "Iron_Cast_13":   dict(specular=0.28, shininess=0.18, reflectance=0.07, emission=0.0),
    "Iron_Cast_14":   dict(specular=0.28, shininess=0.18, reflectance=0.07, emission=0.0),
    "Iron_Cast_15":   dict(specular=0.28, shininess=0.18, reflectance=0.07, emission=0.0),

    # ── 철 Brushed 계열 ────────────────────────────────────────────────────────
    # 브러시 처리 → 방향성 산란, 주철보다 약간 높은 반사
    "Iron_Brushed_02": dict(specular=0.45, shininess=0.35, reflectance=0.15, emission=0.0),

    # ── 알루미늄 Scratched 계열 ────────────────────────────────────────────────
    # 알루미늄 반사율 약 80~90% (순수), 스크래치로 분산
    # roughness≈0.5 적용 → 반사 강도 중간
    "Aluminum_Scratched":    dict(specular=0.7, shininess=0.45, reflectance=0.35, emission=0.0),
    "Aluminum_Scratched_01": dict(specular=0.65, shininess=0.4, reflectance=0.3,  emission=0.0),
    "Aluminum_Scratched_02": dict(specular=0.65, shininess=0.4, reflectance=0.3,  emission=0.0),

    # ── 알루미늄 Brushed 계열 ──────────────────────────────────────────────────
    # 브러시 처리 알루미늄: 방향성 광택, roughness≈0.35
    "Aluminum_Brushed":    dict(specular=0.8,  shininess=0.6,  reflectance=0.45, emission=0.0),
    "Aluminum_Brushed_01": dict(specular=0.78, shininess=0.58, reflectance=0.42, emission=0.0),

    # ── 알루미늄 Clean ────────────────────────────────────────────────────────
    # 처리 없는 알루미늄, roughness≈0.2
    "Aluminum_01": dict(specular=0.9, shininess=0.75, reflectance=0.55, emission=0.0),

    # ── Plastic ABS ───────────────────────────────────────────────────────────
    # ABS 플라스틱: 무광~반광, 금속 반사 없음
    # 참고: 플라스틱 반사율 약 4~5% (non-metallic)
    "Plastic_ABS":    dict(specular=0.3, shininess=0.4, reflectance=0.0, emission=0.0),
    "Plastic_ABS_01": dict(specular=0.3, shininess=0.4, reflectance=0.0, emission=0.0),
    "Plastic_ABS_02": dict(specular=0.25, shininess=0.35, reflectance=0.0, emission=0.0),

    # ── Paper ─────────────────────────────────────────────────────────────────
    # 완전 무광, 난반사만 존재
    "Paper_01": dict(specular=0.05, shininess=0.05, reflectance=0.0, emission=0.0),

    # ── Black Metal ───────────────────────────────────────────────────────────
    # 검은 금속 코팅: 색은 어둡지만 금속 광택 유지
    "BlackMetal":   dict(specular=0.8, shininess=0.6, reflectance=0.4, emission=0.0),
    "BlackMaterial": dict(specular=0.2, shininess=0.2, reflectance=0.05, emission=0.0),
}

DEFAULT_PROPS = dict(specular=0.3, shininess=0.3, reflectance=0.1, emission=0.0)

# ── USD에서 링크-Material 바인딩 수집 ────────────────────────────────────────
stage = Usd.Stage.Open(USD_PATH)
seen  = {}

for prim in stage.Traverse():
    if prim.GetName() != "mesh":
        continue
    api = UsdShade.MaterialBindingAPI(prim)
    mat, _ = api.ComputeBoundMaterial()
    if not mat:
        continue
    mat_name = mat.GetPrim().GetName()
    if mat_name == "BlackMaterial":
        continue

    parts = str(prim.GetPath()).split("/")
    try:
        link = parts[parts.index("Crusher_IsaacSim") + 1]
    except (ValueError, IndexError):
        continue

    seen[link] = mat_name

# ── CSV 생성 ─────────────────────────────────────────────────────────────────
rows = []
for link, mat_name in sorted(seen.items()):
    props = MATERIAL_PROPS.get(mat_name, DEFAULT_PROPS)
    rows.append({
        "link":        link,
        "material":    mat_name,
        "r":           "",
        "g":           "",
        "b":           "",
        "a":           "1.0",
        "specular":    props["specular"],
        "shininess":   props["shininess"],
        "reflectance": props["reflectance"],
        "emission":    props["emission"],
    })

fields = ["link","material","r","g","b","a","specular","shininess","reflectance","emission"]
with open(OUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)

print(f"{len(rows)}개 링크 → {OUT_CSV}")
print()
print(f"{'링크':<35} {'Material':<25} spec   shin   refl   emit")
print("-" * 85)
for r in rows:
    print(f"  {r['link']:<33} {r['material']:<25}"
          f" {r['specular']:<6} {r['shininess']:<6} {r['reflectance']:<6} {r['emission']}")
