"""
Read_Collision.py  -  Isaac Sim Script Editor / 독립 실행 모두 가능
================================================================
Crushing_CollisionFree.usd 내 모든 Collision 설정을 한눈에 출력합니다.

출력 항목:
  - collisionEnabled  (True / False)
  - approximation     (convexHull / convexDecomposition / meshSimplification 등)
  - 소속 링크 이름
"""

import os, csv

USD_PATH = r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\Crushing_CollisionFree.usd"
CSV_PATH = r"C:\Crusher_isaacsim\20260513\Collision_output.csv"

from pxr import Usd, UsdPhysics

# ── Stage 열기 ────────────────────────────────────────────────────────────────
try:
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    print("[모드] Isaac Sim Script Editor")
except ImportError:
    stage = Usd.Stage.Open(USD_PATH)
    print("[모드] 독립 실행")

assert stage

# ── CollisionAPI 적용된 prim 수집 ─────────────────────────────────────────────
rows = []
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue

    enabled = prim.GetAttribute("physics:collisionEnabled").Get()
    approx  = prim.GetAttribute("physics:approximation").Get()

    # 소속 링크: path 에서 /World/Crusher_IsaacSim/<link> 추출
    parts = str(prim.GetPath()).split("/")
    link  = parts[3] if len(parts) > 3 else "—"

    rows.append({
        "link"    : link,
        "prim"    : prim.GetName(),
        "path"    : str(prim.GetPath()),
        "enabled" : enabled,
        "approx"  : approx or "—",
    })

# ── 정렬: enabled=True 먼저, 그 다음 link 이름 순 ────────────────────────────
rows.sort(key=lambda r: (not r["enabled"], r["link"]))

# ── 콘솔 출력 ─────────────────────────────────────────────────────────────────
enabled_rows  = [r for r in rows if r["enabled"]]
disabled_rows = [r for r in rows if not r["enabled"]]

W_LINK  = max(len(r["link"])  for r in rows) + 2 if rows else 20
W_PRIM  = max(len(r["prim"])  for r in rows) + 2 if rows else 30
W_APPX  = max(len(r["approx"]) for r in rows) + 2 if rows else 20

header = (f"  {'링크':<{W_LINK}}  {'프림':<{W_PRIM}}  {'approximation':<{W_APPX}}")
sep    = "  " + "-" * (W_LINK + W_PRIM + W_APPX + 6)

print()
print("=" * 70)
print(f"  CollisionEnabled = True  ({len(enabled_rows)}개)")
print("=" * 70)
print(header)
print(sep)
for r in enabled_rows:
    print(f"  {r['link']:<{W_LINK}}  {r['prim']:<{W_PRIM}}  {r['approx']:<{W_APPX}}")

print()
print("=" * 70)
print(f"  CollisionEnabled = False  ({len(disabled_rows)}개)")
print("=" * 70)
print(header)
print(sep)
for r in disabled_rows:
    print(f"  {r['link']:<{W_LINK}}  {r['prim']:<{W_PRIM}}  {r['approx']:<{W_APPX}}")

print()
print(f"  합계: 전체 {len(rows)}개  |  활성 {len(enabled_rows)}개  |  비활성 {len(disabled_rows)}개")

# ── CSV 저장 ──────────────────────────────────────────────────────────────────
with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.DictWriter(f, fieldnames=["link","prim","enabled","approx","path"])
    w.writeheader()
    w.writerows(rows)

print(f"\n  → CSV: {CSV_PATH}")
print("=" * 70)
