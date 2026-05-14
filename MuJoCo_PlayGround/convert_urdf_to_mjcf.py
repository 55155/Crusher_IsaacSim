"""
convert_urdf_to_mjcf.py
=======================
Crusher_IsaacSim.urdf -> MJCF/Crusher_IsaacSim.xml
"""

import shutil
from pathlib import Path
import mujoco

URDF_PATH  = Path(r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
MJCF_DIR   = Path(r"C:\Crusher_isaacsim\MuJoCo_PlayGround\MJCF")
MESH_SRC   = Path(r"C:\Crusher_isaacsim\Crusher_IsaacSim_description\urdf\meshes")
OUT_XML    = MJCF_DIR / "Crusher_IsaacSim.xml"

# ── 1. STL 파일을 MJCF_DIR 바로 아래에 복사 ──────────────────────────────────
#     URDF의 mesh filename이 "xxx.stl" (경로 없음) 형태이므로
#     URDF와 같은 디렉토리에 STL이 있어야 MuJoCo가 찾을 수 있음
copied = 0
for stl in MESH_SRC.glob("*.stl"):
    shutil.copy2(stl, MJCF_DIR / stl.name)
    copied += 1
print(f"[1] STL {copied}개 복사 완료 → {MJCF_DIR}")

# ── 2. URDF 로드 (MuJoCo는 mesh 경로를 URDF 기준으로 찾음) ───────────────────
#     MuJoCo가 STL을 찾을 수 있도록 URDF 파일을 MJCF 폴더에 임시 복사
URDF_TMP = MJCF_DIR / URDF_PATH.name
shutil.copy2(URDF_PATH, URDF_TMP)

# URDF 안의 mesh 경로가 "meshes/xxx.stl" 형태여야 함
# 이미 MJCF_DIR/meshes/ 에 복사했으므로 경로 일치
print(f"[2] URDF 로드 중: {URDF_TMP}")
model = mujoco.MjModel.from_xml_path(str(URDF_TMP))
print(f"    로드 성공: {model.nbody} bodies, {model.njnt} joints, {model.ngeom} geoms")

# ── 3. MJCF 저장 ──────────────────────────────────────────────────────────────
mujoco.mj_saveLastXML(str(OUT_XML), model)
print(f"[3] MJCF 저장 완료 → {OUT_XML}")

# ── 4. 임시 URDF 삭제 ────────────────────────────────────────────────────────
URDF_TMP.unlink()
print("[4] 임시 URDF 삭제")

print(f"\n완료. 파일 목록:")
for f in sorted(MJCF_DIR.iterdir()):
    print(f"  {f.name}")
