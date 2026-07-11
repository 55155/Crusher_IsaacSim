"""
paths.py — 중앙 경로 해석기 (single source of truth for filesystem paths).

config.json(경로 원장)을 읽어 프로젝트 루트(Crusher_Genesis/) 기준 상대경로를
절대경로로 변환한다. 스크립트는 asset 위치를 `__file__` 로 직접 계산하지 않고
여기서 가져온다 → 스크립트를 하위 폴더로 옮겨도 경로가 깨지지 않는다.

사용법
------
스크립트 상단에 아래 부트스트랩을 넣으면 위치에 무관하게 import 된다:

    import os, sys
    _r = os.path.dirname(os.path.abspath(__file__))
    while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
        _r = os.path.dirname(_r)
    sys.path.insert(0, _r)
    import paths

그 뒤:
    xml = paths.MJCF_MAIN                          # Crusher MJCF (절대경로)
    robot = paths.ROBOT_M0609_RG2                  # m0609_rg2.xml
    out = paths.SIM_RESULT                         # Sim_result/ (없으면 생성됨)
    stl = paths.asset("MJCF", "L1_Wall1_1.stl")    # assets/ 하위 임의 경로
"""
import json
import os

# paths.py 자신이 프로젝트 루트에 위치 → 자기 위치를 루트 앵커로 사용.
ROOT = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(ROOT, "config.json"), encoding="utf-8") as _f:
    _CFG = json.load(_f)


def _abs(rel):
    return os.path.normpath(os.path.join(ROOT, rel))


def asset(*parts):
    """assets/ 하위 경로를 절대경로로. 예: asset('robots', 'assets', 'aluminum_plate.stl')."""
    return os.path.join(ASSETS_DIR, *parts)


ASSETS_DIR      = _abs(_CFG["assets_dir"])
MJCF_DIR        = _abs(_CFG["mjcf_dir"])
MJCF_MAIN       = _abs(_CFG["mjcf_main"])
ROBOTS_DIR      = _abs(_CFG["robots_dir"])
ROBOT_M0609_RG2 = _abs(_CFG["robot_m0609_rg2"])
ALUMINUM_PLATE  = _abs(_CFG["aluminum_plate"])
TABLETS_STL     = _abs(_CFG["tablets_stl"])
SIM_RESULT      = _abs(_CFG["sim_result"])
REAL_RESULT     = _abs(_CFG["real_result"])

# 출력 폴더는 없으면 생성 (스크립트마다 makedirs 반복 안 하도록)
for _d in (SIM_RESULT,):
    os.makedirs(_d, exist_ok=True)
