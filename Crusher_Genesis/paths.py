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


def ascii_safe_mjcf(path):
    """mujoco.MjModel.from_xml_path 가 이 머신에서 비-ASCII(한글 등) 경로를 못 여는
    버그 우회(격리 테스트로 확인, 2026-07-27 — narrow fopen 이 UTF-8 바이트를
    시스템 ANSI 코드페이지로 잘못 재해석해 생기는 것으로 추정). 비-ASCII 경로일
    때만 그 디렉토리 전체(형제 meshes/ 등 상대참조 유지 위해)를 tempdir 밑
    ASCII 이름 캐시로 미러링해 거기서 로드하게 한다 — 원본 assets/ 디렉토리명은
    그대로 두어 이 문제가 없는 다른 머신과의 호환성을 유지한다.
    """
    if path.isascii():
        return path
    import hashlib
    import shutil
    import tempfile

    src_dir = os.path.dirname(path)
    fname = os.path.basename(path)
    cache_root = os.path.join(tempfile.gettempdir(), "crusher_ascii_mjcf_cache")
    key = hashlib.md5(os.path.abspath(src_dir).encode("utf-8")).hexdigest()[:16]
    dst_dir = os.path.join(cache_root, key)
    # **디렉터리 존재 != 캐시 완전.** temp 청소가 파일만 지우고 디렉터리는 남기면
    # copytree 가 영구히 건너뛰어져 빠진 파일이 다시는 안 채워진다(실제로 최상위
    # 고정장치.xml 과 meshes/L1_1_ss.obj 가 사라진 캐시를 만나 매 실행 실패했다).
    # 그래서 항상 원본을 훑어 **빠진 것만** 채운다 — 있는 파일은 stat 만 하므로
    # 정상 캐시에서는 사실상 공짜다.
    if not os.path.isdir(dst_dir):
        shutil.copytree(src_dir, dst_dir)
    else:
        for root, _dirs, files in os.walk(src_dir):
            rel = os.path.relpath(root, src_dir)
            out = dst_dir if rel == os.curdir else os.path.join(dst_dir, rel)
            os.makedirs(out, exist_ok=True)
            for fn in files:
                if not os.path.isfile(os.path.join(out, fn)):
                    shutil.copy2(os.path.join(root, fn), os.path.join(out, fn))
    dst_fname = fname if fname.isascii() else hashlib.md5(fname.encode("utf-8")).hexdigest()[:12] + os.path.splitext(fname)[1]
    dst_path = os.path.join(dst_dir, dst_fname)
    if not os.path.isfile(dst_path):
        # 원본에서 가져온다(캐시에서 읽으면 위와 같은 이유로 막힌다). 비-ASCII
        # 경로를 못 여는 건 MuJoCo 의 narrow fopen 이지 파이썬이 아니다.
        shutil.copy2(os.path.join(src_dir, fname), dst_path)
    return dst_path


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
