"""
launch_mujoco.py  —  선택한 정제(Tablet) STL 을 MuJoCo 뷰어에서 엽니다.
===========================================================================
viewer_desktop.py 의 [MuJoCo에서 열기] 버튼에 의해 subprocess 로 호출됩니다.
별도 창이 뜨므로, PyVista 뷰어가 멈추지 않습니다.

직접 실행:
    python launch_mujoco.py <stl_path>
    python launch_mujoco.py            (인자 없으면 파일 선택 다이얼로그)

의존성:
    pip install mujoco
"""

import os
import sys
import argparse

# ── 경로 설정 ────────────────────────────────────────────────────────
_HERE   = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.normpath(os.path.join(_HERE, "..", "stl"))

# ── MuJoCo 씬 XML 템플릿 ─────────────────────────────────────────────
#
# STL 단위: mm (Fusion 360 export 기본) → scale=".01 .01 .01" 으로 m 변환 후 × 10 확대 (시각 확인용)
#
# 물성치:
#   density  1200 kg/m³  — 일반 경구정(압축정) 밀도
#   friction  0.5 / 0.02 / 0.01  — 알약↔바닥 접촉 마찰
#   condim    4  — 타원형 접촉면에 적합한 조건 (미끄러짐 방향 2개)
#
# 씬 구성:
#   ① 격자 바닥 (plane)
#   ② 정제 본체 (freejoint → 중력으로 낙하·정착)
#   ③ 조명 2개 (key + fill)
#   ④ 스카이박스 그라디언트
# ────────────────────────────────────────────────────────────────────
_SCENE_XML = """\
<mujoco model="tablet_preview">

  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" iterations="50"/>

  <visual>
    <headlight ambient=".35 .35 .35" diffuse=".9 .9 .9" specular=".1 .1 .1"/>
    <rgba haze=".15 .25 .35 1"/>
    <map shadowclip=".5" shadowscale=".6"/>
  </visual>

  <asset>
    <texture name="skybox" type="skybox" builtin="gradient"
             rgb1=".3 .5 .7" rgb2="0 0 0" width="512" height="3072"/>
    <texture name="grid" type="2d" builtin="checker" mark="edge"
             rgb1=".22 .22 .22" rgb2=".30 .30 .30" markrgb=".55 .55 .55"
             width="300" height="300"/>
    <material name="grid"   texture="grid" texuniform="true"
              texrepeat="4 4" reflectance=".15"/>
    <material name="tablet" rgba=".85 .80 .72 1" specular=".4" shininess=".3"/>

    <!-- STL 단위 mm → m 변환 후 × 10 확대 (시각 확인용) -->
    <mesh name="tablet_mesh" file="tablet.stl" scale=".01 .01 .01"/>
  </asset>

  <default>
    <geom friction=".5 .02 .01" solimp=".9 .95 .001" solref=".02 1"/>
  </default>

  <worldbody>
    <!-- 주 조명: 왼쪽 위 →  선명한 그림자 -->
    <light name="key" pos="-.3 -.5 1.2" dir=".2 .4 -1"
           directional="true" diffuse=".85 .85 .85" specular=".3 .3 .3"/>
    <!-- 보조 조명: 오른쪽 → 완전 검정 방지 -->
    <light name="fill" pos=".4 .3 .8"
           diffuse=".35 .38 .42" specular=".05 .05 .05"/>

    <!-- 바닥 -->
    <geom name="floor" type="plane" size="0 0 .05"
          material="grid" condim="3"/>

    <!-- 정제: (0, 0, 5) 고정 배치 (freejoint 제거 → 중력 영향 없음) -->
    <body name="tablet" pos="0 0 5">
      <geom name="tablet_geom" type="mesh" mesh="tablet_mesh"
            material="tablet" density="1200" condim="4"/>
{com_sites}    </body>
  </worldbody>

</mujoco>
"""


def launch(stl_path: str):
    """STL 파일을 로드하여 MuJoCo 뷰어를 실행합니다."""
    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print("[ERROR] mujoco 패키지가 설치되지 않았습니다.")
        print("        pip install mujoco")
        sys.exit(1)

    stl_path = os.path.abspath(stl_path)
    if not os.path.exists(stl_path):
        print(f"[ERROR] STL 파일을 찾을 수 없습니다:\n        {stl_path}")
        sys.exit(1)

    fname = os.path.basename(stl_path)
    print(f"[Tablet → MuJoCo]")
    print(f"  파일   : {fname}")

    import numpy as np

    # STL 바이너리를 assets 딕셔너리로 전달 → 임시 파일 없이 로드
    stl_bytes = open(stl_path, "rb").read()
    assets    = {"tablet.stl": stl_bytes}

    # ── Pass 1: COM 계산 ─────────────────────────────────────────────
    # site 없이 일단 로드 → body_ipos(body local frame 질량 중심) 추출
    try:
        model_tmp = mujoco.MjModel.from_xml_string(
            _SCENE_XML.format(com_sites=""), assets=assets)
    except Exception as e:
        print(f"[ERROR] MJCF 생성 실패: {e}")
        sys.exit(1)

    body_id  = mujoco.mj_name2id(model_tmp, mujoco.mjtObj.mjOBJ_BODY, "tablet")
    com_loc  = model_tmp.body_ipos[body_id].copy()   # body local frame COM
    body_pos = model_tmp.body_pos[body_id].copy()     # world pos of body origin
    com_world = body_pos + com_loc

    print(f"  [질량 중심] body frame : ({com_loc[0]:+.4f}, {com_loc[1]:+.4f}, {com_loc[2]:+.4f}) m")
    print(f"  [질량 중심] world      : ({com_world[0]:+.4f}, {com_world[1]:+.4f}, {com_world[2]:+.4f}) m")
    total_mass = float(np.sum(model_tmp.body_mass))
    print(f"  [질량]      {total_mass*1000:.2f} g  (density=1200 kg/m³, ×10 scale 포함)")

    # ── COM 크로스헤어 site XML 생성 ────────────────────────────────
    # 어느 각도에서도 반드시 하나의 축이 mesh 밖으로 보이도록
    # 반길이 0.25 m → 최대 정제(R=8.5mm × 10 × AR=2.5 → 약 21 cm) 보다 충분히 큼
    cx, cy, cz = com_loc
    hl = 0.25   # 크로스헤어 반길이 [m]
    sr = 0.018  # 중심 구 반지름 [m]
    cr = 0.005  # 캡슐 반지름 [m]

    com_sites = f"""\
      <!-- ── COM 크로스헤어 (질량 중심 기즈모) ── -->
      <!-- 중심 : 주황색 구 -->
      <site name="com_sphere"
            type="sphere" size="{sr}"
            pos="{cx:.5f} {cy:.5f} {cz:.5f}"
            rgba="1.0 0.45 0.0 1.0"/>
      <!-- X축 : 빨강 캡슐 -->
      <site name="com_x" type="capsule" size="{cr}"
            fromto="{cx-hl:.5f} {cy:.5f} {cz:.5f}  {cx+hl:.5f} {cy:.5f} {cz:.5f}"
            rgba="1.0 0.15 0.15 0.95"/>
      <!-- Y축 : 초록 캡슐 -->
      <site name="com_y" type="capsule" size="{cr}"
            fromto="{cx:.5f} {cy-hl:.5f} {cz:.5f}  {cx:.5f} {cy+hl:.5f} {cz:.5f}"
            rgba="0.15 1.0 0.15 0.95"/>
      <!-- Z축 : 파랑 캡슐 -->
      <site name="com_z" type="capsule" size="{cr}"
            fromto="{cx:.5f} {cy:.5f} {cz-hl:.5f}  {cx:.5f} {cy:.5f} {cz+hl:.5f}"
            rgba="0.15 0.4 1.0 0.95"/>
"""

    # ── Pass 2: COM site 포함하여 실제 모델 로드 ────────────────────
    try:
        model = mujoco.MjModel.from_xml_string(
            _SCENE_XML.format(com_sites=com_sites), assets=assets)
    except Exception as e:
        print(f"[ERROR] MJCF(COM site 포함) 생성 실패: {e}")
        sys.exit(1)

    data = mujoco.MjData(model)

    # 파라미터 파싱해서 출력 (파일명에서 추출)
    _print_params(fname)

    print(f"  조작   : Space=일시정지  ESC=종료  Ctrl+R=리셋")
    print()

    # ── 뷰어 실행 ───────────────────────────────────────────────────
    with mujoco.viewer.launch_passive(model, data) as viewer:

        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT]      = False  # 조인트 마커 숨김
        viewer.opt.frame                                      = mujoco.mjtFrame.mjFRAME_NONE
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONVEXHULL] = False  # collision hull 숨김
        # site 표시: sitegroup[n] 으로 제어 (mjVIS_SITE 없음)
        for _sg in range(5):
            viewer.opt.sitegroup[_sg] = True

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


def _print_params(fname: str):
    """파일명에서 R/AR/CV 파라미터를 파싱해 출력합니다."""
    import re
    # 확장자 제거 후 파싱 → .stl이 CV 값에 붙는 문제 방지
    stem = os.path.splitext(fname)[0]
    m = re.search(r"R([\d.]+)_AR([\d.]+)_CV([\d.]+)", stem)
    if not m:
        return
    R, AR, CV = float(m.group(1)), float(m.group(2)), float(m.group(3))
    cd = CV * 2 * R
    bh = R * 0.20
    th = bh + 2 * cd
    Rs = (R*R + cd*cd) / (2*cd)
    print(f"  단반경 : {R:.1f} mm   (직경 {R*2:.1f} mm)")
    print(f"  형태비 : {AR:.2f}   (장축 직경 {R*2*AR:.1f} mm)")
    print(f"  곡률   : {CV:.2f}   (구면 반경 {Rs:.1f} mm)")
    print(f"  두께   : {th:.2f} mm")


# ── 진입점 ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Tablet STL → MuJoCo 뷰어",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예시:\n  python launch_mujoco.py tablet_R6.0_AR1.50_CV0.20.stl"
    )
    parser.add_argument("stl", nargs="?", default=None, help="STL 파일 경로")
    args = parser.parse_args()

    stl_path = args.stl

    # 인자 없으면 파일 선택 다이얼로그
    if stl_path is None:
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk()
            root.withdraw()
            stl_path = filedialog.askopenfilename(
                title="Tablet STL 선택",
                initialdir=STL_DIR if os.path.isdir(STL_DIR) else os.path.expanduser("~"),
                filetypes=[("STL Files", "*.stl")]
            )
            root.destroy()
        except Exception:
            pass

    if not stl_path:
        print("사용법: python launch_mujoco.py <path>.stl")
        sys.exit(0)

    launch(stl_path)
