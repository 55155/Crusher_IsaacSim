from __future__ import annotations
# =============================================================================
# tablet_generator_StadiumShape.py
# Fusion 360 Script — Stadium(스타디움/캡렛) 양볼록 정제 STL 파일 1,000개 배치 생성
# =============================================================================
#
# ⚠️  이 스크립트는 로컬 Python 환경에서 직접 실행할 수 없습니다.
#     반드시 Autodesk Fusion 360 내부의 스크립트 실행 환경에서만 동작합니다.
#     (adsk.core / adsk.fusion 모듈은 Fusion 360 전용 API입니다.)
#
# ─── Stadium(스타디움) 형태란? ────────────────────────────────────────────────
#
#  위에서 내려다본 평면 윤곽:
#    - 양쪽 끝: 반지름 R 의 반원
#    - 가운데:  폭 2R, 길이 2·half_s 의 직사각형
#    - 전체 길이 = 2R × AR,  half_s = R × (AR − 1)
#    - AR = 1.0 → 완전한 원 (half_s = 0),  AR = 2.5 → 길쭉한 캡렛
#
#  단면 두께 방향: 기존 biconvex 정제와 동일한 양볼록 프로파일 사용
#
# ─── 3-Body Boolean Union 생성 전략 ──────────────────────────────────────────
#
#  STEP A: 오른쪽 반원 캡 (round biconvex) → +half_s 방향(Z)으로 이동
#  STEP B: 왼쪽  반원 캡 (round biconvex) → -half_s 방향(Z)으로 이동
#  STEP C: 중간 직사각 구간
#          → XY 평면에 완전 폐쇄된 biconvex 렌즈 단면(타원형 윤곽) 스케치
#          → ±half_s 방향으로 대칭 돌출(Symmetric Extrude)
#  UNION: A + B + C → 하나의 연속 스타디움 바디
#
# ─── 사용 방법 ────────────────────────────────────────────────────────────────
#
#  STEP 1.  이 파일을 Fusion 360 Scripts 폴더에 복사합니다.
#
#           복사 위치 (Windows):
#             C:\Users\<사용자명>\AppData\Roaming\Autodesk\
#                 Autodesk Fusion 360\API\Scripts\tablet_generator_StadiumShape\
#
#           ※ 폴더명과 파일명이 반드시 일치해야 합니다.
#
#  STEP 2.  Fusion 360 실행 → 빈 디자인 문서 열기
#
#  STEP 3.  UTILITIES > Scripts and Add-ins (Shift+S)
#
#  STEP 4.  [Scripts] 탭 > [+] > 이 파일이 있는 폴더 선택
#
#  STEP 5.  "tablet_generator_StadiumShape" 선택 > [Run]
#
# ─── 출력 위치 ────────────────────────────────────────────────────────────────
#
#  ~/Desktop/Crusher_IsaacSim/tablets_stl/stl_stadium/
#
#  생성 파일 예시:
#    tablet_stadium_R4.0_AR1.00_CV0.08.stl
#    ...
#    tablet_stadium_R8.5_AR2.50_CV0.35.stl   (총 1,000개)
#    _tablet_stadium_index.csv                (파라미터 메타데이터)
#
# ─── 파라미터 설명 ────────────────────────────────────────────────────────────
#
#  R  (단반경, mm)  : 정제 단축 반경. 직경 = R × 2
#  AR (형태비)      : 장/단축 비율. 1.0 = 원형(원통형), 2.5 = 길쭉한 캡렛
#                     half_s = R × (AR − 1)  ← 직사각 구간 절반 길이
#  CV (곡률 비율)   : cup_depth / (2×R). 0.08 = 편평, 0.35 = 볼록
#
# =============================================================================

import adsk.core
import adsk.fusion
import traceback
import os
import csv
import math

# ─── 사용자 설정 ───────────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop",
                          "Crusher_IsaacSim", "tablets_stl", "stl_stadium")

# 각 파라미터 10단계 (총 10 × 10 × 10 = 1,000개)
RADII_MM = [4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5]
#  └ 단반경 (short-axis radius). 직경 = R × 2.

ASPECT_RATIOS = [1.0, 1.17, 1.33, 1.5, 1.67, 1.83, 2.0, 2.17, 2.33, 2.5]
#  └ 1.0 = 원형,  2.5 = 길쭉한 캡렛.
#    half_s (직사각 구간 절반 길이) = R × (AR − 1)

CURVATURES = [0.08, 0.11, 0.14, 0.17, 0.20, 0.23, 0.26, 0.29, 0.32, 0.35]
#  └ cup_depth / (2 × R). 0.08 ≈ 편평, 0.35 ≈ 볼록.

BAND_RATIO = 0.20
#  └ 밴드 높이 = r_cm × BAND_RATIO
# ──────────────────────────────────────────────────────────────────────────────


# ─── 헬퍼 함수 ────────────────────────────────────────────────────────────────

def _pt(x_cm: float, y_cm: float) -> adsk.core.Point3D:
    """
    스케치 로컬 좌표 헬퍼.
    스케치 평면: XY
      - 스케치 X → 반경 방향 (radial)
      - 스케치 Y → 축 방향 (axial / height)
    """
    return adsk.core.Point3D.create(x_cm, y_cm, 0.0)


def _compute_geometry(r_cm: float, cd_cm: float, bh_cm: float) -> tuple:
    """
    biconvex 단면의 핵심 기하 수치 계산.

    반환: (R_s, z_tc, z_tb, z_bb, z_bc, z_sc_top, z_sc_bot)
      R_s      : 구면 반경 (cm)
      z_tc     : 상면 apex Y 좌표
      z_tb     : 상면 밴드 경계 Y 좌표
      z_bb     : 하면 밴드 경계 Y 좌표
      z_bc     : 하면 apex Y 좌표
      z_sc_top : 상면 구면 중심 Y 좌표
      z_sc_bot : 하면 구면 중심 Y 좌표
    """
    R_s      = (r_cm**2 + cd_cm**2) / (2.0 * cd_cm)
    z_tc     =  bh_cm / 2.0 + cd_cm
    z_tb     =  bh_cm / 2.0
    z_bb     = -bh_cm / 2.0
    z_bc     = -(bh_cm / 2.0 + cd_cm)
    z_sc_top =  z_tb - (R_s - cd_cm)
    z_sc_bot =  z_bb + (R_s - cd_cm)
    return R_s, z_tc, z_tb, z_bb, z_bc, z_sc_top, z_sc_bot


def _move_body_along_z(comp: adsk.fusion.Component,
                       body: adsk.fusion.BRepBody,
                       z_offset_cm: float) -> adsk.fusion.BRepBody:
    """
    body 를 Z 축 방향으로 z_offset_cm 만큼 평행이동.
    (Fusion 360 에서 Y 축이 회전축이므로 캡 이동에는 실제 모델 Z 축 사용)

    moveFeatures 는 ObjectCollection 과 MoveFeatureInput 을 받음.
    """
    if abs(z_offset_cm) < 1e-9:
        return body

    col = adsk.core.ObjectCollection.create()
    col.add(body)

    T = adsk.core.Matrix3D.create()
    T.setWithCoordinateSystem(
        adsk.core.Point3D.create(0.0, 0.0, z_offset_cm),   # 원점 이동
        adsk.core.Vector3D.create(1.0, 0.0, 0.0),           # X 축 유지
        adsk.core.Vector3D.create(0.0, 1.0, 0.0),           # Y 축 유지
        adsk.core.Vector3D.create(0.0, 0.0, 1.0)            # Z 축 유지
    )

    mv_in = comp.features.moveFeatures.createInput(col, T)
    comp.features.moveFeatures.add(mv_in)
    return body


# ─── 바디 생성 함수 ───────────────────────────────────────────────────────────

def build_round_biconvex(comp: adsk.fusion.Component,
                         r_cm: float,
                         cd_cm: float,
                         bh_cm: float) -> adsk.fusion.BRepBody:
    """
    원형 양볼록 캡 생성 (기존 tablet_generator.py 와 동일한 방식).

    XY 평면에 반쪽 단면 프로파일(반지름 방향 + 높이 방향)을 그린 뒤
    Y 축으로 360° 회전하여 완전한 회전체를 만든다.

    단면 (CCW):
        ① 축 중심선  : (0, z_tc) → (0, z_bc)
        ② 하면 호    : (0, z_bc) → (r, z_bb)
        ③ 측면 직선  : (r, z_bb) → (r, z_tb)
        ④ 상면 호    : (r, z_tb) → (0, z_tc)
    """
    _, z_tc, z_tb, z_bb, z_bc, z_sc_top, z_sc_bot = _compute_geometry(
        r_cm, cd_cm, bh_cm)

    sk    = comp.sketches.add(comp.xYConstructionPlane)
    arcs  = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines

    # ① 축 중심선
    lines.addByTwoPoints(_pt(0, z_tc), _pt(0, z_bc))
    # ② 하면 호
    arcs.addByCenterStartEnd(_pt(0, z_sc_bot), _pt(0, z_bc),  _pt(r_cm, z_bb))
    # ③ 측면 직선
    lines.addByTwoPoints(_pt(r_cm, z_bb), _pt(r_cm, z_tb))
    # ④ 상면 호
    arcs.addByCenterStartEnd(_pt(0, z_sc_top), _pt(r_cm, z_tb), _pt(0, z_tc))

    prof = sk.profiles.item(0)
    rev_in = comp.features.revolveFeatures.createInput(
        prof,
        comp.yConstructionAxis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    rev_in.setAngleExtent(False,
                          adsk.core.ValueInput.createByReal(2.0 * math.pi))
    rev = comp.features.revolveFeatures.add(rev_in)
    return rev.bodies.item(0)


def build_biconvex_lens_extrusion(comp: adsk.fusion.Component,
                                  r_cm: float,
                                  cd_cm: float,
                                  bh_cm: float,
                                  half_s_cm: float) -> adsk.fusion.BRepBody:
    """
    스타디움 중간 직사각 구간 생성.

    XY 평면에 완전 폐쇄된 biconvex 렌즈 윤곽(타원형, 좌우 대칭)을 스케치한 뒤
    Z 축 방향으로 ±half_s_cm 씩 대칭 돌출(Symmetric Extrude).

    렌즈 단면 (CCW, 총 6 세그먼트):
        ① 오른쪽 상면 호 : (r, z_tb)  → (0, z_tc)
        ② 왼쪽  상면 호 : (0, z_tc)  → (−r, z_tb)
        ③ 왼쪽  측면    : (−r, z_tb) → (−r, z_bb)
        ④ 왼쪽  하면 호 : (−r, z_bb) → (0, z_bc)
        ⑤ 오른쪽 하면 호: (0, z_bc)  → (r, z_bb)
        ⑥ 오른쪽 측면   : (r, z_bb)  → (r, z_tb)
    """
    _, z_tc, z_tb, z_bb, z_bc, z_sc_top, z_sc_bot = _compute_geometry(
        r_cm, cd_cm, bh_cm)

    sk    = comp.sketches.add(comp.xYConstructionPlane)
    arcs  = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines

    # ① 오른쪽 상면 호
    arcs.addByCenterStartEnd(
        _pt(0, z_sc_top), _pt( r_cm, z_tb), _pt(0,     z_tc))
    # ② 왼쪽 상면 호
    arcs.addByCenterStartEnd(
        _pt(0, z_sc_top), _pt(0,     z_tc), _pt(-r_cm, z_tb))
    # ③ 왼쪽 측면
    lines.addByTwoPoints(_pt(-r_cm, z_tb), _pt(-r_cm, z_bb))
    # ④ 왼쪽 하면 호
    arcs.addByCenterStartEnd(
        _pt(0, z_sc_bot), _pt(-r_cm, z_bb), _pt(0,     z_bc))
    # ⑤ 오른쪽 하면 호
    arcs.addByCenterStartEnd(
        _pt(0, z_sc_bot), _pt(0,     z_bc), _pt( r_cm, z_bb))
    # ⑥ 오른쪽 측면
    lines.addByTwoPoints(_pt(r_cm, z_bb), _pt(r_cm, z_tb))

    prof = sk.profiles.item(0)
    ext_in = comp.features.extrudeFeatures.createInput(
        prof,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    ext_in.setSymmetricExtent(
        adsk.core.ValueInput.createByReal(half_s_cm),
        False   # taper angle = 0
    )
    ext = comp.features.extrudeFeatures.add(ext_in)
    return ext.bodies.item(0)


def build_stadium_biconvex(comp: adsk.fusion.Component,
                           r_cm: float,
                           ar: float,
                           cd_cm: float,
                           bh_cm: float) -> adsk.fusion.BRepBody:
    """
    스타디움 양볼록 정제 바디 생성 (3-Body Boolean Union).

    ar = 1.0 일 때: half_s = 0 → 원형 biconvex 와 동일.
    ar > 1.0 일 때:
        BODY A (오른쪽 캡) = round_biconvex → Z = +half_s 이동
        BODY B (왼쪽  캡) = round_biconvex → Z = −half_s 이동
        BODY C (중간 구간) = biconvex_lens_extrusion(half_s)
        최종 = Union(A, B, C)

    반환:
        합체된 단일 BRepBody
    """
    half_s_cm = r_cm * (ar - 1.0)

    # ── AR = 1.0: 원형 biconvex 그대로 반환 ──
    if half_s_cm < 1e-6:
        return build_round_biconvex(comp, r_cm, cd_cm, bh_cm)

    # ── BODY A: 오른쪽 캡 ──────────────────────────────────────────────
    body_a = build_round_biconvex(comp, r_cm, cd_cm, bh_cm)
    body_a = _move_body_along_z(comp, body_a, +half_s_cm)

    # ── BODY B: 왼쪽 캡 ───────────────────────────────────────────────
    body_b = build_round_biconvex(comp, r_cm, cd_cm, bh_cm)
    body_b = _move_body_along_z(comp, body_b, -half_s_cm)

    # ── BODY C: 중간 직사각 구간 (biconvex 렌즈 대칭 돌출) ────────────
    body_c = build_biconvex_lens_extrusion(comp, r_cm, cd_cm, bh_cm, half_s_cm)

    # ── Union: A ∪ B ∪ C ──────────────────────────────────────────────
    # 첫 번째 결합: body_c (돌출) 를 target, body_a 를 tool
    tool_col_1 = adsk.core.ObjectCollection.create()
    tool_col_1.add(body_a)
    comb_in_1 = comp.features.combineFeatures.createInput(body_c, tool_col_1)
    comb_in_1.operation       = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comb_in_1.isKeepToolBodies = False
    comp.features.combineFeatures.add(comb_in_1)

    # 두 번째 결합: (body_c ∪ body_a) 에 body_b 추가
    tool_col_2 = adsk.core.ObjectCollection.create()
    tool_col_2.add(body_b)
    comb_in_2 = comp.features.combineFeatures.createInput(body_c, tool_col_2)
    comb_in_2.operation       = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comb_in_2.isKeepToolBodies = False
    comp.features.combineFeatures.add(comb_in_2)

    return body_c   # 결합 후에도 같은 객체 참조가 최종 바디


# ─── STL 내보내기 / 정리 ──────────────────────────────────────────────────────

def export_stl(design: adsk.fusion.Design,
               body: adsk.fusion.BRepBody,
               filepath: str) -> None:
    """BRepBody를 바이너리 STL 파일로 내보내기."""
    em   = design.exportManager
    opts = em.createSTLExportOptions(body, filepath)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    opts.isBinaryFormat = True
    em.execute(opts)


def _cleanup_root(root: adsk.fusion.Component) -> None:
    """루프 반복 사이에 root 컴포넌트의 body·sketch 잔여물 제거."""
    for i in range(root.bRepBodies.count - 1, -1, -1):
        try:
            root.bRepBodies.item(i).deleteMe()
        except Exception:
            pass
    for i in range(root.sketches.count - 1, -1, -1):
        try:
            root.sketches.item(i).deleteMe()
        except Exception:
            pass


# ─── 메인 엔트리 포인트 ───────────────────────────────────────────────────────

def run(context):                                   # noqa: C901
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        # Direct Design 모드: body.deleteMe() 로 루프마다 깨끗하게 삭제
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        root = design.rootComponent
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        total     = len(RADII_MM) * len(ASPECT_RATIOS) * len(CURVATURES)
        count     = 0
        success   = 0
        error_log: list[str] = []

        # ── CSV 메타데이터 파일 ───────────────────────────────────────
        csv_path   = os.path.join(OUTPUT_DIR, "_tablet_stadium_index.csv")
        csv_file   = open(csv_path, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "filename",
            "R_mm", "AR", "CV",
            "minor_diameter_mm",   # = R × 2  (단축 직경)
            "major_diameter_mm",   # = R × 2 × AR  (장축 끝점간 거리)
            "straight_length_mm",  # = R × 2 × (AR − 1)  (직사각 구간 길이)
            "total_thickness_mm",  # = band_height + 2 × cup_depth
            "band_height_mm",
            "cup_depth_mm",
            "sphere_radius_mm"
        ])

        # ── 진행 다이얼로그 ───────────────────────────────────────────
        prog = ui.createProgressDialog()
        prog.isCancelButtonShown = True
        prog.show("스타디움 정제 STL 생성 중", "준비 중...", 0, total, 1)

        # ── 메인 루프 ─────────────────────────────────────────────────
        for r_mm in RADII_MM:
            for ar in ASPECT_RATIOS:
                for cv in CURVATURES:
                    if prog.wasCancelled:
                        break

                    r_cm  = r_mm / 10.0
                    cd_cm = cv * 2.0 * r_cm      # cup_depth
                    bh_cm = r_cm * BAND_RATIO     # band_height

                    R_s_mm             = (r_mm**2 + (cd_cm * 10)**2) / (2.0 * cd_cm * 10)
                    total_thickness_mm = (bh_cm + 2.0 * cd_cm) * 10.0
                    straight_len_mm    = r_mm * 2.0 * (ar - 1.0)

                    fname = (f"tablet_stadium_R{r_mm:.1f}"
                             f"_AR{ar:.2f}_CV{cv:.2f}.stl")
                    fpath = os.path.join(OUTPUT_DIR, fname)

                    count += 1
                    prog.progressValue = count
                    prog.message = f"({count}/{total})  {fname}"

                    try:
                        body = build_stadium_biconvex(
                            root, r_cm, ar, cd_cm, bh_cm)
                        export_stl(design, body, fpath)

                        csv_writer.writerow([
                            fname,
                            r_mm, round(ar, 2), round(cv, 2),
                            round(r_mm * 2, 1),
                            round(r_mm * 2 * ar, 1),
                            round(straight_len_mm, 2),
                            round(total_thickness_mm, 2),
                            round(bh_cm * 10, 2),
                            round(cd_cm * 10, 2),
                            round(R_s_mm, 2)
                        ])
                        success += 1

                    except Exception as e:
                        error_log.append(f"{fname}: {e}")

                    finally:
                        _cleanup_root(root)

                    adsk.doEvents()

        # ── 마무리 ────────────────────────────────────────────────────
        csv_file.close()
        prog.hide()

        msg = (
            f"✅ 완료: {success} / {total}개 STL 생성\n"
            f"📁 저장 위치: {OUTPUT_DIR}\n"
            f"📋 인덱스: _tablet_stadium_index.csv\n\n"
            f"💊 형태: Stadium(캡렛) 양볼록  ← R × (AR−1) = 직사각 구간 절반"
        )
        if error_log:
            msg += f"\n\n⚠️  오류 {len(error_log)}건:\n"
            msg += "\n".join(error_log[:5])
            if len(error_log) > 5:
                msg += f"\n... 외 {len(error_log)-5}건"

        ui.messageBox(msg)

    except Exception:
        if ui:
            ui.messageBox(f"스크립트 오류:\n{traceback.format_exc()}")
        raise
