# =============================================================================
# tablet_generator.py
# Fusion 360 Script — 정제(Tablet) STL 파일 1,000개 배치 생성
# =============================================================================
#
# ⚠️  이 스크립트는 로컬 Python 환경에서 직접 실행할 수 없습니다.
#     반드시 Autodesk Fusion 360 내부의 스크립트 실행 환경에서만 동작합니다.
#     (adsk.core / adsk.fusion 모듈은 Fusion 360 전용 API입니다.)
#
# ─── 사용 방법 ────────────────────────────────────────────────────────────────
#
#  STEP 1.  이 파일을 Fusion 360 Scripts 폴더에 복사합니다.
#
#           복사 위치 (Windows):
#             C:\Users\<사용자명>\AppData\Roaming\Autodesk\
#                 Autodesk Fusion 360\API\Scripts\tablet_generator\
#
#           ※ 폴더명과 파일명이 반드시 일치해야 합니다.
#              tablet_generator\tablet_generator.py
#
#  STEP 2.  Fusion 360 을 실행하고, 빈 디자인 문서를 엽니다.
#           (File > New Design — "부품(Part)" 모드 무관, 실행 시 Direct 모드로 전환됨)
#
#  STEP 3.  상단 메뉴: UTILITIES > Scripts and Add-ins (단축키: Shift+S)
#
#  STEP 4.  [Scripts] 탭 > 좌측 하단 [+] 버튼 > 이 파일이 있는 폴더 선택
#
#  STEP 5.  목록에서 "tablet_generator" 선택 > [Run] 클릭
#
#  STEP 6.  진행 다이얼로그가 표시되며 1,000개 STL 파일 생성 시작.
#           완료 시 결과 메시지 박스 표시.
#
# ─── 출력 위치 ────────────────────────────────────────────────────────────────
#
#  OUTPUT_DIR (아래 설정 참고):
#    ~/Desktop/Crusher_IsaacSim/tablets_stl/stl/
#
#  생성 파일 예시:
#    tablet_R4.0_AR1.00_CV0.08.stl
#    tablet_R4.0_AR1.00_CV0.11.stl
#    ...
#    tablet_R8.5_AR2.50_CV0.35.stl   (총 1,000개)
#    _tablet_index.csv                (파라미터 메타데이터)
#
# ─── 파라미터 설명 ────────────────────────────────────────────────────────────
#
#  R  (단반경, mm)   : 정제 단축 반경. 직경 = R × 2
#  AR (형태비)       : 장/단축 비율. 1.0 = 원형, 2.5 = 매우 긴 타원(캡렛)
#  CV (곡률 비율)    : cup_depth / (2×R). 0.08 = 편평, 0.35 = 구면에 가까운 볼록
#
# =============================================================================

import adsk.core
import adsk.fusion
import traceback
import os
import csv
import math

# ─── 사용자 설정 ───────────────────────────────────────────────────────────────
# STL 출력 경로: 현재 사용자의 Desktop/Crusher_IsaacSim/tablets_stl/stl/
OUTPUT_DIR = os.path.join(os.path.expanduser("~"), "Desktop",
                          "Crusher_IsaacSim", "tablets_stl", "stl")

# 각 파라미터 10단계 (총 10 × 10 × 10 = 1,000개)
RADII_MM = [4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5]
#  └ 단반경 (short-axis radius). 직경 = R × 2.

ASPECT_RATIOS = [1.0, 1.17, 1.33, 1.5, 1.67, 1.83, 2.0, 2.17, 2.33, 2.5]
#  └ 1.0 = 원형, 2.5 = 매우 긴 타원(캡렛). 장축 반경 = R × AR.

CURVATURES = [0.08, 0.11, 0.14, 0.17, 0.20, 0.23, 0.26, 0.29, 0.32, 0.35]
#  └ cup_depth / (2 × R). 0.08 ≈ 편평, 0.35 ≈ 볼록.

BAND_RATIO = 0.20
#  └ 밴드 높이 = R_cm × BAND_RATIO (고정값, 비례).
# ──────────────────────────────────────────────────────────────────────────────


def _pt(x_cm, z_cm):
    """
    스케치 로컬 좌표 헬퍼.
    스케치 평면: XY (World XY = Sketch XY, 혼동 없는 평면 선택)
      - 스케치 X → 반경 방향 (radial)
      - 스케치 Y → 축 방향 (axial / height)
      - 스케치 Z = 0 (항상 평면 위)
    """
    return adsk.core.Point3D.create(x_cm, z_cm, 0.0)


def build_round_biconvex(comp, r_cm, cd_cm, bh_cm):
    """
    원형 양볼록 정제(round biconvex tablet) BRepBody 생성.

    XY 평면에 닫힌 단면 프로파일을 그린 후 Y축으로 360° 회전.

    단면 구성 (CCW 방향으로 연결된 4개 세그먼트):
        ① 축 중심선  : (0, z_tc) → (0, z_bc)
        ② 하면 호    : (0, z_bc) → (r, z_bb)   [구면 중심: (0, z_sc_bot)]
        ③ 측면 직선  : (r, z_bb) → (r, z_tb)
        ④ 상면 호    : (r, z_tb) → (0, z_tc)   [구면 중심: (0, z_sc_top)]

    매개변수:
        r_cm  : 단반경 (cm)
        cd_cm : 컵 깊이 (cm) — 볼록면 apex까지의 높이
        bh_cm : 밴드 높이 (cm) — 측면 직선 구간

    반환:
        adsk.fusion.BRepBody
    """
    # 구면 반경: R_s = (r² + cd²) / (2·cd)
    R_s = (r_cm ** 2 + cd_cm ** 2) / (2.0 * cd_cm)

    # 높이 기준점 (Z 좌표)
    z_tc = bh_cm / 2.0 + cd_cm       # 상면 apex
    z_tb = bh_cm / 2.0               # 상면 밴드 경계
    z_bb = -bh_cm / 2.0              # 하면 밴드 경계
    z_bc = -(bh_cm / 2.0 + cd_cm)   # 하면 apex

    # 구면 중심 Z 좌표
    # 유도: 구면이 밴드 경계 (r, z_tb) 를 지나야 함
    # → z_sc_top = z_tb - sqrt(R_s² - r²) = z_tb - (R_s - cd_cm)
    z_sc_top = z_tb - (R_s - cd_cm)
    z_sc_bot = z_bb + (R_s - cd_cm)

    # ── 스케치 생성 (XY 평면: X=반경, Y=높이, Z=0 고정) ──────────────
    sk = comp.sketches.add(comp.xYConstructionPlane)
    arcs  = sk.sketchCurves.sketchArcs
    lines = sk.sketchCurves.sketchLines

    # ① 축 중심선 (회전축 쪽 닫힘선)
    lines.addByTwoPoints(_pt(0, z_tc), _pt(0, z_bc))

    # ② 하면 호 (CCW: apex → 밴드 경계)
    arcs.addByCenterStartEnd(
        _pt(0,    z_sc_bot),
        _pt(0,    z_bc),
        _pt(r_cm, z_bb)
    )

    # ③ 측면 직선
    lines.addByTwoPoints(_pt(r_cm, z_bb), _pt(r_cm, z_tb))

    # ④ 상면 호 (CCW: 밴드 경계 → apex)
    arcs.addByCenterStartEnd(
        _pt(0,    z_sc_top),
        _pt(r_cm, z_tb),
        _pt(0,    z_tc)
    )

    # ── Y축 기준 360° 회전체 생성 ────────────────────────────────────
    prof = sk.profiles.item(0)
    rev_input = comp.features.revolveFeatures.createInput(
        prof,
        comp.yConstructionAxis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    )
    rev_input.setAngleExtent(
        False,
        adsk.core.ValueInput.createByReal(2.0 * math.pi)
    )
    rev = comp.features.revolveFeatures.add(rev_input)
    return rev.bodies.item(0)


def apply_oval_scale(comp, body, ar):
    """
    X축 방향으로 ar 배 확장 → 타원형 정제.
    ar == 1.0 이면 그대로 반환.

    주의: scaleFeatures.createInput() 의 기준점은 반드시
          SketchPoint / ConstructionPoint 여야 함 (bare Point3D 불가).
          → XZ 평면에 임시 스케치 포인트를 원점에 생성해 기준점으로 사용.
    """
    if ar <= 1.001:
        return body

    ref_sk  = comp.sketches.add(comp.xZConstructionPlane)
    ref_pt  = ref_sk.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))

    entities = adsk.core.ObjectCollection.create()
    entities.add(body)

    sc_input = comp.features.scaleFeatures.createInput(
        entities,
        ref_pt,
        adsk.core.ValueInput.createByReal(1.0)
    )
    sc_input.setToNonUniform(
        adsk.core.ValueInput.createByReal(ar),   # X: 장축 방향으로 확장
        adsk.core.ValueInput.createByReal(1.0),  # Y: 회전축 (높이) 그대로
        adsk.core.ValueInput.createByReal(1.0)   # Z: 그대로
    )
    comp.features.scaleFeatures.add(sc_input)
    return body


def export_stl(design, body, filepath):
    """BRepBody를 바이너리 STL 파일로 내보내기."""
    em   = design.exportManager
    opts = em.createSTLExportOptions(body, filepath)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    opts.isBinaryFormat = True
    em.execute(opts)


def _cleanup_root(root):
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


def run(context):
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        # ── Direct Design 모드 강제 설정 ──────────────────────────────
        # Parametric 모드의 "부품" 문서는 서브 컴포넌트를 허용하지 않음.
        # Direct 모드에서는 body.deleteMe()로 루프마다 깨끗하게 삭제 가능.
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        root = design.rootComponent
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        total     = len(RADII_MM) * len(ASPECT_RATIOS) * len(CURVATURES)
        count     = 0
        success   = 0
        error_log = []

        # ── CSV 메타데이터 파일 준비 ───────────────────────────────────
        csv_path   = os.path.join(OUTPUT_DIR, "_tablet_index.csv")
        csv_file   = open(csv_path, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "filename",
            "R_mm", "AR", "CV",
            "diameter_mm", "major_diameter_mm",
            "total_thickness_mm", "band_height_mm",
            "cup_depth_mm", "sphere_radius_mm"
        ])

        # ── 진행 다이얼로그 ───────────────────────────────────────────
        prog = ui.createProgressDialog()
        prog.isCancelButtonShown = True
        prog.show("정제 STL 생성 중", "준비 중...", 0, total, 1)

        # ── 메인 루프 ─────────────────────────────────────────────────
        for r_mm in RADII_MM:
            for ar in ASPECT_RATIOS:
                for cv in CURVATURES:
                    if prog.wasCancelled:
                        break

                    r_cm  = r_mm / 10.0
                    cd_cm = cv * 2.0 * r_cm
                    bh_cm = r_cm * BAND_RATIO

                    R_s_mm             = (r_mm**2 + (cd_cm*10)**2) / (2.0 * cd_cm * 10)
                    total_thickness_mm = (bh_cm + 2.0 * cd_cm) * 10.0

                    fname = f"tablet_R{r_mm:.1f}_AR{ar:.2f}_CV{cv:.2f}.stl"
                    fpath = os.path.join(OUTPUT_DIR, fname)

                    count += 1
                    prog.progressValue = count
                    prog.message = f"({count}/{total})  {fname}"

                    try:
                        body = build_round_biconvex(root, r_cm, cd_cm, bh_cm)
                        body = apply_oval_scale(root, body, ar)
                        export_stl(design, body, fpath)

                        csv_writer.writerow([
                            fname, r_mm, round(ar,2), round(cv,2),
                            round(r_mm*2, 1), round(r_mm*2*ar, 1),
                            round(total_thickness_mm, 2),
                            round(bh_cm*10, 2),
                            round(cd_cm*10, 2),
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
            f"📋 인덱스: _tablet_index.csv"
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
