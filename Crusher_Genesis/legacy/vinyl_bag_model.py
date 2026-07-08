"""
Fusion 360 Python API Script
비닐봉투 (Vinyl Bag) Solid Model

형태: 직사각형 평면 비닐봉투, 좌우 + 하단 사이드 실링
사용법: Fusion 360 > Tools > Add-Ins > Scripts > 이 파일 실행
"""

import adsk.core
import adsk.fusion
import traceback

# ─────────────────────────────────────────────
#  파라미터 (단위: cm)
# ─────────────────────────────────────────────
BAG_WIDTH   = 20.0   # 봉투 폭
BAG_HEIGHT  = 30.0   # 봉투 높이 (세로 길이)
BAG_THICK   = 0.20   # 봉투 전체 두께 (비닐 2겹 합산)

SEAL_W_SIDE = 0.80   # 좌우 실링 폭
SEAL_W_BTM  = 0.80   # 하단 실링 높이
SEAL_EXTRA  = 0.05   # 실링부 돌출 두께 (실링이 약간 두꺼움)
# ─────────────────────────────────────────────


def create_rect_profile(sketch, x0, y0, x1, y1):
    """스케치에 사각형 프로파일을 그리고 해당 프로파일을 반환."""
    lines = sketch.sketchCurves.sketchLines
    p = [
        adsk.core.Point3D.create(x0, y0, 0),
        adsk.core.Point3D.create(x1, y0, 0),
        adsk.core.Point3D.create(x1, y1, 0),
        adsk.core.Point3D.create(x0, y1, 0),
    ]
    lines.addByTwoPoints(p[0], p[1])
    lines.addByTwoPoints(p[1], p[2])
    lines.addByTwoPoints(p[2], p[3])
    lines.addByTwoPoints(p[3], p[0])
    return sketch.profiles.item(0)


def extrude(comp, profile, depth, operation):
    """프로파일을 depth 만큼 돌출."""
    ext_input = comp.features.extrudeFeatures.createInput(profile, operation)
    ext_input.setDistanceExtent(
        False, adsk.core.ValueInput.createByReal(depth)
    )
    return comp.features.extrudeFeatures.add(ext_input)


def run(context):
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root   = design.rootComponent
        xy     = root.xYConstructionPlane

        NewBody = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        Join    = adsk.fusion.FeatureOperations.JoinFeatureOperation

        # ── 1. 메인 바디 (봉투 전체 면적) ──────────────────────
        sk_main = root.sketches.add(xy)
        prof_main = create_rect_profile(
            sk_main, 0, 0, BAG_WIDTH, BAG_HEIGHT
        )
        feat_main = extrude(root, prof_main, BAG_THICK, NewBody)
        feat_main.bodies.item(0).name = "Bag_Body"

        # ── 2. 좌측 실링 ───────────────────────────────────────
        sk_left = root.sketches.add(xy)
        prof_left = create_rect_profile(
            sk_left, 0, 0, SEAL_W_SIDE, BAG_HEIGHT
        )
        feat_left = extrude(root, prof_left, BAG_THICK + SEAL_EXTRA, Join)

        # ── 3. 우측 실링 ───────────────────────────────────────
        sk_right = root.sketches.add(xy)
        prof_right = create_rect_profile(
            sk_right,
            BAG_WIDTH - SEAL_W_SIDE, 0,
            BAG_WIDTH,               BAG_HEIGHT,
        )
        feat_right = extrude(root, prof_right, BAG_THICK + SEAL_EXTRA, Join)

        # ── 4. 하단 실링 ───────────────────────────────────────
        sk_btm = root.sketches.add(xy)
        prof_btm = create_rect_profile(
            sk_btm, 0, 0, BAG_WIDTH, SEAL_W_BTM
        )
        feat_btm = extrude(root, prof_btm, BAG_THICK + SEAL_EXTRA, Join)

        # ── 5. 실링 경계선 스케치 (시각 참조용) ────────────────
        sk_ref = root.sketches.add(xy)
        ref_lines = sk_ref.sketchCurves.sketchLines

        # 좌측 실링 내부 경계
        ref_lines.addByTwoPoints(
            adsk.core.Point3D.create(SEAL_W_SIDE, 0,          0),
            adsk.core.Point3D.create(SEAL_W_SIDE, BAG_HEIGHT, 0),
        )
        # 우측 실링 내부 경계
        ref_lines.addByTwoPoints(
            adsk.core.Point3D.create(BAG_WIDTH - SEAL_W_SIDE, 0,          0),
            adsk.core.Point3D.create(BAG_WIDTH - SEAL_W_SIDE, BAG_HEIGHT, 0),
        )
        # 하단 실링 내부 경계
        ref_lines.addByTwoPoints(
            adsk.core.Point3D.create(0,         SEAL_W_BTM, 0),
            adsk.core.Point3D.create(BAG_WIDTH, SEAL_W_BTM, 0),
        )
        sk_ref.name = "Seal_Reference_Lines"

        ui.messageBox(
            "비닐봉투 모델링 완료!\n\n"
            f"폭    : {BAG_WIDTH} cm\n"
            f"높이  : {BAG_HEIGHT} cm\n"
            f"두께  : {BAG_THICK} cm\n"
            f"실링폭: {SEAL_W_SIDE} cm (좌/우), {SEAL_W_BTM} cm (하단)"
        )

    except Exception:
        if ui:
            ui.messageBox("오류 발생:\n" + traceback.format_exc())
