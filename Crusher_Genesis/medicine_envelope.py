"""
Fusion 360 Python API Script
약봉투 (Korean Medicine Envelope) Solid Model

형태:
  - 직사각형 납작 파우치 바디
  - 상단: 삼각형 접힘 플랩 (fold-over flap) — 약봉투 특유의 뾰족한 상단
  - 좌/우: 사이드 실링 (약간 두꺼운 띠)
  - 하단: 바텀 실링

사용법: Fusion 360 > Tools > Add-Ins > Scripts > 이 파일 실행
"""

import adsk.core
import adsk.fusion
import math
import traceback

# ─────────────────────────────────────────────
#  파라미터 (단위: cm)
# ─────────────────────────────────────────────
BAG_WIDTH    = 8.0    # 봉투 폭
BAG_HEIGHT   = 12.0   # 봉투 높이 (플랩 포함)
BAG_THICK    = 0.15   # 봉투 바디 두께 (비닐/종이 2겹)

SEAL_SIDE_W  = 0.5    # 좌우 실링 폭
SEAL_BTM_H   = 0.5    # 하단 실링 높이
SEAL_EXTRA   = 0.04   # 실링부 추가 두께

FLAP_HEIGHT  = 2.5    # 상단 플랩 높이 (접히는 부분 세로 길이)
# 플랩은 상단 중앙이 뾰족한 삼각형 형태로 접힘
# 플랩 피크(꼭짓점)의 X 오프셋 = 중앙(BAG_WIDTH/2)
# ─────────────────────────────────────────────


def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def extrude_profile(comp, profile, depth,
                    operation=adsk.fusion.FeatureOperations.NewBodyFeatureOperation):
    feat_input = comp.features.extrudeFeatures.createInput(profile, operation)
    feat_input.setDistanceExtent(
        False, adsk.core.ValueInput.createByReal(depth)
    )
    return comp.features.extrudeFeatures.add(feat_input)


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

        # ──────────────────────────────────────────────────
        #  1. 바디 하단부: 직사각형 메인 파우치
        #     Y = 0 (바닥 실링 하단) ~ BAG_HEIGHT - FLAP_HEIGHT (플랩 시작)
        # ──────────────────────────────────────────────────
        body_top = BAG_HEIGHT - FLAP_HEIGHT

        sk_body = root.sketches.add(xy)
        lines_b = sk_body.sketchCurves.sketchLines
        lines_b.addByTwoPoints(pt(0,         0       ), pt(BAG_WIDTH, 0       ))
        lines_b.addByTwoPoints(pt(BAG_WIDTH, 0       ), pt(BAG_WIDTH, body_top))
        lines_b.addByTwoPoints(pt(BAG_WIDTH, body_top), pt(0,         body_top))
        lines_b.addByTwoPoints(pt(0,         body_top), pt(0,         0       ))

        feat_body = extrude_profile(root, sk_body.profiles.item(0), BAG_THICK, NewBody)
        feat_body.bodies.item(0).name = "Bag_Body"

        # ──────────────────────────────────────────────────
        #  2. 상단 플랩: 사다리꼴 + 삼각형 조합
        #
        #  플랩 형태 (약봉투 특유의 접힘):
        #
        #         (cx, BAG_HEIGHT)  ← 꼭짓점 (중앙 최상단)
        #               /\
        #              /  \
        #  (0, body_top)──(BAG_WIDTH, body_top)
        #
        #  삼각형 프로파일로 플랩을 표현
        # ──────────────────────────────────────────────────
        cx = BAG_WIDTH / 2.0

        sk_flap = root.sketches.add(xy)
        lines_f = sk_flap.sketchCurves.sketchLines
        lines_f.addByTwoPoints(pt(0,         body_top    ), pt(BAG_WIDTH, body_top    ))
        lines_f.addByTwoPoints(pt(BAG_WIDTH, body_top    ), pt(cx,        BAG_HEIGHT  ))
        lines_f.addByTwoPoints(pt(cx,        BAG_HEIGHT  ), pt(0,         body_top    ))

        feat_flap = extrude_profile(root, sk_flap.profiles.item(0), BAG_THICK, Join)

        # ──────────────────────────────────────────────────
        #  3. 좌측 실링 (바디 부분만, 플랩 제외)
        # ──────────────────────────────────────────────────
        sk_sl = root.sketches.add(xy)
        lines_sl = sk_sl.sketchCurves.sketchLines
        lines_sl.addByTwoPoints(pt(0,            0       ), pt(SEAL_SIDE_W, 0       ))
        lines_sl.addByTwoPoints(pt(SEAL_SIDE_W,  0       ), pt(SEAL_SIDE_W, body_top))
        lines_sl.addByTwoPoints(pt(SEAL_SIDE_W,  body_top), pt(0,           body_top))
        lines_sl.addByTwoPoints(pt(0,            body_top), pt(0,           0       ))

        extrude_profile(root, sk_sl.profiles.item(0), BAG_THICK + SEAL_EXTRA, Join)

        # ──────────────────────────────────────────────────
        #  4. 우측 실링
        # ──────────────────────────────────────────────────
        rx = BAG_WIDTH - SEAL_SIDE_W
        sk_sr = root.sketches.add(xy)
        lines_sr = sk_sr.sketchCurves.sketchLines
        lines_sr.addByTwoPoints(pt(rx,        0       ), pt(BAG_WIDTH, 0       ))
        lines_sr.addByTwoPoints(pt(BAG_WIDTH, 0       ), pt(BAG_WIDTH, body_top))
        lines_sr.addByTwoPoints(pt(BAG_WIDTH, body_top), pt(rx,        body_top))
        lines_sr.addByTwoPoints(pt(rx,        body_top), pt(rx,        0       ))

        extrude_profile(root, sk_sr.profiles.item(0), BAG_THICK + SEAL_EXTRA, Join)

        # ──────────────────────────────────────────────────
        #  5. 하단 실링
        # ──────────────────────────────────────────────────
        sk_sb = root.sketches.add(xy)
        lines_sb = sk_sb.sketchCurves.sketchLines
        lines_sb.addByTwoPoints(pt(0,         0          ), pt(BAG_WIDTH, 0          ))
        lines_sb.addByTwoPoints(pt(BAG_WIDTH, 0          ), pt(BAG_WIDTH, SEAL_BTM_H ))
        lines_sb.addByTwoPoints(pt(BAG_WIDTH, SEAL_BTM_H ), pt(0,         SEAL_BTM_H ))
        lines_sb.addByTwoPoints(pt(0,         SEAL_BTM_H ), pt(0,         0          ))

        extrude_profile(root, sk_sb.profiles.item(0), BAG_THICK + SEAL_EXTRA, Join)

        # ──────────────────────────────────────────────────
        #  6. 참조 스케치 — 실링 경계선
        # ──────────────────────────────────────────────────
        sk_ref = root.sketches.add(xy)
        ref = sk_ref.sketchCurves.sketchLines
        # 좌측 실링 경계
        ref.addByTwoPoints(pt(SEAL_SIDE_W, 0       ), pt(SEAL_SIDE_W, body_top))
        # 우측 실링 경계
        ref.addByTwoPoints(pt(rx,          0       ), pt(rx,          body_top))
        # 하단 실링 경계
        ref.addByTwoPoints(pt(0,           SEAL_BTM_H), pt(BAG_WIDTH, SEAL_BTM_H))
        # 플랩 시작선
        ref.addByTwoPoints(pt(0,           body_top), pt(BAG_WIDTH, body_top))
        sk_ref.name = "Seal_Reference_Lines"

        ui.messageBox(
            "약봉투 모델링 완료!\n\n"
            f"폭      : {BAG_WIDTH} cm\n"
            f"전체높이 : {BAG_HEIGHT} cm\n"
            f"바디높이 : {body_top:.2f} cm\n"
            f"플랩높이 : {FLAP_HEIGHT} cm\n"
            f"봉투두께 : {BAG_THICK} cm\n"
            f"실링폭   : 좌우 {SEAL_SIDE_W} cm / 하단 {SEAL_BTM_H} cm"
        )

    except Exception:
        if ui:
            ui.messageBox("오류 발생:\n" + traceback.format_exc())
