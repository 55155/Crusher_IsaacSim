"""
Fusion 360 Python API Script
약봉투 (Medicine Envelope) — Surface Model for Genesis PBD

구조 (5개 Surface Body):

      상단 개방 (open top)
   ┌──────────────────┐
   │   Front_Panel    │  ← Z = 0      (앞면)
   │                  │
   └──────────────────┘
        ↕ BAG_DEPTH
   ┌──────────────────┐
   │   Back_Panel     │  ← Z = BAG_DEPTH  (뒷면)
   │                  │
   └──────────────────┘

   + Left_Seal   : X = 0         (좌측 면)
   + Right_Seal  : X = BAG_WIDTH (우측 면)
   + Bottom_Seal : Y = 0         (하단 면)

Genesis PBD 용도:
  각 Surface Body → Triangle Mesh → Particle + Distance Constraint
  실링 엣지 = 인접 패널 공유 엣지 → 시뮬레이션 내 Weld Constraint

사용법: Fusion 360 > Tools > Add-Ins > Scripts > medicine_envelope > Run
"""

import adsk.core
import adsk.fusion
import traceback

# ─────────────────────────────────────────────
#  파라미터 (단위: cm)
# ─────────────────────────────────────────────
BAG_WIDTH  = 8.0   # 봉투 폭   (X 방향)
BAG_HEIGHT = 12.0  # 봉투 높이 (Y 방향)
BAG_DEPTH  = 1.0   # 내부 깊이 (Z 방향, 내용물 공간)
# ─────────────────────────────────────────────


def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def make_rect_patch(root, plane, corners, name):
    """
    corners: [p0, p1, p2, p3] — 사각형 꼭짓점 (3D 좌표, 순서대로)
    지정 평면에 사각형 서피스 패치를 생성하고 name 을 부여한다.
    """
    sk = root.sketches.add(plane)
    ls = sk.sketchCurves.sketchLines
    for i in range(4):
        ls.addByTwoPoints(corners[i], corners[(i + 1) % 4])

    patch_in = root.features.patchFeatures.createInput(
        sk.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    feat = root.features.patchFeatures.add(patch_in)
    feat.bodies.item(0).name = name
    return feat


def offset_plane(root, base_plane, offset_cm):
    """base_plane 에서 offset_cm 만큼 이동한 construction plane 반환."""
    pi = root.constructionPlanes.createInput()
    pi.setByOffset(base_plane, adsk.core.ValueInput.createByReal(offset_cm))
    return root.constructionPlanes.add(pi)


def run(context):
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        root   = design.rootComponent

        W = BAG_WIDTH
        H = BAG_HEIGHT
        D = BAG_DEPTH

        xy = root.xYConstructionPlane   # Z = 0  (앞면 기준)
        xz = root.xZConstructionPlane   # Y = 0  (하단 실링 기준)
        yz = root.yZConstructionPlane   # X = 0  (좌측 실링 기준)

        # ── 1. 앞면 패널  Front_Panel  (XY, Z=0) ────────────────
        make_rect_patch(root, xy,
            [pt(0,0,0), pt(W,0,0), pt(W,H,0), pt(0,H,0)],
            "Front_Panel")

        # ── 2. 뒷면 패널  Back_Panel   (XY 오프셋, Z=D) ─────────
        back_plane = offset_plane(root, xy, D)
        back_plane.name = "Back_Plane"
        make_rect_patch(root, back_plane,
            [pt(0,0,D), pt(W,0,D), pt(W,H,D), pt(0,H,D)],
            "Back_Panel")

        # ── 3. 하단 실링  Bottom_Seal  (XZ, Y=0) ────────────────
        make_rect_patch(root, xz,
            [pt(0,0,0), pt(W,0,0), pt(W,0,D), pt(0,0,D)],
            "Bottom_Seal")

        # ── 4. 좌측 실링  Left_Seal    (YZ, X=0) ────────────────
        make_rect_patch(root, yz,
            [pt(0,0,0), pt(0,H,0), pt(0,H,D), pt(0,0,D)],
            "Left_Seal")

        # ── 5. 우측 실링  Right_Seal   (YZ 오프셋, X=W) ─────────
        right_plane = offset_plane(root, yz, W)
        right_plane.name = "Right_Plane"
        make_rect_patch(root, right_plane,
            [pt(W,0,0), pt(W,H,0), pt(W,H,D), pt(W,0,D)],
            "Right_Seal")

        ui.messageBox(
            "약봉투 Surface 모델링 완료!\n\n"
            f"폭      : {W} cm  (X)\n"
            f"높이    : {H} cm  (Y)\n"
            f"내부깊이: {D} cm  (Z)\n"
            f"상단    : 개방 (open)\n\n"
            "Surface Bodies:\n"
            "  · Front_Panel   Z = 0\n"
            "  · Back_Panel    Z = D\n"
            "  · Left_Seal     X = 0\n"
            "  · Right_Seal    X = W\n"
            "  · Bottom_Seal   Y = 0"
        )

    except Exception:
        if ui:
            ui.messageBox("오류 발생:\n" + traceback.format_exc())
