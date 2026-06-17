"""
Fusion 360 Add-In
약봉투 (Medicine Envelope) — Surface Model Generator v2

변경사항 (v2):
  - 5개 Surface Patch 생성 후 Stitch 로 단일 Open Shell로 봉합
  - 결과: 위가 열린 직사각형 주머니 형태 (봉투)
  - 비율 조정: 깊이(Z)를 더 두껍게 하여 봉투 형태 명확화

툴바: Surface 탭 > Create > "약봉투 생성"
"""

import adsk.core
import adsk.fusion
import traceback

_handlers = []

CMD_ID      = "medicine_envelope_cmd"
CMD_NAME    = "약봉투 생성"
CMD_TOOLTIP = "약봉투 Surface Model을 생성합니다.\nGenesis PBD 시뮬레이션용 Open Shell."
PANEL_ID    = "SurfaceCreatePanel"
WORKSPACE   = "FusionSolidEnvironment"


# ─────────────────────────────────────────────────────────────
#  헬퍼
# ─────────────────────────────────────────────────────────────

def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def make_rect_patch(root, plane, corners):
    """사각형 Surface Patch 생성 → SurfaceBody 반환."""
    sk = root.sketches.add(plane)
    ls = sk.sketchCurves.sketchLines
    for i in range(4):
        ls.addByTwoPoints(corners[i], corners[(i + 1) % 4])
    patch_in = root.features.patchFeatures.createInput(
        sk.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    feat = root.features.patchFeatures.add(patch_in)
    return feat.bodies.item(0)


def offset_plane(root, base, dist):
    pi = root.constructionPlanes.createInput()
    pi.setByOffset(base, adsk.core.ValueInput.createByReal(dist))
    pl = root.constructionPlanes.add(pi)
    return pl


def build_envelope(root, W, H, D):
    """
    5개 Surface Patch → Stitch → 단일 Open Shell (위가 열린 봉투)

    좌표계:
      X: 봉투 폭  (0 ~ W)
      Y: 봉투 높이 (0 ~ H)   ← Y+ 가 상단 (개방부)
      Z: 봉투 깊이 (0 ~ D)   ← 앞/뒤 두께
    """
    xy = root.xYConstructionPlane
    xz = root.xZConstructionPlane
    yz = root.yZConstructionPlane

    bodies = []

    # ── 1. 앞면  Front_Panel   (XY, Z=0) ──────────────────
    b = make_rect_patch(root, xy,
        [pt(0,0,0), pt(W,0,0), pt(W,H,0), pt(0,H,0)])
    b.name = "Front_Panel"
    bodies.append(b)

    # ── 2. 뒷면  Back_Panel    (XY offset, Z=D) ───────────
    back_pl = offset_plane(root, xy, D)
    back_pl.name = "Back_Plane"
    b = make_rect_patch(root, back_pl,
        [pt(0,0,D), pt(W,0,D), pt(W,H,D), pt(0,H,D)])
    b.name = "Back_Panel"
    bodies.append(b)

    # ── 3. 하단  Bottom_Seal   (XZ, Y=0) ─────────────────
    b = make_rect_patch(root, xz,
        [pt(0,0,0), pt(W,0,0), pt(W,0,D), pt(0,0,D)])
    b.name = "Bottom_Seal"
    bodies.append(b)

    # ── 4. 좌측  Left_Seal     (YZ, X=0) ─────────────────
    b = make_rect_patch(root, yz,
        [pt(0,0,0), pt(0,H,0), pt(0,H,D), pt(0,0,D)])
    b.name = "Left_Seal"
    bodies.append(b)

    # ── 5. 우측  Right_Seal    (YZ offset, X=W) ──────────
    right_pl = offset_plane(root, yz, W)
    right_pl.name = "Right_Plane"
    b = make_rect_patch(root, right_pl,
        [pt(W,0,0), pt(W,H,0), pt(W,H,D), pt(W,0,D)])
    b.name = "Right_Seal"
    bodies.append(b)

    # ── 6. Stitch: 5개 패치 → 단일 Open Shell ─────────────
    body_col = adsk.core.ObjectCollection.create()
    for b in bodies:
        body_col.add(b)

    stitch_in = root.features.stitchFeatures.createInput(
        body_col,
        adsk.core.ValueInput.createByReal(0.005),   # tolerance 0.005 cm
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    stitch_feat = root.features.stitchFeatures.add(stitch_in)

    # Stitch 결과 바디에 이름 부여
    if stitch_feat.bodies.count > 0:
        stitch_feat.bodies.item(0).name = "Medicine_Envelope"

    return stitch_feat


# ─────────────────────────────────────────────────────────────
#  커맨드 핸들러
# ─────────────────────────────────────────────────────────────

class ExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            W = inputs.itemById("bag_width").value
            H = inputs.itemById("bag_height").value
            D = inputs.itemById("bag_depth").value

            app    = adsk.core.Application.get()
            ui     = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            root   = design.rootComponent

            build_envelope(root, W, H, D)

            ui.messageBox(
                "약봉투 생성 완료!\n\n"
                f"폭      : {W:.1f} cm\n"
                f"높이    : {H:.1f} cm\n"
                f"깊이    : {D:.1f} cm\n"
                f"상단    : 개방 (open)\n\n"
                "Bodies Browser에서\n"
                "'Medicine_Envelope' 확인"
            )

        except Exception:
            adsk.core.Application.get().userInterface.messageBox(
                "Execute 오류:\n" + traceback.format_exc()
            )


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd    = args.command
            inputs = cmd.commandInputs

            inputs.addValueInput(
                "bag_width",  "봉투 폭   (X)",
                "cm", adsk.core.ValueInput.createByReal(8.0)
            )
            inputs.addValueInput(
                "bag_height", "봉투 높이 (Y)",
                "cm", adsk.core.ValueInput.createByReal(12.0)
            )
            inputs.addValueInput(
                "bag_depth",  "봉투 깊이 (Z)",
                "cm", adsk.core.ValueInput.createByReal(2.0)   # 기본값 2cm
            )

            on_execute = ExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

        except Exception:
            adsk.core.Application.get().userInterface.messageBox(
                "CommandCreated 오류:\n" + traceback.format_exc()
            )


# ─────────────────────────────────────────────────────────────
#  Add-In 진입점
# ─────────────────────────────────────────────────────────────

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP
        )
        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        workspace = ui.workspaces.itemById(WORKSPACE)
        panel     = workspace.toolbarPanels.itemById(PANEL_ID)
        panel.controls.addCommand(cmd_def, "", False)

    except Exception:
        if ui:
            ui.messageBox("run() 오류:\n" + traceback.format_exc())


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        workspace = ui.workspaces.itemById(WORKSPACE)
        panel     = workspace.toolbarPanels.itemById(PANEL_ID)
        ctrl      = panel.controls.itemById(CMD_ID)
        if ctrl:
            ctrl.deleteMe()

        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        _handlers.clear()

    except Exception:
        if ui:
            ui.messageBox("stop() 오류:\n" + traceback.format_exc())
