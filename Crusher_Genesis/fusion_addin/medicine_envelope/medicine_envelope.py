"""
Fusion 360 Add-In
약봉투 (Medicine Envelope) — Surface Model Generator

툴바 위치: Surface 탭 > Create 패널 > "약봉투 생성"
기능:
  - 파라미터 다이얼로그(폭/높이/깊이) 입력
  - 5개 Surface Patch 자동 생성
    Front_Panel / Back_Panel / Left_Seal / Right_Seal / Bottom_Seal
"""

import adsk.core
import adsk.fusion
import traceback

# Add-In 생애주기 동안 핸들러 참조를 유지 (GC 방지)
_handlers = []

CMD_ID      = "medicine_envelope_cmd"
CMD_NAME    = "약봉투 생성"
CMD_TOOLTIP = "약봉투 Surface Model(5개 패치)을 생성합니다.\nGenesis PBD 시뮬레이션용."
PANEL_ID    = "SurfaceCreatePanel"        # Surface 탭 > Create
WORKSPACE   = "FusionSolidEnvironment"


# ─────────────────────────────────────────────────────────────
#  헬퍼 함수
# ─────────────────────────────────────────────────────────────

def pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def make_rect_patch(root, plane, corners, name):
    """corners: [p0,p1,p2,p3] 순서로 사각형 서피스 패치 생성."""
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


def build_envelope(root, W, H, D):
    """5개 Surface Patch로 약봉투 생성."""
    xy = root.xYConstructionPlane
    xz = root.xZConstructionPlane
    yz = root.yZConstructionPlane

    def offset_plane(base, dist):
        pi = root.constructionPlanes.createInput()
        pi.setByOffset(base, adsk.core.ValueInput.createByReal(dist))
        return root.constructionPlanes.add(pi)

    # 1. 앞면  Front_Panel   Z = 0
    make_rect_patch(root, xy,
        [pt(0,0,0), pt(W,0,0), pt(W,H,0), pt(0,H,0)],
        "Front_Panel")

    # 2. 뒷면  Back_Panel    Z = D
    back_pl = offset_plane(xy, D)
    back_pl.name = "Back_Plane"
    make_rect_patch(root, back_pl,
        [pt(0,0,D), pt(W,0,D), pt(W,H,D), pt(0,H,D)],
        "Back_Panel")

    # 3. 하단  Bottom_Seal   Y = 0
    make_rect_patch(root, xz,
        [pt(0,0,0), pt(W,0,0), pt(W,0,D), pt(0,0,D)],
        "Bottom_Seal")

    # 4. 좌측  Left_Seal     X = 0
    make_rect_patch(root, yz,
        [pt(0,0,0), pt(0,H,0), pt(0,H,D), pt(0,0,D)],
        "Left_Seal")

    # 5. 우측  Right_Seal    X = W
    right_pl = offset_plane(yz, W)
    right_pl.name = "Right_Plane"
    make_rect_patch(root, right_pl,
        [pt(W,0,0), pt(W,H,0), pt(W,H,D), pt(W,0,D)],
        "Right_Seal")


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
            design = adsk.fusion.Design.cast(app.activeProduct)
            root   = design.rootComponent

            build_envelope(root, W, H, D)

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

            # ── 파라미터 입력 다이얼로그 ──────────────────────
            inputs.addValueInput(
                "bag_width", "봉투 폭 (X)",
                "cm", adsk.core.ValueInput.createByReal(8.0)
            )
            inputs.addValueInput(
                "bag_height", "봉투 높이 (Y)",
                "cm", adsk.core.ValueInput.createByReal(12.0)
            )
            inputs.addValueInput(
                "bag_depth", "내부 깊이 (Z)",
                "cm", adsk.core.ValueInput.createByReal(1.0)
            )

            # Execute 핸들러 연결
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

        # 기존 커맨드 정의 제거 (재로드 대비)
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()

        # 커맨드 버튼 정의
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP
        )

        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        # Surface 탭 > Create 패널에 버튼 추가
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

        # 툴바에서 컨트롤 제거
        workspace = ui.workspaces.itemById(WORKSPACE)
        panel     = workspace.toolbarPanels.itemById(PANEL_ID)
        ctrl      = panel.controls.itemById(CMD_ID)
        if ctrl:
            ctrl.deleteMe()

        # 커맨드 정의 제거
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        _handlers.clear()

    except Exception:
        if ui:
            ui.messageBox("stop() 오류:\n" + traceback.format_exc())
