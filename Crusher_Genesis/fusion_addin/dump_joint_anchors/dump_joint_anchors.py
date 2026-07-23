# -*- coding: utf-8 -*-
"""
Fusion 360 Add-In — Dump Joint Anchors

폐루프 어셈블리에서 하나의 부품이 두 개의 joint에 물려 있으면(예: Crank가
PULLEY와 M_Top 양쪽에 연결), fusion2urdf는 URDF 트리 제약 때문에 그 부품을
링크 2개로 쪼개 export한다. 이 add-in은 그 원본 부품(Fusion 상에서는 여전히
occurrence 1개)에 물린 모든 joint의 원점을 월드 좌표와 그 부품 자신의 로컬
좌표(=MJCF <equality><connect anchor="..."/></equality>에 바로 쓸 값) 양쪽으로
뽑고, anchor가 2개면 서로 간 거리(=커넥팅로드/크랭크의 실제 핀-핀 길이)까지
계산해서 보여준다.

같은 로직의 단발성 Script 버전은 utills/dump_joint_anchors.py 에 있다 — 매번
켤 때마다 Scripts 탭에서 파일을 추가하기 싫고, 항상 떠 있는 툴바 버튼이
필요하면 이 Add-In(폴더 통째로 Fusion Add-Ins 폴더에 설치)을 쓴다.

툴바: Tools 탭 > ADD-INS 패널 > "Joint Anchor 추출"
"""

import adsk.core, adsk.fusion, traceback
import math
import os

_handlers = []

_LOG_PATH = os.path.join(os.path.expanduser("~"), "dump_joint_anchors_debug.log")


def _log(msg):
    """run()/stop()이 도중에 조용히 멈출 때 어디까지 진행됐는지 추적하기 위한
    파일 로그 -- messageBox조차 못 뜨는 극단적인 상황에서도 확인 가능하도록."""
    try:
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass

CMD_ID = "dump_joint_anchors_cmd"
CMD_NAME = "Joint Anchor 추출"
CMD_TOOLTIP = (
    "폐루프에서 중복 export되는 부품의 joint anchor를 월드/로컬 좌표로 뽑고,\n"
    "핀-핀 거리(로드 길이)를 계산합니다. MJCF <connect anchor=.../> 값을 바로 얻는 도구."
)
PANEL_ID = "SolidScriptsAddinsPanel"
WORKSPACE = "FusionSolidEnvironment"

DEFAULT_PATTERN = ""  # 빈 문자열 = 디자인 전체 대상 (특정 부품만 보려면 이름 입력)


# ─────────────────────────────────────────────────────────────
#  핵심 로직 (utills/dump_joint_anchors_core.py 와 동일 -- add-in은 어디에
#  설치되든 동작해야 하므로 외부 프로젝트 경로에 의존하지 않게 자체 포함)
# ─────────────────────────────────────────────────────────────

CSV_COLUMNS = [
    "record_type", "component", "joint", "occurrence_path", "tag", "other_component",
    "world_x_m", "world_y_m", "world_z_m",
    "local_x_m", "local_y_m", "local_z_m",
    "distance_m", "anchor_vec_x_m", "anchor_vec_y_m", "anchor_vec_z_m",
]


def _mat_to_local(occ, world_point):
    transform = occ.transform2.copy()
    ok = transform.invert()
    if not ok:
        return None
    p = world_point.copy()
    p.transformBy(transform)
    return p


def _iter_all_joints(comp, occ_path=""):
    for j in comp.joints:
        yield j, occ_path
    for occ in comp.occurrences:
        new_path = f"{occ_path}/{occ.name}" if occ_path else occ.name
        yield from _iter_all_joints(occ.component, new_path)


def collect_records(design, name_pattern: str):
    """name_pattern이 "", "*", "all"이면 필터 없이 디자인 전체를 대상으로 한다."""
    root = design.rootComponent
    name_pattern_lower = name_pattern.strip().lower()
    match_all = name_pattern_lower in ("", "*", "all")

    all_occs = root.allOccurrences
    if match_all:
        target_occs = list(all_occs)
    else:
        target_occs = [occ for occ in all_occs if name_pattern_lower in occ.component.name.lower()]
    target_comp_names = {occ.component.name for occ in target_occs}

    all_joints = list(_iter_all_joints(root))

    records = []
    anchors_by_component = {}

    for joint, path in all_joints:
        occ_one = joint.occurrenceOne
        occ_two = joint.occurrenceTwo
        comp_one_name = occ_one.component.name if occ_one else root.name
        comp_two_name = occ_two.component.name if occ_two else root.name

        if comp_one_name not in target_comp_names and comp_two_name not in target_comp_names:
            continue

        pairs = (
            ("One", joint.geometryOrOriginOne, occ_one, comp_one_name, comp_two_name),
            ("Two", joint.geometryOrOriginTwo, occ_two, comp_two_name, comp_one_name),
        )
        for tag, geo, occ, comp_name, other_name in pairs:
            try:
                world_pt = geo.origin
            except Exception:
                continue

            local_pt = None
            if comp_name in target_comp_names and occ is not None:
                local_pt = _mat_to_local(occ, world_pt)
                if local_pt is not None:
                    anchors_by_component.setdefault(comp_name, []).append((joint.name, local_pt))

            records.append({
                "record_type": "anchor",
                "component": comp_name,
                "joint": joint.name,
                "occurrence_path": path,
                "tag": tag,
                "other_component": other_name,
                "world_x_m": world_pt.x * 0.01,
                "world_y_m": world_pt.y * 0.01,
                "world_z_m": world_pt.z * 0.01,
                "local_x_m": local_pt.x * 0.01 if local_pt else "",
                "local_y_m": local_pt.y * 0.01 if local_pt else "",
                "local_z_m": local_pt.z * 0.01 if local_pt else "",
                "distance_m": "",
                "anchor_vec_x_m": "", "anchor_vec_y_m": "", "anchor_vec_z_m": "",
            })

    for comp_name, pts in anchors_by_component.items():
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                p1, p2 = pts[i][1], pts[j][1]
                dx, dy, dz = (p2.x - p1.x) * 0.01, (p2.y - p1.y) * 0.01, (p2.z - p1.z) * 0.01
                d = math.sqrt(dx * dx + dy * dy + dz * dz)
                records.append({
                    "record_type": "distance",
                    "component": comp_name,
                    "joint": f"{pts[i][0]} <-> {pts[j][0]}",
                    "occurrence_path": "", "tag": "", "other_component": "",
                    "world_x_m": "", "world_y_m": "", "world_z_m": "",
                    "local_x_m": "", "local_y_m": "", "local_z_m": "",
                    "distance_m": d,
                    "anchor_vec_x_m": dx, "anchor_vec_y_m": dy, "anchor_vec_z_m": dz,
                })

    return records, target_occs, len(all_joints)


def write_csv(records, filepath):
    import csv
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in records:
            w.writerow(r)


def _summary_text(records, name_pattern, target_occs, n_joints):
    anchor_rows = [r for r in records if r["record_type"] == "anchor"]
    dist_rows = [r for r in records if r["record_type"] == "distance"]

    lines = []
    lines.append(f"'{name_pattern}' 매칭 occurrence {len(target_occs)}개 / 전체 joint {n_joints}개")
    lines.append(f"anchor 레코드 {len(anchor_rows)}개, distance 레코드 {len(dist_rows)}개")
    lines.append("")

    if dist_rows:
        lines.append("[핀-핀 거리 / MJCF anchor 벡터]")
        for r in dist_rows[:30]:
            lines.append(
                f"  [{r['component']}] {r['joint']}\n"
                f"    거리={r['distance_m']:.6f} m\n"
                f"    anchor=({r['anchor_vec_x_m']:.6f}, {r['anchor_vec_y_m']:.6f}, {r['anchor_vec_z_m']:.6f})"
            )
        if len(dist_rows) > 30:
            lines.append(f"  ... 외 {len(dist_rows) - 30}개 더 (전체 내용은 CSV 참고)")
    else:
        lines.append("(같은 컴포넌트에 joint가 2개 이상 물린 경우가 없어 거리 계산 결과 없음)")
        lines.append("")
        lines.append("[개별 anchor 좌표(최대 10개까지 표시)]")
        for r in anchor_rows[:10]:
            local = (f"local=({r['local_x_m']:.6f}, {r['local_y_m']:.6f}, {r['local_z_m']:.6f})"
                     if r["local_x_m"] != "" else "local=(대상 컴포넌트 아님)")
            lines.append(f"  joint={r['joint']} tag={r['tag']} {local}")

    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────
#  UI (커맨드 다이얼로그: 컴포넌트 이름 패턴 입력 -> 실행)
# ─────────────────────────────────────────────────────────────

class ExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            inputs = args.command.commandInputs
            name_pattern = inputs.itemById("name_pattern").value

            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox("활성 문서가 Fusion 360 디자인이 아닙니다.")
                return

            records, target_occs, n_joints = collect_records(design, name_pattern)
            if not records:
                ui.messageBox(
                    f"'{name_pattern}' 을 이름에 포함하는 컴포넌트에 물린 joint를 "
                    f"찾지 못했습니다.\n매칭된 occurrence 수: {len(target_occs)}\n"
                    f"디자인 전체 joint 수: {n_joints}"
                )
                return

            # 1차 출력: 항상 뜨는 messageBox (Text Commands 팔레트는 환경에 따라
            # 숨겨져 있거나 다른 창 뒤에 있어서 못 볼 수 있어 이걸 우선함)
            summary = _summary_text(records, name_pattern, target_occs, n_joints)
            ui.messageBox(summary, "Joint Anchor 결과 요약")

            try:
                text_palette = ui.palettes.itemById("TextCommands")
                if text_palette:
                    text_palette.isVisible = True
                    text_palette.writeText(summary)
            except Exception:
                pass

            save_dlg = ui.createFileDialog()
            save_dlg.title = "joint_anchors_report.csv 저장 위치 선택"
            save_dlg.filter = "CSV files (*.csv)"
            save_dlg.initialFilename = f"joint_anchors_{name_pattern.strip()}.csv"
            if save_dlg.showSave() == adsk.core.DialogResults.DialogOK:
                write_csv(records, save_dlg.filename)
                ui.messageBox(f"CSV 저장 완료: {save_dlg.filename}\n레코드 {len(records)}개")
            else:
                ui.messageBox("저장 취소됨 — CSV 파일은 생성되지 않았습니다.\n"
                               "(위에서 본 요약 대화상자에 핵심 결과는 이미 표시됨)")

        except Exception:
            ui.messageBox("Execute 오류:\n" + traceback.format_exc())


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            inputs.addStringValueInput(
                "name_pattern", "대상 컴포넌트 이름(부분 문자열, 비워두면 전체)", DEFAULT_PATTERN
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

# Fusion 버전에 따라 ADD-INS 패널 ID가 다르거나 없을 수 있어, 여러 후보를
# 순서대로 시도하고 전부 없으면 새 패널을 만든다(어떤 버전에서도 버튼이
# 반드시 뜨도록).
FALLBACK_PANEL_IDS = [
    "SolidScriptsAddinsPanel",
    "AddInsPanel",
    "UtilityAddinsPanel",
    "ScriptsAddinsPanel",
]
CUSTOM_PANEL_ID = "dump_joint_anchors_panel"
CUSTOM_PANEL_NAME = "Joint Anchor Tools"


def _get_or_create_panel(workspace):
    for pid in FALLBACK_PANEL_IDS:
        panel = workspace.toolbarPanels.itemById(pid)
        if panel:
            return panel
    existing = workspace.toolbarPanels.itemById(CUSTOM_PANEL_ID)
    if existing:
        return existing
    return workspace.toolbarPanels.add(CUSTOM_PANEL_ID, CUSTOM_PANEL_NAME)


def run(context):
    ui = None
    _log("=" * 40)
    _log("run() 시작")
    try:
        app = adsk.core.Application.get()
        _log("1/6 Application.get() 성공")
        ui = app.userInterface
        _log("2/6 userInterface 획득 성공")

        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
            _log("   (기존 commandDefinition 삭제)")

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP
        )
        _log("3/6 commandDefinition 생성 성공")
        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        _log("4/6 commandCreated 핸들러 등록 성공")

        workspace = ui.workspaces.itemById(WORKSPACE)
        if not workspace:
            _log(f"!! workspace '{WORKSPACE}' 를 찾지 못함 -> 중단")
            ui.messageBox(f"workspace '{WORKSPACE}' 를 찾을 수 없습니다.\n로그: {_LOG_PATH}")
            return
        _log(f"5/6 workspace 확보: {WORKSPACE}")

        panel = _get_or_create_panel(workspace)
        if not panel:
            _log("!! 패널을 확보하지 못함 -> 중단")
            ui.messageBox(f"패널을 확보하지 못했습니다.\n로그: {_LOG_PATH}")
            return
        _log(f"6/6 panel 확보: id={panel.id}")

        if not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def, "", False)
            _log("   버튼 추가 완료")
        else:
            _log("   버튼이 이미 존재함(재사용)")

        _log("run() 정상 종료")
        ui.messageBox(
            f"dump_joint_anchors 로드 완료.\n\n"
            f"workspace = {WORKSPACE}\n"
            f"panel     = {panel.id}\n\n"
            f"해당 workspace/panel에서 '{CMD_NAME}' 버튼을 찾아보세요.\n"
            f"(패널 ID가 낯설면 새로 만든 커스텀 패널일 수 있습니다 -- "
            f"리본의 아무 탭이나 끝까지 훑어보세요.)\n\n"
            f"로그 파일: {_LOG_PATH}"
        )

    except Exception:
        tb = traceback.format_exc()
        _log("!! 예외 발생:\n" + tb)
        if ui:
            ui.messageBox("run() 오류:\n" + tb)


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        workspace = ui.workspaces.itemById(WORKSPACE)
        panel = workspace.toolbarPanels.itemById(CUSTOM_PANEL_ID)
        if not panel:
            for pid in FALLBACK_PANEL_IDS:
                panel = workspace.toolbarPanels.itemById(pid)
                if panel and panel.controls.itemById(CMD_ID):
                    break
            else:
                panel = None
        ctrl = panel.controls.itemById(CMD_ID) if panel else None
        if ctrl:
            ctrl.deleteMe()

        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        _handlers.clear()
    except Exception:
        if ui:
            ui.messageBox("stop() 오류:\n" + traceback.format_exc())
