# -*- coding: utf-8 -*-
"""
fusion2xml — Fusion 360 디자인을 MuJoCo MJCF(.xml) 로 바로 내보내는 Add-In.

fusion2urdf 와의 차이
--------------------
URDF 에는 equality constraint 가 없다. 링크마다 부모가 정확히 하나인 트리만
표현할 수 있어서, 폐루프 기구(커넥팅로드-크랭크 같은)는 원리적으로 담을 수 없다.
fusion2urdf 는 루프를 닫는 조인트를 URDF 에서 빼 사이드카 파일에 좌표만 적어
두고, 사람이 MJCF 로 손으로 옮겨야 했다.

이 Add-In 은 처음부터 MJCF 를 쓴다. `<body>` 중첩은 여전히 트리지만, 루프를
닫는 조인트는 같은 XML 안의 `<equality>` 로 닫히므로 손으로 옮길 게 없다.

**어느 두 링크를 equality 로 묶을지는 대화상자에서 직접 고른다.** 자동 감지도
하지만(무향 그래프에서 사이클을 찾으면 그 조인트를 자동으로 뺀다), 아직 Fusion
에서 루프를 안 닫아 둔 상태에서는 감지될 게 없으므로 수동 지정이 필요하다.

anchor 는 두 링크 사이에 조인트가 있으면 거기서 가져오고(정확), 없으면 사용자가
직접 입력한 root 좌표를 쓴다.

툴바: Utilities(또는 Tools) 탭 > ADD-INS 패널 > "Fusion2MJCF 내보내기"
"""

import adsk
import adsk.core
import adsk.fusion
import traceback
import os

from .core import Tree, Mjcf, Extract, Mesh

_handlers = []

CMD_ID = 'fusion2xml_export_cmd'
CMD_NAME = 'Fusion2MJCF 내보내기'
CMD_TOOLTIP = ('Fusion 디자인을 MuJoCo MJCF(.xml) 로 내보냅니다.\n'
               '폐루프를 닫을 두 링크를 골라 equality constraint 로 묶을 수 있습니다.')
WORKSPACE = 'FusionSolidEnvironment'
FALLBACK_PANEL_IDS = ['SolidScriptsAddinsPanel', 'AddInsPanel',
                      'UtilityAddinsPanel', 'ScriptsAddinsPanel']
CUSTOM_PANEL_ID = 'fusion2xml_panel'
CUSTOM_PANEL_NAME = 'Fusion2MJCF'

NONE_LABEL = '(지정 안 함)'


# ── 대화상자 입력 파싱 ──────────────────────────────────────────────────────

def parse_extra_pairs(text, known_links):
    """'A:B:connect, C:D' 형식을 [(b1, b2, type|None), ...] 로. (pairs, errors)"""
    pairs, errors = [], []
    for chunk in (text or '').split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(':')]
        if len(parts) < 2:
            errors.append("'{}' 는 'body1:body2' 또는 'body1:body2:connect' "
                          '형식이어야 한다.'.format(chunk))
            continue
        b1, b2 = parts[0], parts[1]
        eq = parts[2].lower() if len(parts) > 2 and parts[2] else None
        if eq not in (None, 'connect', 'weld'):
            errors.append("'{}' 의 equality 종류 '{}' 는 connect 나 weld 여야 한다."
                          .format(chunk, parts[2]))
            continue
        for b in (b1, b2):
            if b not in known_links:
                errors.append("'{}' 라는 링크가 없다.".format(b))
        if b1 == b2:
            errors.append("'{}' 는 같은 링크끼리 묶으려 한다.".format(chunk))
        pairs.append((b1, b2, eq))
    return pairs, errors


def parse_anchor(text):
    """'0.1 0.02 0.3' -> [0.1, 0.02, 0.3]. 비어 있으면 None. (anchor, error)"""
    text = (text or '').strip()
    if not text:
        return None, None
    parts = text.replace(',', ' ').split()
    if len(parts) != 3:
        return None, 'anchor 는 숫자 3개(root 기준 m)여야 한다: 예) 0.1 0.02 0.3'
    try:
        return [float(p) for p in parts], None
    except ValueError:
        return None, "anchor 를 숫자로 읽지 못했다: '{}'".format(text)


def detect_loops(joints):
    """자동으로 감지되는 폐루프 폐합 조인트 이름들."""
    _, loops, _, _ = Tree.reorient_to_root(joints)
    return sorted(loops)


# ── 실행 ────────────────────────────────────────────────────────────────────

def _report(model_name, plan, warnings, exported, failed, save_dir):
    L = ['모델: {}'.format(model_name), '출력: {}'.format(save_dir), '']
    if plan['equalities']:
        L.append('[equality constraint {}개]'.format(len(plan['equalities'])))
        for e in plan['equalities']:
            L.append('  <{} body1="{}" body2="{}">'.format(
                e['type'], e['body1'], e['body2']))
            L.append('     anchor(root) = {}'.format(e['anchor_root']))
            L.append('     {}'.format(e['source']))
            L.append('     {}'.format(e['why']))
    else:
        L.append('[equality constraint] 없음 — 폐루프가 감지되지 않았고 수동 '
                 '지정도 없었다. 폐루프 기구라면 대화상자에서 두 링크를 직접 '
                 '골라야 한다.')
    L.append('')
    if plan['notices']:
        L.append('[알림]')
        L += ['  - ' + n for n in plan['notices']]
        L.append('')
    if warnings:
        L.append('[경고 {}개]'.format(len(warnings)))
        L += ['  - ' + w for w in warnings]
        L.append('')
    L.append('[메시] 내보냄 {}개{}'.format(
        len(exported), (' / 실패 {}개: {}'.format(len(failed), ', '.join(failed))
                        if failed else '')))
    return '\n'.join(L)


def do_export(ui, design, inputs):
    root = design.rootComponent
    model_name = Extract.sanitize(root.name.split()[0])

    joints, warns = Extract.collect_joints(root)
    if not joints:
        ui.messageBox('조인트를 찾지 못했습니다.', CMD_NAME)
        return
    inertia, w2 = Extract.collect_inertia(root)
    warns += w2
    if 'base_link' not in inertia:
        ui.messageBox("base_link 가 없습니다. 루트가 될 컴포넌트의 이름을 "
                      "'base_link' 로 바꾸고 다시 실행하세요.", CMD_NAME)
        return

    links = Extract.link_names(root)

    # 사용자가 고른 equality 쌍
    forced = []
    b1 = inputs.itemById('body1').selectedItem.name
    b2 = inputs.itemById('body2').selectedItem.name
    if b1 != NONE_LABEL and b2 != NONE_LABEL:
        if b1 == b2:
            ui.messageBox('equality 로 묶을 두 링크가 같습니다.', CMD_NAME)
            return
        eq_sel = inputs.itemById('eq_type').selectedItem.name
        eq = None if eq_sel.startswith('자동') else eq_sel
        anchor, err = parse_anchor(inputs.itemById('anchor').value)
        if err:
            ui.messageBox(err, CMD_NAME)
            return
        forced.append((b1, b2, eq, anchor))
    elif (b1 == NONE_LABEL) != (b2 == NONE_LABEL):
        ui.messageBox('equality 로 묶을 링크를 하나만 골랐습니다. 둘 다 고르거나 '
                      '둘 다 비워 두세요.', CMD_NAME)
        return

    extra, errs = parse_extra_pairs(inputs.itemById('extra_pairs').value, links)
    if errs:
        ui.messageBox('추가 쌍 입력에 문제가 있습니다:\n\n  ' + '\n  '.join(errs), CMD_NAME)
        return
    forced += extra

    plan = Tree.plan(joints, forced)
    if plan['problems']:
        ui.messageBox('MJCF 를 만들 수 없습니다:\n\n  ' +
                      '\n  '.join(plan['problems']), CMD_NAME)
        return

    save_dir = _ask_folder(ui, model_name)
    if not save_dir:
        ui.messageBox('취소되었습니다 — 파일은 생성되지 않았습니다.', CMD_NAME)
        return

    export_meshes = inputs.itemById('export_meshes').value
    options = {
        'fix_base': inputs.itemById('fix_base').value,
        'actuator': inputs.itemById('actuator').selectedItem.name,
    }

    # 메시를 먼저 내보내야 어떤 링크에 STL 이 있는지 알고 <asset> 을 쓸 수 있다.
    # copy_occs 는 디자인을 실제로 바꾸므로 되돌리라고 알려야 한다.
    exported, failed = set(), []
    if export_meshes:
        Mesh.copy_occs(root)
        exported, failed = Mesh.export_stl(design, save_dir, design.allComponents)

    xml, w3 = Mjcf.build(model_name, plan, inertia,
                         mesh_names=exported if export_meshes else links,
                         options=options)
    warns += w3

    xml_path = os.path.join(save_dir, model_name + '.xml')
    with open(xml_path, 'w', encoding='utf-8') as f:
        f.write(xml)

    report = _report(model_name, plan, warns, exported, failed, save_dir)
    with open(os.path.join(save_dir, model_name + '.report.txt'),
              'w', encoding='utf-8') as f:
        f.write(report)

    tail = ''
    if not _is_ascii(xml_path):
        tail += ('\n\n[주의] 저장 경로에 한글/비ASCII 문자가 있습니다. MuJoCo 가 '
                 '이런 경로의 메시를 못 읽는 경우가 있으니 문제가 생기면 ASCII '
                 '경로로 옮기세요.')
    if export_meshes:
        tail += ('\n\n[주의] 메시를 내보내느라 디자인의 컴포넌트를 복제했습니다. '
                 '저장하지 말고 Ctrl+Z 로 되돌리세요.')

    ui.messageBox('MJCF 저장 완료:\n  {}\n\n{}{}'.format(
        os.path.basename(xml_path), report, tail), CMD_NAME)


def _is_ascii(s):
    try:
        s.encode('ascii')
        return True
    except UnicodeEncodeError:
        return False


def _ask_folder(ui, model_name):
    dlg = ui.createFolderDialog()
    dlg.title = 'MJCF 를 저장할 폴더를 선택하세요'
    if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
        return None
    save_dir = os.path.join(dlg.folder, model_name + '_mjcf')
    try:
        os.mkdir(save_dir)
    except Exception:
        pass
    return save_dir


# ── UI ──────────────────────────────────────────────────────────────────────

class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox('활성 문서가 Fusion 360 디자인이 아닙니다.', CMD_NAME)
                return
            do_export(ui, design, args.command.commandInputs)
        except Exception:
            ui.messageBox('내보내기 오류:\n' + traceback.format_exc(), CMD_NAME)


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            cmd = args.command
            cmd.setDialogInitialSize(520, 620)
            inputs = cmd.commandInputs

            design = adsk.fusion.Design.cast(app.activeProduct)
            links, auto = [], []
            if design:
                root = design.rootComponent
                links = Extract.link_names(root)
                joints, _ = Extract.collect_joints(root)
                auto = detect_loops(joints)

            note = ('자동 감지된 폐루프 폐합 조인트: '
                    + (', '.join(auto) if auto else '없음'))
            inputs.addTextBoxCommandInput(
                'info', '', note + '\n\n감지된 것은 자동으로 equality 로 나갑니다. '
                'Fusion 에서 아직 루프를 안 닫아 뒀다면 아래에서 두 링크를 직접 '
                '고르세요.', 4, True)

            d1 = inputs.addDropDownCommandInput(
                'body1', 'equality body1', adsk.core.DropDownStyles.TextListDropDownStyle)
            d2 = inputs.addDropDownCommandInput(
                'body2', 'equality body2', adsk.core.DropDownStyles.TextListDropDownStyle)
            for d in (d1, d2):
                d.listItems.add(NONE_LABEL, True)
                for name in links:
                    d.listItems.add(name, False)

            eq = inputs.addDropDownCommandInput(
                'eq_type', 'equality 종류', adsk.core.DropDownStyles.TextListDropDownStyle)
            eq.listItems.add('자동(조인트 타입에서 고름)', True)
            eq.listItems.add('connect', False)
            eq.listItems.add('weld', False)

            inputs.addStringValueInput(
                'anchor', 'anchor (root 기준 m, 비우면 조인트에서)', '')
            inputs.addStringValueInput(
                'extra_pairs', '추가 쌍  A:B:connect, C:D:weld', '')

            inputs.addBoolValueInput('fix_base', 'base_link 를 world 에 고정',
                                     True, '', True)
            inputs.addBoolValueInput('export_meshes', 'STL 메시도 내보내기',
                                     True, '', True)

            act = inputs.addDropDownCommandInput(
                'actuator', '액추에이터', adsk.core.DropDownStyles.TextListDropDownStyle)
            for name in ('position', 'motor', 'none'):
                act.listItems.add(name, name == 'position')

            on_execute = ExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox('대화상자 생성 오류:\n' + traceback.format_exc(), CMD_NAME)


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
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP)
        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        workspace = ui.workspaces.itemById(WORKSPACE)
        if not workspace:
            ui.messageBox("workspace '{}' 를 찾을 수 없습니다.".format(WORKSPACE), CMD_NAME)
            return
        panel = _get_or_create_panel(workspace)
        if not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def, '', False)

        ui.messageBox("fusion2xml 로드 완료.\n\npanel = {}\n\n해당 패널에서 "
                      "'{}' 버튼을 찾으세요.".format(panel.id, CMD_NAME), CMD_NAME)
    except Exception:
        if ui:
            ui.messageBox('run() 오류:\n' + traceback.format_exc(), CMD_NAME)


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        workspace = ui.workspaces.itemById(WORKSPACE)
        panel = workspace.toolbarPanels.itemById(CUSTOM_PANEL_ID)
        if not panel:
            for pid in FALLBACK_PANEL_IDS:
                p = workspace.toolbarPanels.itemById(pid)
                if p and p.controls.itemById(CMD_ID):
                    panel = p
                    break
        ctrl = panel.controls.itemById(CMD_ID) if panel else None
        if ctrl:
            ctrl.deleteMe()
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        _handlers.clear()
    except Exception:
        if ui:
            ui.messageBox('stop() 오류:\n' + traceback.format_exc(), CMD_NAME)
