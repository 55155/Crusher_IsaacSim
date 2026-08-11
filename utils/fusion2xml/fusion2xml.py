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
    """추가 쌍 문법을 [(b1, b2, type|None, anchor|None), ...] 로. (pairs, errors)

        A:B                       종류 자동, anchor 는 조인트에서
        A:B:connect               종류 지정
        A:B:connect:0.1 0.02 0.3  anchor 까지 지정(root 기준 m)

    **anchor 칸이 핵심이다.** 예전에는 3칸짜리만 만들어서 `Tree.plan` 이
    anchor 를 항상 None 으로 받았다. 그래서 두 링크 사이에 Fusion 조인트가
    없으면 무조건 막혔는데, 하필 그게 수동 지정이 필요한 유일한 경우다
    (Fusion 에서 루프를 안 닫아 뒀을 때). 게다가 막으면서 내놓는 안내가
    "anchor 좌표를 직접 입력해야 한다" 였는데 입력할 칸이 없었다.
    """
    pairs, errors = [], []
    for chunk in (text or '').split(','):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(':')]
        if len(parts) < 2:
            errors.append("'{}' 는 'body1:body2', 'body1:body2:connect', "
                          "'body1:body2:connect:x y z' 중 하나여야 한다."
                          .format(chunk))
            continue
        if len(parts) > 4:
            errors.append("'{}' 에 칸이 너무 많다 — 최대 "
                          "'body1:body2:종류:x y z' 까지다.".format(chunk))
            continue

        bad = False
        b1, b2 = parts[0], parts[1]
        eq = parts[2].lower() if len(parts) > 2 and parts[2] else None
        if eq not in (None, 'connect', 'weld'):
            errors.append("'{}' 의 equality 종류 '{}' 는 connect 나 weld 여야 한다."
                          .format(chunk, parts[2]))
            continue

        anchor = None
        if len(parts) > 3 and parts[3]:
            anchor, err = parse_anchor(parts[3])
            if err:
                errors.append("'{}' 의 {}".format(chunk, err))
                bad = True

        for b in (b1, b2):
            if b not in known_links:
                errors.append("'{}' 라는 링크가 없다.".format(b))
                bad = True
        if b1 == b2:
            errors.append("'{}' 는 같은 링크끼리 묶으려 한다.".format(chunk))
            bad = True
        if bad:
            continue
        pairs.append((b1, b2, eq, anchor))
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

def _report(model_name, plan, warnings, exported, failed, save_dir, renames=(),
            joints=None):
    L = ['모델: {}'.format(model_name), '출력: {}'.format(save_dir), '']
    # 하위 컴포넌트 안의 조인트는 원점 계산이 의심스럽다. `_joint_origin` 은
    # `occurrenceTwo.transform` 을 쓰는데 그건 **그 occurrence 의 부모 프레임**
    # 기준이라, 최상위 조인트만 보던 fusion2urdf 에서는 root 와 같아서 맞았지만
    # 하위 컴포넌트에서는 root 기준이 아니다. 어느 조인트가 거기서 왔는지
    # 적어 둬야 접합 위치가 이상할 때 여기부터 의심할 수 있다.
    nested = sorted(n for n, j in (joints or {}).items()
                    if j.get('occurrence_path'))
    if nested:
        L.append('[하위 컴포넌트 조인트 {}개] 원점이 root 기준이 아닐 수 있다 — '
                 '접합 위치가 어긋나면 여기부터 확인하라.'.format(len(nested)))
        for n in nested:
            L.append('  {}  (경로: {},  xyz={})'.format(
                n, joints[n]['occurrence_path'], joints[n]['xyz']))
        L.append('')
    if renames:
        L.append('[이름 변경 {}개] MuJoCo 가 비ASCII 파일명을 못 열어서 바꿨다. '
                 'Fusion 에서 보이는 이름과 다르니 주의.'.format(len(renames)))
        for old, new in renames:
            L.append('  {}  ->  {}'.format(old, new))
        L.append('')
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
    L.append('[메시] 링크 {}개 중 {}개에 STL 이 붙었다{}'.format(
        len(plan['origins']), len(exported),
        (' / 실패 {}개: {}'.format(len(failed), ', '.join(failed))
         if failed else '')))
    no_mesh = sorted(set(plan['origins']) - set(exported))
    if no_mesh:
        L.append('  STL 이 없는 링크 {}개: {}'.format(
            len(no_mesh), ', '.join(no_mesh)))
    return '\n'.join(L)


def do_export(ui, design, inputs):
    root = design.rootComponent
    model_name = (inputs.itemById('model_name').value or '').strip()
    model_name = Extract.sanitize(model_name or root.name.split()[0])

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
    # copy_occs 가 원본 이름을 'old_component' 로 바꿔 버리므로 그 전에 모아 둔다.
    renames = Extract.collect_renames(root)

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
    act_joints = [s.strip() for s
                  in (inputs.itemById('actuator_joints').value or '')
                  .replace(',', ' ').split() if s.strip()]
    options = {
        'fix_base': inputs.itemById('fix_base').value,
        'actuator': inputs.itemById('actuator').selectedItem.name,
        'actuator_joints': act_joints or None,
        'add_collision': inputs.itemById('add_collision').value,
        'exclude_adjacent': inputs.itemById('exclude_adjacent').value,
    }

    # 메시를 먼저 내보내야 어떤 링크에 STL 이 있는지 알고 <asset> 을 쓸 수 있다.
    # copy_occs 는 디자인을 실제로 바꾸므로 되돌리라고 알려야 한다.
    #
    # body 트리에 실제로 들어간 링크만 내보낸다 — 예전처럼 디자인 전체를
    # 훑으면 하위 컴포넌트 원본이 새어 나와 죽은 asset 이 쌓이고, 그중 이름에
    # 한글이 섞이면 모델 전체가 로딩에 실패한다.
    wanted = set(plan['origins'])
    exported, failed = set(), []
    if export_meshes:
        Mesh.copy_occs(root)
        exported, failed = Mesh.export_stl(design, root, save_dir, wanted)
        mesh_names = exported
    else:
        # STL 을 안 내보낼 때는 "이미 이 폴더에 있겠거니" 하고 링크 전체를
        # <asset> 에 찍었었다. 폴더가 비어 있으면 로딩 안 되는 XML 이 조용히
        # 나간다 — 실제로 있는 파일만 쓴다.
        mesh_names = _existing_meshes(save_dir, wanted)
        warns.append('STL 을 내보내지 않았다 — 저장 폴더의 meshes/ 에 이미 있는 '
                     '{}개만 <asset> 에 넣었다.'.format(len(mesh_names)))

    xml, w3 = Mjcf.build(model_name, plan, inertia,
                         mesh_names=mesh_names, options=options)
    warns += w3

    xml_path = os.path.join(save_dir, model_name + '.xml')
    with open(xml_path, 'w', encoding='utf-8') as f:
        f.write(xml)

    report = _report(model_name, plan, warns, mesh_names, failed, save_dir,
                     renames=renames, joints=joints)
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


def _existing_meshes(save_dir, wanted):
    """`save_dir/meshes/` 에 실제로 있는 STL 중 트리에 쓰이는 링크 이름 집합."""
    mesh_dir = os.path.join(save_dir, 'meshes')
    try:
        files = os.listdir(mesh_dir)
    except Exception:
        return set()
    have = {os.path.splitext(f)[0] for f in files if f.lower().endswith('.stl')}
    return have & set(wanted)


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
            links, auto, jnames, default_name, posed = [], [], [], '', []
            if design:
                root = design.rootComponent
                links = Extract.link_names(root)
                joints, jwarns = Extract.collect_joints(root)
                auto = detect_loops(joints)
                jnames = sorted(n for n, j in joints.items()
                                if j['type'] != 'fixed')
                default_name = Extract.sanitize(root.name.split()[0])
                posed = [n for n, j in joints.items()
                         if abs(j.get('current_value') or 0.0) > 1e-6]

            note = ('자동 감지된 폐루프 폐합 조인트: '
                    + (', '.join(auto) if auto else '없음'))
            if jnames:
                note += '\n움직이는 관절: ' + ', '.join(jnames)
            # 자세가 0 이 아니면 여기서 먼저 막아 세운다 — 내보내고 나면
            # 조인트 원점이 그 자세로 굳어 기구 치수가 조용히 틀어진다.
            if posed:
                note = ('[경고] 조인트 {}개가 0 이 아닌 자세입니다 ({}). 그 자세가 '
                        'MJCF 의 기준 자세가 되고 조인트 원점과 리밋이 함께 밀립니다. '
                        '되돌린 뒤 다시 실행하세요.\n\n'.format(
                            len(posed), ', '.join(posed)) + note)
            inputs.addTextBoxCommandInput(
                'info', '', note + '\n\n감지된 것은 자동으로 equality 로 나갑니다. '
                'Fusion 에서 아직 루프를 안 닫아 뒀다면 아래에서 두 링크를 직접 '
                '고르세요.', 7, True)

            inputs.addStringValueInput('model_name', '모델 이름 (파일명)',
                                       default_name)

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
                'extra_pairs', '추가 쌍  A:B:connect:x y z, C:D:weld', '')

            inputs.addBoolValueInput('fix_base', 'base_link 를 world 에 고정',
                                     True, '', True)
            inputs.addBoolValueInput('export_meshes', 'STL 메시도 내보내기',
                                     True, '', True)
            inputs.addBoolValueInput('add_collision', '충돌용 geom 도 생성',
                                     True, '', True)
            inputs.addBoolValueInput(
                'exclude_adjacent', '인접/equality 쌍의 자기충돌 제외', True, '', True)

            act = inputs.addDropDownCommandInput(
                'actuator', '액추에이터', adsk.core.DropDownStyles.TextListDropDownStyle)
            for name in ('position', 'motor', 'none'):
                act.listItems.add(name, name == 'position')
            inputs.addStringValueInput(
                'actuator_joints', '구동할 관절 (비우면 전부)', '')

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
