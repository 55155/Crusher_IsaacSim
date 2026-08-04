# -*- coding: utf-8 -*-
"""
Constraint.py — 폐루프(closed kinematic loop)를 닫는 조인트를 URDF 조인트가 아니라
**equality constraint** 로 내보낸다.

배경
----
URDF 는 운동학 트리만 표현할 수 있다(링크마다 부모가 정확히 하나). 그런데
`Write.write_link_urdf` 는 **조인트마다 링크를 하나씩** 찍는다:

    for joint in joints_dict:
        name = joints_dict[joint]['child']      # <- 같은 child 가 두 번 나오면 링크도 두 번

그래서 폐루프가 있는 기구(크랭크-슬라이더 등)를 내보내면 루프를 닫는 링크가
**같은 이름으로 두 번** 정의된 불법 URDF 가 나온다. 실제 사례: 회수장치2 의
`Crank_1` 이 `회전 29`(부모 PULLEY_1) 와 `회전 34`(부모 M_Top_1) 양쪽의 child 라
link 요소 34개 / 고유 이름 33개가 됐다.

해결
----
루프를 닫는 조인트를 **URDF 에서 빼고**, 대신 사이드카 파일에 기록해 downstream
(MJCF 변환)에서 `<equality><connect>` 로 만들게 한다. MuJoCo 의 connect 는 링크를
복제하지 않고 기존 두 body 를 직접 묶을 수 있으므로, 트리에서 조인트 하나만
빼면 URDF 는 정상 트리가 되고 루프 정보는 보존된다.

    <equality>
      <connect name="..." body1="<child>" body2="<parent>" anchor="<xyz>"/>
    </equality>

이 모듈의 순수 함수(detect_loop_joints / split_joints / build_constraint_records /
render_*)는 `adsk` 에 의존하지 않는다 — Fusion 밖에서 단위 테스트 가능.
UI 를 쓰는 select_constraint_joints 만 Fusion 안에서 동작한다.
"""

import json
import os


def detect_loop_joints(joints_dict):
    """child 가 두 번 이상 등장하는 조인트 중 **두 번째 이후**를 돌려준다.

    첫 등장은 트리 경로로 남기고, 나머지가 루프를 닫는 조인트다.
    dict 순서(=Fusion 의 조인트 순서)가 기준이므로, 어느 쪽을 트리로 삼을지
    바꾸고 싶으면 select_constraint_joints 에서 사용자가 직접 고르면 된다.

    Returns
    -------
    list[str] : 루프를 닫는 조인트 이름들
    """
    seen = set()
    loop = []
    for name, j in joints_dict.items():
        child = j['child']
        if child in seen:
            loop.append(name)
        else:
            seen.add(child)
    return loop


def split_joints(joints_dict, constraint_names):
    """joints_dict 를 (트리용, 제약용) 두 개로 나눈다. 원본은 안 건드린다."""
    constraint_names = set(constraint_names)
    tree = {k: v for k, v in joints_dict.items() if k not in constraint_names}
    constraints = {k: v for k, v in joints_dict.items() if k in constraint_names}
    return tree, constraints


def validate_tree(tree_joints, base_link='base_link'):
    """제약을 뺀 뒤 정말 트리가 됐는지 검사한다.

    Returns
    -------
    (ok: bool, problems: list[str])
    """
    problems = []
    parents = {}
    for name, j in tree_joints.items():
        child = j['child']
        if child in parents:
            problems.append(
                "링크 '{}' 의 부모가 아직 둘이다 ('{}', '{}') — 이 조인트도 제약으로 "
                "지정해야 한다.".format(child, parents[child], name))
        parents[child] = name
        if child == base_link:
            problems.append("base_link 가 조인트 '{}' 의 child 다 — 루트가 될 수 없다.".format(name))
    return (not problems), problems


_ROT_TYPES = ('revolute', 'continuous')


def loop_joint_names(joints_dict, loop_child):
    """loop_child 로 들어오는 두 경로(공통 조상까지)를 이루는 조인트 이름들."""
    into = {}
    for n, j in joints_dict.items():
        into.setdefault(j['child'], []).append(n)
    paths = []
    for start in into.get(loop_child, []):
        chain, cur = [], start
        seen = set()
        while cur and cur not in seen:
            seen.add(cur)
            chain.append(cur)
            ins = into.get(joints_dict[cur]['parent'], [])
            cur = ins[0] if ins else None
        paths.append(list(reversed(chain)))
    if len(paths) < 2:
        return []
    a, b = paths[0], paths[1]
    i = 0
    while i < min(len(a), len(b)) and a[i] == b[i]:
        i += 1
    return a[i:] + b[i:]


def _parallel(u, v, tol=1e-6):
    if not u or not v:
        return False
    cx = u[1] * v[2] - u[2] * v[1]
    cy = u[2] * v[0] - u[0] * v[2]
    cz = u[0] * v[1] - u[1] * v[0]
    return (cx * cx + cy * cy + cz * cz) ** 0.5 < tol


def is_planar_loop(joints_dict, loop_names):
    """루프 안의 회전축이 전부 평행하면 평면 기구다.

    평면이면, hinge 를 끊고 connect(구면)로 닫아도 **과소구속이 아니다** —
    connect 가 남기는 면외(out-of-plane) 회전 2자유도를 트리의 다른 조인트들이
    이미 막고 있기 때문. 실제 사례(회수장치2 크랭크-슬라이더): 회전축 3개가
    전부 z, 슬라이더는 x 병진 → 평면.
    """
    axes = [joints_dict[n].get('axis') for n in loop_names
            if joints_dict[n].get('type') in _ROT_TYPES]
    axes = [a for a in axes if a and any(a)]
    return bool(axes) and all(_parallel(axes[0], a) for a in axes[1:])


def recommend_equality(joints_dict, joint_name):
    """끊는 조인트의 자유도에 맞는 equality 종류를 고른다.

    끊은 조인트가 없앤 자유도를 그대로 되돌려줘야 한다:
      fixed(0 DOF)  -> weld    (6자유도 전부 구속, 정확)
      Ball(3 rot)   -> connect (병진 3만 구속, 정확)
      hinge(1 rot)  -> 정확히 대응하는 equality 가 MJCF 에 없다.
                       weld 는 회전까지 죽여 **과다구속**이므로 쓰면 안 되고,
                       connect 는 회전 3을 남겨 원칙적으로 2 부족하지만,
                       **평면 루프면 그 2 를 트리가 이미 막고 있어 정확**해진다.
      slide(1 trans)-> 마찬가지로 정확한 대응이 없다.

    Returns
    -------
    (equality: str, note: str)
    """
    t = joints_dict[joint_name].get('type')
    if t == 'fixed':
        return 'weld', 'fixed(0 DOF) 를 끊었으므로 weld 가 정확하다.'
    if t == 'Ball':
        return 'connect', 'Ball(회전 3 DOF) 를 끊었으므로 connect 가 정확하다.'
    if t in _ROT_TYPES:
        loop = loop_joint_names(joints_dict, joints_dict[joint_name]['child'])
        if is_planar_loop(joints_dict, loop):
            return 'connect', ('hinge 를 끊었지만 루프의 회전축이 전부 평행한 '
                               '평면 기구다 — connect 가 남기는 면외 회전 2 자유도는 '
                               '트리가 이미 막고 있어 결과적으로 정확하다. '
                               'weld 는 필요한 회전까지 죽이므로 쓰면 안 된다.')
        return 'connect', ('hinge 를 끊었고 루프가 평면이 아니다 — connect 는 면외 '
                           '회전 2 자유도를 남겨 **과소구속**이다. 루프 안의 fixed '
                           '조인트를 대신 끊고 weld 로 닫는 편이 정확하다.')
    return 'connect', 'type={} — 정확히 대응하는 equality 가 없어 connect 로 둔다.'.format(t)


def build_constraint_records(constraint_joints, links_xyz_dict=None, joints_dict=None):
    """제약으로 뺀 조인트를 MJCF equality 로 옮길 수 있는 형태로 정리한다.

    anchor 는 Fusion 이 준 조인트 원점(루트 기준, meter)을 그대로 싣는다.
    MJCF 변환 단계에서 body 로컬 좌표로 바꿔 쓰면 된다 — 그래서 두 링크의
    원점(links_xyz_dict)도 같이 기록해 변환에 필요한 값이 다 들어가게 한다.
    """
    records = []
    ref = joints_dict if joints_dict is not None else constraint_joints
    for name, j in constraint_joints.items():
        eq, note = recommend_equality(ref, name)
        rec = {
            'name': name,
            'equality': eq,
            'why': note,
            'body1': j['child'],
            'body2': j['parent'],
            'anchor_root': j.get('xyz'),        # 루트 기준 조인트 원점 [m]
            'original_joint_type': j.get('type'),
            'axis': j.get('axis'),
        }
        if links_xyz_dict:
            rec['body1_origin_root'] = links_xyz_dict.get(j['child'])
            rec['body2_origin_root'] = links_xyz_dict.get(j['parent'])
        records.append(rec)
    return records


def render_constraints_json(records, robot_name):
    return json.dumps({'robot': robot_name, 'constraints': records},
                      indent=2, ensure_ascii=False)


def render_constraints_mjcf(records):
    """참고용 MJCF 스니펫. 좌표계 변환은 downstream 몫이라 anchor 는 루트 기준값
    그대로 들어간다 — 그대로 붙여넣지 말고 확인하라는 뜻으로 주석을 단다."""
    lines = ['<!-- fusion2urdf: 폐루프를 닫는 조인트를 equality 로 내보냄.',
             '     anchor 는 루트(base_link) 기준 좌표다. MJCF 에 붙일 때는',
             '     body1 의 로컬 좌표로 변환해야 한다. -->',
             '<equality>']
    for r in records:
        anchor = r.get('anchor_root') or [0, 0, 0]
        if r.get('why'):
            lines.append('  <!-- {}: {} -->'.format(r['original_joint_type'], r['why']))
        lines.append(
            '  <{} name="{}_loop_close" body1="{}" body2="{}" anchor="{}"/>'.format(
                r['equality'], r['name'].replace(' ', '_'),
                r['body1'], r['body2'],
                ' '.join(str(v) for v in anchor)))
    lines.append('</equality>')
    return '\n'.join(lines)


def write_constraints(records, robot_name, save_dir):
    """사이드카 2종(.constraints.json / .constraints.mjcf)을 urdf/ 옆에 쓴다."""
    d = os.path.join(save_dir, 'urdf')
    try:
        os.mkdir(d)
    except Exception:
        pass
    p_json = os.path.join(d, robot_name + '.constraints.json')
    with open(p_json, 'w', encoding='utf-8') as f:
        f.write(render_constraints_json(records, robot_name))
    p_mjcf = os.path.join(d, robot_name + '.constraints.mjcf')
    with open(p_mjcf, 'w', encoding='utf-8') as f:
        f.write(render_constraints_mjcf(records))
    return p_json, p_mjcf


# ── Fusion UI (adsk 필요) ───────────────────────────────────────────────────
def select_constraint_joints(ui, joints_dict, title='Fusion2URDF'):
    """equality constraint 로 쓸 조인트를 사용자에게 확인받는다.

    폐루프를 자동 감지해 기본값으로 채워 주고, 사용자가 쉼표로 구분해 고칠 수
    있게 한다. 빈 문자열이면 제약 없음(= 기존 동작).

    Returns
    -------
    (names: list[str], cancelled: bool)
    """
    auto = detect_loop_joints(joints_dict)
    dup_note = ''
    if auto:
        dup_note = ('\n\n자동 감지된 폐루프(부모가 둘인 링크):\n' +
                    '\n'.join('  {}  →  링크 "{}"'.format(n, joints_dict[n]['child'])
                              for n in auto))
    prompt = (
        'equality constraint 로 내보낼 조인트 이름을 쉼표로 구분해 입력하세요.\n'
        '여기 적은 조인트는 URDF <joint> 로 나가지 않고, 사이드카 파일'
        '(*.constraints.json / .mjcf)에 기록됩니다.\n'
        '비워 두면 제약 없이 기존 방식대로 내보냅니다.' + dup_note)
    default = ', '.join(auto)
    try:
        text, cancelled = ui.inputBox(prompt, title, default)
    except Exception:
        # inputBox 가 (str, bool) 대신 예외를 던지는 환경 대비
        return auto, False
    if cancelled:
        return [], True
    names = [s.strip() for s in str(text).split(',') if s.strip()]
    unknown = [n for n in names if n not in joints_dict]
    if unknown:
        ui.messageBox('다음 조인트를 찾을 수 없습니다:\n  ' + '\n  '.join(unknown) +
                      '\n\n조인트 이름을 확인하고 다시 실행하세요.', title)
        return [], True
    return names, False
