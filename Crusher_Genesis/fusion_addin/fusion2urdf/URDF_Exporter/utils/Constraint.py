# -*- coding: utf-8 -*-
"""
Constraint.py — 폐루프(closed kinematic loop)를 닫는 조인트를 URDF <joint> 가 아니라
**MJCF equality constraint** 로 내보낸다.

배경
----
URDF 는 운동학 트리만 표현할 수 있다(링크마다 부모가 정확히 하나). 그런데
`Write.write_link_urdf` 는 **조인트마다 링크를 하나씩** 찍는다:

    for joint in joints_dict:
        name = joints_dict[joint]['child']      # <- 같은 child 가 두 번 나오면 링크도 두 번

그래서 폐루프가 있는 기구를 내보내면 루프를 닫는 링크가 **같은 이름으로 두 번**
정의된 불법 URDF 가 나온다. 실측(회수장치2): `Crank_1` 이 `회전 29`(부모 PULLEY_1)
와 `회전 34`(부모 M_Top_1) 양쪽의 child 였다.

해결은 두 단계다.

1. 방향 재정렬 (reorient_to_root)
   Fusion 의 component1/component2 순서는 **조립 편의상의 산물**이지 운동학적
   부모/자식이 아니다. 그런데 fusion2urdf 는 component2 를 그대로 parent 로 쓴다.
   그래서 조립 순서에 따라 어떤 링크는 부모가 둘, 어떤 링크는 부모가 0개가 된다.
   부모가 0개인 링크를 다른 조인트가 parent 로 참조하면
   `Write.write_joint_urdf` 가 `links_xyz_dict[parent]` 에서 KeyError 를 낸다
   (실측: M_Top_1). 이 모듈은 base_link 에서 BFS 해 **모든 조인트를 트리 방향으로
   다시 세운다** — 뒤집힌 조인트는 parent/child 를 바꾸고 축을 반전한다.

2. 루프 폐합 조인트 분리
   재정렬 중 양 끝이 이미 트리에 들어온 조인트가 곧 루프를 닫는 조인트다.
   이걸 URDF 에서 빼면 트리가 완성되고, 위치 정보는 사이드카로 보존해
   MJCF 에서 <equality><connect> 로 되살린다.

        <equality>
          <connect name="..." body1="<child>" body2="<parent>" anchor="<xyz>"/>
        </equality>

anchor 좌표계 (정확도의 핵심)
----------------------------
MuJoCo 의 connect anchor 는 **body1 의 로컬 프레임** 기준이다. fusion2urdf 는
- 모든 joint origin 과 link origin 을 `rpy="0 0 0"` 으로 쓰고,
- 링크 프레임 원점을 그 링크의 부모 조인트 위치에 둔다
  (`Link.xyz = -joint_xyz` 로 메시를 되밀어 맞춘다).
따라서 **모든 링크 프레임은 root 와 축정렬**돼 있고, 프레임 간 변환은 순수 평행이동이다.
그래서 body 로컬 anchor 는 정확히 뺄셈으로 얻어진다:

    anchor_body1_local = joint_origin_root - link_origin_root(body1)

회전이 끼어들 여지가 없으므로 이 값은 근사가 아니라 정확값이다.

이 모듈의 순수 함수(reorient_to_root / split_joints / build_constraint_records /
render_* 등)는 `adsk` 에 의존하지 않는다 — Fusion 밖에서 단위 테스트 가능.
UI 를 쓰는 select_constraint_joints 만 Fusion 안에서 동작한다.
"""

import collections
import json
import os


# ── 방향 재정렬 ─────────────────────────────────────────────────────────────

def reorient_to_root(joints_dict, base_link='base_link'):
    """base_link 에서 BFS 해 모든 조인트를 트리 방향(parent -> child)으로 세운다.

    Fusion 의 component1/component2 순서를 신뢰하지 않고, base_link 로부터의
    도달 방향으로 parent/child 를 다시 정한다. 뒤집는 조인트는 축을 반전해
    관절값의 부호 규약을 보존한다(자식과 부모를 맞바꾸면 같은 물리 회전이
    -θ 가 되므로, 축을 뒤집으면 θ 와 limit 을 그대로 쓸 수 있다).

    Returns
    -------
    tree      : {name: joint}  트리 방향으로 교정된 조인트
    loops     : {name: joint}  양 끝이 이미 트리에 있는 = 루프를 닫는 조인트(원본 방향)
    flipped   : [name, ...]    방향을 뒤집은 조인트 이름
    unreached : {name: joint}  base_link 에서 도달 못 한 조인트
    """
    adj = collections.defaultdict(list)
    for name, j in joints_dict.items():
        adj[j['parent']].append((name, j['child']))
        adj[j['child']].append((name, j['parent']))

    visited = {base_link}
    tree, loops, flipped = {}, {}, []
    used = set()
    queue = collections.deque([base_link])

    while queue:
        cur = queue.popleft()
        for jname, other in adj.get(cur, []):
            if jname in used:
                continue
            used.add(jname)
            if other in visited:
                loops[jname] = joints_dict[jname]        # 루프 폐합
                continue
            visited.add(other)
            j = dict(joints_dict[jname])
            if j['parent'] != cur:
                j['parent'], j['child'] = cur, other
                axis = j.get('axis') or [0, 0, 0]
                j['axis'] = [-a for a in axis]
                flipped.append(jname)
            tree[jname] = j
            queue.append(other)

    unreached = {n: j for n, j in joints_dict.items() if n not in used}
    return tree, loops, flipped, unreached


def split_joints(joints_dict, constraint_names):
    """joints_dict 를 (트리용, 제약용) 두 개로 나눈다. 원본은 안 건드린다."""
    constraint_names = set(constraint_names)
    tree = {k: v for k, v in joints_dict.items() if k not in constraint_names}
    constraints = {k: v for k, v in joints_dict.items() if k in constraint_names}
    return tree, constraints


def plan_export(joints_dict, forced_constraints=(), base_link='base_link'):
    """내보내기 계획을 한 번에 세운다.

    사용자가 지정한 조인트를 먼저 빼고, 남은 것들을 base_link 기준으로 재정렬해
    자동 감지된 루프 폐합 조인트까지 제약 쪽으로 넘긴다.

    Returns
    -------
    dict(tree=, constraints=, flipped=, unreached=, problems=)
    """
    rest, forced = split_joints(joints_dict, forced_constraints)
    tree, loops, flipped, unreached = reorient_to_root(rest, base_link)

    constraints = dict(forced)
    constraints.update(loops)

    problems = []
    if base_link not in _all_links(joints_dict) and joints_dict:
        problems.append("base_link 를 찾을 수 없다 — 루트 컴포넌트를 base_link 로 지정하라.")
    for name, j in unreached.items():
        problems.append(
            "조인트 '{}' ({} ↔ {}) 가 base_link 에서 도달 불가 — 중간 연결이 빠졌거나 "
            "As-built Joint 로만 붙어 있다.".format(name, j['parent'], j['child']))
    problems += _orphan_problems(tree, base_link)

    return {'tree': tree, 'constraints': constraints, 'flipped': flipped,
            'unreached': unreached, 'problems': problems}


def _all_links(joints_dict):
    s = set()
    for j in joints_dict.values():
        s.add(j['parent'])
        s.add(j['child'])
    return s


def _orphan_problems(tree_joints, base_link='base_link'):
    """parent 로 쓰이는데 한 번도 child 가 아닌 링크 = Write 가 KeyError 를 내는 링크.

    재정렬을 거치면 원래 안 나와야 하지만, 사용자가 강제로 뺀 조인트 때문에
    링크가 트리에서 떨어져 나갈 수 있어 마지막에 한 번 더 검사한다.
    """
    children = {j['child'] for j in tree_joints.values()}
    parents = {j['parent'] for j in tree_joints.values()}
    problems = []
    for link in sorted(parents - children - {base_link}):
        problems.append(
            "링크 '{}' 이 부모가 0개인데 다른 조인트의 parent 로 쓰인다 — "
            "Write.write_joint_urdf 가 KeyError 를 낸다. 이 링크를 트리에 붙이는 "
            "조인트가 필요하다.".format(link))
    counts = collections.Counter(j['child'] for j in tree_joints.values())
    for link, n in sorted(counts.items()):
        if n > 1:
            problems.append("링크 '{}' 의 부모가 아직 {}개다 — 제약으로 더 빼야 한다."
                            .format(link, n))
        if link == base_link:
            problems.append("base_link 가 조인트의 child 다 — 루트가 될 수 없다.")
    return problems


def validate_tree(tree_joints, base_link='base_link'):
    """제약을 뺀 뒤 정말 트리가 됐는지 검사한다. (ok, problems)"""
    problems = _orphan_problems(tree_joints, base_link)
    return (not problems), problems


def detect_loop_joints(joints_dict):
    """child 가 두 번 이상 등장하는 조인트 중 두 번째 이후 (구버전 호환용)."""
    seen = set()
    loop = []
    for name, j in joints_dict.items():
        child = j['child']
        if child in seen:
            loop.append(name)
        else:
            seen.add(child)
    return loop


# ── equality 종류 판정 ──────────────────────────────────────────────────────

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
    이미 막고 있기 때문. 실측(회수장치2): 회전축 3개가 전부 z, 슬라이더는 x 병진.
    """
    axes = [joints_dict[n].get('axis') for n in loop_names
            if joints_dict[n].get('type') in _ROT_TYPES]
    axes = [a for a in axes if a and any(a)]
    return bool(axes) and all(_parallel(axes[0], a) for a in axes[1:])


def recommend_equality(joints_dict, joint_name):
    """끊는 조인트의 자유도에 맞는 equality 종류를 고른다. (equality, note)

      fixed(0 DOF)  -> weld    (6자유도 전부 구속, 정확)
      Ball(3 rot)   -> connect (병진 3만 구속, 정확)
      hinge(1 rot)  -> connect. 원칙적으로 면외 회전 2 자유도가 남지만,
                      **평면 루프면 그 2 를 트리가 이미 막고 있어 정확**해진다.
                      weld 는 필요한 회전까지 죽이므로 쓰면 안 된다.
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


# ── 사이드카 기록 ───────────────────────────────────────────────────────────

def _sub(a, b, nd=6):
    if a is None or b is None:
        return None
    return [round(x - y, nd) for x, y in zip(a, b)]


def build_constraint_records(constraint_joints, links_xyz_dict=None, joints_dict=None):
    """제약으로 뺀 조인트를 MJCF equality 로 옮길 수 있는 형태로 정리한다.

    anchor 는 세 가지 좌표계로 전부 기록한다:
      anchor_root         Fusion 이 준 조인트 원점 (root/base_link 기준) [m]
      anchor_body1_local  body1(child) 로컬 — **MJCF connect 가 요구하는 값**
      anchor_body2_local  body2(parent) 로컬 — 검산/weld 용

    모든 링크 프레임이 root 와 축정렬(rpy=0 0 0)이라 로컬값은 단순 뺄셈으로
    얻어지는 **정확값**이다. body 원점을 못 찾으면 None 을 넣어 downstream 이
    조용히 틀린 값을 쓰는 대신 바로 알아채게 한다.
    """
    records = []
    ref = joints_dict if joints_dict is not None else constraint_joints
    for name, j in constraint_joints.items():
        eq, note = recommend_equality(ref, name)
        body1, body2 = j['child'], j['parent']
        anchor = j.get('xyz')
        o1 = (links_xyz_dict or {}).get(body1)
        o2 = (links_xyz_dict or {}).get(body2)
        rec = {
            'name': name,
            'equality': eq,
            'why': note,
            'body1': body1,
            'body2': body2,
            'anchor_root': anchor,
            'anchor_body1_local': _sub(anchor, o1),
            'anchor_body2_local': _sub(anchor, o2),
            'body1_origin_root': o1,
            'body2_origin_root': o2,
            'original_joint_type': j.get('type'),
            'axis': j.get('axis'),
            'frame_note': ('모든 링크 프레임이 root 와 축정렬(rpy=0 0 0)이라 '
                           'local = root - link_origin 뺄셈이 정확값이다.'),
        }
        if rec['anchor_body1_local'] is None:
            rec['warning'] = ("body1 '{}' 의 링크 원점을 찾지 못했다 — 이 링크가 URDF "
                              "트리에 없다는 뜻이다. anchor 를 쓰기 전에 확인하라."
                              .format(body1))
        records.append(rec)
    return records


def render_constraints_json(records, robot_name):
    return json.dumps({'robot': robot_name, 'constraints': records},
                      indent=2, ensure_ascii=False)


def render_constraints_mjcf(records):
    """붙여넣어 바로 쓸 수 있는 MJCF 스니펫. anchor 는 body1 로컬 좌표다."""
    lines = ['<!-- fusion2urdf: 폐루프를 닫는 조인트를 URDF <joint> 에서 빼고',
             '     equality 로 내보냄. anchor 는 MuJoCo 규약대로 body1 로컬 좌표.',
             '     (링크 프레임이 전부 root 와 축정렬이라 단순 뺄셈으로 정확) -->',
             '<equality>']
    for r in records:
        anchor = r.get('anchor_body1_local')
        if anchor is None:
            lines.append('  <!-- {}: anchor 계산 실패 — {} -->'.format(
                r['name'], r.get('warning', 'body1 링크 원점 없음')))
            continue
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

def select_constraint_joints(ui, joints_dict, title='Fusion2URDF', base_link='base_link'):
    """equality constraint 로 쓸 조인트를 사용자에게 확인받는다.

    base_link 기준 재정렬로 루프 폐합 조인트를 자동 감지해 기본값으로 채운다.
    빈 문자열이면 자동 감지분만 쓴다.

    Returns
    -------
    (names: list[str], cancelled: bool)
    """
    plan = plan_export(joints_dict, (), base_link)
    auto = sorted(plan['constraints'])

    note = ''
    if auto:
        note += ('\n\n자동 감지된 폐루프 폐합 조인트:\n' +
                 '\n'.join('  {}  ({} ↔ {})'.format(
                     n, joints_dict[n]['parent'], joints_dict[n]['child']) for n in auto))
    if plan['flipped']:
        note += ('\n\nbase_link 기준으로 방향을 교정할 조인트 {}개:\n  '
                 .format(len(plan['flipped'])) + ', '.join(sorted(plan['flipped'])))
    if plan['problems']:
        note += '\n\n경고:\n  ' + '\n  '.join(plan['problems'])

    prompt = (
        'equality constraint 로 내보낼 조인트 이름을 쉼표로 구분해 입력하세요.\n'
        '여기 적은 조인트는 URDF <joint> 로 나가지 않고, 사이드카 파일'
        '(*.constraints.json / .mjcf)에 접합 위치만 기록됩니다.\n'
        '비워 두면 아래 자동 감지분만 제약으로 뺍니다.' + note)
    try:
        text, cancelled = ui.inputBox(prompt, title, ', '.join(auto))
    except Exception:
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
