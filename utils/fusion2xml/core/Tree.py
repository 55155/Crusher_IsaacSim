# -*- coding: utf-8 -*-
"""
Tree.py — 조인트 목록을 MJCF body 트리로 세우고, 폐루프를 equality 로 분리한다.

`adsk` 에 의존하지 않는다 — Fusion 밖에서 단위 테스트할 수 있다.

좌표 규약 (fusion2urdf 와 동일)
------------------------------
- 조인트의 `xyz` 는 **root 기준 절대 위치**(m)다.
- 링크의 프레임 원점 = 그 링크를 자식으로 갖는 조인트의 위치. base_link 는 원점.
- 모든 프레임은 root 와 축정렬(회전 없음)이므로 프레임 간 변환은 순수 평행이동이다.

이 규약 덕분에 MJCF 변환이 뺄셈만으로 끝난다:
    <body pos> = 링크원점(자신) - 링크원점(부모)
    <joint pos> = "0 0 0"          (링크 프레임 원점이 곧 조인트 위치)
    <geom pos>  = -링크원점(자신)   (메시는 root 좌표로 내보내지므로 되밀어준다)

폐루프
------
MJCF 는 URDF 와 달리 `<equality>` 가 있어서 폐루프를 표현할 수 있다. 다만
`<body>` 중첩 자체는 여전히 트리여야 하므로, 루프를 닫는 조인트 하나를 트리에서
빼서 equality 로 돌리는 절차는 똑같이 필요하다. 차이는 그 결과가 URDF 처럼
사이드카로 밀려나지 않고 **같은 XML 안에서 닫힌다**는 것이다.
"""

import collections

BASE_LINK = 'base_link'


# ── 방향 재정렬 ─────────────────────────────────────────────────────────────

def reorient_to_root(joints, base=BASE_LINK):
    """base 에서 BFS 해 모든 조인트를 트리 방향(parent -> child)으로 세운다.

    Fusion 의 component1/component2 순서는 조립 편의상의 산물이지 운동학적
    부모/자식이 아니다. 뒤집는 조인트는 축을 반전해 관절값 부호를 보존한다
    (부모/자식을 맞바꾸면 같은 물리 회전이 -θ 이므로, 축을 뒤집으면 θ 와 limit 을
    그대로 쓸 수 있다).

    Returns
    -------
    tree      : {name: joint}  트리 방향으로 교정된 조인트
    loops     : {name: joint}  양 끝이 이미 트리에 있는 = 루프를 닫는 조인트(원본 방향)
    flipped   : [name, ...]    방향을 뒤집은 조인트
    unreached : {name: joint}  base 에서 도달 못 한 조인트
    """
    adj = collections.defaultdict(list)
    for name, j in joints.items():
        adj[j['parent']].append((name, j['child']))
        adj[j['child']].append((name, j['parent']))

    visited = {base}
    tree, loops, flipped = {}, {}, []
    used = set()
    queue = collections.deque([base])

    while queue:
        cur = queue.popleft()
        for jname, other in adj.get(cur, []):
            if jname in used:
                continue
            used.add(jname)
            if other in visited:
                loops[jname] = joints[jname]
                continue
            visited.add(other)
            j = dict(joints[jname])
            if j['parent'] != cur:
                j['parent'], j['child'] = cur, other
                j['axis'] = [-a for a in (j.get('axis') or [0, 0, 0])]
                flipped.append(jname)
            tree[jname] = j
            queue.append(other)

    unreached = {n: j for n, j in joints.items() if n not in used}
    return tree, loops, flipped, unreached


# ── 링크 원점 / body 트리 ───────────────────────────────────────────────────

def link_origins(tree_joints, base=BASE_LINK):
    """링크 이름 -> root 기준 프레임 원점(m).

    링크의 원점은 그 링크를 자식으로 갖는 조인트의 위치다. 조인트를 뒤집으면
    어느 쪽이 자식인지 바뀌므로 원점도 따라 바뀌는데, `<geom pos>` 가 항상
    `-원점` 으로 상쇄하므로 기하는 어느 쪽이든 제자리에 놓인다.
    """
    origins = {base: [0.0, 0.0, 0.0]}
    for j in tree_joints.values():
        origins[j['child']] = list(j['xyz'])
    return origins


def build_body_tree(tree_joints, base=BASE_LINK):
    """중첩 body 구조를 만든다.

    Returns
    -------
    {'name': str, 'joints': [joint, ...], 'children': [node, ...]}
      루트 노드의 'joints' 는 비어 있다(base_link 는 부모가 없다).
    """
    by_parent = collections.defaultdict(list)
    for name, j in tree_joints.items():
        by_parent[j['parent']].append((name, j))

    def node(link, seen):
        if link in seen:      # 재정렬을 거치면 안 생기지만 무한재귀만은 막는다
            return {'name': link, 'joints': [], 'children': []}
        seen = seen | {link}
        children = []
        for name, j in sorted(by_parent.get(link, []), key=lambda t: t[0]):
            child = node(j['child'], seen)
            child['joints'] = [dict(j, name=name)]
            children.append(child)
        return {'name': link, 'joints': [], 'children': children}

    return node(base, set())


def iter_bodies(node):
    """body 트리를 깊이우선으로 훑는다."""
    yield node
    for c in node['children']:
        for n in iter_bodies(c):
            yield n


# ── equality 계획 ───────────────────────────────────────────────────────────

def joints_between(joints, a, b):
    """두 링크를 직접 잇는 조인트 이름들."""
    return [n for n, j in joints.items()
            if {j['parent'], j['child']} == {a, b}]


def recommend_equality(joint_type):
    """끊는 조인트의 자유도에 맞는 equality 종류. (type, note)

      fixed(0 DOF)      -> weld    (6자유도 전부 구속, 정확)
      Ball(회전 3)       -> connect (병진 3만 구속, 정확)
      revolute(회전 1)   -> connect. 면외 회전 2자유도가 남지만, 평면 기구라면
                          트리의 다른 조인트들이 그 2를 이미 막고 있어 결과적으로
                          정확하다. weld 는 살아 있어야 할 회전까지 죽인다.
    """
    if joint_type == 'fixed':
        return 'weld', 'fixed(0 DOF) 를 끊었으므로 weld 가 정확하다.'
    if joint_type == 'Ball':
        return 'connect', 'Ball(회전 3 DOF) 를 끊었으므로 connect 가 정확하다.'
    if joint_type in ('revolute', 'continuous'):
        return 'connect', ('hinge 를 끊었다 — connect 는 병진 3 만 구속하므로 '
                           '면외 회전 2 자유도가 남는다. 루프의 회전축이 모두 '
                           '평행한 평면 기구면 그 2 를 트리가 이미 막고 있어 '
                           '정확하고, 아니면 과소구속이다.')
    if joint_type == 'prismatic':
        return 'connect', ('slider 를 끊었다 — connect 로는 미끄럼 방향까지 '
                           '막히므로 과대구속이다. 루프 안의 다른 조인트를 '
                           '끊는 편이 낫다.')
    return 'connect', 'type={} — 정확히 대응하는 equality 가 없어 connect 로 둔다.'.format(
        joint_type)


def plan(joints, forced_pairs=(), base=BASE_LINK):
    """MJCF 내보내기 계획을 세운다.

    Parameters
    ----------
    joints : {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    forced_pairs : [(body1, body2, eq_type or None), ...]
        사용자가 "이 두 링크는 equality 로 묶어라" 라고 지정한 쌍.
        eq_type 이 None 이면 사이에 있는 조인트 타입을 보고 자동으로 고른다.

    Returns
    -------
    dict(tree=, body_tree=, origins=, equalities=, flipped=, problems=, notices=)
    """
    joints = dict(joints)
    rest = dict(joints)
    equalities = []
    problems = []
    notices = []

    # 1) 사용자가 지정한 쌍을 먼저 트리에서 빼낸다.
    for pair in forced_pairs:
        b1, b2 = pair[0], pair[1]
        eq_type = pair[2] if len(pair) > 2 else None
        anchor_root = pair[3] if len(pair) > 3 else None

        between = joints_between(joints, b1, b2)
        src_type = None
        if between:
            # 두 링크 사이에 실제 조인트가 있으면 그걸 빼서 anchor 로 쓴다.
            # 이게 정상 경로다 — Fusion 에서 루프를 닫아 두면 위치가 정확해진다.
            jname = between[0]
            src_type = joints[jname]['type']
            if anchor_root is None:
                anchor_root = list(joints[jname]['xyz'])
            rest.pop(jname, None)
            source = "조인트 '{}' 에서 anchor 를 가져왔다".format(jname)
            if len(between) > 1:
                notices.append("'{}' 과 '{}' 사이에 조인트가 {}개 있다 — '{}' 를 "
                               '썼다.'.format(b1, b2, len(between), jname))
        elif anchor_root is None:
            problems.append(
                "'{}' 과 '{}' 사이에 조인트가 없고 anchor 도 지정되지 않았다 — "
                'Fusion 에서 두 링크를 잇는 조인트를 만들거나, anchor 좌표를 직접 '
                '입력해야 한다.'.format(b1, b2))
            continue
        else:
            source = '사용자가 입력한 anchor(root 좌표)'

        if eq_type is None:
            eq_type, note = recommend_equality(src_type or 'fixed')
        else:
            note = '사용자가 {} 로 지정했다.'.format(eq_type)

        equalities.append({
            'body1': b1, 'body2': b2, 'type': eq_type,
            'anchor_root': anchor_root, 'source': source, 'why': note,
            'from_joint': between[0] if between else None,
        })

    # 2) 남은 조인트를 base 기준으로 재정렬 -> 자동으로 남는 루프를 찾아낸다.
    tree, loops, flipped, unreached = reorient_to_root(rest, base)

    for jname, j in loops.items():
        eq_type, note = recommend_equality(j['type'])
        equalities.append({
            'body1': j['child'], 'body2': j['parent'], 'type': eq_type,
            'anchor_root': list(j['xyz']), 'why': note,
            'source': "폐루프를 닫는 조인트 '{}' 를 자동으로 분리했다".format(jname),
            'from_joint': jname,
        })

    # 3) 검증
    all_links = set()
    for j in joints.values():
        all_links.add(j['parent'])
        all_links.add(j['child'])
    if joints and base not in all_links:
        problems.append("base_link 를 찾을 수 없다 — 루트 컴포넌트 이름을 "
                        "'base_link' 로 지정하라.")
    for name, j in unreached.items():
        problems.append(
            "조인트 '{}' ({} ↔ {}) 가 base_link 에서 도달 불가 — 중간 연결이 "
            '빠졌거나, 사용자가 지정한 equality 쌍이 트리를 끊었다.'
            .format(name, j['parent'], j['child']))

    origins = link_origins(tree, base)
    for eq in equalities:
        for b in (eq['body1'], eq['body2']):
            if b not in origins:
                problems.append(
                    "equality 의 '{}' 이 트리에 없다 — MJCF 에 그런 body 가 없어 "
                    'MuJoCo 가 로딩에 실패한다.'.format(b))

    body_tree = build_body_tree(tree, base)
    reached = {n['name'] for n in iter_bodies(body_tree)}
    for link in sorted(all_links - reached - {base}):
        problems.append("링크 '{}' 이 body 트리에 안 들어갔다.".format(link))

    if flipped:
        notices.append('base_link 기준으로 방향을 교정한 조인트 {}개: {}'
                       .format(len(flipped), ', '.join(sorted(flipped))))

    return {
        'tree': tree,
        'body_tree': body_tree,
        'origins': origins,
        'equalities': equalities,
        'flipped': flipped,
        'problems': problems,
        'notices': notices,
    }
