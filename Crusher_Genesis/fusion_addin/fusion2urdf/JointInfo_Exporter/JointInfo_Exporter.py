# -*- coding: utf-8 -*-
"""
JointInfo_Exporter.py — URDF/MJCF 를 만드는 데 필요한 접합(joint) 정보를
디자인에서 하나도 빠짐없이 긁어 파일로 쓴다. (Fusion 360 Script)

왜 필요한가
----------
`core/Joint.py:make_joints_dict` 는 **`root.joints` 만** 읽는다. 그래서

  - 서브어셈블리(하위 컴포넌트) 안에 만든 조인트
  - As-built Joint (`component.asBuiltJoints`)
  - Rigid Group (`component.rigidGroups`)
  - suppressed / 전구 꺼진 조인트

가 전부 조용히 빠진다. 링크가 트리에서 떨어져 나가거나(`Constraint.py` 의
"base_link 에서 도달 불가") 폐루프가 열린 채로 보이는 원인이 대개 여기다 —
Fusion 에서는 분명히 붙여놨는데 익스포터 눈에는 안 보이는 것이다.

`fusion_addin/dump_joint_anchors` 는 하위 컴포넌트까지 재귀로 훑지만 anchor
좌표만 뽑고 타입/축/리밋이 없어서 그것만으로는 XML 을 못 만든다.

이 스크립트는 둘을 합쳐서 **XML 한 벌을 만드는 데 필요한 접합 정보 전부**를
한 번에 덤프한다. 익스포터를 돌리기 전에 이걸 먼저 돌려서 "익스포터가 볼 수
있는 것"과 "실제 디자인에 있는 것"의 차이를 확인하는 용도다.

출력 (선택한 폴더에 3개)
-----------------------
  <design>_jointinfo.json     전부. `joints_dict` 섹션은 fusion2urdf 의
                              joints_dict 와 **키 구조가 동일**해서 Fusion 밖에서
                              `Constraint.plan_export()` 에 그대로 넣을 수 있다.
  <design>_jointinfo.csv      조인트 1개 = 1행. 스프레드시트로 훑어볼 용도.
  <design>_jointinfo.txt      사람이 읽는 요약 + 트리 문제 진단.

좌표/단위 규약
-------------
Fusion 내부 단위는 cm 라 전부 /100 해서 m 로 쓴다. 회전 리밋은 이미 rad 이라
그대로 두고, 슬라이드 리밋만 m 로 바꾼다(`Joint.py` 와 동일).

`xyz` 는 `Joint.py:make_joints_dict` 의 조인트 원점 계산을 **그대로 복제**했다
(trans/allclose/case1·case2 분기 + JointOrigin 폴백). 익스포터가 실제로 URDF 에
쓸 값과 같은 값을 봐야 비교가 의미 있기 때문이다.

`anchor_*_local` 은 각 occurrence 의 역변환을 실제로 적용해서 구한다 —
`Constraint.build_constraint_records` 의 뺄셈 근사(모든 링크 프레임이 root 와
축정렬이라는 가정)와 달리 회전이 섞여 있어도 맞다. 두 값이 다르면 그 가정이
깨진 것이므로 요약에 경고로 찍는다.

실행
----
Fusion 360 > Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭에서 이
폴더를 추가하고 Run. (Add-In 이 아니라 Script 다 — 버튼은 안 생긴다.)
"""

import adsk.core
import adsk.fusion
import traceback
import json
import os
import re
import collections


# ── fusion2urdf 와 공유하는 규약 ────────────────────────────────────────────

# adsk.fusion.JointTypes 의 인덱스 순서. core/Joint.py 의 joint_type_list 와
# 반드시 같아야 한다 — 오타(Cylinderical/Planner)까지 그대로 맞춘다.
JOINT_TYPE_LIST = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball',
]

BASE_LINK = 'base_link'


def sanitize(name):
    """core/Joint.py 와 동일한 이름 정규화."""
    return re.sub('[ :()]', '_', name)


def _m(v):
    """cm -> m, 소수 6자리."""
    return round(v / 100.0, 6)


def _m3(arr):
    return [_m(v) for v in arr]


# ── 조인트 수집 ─────────────────────────────────────────────────────────────

def _iter_joints(comp, occ_path='', seen=None):
    """디자인 전체의 조인트를 (joint, occ_path, is_as_built) 로 재귀 순회한다.

    같은 컴포넌트가 여러 번 인스턴스화돼 있으면 그 안의 조인트가 여러 번
    잡히므로 (컴포넌트 이름, 조인트 이름) 으로 한 번만 낸다 — 조인트는
    컴포넌트에 속하지 occurrence 에 속하지 않기 때문.
    """
    if seen is None:
        seen = set()

    for coll, is_as_built in ((comp.joints, False),
                              (getattr(comp, 'asBuiltJoints', []), True)):
        for j in coll:
            key = (comp.name, j.name, is_as_built)
            if key in seen:
                continue
            seen.add(key)
            yield j, occ_path, is_as_built

    for occ in comp.occurrences:
        sub = '{}/{}'.format(occ_path, occ.name) if occ_path else occ.name
        for item in _iter_joints(occ.component, sub, seen):
            yield item


def _joint_origin_xyz(joint):
    """core/Joint.py:make_joints_dict 의 조인트 원점 계산을 그대로 복제.

    Returns (xyz_m or None, how: str)
    """
    def trans(M, a):
        ex = [M[0], M[4], M[8]]
        ey = [M[1], M[5], M[9]]
        ez = [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        b = [0, 0, 0]
        for i in range(3):
            b[i] = a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i]
        return b

    def allclose(v1, v2, tol=1e-6):
        return max([abs(a - b) for a, b in zip(v1, v2)]) < tol

    try:
        xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray()
        xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray()
        xyz_of_one = joint.occurrenceOne.transform.translation.asArray()
        M_two = joint.occurrenceTwo.transform.asArray()

        case1 = allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
        case2 = allclose(xyz_from_two_to_joint, xyz_of_one)
        if case1 or case2:
            return _m3(xyz_from_two_to_joint), 'geometryOrOriginTwo(직접)'
        return _m3(trans(M_two, xyz_from_two_to_joint)), 'occurrenceTwo 변환 적용'
    except Exception:
        pass

    # As-built joint 는 geometryOrOrigin* 가 없고 geometry 하나만 있다.
    try:
        return _m3(joint.geometry.origin.asArray()), 'asBuilt geometry.origin'
    except Exception:
        pass

    try:
        if type(joint.geometryOrOriginTwo) == adsk.fusion.JointOrigin:
            data = joint.geometryOrOriginTwo.geometry.origin.asArray()
        else:
            data = joint.geometryOrOriginTwo.origin.asArray()
        return _m3(data), 'JointOrigin 폴백'
    except Exception:
        return None, '조인트 원점 없음 — Fusion 에서 원점을 지정해야 한다'


def _to_local(occ, world_xyz_m):
    """월드 좌표(m)를 그 occurrence 의 로컬 좌표(m)로 옮긴다.

    occurrence 변환의 역행렬을 실제로 적용하므로 회전이 섞여 있어도 맞다.
    """
    if occ is None or world_xyz_m is None:
        return None
    try:
        t = (occ.transform2 if hasattr(occ, 'transform2') else occ.transform).copy()
        if not t.invert():
            return None
        p = adsk.core.Point3D.create(world_xyz_m[0] * 100.0,
                                     world_xyz_m[1] * 100.0,
                                     world_xyz_m[2] * 100.0)
        p.transformBy(t)
        return _m3([p.x, p.y, p.z])
    except Exception:
        return None


def _motion_info(joint, urdf_type):
    """축과 리밋을 뽑는다. (axis, upper, lower, extra, notes)

    axis 는 URDF 규약대로 정규화된 벡터, 리밋은 회전 rad / 병진 m.
    """
    axis = [0, 0, 0]
    upper = 0.0
    lower = 0.0
    extra = {}
    notes = []

    try:
        motion = joint.jointMotion
    except Exception:
        return axis, upper, lower, extra, ['jointMotion 접근 실패']

    def limits(lim, scale, label):
        """리밋을 (upper, lower) 로. 한쪽만 켜져 있으면 익스포터가 죽으므로 경고."""
        try:
            hi_on = lim.isMaximumValueEnabled
            lo_on = lim.isMinimumValueEnabled
        except Exception:
            return 0.0, 0.0
        if hi_on and lo_on:
            return round(lim.maximumValue * scale, 6), round(lim.minimumValue * scale, 6)
        if hi_on != lo_on:
            notes.append('{} 리밋이 한쪽만 켜져 있다 — 익스포터가 여기서 멈춘다. '
                         '양쪽 다 켜거나 양쪽 다 끄라.'.format(label))
        else:
            notes.append('{} 리밋이 없다 — URDF 로는 continuous 로 나간다.'.format(label))
        return 0.0, 0.0

    if urdf_type == 'revolute':
        axis = [round(i, 6) for i in motion.rotationAxisVector.asArray()]
        upper, lower = limits(motion.rotationLimits, 1.0, '회전')
        if upper == 0.0 and lower == 0.0:
            extra['urdf_type_effective'] = 'continuous'
    elif urdf_type == 'prismatic':
        axis = [round(i, 6) for i in motion.slideDirectionVector.asArray()]
        upper, lower = limits(motion.slideLimits, 0.01, '슬라이드')
    elif urdf_type == 'Cylinderical':
        axis = [round(i, 6) for i in motion.rotationAxisVector.asArray()]
        upper, lower = limits(motion.rotationLimits, 1.0, '회전')
        notes.append('Cylindrical(회전1+병진1) — URDF 에 대응 타입이 없다. '
                     'revolute+prismatic 두 개로 쪼개거나 MJCF 에서 직접 써야 한다.')
    elif urdf_type == 'PinSlot':
        axis = [round(i, 6) for i in motion.rotationAxisVector.asArray()]
        extra['slide_axis'] = [round(i, 6) for i in motion.slideDirectionVector.asArray()]
        notes.append('PinSlot — URDF 에 대응 타입이 없다.')
    elif urdf_type == 'Planner':
        try:
            extra['normal'] = [round(i, 6) for i in motion.normalDirectionVector.asArray()]
            extra['primary'] = [round(i, 6) for i in motion.primarySlideDirection.asArray()]
        except Exception:
            pass
        notes.append('Planar(2병진+1회전) — URDF 에 대응 타입이 없다. MJCF 에서는 '
                     'slide 2 + hinge 1 로 쪼개야 한다.')
    elif urdf_type == 'Ball':
        notes.append('Ball(회전 3) — URDF 에 대응 타입이 없다. MJCF 에서는 '
                     'ball 조인트 또는 equality connect 로 표현한다.')
    # 'fixed' 는 축도 리밋도 없다.

    return axis, upper, lower, extra, notes


def collect(design):
    """디자인 전체의 접합 정보를 모은다."""
    root = design.rootComponent
    joints = []
    problems = []

    for joint, occ_path, is_as_built in _iter_joints(root):
        rec = {
            'name': sanitize(joint.name),
            'name_raw': joint.name,
            'occurrence_path': occ_path,
            'is_as_built': is_as_built,
            'notes': [],
        }

        # 타입
        try:
            idx = joint.jointMotion.jointType
            rec['fusion_type_index'] = idx
            urdf_type = JOINT_TYPE_LIST[idx] if 0 <= idx < len(JOINT_TYPE_LIST) else None
        except Exception:
            idx, urdf_type = None, None
        if urdf_type is None:
            urdf_type = 'fixed'
            rec['notes'].append('조인트 타입을 읽지 못해 fixed 로 간주했다.')
        rec['type'] = urdf_type

        # 활성 상태 — 꺼진 조인트는 익스포터가 그냥 지나치므로 명시적으로 남긴다
        for attr, key in (('isSuppressed', 'is_suppressed'),
                          ('isLightBulbOn', 'is_light_bulb_on')):
            try:
                rec[key] = bool(getattr(joint, attr))
            except Exception:
                rec[key] = None

        # 부모/자식 — fusion2urdf 규약: child=occurrenceOne, parent=occurrenceTwo
        occ_one = getattr(joint, 'occurrenceOne', None)
        occ_two = getattr(joint, 'occurrenceTwo', None)
        rec['child'] = sanitize(occ_one.name) if occ_one else None
        if occ_two is None:
            rec['parent'] = None
        elif occ_two.component.name == BASE_LINK:
            rec['parent'] = BASE_LINK
        else:
            rec['parent'] = sanitize(occ_two.name)
        rec['child_component'] = occ_one.component.name if occ_one else None
        rec['parent_component'] = occ_two.component.name if occ_two else None

        # 각 body 의 원점(root 기준) — Constraint.py 의 anchor 뺄셈에 쓰이는 값
        for occ, key in ((occ_one, 'child_origin_root'), (occ_two, 'parent_origin_root')):
            try:
                rec[key] = _m3(occ.transform.translation.asArray()) if occ else None
            except Exception:
                rec[key] = None

        # 조인트 원점
        xyz, how = _joint_origin_xyz(joint)
        rec['xyz'] = xyz
        rec['xyz_source'] = how
        if xyz is None:
            rec['notes'].append(how)

        # MJCF connect anchor — body 로컬 (역변환을 실제로 적용한 정확값)
        rec['anchor_child_local'] = _to_local(occ_one, xyz)
        rec['anchor_parent_local'] = _to_local(occ_two, xyz)

        # Constraint.py 의 뺄셈 가정이 유효한지 교차 검증
        sub = None
        if xyz and rec.get('child_origin_root'):
            sub = [round(a - b, 6) for a, b in zip(xyz, rec['child_origin_root'])]
        rec['anchor_child_local_by_subtraction'] = sub
        if sub and rec['anchor_child_local']:
            if max(abs(a - b) for a, b in zip(sub, rec['anchor_child_local'])) > 1e-4:
                rec['notes'].append(
                    'anchor 를 뺄셈으로 구한 값과 역변환으로 구한 값이 다르다 — 이 '
                    '링크 프레임이 root 와 축정렬이 아니라는 뜻이다. '
                    'Constraint.build_constraint_records 의 뺄셈 값을 쓰면 안 된다.')

        # 축 / 리밋
        axis, upper, lower, extra, notes = _motion_info(joint, urdf_type)
        rec['axis'] = axis
        rec['upper_limit'] = upper
        rec['lower_limit'] = lower
        rec['notes'].extend(notes)
        rec.update(extra)

        if is_as_built:
            rec['notes'].append('As-built Joint — fusion2urdf 는 root.joints 만 읽으므로 '
                                '이 조인트는 URDF 에 안 나간다.')
        if occ_path:
            rec['notes'].append("하위 컴포넌트 '{}' 안의 조인트 — fusion2urdf 는 "
                                'root.joints 만 읽으므로 URDF 에 안 나간다.'.format(occ_path))

        joints.append(rec)

    # Rigid group — 사실상 weld 다. URDF 에는 안 나가므로 반드시 별도로 기록.
    rigid_groups = []
    for comp_name, group in _iter_rigid_groups(root):
        try:
            rigid_groups.append({
                'name': group.name,
                'component': comp_name,
                'is_suppressed': bool(getattr(group, 'isSuppressed', False)),
                'occurrences': [sanitize(o.name) for o in group.occurrences],
            })
        except Exception:
            pass

    # 링크 목록 — occurrence 이름과 root 기준 원점
    links = {}
    for occ in root.allOccurrences:
        try:
            links[sanitize(occ.name)] = {
                'component': occ.component.name,
                'origin_root': _m3(occ.transform.translation.asArray()),
                'is_light_bulb_on': bool(occ.isLightBulbOn),
            }
        except Exception:
            pass

    return joints, rigid_groups, links, problems


def _iter_rigid_groups(comp, seen=None):
    if seen is None:
        seen = set()
    for g in getattr(comp, 'rigidGroups', []):
        key = (comp.name, g.name)
        if key not in seen:
            seen.add(key)
            yield comp.name, g
    for occ in comp.occurrences:
        for item in _iter_rigid_groups(occ.component, seen):
            yield item


# ── 트리 진단 ───────────────────────────────────────────────────────────────

def is_visible_to_exporter(j):
    """fusion2urdf 가 실제로 이 조인트를 URDF 에 쓰는가.

    `make_joints_dict` 는 `root.joints` 만 훑으므로 As-built / 하위 컴포넌트 /
    suppressed 는 전부 빠진다.
    """
    return (not j['is_as_built']
            and not j['occurrence_path']
            and not j.get('is_suppressed'))


def _bfs(visible, base=BASE_LINK):
    """base 에서 무향 BFS. `Constraint.reorient_to_root` 와 같은 판정을 한다.

    Returns (loop_idx, unreached_idx, reached_links)
      loop_idx      : 양 끝이 이미 트리에 있는 = **진짜 폐루프를 닫는** 조인트
      unreached_idx : base 에서 도달 못 한 조인트
    """
    adj = collections.defaultdict(list)
    for i, j in enumerate(visible):
        adj[j['parent']].append((i, j['child']))
        adj[j['child']].append((i, j['parent']))

    visited = {base}
    used, loops = set(), []
    q = collections.deque([base])
    while q:
        cur = q.popleft()
        for i, other in adj.get(cur, []):
            if i in used:
                continue
            used.add(i)
            if other in visited:
                loops.append(i)
                continue
            visited.add(other)
            q.append(other)

    unreached = [i for i in range(len(visible)) if i not in used]
    return loops, unreached, visited


def diagnose(joints):
    """URDF 트리로 만들 수 있는지 익스포터를 돌리기 전에 미리 본다.

    익스포터가 실제로 보는 조인트(root 직속 + 활성)만 대상으로 한다 — 그게
    문제가 생기는 집합이다.

    중요한 구분: "링크의 부모가 2개"는 그 자체로는 문제가 아니다. Fusion 의
    component1/component2 순서는 조립 편의상의 산물이라 방향만 어긋난 경우가
    대부분이고, 그건 `Constraint.reorient_to_root` 의 재정렬만으로 해결된다.
    **진짜로 equality constraint 가 필요한 건 무향 그래프에 사이클이 있을 때뿐**
    이므로, 둘을 갈라서 보고한다.
    """
    exported = [j for j in joints if is_visible_to_exporter(j)]
    visible = [j for j in exported if j.get('parent') and j.get('child')]
    broken = [j for j in exported if j not in visible]

    problems_head = [
        "조인트 '{}' 의 parent/child 를 읽지 못했다 — 익스포터가 여기서 죽는다."
        .format(j['name']) for j in broken]

    child_count = collections.Counter(j['child'] for j in visible)
    parents = {j['parent'] for j in visible}
    children = set(child_count)

    problems = list(problems_head)   # 익스포터가 막힌다
    notices = []                     # 자동으로 해결된다 / 알고만 있으면 된다

    if not visible:
        problems.append('익스포터가 볼 수 있는 조인트가 하나도 없다.')
        return {'visible_to_exporter': 0,
                'hidden_from_exporter': len(joints) - len(exported),
                'loop_closing_joints': [], 'problems': problems, 'notices': notices}

    if BASE_LINK not in (parents | children):
        problems.append('base_link 가 어느 조인트에도 안 나온다 — 루트 컴포넌트 '
                        '이름을 base_link 로 바꿔야 한다.')

    loops, unreached, reached = _bfs(visible)

    for i in loops:
        j = visible[i]
        problems.append(
            "조인트 '{}' ({} ↔ {}) 가 폐루프를 닫는다 — URDF <joint> 로는 못 나가므로 "
            'equality constraint 로 빼야 한다. Constraint.plan_export 가 자동으로 '
            '감지해 뺀다.'.format(j['name'], j['parent'], j['child']))

    for i in unreached:
        j = visible[i]
        problems.append(
            "조인트 '{}' ({} ↔ {}) 가 base_link 에서 도달 불가 — 중간 연결이 빠졌다. "
            '아래 "익스포터가 못 보는 조인트" 목록에 그 연결이 있는지 확인하라.'
            .format(j['name'], j['parent'], j['child']))

    loop_links = {visible[i]['child'] for i in loops} | {visible[i]['parent'] for i in loops}
    for link, n in sorted(child_count.items()):
        if n <= 1:
            continue
        owners = [j['name'] for j in visible if j['child'] == link]
        if link in loop_links:
            continue  # 위 폐루프 항목에서 이미 다뤘다
        notices.append(
            "링크 '{}' 의 부모가 {}개다 ({}) — 다만 무향 그래프에 사이클이 없으니 "
            'Fusion 의 component1/component2 순서가 어긋난 것뿐이다. '
            'Constraint.reorient_to_root 의 재정렬만으로 해결되므로 그냥 '
            '익스포터를 돌리면 된다.'.format(link, n, ', '.join(owners)))

    for link in sorted(parents - children - {BASE_LINK}):
        notices.append(
            "링크 '{}' 이 부모가 0개인데 다른 조인트의 parent 로 쓰인다 — "
            '재정렬 전 기준이라 정상이다. 재정렬 후에도 남으면 그때 익스포터가 '
            '막아준다.'.format(link))

    if BASE_LINK in child_count:
        problems.append('base_link 가 어떤 조인트의 child 다 — 루트가 될 수 없다.')

    orphan_links = sorted((parents | children) - reached)
    if orphan_links:
        problems.append('base_link 에서 도달 못 하는 링크: ' + ', '.join(orphan_links))

    return {
        'visible_to_exporter': len(visible),
        'hidden_from_exporter': len(joints) - len(exported),
        'loop_closing_joints': [visible[i]['name'] for i in loops],
        # joints_dict 는 원본 이름으로 키를 잡으므로, Constraint.plan_export 결과와
        # 대조하려면 정규화 전 이름이 필요하다.
        'loop_closing_joints_raw': [visible[i]['name_raw'] for i in loops],
        'problems': problems,
        'notices': notices,
    }


# ── 출력 ────────────────────────────────────────────────────────────────────

CSV_COLUMNS = [
    'name', 'name_raw', 'type', 'parent', 'child',
    'xyz_x', 'xyz_y', 'xyz_z',
    'axis_x', 'axis_y', 'axis_z',
    'upper_limit', 'lower_limit',
    'anchor_child_local_x', 'anchor_child_local_y', 'anchor_child_local_z',
    'is_as_built', 'occurrence_path', 'is_suppressed', 'is_light_bulb_on',
    'visible_to_exporter', 'notes',
]


def _csv_rows(joints):
    for j in joints:
        xyz = j.get('xyz') or ['', '', '']
        axis = j.get('axis') or ['', '', '']
        anc = j.get('anchor_child_local') or ['', '', '']
        visible = is_visible_to_exporter(j)
        yield {
            'name': j['name'], 'name_raw': j['name_raw'], 'type': j['type'],
            'parent': j.get('parent') or '', 'child': j.get('child') or '',
            'xyz_x': xyz[0], 'xyz_y': xyz[1], 'xyz_z': xyz[2],
            'axis_x': axis[0], 'axis_y': axis[1], 'axis_z': axis[2],
            'upper_limit': j.get('upper_limit', ''),
            'lower_limit': j.get('lower_limit', ''),
            'anchor_child_local_x': anc[0],
            'anchor_child_local_y': anc[1],
            'anchor_child_local_z': anc[2],
            'is_as_built': j['is_as_built'],
            'occurrence_path': j['occurrence_path'],
            'is_suppressed': j.get('is_suppressed'),
            'is_light_bulb_on': j.get('is_light_bulb_on'),
            'visible_to_exporter': visible,
            'notes': ' | '.join(j.get('notes') or []),
        }


def write_csv(joints, path):
    import csv
    with open(path, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for row in _csv_rows(joints):
            w.writerow(row)


def build_joints_dict(joints):
    """fusion2urdf 의 joints_dict 와 키 구조가 같은 딕셔너리.

    Fusion 밖에서 `Constraint.plan_export()` 에 그대로 넣어 재정렬/루프 분리를
    돌려볼 수 있게 하려는 것이다. 익스포터가 실제로 보는 조인트만 담는다.
    """
    out = {}
    for j in joints:
        if not is_visible_to_exporter(j):
            continue
        if not j.get('parent') or not j.get('child') or j.get('xyz') is None:
            continue
        t = j['type']
        if t == 'revolute' and j['upper_limit'] == 0.0 and j['lower_limit'] == 0.0:
            t = 'continuous'
        out[j['name_raw']] = {
            'type': t,
            'axis': j['axis'],
            'upper_limit': j['upper_limit'],
            'lower_limit': j['lower_limit'],
            'parent': j['parent'],
            'child': j['child'],
            'xyz': j['xyz'],
        }
    return out


def write_json(joints, rigid_groups, links, diag, design_name, path):
    payload = {
        'design': design_name,
        'units': {'length': 'm', 'rotation': 'rad'},
        'diagnosis': diag,
        'joints': joints,
        'rigid_groups': rigid_groups,
        'links': links,
        'joints_dict': build_joints_dict(joints),
        'joints_dict_note': (
            'fusion2urdf 의 joints_dict 와 키 구조가 동일하다. Fusion 밖에서 '
            'Constraint.plan_export() 에 그대로 넣을 수 있다.'),
    }
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def summary_text(joints, rigid_groups, diag, design_name):
    L = []
    L.append('디자인: {}'.format(design_name))
    L.append('조인트 총 {}개 — 익스포터가 보는 것 {}개 / 못 보는 것 {}개'.format(
        len(joints), diag['visible_to_exporter'], diag['hidden_from_exporter']))
    L.append('Rigid Group {}개 (URDF 에는 안 나간다 — MJCF weld 로 직접 넣어야 함)'
             .format(len(rigid_groups)))
    L.append('')

    if diag['problems']:
        L.append('[막히는 문제 — 고쳐야 익스포터가 돈다]')
        for p in diag['problems']:
            L.append('  - ' + p)
    else:
        L.append('[막히는 문제] 없음')
    L.append('')

    if diag.get('notices'):
        L.append('[자동으로 해결되는 것 — 손댈 필요 없다]')
        for n in diag['notices']:
            L.append('  - ' + n)
        L.append('')

    hidden = [j for j in joints if not is_visible_to_exporter(j)]
    if hidden:
        L.append('[익스포터가 못 보는 조인트 — 이게 트리가 끊기는 흔한 원인이다]')
        for j in hidden:
            why = []
            if j['is_as_built']:
                why.append('As-built')
            if j['occurrence_path']:
                why.append('하위 컴포넌트 {}'.format(j['occurrence_path']))
            if j.get('is_suppressed'):
                why.append('suppressed')
            L.append('  - {} ({} ↔ {})  [{}]'.format(
                j['name'], j.get('parent'), j.get('child'), ', '.join(why)))
        L.append('')

    warned = [j for j in joints if j.get('notes') and is_visible_to_exporter(j)]
    if warned:
        L.append('[그 밖의 경고]')
        for j in warned:
            for n in j['notes']:
                L.append('  - {}: {}'.format(j['name'], n))
        L.append('')

    if rigid_groups:
        L.append('[Rigid Group]')
        for g in rigid_groups:
            L.append('  - {} ({}개): {}'.format(
                g['name'], len(g['occurrences']), ', '.join(g['occurrences'])))

    return '\n'.join(L)


# ── 진입점 ──────────────────────────────────────────────────────────────────

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('활성 문서가 Fusion 360 디자인이 아닙니다.', 'JointInfo Exporter')
            return

        design_name = design.rootComponent.name
        joints, rigid_groups, links, _ = collect(design)
        if not joints:
            ui.messageBox('조인트를 하나도 찾지 못했습니다.', 'JointInfo Exporter')
            return

        diag = diagnose(joints)
        summary = summary_text(joints, rigid_groups, diag, design_name)
        ui.messageBox(summary, 'JointInfo Exporter — 요약')

        dlg = ui.createFolderDialog()
        dlg.title = '접합 정보를 저장할 폴더를 선택하세요'
        if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            ui.messageBox('저장 취소됨 — 파일은 생성되지 않았습니다.\n'
                          '(핵심 결과는 위 요약에 이미 표시됨)', 'JointInfo Exporter')
            return

        base = os.path.join(dlg.folder, sanitize(design_name) + '_jointinfo')
        p_json, p_csv, p_txt = base + '.json', base + '.csv', base + '.txt'
        write_json(joints, rigid_groups, links, diag, design_name, p_json)
        write_csv(joints, p_csv)
        with open(p_txt, 'w', encoding='utf-8') as f:
            f.write(summary)

        ui.messageBox('저장 완료:\n  {}\n  {}\n  {}'.format(
            os.path.basename(p_json), os.path.basename(p_csv),
            os.path.basename(p_txt)), 'JointInfo Exporter')

    except Exception:
        if ui:
            ui.messageBox('JointInfo Exporter 오류:\n' + traceback.format_exc(),
                          'JointInfo Exporter')
