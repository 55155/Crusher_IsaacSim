# -*- coding: utf-8 -*-
"""
Extract.py — Fusion 디자인에서 조인트/링크/관성을 긁어온다. (`adsk` 필요)

fusion2urdf 의 `core/Joint.py` / `core/Link.py` 를 그대로 따르되 두 가지가 다르다.

1. `root.joints` 만 보지 않고 **하위 컴포넌트 안의 조인트까지** 훑는다.
   fusion2urdf 가 이걸 안 봐서, Fusion 에서는 분명히 붙여놨는데 트리가 끊기는
   일이 생긴다. 폐루프를 닫는 조인트가 하필 그런 위치에 있으면 루프 자체가
   안 보인다 — MJCF 로 가는 이상 그 조인트야말로 equality 로 살려야 할 것이므로
   놓치면 안 된다.

   As-built Joint(한글 UI 의 "현재 위치에서 접합")는 **일부러 안 읽는다**.
   조인트 원점(`geometryOrOriginOne/Two`)이 없고 조립된 자세만 갖고 있어서
   equality anchor 를 뽑을 근거가 없다. 필요해지면 `_iter_joints` 에
   `comp.asBuiltJoints` 를 더하면 되지만, 그때는 원점을 따로 정해줘야 한다.
2. 조인트 원점 계산은 `Joint.py:make_joints_dict` 를 **그대로 복제**했다.
   익스포터가 URDF 에 쓰던 값과 같은 값이 나와야 두 출력을 비교할 수 있다.
"""

import adsk
import adsk.core
import adsk.fusion
import hashlib
import re

# adsk.fusion.JointTypes 인덱스 순서. fusion2urdf 의 joint_type_list 와 같아야
# 하므로 오타(Cylinderical/Planner)까지 그대로 맞춘다.
JOINT_TYPE_LIST = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball',
]

BASE_LINK = 'base_link'


def sanitize(name):
    """Fusion 이름 -> MJCF 에서 쓸 수 있는 식별자.

    fusion2urdf 는 공백/콜론/괄호만 밑줄로 바꿨는데, 그러면 한글이 그대로
    남아 메시 파일명이 된다. MuJoCo 는 그런 파일을 못 연다 — 실측:

        Error: Error opening file 'meshes/키-30-3 v1.stl'

    링크 이름 = 메시 이름 = STL 파일명이 한 몸이라 파일명만 따로 바꿀 수도
    없다. 그래서 여기서 비ASCII 를 통째로 밑줄로 바꾼다. 서로 다른 원본이
    같은 ASCII 이름으로 뭉치면 링크가 조용히 합쳐지므로, 바뀐 경우에만
    원본 해시 4자리를 붙여 구분한다.
    """
    out = re.sub('[ :()]', '_', name)
    safe = re.sub(r'[^A-Za-z0-9_.\-]', '_', out)
    if safe != out:
        stem = safe.strip('_-.') or 'link'
        safe = '{}_{}'.format(
            stem, hashlib.md5(out.encode('utf-8')).hexdigest()[:4])
    return safe


def is_ascii(s):
    try:
        s.encode('ascii')
        return True
    except UnicodeEncodeError:
        return False


def _m3(arr):
    return [round(v / 100.0, 6) for v in arr]      # Fusion 은 cm, 우리는 m


# ── 조인트 ──────────────────────────────────────────────────────────────────

def _iter_joints(comp, occ_path='', seen=None):
    """디자인 전체의 조인트를 (joint, occ_path) 로 재귀 순회.

    조인트는 occurrence 가 아니라 컴포넌트에 속하므로, 같은 컴포넌트가 여러 번
    인스턴스화돼 있어도 한 번만 낸다. As-built Joint 는 다루지 않는다(위 참조).
    """
    if seen is None:
        seen = set()
    for j in comp.joints:
        key = (comp.name, j.name)
        if key in seen:
            continue
        seen.add(key)
        yield j, occ_path
    for occ in comp.occurrences:
        sub = '{}/{}'.format(occ_path, occ.name) if occ_path else occ.name
        for item in _iter_joints(occ.component, sub, seen):
            yield item


def _joint_origin(joint):
    """조인트 원점(root 기준 m). `Joint.py:make_joints_dict` 의 계산을 복제."""
    def trans(M, a):
        ex, ey, ez = [M[0], M[4], M[8]], [M[1], M[5], M[9]], [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        return [a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i] for i in range(3)]

    def allclose(v1, v2, tol=1e-6):
        return max([abs(a - b) for a, b in zip(v1, v2)]) < tol

    try:
        one = joint.geometryOrOriginOne.origin.asArray()
        two = joint.geometryOrOriginTwo.origin.asArray()
        xyz_of_one = joint.occurrenceOne.transform.translation.asArray()
        M_two = joint.occurrenceTwo.transform.asArray()
        if allclose(two, one) or allclose(two, xyz_of_one):
            return _m3(two)
        return _m3(trans(M_two, two))
    except Exception:
        pass
    try:
        if type(joint.geometryOrOriginTwo) == adsk.fusion.JointOrigin:
            return _m3(joint.geometryOrOriginTwo.geometry.origin.asArray())
        return _m3(joint.geometryOrOriginTwo.origin.asArray())
    except Exception:
        return None


def _motion(joint, urdf_type):
    """축과 리밋. (axis, upper, lower, extra, warnings)"""
    axis, upper, lower, extra, warns = [0.0, 0.0, 1.0], 0.0, 0.0, {}, []
    try:
        motion = joint.jointMotion
    except Exception:
        return axis, upper, lower, extra, ['jointMotion 을 읽지 못했다']

    def lim(limits, scale, label):
        try:
            hi_on, lo_on = limits.isMaximumValueEnabled, limits.isMinimumValueEnabled
        except Exception:
            return 0.0, 0.0
        if hi_on and lo_on:
            return (round(limits.maximumValue * scale, 6),
                    round(limits.minimumValue * scale, 6))
        if hi_on != lo_on:
            warns.append("'{}' 의 {} 리밋이 한쪽만 켜져 있다 — 리밋 없이 내보냈다."
                         .format(joint.name, label))
        return 0.0, 0.0

    try:
        if urdf_type in ('revolute', 'Cylinderical', 'PinSlot'):
            axis = [round(i, 6) for i in motion.rotationAxisVector.asArray()]
            upper, lower = lim(motion.rotationLimits, 1.0, '회전')
            extra['current_value'] = float(getattr(motion, 'rotationValue', 0.0))
            if urdf_type == 'PinSlot':
                extra['slide_axis'] = [round(i, 6)
                                       for i in motion.slideDirectionVector.asArray()]
        elif urdf_type == 'prismatic':
            axis = [round(i, 6) for i in motion.slideDirectionVector.asArray()]
            upper, lower = lim(motion.slideLimits, 0.01, '슬라이드')
            extra['current_value'] = float(getattr(motion, 'slideValue', 0.0))
        elif urdf_type == 'Planner':
            extra['normal'] = [round(i, 6) for i in motion.normalDirectionVector.asArray()]
            extra['primary'] = [round(i, 6) for i in motion.primarySlideDirection.asArray()]
    except Exception as e:
        warns.append("'{}' 의 축/리밋을 읽지 못했다: {}".format(joint.name, e))

    return axis, upper, lower, extra, warns


def collect_joints(root, top_level_only=False):
    """{name: {type, axis, upper_limit, lower_limit, parent, child, xyz, ...}}

    fusion2urdf 의 joints_dict 와 키 구조가 같고, `occurrence_path`(하위 컴포넌트
    안의 조인트면 그 경로)를 더 담는다. suppressed 조인트는 건너뛴다.

    top_level_only=True 면 fusion2urdf 와 똑같이 `root.joints` 만 본다.

    Returns (joints, warnings)
    """
    joints, warns = {}, []
    for joint, occ_path in _iter_joints(root):
        try:
            suppressed = bool(getattr(joint, 'isSuppressed', False))
        except Exception:
            suppressed = False
        if suppressed:
            continue
        if top_level_only and occ_path:
            continue

        try:
            idx = joint.jointMotion.jointType
            urdf_type = JOINT_TYPE_LIST[idx]
        except Exception:
            warns.append("'{}' 의 타입을 읽지 못해 건너뛴다.".format(joint.name))
            continue

        occ_one = getattr(joint, 'occurrenceOne', None)
        occ_two = getattr(joint, 'occurrenceTwo', None)
        if occ_one is None or occ_two is None:
            warns.append("'{}' 의 연결 대상을 읽지 못해 건너뛴다.".format(joint.name))
            continue

        xyz = _joint_origin(joint)
        if xyz is None:
            warns.append("'{}' 에 조인트 원점이 없어 건너뛴다 — Fusion 에서 원점을 "
                         '지정해야 한다.'.format(joint.name))
            continue

        axis, upper, lower, extra, w = _motion(joint, urdf_type)
        warns.extend(w)

        if urdf_type == 'revolute' and upper == lower:
            urdf_type = 'continuous'

        parent = (BASE_LINK if occ_two.component.name == BASE_LINK
                  else sanitize(occ_two.name))
        rec = {
            'type': urdf_type,
            'axis': axis,
            'upper_limit': upper,
            'lower_limit': lower,
            'parent': parent,
            'child': sanitize(occ_one.name),
            'xyz': xyz,
            'occurrence_path': occ_path,
        }
        rec.update(extra)

        name = sanitize(joint.name)
        if name in joints:                     # 이름 충돌은 조용히 덮어쓰면 안 된다
            n = 2
            while '{}_{}'.format(name, n) in joints:
                n += 1
            warns.append("조인트 이름 '{}' 이 중복이라 '{}_{}' 로 바꿨다."
                         .format(name, name, n))
            name = '{}_{}'.format(name, n)
        joints[name] = rec

    # 조인트를 드래그해 둔 자세 그대로 내보내면 **조인트 원점이 그 자세로 굳어
    # 들어간다.** 메시는 별개로 배치되므로 기구 치수가 조용히 틀어진다.
    # 실제 사례(docs/DigitalTwin.md §12-3): 크랭크를 사점(180deg)에 둔 채
    # 내보냈더니 크랭크핀과 equality anchor 가 둘 다 정확히 35mm(=2x편심)
    # 밀려, 커넥팅로드 유효길이가 60mm -> 25mm 로 뭉개졌다. 행정은 2x편심이라
    # 편심 방향이 반대여도 같은 값이 나와서 발견이 늦었다.
    posed = [(n, j.get('current_value') or 0.0) for n, j in joints.items()
             if abs(j.get('current_value') or 0.0) > 1e-6]
    if posed:
        warns.append(
            '조인트 {}개가 0 이 아닌 자세로 놓여 있다 — 그 자세가 그대로 MJCF 의 '
            '기준 자세(qpos=0)가 되고 조인트 원점·리밋이 함께 밀린다. Fusion 에서 '
            '모두 0 으로 되돌린 뒤 다시 내보내라: {}'.format(
                len(posed), ', '.join('{}={:.4g}'.format(n, v) for n, v in posed)))

    return joints, warns


# ── 관성 ────────────────────────────────────────────────────────────────────

def _origin2com(inertia, com, mass):
    """월드 원점 기준 관성 -> 질량중심 기준 관성 (평행축 정리).

    fusion2urdf `utils.origin2center_of_mass` 와 같다. 반올림 자릿수도 12 로
    맞춘다 — 6 으로 줄이면 너트/베어링 같은 작은 부품의 관성(~1e-7)이 통째로
    0 이 되고, MuJoCo 는 그런 퇴화 텐서를 거부한다.
    """
    x, y, z = com
    t = [y * y + z * z, x * x + z * z, x * x + y * y, -x * y, -y * z, -x * z]
    return [round(i - mass * k, 12) for i, k in zip(inertia, t)]


def collect_inertia(root):
    """{link: {mass, center_of_mass(root 기준 m), inertia}}  (dict, warnings)"""
    out, warns = {}, []
    for occ in root.occurrences:
        try:
            prop = occ.getPhysicalProperties(
                adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
            mass = prop.mass
            com = [v / 100.0 for v in prop.centerOfMass.asArray()]
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            world = [v / 10000.0 for v in (xx, yy, zz, xy, yz, xz)]  # kg·cm² -> kg·m²
            name = (BASE_LINK if occ.component.name == BASE_LINK
                    else sanitize(occ.name))
            out[name] = {
                'mass': mass,
                'center_of_mass': com,
                'inertia': _origin2com(world, com, mass),
            }
        except Exception as e:
            warns.append("'{}' 의 물성을 읽지 못했다: {}".format(occ.name, e))
    return out, warns


def link_names(root):
    """최상위 occurrence 이름 목록 — UI 드롭다운에 쓴다."""
    names = []
    for occ in root.occurrences:
        names.append(BASE_LINK if occ.component.name == BASE_LINK
                     else sanitize(occ.name))
    return sorted(set(names))


def collect_renames(root):
    """sanitize 가 이름을 바꾼 것들. [(원본, 바뀐이름), ...]

    비ASCII 때문에 바뀐 이름은 Fusion 에서 보이는 것과 MJCF 안의 것이 달라
    사람이 헷갈린다. 리포트에 적어 둔다.
    """
    out = []
    for occ in root.occurrences:
        if occ.component.name == BASE_LINK:
            continue
        new = sanitize(occ.name)
        if new != re.sub('[ :()]', '_', occ.name):
            out.append((occ.name, new))
    for joint, _ in _iter_joints(root):
        try:
            old = joint.name
        except Exception:
            continue
        new = sanitize(old)
        if new != re.sub('[ :()]', '_', old):
            out.append((old, new))
    return sorted(set(out))
