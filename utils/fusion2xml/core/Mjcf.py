# -*- coding: utf-8 -*-
"""
Mjcf.py — body 트리와 관성 정보를 MuJoCo MJCF XML 로 찍는다.

`adsk` 에 의존하지 않는다 — Fusion 밖에서 단위 테스트할 수 있다.

URDF 와 다른 점 (여기서 실수하기 쉬운 것들)
------------------------------------------
1. `<body pos>` 는 **부모 body 프레임 기준**이다. URDF 의 `<joint><origin>` 이
   맡던 역할을 body 자신이 가져간다.
2. `<joint>` 는 body 안에 들어가고 그 `pos` 는 body 프레임 기준이다. 우리 규약은
   링크 프레임 원점을 그 링크의 부모 조인트 위치에 두므로 항상 `0 0 0` 이다.
3. `fullinertia` 의 순서는 URDF 와 다르다:
       URDF   ixx iyy izz ixy iyz ixz
       MJCF   ixx iyy izz ixy ixz iyz   <- 뒤 두 개가 뒤바뀐다
   그대로 옮겨 쓰면 관성이 조용히 틀어진다.
4. MuJoCo 는 관성 텐서가 양정치가 아니거나 주모멘트가 삼각부등식
   (A+B >= C) 을 어기면 **로딩을 거부**한다. URDF/Gazebo 는 그냥 넘어가므로
   fusion2urdf 로는 안 드러나던 문제가 여기서 처음 터진다. 그래서 여기서 미리
   검사해 어느 링크가 문제인지 알려준다.
"""

import math
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom


def _f(x):
    """XML 속성용 실수 포맷. `+ 0.0` 은 -0.0 을 0 으로 만들기 위한 것."""
    return '{:.9g}'.format(float(x) + 0.0)


def _v(vec):
    return ' '.join(_f(x) for x in vec)


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


# ── 관성 검증 ───────────────────────────────────────────────────────────────

def sym3_eigenvalues(ixx, iyy, izz, ixy, iyz, ixz):
    """대칭 3x3 행렬의 고유값 3개(오름차순). 해석해라 반복이 없다."""
    p1 = ixy * ixy + ixz * ixz + iyz * iyz
    if p1 == 0.0:
        return sorted([ixx, iyy, izz])
    q = (ixx + iyy + izz) / 3.0
    p2 = ((ixx - q) ** 2 + (iyy - q) ** 2 + (izz - q) ** 2) + 2.0 * p1
    p = math.sqrt(p2 / 6.0)
    if p == 0.0:
        return sorted([ixx, iyy, izz])
    # B = (A - qI)/p 의 행렬식 / 2
    b11, b22, b33 = (ixx - q) / p, (iyy - q) / p, (izz - q) / p
    b12, b13, b23 = ixy / p, ixz / p, iyz / p
    detB = (b11 * (b22 * b33 - b23 * b23)
            - b12 * (b12 * b33 - b23 * b13)
            + b13 * (b12 * b23 - b22 * b13))
    r = max(-1.0, min(1.0, detB / 2.0))
    phi = math.acos(r) / 3.0
    e1 = q + 2.0 * p * math.cos(phi)
    e3 = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    e2 = 3.0 * q - e1 - e3
    return sorted([e1, e2, e3])


def check_inertia(name, mass, inertia):
    """MuJoCo 가 받아줄 관성인지 본다. (ok, [문제 설명, ...])

    inertia 는 fusion2urdf 순서 [ixx, iyy, izz, ixy, iyz, ixz].
    """
    problems = []
    if mass is None or mass <= 0.0:
        problems.append("링크 '{}' 의 질량이 {} 다 — MuJoCo 는 움직이는 body 에 "
                        '양의 질량을 요구한다.'.format(name, mass))
        return False, problems

    ixx, iyy, izz, ixy, iyz, ixz = inertia
    eig = sym3_eigenvalues(ixx, iyy, izz, ixy, iyz, ixz)
    if eig[0] <= 0.0:
        problems.append("링크 '{}' 의 관성 텐서가 양정치가 아니다 (최소 고유값 "
                        '{:.3g}) — MuJoCo 가 로딩을 거부한다.'.format(name, eig[0]))
    elif eig[0] + eig[1] < eig[2] * (1.0 - 1e-9):
        problems.append("링크 '{}' 의 주모멘트가 삼각부등식을 어긴다 "
                        '({:.3g} + {:.3g} < {:.3g}) — compiler 의 '
                        'balanceinertia 가 값을 보정한다.'.format(
                            name, eig[0], eig[1], eig[2]))
    return (not problems), problems


def fullinertia_mjcf(inertia):
    """fusion2urdf 순서 [ixx,iyy,izz,ixy,iyz,ixz] -> MJCF 순서 ixx iyy izz ixy ixz iyz."""
    ixx, iyy, izz, ixy, iyz, ixz = inertia
    return [ixx, iyy, izz, ixy, ixz, iyz]


# ── 조인트 매핑 ─────────────────────────────────────────────────────────────

def joint_elements(joint):
    """Fusion/URDF 조인트 하나 -> MJCF `<joint>` 속성 목록. (elems, warnings)

    fixed 는 조인트를 안 만든다 — MJCF 에서는 body 중첩 자체가 고정 결합이다.
    """
    t = joint['type']
    axis = joint.get('axis') or [0, 0, 1]
    lo = joint.get('lower_limit', 0.0)
    hi = joint.get('upper_limit', 0.0)
    name = joint['name']
    warns = []

    def hinge(nm, ax, limited):
        a = {'name': nm, 'type': 'hinge', 'pos': '0 0 0', 'axis': _v(ax)}
        if limited:
            a['range'] = '{} {}'.format(_f(lo), _f(hi))
        return a

    def slide(nm, ax, limited):
        a = {'name': nm, 'type': 'slide', 'pos': '0 0 0', 'axis': _v(ax)}
        if limited:
            a['range'] = '{} {}'.format(_f(lo), _f(hi))
        return a

    if t == 'fixed':
        return [], warns
    if t == 'revolute':
        return [hinge(name, axis, lo != hi)], warns
    if t == 'continuous':
        return [hinge(name, axis, False)], warns
    if t == 'prismatic':
        return [slide(name, axis, lo != hi)], warns
    if t == 'Ball':
        return [{'name': name, 'type': 'ball', 'pos': '0 0 0'}], warns
    if t == 'Cylinderical':
        warns.append("'{}' 는 Cylindrical(회전1+병진1) 이라 hinge + slide 두 개로 "
                     '쪼갰다. 리밋은 회전 쪽에만 붙였다.'.format(name))
        return [hinge(name, axis, lo != hi),
                slide(name + '_slide', axis, False)], warns
    if t == 'PinSlot':
        warns.append("'{}' 는 PinSlot 이라 hinge + slide 두 개로 쪼갰다. 미끄럼 "
                     '방향은 Fusion 의 slide 축을 그대로 썼다.'.format(name))
        return [hinge(name, axis, lo != hi),
                slide(name + '_slide', joint.get('slide_axis') or [1, 0, 0], False)], warns
    if t == 'Planner':
        warns.append("'{}' 는 Planar(2병진+1회전) 이라 slide 2 + hinge 1 로 "
                     '쪼갰다. 축은 법선에서 유도했으므로 확인이 필요하다.'.format(name))
        n = joint.get('normal') or [0, 0, 1]
        u = joint.get('primary') or _perp(n)
        w = _cross(n, u)
        return [slide(name + '_x', u, False),
                slide(name + '_y', w, False),
                hinge(name, n, False)], warns

    warns.append("'{}' 의 타입 '{}' 은 MJCF 로 옮길 방법이 없어 고정 결합으로 "
                 '뒀다.'.format(name, t))
    return [], warns


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _perp(n):
    """n 과 수직인 아무 단위벡터."""
    a = [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0]
    v = _cross(n, a)
    L = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / L for x in v]


# ── MJCF 조립 ───────────────────────────────────────────────────────────────

DEFAULT_OPTIONS = {
    'mesh_dir': 'meshes',
    'mesh_scale': 0.001,        # Fusion STL 은 mm, MuJoCo 는 m
    'fix_base': True,           # base_link 를 world 에 고정
    'actuator': 'position',     # 'none' | 'motor' | 'position'
    'position_kp': 100.0,
    'joint_damping': 0.1,
    'timestep': 0.002,
    'add_collision': True,
    'rgba': '0.7 0.7 0.7 1',
}


def build(model_name, plan, inertial_dict, mesh_names=(), options=None):
    """MJCF 문자열을 만든다. (xml_str, warnings)

    Parameters
    ----------
    plan : Tree.plan() 의 반환값
    inertial_dict : {link: {mass, center_of_mass(root 기준 m), inertia}}
    mesh_names : STL 이 실제로 존재하는 링크 이름 집합
    """
    opt = dict(DEFAULT_OPTIONS)
    opt.update(options or {})
    warnings = []
    mesh_names = set(mesh_names)
    origins = plan['origins']

    mujoco = Element('mujoco', {'model': model_name})

    SubElement(mujoco, 'compiler', {
        'angle': 'radian',
        'meshdir': opt['mesh_dir'],
        'autolimits': 'true',
        # 관성이 삼각부등식을 어기면 MuJoCo 가 로딩을 거부한다. 어느 링크가
        # 문제인지는 아래 check_inertia 가 따로 보고하므로, 여기서는 일단
        # 로딩은 되게 해 둔다.
        'balanceinertia': 'true',
    })
    SubElement(mujoco, 'option', {'timestep': _f(opt['timestep']),
                                  'integrator': 'implicitfast'})

    default = SubElement(mujoco, 'default')
    SubElement(default, 'joint', {'damping': _f(opt['joint_damping'])})
    SubElement(default, 'geom', {'rgba': opt['rgba']})

    # asset
    if mesh_names:
        asset = SubElement(mujoco, 'asset')
        s = opt['mesh_scale']
        for name in sorted(mesh_names):
            SubElement(asset, 'mesh', {
                'name': name, 'file': name + '.stl',
                'scale': '{} {} {}'.format(_f(s), _f(s), _f(s))})

    # worldbody
    worldbody = SubElement(mujoco, 'worldbody')
    SubElement(worldbody, 'light', {'pos': '0 0 2', 'dir': '0 0 -1',
                                    'directional': 'true'})

    actuated = []

    def emit(node, parent_elem, parent_origin, is_root=False):
        name = node['name']
        origin = origins.get(name, [0.0, 0.0, 0.0])
        body = SubElement(parent_elem, 'body', {
            'name': name, 'pos': _v(_sub(origin, parent_origin))})

        # base_link 를 world 에 고정하지 않으면 6자유도 free joint 를 준다.
        if is_root and not opt['fix_base']:
            SubElement(body, 'freejoint', {'name': name + '_free'})

        for j in node['joints']:
            elems, w = joint_elements(j)
            warnings.extend(w)
            for attrib in elems:
                SubElement(body, 'joint', attrib)
                if attrib.get('type') in ('hinge', 'slide'):
                    actuated.append(attrib['name'])

        # inertial — 메시에서 추정하게 두지 않고 Fusion 의 실측값을 쓴다
        info = inertial_dict.get(name)
        if info:
            _, probs = check_inertia(name, info.get('mass'), info.get('inertia'))
            warnings.extend(probs)
            # 질량이 0 이하면 <inertial> 을 쓰는 순간 MuJoCo 가 거부하므로 아예
            # 빼서 기하에서 추정하게 둔다. 삼각부등식 위반은 balanceinertia 가
            # 보정해 주므로 값을 그대로 내보낸다(위에 경고는 남겼다).
            if (info.get('mass') or 0.0) > 0.0:
                com_local = _sub(info['center_of_mass'], origin)
                SubElement(body, 'inertial', {
                    'pos': _v(com_local),
                    'mass': _f(info['mass']),
                    'fullinertia': _v(fullinertia_mjcf(info['inertia']))})
        else:
            warnings.append("링크 '{}' 의 관성 정보가 없다 — MuJoCo 가 기하에서 "
                            '추정한다.'.format(name))

        if name in mesh_names:
            SubElement(body, 'geom', {
                'name': name + '_vis', 'type': 'mesh', 'mesh': name,
                'pos': _v([-c for c in origin]),
                'contype': '0', 'conaffinity': '0', 'group': '2'})
            if opt['add_collision']:
                SubElement(body, 'geom', {
                    'name': name + '_col', 'type': 'mesh', 'mesh': name,
                    'pos': _v([-c for c in origin]), 'group': '3'})
        else:
            warnings.append("링크 '{}' 에 해당하는 STL 이 없어 geom 을 못 넣었다 — "
                            'MuJoCo 는 기하도 관성도 없는 body 를 거부한다.'.format(name))

        for c in node['children']:
            emit(c, body, origin)

    emit(plan['body_tree'], worldbody, [0.0, 0.0, 0.0], is_root=True)

    # equality — 여기가 URDF 로는 못 하던 부분이다
    if plan['equalities']:
        eq = SubElement(mujoco, 'equality')
        for i, e in enumerate(plan['equalities']):
            b1, b2 = e['body1'], e['body2']
            nm = '{}_{}_{}'.format(e['type'], b1, b2)
            if e['type'] == 'weld':
                # relpose 는 body2 를 body1 기준으로 본 상대 자세다. 모든 프레임이
                # 축정렬이라 회전은 단위 사원수, 병진은 원점 차이로 정확히 나온다.
                rel = _sub(origins.get(b2, [0, 0, 0]), origins.get(b1, [0, 0, 0]))
                SubElement(eq, 'weld', {
                    'name': nm, 'body1': b1, 'body2': b2,
                    'relpose': '{} 1 0 0 0'.format(_v(rel))})
            else:
                anchor = _sub(e['anchor_root'], origins.get(b1, [0, 0, 0]))
                SubElement(eq, 'connect', {
                    'name': nm, 'body1': b1, 'body2': b2, 'anchor': _v(anchor)})

    # actuator
    if opt['actuator'] != 'none' and actuated:
        act = SubElement(mujoco, 'actuator')
        for jn in actuated:
            if opt['actuator'] == 'position':
                SubElement(act, 'position', {
                    'name': jn + '_act', 'joint': jn, 'kp': _f(opt['position_kp'])})
            else:
                SubElement(act, 'motor', {'name': jn + '_act', 'joint': jn})

    return _prettify(mujoco), warnings


def _prettify(elem):
    rough = tostring(elem, 'utf-8')
    return minidom.parseString(rough).toprettyxml(indent='  ')
