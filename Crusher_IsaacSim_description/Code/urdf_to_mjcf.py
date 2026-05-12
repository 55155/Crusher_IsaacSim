"""
urdf_to_mjcf.py
Crusher_IsaacSim.urdf → Crusher_IsaacSim.xml (MJCF) 변환

Input : C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim.urdf
Output: C:/TEMP/Crusher_IsaacSim_description/Crusher_IsaacSim.xml

변환 규칙:
  URDF fixed      → MJCF body (joint 없음, pos/quat만)
  URDF continuous → MJCF joint type="hinge"  limited="false"
  URDF revolute   → MJCF joint type="hinge"  limited="true" range=...
  URDF prismatic  → MJCF joint type="slide"  limited="true" range=...
  URDF inertial   → MJCF inertial (fullinertia 포맷)
  URDF visual     → MJCF geom type="mesh"  (collision 겸용)
  meshdir         → "urdf"  (MJCF 위치 기준 상대경로)
"""

import xml.etree.ElementTree as ET
import xml.dom.minidom
import numpy as np
from pathlib import Path

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
URDF_PATH = "C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim.urdf"
MJCF_PATH = "C:/TEMP/Crusher_IsaacSim_description/Crusher_IsaacSim.xml"
MESHDIR   = "urdf"   # MJCF 기준 STL 디렉토리 (urdf/ 폴더 내 L-prefix 파일)

# ── 유틸리티 ──────────────────────────────────────────────────────────────────
def parse_floats(s, n=None):
    vals = [float(v) for v in s.strip().split()]
    return vals[:n] if n else vals

def rpy_to_quat(r, p, y):
    """URDF RPY (extrinsic XYZ) → MJCF quaternion (w x y z)"""
    cr, sr = np.cos(r/2), np.sin(r/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cy, sy = np.cos(y/2), np.sin(y/2)
    w =  cr*cp*cy + sr*sp*sy
    x =  sr*cp*cy - cr*sp*sy
    yq = cr*sp*cy + sr*cp*sy
    z =  cr*cp*sy - sr*sp*cy
    return w, x, yq, z

def origin_attrs(origin_elem, default_pos='0 0 0'):
    """<origin> → (pos_str, quat_str)"""
    if origin_elem is None:
        return default_pos, '1 0 0 0'
    xyz = parse_floats(origin_elem.get('xyz', '0 0 0'))
    rpy = parse_floats(origin_elem.get('rpy', '0 0 0'))
    w, x, y, z = rpy_to_quat(*rpy)
    pos_s  = f'{xyz[0]:.8g} {xyz[1]:.8g} {xyz[2]:.8g}'
    quat_s = f'{w:.8g} {x:.8g} {y:.8g} {z:.8g}'
    return pos_s, quat_s

# ── URDF 파싱 ─────────────────────────────────────────────────────────────────
tree = ET.parse(URDF_PATH)
urdf_root = tree.getroot()

links    = {e.get('name'): e for e in urdf_root.findall('link')}
joints   = {e.get('name'): e for e in urdf_root.findall('joint')}

children_of = {}   # parent_link → [(child_link, joint_elem)]
parent_of   = {}   # child_link  → joint_elem

for jelem in urdf_root.findall('joint'):
    parent = jelem.find('parent').get('link')
    child  = jelem.find('child').get('link')
    parent_of[child] = jelem
    children_of.setdefault(parent, []).append((child, jelem))

root_links = [l for l in links if l not in parent_of]
assert len(root_links) == 1, f"루트 링크가 1개여야 합니다: {root_links}"
root_link = root_links[0]   # "fixed_base"
print(f"루트 링크: {root_link}")

# ── 메쉬 에셋 수집 (중복 제거) ───────────────────────────────────────────────
mesh_files = {}   # stem → filename  (e.g. "base_link" → "base_link.stl")
for link_name, link_elem in links.items():
    for tag in ('visual', 'collision'):
        for vis in link_elem.findall(tag):
            m = vis.find('geometry/mesh')
            if m is not None:
                fname = m.get('filename')
                stem  = Path(fname).stem
                mesh_files[stem] = fname   # 중복 시 같은 파일이므로 덮어써도 무방

# ── MJCF 재귀 빌더 ───────────────────────────────────────────────────────────
def build_body(parent_xml, link_name, joint_elem):
    link_elem = links[link_name]

    # body pos/quat = 이 body에 연결된 joint의 origin
    pos_s, quat_s = origin_attrs(
        joint_elem.find('origin') if joint_elem is not None else None
    )

    body = ET.SubElement(parent_xml, 'body',
                          name=link_name, pos=pos_s, quat=quat_s)

    # ── joint ────────────────────────────────────────────────────────────────
    if joint_elem is not None:
        jtype = joint_elem.get('type')
        jname = joint_elem.get('name')

        if jtype in ('continuous', 'revolute'):
            axis_e = joint_elem.find('axis')
            axis   = axis_e.get('xyz', '1 0 0') if axis_e is not None else '1 0 0'
            jattrs = dict(name=jname, type='hinge', pos='0 0 0', axis=axis)
            if jtype == 'revolute':
                lim = joint_elem.find('limit')
                if lim is not None:
                    jattrs.update(limited='true',
                                  range=f"{lim.get('lower','0')} {lim.get('upper','0')}")
            else:
                jattrs['limited'] = 'false'
            ET.SubElement(body, 'joint', **jattrs)

        elif jtype == 'prismatic':
            axis_e = joint_elem.find('axis')
            axis   = axis_e.get('xyz', '1 0 0') if axis_e is not None else '1 0 0'
            lim    = joint_elem.find('limit')
            jattrs = dict(name=jname, type='slide', pos='0 0 0', axis=axis)
            if lim is not None:
                lo = float(lim.get('lower', '-0.1'))
                hi = float(lim.get('upper',  '0.1'))
                if hi > lo:
                    jattrs.update(limited='true', range=f'{lo} {hi}')
                else:
                    # URDF limit이 0~0 인 경우 (Fusion2URDF 기본값) → 제한 없음
                    jattrs['limited'] = 'false'
            ET.SubElement(body, 'joint', **jattrs)
        # fixed: joint 없음 (body pos/quat으로 위치만 지정)

    # ── inertial ─────────────────────────────────────────────────────────────
    ine = link_elem.find('inertial')
    if ine is not None:
        origin_e = ine.find('origin')
        mass_e   = ine.find('mass')
        inr_e    = ine.find('inertia')
        iattrs   = {}
        if origin_e is not None:
            xyz = parse_floats(origin_e.get('xyz', '0 0 0'))
            iattrs['pos'] = f'{xyz[0]:.8g} {xyz[1]:.8g} {xyz[2]:.8g}'
        if mass_e is not None:
            iattrs['mass'] = mass_e.get('value', '0.001')
        if inr_e is not None:
            # MJCF fullinertia 순서: ixx iyy izz ixy ixz iyz
            iattrs['fullinertia'] = (
                f"{inr_e.get('ixx','1e-6')} {inr_e.get('iyy','1e-6')} "
                f"{inr_e.get('izz','1e-6')} {inr_e.get('ixy','0')} "
                f"{inr_e.get('ixz','0')} {inr_e.get('iyz','0')}"
            )
        ET.SubElement(body, 'inertial', **iattrs)

    # ── visual geom (collision 겸용) ─────────────────────────────────────────
    for vis in link_elem.findall('visual'):
        m = vis.find('geometry/mesh')
        if m is None:
            continue
        stem   = Path(m.get('filename')).stem
        pos_g, quat_g = origin_attrs(vis.find('origin'))
        ET.SubElement(body, 'geom',
                       type='mesh', mesh=stem,
                       pos=pos_g, quat=quat_g)

    # ── 자식 body 재귀 ───────────────────────────────────────────────────────
    for child_name, child_joint in children_of.get(link_name, []):
        build_body(body, child_name, child_joint)

# ── MJCF 최상위 구조 ─────────────────────────────────────────────────────────
mujoco_xml = ET.Element('mujoco', model='Crusher_IsaacSim')

ET.SubElement(mujoco_xml, 'compiler',
              meshdir=MESHDIR,
              angle='radian',
              eulerseq='xyz',
              autolimits='true')

ET.SubElement(mujoco_xml, 'option',
              timestep='0.002',
              gravity='0 0 -9.81',
              integrator='implicitfast')

size_e = ET.SubElement(mujoco_xml, 'size', nconmax='200', njmax='500')

default = ET.SubElement(mujoco_xml, 'default')
ET.SubElement(default, 'geom',
              type='mesh',
              contype='1', conaffinity='1',
              friction='0.7 0.005 0.0001',
              rgba='0.78 0.78 0.80 1')
ET.SubElement(default, 'joint',
              damping='0.5', armature='0.001',
              frictionloss='0.01')

# asset: 메쉬 정의
asset = ET.SubElement(mujoco_xml, 'asset')
for stem, fname in sorted(mesh_files.items()):
    ET.SubElement(asset, 'mesh',
                   name=stem, file=fname,
                   scale='0.001 0.001 0.001')

# worldbody
worldbody = ET.SubElement(mujoco_xml, 'worldbody')
build_body(worldbody, root_link, None)   # fixed_base (world 고정)

# actuator: 비-fixed joint 모두 모터로 등록
actuator = ET.SubElement(mujoco_xml, 'actuator')
for jname, jelem in joints.items():
    jtype = jelem.get('type')
    if jtype == 'fixed':
        continue
    child_link = jelem.find('child').get('link')
    if jtype in ('continuous', 'revolute'):
        ET.SubElement(actuator, 'motor',
                       name=f'act_{jname}',
                       joint=jname,
                       gear='1',
                       ctrllimited='false')
    elif jtype == 'prismatic':
        ET.SubElement(actuator, 'motor',
                       name=f'act_{jname}',
                       joint=jname,
                       gear='1',
                       ctrllimited='false')

# ── XML 직렬화 (들여쓰기 포맷) ───────────────────────────────────────────────
raw_xml  = ET.tostring(mujoco_xml, encoding='unicode')
dom      = xml.dom.minidom.parseString(raw_xml)
pretty   = dom.toprettyxml(indent='  ')
# XML 선언 제거 (MuJoCo는 불필요)
lines    = pretty.split('\n')
body_xml = '\n'.join(l for l in lines if not l.startswith('<?xml'))

with open(MJCF_PATH, 'w', encoding='utf-8') as f:
    f.write(body_xml)

print(f"\nMJCF 저장: {MJCF_PATH}")
print(f"  메쉬 에셋: {len(mesh_files)}개")

# ── MuJoCo로 검증 ─────────────────────────────────────────────────────────────
import mujoco
try:
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    print(f"\n[검증 OK] MuJoCo 로드 성공")
    print(f"  bodies : {model.nbody}")
    print(f"  DOF(nv): {model.nv}")
    print(f"  actuators: {model.nu}")
    print(f"  geoms  : {model.ngeom}")
    joint_names = [model.joint(i).name for i in range(model.njnt)]
    print(f"  joints : {joint_names}")
except Exception as e:
    print(f"\n[검증 오류] {e}")
