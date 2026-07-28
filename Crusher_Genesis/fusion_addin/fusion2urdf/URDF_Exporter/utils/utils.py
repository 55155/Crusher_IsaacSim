# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
import shutil  # Replaced distutils with shutil
import fileinput
import sys

def copy_occs(root):    
    """    
    duplicate all the components
    """    
    def copy_body(allOccs, occs):
        """    
        copy the old occs to new component
        """
        
        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()
        
        # Create new components from occs
        # This support even when a component has some occses. 

        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
        new_occs = allOccs.item((allOccs.count-1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)
    
    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


def export_stl(design, save_dir, components):  
    """
    export stl files into "save_dir/"
    
    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """
          
    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'  
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name              
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + ' has something wrong.')


def _body_appearance(body, occ):
    """
    body -> occurrence 순으로 상속 체인을 따라가며
    body(부품 전체)에 적용된 appearance를 찾는다.
    """
    return body.appearance or occ.appearance


def _appearance_rgba(appearance):
    """
    appearance의 대표 색(ColorProperty)을 (r, g, b, a) (0~1)로 반환한다.
    색 정보가 없으면 무채색 회색으로 대체한다.
    """
    if appearance:
        for prop in appearance.appearanceProperties:
            if type(prop) is adsk.core.ColorProperty and prop.value:
                c = prop.value
                return (c.red / 255.0, c.green / 255.0, c.blue / 255.0, c.opacity / 255.0)
    return (0.7, 0.7, 0.7, 1.0)


def export_dae(design, save_dir, components):
    """
    export dae(COLLADA) files into "save_dir/meshes"

    STL과 달리 body(부품) 단위로 지정된 appearance 색상을 그대로 반영한다.
    Fusion의 exportManager는 DAE를 직접 지원하지 않으므로, body 단위로
    tessellation(meshManager)한 뒤 색상별로 묶어서 COLLADA XML을 직접 작성한다.
    실행할 때마다 어떤 색이 뽑혔는지 콘솔(TEXT COMMANDS 창)에 로그를 남긴다.

    component/occurrence 레벨에서만 지정된 appearance는 copy_occs()의 복사
    과정에서 유실될 수 있다. body에 직접 칠해진 색만 보존이 보장된다.

    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'

    ok_count = 0
    fail_count = 0

    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name + '.dae'
                    _write_dae(occ, fileName)
                    ok_count += 1
                except:
                    fail_count += 1
                    print('Component ' + occ.component.name + ' has something wrong.')

    print('===== export_dae summary: %d ok, %d failed =====' % (ok_count, fail_count))


def _write_dae(occ, fileName):
    """
    하나의 occurrence(부품)를 body별 색상을 보존한 COLLADA(.dae)로 저장한다.
    (한 occurrence에 body가 여러 개면 body마다 다른 색을 가질 수 있다)

    Fusion API 좌표는 항상 cm 단위이므로, 기존 STL 파이프라인(mm 단위,
    urdf의 mesh scale="0.001")과 동일하게 맞추기 위해 10을 곱해 mm로 환산한다.
    """
    CM_TO_MM = 10.0

    # appearance.id -> {'rgba', 'tri_indices'}
    groups = {}
    material_order = []
    all_positions = []
    all_normals = []
    node_count = 0

    for body in occ.bRepBodies:
        meshCalc = body.meshManager.createMeshCalculator()
        meshCalc.setQuality(adsk.fusion.TriangleMeshQualityOptions.LowQualityTriangleMesh)
        triMesh = meshCalc.calculate()

        appearance = _body_appearance(body, occ)
        rgba = _appearance_rgba(appearance)
        print('  [color] body=%s appearance=%s rgba=(%.2f, %.2f, %.2f, %.2f)' % (
            body.name,
            appearance.name if appearance else 'NONE -> default gray',
            rgba[0], rgba[1], rgba[2], rgba[3]))

        if triMesh is None:
            print('  [color] body=%s: tessellation failed, skipped' % body.name)
            continue

        coords = triMesh.nodeCoordinatesAsDouble
        normals = triMesh.normalVectorsAsDouble
        tri_idx = triMesh.nodeIndices
        if not coords or not tri_idx:
            continue

        key = appearance.id if appearance else 'default'
        if key not in groups:
            groups[key] = {'rgba': rgba, 'tri_indices': []}
            material_order.append(key)

        for i in range(0, len(coords), 3):
            all_positions.append(coords[i] * CM_TO_MM)
            all_positions.append(coords[i + 1] * CM_TO_MM)
            all_positions.append(coords[i + 2] * CM_TO_MM)

        if normals and len(normals) == len(coords):
            all_normals.extend(normals)
        else:
            all_normals.extend([0.0, 0.0, 1.0] * (len(coords) // 3))

        groups[key]['tri_indices'].extend(idx + node_count for idx in tri_idx)
        node_count += len(coords) // 3

    if not material_order:
        return

    mesh_name = occ.component.name
    _write_dae_xml(fileName, mesh_name, material_order, groups, all_positions, all_normals)


def _write_dae_xml(fileName, mesh_name, material_order, groups, positions, normals):
    """
    수집된 vertex/normal/재질별 triangle 인덱스를 COLLADA 1.4.1 XML 문자열로 작성한다.
    """
    mesh_id = mesh_name + '-mesh'

    effect_blocks = []
    material_blocks = []
    triangles_blocks = []
    bind_material_blocks = []

    for i, key in enumerate(material_order):
        group = groups[key]
        mat_id = 'Material_%03d' % i
        r, g, b, a = group['rgba']

        effect_blocks.append(
            '    <effect id="%s-effect">\n'
            '      <profile_COMMON>\n'
            '        <technique sid="common">\n'
            '          <phong>\n'
            '            <emission><color sid="emission">0 0 0 1</color></emission>\n'
            '            <ambient><color sid="ambient">0 0 0 1</color></ambient>\n'
            '            <diffuse><color sid="diffuse">%.6g %.6g %.6g %.6g</color></diffuse>\n'
            '            <specular><color sid="specular">0.5 0.5 0.5 1</color></specular>\n'
            '            <shininess><float sid="shininess">10</float></shininess>\n'
            '          </phong>\n'
            '        </technique>\n'
            '      </profile_COMMON>\n'
            '    </effect>\n' % (mat_id, r, g, b, a)
        )
        material_blocks.append(
            '    <material id="%s-material" name="%s">\n'
            '      <instance_effect url="#%s-effect"/>\n'
            '    </material>\n' % (mat_id, mat_id, mat_id)
        )

        tri_indices = group['tri_indices']
        p_str = ' '.join('%d %d' % (n, n) for n in tri_indices)
        triangles_blocks.append(
            '        <triangles material="%s-material" count="%d">\n'
            '          <input semantic="VERTEX" source="#%s-vertices" offset="0"/>\n'
            '          <input semantic="NORMAL" source="#%s-normals" offset="1"/>\n'
            '          <p>%s</p>\n'
            '        </triangles>\n' % (mat_id, len(tri_indices) // 3, mesh_id, mesh_id, p_str)
        )
        bind_material_blocks.append(
            '              <instance_material symbol="%s-material" target="#%s-material"/>\n' % (mat_id, mat_id)
        )

    pos_str = ' '.join('%.6g' % v for v in positions)
    norm_str = ' '.join('%.6g' % v for v in normals)

    dae = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
        '  <asset>\n'
        '    <contributor><authoring_tool>fusion2urdf export_dae</authoring_tool></contributor>\n'
        '    <unit name="millimeter" meter="0.001"/>\n'
        '    <up_axis>Z_UP</up_axis>\n'
        '  </asset>\n'
        '  <library_effects>\n%s  </library_effects>\n'
        '  <library_materials>\n%s  </library_materials>\n'
        '  <library_geometries>\n'
        '    <geometry id="%s" name="%s">\n'
        '      <mesh>\n'
        '        <source id="%s-positions">\n'
        '          <float_array id="%s-positions-array" count="%d">%s</float_array>\n'
        '          <technique_common>\n'
        '            <accessor source="#%s-positions-array" count="%d" stride="3">\n'
        '              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>\n'
        '            </accessor>\n'
        '          </technique_common>\n'
        '        </source>\n'
        '        <source id="%s-normals">\n'
        '          <float_array id="%s-normals-array" count="%d">%s</float_array>\n'
        '          <technique_common>\n'
        '            <accessor source="#%s-normals-array" count="%d" stride="3">\n'
        '              <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>\n'
        '            </accessor>\n'
        '          </technique_common>\n'
        '        </source>\n'
        '        <vertices id="%s-vertices">\n'
        '          <input semantic="POSITION" source="#%s-positions"/>\n'
        '        </vertices>\n'
        '%s'
        '      </mesh>\n'
        '    </geometry>\n'
        '  </library_geometries>\n'
        '  <library_visual_scenes>\n'
        '    <visual_scene id="Scene" name="Scene">\n'
        '      <node id="%s" name="%s" type="NODE">\n'
        '        <instance_geometry url="#%s" name="%s">\n'
        '          <bind_material>\n'
        '            <technique_common>\n'
        '%s'
        '            </technique_common>\n'
        '          </bind_material>\n'
        '        </instance_geometry>\n'
        '      </node>\n'
        '    </visual_scene>\n'
        '  </library_visual_scenes>\n'
        '  <scene><instance_visual_scene url="#Scene"/></scene>\n'
        '</COLLADA>\n'
    ) % (
        ''.join(effect_blocks),
        ''.join(material_blocks),
        mesh_id, mesh_name,
        mesh_id, mesh_id, len(positions), pos_str, mesh_id, len(positions) // 3,
        mesh_id, mesh_id, len(normals), norm_str, mesh_id, len(normals) // 3,
        mesh_id, mesh_id,
        ''.join(triangles_blocks),
        mesh_name, mesh_name, mesh_id, mesh_name,
        ''.join(bind_material_blocks),
    )

    with open(fileName, 'w', encoding='utf-8') as f:
        f.write(dae)


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2,
                         -x*y, -y*z, -x*z]
    # 소수점 6자리 반올림은 작은 부품(너트/베어링 등, 관성 ~1e-7 이하)의 관성을
    # 통째로 0.0으로 뭉갬 — 다운스트림 시뮬레이터(URDF/MJCF 컴파일)가 이런
    # 퇴화 관성 텐서(0 또는 삼각부등식 위반)를 거부한다. 부동소수점 표현
    # 노이즈 정리가 목적이었을 뿐 유의미한 정밀도 손실은 의도가 아니었으므로,
    # 자릿수를 12로 올려 소형 부품의 실제 관성값을 보존한다.
    return [round(i - mass*t, 12) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element
    
    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        # Check if the target directory exists, if not, create it
        if not os.path.exists(save_dir + '/launch'):
            os.mkdir(save_dir + '/launch')
        if not os.path.exists(save_dir + '/urdf'):
            os.mkdir(save_dir + '/urdf')
        
        # Check if the package directory exists and copy it
        if os.path.exists(package_dir):
            shutil.copytree(package_dir, save_dir, dirs_exist_ok=True)  # dirs_exist_ok=True allows overwriting
        else:
            print(f"Package directory '{package_dir}' does not exist.")
        
    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
