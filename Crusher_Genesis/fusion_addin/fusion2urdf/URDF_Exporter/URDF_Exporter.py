#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils, Constraint
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 
        components = design.allComponents

        # set the names        
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        
        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass     

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'
        
        # --------------------
        # set dictionaries
        
        # Generate joints_dict. All joints are related to root. 
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0   
        
        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0
        
        # --------------------
        # 폐루프 처리: equality constraint 로 쓸 조인트를 골라 URDF 에서 뺀다.
        # URDF 는 링크마다 부모가 하나여야 하는데, Write.write_link_urdf 가
        # "조인트마다 링크 하나"를 찍기 때문에 폐루프가 있으면 같은 링크가 두 번
        # 정의된 불법 URDF 가 나온다(예: 회수장치2 의 Crank_1). 루프를 닫는
        # 조인트를 빼면 트리가 정상화되고, 루프 정보는 사이드카로 보존된다.
        constraint_names, cancelled = Constraint.select_constraint_joints(ui, joints_dict, title)
        if cancelled:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        # equality 종류 판정은 **자르기 전** 전체 joints_dict 로 해야 한다
        # (루프 구성/평면성 판정에 나머지 조인트가 필요하다).
        joints_dict_full = dict(joints_dict)
        joints_dict, constraint_joints = Constraint.split_joints(joints_dict, constraint_names)

        ok, problems = Constraint.validate_tree(joints_dict)
        if not ok:
            ui.messageBox('제약을 뺐는데도 트리가 아닙니다:\n\n  ' + '\n  '.join(problems),
                          title)
            return 0

        links_xyz_dict = {}

        # --------------------
        # Generate URDF
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)
        Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
        Write.write_yaml(package_name, robot_name, save_dir, joints_dict)
        
        # 제약 사이드카 — links_xyz_dict 는 write_urdf 가 채워 놓은 상태여야 하므로
        # URDF 를 다 쓴 뒤에 기록한다.
        if constraint_joints:
            records = Constraint.build_constraint_records(
                constraint_joints, links_xyz_dict, joints_dict_full)
            p_json, p_mjcf = Constraint.write_constraints(records, robot_name, save_dir)
            msg += ('\n\nequality constraint 로 내보낸 조인트 {}개:\n  '.format(len(records))
                    + '\n  '.join('{}  ({} ↔ {})  -> {}\n     {}'.format(
                        r['name'], r['body1'], r['body2'], r['equality'], r['why'])
                                  for r in records)
                    + '\n\n사이드카:\n  ' + os.path.basename(p_json)
                    + '\n  ' + os.path.basename(p_mjcf))

        # copy over package files
        utils.copy_package(save_dir, package_dir)
        utils.update_cmakelists(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate DAE files (surface별 색상 정보 포함)
        utils.copy_occs(root)
        utils.export_dae(design, save_dir, components)
        
        ui.messageBox(msg, title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
