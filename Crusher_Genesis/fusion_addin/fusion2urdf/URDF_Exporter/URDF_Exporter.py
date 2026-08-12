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
        # 폐루프 처리
        #
        # URDF 는 링크마다 부모가 하나여야 하는데, Fusion 의 component1/component2
        # 순서는 조립 편의상의 산물이라 그대로 쓰면 어떤 링크는 부모가 둘, 어떤
        # 링크는 0개가 된다(실측: Crank_1 부모 2개 / M_Top_1 부모 0개). 그래서
        #   1) base_link 기준으로 모든 조인트 방향을 다시 세우고
        #   2) 루프를 닫는 조인트는 URDF 에서 빼 사이드카에 접합 위치만 남긴다.
        joints_dict_full = dict(joints_dict)
        constraint_names, cancelled = Constraint.select_constraint_joints(
            ui, joints_dict, title)
        if cancelled:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        plan = Constraint.plan_export(joints_dict, constraint_names)
        if plan['problems']:
            ui.messageBox('URDF 트리를 만들 수 없습니다:\n\n  ' +
                          '\n  '.join(plan['problems']), title)
            return 0
        joints_dict = plan['tree']
        constraint_joints = plan['constraints']

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

        # 제약 사이드카 — anchor 를 body 로컬로 환산하려면 links_xyz_dict 가
        # 채워져 있어야 하므로 URDF 를 다 쓴 뒤에 기록한다.
        if constraint_joints:
            records = Constraint.build_constraint_records(
                constraint_joints, links_xyz_dict, joints_dict_full)
            p_json, p_mjcf = Constraint.write_constraints(records, robot_name, save_dir)
            msg += ('\n\nURDF <joint> 에서 빼고 equality 로 내보낸 조인트 {}개:\n  '
                    .format(len(records))
                    + '\n  '.join('{}  ({} ↔ {})  -> {}\n     anchor(body1 로컬) = {}'
                                  .format(r['name'], r['body1'], r['body2'],
                                          r['equality'], r['anchor_body1_local'])
                                  for r in records)
                    + '\n\n사이드카:\n  ' + os.path.basename(p_json)
                    + '\n  ' + os.path.basename(p_mjcf))
        if plan['flipped']:
            msg += ('\n\nbase_link 기준으로 방향을 교정한 조인트 {}개:\n  '
                    .format(len(plan['flipped'])) + ', '.join(sorted(plan['flipped'])))

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
