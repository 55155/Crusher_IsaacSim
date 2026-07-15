"""
slot_ik_check.py — full_workflow.py 의 above/insert IK 목표를 무거운 IPC+FEM
씬을 빌드하지 않고 저비용으로 검증한다(Rigid-only, Crusher+Robot 만, coupler 기본값).

목적(사용자 지시, 2026-07-15 4차): "Scene 전체를 빌드하기 이전에 이거 위치만
잘 조정하고 가자" — above/insert 목표에서 로봇팔(손가락 제외 링크)이 Crusher
구조물과 충돌/침투하는지 rigid contact 로 빠르게 확인.

목표 위치 계산(이번 라운드 변경):
  - XY: 기존엔 gap 중심(Wall3~Left_Wall 사이 슬릿의 중점)을 그대로 썼는데,
    실측 결과 봉투가 항상 그 중심보다 로봇 쪽으로 짧게 도착했다(§full_workflow.py
    run4/run5 로그). 원인은 슬릿 중심이 곧 포켓의 물리적 중심이 아니라는 것 —
    L1_Wall1_1(포켓 바닥 플레이트, Crusher_Samplebag.py 주석 "L1_Wall1_1 은
    바닥 플레이트") 의 world AABB 중심을 XY 타깃으로 쓴다(사용자 지시: "봉투의
    가장 하단이 Wall_1.stl 의 거의 중앙쯤").
  - Z: Crusher_Samplebag.py 의 partial-insertion 규칙(포켓 깊이의 세로 중앙에서
    정지)을 그대로 재사용 — bag_bottom_target = wall_top_z - wall_depth/2,
    gripper_z = bag_bottom_target + BAG_HALF_H.

실행: python slot_ik_check.py
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_workflow as fw

import genesis as gs

gs.init(backend=gs.gpu, logging_level="warning", precision="32")

crusher_xml = fw._prepare_crusher_mjcf()
robot_xml = fw._prepare_robot_mjcf()

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=fw.DT, gravity=(0, 0, -9.81)),
    show_viewer=False,
)
scene.add_entity(gs.morphs.Plane())
crusher = scene.add_entity(
    gs.morphs.MJCF(file=crusher_xml, pos=fw.CRUSHER_POS, euler=fw.CRUSHER_EULER,
                   decimate=True, convexify=True),
)
robot = scene.add_entity(
    gs.morphs.MJCF(file=robot_xml, pos=tuple(fw.ROBOT_OFFSET), decimate=False),
)
print("[build] rigid-only scene.build() 시작...")
scene.build(n_envs=0)
print("[build] 성공 (IPC/FEM 없이 rigid만 — 훨씬 빠름)")

crusher_joints = {j.name: j for j in crusher.joints if j.name}
def _scalar_dof(name):
    d = crusher_joints[name].dofs_idx_local
    return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d
crank_dof = _scalar_dof(fw.CRANK_JOINT)
wall_dof = _scalar_dof(fw.WALL_JOINT)
crusher.set_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
crusher.set_dofs_position(np.array([fw.WALL_OFFSET]), dofs_idx_local=[wall_dof])

gripper_link = robot.get_link("gripper_body")

# ── 슬롯/포켓 기하 ───────────────────────────────────────────────────────────
wb_lo, wb_hi = fw.crusher_mesh_world_aabb(fw.WALL_BACK_MESH)
wl_lo, wl_hi = fw.crusher_mesh_world_aabb(fw.WALL_LEFT_MESH, fw.LEFTWALL_BODY_POS, fw.LEFTWALL_GEOM_POS)
gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
gap_cx = (gap_lo_x + gap_hi_x) / 2.0
y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
gap_cy = (y_lo + y_hi) / 2.0
wall_top_z = max(wb_hi[2], wl_hi[2])
wall_depth = wall_top_z - wb_lo[2]

w1_lo, w1_hi = fw.crusher_mesh_world_aabb("L1_Wall1_1")
pocket_cx = (w1_lo[0] + w1_hi[0]) / 2.0
pocket_cy = (w1_lo[1] + w1_hi[1]) / 2.0

bag_bottom_target = wall_top_z - wall_depth / 2.0
insert_z = bag_bottom_target + fw.BAG_HALF_H
above_z = wall_top_z + 0.20

print(f"[slot] gap_center=({gap_cx:.4f},{gap_cy:.4f})  pocket(L1_Wall1_1)_center=({pocket_cx:.4f},{pocket_cy:.4f})")
print(f"[slot] wall_top_z={wall_top_z:.4f}  wall_depth={wall_depth*1000:.1f}mm  bag_bottom_target={bag_bottom_target:.4f}")
print(f"[target] xy=({pocket_cx:.4f},{pocket_cy:.4f})  above_z={above_z:.4f}  insert_z={insert_z:.4f}")

# ── wrist-untwisted quat (full_workflow.py 와 동일 로직) ───────────────────
q_lift = fw.Q_LIFT
robot.set_dofs_position(np.concatenate([q_lift, [fw.FING_CLOSE] * 6]))
q_insert_ref = q_lift.copy()
q_insert_ref[5] = 0.0
robot.set_dofs_position(np.concatenate([q_insert_ref, [fw.FING_CLOSE] * 6]))
q_insert_quat = fw._npy(gripper_link.get_quat()).squeeze()
robot.set_dofs_position(np.concatenate([q_lift, [fw.FING_CLOSE] * 6]))

target_above = np.array([pocket_cx, pocket_cy, above_z])
target_insert = np.array([pocket_cx, pocket_cy, insert_z])
qpos_above = fw._npy(robot.inverse_kinematics(
    link=gripper_link, pos=target_above, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
qpos_insert = fw._npy(robot.inverse_kinematics(
    link=gripper_link, pos=target_insert, quat=q_insert_quat, dofs_idx_local=np.arange(6)))[:6]
print(f"[ik] above arm_q={qpos_above}")
print(f"[ik] insert arm_q={qpos_insert}")

# ── 손가락 제외 링크가 Crusher 와 접촉/침투하는지 rigid contact 로 확인 ─────
def check_pose(name, q):
    robot.set_dofs_position(np.concatenate([q, [fw.FING_CLOSE] * 6]))
    scene.step()
    contacts = robot.get_contacts(with_entity=crusher)
    n = len(contacts["link_a"]) if contacts.get("link_a") is not None else 0
    print(f"\n[check] {name}: qpos={q}")
    if n == 0:
        print(f"  접촉 없음 (0 contacts)")
        return
    bad = []
    for i in range(n):
        la = int(contacts["link_a"][i]); lb = int(contacts["link_b"][i])
        link_names = []
        for li in (la, lb):
            link = scene.rigid_solver.links[li]
            link_names.append(link.name)
        # 로봇 쪽 링크명이 FINGER_LINKS 에 없으면 "의도치 않은 접촉" 후보
        robot_side_names = [nm for nm in link_names if nm not in ("base_link",) and
                             any(nm == fl for fl in fw.FINGER_LINKS) is False]
        print(f"  contact[{i}] {link_names[0]} <-> {link_names[1]}  pos={fw._npy(contacts['position'][i])}")
        if not any(nm in fw.FINGER_LINKS for nm in link_names):
            bad.append(link_names)
    if bad:
        print(f"  [WARN] 손가락이 아닌 링크가 Crusher 와 접촉: {bad}")
    else:
        print(f"  모든 접촉이 손가락(FINGER_LINKS) 관련 — 의도된 접촉으로 보임")

check_pose("above", qpos_above)
check_pose("insert", qpos_insert)

# 참고용: insert 자세에서 팔 각 링크의 world z (포켓보다 낮게 처박히는 링크 확인)
print("\n[insert pose] 팔 링크별 world pos (finger 제외 z<0.2 인 것만):")
robot.set_dofs_position(np.concatenate([qpos_insert, [fw.FING_CLOSE] * 6]))
for link in robot.links:
    if link.name in fw.FINGER_LINKS:
        continue
    p = fw._npy(link.get_pos()).squeeze()
    if p[2] < 0.25:
        print(f"  {link.name:<24s} pos=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})")
