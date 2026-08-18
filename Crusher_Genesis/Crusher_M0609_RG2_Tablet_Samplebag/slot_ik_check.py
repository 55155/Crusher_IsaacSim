"""slot_ik_check.py — full_workflow.py 의 above/insert IK 목표를 무거운 IPC+FEM
씬을 빌드하지 않고 저비용으로 검증한다(Rigid-only, Crusher+Robot 만, coupler 기본값).

목적(사용자 지시, 2026-07-15 4차): "Scene 전체를 빌드하기 이전에 이거 위치만
잘 조정하고 가자" — above/insert 목표에서 로봇팔(손가락 제외 링크)이 Crusher
구조물과 충돌/침투하는지 rigid contact 로 빠르게 확인.

**2026-08-14 재작성.** 예전 버전은 XY 목표를 `L1_Wall1_1`(포켓 바닥판) 중심으로
잡고 있었는데, 이건 §DigitalTwin.md §9 "5-2 포켓 바닥판 오인" 에서 도달 불가능한
목표로 판명나 §5-3 에서 이미 폐기된 안이다(로봇은 gap 근처까지만, 이후 Left_Wall
이 클램프). 목표 계산을 중복 정의하지 않고 **full_workflow 의 상수를 그대로
재사용**해 두 파일이 다시 어긋나지 않게 한다.

검증 항목:
  1. above/insert 자세에서 손가락 이외 링크가 Crusher 와 접촉하는가
  2. **봉투 몸체**(핑거가 아니라)가 gap 의 X/Y 창 안에 들어오는가 — 그리퍼가
     봉투의 세로 실링 가장자리를 물기 때문에 핑거 TCP 와 봉투 중심은 Y 로
     `fw.BAG_DY_FROM_FINGER`(+28mm) 어긋나 있다
  3. above->insert 를 조인트각 선형보간으로 내려갈 때 카테시안 경로가 옆으로
     얼마나 부푸는가(gap X 여유가 3mm/쪽뿐이라 이 활 모양이 그대로 벽에 긁힌다)

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

left_link = robot.get_link(fw.FINGER_LINKS[0])

# ── 슬롯/포켓 기하 (full_workflow.main() 과 동일 계산) ───────────────────────
wb_lo, wb_hi = fw.crusher_mesh_world_aabb(fw.WALL_BACK_MESH)
wl_lo, wl_hi = fw.crusher_mesh_world_aabb(fw.WALL_LEFT_MESH, fw.LEFTWALL_BODY_POS, fw.LEFTWALL_GEOM_POS)
gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
gap_cx = (gap_lo_x + gap_hi_x) / 2.0
gap_width = gap_hi_x - gap_lo_x
y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
gap_cy = (y_lo + y_hi) / 2.0
wall_top_z = max(wb_hi[2], wl_hi[2])
wall_center_z = (wall_top_z + wb_lo[2]) / 2.0

above_z = wall_top_z + 0.20
insert_z = wall_center_z + fw.BAG_HANG_BELOW_FINGER
target_xy = np.array([gap_cx, gap_cy - fw.BAG_DY_FROM_FINGER])

print(f"\n[slot] gap X창=[{gap_lo_x:.4f},{gap_hi_x:.4f}] ({gap_width*1e3:.1f}mm)  "
      f"Y창=[{y_lo:.4f},{y_hi:.4f}] ({(y_hi-y_lo)*1e3:.1f}mm)")
print(f"[slot] wall_top_z={wall_top_z:.4f}  wall_center_z={wall_center_z:.4f}")
print(f"[target] finger xy=({target_xy[0]:.4f},{target_xy[1]:.4f})  above_z={above_z:.4f}  "
      f"insert_z={insert_z:.4f}")
print(f"[target] 봉투 몸체 중심 -> ({target_xy[0]:.4f},{target_xy[1]+fw.BAG_DY_FROM_FINGER:.4f})  "
      f"(보정 dy={fw.BAG_DY_FROM_FINGER*1e3:+.1f}mm)")

# ── 2. 봉투 몸체가 gap 창 안에 드는가 ────────────────────────────────────────
# 봉투 world 치수: 두께(X) 6mm / 폭(Y) 64mm / 높이(Z) 90mm — BAG_EULER=(90,0,90).
BAG_HALF_T, BAG_HALF_W = 0.003, 0.032
bcx, bcy = target_xy[0], target_xy[1] + fw.BAG_DY_FROM_FINGER
mx = min(bcx - BAG_HALF_T - gap_lo_x, gap_hi_x - (bcx + BAG_HALF_T))
my = min(bcy - BAG_HALF_W - y_lo, y_hi - (bcy + BAG_HALF_W))
print(f"\n[fit] 봉투 X 스팬=[{bcx-BAG_HALF_T:.4f},{bcx+BAG_HALF_T:.4f}] -> 여유 {mx*1e3:+.2f}mm/쪽")
print(f"[fit] 봉투 Y 스팬=[{bcy-BAG_HALF_W:.4f},{bcy+BAG_HALF_W:.4f}] -> 여유 {my*1e3:+.2f}mm/쪽")
if mx < 0 or my < 0:
    print("[fit] **음수 = 그만큼 벽 윗면에 얹힌다** — 이 상태로는 천이 접힐 수밖에 없다")
else:
    print("[fit] 두 축 모두 창 안 — 봉투가 슬릿 위에 정렬됨")

# ── wrist-untwisted quat / IK ───────────────────────────────────────────────
q_lift = fw.Q_LIFT
robot.set_dofs_position(np.concatenate([q_lift, [fw.FING_CLOSE] * 6]))

def ik(z):
    return fw._npy(robot.inverse_kinematics(
        link=left_link, pos=np.array([target_xy[0], target_xy[1], z]),
        quat=fw.VERTICAL_QUAT, local_point=fw.FINGER_TCP_LOCAL,
        dofs_idx_local=np.arange(6)))[:6]

def fk_tcp(q):
    robot.set_dofs_position(np.concatenate([q, [fw.FING_CLOSE] * 6]))
    p = fw._npy(left_link.get_pos()).squeeze()
    R = _quat_R(fw._npy(left_link.get_quat()).squeeze())
    return p + R @ fw.FINGER_TCP_LOCAL

def _quat_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])

qpos_above = ik(above_z)
qpos_insert = ik(insert_z)
print(f"\n[ik] above  arm_q={np.round(qpos_above,5)}  FK오차={np.linalg.norm(fk_tcp(qpos_above)-np.array([*target_xy, above_z]))*1e3:.3f}mm")
print(f"[ik] insert arm_q={np.round(qpos_insert,5)}  FK오차={np.linalg.norm(fk_tcp(qpos_insert)-np.array([*target_xy, insert_z]))*1e3:.3f}mm")

# ── 3. 조인트각 선형보간이 카테시안 직선에서 얼마나 벗어나는가 ───────────────
# gap X 여유가 3mm/쪽뿐이라, 하강 중 옆으로 부푸는 양이 그 여유를 먹으면
# 봉투가 벽에 긁힌다. 크면 웨이포인트를 잘게 나눠 IK 로 푸는 편이 맞다.
print("\n[path] above->insert 조인트각 선형보간의 카테시안 이탈:")
worst = np.zeros(2); worst_s = 0.0
for s in np.linspace(0, 1, 41):
    q = qpos_above + (qpos_insert - qpos_above) * fw.ease(s)
    p = fk_tcp(q)
    dev = np.abs(p[:2] - target_xy)
    if dev.max() > worst.max():
        worst, worst_s = dev, s
print(f"  최대 이탈 s={worst_s:.2f} 에서 dx={worst[0]*1e3:+.2f}mm dy={worst[1]*1e3:+.2f}mm")
print(f"  (gap X 여유 {mx*1e3:.2f}mm/쪽, Y 여유 {my*1e3:.2f}mm/쪽 대비 판단)")

# ── 1. 손가락 제외 링크가 Crusher 와 접촉/침투하는지 rigid contact 로 확인 ───
def check_pose(name, q):
    robot.set_dofs_position(np.concatenate([q, [fw.FING_CLOSE] * 6]))
    scene.step()
    contacts = robot.get_contacts(with_entity=crusher)
    n = len(contacts["link_a"]) if contacts.get("link_a") is not None else 0
    print(f"\n[check] {name}: qpos={np.round(q,5)}")
    if n == 0:
        print(f"  접촉 없음 (0 contacts)")
        return
    bad = []
    for i in range(n):
        la = int(contacts["link_a"][i]); lb = int(contacts["link_b"][i])
        link_names = [scene.rigid_solver.links[li].name for li in (la, lb)]
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
print("\n[insert pose] 팔 링크별 world pos (finger 제외 z<0.25 인 것만):")
robot.set_dofs_position(np.concatenate([qpos_insert, [fw.FING_CLOSE] * 6]))
for link in robot.links:
    if link.name in fw.FINGER_LINKS:
        continue
    p = fw._npy(link.get_pos()).squeeze()
    if p[2] < 0.25:
        print(f"  {link.name:<24s} pos=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})")
