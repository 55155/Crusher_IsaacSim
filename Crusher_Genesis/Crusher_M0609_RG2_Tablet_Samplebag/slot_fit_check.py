"""
slot_fit_check.py — 로봇팔을 완전히 빼고, 봉투(+정제)가 Crusher 슬롯(Wall3~
Left_Wall gap)을 실제로 통과할 수 있는 두께/드랍 위치인지만 격리 검증한다.

배경(사용자 지시, 2026-07-16): full_workflow.py 의 above/insert 결과는 로봇
IK 오차·핑거-Crusher 충돌(핑거 폭 > gap 12mm)이 섞여 있어 "봉투 자체가
물리적으로 gap 을 통과할 수 있는 두께인가"를 순수하게 확인하기 어려웠다.
이 스크립트는 로봇 없이 봉투 입구(mouth) 근방 정점을 가상 캐리어(vertex
constraint, Crusher_Samplebag.py 의 PBD carrier 와 동일한 발상)로 직접
내려서, 정제를 담은 봉투가 gap 을 실제로 통과해 포켓 안까지 들어갈 수
있는지만 확인한다.

시퀀스: settle(봉투 형상 고정 중 정제 낙하) -> release(형상 고정 해제) ->
carrier descend(입구 정점을 gap 통과 목표까지 서서히 하강) -> clamp(Left_Wall
닫기) 확인.
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_workflow as fw

import genesis as gs

# Y_OFFSET_MM(2026-07-23, 사용자 지시): gap_cy(추정 위치) 근방에서 봉투를 world
# Y 축으로 조금씩 옮겨가며 gap 통과 여부를 스캔한다 — 슬롯 gap 의 Y 방향 여유가
# 봉투 폭(64mm) 대비 겨우 0.5mm/쪽(§full_workflow.py BAG_EULER 주석)이라, 추정
# 위치를 그대로 써도 통과가 잘 안 되는 게 misalignment 때문인지 확인하는 용도.
Y_OFFSET_MM = float(os.environ.get("Y_OFFSET_MM", "0"))
Y_OFFSET = Y_OFFSET_MM * 1e-3

OUT_DIR = fw.OUT_DIR
_TS = fw._TS
MP4 = os.path.join(OUT_DIR, f"slot_fit_check_yoff{Y_OFFSET_MM:+.1f}mm_{_TS}.mp4")

gs.init(backend=gs.gpu, logging_level="warning", precision="32")
fw.patch_fem_vertex_constraints()

crusher_xml = fw._prepare_crusher_mjcf()
bag_obj, bag_seal_tex = fw._prepare_seal_colored_bag()

scene = gs.Scene(
    sim_options=gs.options.SimOptions(dt=fw.DT, gravity=(0, 0, -9.81)),
    coupler_options=gs.options.IPCCouplerOptions(
        contact_d_hat=fw.IPC_D_HAT,
        contact_friction_enable=True,
        two_way_coupling=True,
        enable_rigid_rigid_contact=False,
        enable_rigid_ground_contact=False,
        constraint_strength_translation=100.0,
        constraint_strength_rotation=100.0,
    ),
    fem_options=gs.options.FEMOptions(damping=fw.FEM_DAMPING),
    vis_options=gs.options.VisOptions(
        background_color=(0.93, 0.94, 0.96),
        ambient_light=(0.16, 0.16, 0.18),
        lights=[
            {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
            {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.2},
        ],
    ),
    show_viewer=False,
)

scene.add_entity(gs.morphs.Plane(visualization=False), material=gs.materials.Rigid(coup_type="ipc_only"))
crusher = scene.add_entity(
    gs.morphs.MJCF(file=crusher_xml, pos=fw.CRUSHER_POS, euler=fw.CRUSHER_EULER, decimate=True, convexify=True),
    material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    surface=gs.surfaces.Default(smooth=False),
)

# ── 슬롯 gap 위치 (씬 build 없이도 계산 가능한 순수 mesh 기하) ────────────────
wb_lo, wb_hi = fw.crusher_mesh_world_aabb(fw.WALL_BACK_MESH)
wl_lo, wl_hi = fw.crusher_mesh_world_aabb(fw.WALL_LEFT_MESH, fw.LEFTWALL_BODY_POS, fw.LEFTWALL_GEOM_POS)
gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
gap_cx = (gap_lo_x + gap_hi_x) / 2.0
gap_width = gap_hi_x - gap_lo_x
y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
gap_cy = (y_lo + y_hi) / 2.0
wall_top_z = max(wb_hi[2], wl_hi[2])
wall_center_z = (wall_top_z + wb_lo[2]) / 2.0
print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} gap_width={gap_width*1000:.1f}mm "
      f"wall_top_z={wall_top_z:.4f} wall_center_z={wall_center_z:.4f}")
print(f"[sweep] Y_OFFSET_MM={Y_OFFSET_MM:+.1f}mm -> bag_y_target={gap_cy+Y_OFFSET:.4f} "
      f"(gap_cy={gap_cy:.4f})")

# ── 봉투: gap 바로 위, mouth(입구) 를 gap 중심(+Y_OFFSET) 위 여유높이에서 시작 ─
MOUTH_START_Z = wall_top_z + 0.30
BAG_POS_TEST = (gap_cx, gap_cy + Y_OFFSET, MOUTH_START_Z - fw.BAG_HALF_H)
BAG_MOUTH_Z_TEST = BAG_POS_TEST[2] + fw.BAG_HALF_H

SHELF_SIZE = fw.SHELF_SIZE
SHELF_TOP = BAG_POS_TEST[2] - fw.BAG_HALF_H - 0.0015
SHELF_POS = (BAG_POS_TEST[0], BAG_POS_TEST[1], SHELF_TOP - SHELF_SIZE[2] / 2)
scene.add_entity(
    gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
    material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
    surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
)

bag = scene.add_entity(
    material=gs.materials.FEM.Cloth(
        E=fw.CLOTH_E, nu=fw.CLOTH_NU, rho=fw.CLOTH_RHO,
        thickness=fw.CLOTH_THICK, bending_stiffness=fw.CLOTH_BEND,
        friction_mu=fw.CLOTH_FRICTION,
    ),
    morph=gs.morphs.Mesh(file=bag_obj, scale=fw.BAG_SCALE, pos=BAG_POS_TEST, euler=fw.BAG_EULER),
    surface=gs.surfaces.Default(opacity=0.7, roughness=0.9, double_sided=True,
                                 diffuse_texture=gs.textures.ImageTexture(image_array=bag_seal_tex)),
)

cap_verts_mm, cap_elems = fw.make_capsule_tets_v2(
    radius_mm=fw.CAP_RADIUS_MM, cyl_height_mm=fw.CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
)
TABLET_POS_TEST = (BAG_POS_TEST[0], BAG_POS_TEST[1], BAG_MOUTH_Z_TEST + fw.TABLET_DROP_H)
tablet = fw.add_analytic_fem_entity(
    scene, key=os.path.join(OUT_DIR, "_analytic_capsule_v2.stl"),
    verts_mm=cap_verts_mm, elems=cap_elems,
    material=gs.materials.FEM.Elastic(
        E=fw.TABLET_E, nu=fw.TABLET_NU, rho=fw.TABLET_RHO,
        friction_mu=fw.TABLET_FRICTION, model="stable_neohookean",
    ),
    scale=1e-3, pos=TABLET_POS_TEST,
    surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
)

cam_side = scene.add_camera(res=(960, 720), pos=(gap_cx + 0.25, gap_cy - 0.25, wall_top_z + 0.15),
                             lookat=(gap_cx, gap_cy, wall_center_z), fov=45, GUI=False)
cam_over = scene.add_camera(res=(1280, 960), pos=fw.OVERVIEW_CAM_POS, lookat=fw.OVERVIEW_CAM_LOOK,
                             fov=48, GUI=False)

print("\n[build] scene.build() 시작...")
scene.build(n_envs=0)
print("[build] 성공")

crusher_joints = {j.name: j for j in crusher.joints if j.name}
def _scalar_dof(name):
    d = crusher_joints[name].dofs_idx_local
    return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d
crank_dof = _scalar_dof(fw.CRANK_JOINT)
wall_dof = _scalar_dof(fw.WALL_JOINT)
crusher.set_dofs_kp(np.array([fw.CRANK_KP]), dofs_idx_local=[crank_dof])
crusher.set_dofs_kv(np.array([fw.CRANK_KV]), dofs_idx_local=[crank_dof])
crusher.set_dofs_kp(np.array([fw.WALL_KP]), dofs_idx_local=[wall_dof])
crusher.set_dofs_kv(np.array([fw.WALL_KV]), dofs_idx_local=[wall_dof])
crusher.set_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
crusher.set_dofs_position(np.array([fw.WALL_OFFSET]), dofs_idx_local=[wall_dof])

def _npy(x):
    return fw._npy(x)

# ── 봉투 형상 고정(바닥+양측면, 입구는 자유) — full_workflow.py 와 동일 절차 ──
# BAG_EULER=(90,0,90)(2026-07-16 9차 수정)에서는 폭(측면 고정 대상) 축이
# world Y — by 기준으로 side_mask 를 잡는다(높이=Z 는 불변).
bag_pos0 = _npy(bag.get_state().pos).squeeze()
bx, by, bz = bag_pos0[:, 0], bag_pos0[:, 1], bag_pos0[:, 2]
bag_bottom_mask = bz < bz.min() + 0.012
bag_side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
bag_fixed_idx = np.where(bag_bottom_mask | bag_side_mask)[0]
bag.set_vertex_constraints(verts_idx_local=bag_fixed_idx.tolist(), is_soft_constraint=False)
print(f"[bag] shape 고정: {len(bag_fixed_idx)}/{len(bz)} 정점(바닥+양측면)")

# ── 입구(mouth) 밴드 = 캐리어로 쓸 정점(로봇 대신 직접 내림) ────────────────
# 버그 수정: BAG_EULER=(90,0,0) 이라 봉투 높이(local Y)는 world **Z** 로 매핑된다
# (§bag_bottom_mask 도 bz 기준). world Y(by)로 걸렀더니 두께축(6mm)이라 771개
# 전부(봉투 전체) 걸려서 캐리어가 아무것도 못 끌어내렸다.
mouth_mask = bz > bz.max() - 0.010
mouth_idx = np.where(mouth_mask)[0]
mouth_pos0 = bag_pos0[mouth_idx].copy()
print(f"[bag] mouth carrier verts: {len(mouth_idx)}")

cam_side.start_recording()
cam_over.start_recording()

def render():
    cam_side.render()
    cam_over.render()

def hold_crusher():
    crusher.control_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
    crusher.control_dofs_position(np.array([fw.WALL_OFFSET]), dofs_idx_local=[wall_dof])

N_SETTLE_TABLET = 400   # 정제 낙하 + 안정화 (봉투 형상 고정 유지)
N_DESCEND = 800          # mouth 캐리어 하강 (느리게, gap 통과 관찰)
N_HOLD = 200
N_CLAMP = 400

print(f"\n[phase] settle ({N_SETTLE_TABLET*fw.DT:.1f}s) — 정제 낙하, 봉투 형상 고정 유지")
for k in range(N_SETTLE_TABLET):
    hold_crusher()
    scene.step()
    render()
tp = _npy(tablet.get_state().pos).squeeze()
print(f"[phase] settle   @done  tablet_z={tp[:,2].mean()*1e3:+.2f}mm")

# **버그 발견 1(2026-07-23, Y 오프셋 스윕 검증 중)**: `remove_vertex_constraints()`를
# 인자 없이 부르면 Genesis 내부적으로 모든 정점의 `is_constrained` 플래그를 한꺼번에
# False 로 리셋한다(`fem_solver.py`의 `remove_vertex_constraints`/`is_constrained.fill`
# 확인) — 그 뒤 `update_constraint_targets()`는 `target_pos` 값만 바꿀 뿐 `is_constrained`
# 를 다시 켜주지 않으므로(`_kernel_update_constraint_targets` 확인), mouth carrier가
# **전혀 적용되지 않고 봉투가 그 자리에 멈춰 선다**(실측: descend 4초 내내 bag_bottom_z/
# bag_top_z 완전 고정, mouth_z 목표는 계속 내려갔는데도 무반응). 바닥+측면만 선택 해제하고,
# mouth(캐리어로 쓸 정점)는 현재 위치를 target 으로 새로 등록해야 이후
# update_constraint_targets 가 실제로 먹는다.
#
# **버그 발견 2(2026-07-23, 사용자 지적)**: 위 수정 후에도 `is_soft_constraint=False`
# (hard constraint)로 두면 Genesis 가 매 스텝 mouth 정점 위치를 물리 계산과 무관하게
# 강제로 덮어써버린다(`apply_hard_constraints`, 탄성/충돌 솔버 결과를 override) —
# 벽을 그냥 통과하고, 나머지 봉투 몸통도 실제 탄성 저항 없이 늘어나기만 하는 비정상
# 거동(실측: 4초 만에 90mm→330mm, 3.67배 "늘어남" — 찢어지거나 반발력이 커지는 정상
# 탄성 거동이 아님)이 나왔고, 그 결과 Y 오프셋 ±2mm 를 스윕해도 9개 전부 비트단위로
# 동일한 결과가 나와 이 테스트가 Y 위치에 전혀 민감하지 않았다. `is_soft_constraint=True`
# (스프링, `apply_soft_constraints`)로 바꾸면 목표를 향해 당기는 힘일 뿐 위치를 직접
# 덮어쓰지 않으므로 탄성 저항·벽 충돌 반응이 물리적으로 살아난다.
# MOUTH_STIFFNESS: 질량 정규화된 임계감쇠 스프링(`apply_soft_constraints` 참고,
# spring_force=-k*x, damping=2*sqrt(k)*v — m=1 가정)의 k=omega^2. omega=1/0.05s^-1
# 근방(응답 시간 상수 ~0.05s, settle ~0.2s)이 되도록 400 선택 — descend 총 4s 대비
# 충분히 빠르게 목표를 따라가면서도 explicit 솔버(dt=5ms) 안정성 여유를 둔 값.
MOUTH_STIFFNESS = float(os.environ.get("MOUTH_STIFFNESS", "400.0"))
bag.remove_vertex_constraints(verts_idx_local=bag_fixed_idx.tolist())
bag.set_vertex_constraints(verts_idx_local=mouth_idx.tolist(), target_poss=mouth_pos0,
                           is_soft_constraint=True, stiffness=MOUTH_STIFFNESS)
print(f"[bag] 바닥/측면 고정 해제, 입구(mouth) constraint 를 soft carrier(k={MOUTH_STIFFNESS:.0f})로 전환")

DESCEND_TARGET_Z = wall_center_z  # 완전히 포켓 중앙까지 내려가는지 확인(로봇 제약 없음)
print(f"\n[phase] descend ({N_DESCEND*fw.DT:.1f}s) — mouth carrier {MOUTH_START_Z:.4f} -> "
      f"{DESCEND_TARGET_Z + (mouth_pos0[:,2].mean()-BAG_MOUTH_Z_TEST):.4f}(pocket 중앙 수준)")
nominal_width_y = float(by.max() - by.min())  # 스윕 판정용: 초기(변형 전) 봉투 폭(world Y)
_bottom_hist = []  # (k, bag_bottom_z) — 마지막 구간 진행량으로 걸림(stuck) 판정
for k in range(N_DESCEND):
    s = (k + 1) / N_DESCEND
    delta_z = (DESCEND_TARGET_Z - MOUTH_START_Z) * s
    target = mouth_pos0.copy()
    target[:, 2] += delta_z
    bag.update_constraint_targets(verts_idx_local=mouth_idx.tolist(), target_poss=target)
    hold_crusher()
    scene.step()
    render()
    if k % 100 == 0:
        bp = _npy(bag.get_state().pos).squeeze()
        tp = _npy(tablet.get_state().pos).squeeze()
        _bottom_hist.append((k, float(bp[:, 2].min())))
        bag_height_now = float(bp[:, 2].max() - bp[:, 2].min())
        print(f"    [descend k={k:4d}] mouth_z={target[:,2].mean():.4f} "
              f"bag_bottom_z={bp[:,2].min():.4f} bag_height={bag_height_now*1e3:.1f}mm(nominal 90mm) "
              f"tablet_z={tp[:,2].mean()*1e3:+.2f}mm")
bp = _npy(bag.get_state().pos).squeeze()
tp = _npy(tablet.get_state().pos).squeeze()
_bottom_hist.append((N_DESCEND, float(bp[:, 2].min())))
descend_final_width_y = float(bp[:, 1].max() - bp[:, 1].min())
descend_final_height_z = float(bp[:, 2].max() - bp[:, 2].min())
print(f"[phase] descend  @done  bag_bottom_z={bp[:,2].min():.4f}  bag_top_z={bp[:,2].max():.4f}  "
      f"bag_height={descend_final_height_z*1e3:.1f}mm(nominal 90mm)  tablet_z={tp[:,2].mean()*1e3:+.2f}mm")

print(f"\n[phase] hold ({N_HOLD*fw.DT:.1f}s)")
for k in range(N_HOLD):
    hold_crusher()
    scene.step()
    render()

print(f"\n[phase] clamp ({N_CLAMP*fw.DT:.1f}s) — Left_Wall {fw.WALL_OFFSET*1000:+.1f}mm -> "
      f"{fw.CLAMP_TARGET*1000:+.1f}mm")
for k in range(N_CLAMP):
    s = (k + 1) / N_CLAMP
    wq = fw.WALL_OFFSET + (fw.CLAMP_TARGET - fw.WALL_OFFSET) * s
    crusher.control_dofs_position(np.array([wq]), dofs_idx_local=[wall_dof])
    crusher.control_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
    scene.step()
    render()
wq_final = _npy(crusher.get_dofs_position())[wall_dof]
bp = _npy(bag.get_state().pos).squeeze()
clamp_bottom_z = float(bp[:, 2].min())
print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm  bag_bottom_z={clamp_bottom_z:.4f} "
      f"bag_top_z={bp[:,2].max():.4f}")

# ── PASS/FAIL 판정(사용자 지시, 2026-07-23): descend 종료 시 봉투 하단이
# wall_center_z 근처(±15mm)까지 도달했고, 마지막 20% 구간에서 진행이 멈추지
# 않았으며(걸림 없음), 폭(world Y)이 초기값 대비 크게 줄지 않았고(구겨짐/끼임 없음),
# 높이(world Z, bag_top-bag_bottom)가 원래 치수(90mm) 대비 비정상적으로 늘어나지
# 않았는지(soft carrier 로 바꾸기 전 hard constraint 버그 때 90mm->330mm 로 늘어났던
# 것과 같은 비물리적 신축 재발 감지용)를 본다 — 네 조건 모두 만족해야 PASS.
REACH_TOL = 0.015
STUCK_TOL = 0.003
WIDTH_TOL_FRAC = 0.85
ELONGATION_TOL_FRAC = 1.5  # bag_height_final < 90mm*1.5 = 135mm 까지만 정상으로 인정

descend_bottom_final = _bottom_hist[-1][1]
reached = abs(descend_bottom_final - DESCEND_TARGET_Z) < REACH_TOL
k_late, z_late = _bottom_hist[max(0, len(_bottom_hist) - 1 - max(1, len(_bottom_hist) // 5))]
late_progress = abs(descend_bottom_final - z_late)
not_stuck = reached or late_progress > STUCK_TOL
width_ok = descend_final_width_y > nominal_width_y * WIDTH_TOL_FRAC
nominal_height_z = 2 * fw.BAG_HALF_H
elongation_ok = descend_final_height_z < nominal_height_z * ELONGATION_TOL_FRAC

verdict = "PASS" if (reached and not_stuck and width_ok and elongation_ok) else "FAIL"
reasons = []
if not reached:
    reasons.append(f"미도달(bag_bottom_z={descend_bottom_final:.4f} vs target={DESCEND_TARGET_Z:.4f}, "
                    f"diff={abs(descend_bottom_final-DESCEND_TARGET_Z)*1000:.1f}mm>{REACH_TOL*1000:.0f}mm)")
if not not_stuck:
    reasons.append(f"걸림 의심(마지막 20% 구간 진행 {late_progress*1000:.2f}mm<{STUCK_TOL*1000:.1f}mm)")
if not width_ok:
    reasons.append(f"폭 붕괴(현재 {descend_final_width_y*1000:.1f}mm < 초기 {nominal_width_y*1000:.1f}mm의 "
                    f"{WIDTH_TOL_FRAC*100:.0f}%, 구겨짐/끼임 의심)")
if not elongation_ok:
    reasons.append(f"비정상 신축(높이 {descend_final_height_z*1000:.1f}mm > 원래 {nominal_height_z*1000:.1f}mm의 "
                    f"{ELONGATION_TOL_FRAC*100:.0f}%, hard constraint 버그 재발 의심)")
reason_str = "; ".join(reasons) if reasons else "gap 통과 확인, 걸림/붕괴/비정상 신축 없음"
print(f"\n[RESULT] Y_OFFSET_MM={Y_OFFSET_MM:+.1f}mm  verdict={verdict}  ({reason_str})")
print(f"[RESULT] descend_bottom_z={descend_bottom_final:.4f}  target={DESCEND_TARGET_Z:.4f}  "
      f"width_y={descend_final_width_y*1000:.1f}mm(nominal {nominal_width_y*1000:.1f}mm)  "
      f"height_z={descend_final_height_z*1000:.1f}mm(nominal {nominal_height_z*1000:.1f}mm)  "
      f"clamp_bottom_z={clamp_bottom_z:.4f}")

cam_side.stop_recording(save_to_filename=MP4.replace(".mp4", "_side.mp4"), fps=30)
cam_over.stop_recording(save_to_filename=MP4.replace(".mp4", "_over.mp4"), fps=30)
print(f"\n[saved] {MP4.replace('.mp4', '_side.mp4')}")
print(f"[saved] {MP4.replace('.mp4', '_over.mp4')}")
print("완료.")
