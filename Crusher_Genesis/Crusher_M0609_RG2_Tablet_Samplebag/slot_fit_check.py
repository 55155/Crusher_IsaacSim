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

OUT_DIR = fw.OUT_DIR
_TS = fw._TS
MP4 = os.path.join(OUT_DIR, f"slot_fit_check_{_TS}.mp4")

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

# ── 봉투: gap 바로 위, mouth(입구) 를 gap 중심 위 여유높이에서 시작 ─────────
MOUTH_START_Z = wall_top_z + 0.30
BAG_POS_TEST = (gap_cx, gap_cy, MOUTH_START_Z - fw.BAG_HALF_H)
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

bag.remove_vertex_constraints()
print("[bag] shape 고정 해제")

DESCEND_TARGET_Z = wall_center_z  # 완전히 포켓 중앙까지 내려가는지 확인(로봇 제약 없음)
print(f"\n[phase] descend ({N_DESCEND*fw.DT:.1f}s) — mouth carrier {MOUTH_START_Z:.4f} -> "
      f"{DESCEND_TARGET_Z + (mouth_pos0[:,2].mean()-BAG_MOUTH_Z_TEST):.4f}(pocket 중앙 수준)")
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
        print(f"    [descend k={k:4d}] mouth_z={target[:,2].mean():.4f} "
              f"bag_bottom_z={bp[:,2].min():.4f} tablet_z={tp[:,2].mean()*1e3:+.2f}mm")
bp = _npy(bag.get_state().pos).squeeze()
tp = _npy(tablet.get_state().pos).squeeze()
print(f"[phase] descend  @done  bag_bottom_z={bp[:,2].min():.4f}  bag_top_z={bp[:,2].max():.4f}  "
      f"tablet_z={tp[:,2].mean()*1e3:+.2f}mm")

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
print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm  bag_bottom_z={bp[:,2].min():.4f} "
      f"bag_top_z={bp[:,2].max():.4f}")

cam_side.stop_recording(save_to_filename=MP4.replace(".mp4", "_side.mp4"), fps=30)
cam_over.stop_recording(save_to_filename=MP4.replace(".mp4", "_over.mp4"), fps=30)
print(f"\n[saved] {MP4.replace('.mp4', '_side.mp4')}")
print(f"[saved] {MP4.replace('.mp4', '_over.mp4')}")
print("완료.")
