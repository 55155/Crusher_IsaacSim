"""
interactive_probe.py — 봉투 크기의 프로브 박스를 라이브 뷰어에서 마우스로
드래그해 Crusher 슬롯에 넣어보는 인터랙티브 도구.

2026-07-16 재작성: 공식 예제(`genesis-world/examples/viewer_plugin/
mouse_interaction.py`)를 그대로 참고해 훨씬 단순하게 만들었다. 이전 버전은
(1) GPU 백엔드 + `use_force=True`(스프링힘) 조합에서 Crusher 관절을 잘못
잡으면 NaN 발산, (2) 매 스텝 Crusher 를 PD 로 계속 붙잡고 있어서 드래그
스프링힘과 충돌 — 두 문제 다 공식 예제 패턴을 따르면 자연히 사라진다:
CPU 백엔드, `use_force=False`(스프링 대신 위치를 직접 set — 발산 불가능),
Crusher 관절은 루프 진입 전 딱 한 번만 세팅(이후 안 건드림, PD 충돌 없음).

사용법: python interactive_probe.py
    좌클릭 드래그로 주황색 프로브 박스를 슬롯(gap)에 넣어본다.
    ESC 또는 Ctrl+C 로 종료. 0.5초마다 프로브 위치를 출력한다.
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_workflow as fw

import genesis as gs
import genesis.vis.keybindings as kb

gs.init(backend=gs.cpu, logging_level="warning")

crusher_xml = fw._prepare_crusher_mjcf()

# ── 슬롯 gap 위치 (순수 mesh 기하, 빌드 불필요) ─────────────────────────────
wb_lo, wb_hi = fw.crusher_mesh_world_aabb(fw.WALL_BACK_MESH)
wl_lo, wl_hi = fw.crusher_mesh_world_aabb(fw.WALL_LEFT_MESH, fw.LEFTWALL_BODY_POS, fw.LEFTWALL_GEOM_POS)
gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
gap_cx = (gap_lo_x + gap_hi_x) / 2.0
y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
gap_cy = (y_lo + y_hi) / 2.0
wall_top_z = max(wb_hi[2], wl_hi[2])
wall_center_z = (wall_top_z + wb_lo[2]) / 2.0
print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} wall_top_z={wall_top_z:.4f} "
      f"wall_center_z={wall_center_z:.4f}")

scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(gap_cx + 0.35, gap_cy - 0.05, wall_top_z + 0.30),
        camera_lookat=(gap_cx, gap_cy, wall_center_z),
        camera_fov=45,
    ),
    vis_options=gs.options.VisOptions(
        background_color=(0.93, 0.94, 0.96),
        ambient_light=(0.25, 0.25, 0.28),
    ),
    show_viewer=True,
)

scene.add_entity(gs.morphs.Plane())
crusher = scene.add_entity(
    gs.morphs.MJCF(file=crusher_xml, pos=fw.CRUSHER_POS, euler=fw.CRUSHER_EULER, decimate=True, convexify=True),
    surface=gs.surfaces.Default(smooth=False),
)

# 프로브 박스: 봉투 실측 치수(두께6mm=X, 폭64mm=Y, 높이90mm=Z)
PROBE_SIZE = (0.006, 0.064, 0.090)
PROBE_START = (gap_cx, gap_cy, wall_top_z + 0.15)
probe = scene.add_entity(
    gs.morphs.Box(size=PROBE_SIZE, pos=PROBE_START, fixed=False),
    surface=gs.surfaces.Default(color=(0.95, 0.55, 0.35), opacity=0.85),
)

scene.viewer.add_plugin(
    gs.vis.viewer_plugins.MouseInteractionPlugin(
        use_force=False,  # 스프링힘 대신 위치를 직접 set — 발산 불가능(공식 예제 기본값)
        color=(0.95, 0.55, 0.35, 0.6),
    )
)

print("\n[build] scene.build() 시작...")
scene.build(n_envs=0)
print("[build] 성공")

crusher_joints = {j.name: j for j in crusher.joints if j.name}
def _scalar_dof(name):
    d = crusher_joints[name].dofs_idx_local
    return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d
crank_dof = _scalar_dof(fw.CRANK_JOINT)
wall_dof = _scalar_dof(fw.WALL_JOINT)
# 루프 진입 전 딱 한 번만 세팅 — 이후 매 스텝 PD 로 안 붙잡으므로 드래그와
# 충돌할 일이 없다(이전 버전의 NaN 발산 원인).
crusher.set_dofs_position(np.array([fw.CRANK_START_Q]), dofs_idx_local=[crank_dof])
crusher.set_dofs_position(np.array([fw.WALL_OFFSET]), dofs_idx_local=[wall_dof])

print("\n[ready] 좌클릭 드래그로 주황색 프로브 박스를 슬롯(gap)에 넣어보세요.")
print("        ESC 또는 Ctrl+C 로 종료. 0.5초마다 위치를 출력합니다.\n")

is_running = True
def stop():
    global is_running
    is_running = False
scene.viewer.register_keybinds(kb.Keybind("quit", kb.Key.ESCAPE, kb.KeyAction.RELEASE, callback=stop))

def _npy(x):
    return fw._npy(x)

step = 0
try:
    while is_running:
        scene.step()
        step += 1
        if step % 30 == 0:  # show_viewer 자동 FPS 제한(~60) 기준 대략 0.5s
            p = _npy(probe.get_pos()).squeeze()
            print(f"[step={step:5d}] probe pos = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})   "
                  f"(gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} wall_center_z={wall_center_z:.4f})")
except KeyboardInterrupt:
    pass
finally:
    p = _npy(probe.get_pos()).squeeze()
    print(f"\n[종료] 마지막 프로브 위치 = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})")
