"""
sand_wheel_repro.py — Genesis 공식 예제 examples/coupling/sand_wheel.py 재현
(MPM.Sand + Rigid(URDF wheel/plane), rigid_mpm 커플링).

배경(사용자 지시, 2026-07-31): docs/DigitalTwin.md 조합10(mpm_pbd/fem_mpm 충돌
인식 불안정)을 근거로 Genesis 측에 문의하기 전에, "일단 Rigid와는 상호작용이
괜찮다"는 사용자 관찰을 재현해서 확인한다 — 공식 예제가 그대로 재현되면
rigid_mpm 자체는 문제가 없고, 문제는 mpm_pbd/fem_mpm처럼 재질이 둘 다
"변형체/입자"인 조합에 국한된다는 근거가 된다.

원본과의 차이:
  - gs.cpu -> gs.gpu(이 환경 기본 백엔드), show_viewer=True(GUI) -> False +
    offscreen 카메라로 mp4 녹화(headless 환경이라 GUI 창을 띄울 수 없음).
  - horizon 1000 -> N_STEPS(환경변수로 축소 가능, 기본 300 — 영상 하나 만드는
    용도라 원본 전체 1000스텝은 과함).
"""
import os
import sys

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

from datetime import datetime

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")

N_STEPS = int(os.environ.get("N_STEPS", "300"))


def main():
    import genesis as gs
    gs.init(backend=gs.gpu, precision="32", logging_level="info")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=3e-3, substeps=10),
        mpm_options=gs.options.MPMOptions(
            lower_bound=(0.0, -1.0, -0.1),
            upper_bound=(0.57, 1.0, 2.4),
            grid_density=64,
        ),
        vis_options=gs.options.VisOptions(visualize_mpm_boundary=True),
        show_viewer=False,
    )

    scene.add_entity(
        material=gs.materials.Rigid(needs_coup=True, coup_friction=0.2),
        morph=gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True),
    )
    mat_wheel = gs.materials.Rigid(needs_coup=True, coup_softness=0.0)

    wheel_positions = [
        (0.5, -0.2, 1.6),
        (0.5, 0.3, 1.2),
        (0.5, -0.3, 0.8),
        (0.5, 0.4, 0.4),
    ]
    for pos in wheel_positions:
        scene.add_entity(
            material=mat_wheel,
            morph=gs.morphs.URDF(
                file="urdf/wheel/wheel.urdf", pos=pos, euler=(0, 0, 90), scale=0.6,
                convexify=False, fixed=True,
            ),
        )

    emitter = scene.add_emitter(
        material=gs.materials.MPM.Sand(),
        max_particles=200000,
        surface=gs.surfaces.Rough(color=(1.0, 0.9, 0.6, 1.0)),
    )
    sand = emitter.entity

    cam = scene.add_camera(res=(1024, 768), pos=(4.5, 0.0, 1.42), lookat=(1.0, 0.0, 1.0), fov=30, GUI=False)

    print(f"\n[build] sand_wheel_repro  N_STEPS={N_STEPS}  scene.build() 시작...")
    scene.build(n_envs=0)
    print("[build] 성공")

    mp4_path = os.path.join(OUT_DIR, f"sand_wheel_repro_{_TS}.mp4")
    cam.start_recording(save_to_filename=mp4_path, fps=30)

    for i in range(N_STEPS):
        emitter.emit(
            pos=np.array([0.5, 0.0, 2.3]),
            direction=np.array([0.0, np.sin(i / 10) * 0.35, -1.0]),
            speed=8.0,
            droplet_shape="rectangle",
            droplet_size=[0.03, 0.05],
        )
        scene.step()
        cam.render()
        if (i + 1) % 50 == 0:
            active = sand.get_particles_active()
            active = active.cpu().numpy() if hasattr(active, "cpu") else np.asarray(active)
            active = active[0] if active.ndim == 2 else active
            pos = sand.get_particles_pos()
            pos = pos.cpu().numpy() if hasattr(pos, "cpu") else np.asarray(pos)
            pos = pos[0] if pos.ndim == 3 else pos
            sp = pos[active.astype(bool)] if active.dtype != bool else pos[active]
            print(f"[step {i+1:4d}] sand N={len(sp)}  z range=[{sp[:,2].min():.3f},{sp[:,2].max():.3f}]"
                  if len(sp) else f"[step {i+1:4d}] sand N=0")

    cam.stop_recording()
    print(f"\n[saved] {mp4_path}")
    print("\n[RESULT] rigid_mpm 커플링 재현 완료 — crash/NaN 없으면 공식 예제 그대로 재현된 것.")


if __name__ == "__main__":
    main()
