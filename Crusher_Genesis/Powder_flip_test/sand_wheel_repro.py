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
  - horizon 1000 -> N_STEPS(환경변수, 기본 900 = 시뮬 2.7초).
  - **영상 길이를 VIDEO_SEC(기본 10초)로 직접 지정한다.** Genesis 녹화는
    실시간 페이스라 그냥 두면 영상 길이 = 시뮬 시간이 된다 — 예전 영상
    (sand_wheel_repro_20260731_170221.mp4)이 300스텝을 돌고도 27프레임
    0.89초로 끝난 이유다. 자세한 계산은 아래 상수 주석 참고.

env:
  N_STEPS    시뮬 스텝수 (기본 900)
  VIDEO_SEC  원하는 영상 길이 초 (기본 10)
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

DT = 3e-3
FPS = 30
# Genesis 는 **실시간 페이스**로 녹화한다(vis/camera.py):
#     steps_per_frame = max(1, round(realtime_factor / (fps * dt)))
# 기본 realtime_factor=1.0, fps=30, dt=3e-3 이면 steps_per_frame=11 이라 300스텝을
# 돌려도 27프레임(0.89초)만 남는다 — 예전 영상이 순식간에 끝난 게 이것 때문이다.
# N_STEPS 를 늘려도 realtime_factor 가 1 이면 영상 길이 = 시뮬 시간이라 여전히 짧다.
# 그래서 원하는 영상 길이에서 realtime_factor 를 **역산**한다:
#     video_sec = N_STEPS * dt / realtime_factor
N_STEPS = int(os.environ.get("N_STEPS", "900"))       # 시뮬 2.7초 분량
VIDEO_SEC = float(os.environ.get("VIDEO_SEC", "10"))  # 원하는 영상 길이(초)
RT_FACTOR = N_STEPS * DT / VIDEO_SEC                  # 0.27 -> 약 3.3배 슬로모션


def main():
    import genesis as gs
    gs.init(backend=gs.gpu, precision="32", logging_level="info")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=10),
        mpm_options=gs.options.MPMOptions(
            lower_bound=(0.0, -1.0, -0.1),
            upper_bound=(0.57, 1.0, 2.4),
            grid_density=64,
        ),
        vis_options=gs.options.VisOptions(visualize_mpm_boundary=True),
        # 뷰어를 안 띄워도 녹화 페이스는 viewer_options.realtime_factor 를 읽는다
        # (viewer 가 None 이면 이 값으로 폴백 — vis/camera.py). 슬로모션의 정체.
        viewer_options=gs.options.ViewerOptions(realtime_factor=RT_FACTOR),
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

    spf = max(1, round(RT_FACTOR / (FPS * DT)))
    print(f"\n[build] sand_wheel_repro  N_STEPS={N_STEPS}  scene.build() 시작...")
    print(f"[video] 시뮬 {N_STEPS*DT:.2f}s -> 영상 {VIDEO_SEC:.1f}s @{FPS}fps "
          f"(realtime_factor={RT_FACTOR:.3f}, {spf} 스텝당 1프레임, "
          f"예상 {N_STEPS//spf}프레임, {1/RT_FACTOR:.1f}배 슬로모션)")
    scene.build(n_envs=0)
    print("[build] 성공")

    mp4_path = os.path.join(OUT_DIR, f"sand_wheel_repro_{_TS}.mp4")
    cam.start_recording(save_to_filename=mp4_path, fps=FPS)

    for i in range(N_STEPS):
        emitter.emit(
            pos=np.array([0.5, 0.0, 2.3]),
            direction=np.array([0.0, np.sin(i / 10) * 0.35, -1.0]),
            speed=8.0,
            droplet_shape="rectangle",
            droplet_size=[0.03, 0.05],
        )
        # cam.render() 를 직접 부르지 않는다 — 녹화 중에는 scene.step() 이
        # 페이스에 맞춰 알아서 렌더/인코딩한다(camera.py "renders itself as the
        # scene is stepped"). 직접 부르면 인코딩되지도 않는 프레임을 그리느라
        # GPU 시간만 버린다.
        scene.step()
        if (i + 1) % 100 == 0:
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
