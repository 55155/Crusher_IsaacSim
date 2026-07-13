"""
fem_tablet_drop.py — 알약(정제)을 FEM.Elastic 재질로 불러와 중력 낙하시키는
최소 드롭 테스트. **미해결 엔진 버그로 인해 결과가 물리적으로 정확하지
않다 — 아래 내용 필독.**

**엔진 한계 발견(격리 테스트로 재현 확인, 2026-07-13)**:
  1. implicit FEM 솔버(`use_implicit_solver=True`)는 완전 정지(v=0)에서
     시작하면 `sim_options`/`fem_options` 양쪽에 중력을 제대로 줘도(실측:
     `fem_solver.get_gravity()` 로 값 확인됨) 중력을 전혀 반영하지 않고
     그대로 얼어붙는다(수십 스텝 동안 위치 변화 정확히 0).
  2. **회피 시도** — 초기 하향 속도를 한 번 주는(`set_velocity` 킥) 우회:
     - 구(Sphere) primitive: **-1mm/s 킥으로 정상 자유낙하** (검증됨).
     - 이 정제 STL(tetgen 산출, 납작한 biconvex): -1mm/s, -50mm/s 킥은
       아무 효과 없음(계속 얼어붙음). **-1m/s** 킥을 줘야 겨우 움직이는데,
       그마저도 첫 10ms 는 급격히 찌그러졌다가(min_z가 -3.5mm 까지 내려감)
       그 뒤로는 **중력과 반대로 계속 위로 올라간다**(t=60ms 에 +17mm) —
       순수 자유낙하가 전혀 아니고 solver 자체가 불안정/비물리적이다.
     즉 "적절한 킥 크기"가 형상/메시에 따라 달라지고, 통하는 킥조차 물리적
     정확성을 보장 못 한다 — 근본 해결이 아니라 임시방편.
  3. 씬에 Rigid 엔티티가 하나라도 있으면(SAPCouplerOptions,
     LegacyCouplerOptions(rigid_fem=False) 둘 다 시도) 위 킥을 줘도 다시
     완전히 얼어붙는다 — rigid solver 존재 자체가 FEM 중력 적분을 막는
     것으로 보이는 별도 버그.
  → 바닥판(Rigid) 은 뺐다(넣으면 100% 얼어붙음이 재현되므로). 이 스크립트는
    "FEM 재질로 정제 메시 로드는 된다"까지만 확인하는 진단용이며, 실제
    낙하 물리는 **신뢰하면 안 된다**. Genesis 쪽 버그 리포트/후속 조사 필요.

재료값(이 프로젝트 표준, DigitalTwin.md §7-8 / fem_uniaxial_compression.py):
  E=2GPa, ν=0.25, ρ=1300 kg/m³, model=linear_corotated.

출력: Sim_result/fem_tablet_drop_<ts>.mp4
"""
import os, sys
from datetime import datetime
import numpy as np
import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

DT, SUBSTEPS = 5e-4, 1
DURATION = 0.06   # 바닥 없는 자유낙하라 짧게 — 60ms 면 자유낙하 거리 ≈17.6mm
N_STEPS = int(DURATION / DT)
RENDER_EVERY = 2

E_TABLET, NU_TABLET, RHO_TABLET = 2.0e9, 0.25, 1300.0

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TABLET_STL = os.path.join(_REPO_ROOT, "tablets_stl", "stl", "tablet_R4.0_AR1.00_CV0.20.stl")
TABLET_SCALE = 1e-3   # mm → m

DROP_GAP = 3.0e-3   # 정제 바닥 ~ 바닥판 사이 초기 간극 (3mm 낙하)
PLATE_SIZE = (0.03, 0.03, 0.004)

_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(_DIR, "..", "Sim_result"); os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"fem_tablet_drop_{_TS}.mp4")


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" FEM Tablet Drop Test (viewer={use_viewer})")
    print("=" * 60)
    print(f"  E={E_TABLET/1e9:.1f} GPa  ν={NU_TABLET}  ρ={RHO_TABLET} kg/m³")
    print(f"  STL = {os.path.basename(TABLET_STL)}  drop_gap={DROP_GAP*1e3:.1f}mm")

    # fem_uniaxial_compression.py 와 동일: 두께(Y)를 낙하축 Z 로 회전.
    raw_mesh = tm.load(TABLET_STL)
    raw_mesh.apply_transform(tm.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    STL_TMP = os.path.join(OUT_DIR, "_tablet_drop_src.stl")
    raw_mesh.export(STL_TMP)

    bb = raw_mesh.bounding_box.bounds * TABLET_SCALE
    tab_lo, tab_hi = bb[0], bb[1]
    tab_h = float(tab_hi[2] - tab_lo[2])
    print(f"[stl] bbox x=[{tab_lo[0]*1e3:.2f},{tab_hi[0]*1e3:.2f}] "
          f"y=[{tab_lo[1]*1e3:.2f},{tab_hi[1]*1e3:.2f}] z=[{tab_lo[2]*1e3:.2f},{tab_hi[2]*1e3:.2f}] mm")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="64")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        fem_options=gs.options.FEMOptions(use_implicit_solver=True, gravity=(0, 0, -9.81)),
        vis_options=gs.options.VisOptions(background_color=(0.95, 0.95, 0.97)),
        show_viewer=use_viewer,
    )

    # 바닥판(Rigid) 은 넣지 않는다 — 헤더 docstring 참고: 씬에 Rigid 엔티티가
    # 있으면(coupler 유무/종류 무관) implicit FEM 이 중력을 적분하지 않는
    # 재현 가능한 버그를 만난다. 시각 참고용 높이 표시선만 plate_top_z=0 로 유지.
    plate_top_z = 0.0

    # 정제: bbox 바닥이 plate_top_z + DROP_GAP 에 오도록 배치.
    tablet_pos = (
        -0.5 * (tab_lo[0] + tab_hi[0]),
        -0.5 * (tab_lo[1] + tab_hi[1]),
        plate_top_z + DROP_GAP - tab_lo[2],
    )
    tablet = scene.add_entity(
        material=gs.materials.FEM.Elastic(
            E=E_TABLET, nu=NU_TABLET, rho=RHO_TABLET, model="linear_corotated",
            friction_mu=0.5,
        ),
        morph=gs.morphs.Mesh(file=STL_TMP, scale=TABLET_SCALE, pos=tablet_pos),
    )

    z_eye = plate_top_z + tab_h
    cam_pos = (0.02, -0.06, z_eye + 0.01)
    cam = scene.add_camera(res=(960, 720), pos=cam_pos, lookat=(0, 0, z_eye), fov=30, GUI=False)

    scene.build(n_envs=0)

    pos0 = _npy(tablet.get_state().pos).squeeze()
    print(f"[tablet] nodes={len(pos0)}  z0_mean={pos0[:,2].mean()*1e3:.2f}mm "
          f"z0_min={pos0[:,2].min()*1e3:.2f}mm")

    # 실측 확인된 엔진 특이사항: implicit FEM 솔버는 완전 정지(v=0)에서 시작하면
    # 중력을 반영 안 하고 그대로 얼어붙는다. 구(Sphere) primitive 는 아주 작은
    # 킥(-1mm/s)으로도 풀렸지만, 이 정제 메시(tetgen 산출, 납작한 biconvex
    # 형상)는 -1mm/s, -50mm/s 모두 안 통하고 **-1m/s 는 통함** — 임계값이
    # 메시/형상에 따라 달라지는 것으로 보이는 별도 엔진 특이사항(§docstring
    # 참고, 후속 조사 필요). 일단 확실히 통하는 값을 킥으로 사용.
    n_verts = pos0.shape[0]
    kick = np.zeros((n_verts, 3))
    kick[:, 2] = -1.0
    tablet.set_velocity(kick)

    cam.start_recording()
    for k in range(N_STEPS):
        scene.step()
        if (k + 1) % RENDER_EVERY == 0:
            cam.render()
        if (k + 1) % 10 == 0:
            p = _npy(tablet.get_state().pos).squeeze()
            zc, zmin = p[:, 2].mean(), p[:, 2].min()
            print(f"  t={(k+1)*DT*1e3:5.1f}ms  com_z={zc*1e3:+.3f}mm  min_z={zmin*1e3:+.3f}mm")

    pf = _npy(tablet.get_state().pos).squeeze()
    fall_mm = (pos0[:, 2].mean() - pf[:, 2].mean()) * 1e3
    print(f"\n[final] com_z={pf[:,2].mean()*1e3:.3f}mm  net_fall={fall_mm:.3f}mm")
    print("[check] 위 헤더 docstring 참고 — 이 결과는 물리적으로 신뢰할 수 없음 "
          "(엔진 버그로 인한 킥-후-비물리적 상승 거동 재현됨). FEM 재질/메시 로드 "
          "자체는 정상 동작 확인.")

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved video] {MP4}")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
