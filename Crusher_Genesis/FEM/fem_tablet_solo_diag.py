"""
fem_tablet_solo_diag.py — 진단 전용: 정제(FEM.Elastic) 단독을 봉투/선반 없이
Plane 하나 위로 낙하시켜, fem_tablet_drop_stiff.py 에서 관찰된 "t=0 직후
즉시 z축(장축) ~10% 압축 후 고정" 현상이 봉투 접촉과 무관하게 나타나는지
확인한다. (사용자 질문: "정제 형상이 우그러지는 건 스케일이 너무 작아서인가?")

씬에 다른 IPC 엔티티(봉투 FEM.Cloth, 선반 Rigid)가 전혀 없는 상태에서도
같은 압축이 나타나면, 원인이 봉투 접촉이 아니라 FEM 솔버 자체의 초기
설정/평형화(assembly) 단계에 있다는 뜻이다.
"""
import os, sys
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_DIR = os.path.dirname(os.path.abspath(__file__))
_r = _DIR
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths  # noqa: F401

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity

TABLET_E = float(os.environ.get("TABLET_E", "1e7"))
TABLET_NU, TABLET_RHO, TABLET_FRICTION = 0.45, 1300.0, 0.5
CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0

E_BASE, DT_BASE = 5.0e4, 5e-3
DT = DT_BASE * (E_BASE / TABLET_E) ** 0.5
N_STEPS = 200  # 낙하할 데도 없으니(플레인이 멀리 아래) 짧게 — 초기 거동만 보면 됨

OUT_DIR = os.path.join(_DIR, "Result")
os.makedirs(OUT_DIR, exist_ok=True)


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def main():
    print(f"[solo diag] E={TABLET_E:.2e}  dt={DT*1e3:.4f}ms  N_STEPS={N_STEPS}")

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=float(os.environ.get("D_HAT", "5.0e-4")),
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
        ),
        show_viewer=False,
    )

    # 플레인을 아주 멀리 아래로 둬서(1m) 200스텝 안에는 절대 접촉 불가능하게.
    scene.add_entity(gs.morphs.Plane(pos=(0, 0, -1.0)), material=gs.materials.Rigid(coup_type="ipc_only"))

    cap_verts_mm, cap_elems = make_capsule_tets_v2(
        radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
    )
    tablet = add_analytic_fem_entity(
        scene, key=os.path.join(OUT_DIR, "_analytic_capsule_solo_diag.stl"),
        verts_mm=cap_verts_mm, elems=cap_elems,
        material=gs.materials.FEM.Elastic(
            E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
            friction_mu=TABLET_FRICTION, model="stable_neohookean",
        ),
        scale=1e-3, pos=(0, 0, 0.10),
    )

    scene.build(n_envs=0)

    pos0 = _npy(tablet.get_state().pos).squeeze()
    x0 = (pos0[:, 0].max() - pos0[:, 0].min()) * 1e3
    z0 = (pos0[:, 2].max() - pos0[:, 2].min()) * 1e3
    print(f"[t=build] bbox(mm) x={x0:.4f} z={z0:.4f}  (design: x=4.0 z=5.0)  -- scene.build() 직후, step() 이전")

    for k in range(N_STEPS):
        scene.step()
        if k in (0, 1, 2, 5, 10, 20, 50, 100, 199):
            p = _npy(tablet.get_state().pos).squeeze()
            xspan = (p[:, 0].max() - p[:, 0].min()) * 1e3
            zspan = (p[:, 2].max() - p[:, 2].min()) * 1e3
            zc = p[:, 2].mean()
            print(f"  k={k:4d} t={((k+1)*DT*1e3):7.3f}ms  bbox(mm) x={xspan:.4f} z={zspan:.4f}  com_z={zc*1e3:.3f}mm")


if __name__ == "__main__":
    main()
