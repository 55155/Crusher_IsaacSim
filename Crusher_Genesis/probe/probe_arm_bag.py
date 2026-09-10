"""
probe_arm_bag.py — 로봇팔이 파우더 든 봉투를 쥐고 옮길 때의 거동만 격리한다.
(사용자 지시, 2026-09-10)

`probe_bag_shake.py` 가 "파우더가 원인"까지 갈랐다(§25). 다만 거기서 봉투를 잡는
방식은 **SPC(소프트 위치구속)** 였다 — 입구 정점을 목표로 끌어당기는 것이라,
실제 공정의 **마찰 파지**와 경계조건이 다르다. 마찰 파지는 미끄러짐/고착
(stick-slip)이 생길 수 있고 그건 그 자체로 덜컥거림의 메커니즘이다.

여기서는 **로봇 + 봉투 + 파우더만** 남긴다. 크러셔/회수장치/흡착장치/플레이트를
전부 뺐다 — full_workflow 에서 빼려면 crusher 참조 51곳을 손봐야 해서 위험하다.
상수와 로봇 MJCF 는 `import full_workflow as FW` 로 그대로 재사용한다(이 리포의
기존 방식, powder_containment_test.py 와 동일).

## 무엇을 새로 재나

핑거 링크와 봉투 COM 을 **둘 다** 매 스텝 찍는다. 그래야 갈린다:

    핑거 매끈 + 봉투 덜컥   -> 파지(마찰) 경계에서 생기는 것
    핑거도 덜컥            -> 구동/솔버에서 오는 것

지표는 |가속도| 잔차 [m/s^2] 다 — 기준 궤적이 필요 없고 주파수·dt 와 무관해
조 사이 비교가 된다(§25-1).

## env

    N_GRAINS=524           낟알 수 (0 이면 파우더 없음 = 대조군)
    GRAIN_RADIUS_MM=0.75  GRAIN_RHO=2160
    ARM_VEL=0|1            set_dofs_position 뒤에 유한차분 속도를 같이 쓸지
    DT_MS=5.0
    MOVE=down|shake        파지 뒤 동작 (기본 down)
    DOWN_MM=150  DOWN_MMPS=20             직선 하강량/속도
    SHAKE_DEG=8  SHAKE_HZ=1.0  SHAKE_S=6.0  흔들기(MOVE=shake 일 때)
    NO_VIDEO=1  TAG=...
"""
import os
import sys

import numpy as np
import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
sys.path.insert(0, os.path.join(_r, "Powder_flip_test"))
sys.path.insert(0, os.path.join(_r, "Crusher_M0609_RG2_Tablet_Samplebag"))
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))

from datetime import datetime                                       # noqa: E402
from fem_ipc_workarounds import (patch_fem_vertex_constraints,      # noqa: E402
                                 patch_ipc_vertex_attach)
import full_workflow as FW                                          # noqa: E402

OUT = os.path.join(_r, "probe", "RESULT_armbag")
os.makedirs(OUT, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")

DT = float(os.environ.get("DT_MS", "5.0")) * 1e-3
N_GRAINS = int(os.environ.get("N_GRAINS", "524"))
GR = float(os.environ.get("GRAIN_RADIUS_MM", "0.75")) * 1e-3
GRHO = float(os.environ.get("GRAIN_RHO", "2160"))
ARM_VEL = os.environ.get("ARM_VEL", "0") == "1"
# 파지 마찰. 핑거는 매끈한데 봉투만 덜컥인다면(실측 3.6~4.5배)
# 마찰 경계의 미끄러짐/고착이 유력한 후보다.
GRIP_MU = float(os.environ.get("GRIP_MU", str(FW.CLOTH_FRICTION)))
# MOVE — 파지 뒤에 무엇을 시키나.
#   down   (기본) **아래로 천천히 직선 하강**. 진동이 없는 단조 운동이라
#          덜컥거림이 있으면 바로 드러난다. 실제 insert 구간과 같은 운동이다.
#   shake  1번 관절 사인파. 진동이 섞여 판정이 흐려진다(2026-09-10 사용자 지적).
#   above  **full_workflow 의 이송 경로 그대로**. lift(Q_GRASP->Q_LIFT, N_LIFT)
#          + above(Q_LIFT->슬롯 위 IK 자세, N_ABOVE) 를 ease() 까지 똑같이 쓴다.
#          프레임당 4.5mm 가 나온 바로 그 구간이라 이게 재현 대상이다.
#          크러셔 엔티티는 없어도 된다 — slot_geometry() 가 STL+좌표만 쓴다.
MOVE = os.environ.get("MOVE", "down").lower()
assert MOVE in ("down", "shake", "above"), f"MOVE={MOVE!r}"
DOWN_MM = float(os.environ.get("DOWN_MM", "150"))
DOWN_MMPS = float(os.environ.get("DOWN_MMPS", "20"))   # 느리게가 기본
SHAKE_DEG = float(os.environ.get("SHAKE_DEG", "8"))
SHAKE_HZ = float(os.environ.get("SHAKE_HZ", "1.0"))
SHAKE_S = float(os.environ.get("SHAKE_S", "6.0"))
NO_VIDEO = os.environ.get("NO_VIDEO", "0") == "1"
TAG = os.environ.get("TAG", f"g{N_GRAINS}_v{int(ARM_VEL)}")

N_SETTLE = 100


def _npy(x):
    x = x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)
    return x[0] if x.ndim == 3 else x


def main():
    import genesis as gs
    import genesis.utils.geom as gu
    import uipc

    patch_fem_vertex_constraints()
    patch_ipc_vertex_attach(strength_rate=FW.BAG_ATTACH_K)
    from ipc_grain_coupler import _build_grain_coupler_class

    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=FW.IPC_D_HAT, contact_friction_enable=True,
            two_way_coupling=True, enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=FW.IPC_CONSTRAINT_STRENGTH,
            constraint_strength_rotation=FW.IPC_CONSTRAINT_STRENGTH,
        ),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96), ambient_light=(0.16, 0.16, 0.18),
            lights=[{"type": "directional", "dir": (-1, -1, -1),
                     "color": (1.0, 1.0, 1.0), "intensity": 6.0}]),
        show_viewer=False,
    )
    coupler = _build_grain_coupler_class()(scene._sim, scene._sim.coupler_options)
    scene._sim._coupler = coupler

    # ── 로봇 + 봉투만. 크러셔/회수장치/흡착/플레이트는 넣지 않는다 ────────
    robot = scene.add_entity(
        gs.morphs.MJCF(file=FW._prepare_robot_mjcf(), pos=tuple(FW.ROBOT_OFFSET),
                       decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint",
                                    coup_links=FW.FINGER_LINKS,
                                    coup_friction=GRIP_MU),
    )
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(E=FW.CLOTH_E, nu=FW.CLOTH_NU, rho=FW.CLOTH_RHO,
                                        thickness=FW.CLOTH_THICK,
                                        bending_stiffness=FW.CLOTH_BEND,
                                        friction_mu=GRIP_MU),
        morph=gs.morphs.Mesh(file=FW.BAG_STL, scale=FW.BAG_SCALE,
                             pos=FW.BAG_POS, euler=FW.BAG_EULER),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.85, 0.55), double_sided=True),
    )

    # ── 낟알을 봉투 안에 직접 생성(§full_workflow GRAIN_WHEN=start 와 같은 방식) ──
    grain_spec = None
    if N_GRAINS > 0:
        bm = tm.load(FW.BAG_STL).copy()
        bm.vertices = ((gu.quat_to_R(gu.xyz_to_quat(np.array(FW.BAG_EULER), rpy=True,
                                                    degrees=True))
                        @ (bm.vertices * FW.BAG_SCALE).T).T + np.array(FW.BAG_POS))
        pq = tm.proximity.ProximityQuery(bm)
        mg = FW.CLOTH_THICK + GR + FW.IPC_D_HAT + 0.0002
        rng = np.random.default_rng(0)
        cand = rng.uniform(*bm.bounds, size=(max(60000, N_GRAINS * 60), 3))
        cand = cand[np.abs(pq.signed_distance(cand)) >= mg]
        dmin = 2 * GR + 4 * FW.IPC_D_HAT
        pts = np.empty((0, 3))
        for q in cand:
            if len(pts) == 0 or np.linalg.norm(pts - q, axis=1).min() >= dmin:
                pts = np.vstack([pts, q])
            if len(pts) >= N_GRAINS:
                break
        m1 = GRHO * (4.0 / 3.0) * np.pi * GR ** 3
        grain_spec = coupler.add_grains(pts, radius=GR, mass_density=GRHO,
                                        friction_mu=0.6)
        print(f"[arm] 낟알 {len(pts)}/{N_GRAINS}알 R={GR*1e3:.2f}mm "
              f"총 {m1*len(pts)*1e3:.3f}g")
    else:
        print("[arm] 파우더 없음 (대조군)")

    cam = scene.add_camera(res=(960, 720), pos=(0.45, -0.10, 0.62),
                           lookat=tuple(np.array(FW.BAG_POS) + np.array([0, 0, 0.05])),
                           fov=42, GUI=False, debug=True)

    n_move = (int(round(SHAKE_S / DT)) if MOVE == "shake"
              else FW.N_ABOVE if MOVE == "above"
              else int(round(DOWN_MM / DOWN_MMPS / DT)))
    print(f"[arm] dt={DT*1e3:.2f}ms ARM_VEL={int(ARM_VEL)} MOVE={MOVE}  "
          f"settle {N_SETTLE} + close {FW.N_CLOSE} + grasp {FW.N_GRASP} + "
          f"{MOVE} {n_move}"
          + (f"  ({DOWN_MM:.0f}mm @ {DOWN_MMPS:.0f}mm/s)" if MOVE == "down" else ""))
    scene.build(n_envs=0)

    # ── 봉투 형상 고정(입구는 자유) — close 직전에 푼다 ────────────────────
    vp0 = _npy(bag.get_state().pos).squeeze()
    bx, bz = vp0[:, 0], vp0[:, 2]
    fixed = np.where((bz < bz.min() + 0.012)
                     | (bx < bx.min() + 0.008) | (bx > bx.max() - 0.008))[0]
    slot = scene.sim.coupler.cloth_slots[(bag, 0)]

    def _spc(idx=None, tgt=None):
        g = slot.geometry()
        ic = uipc.view(g.vertices().find(uipc.builtin.is_constrained))
        ap = uipc.view(g.vertices().find(uipc.builtin.aim_position))
        ic[:] = 0
        if idx is None:
            return
        ic[idx] = 1
        if tgt is not None:
            ap[idx] = np.asarray(tgt, dtype=np.float64).reshape(-1, 3, 1)

    _spc(fixed, vp0[fixed])
    print(f"[arm] 봉투 형상 고정 {len(fixed)}/{len(vp0)} 정점")

    fing_link = [l for l in robot.links if l.name == FW.FINGER_LINKS[0]][0]
    mp4 = os.path.join(OUT, f"armbag_{TAG}_{_TS}.mp4")
    if not NO_VIDEO:
        cam.start_recording(save_to_filename=mp4, fps=30)

    rec = []
    qprev = [None]

    def _drive(q, f):
        qn = np.concatenate([q, [f] * 6])
        robot.set_dofs_position(qn)
        if ARM_VEL and qprev[0] is not None:
            robot.set_dofs_velocity((qn - qprev[0]) / DT)
        qprev[0] = qn

    def _tick(phase):
        scene.step()
        if not NO_VIDEO:
            if grain_spec is not None:
                scene.clear_debug_objects()
                scene.draw_debug_spheres(coupler.get_grain_positions(grain_spec),
                                         radius=GR, color=(0.85, 0.75, 0.55, 1.0))
            cam.render()
        vp = _npy(bag.get_state().pos).squeeze()
        fp = _npy(fing_link.get_pos()).reshape(-1)[:3]
        ext = vp.max(axis=0) - vp.min(axis=0)
        rec.append((phase, *vp.mean(axis=0), *fp, *ext))

    for _ in range(N_SETTLE):
        _drive(FW.Q_GRASP, FW.FING_OPEN)
        _tick(0)
    _spc(None)                                  # 형상 고정 해제 — 이후 순수 마찰
    print("[arm] 형상 고정 해제, 파지 시작")
    for k in range(FW.N_CLOSE):
        s = FW.ease((k + 1) / FW.N_CLOSE)
        _drive(FW.Q_GRASP, FW.FING_OPEN + (FW.FING_CLOSE - FW.FING_OPEN) * s)
        _tick(1)
    for _ in range(FW.N_GRASP):
        _drive(FW.Q_GRASP, FW.FING_CLOSE)
        _tick(2)
    if MOVE == "shake":
        for k in range(FW.N_LIFT):
            s = FW.ease((k + 1) / FW.N_LIFT)
            _drive(FW.Q_GRASP + (FW.Q_LIFT - FW.Q_GRASP) * s, FW.FING_CLOSE)
            _tick(3)
        for k in range(n_move):
            t = (k + 1) * DT
            q = FW.Q_LIFT.copy()
            q[0] += np.radians(SHAKE_DEG) * np.sin(2 * np.pi * SHAKE_HZ * t)
            _drive(q, FW.FING_CLOSE)
            _tick(4)
    elif MOVE == "above":
        # full_workflow 의 이송을 그대로 재현한다. lift 뒤 above 자세로 조인트
        # 보간(ease) — 스텝당 이동이 하강 조의 34배라, 덜컥거림이 보이면 여기다.
        for k in range(FW.N_LIFT):
            u = FW.ease((k + 1) / FW.N_LIFT)
            _drive(FW.Q_GRASP + (FW.Q_LIFT - FW.Q_GRASP) * u, FW.FING_CLOSE)
            _tick(3)
        _sg = FW.slot_geometry()
        _tz = _sg["wall_top_z"] + 0.20
        _txy = np.array([_sg["gap_cx"] - FW.BAG_DX_FROM_FINGER,
                         _sg["gap_cy"] - FW.BAG_DY_FROM_FINGER + FW.Y_OFFSET])
        q_ab = _npy(robot.inverse_kinematics(
            link=fing_link, pos=np.array([_txy[0], _txy[1], _tz]),
            quat=FW.VERTICAL_QUAT, local_point=FW.FINGER_TCP_LOCAL,
            dofs_idx_local=np.arange(6)))[:6]
        print(f"[arm] above 목표 {np.round(np.array([_txy[0], _txy[1], _tz])*1e3, 1)}mm "
              f"({FW.N_ABOVE}스텝)")
        robot.set_dofs_position(np.concatenate([FW.Q_LIFT, [FW.FING_CLOSE] * 6]))
        for k in range(n_move):
            u = FW.ease((k + 1) / n_move)
            _drive(FW.Q_LIFT + (q_ab - FW.Q_LIFT) * u, FW.FING_CLOSE)
            _tick(4)
    else:
        # 카테시안 직선 하강. 양 끝 조인트각만 보간하면 경로가 옆으로 부푸므로
        # z 균등 웨이포인트마다 IK 를 풀어 쓴다(FW.solve_descent_waypoints).
        _tcp = _npy(fing_link.get_pos()).reshape(-1)[:3]
        z0, z1 = float(_tcp[2]), float(_tcp[2]) - DOWN_MM * 1e-3
        q_way = FW.solve_descent_waypoints(robot, fing_link, _tcp[:2], z0, z1,
                                           n_way=41)
        print(f"[arm] 하강 웨이포인트 {len(q_way)}점  z {z0*1e3:.1f} -> {z1*1e3:.1f}mm")
        for k in range(n_move):
            u = (k + 1) / n_move * (len(q_way) - 1)   # ease 없이 **등속**
            i = min(int(u), len(q_way) - 2)
            _drive(q_way[i] + (q_way[i + 1] - q_way[i]) * (u - i), FW.FING_CLOSE)
            _tick(4)

    if not NO_VIDEO:
        cam.stop_recording()
        print(f"[saved] {mp4}")

    A = np.asarray(rec)
    ph, com, fp, ext = A[:, 0], A[:, 1:4], A[:, 4:7], A[:, 7:10]
    # 부푸는 축 = 가장 얇은 축(두께). 자세가 바뀌어도 이걸로 잡힌다.
    w = ext[:, int(np.argmin(ext.mean(axis=0)))]

    def _resid(x, m):
        """접촉 없는 매끄러운 운동이면 0 에 가깝다. |2차차분|/dt^2 [m/s^2]."""
        if m.sum() < 5:
            return np.nan, np.nan
        a = np.diff(np.diff(x[m])) / DT ** 2
        return float(np.median(np.abs(a))), float(np.quantile(np.abs(a), .95))

    npz = os.path.join(OUT, f"armbag_{TAG}_{_TS}.npz")
    np.savez(npz, phase=ph, com=com, finger=fp, ext=ext, width=w, dt=DT,
             n_grains=N_GRAINS, arm_vel=int(ARM_VEL))
    print()
    print("=" * 72)
    print(f"[RESULT] 낟알={N_GRAINS} ARM_VEL={int(ARM_VEL)} dt={DT*1e3:.2f}ms mu={GRIP_MU}")
    print(f"{'구간':10s}{'핑거 |a| 중앙':>14}{'봉투 |a| 중앙':>14}{'봉투/핑거':>11}"
          f"{'폭 mm':>10}")
    for p, nm in ((3, "lift"), (4, MOVE)):
        m = ph == p
        fm, _ = _resid(fp[:, 0], m)
        cm, _ = _resid(com[:, 0], m)
        print(f"{nm:10s}{fm:14.3f}{cm:14.3f}{cm/max(fm,1e-9):11.2f}"
              f"{w[m].mean()*1e3:10.1f}")
    print(f"[RESULT] 봉투 두께축 {w.min()*1e3:.1f}~{w.max()*1e3:.1f}mm  "
          f"bbox 평균 {np.round(ext.mean(axis=0)*1e3, 1)}mm")
    _m = ph == 4
    print(f"[RESULT] {MOVE} 구간 봉투 {(com[_m][-1][2]-com[_m][0][2])*1e3:+.1f}mm, "
          f"핑거 {(fp[_m][-1][2]-fp[_m][0][2])*1e3:+.1f}mm "
          f"— 차이가 크면 파지가 미끄러진 것")
    print(f"[saved] {npz}")
    print("=" * 72)


if __name__ == "__main__":
    main()
