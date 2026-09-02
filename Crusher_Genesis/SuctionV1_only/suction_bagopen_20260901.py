"""흡착컵으로 봉투 입구를 100mm 벌린다 — FEM cloth 소프트 구속 실험 (2026-09-01).

문제
----
FEM cloth 의 정점을 `set_vertex_constraints(is_soft_constraint=False)` 로 잡으면
**하드 핀**이라 그 정점의 위치를 덮어쓴다. 당기는 것이 아니라 순간이동이라,
이웃 정점은 재료 탄성으로만 끌려온다. 봉투 막강성이 E*t = 4.0e5 x 1mm = 400 N/m
로 물러서 국부적으로만 늘어나고, 사용자 표현대로 "정말 한 점만 당겨지는 느낌"이
된다. PBD 의 attachment 는 이게 자연스럽게 되는데 FEM 쪽은 그렇지 않다.

접근
----
두 가지를 같이 바꾼다.
  1. **점이 아니라 면으로 잡는다.** 실제 흡착컵은 Ø15mm 면으로 붙는다. 컵 중심에서
     반경 R 안의 정점을 **전부** 잡는다(사용자 지시).
  2. **소프트 구속을 쓴다.** `is_soft_constraint=True` + stiffness. 스프링이라
     당기는 힘이 재료를 통해 퍼진다. 하드 핀은 그 정점만 끌고 간다.
그리고 `update_constraint_targets` 로 목표를 매 스텝 옮겨 컵을 따라가게 한다.

기하 (석션V1.xml 실측)
---------------------
    q =   0mm   컵 간격  28.77mm   닫힘. 컵 면 사이 13.8mm — 봉투를 무는 자세
    q = -50mm   컵 간격 128.77mm   열림. 정확히 +100mm 개구
    컵 반경 7.50mm (Ø15mm)
`suctionV1_only.py` 주석은 "음수=압착"이라 적혀 있으나 **기하상 반대다**(위 실측).

시퀀스
------
    1 approach  컵을 벌린 채(q=-50) 대기, 봉투를 그 사이에 둔다
    2 close     q=-50 -> 0. 컵이 봉투 양면에 닿는다
    3 attach    각 컵 중심 반경 R 안의 정점을 소프트 구속으로 잡는다
    4 open      q=0 -> -50. 목표를 컵과 함께 옮기며 입구를 100mm 벌린다

판정
----
    mouth_mm    입구 개구 폭(두 흡착점 무리의 y 간격). 100mm 에 가까울수록 좋다
    follow      개구 폭 / 컵 이동량. 1.0 이면 봉투가 컵을 그대로 따라온 것
    spread_mm   붙잡힌 정점 무리의 z 퍼짐. 한 점만 끌려가면 작게 나온다
    n_grab      실제로 잡힌 정점 수(좌/우)

사용법
------
    python suction_bagopen_20260901.py                     # 기본 1회 + 영상
    GRAB_R_MM=10 SOFT=1 STIFF=1e4 CLOTH_E=1e6 python ...   # 단일 조건
    python suction_bagopen_20260901.py --sweep             # 스윕(영상 없음)
"""
import argparse
import itertools
import os
import re
import subprocess
import sys
import time
from datetime import datetime

import numpy as np

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
import paths
from fem_ipc_workarounds import (patch_fem_vertex_constraints,
                                 patch_ipc_vertex_attach)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "RESULT", os.environ.get("OUT_SUBDIR", ""))
SUCTION_MJCF = os.path.join(paths.ROBOTS_DIR, "석션V1_description", "석션V1.xml")
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag",
                       "Samplebag_seal_pouch3_sealslab3mm.stl")

# ── 스윕 축 ────────────────────────────────────────────────────────────────
GRAB_R = float(os.environ.get("GRAB_R_MM", "7.5")) * 1e-3   # 흡착 반경(컵 Ø15mm)
SOFT = os.environ.get("SOFT", "1") == "1"                   # 소프트 구속 여부
STIFF = float(os.environ.get("STIFF", "1e4"))               # 소프트 강성
CLOTH_E = float(os.environ.get("CLOTH_E", "4.0e5"))         # 봉투 탄성
CLOTH_THICK = float(os.environ.get("CLOTH_THICK_MM", "1.0")) * 1e-3
N_OPEN = int(os.environ.get("N_OPEN", "600"))               # 개구 스텝(속도)
RUN_TAG = os.environ.get("RUN_TAG", "")
NO_VIDEO = os.environ.get("NO_VIDEO", "0") == "1"
NEUTRAL_COL = os.environ.get("NEUTRAL_COL", "0") == "1"
D_HAT = float(os.environ.get("D_HAT", "1.0e-4"))
ATTACH = os.environ.get("ATTACH", "0") == "1"
ATTACH_K = float(os.environ.get("ATTACH_K", "100.0"))  # SPC 구속 스프링 세기  # IPC 정점 부착 배선(패치 2)
NO_PIN = os.environ.get("NO_PIN", "0") == "1"   # 구속 없이 자유낙하 대조군
TRACE = int(os.environ.get("TRACE", "25"))   # N스텝마다 벽시계+봉투z (0=끔)

DT = float(os.environ.get("DT", "5e-3"))
SUCTION_POS = (0.0, 0.0, 0.30)
# 흡착컵 실측(석션V1.xml, MJCF 자체 프레임 / q=0 기준). 컵 축은 **Y** 이고 면은
# x-z 평면의 Ø15mm 원판이다. q=0 에서 두 컵 면이 y=67.5mm 에서 맞닿는다.
#     L 컵 bbox  x[-85.4,-70.4] y[37.5,67.5] z[278.4,293.4]
#     R 컵 bbox  x[-85.4,-70.4] y[67.5,97.5] z[278.4,293.4]
# 링크명은 Suction_Cup 이 아니라 L/R_E-SMLG9H-100-ES10_2_1 이다(컵은 그 body 의 geom).
CUP_FACE_X, CUP_FACE_Y, CUP_FACE_Z = -0.0779, 0.0675, 0.28594
# **빌드는 봉투를 띄운 채로 한다.** q=0(=MJCF qpos0)에서 두 컵 면이 y=67.5mm 에서
# 맞닿아 틈이 0 이므로, 그 자리에 봉투를 놓으면 IPC 초기 검사에서 관통으로 걸린다
# (실측: Intersection detected 다수). 턱을 연 뒤 set_position 으로 투입한다 —
# 투입은 build 이후라 초기 검사를 타지 않는다(recovery2_bag_clamp.py 와 같은 처방).
PARK_DZ = 0.25
# 입구는 봉투 로컬 z=+45mm 의 테두리 34정점이다 — 경계 에지(면 하나에만 속한
# 에지) 위상으로 확인했다. 5-panel 서피스 모델이라 바운딩박스로는 안 보인다.
CUP_LINKS = ("L_E-SMLG9H-100-ES10_2_1", "R_E-SMLG9H-100-ES10_2_1")
MOUTH_Z = 0.045
CUP_BELOW_RIM = float(os.environ.get("CUP_BELOW_RIM_MM", "10.0")) * 1e-3
                        # 컵 중심을 입구 테두리 아래 얼마에 둘 것인가
# **[수정 2026-09-02] 개방 행정을 50 -> 20mm 로 줄인다(사용자 지시).**
# 조 한계는 -50mm 지만 그 끝까지 열면 불안정하다. 컵 간격 = 2|q| 이므로
# q=-20mm 는 컵이 40mm 벌어진다는 뜻이다.
JAW_OPEN = -float(os.environ.get("JAW_OPEN_MM", "20.0")) * 1e-3
JAW_CLOSED = 0.0
# 조는 actuatorfrcrange=+-100N 짜리 평범한 슬라이드다. set 을 쓸 이유가 없다
# (Crusher 벽은 반력 부재, 회수장치 축은 control 폭주 때문이었고 여기엔 둘 다
# 해당 없음). control 이면 6mm 봉투에 걸려 알아서 멈춘다.
JAW_KP, JAW_KV = 2000.0, 50.0
N_CLOSE = int(os.environ.get("N_CLOSE", "300"))
N_SETTLE = int(os.environ.get("N_SETTLE", "150"))
CLOTH_NU, CLOTH_RHO, CLOTH_FRIC = 0.499, 200.0, 0.8
CLOTH_BEND = float(os.environ.get("CLOTH_BEND", "400.0"))


def main():
    _ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"_{RUN_TAG}" if RUN_TAG else ""
    os.makedirs(OUT_DIR, exist_ok=True)
    mp4 = os.path.join(OUT_DIR, f"suction_bagopen{tag}_{_ts}.mp4")

    print("=" * 72)
    print(f"[cfg] 흡착반경 {GRAB_R*1e3:.1f}mm  구속 {'소프트 k=%g' % STIFF if SOFT else '하드핀'}"
          f"  CLOTH_E {CLOTH_E:g}  t {CLOTH_THICK*1e3:.1f}mm  개구 {N_OPEN}스텝")
    print(f"[cfg] E*t = {CLOTH_E*CLOTH_THICK:.0f} N/m")
    print("=" * 72)

    def _npy_len(e):
        v = e.get_state().pos
        v = v.cpu().numpy() if hasattr(v, 'cpu') else np.asarray(v)
        return v.squeeze().shape[0]

    import genesis as gs
    patch_fem_vertex_constraints()
    if ATTACH:
        patch_ipc_vertex_attach(strength_rate=ATTACH_K)
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=D_HAT, contact_friction_enable=True,
            two_way_coupling=True, enable_rigid_rigid_contact=True,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        # **rigid_options 를 걸지 않는다(2026-09-01).** suctionV1_only.py 는
        # 컵끼리 충돌을 보려고 enable_neutral_collision 을 켰는데, 그 사이에 FEM
        # 봉투가 들어오면 첫 스텝에서 nan 이 난다(실측). full_workflow.py 는 같은
        # d_hat 으로 봉투를 잘 다루면서 rigid_options 를 아예 안 건다 — 거기에 맞춘다.
        **({"rigid_options": gs.options.RigidOptions(
            enable_self_collision=True, enable_neutral_collision=True)}
           if NEUTRAL_COL else {}),
        vis_options=gs.options.VisOptions(background_color=(0.93, 0.94, 0.96)),
        show_viewer=False,
    )
    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))
    gripper = scene.add_entity(
        gs.morphs.MJCF(file=SUCTION_MJCF, pos=SUCTION_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )
    # 봉투를 컵 사이에 세워 둔다. 컵 중심 y=53.12/81.88mm 의 중점이 67.5mm.
    # 입구가 위를 보도록 세우고(euler 90,0,0), 컵 높이에 맞춘다.
    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO, thickness=CLOTH_THICK,
            bending_stiffness=CLOTH_BEND, friction_mu=CLOTH_FRIC),
        morph=gs.morphs.Mesh(file=BAG_STL, scale=1.0,
                             pos=(SUCTION_POS[0] + CUP_FACE_X,
                                  SUCTION_POS[1] + CUP_FACE_Y,
                                  SUCTION_POS[2] + CUP_FACE_Z + PARK_DZ),
                             euler=(90, 0, 0)),
        surface=gs.surfaces.Default(opacity=0.6, double_sided=True),
    )
    _lk = (SUCTION_POS[0] + CUP_FACE_X, SUCTION_POS[1] + CUP_FACE_Y,
           SUCTION_POS[2] + CUP_FACE_Z)
    cam = scene.add_camera(res=(1280, 960),
                           pos=(_lk[0] - 0.40, _lk[1], _lk[2] + 0.05),
                           lookat=_lk, fov=48, GUI=False)
    cam2 = scene.add_camera(res=(1280, 960),
                            pos=(_lk[0], _lk[1] - 0.38, _lk[2] + 0.10),
                            lookat=_lk, fov=48, GUI=False)
    scene.build(n_envs=0)
    # **[수정 2026-09-01] 파킹 중에는 봉투를 통째로 핀으로 잡아 둔다.**
    # PARK_DZ=0.25 로 띄워 두기만 하면 봉투는 자유 FEM 이라 그대로 떨어진다.
    # 45스텝(0.226s)이면 컵 면 높이까지 내려와 2.21m/s 로 그리퍼 위에 얹히고,
    # 그리퍼 위에 얹히면 IPC 가 천-강체 접촉을 d_hat=0.1mm 로 풀어야 한다
    # (실측: open0 300스텝에서 6시간 28분 정체, CPU 100%). 투입 직전에 푼다.
    gripper.set_dofs_kp(np.array([JAW_KP] * 3), dofs_idx_local=[0, 1, 2])
    gripper.set_dofs_kv(np.array([JAW_KV] * 3), dofs_idx_local=[0, 1, 2])

    for _a in ("n_elements", "n_vertices", "n_surfaces"):
        print(f"[mesh] bag.{_a} = {getattr(bag, _a, 'N/A')}", flush=True)
    print(f"[mesh] elems shape = {getattr(getattr(bag, 'elems', None), 'shape', 'N/A')}", flush=True)
    _n_all = int(_npy_len(bag))
    bag.set_vertex_constraints(verts_idx_local=list(range(_n_all)),
                               is_soft_constraint=False)
    print(f"[park] 봉투 {_n_all}정점 전체 고정 — 투입 전까지 낙하 금지", flush=True)
    if not NO_VIDEO:
        cam.start_recording(save_to_filename=mp4, fps=30)
        cam2.start_recording(save_to_filename=mp4.replace(".mp4", "_front.mp4"), fps=30)

    def _attach(idx, tgt, reset=False):
        """uipc 슬롯에 직접 써서 정점을 끌어당긴다.

        set_vertex_constraints 는 Genesis FEM 솔버 버퍼에만 쓰는데 IPC 씬에서는
        uipc 가 천을 스텝하므로 아무도 그걸 안 읽는다(소프트=무반응, 하드=자기
        정점만 순간이동해 메시가 찢김). 여기서는 uipc 자신의 SoftPositionConstraint
        칸에 직접 써서 구속이 솔브 안으로 들어가게 한다 — 이웃이 따라온다.
        reset=False 면 이미 켜진 정점(밑단 고정)을 그대로 둔 채 추가한다.
        """
        import uipc
        slot = scene.sim.coupler.cloth_slots[(bag, 0)]
        g = slot.geometry()
        ic = uipc.view(g.vertices().find(uipc.builtin.is_constrained))
        ap = uipc.view(g.vertices().find(uipc.builtin.aim_position))
        if reset:
            ic[:] = 0
        ic[idx] = 1
        ap[idx] = np.asarray(tgt, dtype=np.float64).reshape(-1, 3, 1)
        return int(np.asarray(ic).sum())

    def _npy(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    def _vp():
        return _npy(bag.get_state().pos).squeeze()

    def _render():
        if not NO_VIDEO:
            cam.render(); cam2.render()

    DRIVEN = [0, 1, 2]  # 스테이지, L, R. R 을 mimic 에만 맡기면 R 의 기본 PD
                        # (목표 0)와 equality(목표 -50mm)가 싸운다. 1:1 미믹
                        # (polycoef='0 1 0 0 0')이므로 qR = qL 로 같이 명령한다.

    def _tick(name, k, n, t0):
        # 스텝당 벽시계를 찍는다. IPC 가 정체하면 여기서 바로 드러난다.
        if TRACE and (k % TRACE == 0 or k == n - 1):
            st = bag.get_state()
            z = _npy(st.pos).squeeze()[:, 2]
            v = _npy(st.vel).squeeze()
            print(f"[trace] {name:8s} {k+1:4d}/{n}  {time.time()-t0:7.1f}s  "
                  f"bag_z={z.mean()*1e3:7.1f}mm (min {z.min()*1e3:7.1f})  "
                  f"|v|max={np.linalg.norm(v, axis=-1).max():7.2f}m/s", flush=True)

    def drive(q_jaw, n, name):
        q_now = _npy(gripper.get_dofs_position())[:3].copy()
        t0 = time.time()
        for k in range(n):
            s = (k + 1) / n
            _q = q_now[1] + (q_jaw - q_now[1]) * s
            gripper.control_dofs_position(np.array([q_now[0], _q, _q]),
                                          dofs_idx_local=DRIVEN)
            scene.step(); _render(); _tick(name, k, n, t0)
        print(f"[phase] {name:8s} @done  {time.time()-t0:.1f}s  "
              f"jaw={_npy(gripper.get_dofs_position())[1]*1e3:+.2f}mm", flush=True)

    def settle(n, name):
        # control 은 명령을 놓으면 목표가 풀린다. 현재 자세를 계속 명령해 유지한다.
        q_hold = _npy(gripper.get_dofs_position())[:3].copy()
        t0 = time.time()
        for k in range(n):
            gripper.control_dofs_position(q_hold, dofs_idx_local=DRIVEN)
            scene.step(); _render(); _tick(name, k, n, t0)
        print(f"[phase] {name:8s} @done  {time.time()-t0:.1f}s", flush=True)

    # ── 턱을 먼저 열고, 그 다음 봉투를 투입한다 ─────────────────────────────
    drive(JAW_OPEN, N_CLOSE, "open0")
    settle(N_SETTLE, "settle0")
    bag.remove_vertex_constraints()      # 파킹 핀 해제 — 이제 투입한다
    # 컵이 입구 테두리 아래를 물도록 봉투 중심을 그만큼 내린다.
    _tgt_xyz = np.array([SUCTION_POS[0] + CUP_FACE_X,
                         SUCTION_POS[1] + CUP_FACE_Y,
                         SUCTION_POS[2] + CUP_FACE_Z - (MOUTH_Z - CUP_BELOW_RIM)])
    # **정착보다 고정이 먼저다.** set_position 은 COM 절대배치가 맞지만(실측:
    # init_COM_offset mean=0), 놓기만 하고 150스텝 정착시키면 아무것도 안 잡아주는
    # 상태로 0.75s 낙하해 바닥(z=2.8mm)에 눕는다. 투입 위치를 해석적으로 계산해
    # 스텝을 돌리기 전에 하단을 그 자리에 못박는다 — 실제로는 회수장치가 문 상태다.
    # 회수장치는 밑면(F_Top)과 좌우 실링(F_Left/RightLink)을 같이 문다. 밑단 20mm
    # 만 잡으면 흐물한 천이 핀 아래로 접혀 내려앉는다(실측: 중심이 핀보다 390mm
    # 아래). 좌우 실링(로컬 x=+-32mm)의 **아래 절반**까지 잡아 세우고, 입구가 있는
    # 위쪽 절반은 컵이 벌릴 수 있게 자유로 둔다.
    p_new = _npy(bag.init_positions_COM_offset) + _tgt_xyz
    _lo = p_new[:, 2] < p_new[:, 2].min() + 0.020
    _seal = (np.abs(p_new[:, 0] - _tgt_xyz[0]) > 0.030) & (p_new[:, 2] < _tgt_xyz[2])
    bottom = np.where(_lo | _seal)[0]
    bag.set_position(_tgt_xyz)
    if ATTACH:
        _n_on = _attach(bottom, p_new[bottom])
        print(f"[attach] SoftPositionConstraint 로 {_n_on}정점 켬", flush=True)
    elif not NO_PIN:
        # 하드 핀은 고정 정점만 끌고 이웃이 안 따라온다 — 90mm 봉투가 500mm 로
        # 늘어난다(실측). 구속 없는 대조군은 형상을 유지하므로 셸이 아니라 핀이
        # 원인이다. 사용자가 처음 지적한 "한 점만 당겨지는 느낌"이 이 증상이다.
        bag.set_vertex_constraints(verts_idx_local=bottom.tolist(),
                                   target_poss=p_new[bottom],
                                   is_soft_constraint=SOFT, stiffness=STIFF)
    print(f"[bag] 투입 — 목표중심 {np.round(_tgt_xyz*1e3, 1)}mm, "
          f"고정 {len(bottom)}/{len(p_new)}정점 (밑단 {_lo.sum()} + 실링하부 {_seal.sum()})", flush=True)
    settle(N_SETTLE, "drop")
    _all = _vp()
    print(f"[bbox] 봉투 bbox(mm) x[{_all[:,0].min()*1e3:7.1f},{_all[:,0].max()*1e3:7.1f}] "
          f"y[{_all[:,1].min()*1e3:7.1f},{_all[:,1].max()*1e3:7.1f}] "
          f"z[{_all[:,2].min()*1e3:7.1f},{_all[:,2].max()*1e3:7.1f}]  "
          f"(원래 64x6x90mm)  유한={np.isfinite(_all).all()}", flush=True)
    _pv = _vp()[bottom] if not NO_PIN else _vp()[:1]
    print(f"[pin?] 고정정점 실제 z {_pv[:,2].mean()*1e3:7.1f}mm  "
          f"목표 {p_new[bottom][:,2].mean()*1e3:7.1f}mm  "
          f"편차 {np.abs(_pv - p_new[bottom]).max()*1e3:.1f}mm", flush=True)
    _c = _vp().mean(0)
    print(f"[bag] 투입 완료 — 중심 {np.round(_c*1000, 1)}mm "
          f"(목표 {np.round(_tgt_xyz*1e3, 1)}, 오차 "
          f"{np.linalg.norm(_c - _tgt_xyz)*1e3:.1f}mm)", flush=True)


    # ── 닫아서 봉투를 문다 ──────────────────────────────────────────────────
    # **램프로만 움직인다.** L 조만 한 번에 -50mm 로 텔레포트하면 미믹 equality
    # (R = L)가 50mm 어긋나 첫 스텝에 constraint force 가 nan 이 된다(실측).
    drive(JAW_CLOSED, N_CLOSE, "close")
    settle(N_SETTLE, "settle1")

    # ── 3) 컵 면에 닿은 정점을 잡는다 ───────────────────────────────────────
    # 컵 중심은 조인트를 따라 움직이므로 **현재 자세에서** 링크 위치로 구한다.
    # 컵 **면** 중심을 쓴다(메시 중심이 아니다 — 컵 몸통이 y 로 30mm 길어서
    # 중심을 쓰면 면에서 15mm 어긋난다). q=0 에서 두 면이 맞닿아 있고, 봉투가
    # 사이에 있으면 각자 봉투 두께의 절반만큼 물러나 있다.
    _jaw_now = _npy(gripper.get_dofs_position())[1]
    _bx = SUCTION_POS[0] + CUP_FACE_X
    _bz = SUCTION_POS[2] + CUP_FACE_Z
    _by = SUCTION_POS[1] + CUP_FACE_Y
    cup_c = [np.array([_bx, _by + _jaw_now, _bz]),
             np.array([_bx, _by - _jaw_now, _bz])]
    print(f"[cup] 컵 중심 {[np.round(c,4).tolist() for c in cup_c]}")

    # 봉투 두께가 6mm(면이 중심에서 +-3mm)인데 컵 반경은 7.5mm 다. 반경만으로
    # 고르면 각 컵의 구가 봉투를 관통해 **반대쪽 면 정점까지** 잡고, 그러면 두 컵이
    # 같은 정점을 반대로 당겨 상쇄된다(실측: 컵당 16개 최근접 1.6mm -> 개구 2.6mm,
    # 컵당 7개 최근접 3.4mm -> 개구 56.1mm). 접촉면 기준 **자기 쪽 정점만** 고른다.
    _mid_y = 0.5 * (cup_c[0][1] + cup_c[1][1])
    vp = _vp()
    grab = []
    for ci, c in enumerate(cup_c):
        d = np.linalg.norm(vp - c, axis=1)
        sgn = np.sign(c[1] - _mid_y) or (1.0 if ci else -1.0)
        sel = (d < GRAB_R) & (np.sign(vp[:, 1] - _mid_y) == sgn)
        idx = np.where(sel)[0]
        grab.append(idx)
        print(f"[grab] 컵{ci} 반경 {GRAB_R*1e3:.1f}mm 안 정점 {len(idx)}개 "
              f"(반경만이면 {int((d < GRAB_R).sum())}개, 최근접 {d.min()*1e3:.1f}mm)")
    if min(len(g) for g in grab) == 0:
        print("[grab] **한쪽 컵이 봉투를 못 잡았다 — 배치 확인 필요**")
        if not NO_VIDEO:
            cam.stop_recording(); cam2.stop_recording()
        print(f"[result] mouth=nan follow=nan spread=nan n_grab={[len(g) for g in grab]}")
        return

    all_grab = np.concatenate(grab)
    if ATTACH:
        _n_on = _attach(all_grab, vp[all_grab])
        print(f"[grab] 총 {len(all_grab)}정점을 SoftPositionConstraint 로 파지 "
              f"(누적 {_n_on}정점 켬 = 밑단 + 파지)", flush=True)
    else:
        bag.set_vertex_constraints(verts_idx_local=all_grab.tolist(),
                                   target_poss=vp[all_grab],
                                   is_soft_constraint=SOFT, stiffness=STIFF)
        print(f"[grab] 총 {len(all_grab)}정점을 "
              f"{'소프트(k=%g)' % STIFF if SOFT else '하드핀'} 으로 구속")

    # ── 4) 벌리면서 목표를 컵과 함께 옮긴다 ─────────────────────────────────
    # **개루프를 폐루프로 바꾼다.** 종전에는 조인트 명령값 dy 만큼 기억한 좌표를
    # y 로 평행이동했다(side 부호를 손으로 지정). 컵이 명령대로 안 가면 목표가
    # 틀리고, 회전은 아예 반영이 안 된다. 흡착은 "컵에 붙는" 것이므로 파지 순간의
    # **컵 로컬 좌표**를 저장하고 매 스텝 컵의 실제 자세로 되돌린다.
    def _R(q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])

    cup_link = [gripper.get_link(n) for n in CUP_LINKS]
    loc, T0 = [], []
    for ci, idx in enumerate(grab):
        pl, ql = _npy(cup_link[ci].get_pos()), _npy(cup_link[ci].get_quat())
        loc.append((vp[idx] - pl) @ _R(ql))     # R^T (v - p)
        T0.append((pl, ql))
    print(f"[grab] 컵 로컬 좌표로 저장 — 링크 {CUP_LINKS[0]} 등", flush=True)
    y0_l, y0_r = vp[grab[0]][:, 1].mean(), vp[grab[1]][:, 1].mean()
    jaw0 = _npy(gripper.get_dofs_position())[1]
    print(f"\n[phase] open ({N_OPEN}스텝) — jaw {jaw0*1e3:+.1f} -> {JAW_OPEN*1e3:+.1f}mm "
          f"(컵 간격 +{abs(JAW_OPEN-jaw0)*2*1e3:.0f}mm)")
    for k in range(N_OPEN):
        s = (k + 1) / N_OPEN
        qj = jaw0 + (JAW_OPEN - jaw0) * s
        gripper.control_dofs_position(np.array([0.0, qj, qj]), dofs_idx_local=DRIVEN)
        # 컵의 **실제** 자세에서 목표를 복원한다(개루프 dy 평행이동이 아니다).
        tgt = np.concatenate([
            loc[ci] @ _R(_npy(cup_link[ci].get_quat())).T + _npy(cup_link[ci].get_pos())
            for ci in range(2)])
        if ATTACH:
            _attach(all_grab, tgt)
        else:
            bag.update_constraint_targets(verts_idx_local=all_grab.tolist(), target_poss=tgt)
        scene.step(); _render()

    # ── 판정 ────────────────────────────────────────────────────────────────
    vpe = _vp()
    yl, yr = vpe[grab[0]][:, 1].mean(), vpe[grab[1]][:, 1].mean()
    mouth = abs(yr - yl) * 1e3
    mouth0 = abs(y0_r - y0_l) * 1e3
    cup_move = abs(JAW_OPEN - jaw0) * 2 * 1e3
    follow = (mouth - mouth0) / cup_move if cup_move else float("nan")
    spread = float(np.ptp(vpe[all_grab][:, 2]) * 1e3)
    print(f"\n[result] mouth={mouth:.1f}mm (시작 {mouth0:.1f}) "
          f"follow={follow:.3f} spread={spread:.1f}mm "
          f"n_grab={[len(g) for g in grab]}")
    print(f"[result] 개구 {mouth - mouth0:+.1f}mm / 컵 이동 {cup_move:.0f}mm")
    if not NO_VIDEO:
        cam.stop_recording(); cam2.stop_recording()
        print(f"[video] {mp4}")
        print(f"[video] {mp4.replace('.mp4', '_front.mp4')}")


SWEEP = dict(
    GRAB_R_MM=["5", "7.5", "10"],
    SOFT=["0", "1"],
    STIFF=["1e3", "1e4", "1e5"],
    CLOTH_E=["4.0e5", "1.0e6", "4.0e6"],
)


def sweep():
    out = os.path.join(HERE, "SWEEP_BAGOPEN")
    os.makedirs(out, exist_ok=True)
    keys = list(SWEEP)
    jobs = [dict(zip(keys, v)) for v in itertools.product(*SWEEP.values())]
    jobs = [j for j in jobs if not (j["SOFT"] == "0" and j["STIFF"] != "1e4")]  # 하드핀은 강성 무관
    print(f"총 {len(jobs)}개 런\n")
    rows = []
    for j in jobs:
        name = f"R{j['GRAB_R_MM']}_S{j['SOFT']}_K{j['STIFF']}_E{j['CLOTH_E']}"
        log = os.path.join(out, f"{name}.log")
        if os.path.exists(log) and "[result]" in open(log, encoding="utf-8", errors="replace").read():
            print(f"[skip] {name}", flush=True)
        else:
            print(f"[run ] {name}", flush=True)
            with open(log, "w", encoding="utf-8") as f:
                subprocess.run([sys.executable, "-u", os.path.abspath(__file__)],
                               env=dict(os.environ, NO_VIDEO="1", RUN_TAG=name, **j),
                               stdout=f, stderr=subprocess.STDOUT)
        txt = open(log, encoding="utf-8", errors="replace").read()
        m = re.search(r"\[result\] mouth=([\d.na]+)mm \(시작 ([\d.]+)\) follow=([-\d.na]+) "
                      r"spread=([\d.na]+)mm n_grab=\[(\d+), (\d+)\]", txt)
        rows.append((name, j, m.groups() if m else None))
        print(f"[done] {name} -> {m.groups() if m else '실패'}", flush=True)

    print("\n" + "=" * 88)
    print(f"{'런':34s}{'반경':>6}{'구속':>8}{'강성':>8}{'E':>9}   "
          f"{'개구mm':>8}{'follow':>8}{'퍼짐mm':>8}{'정점':>10}")
    print("-" * 88)
    best = []
    for name, j, g in rows:
        if not g:
            print(f"{name:34s}   실패 (SWEEP_BAGOPEN/{name}.log)")
            continue
        mouth, m0, fol, spr, nl, nr = g
        print(f"{name:34s}{j['GRAB_R_MM']:>6}"
              f"{'소프트' if j['SOFT']=='1' else '하드':>8}"
              f"{j['STIFF'] if j['SOFT']=='1' else '-':>8}{j['CLOTH_E']:>9}   "
              f"{float(mouth)-float(m0):>8.1f}{fol:>8}{spr:>8}{nl+'/'+nr:>10}")
        try:
            best.append((abs(1.0 - float(fol)), -float(spr), name, j))
        except ValueError:
            pass
    print("=" * 88)
    if best:
        best.sort()
        print(f"follow 가 1.0 에 가장 가까운 설정: {best[0][2]}")
        print(f"  {best[0][3]}")
        print("follow=1.0 이면 봉투가 컵을 그대로 따라온 것이고, 낮으면 미끄러지거나")
        print("한 점만 끌려간 것이다. 퍼짐(spread)이 크면 면으로 잡힌 것이다.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    sweep() if a.sweep else main()
