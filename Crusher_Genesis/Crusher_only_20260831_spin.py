"""크랭크 회전 변인통제 — IPC/FEM 완전 배제, Crusher 단독 (2026-08-31).

full_workflow.py 의 분쇄 구간에서 크랭크가 8 RPM 지령에 0.02 RPM 밖에 안 나온다.
Crusher_only.py 는 같은 게인(kp=2000/kv=5000)·같은 토크(12.5 N·m)로 정상 회전한다.
두 스크립트의 차이는 셋뿐이다:

    substep_dt   5e-5 (dt 5e-4 / substeps 10)   vs   5e-3 (dt 5e-3 / substeps 1)
    IPC 커플러   없음                            vs   있음 (봉투 FEM + 정제)
    크랭크 시작각 -pi/2                           vs   -pi

이 파일은 **IPC 를 뺀 채 substep_dt 만** 바꿔 돌린다. 그래서 회전이 살아나면
범인은 타임스텝이고, 안 살아나면 IPC 다.

핵심 용의자는 weld 시정수다. patch_mjcf 의 `eq_solref="0.0002 50"` 은 주석에
"substep_dt 5e-5 의 4배"라고 적혀 있는데, Genesis 는 시정수가 2*substep_dt 보다
작으면 말없이 늘린다(rigid_solver.py:228). full_workflow 로그에 실제로 찍혀 있다:

    timeconst is changed from `0.0002` to `0.01`     <- 50배 물러짐

크랭크-슬라이더 폐루프를 닫는 것이 그 weld 이므로, 물러지면 크랭크 토크가
슬라이더로 전달되지 않고 루프가 먹는다.

사용법:
    python Crusher_only_20260831_spin.py                    # A: 원본 조건
    DT=5e-3 SUBSTEPS=1 python Crusher_only_20260831_spin.py # B: full_workflow 조건
"""
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import Crusher_only as C          # patch_mjcf / SRC_XML / 관절명을 그대로 재사용한다
                                  # (복사가 이번 불일치의 원인이었으므로 복사하지 않는다)

DT = float(os.environ.get("DT", "5e-4"))
SUBSTEPS = int(os.environ.get("SUBSTEPS", "10"))
KV = float(os.environ.get("KV", "5000"))
KP = float(os.environ.get("KP", "2000"))
TORQUE = float(os.environ.get("TORQUE", "12.5"))
START_Q = float(os.environ.get("START_Q", str(-np.pi / 2)))
SPIN_S = float(os.environ.get("SPIN_S", "2.0"))
EQ_SOLREF = os.environ.get("EQ_SOLREF", "0.0002 50")

# ── 변인 3개 (2026-08-31) ──────────────────────────────────────────────────
# full_workflow 와 이 파일 사이에 한꺼번에 바뀐 것들이라 하나씩 못 갈랐다.
#   IPC  : IPC 커플러 자체 (변형체는 없어도 됨)
#   BAG  : FEM.Cloth 봉투를 씬에 넣는다 (크러셔에서 떨어뜨려 접촉은 없게)
#   SET  : 벽 DOF 를 매 스텝 set_dofs_position 으로 구동 (WALL_KINEMATIC 재현).
#          set_dofs_position 은 맨 앞에서 collider.reset() + constraint_solver.reset()
#          을 부른다 — 매 스텝 구속 솔버를 리셋하면 weld 로 닫힌 크랭크-슬라이더
#          루프가 웜스타트를 잃는다.
IPC = os.environ.get("IPC", "0") == "1"
BAG = os.environ.get("BAG", "0") == "1"
SET = os.environ.get("SET", "0") == "1"
CONFIG = os.environ.get("CONFIG", "")

MATRIX = [
    # name          IPC  BAG  SET
    ("base",         0,   0,   0),   # = 런 B (6.90 RPM 확인됨)
    ("ipc",          1,   0,   0),   # = 런 C (6.95 RPM 확인됨)
    ("ipc_bag",      1,   1,   0),   # 봉투만 추가
    ("ipc_set",      1,   0,   1),   # set 구동만 추가
    ("ipc_bag_set",  1,   1,   1),   # 둘 다 = full_workflow 조건
]

RPM = 8.0
OMEGA = RPM * 2.0 * np.pi / 60.0


def main():
    substep_dt = DT / SUBSTEPS
    print("=" * 68)
    print(f"[cfg] dt={DT:g}  substeps={SUBSTEPS}  substep_dt={substep_dt:g}")
    print(f"[cfg] kp={KP:g} kv={KV:g}  torque=±{TORQUE:g} N·m  start_q={START_Q:+.4f} rad")
    print(f"[cfg] eq_solref='{EQ_SOLREF}'   (2*substep_dt = {2*substep_dt:g})")
    if float(EQ_SOLREF.split()[0]) < 2 * substep_dt:
        print(f"[cfg] ** weld 시정수가 2*substep_dt 보다 작다 -> Genesis 가 "
              f"{2*substep_dt:g} 로 늘린다 ({2*substep_dt/float(EQ_SOLREF.split()[0]):.0f}배) **")
    print("=" * 68)

    import tempfile
    import shutil
    tmp = tempfile.mkdtemp(prefix="crusher_spin_")
    src_dir = os.path.dirname(C.SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp, f))
    patched = os.path.join(tmp, "Crusher_genesis.xml")
    C.patch_mjcf(C.SRC_XML, patched, eq_solref=EQ_SOLREF)

    import genesis as gs
    gs.init(backend=gs.cuda, logging_level="warning")

    scene_kw = dict(
        sim_options=gs.options.SimOptions(dt=DT, substeps=SUBSTEPS, gravity=(0, 0, -9.81)),
        show_viewer=False,
    )
    mat_kw = {}
    if IPC or BAG:
        # full_workflow.py 와 **동일한** 커플러 설정. 변형체(봉투/정제)는 하나도
        # 넣지 않는다 — "IPC 커플러 존재 자체"가 크랭크를 멈추는지만 본다.
        scene_kw["coupler_options"] = gs.options.IPCCouplerOptions(
            contact_d_hat=1e-3,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        )
        mat_kw["material"] = gs.materials.Rigid(coup_type="two_way_soft_constraint",
                                                coup_friction=0.8)
    scene = gs.Scene(**scene_kw)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=patched, decimate=False, convexify=False),
        surface=gs.surfaces.Default(smooth=False),
        **mat_kw,
    )
    if BAG:
        # full_workflow 와 같은 봉투 STL·재질. 다만 **크러셔에서 0.4m 떨어뜨려**
        # 스폰한다 — 접촉을 배제하고 "씬에 있기만 해도" 크랭크가 멈추는지 본다.
        bag_stl = os.path.join(os.path.dirname(_HERE), "Crusher_Genesis", "assets",
                               "robots", "Samplebag",
                               "Samplebag_seal_pouch3_sealslab3mm.stl")
        if not os.path.exists(bag_stl):
            bag_stl = os.path.join(_HERE, "assets", "robots", "Samplebag",
                                   "Samplebag_seal_pouch3_sealslab3mm.stl")
        scene.add_entity(
            material=gs.materials.FEM.Cloth(E=4.0e5, nu=0.499, rho=200.0,
                                            thickness=1.0e-3, bending_stiffness=400.0,
                                            friction_mu=0.8),
            morph=gs.morphs.Mesh(file=bag_stl, scale=1.0, pos=(0.0, -0.4, 0.4),
                                 euler=(90, 0, 0)),
            surface=gs.surfaces.Default(opacity=0.55, double_sided=True),
        )
        print(f"[bag] FEM.Cloth 추가 (크러셔에서 0.4m 이격, 접촉 없음)")
    scene.build(n_envs=0)

    joints = {j.name: j for j in crusher.joints if j.name}

    def _dof(name):
        d = joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d

    crank_dof = _dof(C.CRANK_JOINT)
    slider_dof = _dof(C.PASSIVE_JOINTS[-1])
    wall_dof = _dof(C.WALL_JOINT)

    crusher.set_dofs_kp(np.array([KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([KV]), dofs_idx_local=[crank_dof])
    fmin = np.full(crusher.n_dofs, -np.inf)
    fmax = np.full(crusher.n_dofs, np.inf)
    fmin[crank_dof], fmax[crank_dof] = -TORQUE, TORQUE
    crusher.set_dofs_force_range(lower=fmin, upper=fmax)

    def _q(d):
        return float(crusher.get_dofs_position().cpu().numpy()[d])

    def _v(d):
        return float(crusher.get_dofs_velocity().cpu().numpy()[d])

    # ── WARMUP : 0 -> START_Q 위치제어 램프 (Crusher_only 와 동일) ─────────
    n_warm = int(round(0.5 / DT))
    for k in range(n_warm):
        crusher.control_dofs_position(np.array([START_Q * (k + 1) / n_warm]),
                                      dofs_idx_local=[crank_dof])
        scene.step()
    print(f"[warmup] 목표 {START_Q:+.4f} -> 도달 {_q(crank_dof):+.4f} rad "
          f"(오차 {abs(_q(crank_dof)-START_Q)*1000:.1f} mrad)")

    # ── SPIN : 속도제어 + 토크 클램프, 매 스텝 기록 ────────────────────────
    n_spin = int(round(SPIN_S / DT))
    q0 = _q(crank_dof)
    print(f"\n[spin] {OMEGA:.4f} rad/s ({RPM} RPM) 지령, {n_spin} step = {SPIN_S}s")
    print(f"{'step':>6} {'t(s)':>7} {'ω(rad/s)':>10} {'RPM':>7} {'Δθ(deg)':>9} {'slider(mm)':>11}")
    every = max(1, n_spin // 40)
    wall_q = float(_q(wall_dof))
    for k in range(n_spin):
        crusher.control_dofs_velocity(np.array([OMEGA]), dofs_idx_local=[crank_dof])
        if SET:
            # WALL_KINEMATIC 재현 — 벽을 제자리에 매 스텝 써넣는다.
            crusher.set_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
        scene.step()
        if k % every == 0 or k == n_spin - 1:
            print(f"{k:6d} {(k+1)*DT:7.4f} {_v(crank_dof):10.5f} "
                  f"{_v(crank_dof)*60/(2*np.pi):7.3f} "
                  f"{np.degrees(_q(crank_dof)-q0):9.2f} {_q(slider_dof)*1e3:11.3f}")

    w_end = _v(crank_dof)
    print(f"\n[결과] 최종 ω={w_end:.5f} rad/s ({w_end*60/(2*np.pi):.3f} RPM) "
          f"/ 지령 {OMEGA:.4f} ({RPM} RPM)   추종률 {w_end/OMEGA*100:.1f}%")
    print(f"[결과] 총 회전 {np.degrees(_q(crank_dof)-q0):+.1f} deg "
          f"({(_q(crank_dof)-q0)/(2*np.pi):+.3f} 바퀴)")


def run_matrix():
    """5개 환경을 각각 별도 프로세스로 돌리고 요약표를 찍는다.

    Genesis 는 프로세스당 씬 하나가 안전하므로 자기 자신을 subprocess 로 부른다.
    """
    import subprocess
    import re

    results = []
    for name, ipc, bag, st in MATRIX:
        env = dict(os.environ, CONFIG=name, IPC=str(ipc), BAG=str(bag), SET=str(st),
                   DT=os.environ.get("DT", "5e-3"),
                   SUBSTEPS=os.environ.get("SUBSTEPS", "1"))
        print(f"[run ] {name:12s} IPC={ipc} BAG={bag} SET={st} ...", flush=True)
        p = subprocess.run([sys.executable, "-u", os.path.abspath(__file__)],
                           env=env, capture_output=True, text=True, errors="replace")
        out = p.stdout + p.stderr
        m = re.search(r"최종 ω=([-\d.]+) rad/s \(([-\d.]+) RPM\).*추종률 ([-\d.]+)%", out)
        deg = re.search(r"총 회전 ([-+\d.]+) deg", out)
        if m:
            results.append((name, ipc, bag, st, float(m.group(2)), float(m.group(3)),
                            float(deg.group(1)) if deg else float("nan")))
            print(f"       -> {m.group(2)} RPM (추종률 {m.group(3)}%)", flush=True)
        else:
            err = re.search(r"(GenesisException|Error|Exception):?\s*(.*)", out)
            results.append((name, ipc, bag, st, float("nan"), float("nan"), float("nan")))
            print(f"       -> 실패: {err.group(0)[:90] if err else 'rc=' + str(p.returncode)}",
                  flush=True)

    print("\n" + "=" * 74)
    print(f"{'환경':14s}{'IPC':>4}{'BAG':>5}{'SET':>5}   {'RPM':>8}{'추종률':>9}{'회전(deg)':>11}")
    print("-" * 74)
    for name, ipc, bag, st, rpm, foll, deg in results:
        print(f"{name:14s}{ipc:>4}{bag:>5}{st:>5}   {rpm:>8.3f}{foll:>8.1f}%{deg:>11.2f}")
    print("=" * 74)
    print(f"지령 {OMEGA:.4f} rad/s = {RPM} RPM   (dt={os.environ.get('DT','5e-3')}, "
          f"substeps={os.environ.get('SUBSTEPS','1')})")


if __name__ == "__main__":
    if CONFIG or os.environ.get("SINGLE") == "1":
        main()
    else:
        run_matrix()
