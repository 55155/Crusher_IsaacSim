"""
suctionV1_only.py — 진공 흡착 그리퍼(석션V1, assets/robots/석션V1_description/
석션V1.xml) 단독 실행/시각 확인용.

배치/충돌 설정(2026-07-27, 사용자 지시):
  - 흡착컵 2개(Suction_Cup_M5_0.8mm_15mm_1/2)만 충돌 활성화, 나머지 부품은
    전부 collision-free(contype=0/conaffinity=0) — 석션V1.xml 에 직접 반영.
  - 그리퍼 조(jaw) 슬라이더 2개(L/R_E-SMLG9H-100-ES10_1_1_..._2_1)는 기본이
    닫힌 상태(qpos=0)이고, range=[-0.05, 0](0~-50mm, 사용자 수정지시)로 제한
    — 음(-) 방향이 두 흡착컵을 서로 안쪽으로 더 조여붙이는 방향이라, 이
    range 끝까지 밀면 흡착컵끼리 충돌해야 정상(수정 전엔 부호가 반대라 늘
    벌어지는 방향만 썼고, 그마저도 enable_rigid_rigid_contact=False라
    충돌 자체가 아예 안 걸렸다 — 둘 다 이번에 수정).
  - `enable_rigid_rigid_contact=True` 로 켬 — 이게 꺼져 있으면 흡착컵 geom에
    contype/conaffinity=1을 줘도 IPC 커플러가애초에 rigid-rigid 접촉 자체를
    검사하지 않아 "충돌감지가 안 된다"는 증상으로 나타난다.
  - 메인 리니어스테이지(LSM-NK174218-1204_100MM__1_Dummy_1)는 그대로 무제한
    (해당 조인트는 사용자 지시(50mm 리밋)의 대상이 아님 — 흡착컵 조 슬라이더만
    해당).
  - 미믹(2026-07-27, 사용자 지시): 석션V1.xml에 <equality><joint .../></equality>
    추가해 R 조 슬라이더가 L을 그대로 따라가게 만들었다(polycoef 1:1, 두 joint
    axis가 이미 +Y/-Y로 반대라 world 기준 대칭 개폐가 된다). 이 데모에서도 실제로
    L(과 스테이지)만 `set_dofs_position(dofs_idx_local=[0,1])`으로 명령하고
    R(dof 2)은 건드리지 않아 — R이 equality 제약만으로 따라오는지 확인한다.

동작 시퀀스: 스테이지 전진 -> 조 압착(0->-50mm, L만 명령·R은 미믹으로 추종,
흡착컵끼리 충돌해 끝까지 못 감을 수 있음 — 충돌감지 확인용) -> 조 해제(-50mm->0)
-> 스테이지 후진.

출력: RESULT/suctionV1_only_<ts>.mp4
"""
import os, sys
from datetime import datetime
import numpy as np

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
MP4 = os.path.join(OUT_DIR, f"suctionV1_only_{_TS}.mp4")

SUCTION_MJCF = os.path.join(paths.ROBOTS_DIR, "석션V1_description", "석션V1.xml")
SUCTION_POS = (0.0, 0.0, 0.3)

DT = 5e-3
IPC_D_HAT = 1.0e-4
STAGE_TRAVEL = 0.06   # 메인 리니어스테이지 전진량(m) - "100mm" 부품이라 여유있게 60mm
JAW_SQUEEZE = -0.05    # 사용자 지시: 흡착컵 조 슬라이더 리밋 = 0~-50mm(음수=압착)

N_EXTEND, N_OPEN, N_CLOSE, N_RETRACT, N_HOLD = 100, 120, 120, 100, 40

# 정면(front) 뷰(2026-07-27, 사용자 지시): +X 쪽에서 보면 후면이라는 피드백에
# 따라 반대쪽(-X)에서 180도 돌려서 보도록 카메라를 뒤집었다 - look 지점은 그대로,
# 카메라만 원점 기준 반대편(-X)으로 이동.
CAM_POS = (-1.0, 0.06, 0.48)
CAM_LOOK = (0.0, 0.06, 0.48)


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" SuctionV1 only — 진공 흡착 그리퍼 단독 실행 (viewer={use_viewer})")
    print("=" * 60)

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=True,   # 흡착컵 충돌감지에 필수(2026-07-27 수정)
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        # 2026-07-27 진짜 원인 발견: 흡착컵 2개는 기본자세(qpos0)에서 이미
        # AABB가 맞닿아 있는데, RigidOptions.enable_neutral_collision 기본값이
        # False라 "neutral 자세에서 이미 닿아있는 self-collision 쌍"은 Genesis가
        # 애초에 무시하도록 설계돼 있다(모델링 오차로 흔히 생기는 미세 겹침 때문에
        # 발산하는 걸 막기 위한 디폴트) — enable_rigid_rigid_contact 이 아니라
        # 이게 진짜 원인이었다. 명시적으로 켜준다.
        rigid_options=gs.options.RigidOptions(
            enable_self_collision=True,
            enable_neutral_collision=True,
        ),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96),
            ambient_light=(0.3, 0.3, 0.32),
            lights=[
                {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
                {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.5},
            ],
        ),
        show_viewer=use_viewer,
    )

    scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(coup_type="ipc_only"))

    gripper = scene.add_entity(
        gs.morphs.MJCF(file=SUCTION_MJCF, pos=SUCTION_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    cam = scene.add_camera(res=(1280, 960), pos=CAM_POS, lookat=CAM_LOOK, fov=50, GUI=False)

    print("\n[build] scene.build() 시작...")
    scene.build(n_envs=0)
    print(f"[build] 성공  n_dofs={gripper.n_dofs}")

    cam.start_recording()

    def _npy(x):
        return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)

    # 미믹 확인용: 스테이지(dof0)+L(dof1)만 명령하고 R(dof2)은 손대지 않는다 —
    # equality 제약만으로 R이 L을 따라오는지 보려는 것.
    DRIVEN_DOFS = [0, 1]

    def run(name, q0, q1, n, check_contact=False):
        for k in range(n):
            s = (k + 1) / n
            q = q0 + (q1 - q0) * s
            gripper.set_dofs_position(q, dofs_idx_local=DRIVEN_DOFS)
            scene.step()
            cam.render()
        actual_q = _npy(gripper.get_dofs_position())
        # 소프트 제약(equality)이라 완전히 0은 아니고 수 mm 수준의 추종 오차가 남는다 —
        # 5mm 이내면 정상(2026-07-27 실측: 최대 2.5mm 수준).
        r_track_err = abs(actual_q[2] - actual_q[1])
        print(f"[phase] {name:8s} @done  commanded(stage,L)={q}  actual_q(stage,L,R)={actual_q}"
              f"  [R이 L 추종, 오차={r_track_err*1000:.2f}mm {'OK' if r_track_err < 5e-3 else 'MISMATCH'}]")
        if check_contact:
            contacts = gripper.get_contacts()
            n_c = len(contacts["link_a"]) if contacts.get("link_a") is not None else 0
            print(f"    [contact] 흡착컵 압착 중 self-contact {n_c}건 감지"
                  f"{'' if n_c else ' (충돌 미검출 — 확인 필요)'}")

    zero = np.zeros(2)

    # 흡착컵 충돌감지 확인(2026-07-27): q=(0,0,0) 기본자세에서 두 흡착컵의
    # AABB가 이미 맞닿아 있음을 별도 스크립트로 확인했다(양쪽 다 y=0.0675에서
    # 정확히 접함) — 즉 조(jaw)를 어느 방향으로 움직이든(+/-) 이 지점에서
    # 멀어지기만 하므로, "압착 중" 이 아니라 이 기본자세에서 검사해야 한다.
    for _ in range(10):
        gripper.set_dofs_position(zero, dofs_idx_local=DRIVEN_DOFS)
        scene.step()
        cam.render()
    contacts = gripper.get_contacts()
    n_c = len(contacts["link_a"]) if contacts.get("link_a") is not None else 0
    print(f"[contact] 기본자세(q=0, 흡착컵 AABB 접촉 지점)에서 self-contact {n_c}건 감지"
          f"{'' if n_c else ' (충돌 미검출 — 확인 필요)'}")

    print(f"\n[extend] 스테이지 0 -> {STAGE_TRAVEL*1000:.0f}mm")
    run("extend", zero, np.array([STAGE_TRAVEL, 0.0]), N_EXTEND)

    print(f"[squeeze] L 조 0 -> {JAW_SQUEEZE*1000:.0f}mm (R은 미믹으로만 추종, 실제로는 개방 방향)")
    run("squeeze", np.array([STAGE_TRAVEL, 0.0]),
        np.array([STAGE_TRAVEL, JAW_SQUEEZE]), N_OPEN)

    print(f"[release] L 조 {JAW_SQUEEZE*1000:.0f}mm -> 0 (기본 닫힘 상태로 복귀)")
    run("release", np.array([STAGE_TRAVEL, JAW_SQUEEZE]),
        np.array([STAGE_TRAVEL, 0.0]), N_CLOSE)

    print(f"[retract] 스테이지 {STAGE_TRAVEL*1000:.0f}mm -> 0")
    run("retract", np.array([STAGE_TRAVEL, 0.0]), zero, N_RETRACT)

    for _ in range(N_HOLD):
        scene.step()
        cam.render()

    cam.stop_recording(save_to_filename=MP4, fps=30)
    print(f"\n[saved] {MP4}")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
