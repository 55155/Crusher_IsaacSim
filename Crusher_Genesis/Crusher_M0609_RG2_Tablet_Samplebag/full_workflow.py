"""
full_workflow.py — 정제 낙하 -> 봉투가 받음 -> M0609+RG2 가 봉투를 파지·리프트
-> Crusher 슬롯(Wall3~Left_Wall gap)까지 옮겨서 삽입.

Crusher_Samplebag.py(§PBD 버전, "박스+봉투를 carrier 로 슬롯에 삽입" 시도)를
참고해 슬롯 위치 계산·크랭크/슬라이더 프리셋 로직을 그대로 가져오고, 봉투
이동 수단만 carrier 텔레포트 대신 **M0609+RG2 로봇이 실제로 파지해서 옮기는
것**으로 바꿨다(§docs/DigitalTwin.md §9 조합8 파이프라인 재사용).

시퀀스:
  1. prep      : 슬롯에 넣기 전 준비 — 크랭크 0→-180°(CRANK_START_Q, L8 크러싱헤드
                 완전 후퇴), Left_Wall 0→+6mm(WALL_OFFSET, 슬롯 개방). 이 두 동작이
                 끝나야 슬롯이 "봉투가 통과할 수 있는" 상태가 된다(사용자 지시).
                 정제/봉투는 아직 로봇 워크스페이스에 그대로.
  2. drop/settle/close/grasp/lift/hold : tablet_bag_grasp_pipeline.py 그대로
                 (정제 낙하 -> 봉투 안착 -> 그리퍼 파지 -> 리프트).
  3. above     : 그리퍼(+봉투+정제)를 슬롯 바로 위(gap_cx, gap_cy, 0.30)로 이동.
                 목표 EE pose 는 역기구학(inverse_kinematics)으로 계산 — Q_LIFT
                 자세의 orientation 을 유지한 채 position 만 슬롯 위로 바꾼다.
  4. insert    : 슬롯 안(gap_cx, gap_cy, 0.15, 부분 삽입 — Crusher_Samplebag.py
                 와 동일하게 완전 삽입은 안 함)까지 하강.

**봉투 형상 고정(2026-07-15 추가)**: 1차 시도에서 봉투가 아무 지지 없이
중력만으로 버티다 정제가 들어가기도 전에 처져버려(§docs/DigitalTwin.md §9),
이후 grasp/lift 전부 실패했다(그리퍼는 126mm 올라갔는데 봉투는 그 자리에
그대로). 원인 진단 결과 **봉투를 고정하는 함수가 아예 없었다** — grasp는
처음부터 끝까지 순수 마찰 접촉이었는데(치트 없음), 그 전에 형상 자체가
무너져 있었던 것.
Genesis `FEMEntity.set_vertex_constraints()`가 정확히 이 용도(PBD의
`fix_particles_to_link` 대응)지만, **IPC 커플러 사용 시 예외를 던지는
버그**(`fem_entity.py` `isinstance(coupler, IPCCoupler)` 체크가 뒤집혀 있음,
실측 확인)로 우리 씬(IPC)에서는 그대로 못 쓴다. `utills/fem_ipc_workarounds.
patch_fem_vertex_constraints()`로 이 버그만 우회하는 몽키패치를 적용한다
(Genesis 설치본은 안 건드림).
바닥 밴드만 고정하면 위쪽(입구)이 결국 접혀 무너지길래(격리 테스트 확인),
**바닥 + 양 측면**(정점의 39% 정도, 입구 쪽은 완전히 자유)을 고정 — 설계
치수(90mm)를 1초 이상 정확히 유지함을 격리 테스트로 확인했다. `prep`~
`settle`까지 고정 유지, `close`(그리퍼 닫기) 직전에 `remove_vertex_
constraints()`로 전부 해제 — 이후 grasp/lift는 여전히 순수 마찰로 진행.

카메라 2대: overview(고정 광각, Crusher+로봇 전체) + bagcam(매 프레임 봉투
COM 을 추적하는 동적 카메라).

**후속 튜닝(2026-07-15 2차)**: 사용자 시각 검수 결과 5가지 수정.
  1. 봉투가 above/insert 구간에서 과하게 흔들림 -> CLOTH_E 1e5->4e5,
     CLOTH_BEND 50->400 로 stiffen + FEM_DAMPING=0.2 추가.
  2. 정제가 우그러진 형상으로 보임 -> IPC_D_HAT 1e-4(정제 극 근처 최소 정점
     간격 0.32mm 대비 비율 0.31, 경계값)을 5e-5(비율 0.16)로 낮춤(§docs/
     DigitalTwin.md §9 조합5 절차 재적용).
     **[오진 정정 2026-08-04]** 접촉 파라미터 문제가 아니라 **시각 메시** 문제였다
     — 몽키패치가 반환값 3개 중 세 번째(표면 메시)를 None 으로 지어내 정제가
     깨져 렌더된 것(§DigitalTwin.md §13-9). d_hat 은 1e-4 로 원복돼 있어 무해.
  3. 격자무늬 Ground 가 렌더에 보임 -> `gs.morphs.Plane(visualization=False)`
     로 충돌(안전망)은 유지하되 렌더만 끔 — 씬엔 알루미늄 플레이트만 보임.
  4. 조명이 어두움 -> ambient_light 상향 + 45도 방향 키 라이트(intensity 8)
     + 반대편 약한 필 라이트(intensity 3) 추가.
  5. 슬롯 삽입이 가장 불안정 -> N_ABOVE/N_INSERT 200->400(더 천천히) +
     above/insert 의 목표 orientation 을 Q_LIFT 그대로(wrist=+90°) 대신
     wrist=0 으로 되돌린 순간의 gripper quat 으로 교체(불필요한 90도 twist
     제거, 팔이 덜 웅크리게 됨).

**주의**: 로봇 원래 위치((0, 0.7, 0), scene_setup.py)는 슬롯(약 (-0.33,-0.05,
0.09))과의 거리가 0.87m로 이 자세(orientation 고정)에서 IK 오차가 12cm까지
나서, 로봇을 슬롯에 더 가까운 (-0.33, -0.65, 0) 로 재배치했다(오차 <0.001m
확인). scene_setup.py 의 배치와는 다르다.

출력: RESULT/full_workflow_<ts>_overview.mp4, RESULT/full_workflow_<ts>_bagcam.mp4
"""
import os, sys, shutil, tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try: _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError): pass

_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths

sys.path.insert(0, os.path.join(os.path.dirname(_r), "utills"))
from primitive_tablet_generator import make_capsule_tets_v2, add_analytic_fem_entity
from fem_ipc_workarounds import patch_fem_vertex_constraints

# Y_OFFSET_MM(2026-07-23, 사용자 지시): slot_fit_check.py 의 carrier 기반 스윕이
# IPC 커플러 구조상 근본적으로 불가능하다고 확인된 뒤(soft constraint 는
# `fem_solver.py substep_pre_coupling`이 IPC 커플러일 때 FEM 스텝 자체를 건너뛰어
# 완전히 무반응이고, hard constraint 는 매 스텝 위치를 강제로 덮어써 충돌을 무시함
# — 튜닝으로 해결 불가) — 실제 그리퍼가 마찰로 봉투를 쥐고 옮기는 이 파이프라인의
# above/insert IK 타깃 Y 를 직접 스윕하는 것만이 물리적으로 신뢰 가능한 검증 방법.
Y_OFFSET_MM = float(os.environ.get("Y_OFFSET_MM", "0"))
Y_OFFSET = Y_OFFSET_MM * 1e-3

# 정제 제외 스위치 — 봉투(FEM.Cloth)+IPC 커플러만 격리 검증할 때 쓴다(아래 사용처 주석).
SKIP_TABLET = os.environ.get("SKIP_TABLET", "0") == "1"

# ── 파지 위치 스윕(2026-08-14, 사용자 지시) ─────────────────────────────────
# 봉투 로컬 폭축(BAG_EULER 회전으로 world Y 에 매핑) 상에서 그리퍼가 무는 지점:
#     0    = 봉투 폭 정중앙(입구 한가운데를 뭄) — 봉투가 핑거 바로 아래 매달린다
#   -28mm  = 세로 실링 가장자리(§5-2 에서 검증된 현행 파지)
# 이 값이 곧 핑거 TCP ↔ 봉투 중심의 Y 오프셋(부호 반전)이라, 슬롯 정렬 보정
# (BAG_DY_FROM_FINGER)과 above/insert IK 타깃이 전부 자동으로 따라온다.
# 가장자리를 물수록 봉투에 굽힘 모멘트가 걸려 삽입 중 쐐기처럼 끼는 것으로
# 의심돼(§14-6 trim 발산) 정중앙~가장자리를 스윕해 비교한다.
GRIP_OFFSET_MM = float(os.environ.get("GRIP_OFFSET_MM", "-28"))

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RESULT")
os.makedirs(OUT_DIR, exist_ok=True)
# 케이스별 하위 디렉토리(사용자 지시) — 영상이 케이스마다 분리돼 쌓인다.
CASE_DIR = os.path.join(OUT_DIR, f"grip{GRIP_OFFSET_MM:+03.0f}mm")
os.makedirs(CASE_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
_TAG = f"grip{GRIP_OFFSET_MM:+.0f}mm_yoff{Y_OFFSET_MM:+.1f}mm_{_TS}"
MP4_OVERVIEW = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_overview.mp4")
MP4_BAGCAM = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_bagcam.mp4")
MP4_SIDE = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_sideview.mp4")

# ── Crusher + plate (scene_setup.py 동일) ───────────────────────────────────
CRUSHER_SRC_XML = paths.MJCF_MAIN
CRUSHER_POS = (0.0, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 90.0)
PLATE_PATH = paths.ALUMINUM_PLATE
PLATE_POSITIONS = [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]

WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
L7_LINK3_COM = "0.006 0 -0.005"

# ── Crusher 슬롯 계산에 필요한 벽 mesh(Crusher_Samplebag.py 동일) ──────────
WALL_BACK_MESH = "L2_Wall3_1"
WALL_LEFT_MESH = "L2_Left_Wall1_1"
LEFTWALL_BODY_POS = (-0.017802, 0.286278, 0.016542)
LEFTWALL_GEOM_POS = (-0.286278, -0.016542, 0.017802)
_R_GEOM_HALF = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])

CRANK_JOINT = "L3_Bevel_GearBox_1_L4_Shaft_1"
WALL_JOINT = "L1_Guide1_1_L2_Left_Wall1_1"
CRANK_START_Q = -np.pi   # -180 deg: L8 크러싱헤드 완전 후퇴(슬롯 밖)
WALL_OFFSET = 0.006      # 슬롯 개방(+6mm)
# Left_Wall(Motor2, Rack&Pinion) = 실제 봉투 고정 기구(docs/Crusher.md §5, §11-5:
# "샘플백을 단단히 고정...모터를 계속해서 구동시킴을 통해서 강하게 고정"). 로봇은
# 봉투를 gap 근처까지만 넣어주면 되고, 이후 이 벽이 닫히며 실링부를 Wall3 에
# 눌러 고정한다 — Crusher_Samplebag.py 의 CLAMP_TARGET 재사용(개방 +6mm →
# 클램프 -5mm, 총 11mm 이동해 6mm 두께 봉투를 압착).
CLAMP_TARGET = -0.005
CRANK_KP, CRANK_KV = 2000.0, 100.0
WALL_KP, WALL_KV = 5000.0, 500.0


def patch_crusher_mjcf(src, dst, eq_solref="0.0002 50", eq_solimp="0.999 0.99999 1e-5"):
    tree = ET.parse(src); root = tree.getroot()
    eq = root.find("equality")
    if eq is not None:
        for j in list(eq.findall("joint")):
            eq.remove(j)
        for w in eq.findall("weld"):
            w.set("solref", eq_solref)
            w.set("solimp", eq_solimp)
    wb = root.find("worldbody")
    if wb is not None:
        for g in list(wb.findall("geom")):
            if g.get("name") == "ground":
                wb.remove(g)
        for g in wb.iter("geom"):
            if g.get("mesh") in WALL_GEOMS_TO_ENABLE:
                g.attrib.pop("contype", None)
                g.attrib.pop("conaffinity", None)
        for body in wb.iter("body"):
            if body.get("name") == "L7_Link3_1":
                inertial = body.find("inertial")
                if inertial is not None:
                    inertial.set("pos", L7_LINK3_COM)
    tree.write(dst)


def _prepare_crusher_mjcf():
    tmp_dir = tempfile.mkdtemp(prefix="crusher_mjcf_fw_")
    src_dir = os.path.dirname(CRUSHER_SRC_XML)
    for f in os.listdir(src_dir):
        s = os.path.join(src_dir, f)
        if os.path.isfile(s):
            shutil.copy2(s, os.path.join(tmp_dir, f))
    dst = os.path.join(tmp_dir, "Crusher_genesis.xml")
    patch_crusher_mjcf(CRUSHER_SRC_XML, dst)
    return dst


def crusher_mesh_world_aabb(mesh_name, body_pos=(0., 0., 0.), geom_pos=(0., 0., 0.)):
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw), np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(paths.MJCF_DIR, f"{mesh_name}.stl")).vertices * 0.001
    local = np.asarray(geom_pos) + v
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T
    w = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T
    return w.min(axis=0), w.max(axis=0)


# ── M0609+RG2(v2) + 정제 + 샘플백 ───────────────────────────────────────────
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag", "Samplebag_seal_pouch3.stl")
# 고정장치(fixture jig, fusion2urdf URDF → MJCF 변환, 2026-07-20): 사용자 지시대로
# 나머지 부품(base_link/L1/Back/R1/F1/MotorDriver/T1/Servo1~3/ServoShaft)은 전부
# contype=0/conaffinity=0(collision-free, 시각 전용)이고, 실제로 뭔가를 붙잡는
# Jig_1만 충돌을 켠다 — Jig_1은 CoACD로 64개 볼록껄(hull)로 분해해 collision
# geom으로 추가(utills 관례와 동일, run_coacd_leftwall.py 참고). 덕분에
# Servo3_ServoShaft(continuous) 힌지도 원본 그대로 살렸다 — 예전에 이 힌지를
# 살리면 걸리던 self-intersection sanity check는 Servo 주변 메시들이 이제
# 충돌에 안 걸려서 더 이상 발생하지 않는다. 작업 구역(로봇/Crusher/슬롯)과 안
# 겹치는 플레이트 위 한쪽에 배치, z는 조립체 최저점(~-0.117m)이 플레이트
# 상단(z=0)보다 위로 오도록 여유를 둠.
FIXTURE_MJCF = paths.ascii_safe_mjcf(os.path.join(paths.ROBOTS_DIR, "고정장치_description", "고정장치.xml"))
FIXTURE_POS = (0.5, -0.3, 0.12)

# 석션V1 배치(2026-07-29 재작성, 사용자 지시): 더 이상 정사각형 레이아웃의
# 대칭 위치가 아니다 — 실제 작업 구조("회수장치2가 봉투를 고정하고, 석션V1의
# 흡착컵 2개가 봉투를 양옆으로 당겨서 여는 구조")에 맞춰, 회수장치2의
# F_LeftLink_1-F_RightLink_1 중점에 석션V1의 흡착컵 2개(Suction_Cup_M5_
# 0.8mm_15mm_1/2) 중점이 오도록 정렬한다.
#   F_LeftLink_1/F_RightLink_1 중점(회수장치2 원점 기준, trimesh bounds 실측):
#     (-0.08845, 0.06513, 0.098)
#   흡착컵 중점(석션V1 원점 기준, q=(0,0,0) 기본자세, genesis 충돌 geom 실측):
#     (-0.07790, 0.06750, 0.28594)
#   world_target = RECOVERY2_POS + F링크중점 = (0.347565, -0.22159, 0.367524)
#   SUCTIONV1_POS = world_target - 흡착컵중점 = (0.42547, -0.28909, 0.08158)
# X는 Y,Z와 달리 자유도(요 회전 미정과 동일한 이유) — 위 X 그대로 두면 석션V1의
# 몸체(Dummy_1 등)가 고정장치 본체(world: T1/L1/R1/base_link)와 실측 겹침이
# 확인돼(link 단위 vAABB 대조) X를 +0.25 밀어 물리적 간섭을 제거했다.
# **미해결 이슈**: 이 세 조립체(고정장치+회수장치2+석션V1)를 전부 한 IPC 씬에
# 넣으면, 서로 안 겹치는 상태에서도 build 가 "Intersection detected" 로 죽는다
# — 원인은 회수장치2 "자체 내부"의 Shaft_copy_1↔M_Bottom_1 겹침(둘 다 처음부터
# CAD상 맞닿아있는 구조, 고정장치+회수장치2 둘만 있을 땐 안 걸리다가 3번째
# entity가 추가되면 갑자기 잡힘 — IPC 유효성 검사가 씬의 전체 entity 수에 따라
# 관대함이 달라지는 것으로 추정). contype/conaffinity 비트마스크, MJCF
# <contact><exclude> 전부 시도했으나 해결 못함(exclude는 Genesis mjcf.py 파싱
# 자체가 깨짐). 사용자 확인 필요 - 아래 SUCTIONV1_POS는 좌표 계산은 맞지만
# 현재 이 상태로는 전체 파이프라인이 build 단계에서 죽는다.
SUCTION_MJCF = paths.ascii_safe_mjcf(os.path.join(paths.ROBOTS_DIR, "석션V1_description", "석션V1.xml"))
SUCTIONV1_POS = (0.675, -0.28909000, 0.08158463)

# 회수장치2는 정사각형 레이아웃이 아니라 고정장치 위에 조립(2026-07-27, 사용자
# 지시) — 각자의 원점(0,0,0) 기준 상대좌표로 준 두 정렬점을 world 좌표에서
# 일치시킨다:
#   고정장치 쪽: (-138.00, 72.50, 151.30)mm = ServoShaft 중심
#   회수장치2 쪽: (-74.015, 59.22, 1.776)mm
# 기구적으로 두 점이 정확히 한 점에서 만날 순 없다(사용자 지시) — Y,Z만 맞추면
# 충분하지만, X도 맞춰서 손해볼 게 없어 그대로 3축 다 일치시켰다.
#   world_target = FIXTURE_POS + (-0.138, 0.0725, 0.1513) = (0.362, -0.2275, 0.2713)
#   RECOVERY2_POS = world_target - (-0.074015, 0.05922, 0.001776)
#
# 2026-07-27~29 경위: 처음엔 Jig_1 hull(scale 누락 버그 수정 후) 35/64개가
# 회수장치2 "시각적" 형상과 겹쳐서 Z를 +25mm 띄웠었다. 그런데 이후 Jig hull
# 실제 형상을 렌더링해서 확인해보니 — Jig는 "담아 고정"하는 컵이 아니라
# **양쪽 기둥 2개로 회수장치2의 Shaft Handle(ShaftHandle_1)을 돌려주는
# 포크/렌치 구조**였다(사용자 확인). 즉 Jig와 회수장치2 샤프트 쪽이 근접/접촉
# 하는 건 회피해야 할 버그가 아니라 원래 의도된 동력전달 방식 — ShaftHandle_1
# 자체는 hull 충돌 활성화 후 재검증해도 Jig와 전혀 안 겹쳤다(0/64, 전 회전각).
# 그래서 인위적인 +25mm 간격을 걷어내고 원래 정렬점(X,Y,Z 전부 일치)으로
# 되돌린다 — 요(yaw) 회전은 아직 안 맞춰서(사용자가 추후 지시 예정) 완벽한
# "기둥이 손잡이를 미는" 배치는 아니지만, 최소한 불필요한 뜬 간격은 없앤 상태.
RECOVERY2_MJCF = paths.ascii_safe_mjcf(os.path.join(paths.ROBOTS_DIR, "회수장치2_description", "회수장치2.xml"))
RECOVERY2_POS = (0.436015, -0.28672, 0.269524)
# 실링부 색칠(2026-07-15 4차, 사용자 지시): Genesis 는 로드된 mesh 의
# vertex_colors 를 그대로 렌더에 반영하지 않는다(격리 테스트로 확인 — PLY에
# vertex_colors 를 구워도 단색으로만 나옴). UV + ImageTexture 조합만 동작
# 확인됨(§isolated test) — 봉투 로컬 좌표(폭 ±32mm, 높이 ±45mm, trimesh 로
# 로드시 이미 원점 중심)에 평면 UV(u=x/64mm+0.5, v=y/90mm+0.5)를 구워 OBJ 로
# 내보내고, 그 좌우 가장자리(seal, |local_x|>22mm 부근)에 해당하는 U 구간만
# 색이 다른 텍스처를 입힌다.
BAG_PANEL_HALF_W, BAG_PANEL_HALF_H = 0.032, 0.045
SEAL_BAND_WIDTH = 0.010                     # 가장자리에서 10mm(§docs "~1cm")
BAG_BODY_COLOR = (247, 247, 242)
SEAL_COLOR = (200, 60, 40)

CAP_RADIUS_MM, CAP_CYL_H_MM = 2.0, 1.0
TABLET_E, TABLET_NU, TABLET_RHO = 5.0e4, 0.45, 1300.0
TABLET_FRICTION = 0.5

DT = 5e-3
# contact_d_hat 은 씬 전체(플레이트4개+Crusher+로봇+봉투+정제)의 모든 접촉쌍에
# 적용되는 커플러 전역 설정이라, 5e-5 로 낮췄더니 build() 내부 warm-start 솔브가
# 30분+ 로 폭증(직전 1e-4 실행은 빌드+전체스텝+인코딩 합쳐 16분18초). 정제 극
# 근처 최소 정점 간격(0.32mm) 대비 1e-4 의 비율은 0.31 로 이미 문서 기준
# "1/3 이하" 안전 마진 안이므로, 성능 회귀를 감수할 근거가 부족해 원복한다.
IPC_D_HAT = 1.0e-4

# 봉투 stiffen(2026-07-15): above/insert 구간에서 봉투가 과하게 흔들려 E/굽힘
# 강성을 올림(1e5->4e5, bend 50->400) — 이동 속도를 늦추는 것(N_ABOVE/N_INSERT)과
# 함께 스윙을 줄이기 위한 조합 처방.
CLOTH_E, CLOTH_NU, CLOTH_RHO = 4.0e5, 0.499, 200.0
CLOTH_THICK, CLOTH_BEND = 1.0e-3, 400.0
CLOTH_FRICTION = 0.8
FEM_DAMPING = 0.2

# 2026-07-16 9차: BAG_EULER 를 (90,0,0)->(90,0,90) 로 바꿈에 따라(아래 참고)
# 그리퍼 닫힘축도 world Y->X 로 되돌아가야 해서 손목(joint 6) 트위스트(+90°)
# 를 제거(0°) — FK 실측 확인: wrist=0 일 때 핑거가 X축으로, wrist=+90°일 때
# Y축으로 벌어짐(box 실험 시절의 "네이티브" 축은 X, 봉투 두께=Y 시절에만
# +90°가 필요했다).
FINGER_LINKS = ("f1_flex_finger", "f2_flex_finger")
# 2026-07-24 재계산(사용자 지시, DigitalTwin.md 조합9 후속3/4): 기존 Q_GRASP/
# Q_LIFT는 IK가 아니라 손튜닝값이라 FK 실측 결과 핑거 진행축이 완전 수직에서
# 13.8도 어긋나 있었다(파지 시 봉투 비틀림의 유력 원인). 또한 IK가 지금까지
# `left_link`(f1) "하나"만 타겟으로 잡고 있어 f1-f2 진짜 중앙(TCP)과 20mm(핑거
# 간격 40mm의 절반) 어긋나 있었다(신규 발견) — f1 로컬 프레임 기준 오프셋은
# 정확히 [-20mm,0,0](FING_CLOSE 고정 상태에서 Q_GRASP/Q_LIFT 양쪽 모두 0.12mm
# 이내로 일치, config-불변 확인).
# 기존 두 자세의 "진짜 중앙" 위치는 그대로 유지한 채(이미 물리 검증된 위치),
# orientation만 완전 수직(Y축 180도 회전, quat=[0,0,1,0])으로 강제해 다시 풀었다
# (init_qpos=기존값으로 워밍스타트 -> 같은 분기해로 수렴, 조인트 한계 여유 충분
# 확인: joint3 사용량 1.17rad/0.31rad vs 한계 ±2.618rad). q2+q3+q5 합이
# GRASP_NEW/LIFT_NEW 양쪽에서 거의 동일(3.1414 vs 3.1414)해서 기존 설계의
# "리프트 중 orientation 불변" 트릭도 그대로 유지됨을 확인했다.
FINGER_TCP_LOCAL = np.array([-0.020, 0.0, 0.0])  # f1 로컬 프레임 기준 f1-f2 진짜 중앙 오프셋
VERTICAL_QUAT = np.array([0.0, 0.0, 1.0, 0.0])   # 완전 수직(핑거 진행축이 world -Z와 정확히 반대)
Q_GRASP = np.array([-0.00063, -0.23137, 1.17360, 0.00050, 2.19917, -0.00054], float)
Q_LIFT = np.array([-0.00063, 0.11974, 0.31246, 0.00097, 2.70918, 0.00005], float)
FING_OPEN, FING_CLOSE = 1.00, 1.20

# 슬롯(약 (-0.33,-0.05,0.09))에서 0.87m 떨어진 원래 위치((0,0.7,0))는 orientation
# 고정 IK 오차가 12cm 까지 났다 — 슬롯에 훨씬 가까운 위치로 재배치(오차 <0.001m
# 확인, 이 스크립트 개발 중 격리 테스트로 검증).
ROBOT_OFFSET = np.array([-0.330, -0.65, 0.0])
# 2026-07-24 재계산: Q_GRASP_NEW에서 f1-f2 진짜 중앙(FINGER_TCP_LOCAL 적용)을
# FK로 실측한 값 - ROBOT_OFFSET (이전 FINGER_MID_BASE는 f1 단독 기준이라 위
# 20mm 어긋남 버그를 그대로 포함하고 있었다).
FINGER_MID_BASE = np.array([0.20346321, 0.00617990, 0.43625346])
FINGER_MID = FINGER_MID_BASE + ROBOT_OFFSET

BAG_SCALE = 1.0
# **버그 발견(2026-07-16, 사용자 지적)**: BAG_EULER=(90,0,0) 에서는 봉투의
# world 좌표계 크기가 X=64mm(폭+실링), Y=6mm(두께), Z=90mm(높이) 였는데,
# 슬롯 gap 은 X=12mm(좁음), Y=65mm(여유) — 봉투의 넓은 면(64mm)이 좁은 12mm
# gap 과, 얇은 면(6mm)이 여유로운 65mm 쪽과 부딪히는 **축이 뒤바뀐 상태**였다.
# 원래 참고했던 Crusher_Samplebag.py 는 BAG_EULER=(90,0,**90**) 을 썼는데
# (마지막 Z축 90도가 우리 코드에서 빠져 있었음) 이걸 추가하면 X=6mm(두께,
# gap 12mm 대비 여유 3mm씩), Y=64mm(폭, gap 65mm 대비 여유 0.5mm씩 — 타이트
# 하지만 통과 가능)로 정확히 뒤바뀐다. 높이(local Y->world Z)는 이 변경과
# 무관하게 그대로다(trimesh 로 직접 회전행렬 적용해 검증 완료).
BAG_EULER = (90, 0, 90)
# SEAL_LOCAL_X 는 로컬 mesh 폭축(파지 지점) 오프셋 — 이제 그 축이 world Y 로
# 매핑되므로(위 회전 변경), BAG_POS 적용 위치도 X->Y 로 옮긴다.
# 2026-08-14: 상수에서 GRIP_OFFSET_MM 스윕 파라미터로 승격(파일 상단 주석 참고).
SEAL_LOCAL_X = GRIP_OFFSET_MM * 1e-3
BAG_HALF_H = 0.045
# 파지 위치를 봉투 거의 최상단(입구 쪽)으로 이동(2026-07-15 4차, 사용자 지시).
# 이전엔 FINGER_MID 가 봉투 중간 높이(local_y=0)에 오도록 BAG_POS_z=FINGER_MID_z
# 였는데, 입구에서 8mm 안쪽(맨 가장자리는 잡을 재료가 부족해 8mm 마진)에 오도록
# BAG_POS(봉투 중심)를 그만큼 아래로 내린다.
TOP_GRIP_MARGIN = 0.008
BAG_POS = (FINGER_MID[0], FINGER_MID[1] - SEAL_LOCAL_X,
           FINGER_MID[2] - BAG_HALF_H + TOP_GRIP_MARGIN)

# ── 핑거 TCP ↔ 봉투 몸체의 고정 오프셋 (2026-08-14, 슬롯 미삽입 근본원인) ─────
# 그리퍼는 봉투의 **세로 실링 가장자리**(로컬 폭축 SEAL_LOCAL_X=-28mm, 위 회전
# 으로 world +Y 에 매핑)를 문다 — 종이를 왼쪽 끝만 집어 든 것과 같아서, 64mm
# 폭의 봉투 몸체가 핑거 한쪽으로 통째로 뻗어 있다. 그런데 above/insert IK 는
# **핑거 TCP** 를 gap 중심에 맞추고 있었으므로, 봉투 몸체 중심은 슬롯 중심에서
# 항상 +28mm 어긋난 채 내려갔다(trimesh 실측: 봉투 Y 스팬 [-0.0513,+0.0127] vs
# gap Y 창 [-0.0798,-0.0148] → 폭 64mm 중 27.5mm 가 벽 윗면 위). 즉 봉투 절반이
# Wall3/Left_Wall 상면에 얹힌 채 눌리는 것이고, 천이라 그대로 접혀버린다
# (§docs/DigitalTwin.md §13-7 의 "설명 안 되는 tilt 26.3°" 도 같은 원인).
# → 슬롯 정렬 기준을 핑거가 아니라 **봉투 몸체**로 바꾼다(아래 target_xy).
BAG_DY_FROM_FINGER = BAG_POS[1] - FINGER_MID[1]             # = -SEAL_LOCAL_X = +0.028
# 봉투는 입구에서 TOP_GRIP_MARGIN 아래를 물리므로 핑거 밑으로 이만큼 늘어진다.
# "봉투 최하단을 Wall_1 중간 높이에 둔다"(사용자 목표)를 그대로 역산한 값 —
# insert_z(핑거) = wall_center_z + BAG_HANG_BELOW_FINGER.
# (구값 INSERT_MARGIN_ABOVE_CENTER=0.052 는 핑거-Crusher 충돌 경계에서 뽑은
#  "최소 여유"였을 뿐 목표 깊이와 무관했다 — 82mm 는 그 경계보다 30mm 더 높아
#  충돌 여유는 오히려 늘어난다.)
BAG_HANG_BELOW_FINGER = 2 * BAG_HALF_H - TOP_GRIP_MARGIN    # 0.082

SHELF_TOP = BAG_POS[2] - BAG_HALF_H - 0.0015
SHELF_SIZE = (0.10, 0.10, 0.02)
SHELF_POS = (BAG_POS[0], BAG_POS[1], SHELF_TOP - SHELF_SIZE[2] / 2)

BAG_MOUTH_Z = BAG_POS[2] + BAG_HALF_H
TABLET_DROP_H = 0.015
TABLET_POS = (BAG_POS[0], BAG_POS[1], BAG_MOUTH_Z + TABLET_DROP_H)

N_PREP = 200
N_DROP, N_SETTLE, N_CLOSE, N_GRASP, N_LIFT, N_HOLD = 150, 60, 80, 40, 200, 100
# above/insert 를 2배로 늘려 슬롯 접근을 더 천천히(사용자 지시) — 봉투 스윙도 완화.
N_ABOVE, N_INSERT, N_SETTLE2 = 400, 400, 100
# above 도착 후 하강 전 정지 대기(2026-08-14). 봉투는 핑거에 매달린 펜듈럼이라
# 팔이 멈춰도 바로 멈추지 않는다 — 흔들리는 채로 12mm 슬릿에 넣으면 벽에 긁힌다.
# 여기서 감쇠시킨 뒤 내려간다(ease 프로파일과 함께 쓰는 스윙 억제 처방).
N_ABOVE_SETTLE = 200
# clamp: Left_Wall 이 실링부를 누르는 구간(개방 대비 훨씬 짧고 정밀한 이동이라
# Crusher_Samplebag.py N_CLAMP=2000(@dt=1e-3, 2.0s)와 동일 시간이 되도록 환산).
N_CLAMP, N_RELEASE = 400, 100

CAM_LOOK = tuple(FINGER_MID + np.array([0, 0, 0.03]))
OVERVIEW_CAM_POS = (0.9, -1.7, 1.3)
OVERVIEW_CAM_LOOK = (-0.15, -0.35, 0.15)
BAGCAM_OFFSET = np.array([0.20, -0.20, 0.12])
# 슬롯 삽입 사이드뷰(사용자 요청, 2026-07-27): gap(X, "슬롯 두께" 방향)과 삽입
# 깊이(Z)를 정면 단면으로 보여주는 카메라 — 순수 -Y 방향에서 +Y 를 바라봐
# X-Z 평면이 왜곡 없이 그대로 잡힌다(gap_cx/gap_cy 계산 후 set_pose 로 확정).
SIDECAM_Y_OFFSET = -0.55


def _npy(x):
    return x.cpu().numpy() if hasattr(x, "cpu") else np.asarray(x)


def ease(s):
    """5차 최소저크 프로파일(2026-08-14). 선형 램프 `s=(k+1)/n` 는 구간 양끝에서
    속도가 계단으로 튀어 — 즉 무한대 가속 임펄스 — 매달린 봉투를 펜듈럼처럼
    때린다. `above` 종료 시점의 봉투 스윙(§13-7 "하강 중 어딘가에 끌린다")에
    직접 기여하는 항이라, 시작·끝의 속도와 가속도가 모두 0인 프로파일로 바꾼다.
    총 이동시간(스텝수)은 그대로라 궤적의 1:1 대응(§13-1)은 유지된다."""
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def solve_descent_waypoints(robot, link, target_xy, z0, z1, n_way=41):
    """above->insert 하강을 **카테시안 직선**으로 만드는 웨이포인트 IK(2026-08-14).

    양 끝점의 조인트각만 선형보간하면 카테시안 경로는 직선이 아니라 옆으로 부푼다.
    `slot_ik_check.py` 실측: 하강 중간(s=0.5)에서 dy=+9.67mm — 봉투-슬롯 Y 여유가
    0.50mm/쪽뿐이라 **19배 초과**다. 목표점 두 개만 맞춰놨어도 그 사이에서 봉투가
    벽에 긁히며 접힌다(§DigitalTwin.md §13-7 "하강 중 어딘가에 끌린다"의 나머지 절반).

    z 만 균등하게 내려가는 웨이포인트마다 IK 를 풀고 그 사이만 조인트 보간한다 —
    41점이면 간격 4.6mm 라 구간 내 부풂은 0.01mm 수준으로 사라진다.
    IK 가 현재 qpos 를 초기추정으로 쓰므로 위에서부터 순차적으로 풀어 같은 분기해에
    머물게 하고, 끝나면 호출 전 qpos 를 복원한다(사이에 scene.step() 을 끼우지
    않으므로 물리는 전혀 진행되지 않는다)."""
    q_saved = _npy(robot.get_dofs_position()).squeeze().copy()
    qs = []
    for z in np.linspace(z0, z1, n_way):
        q = _npy(robot.inverse_kinematics(
            link=link, pos=np.array([target_xy[0], target_xy[1], z]),
            quat=VERTICAL_QUAT, local_point=FINGER_TCP_LOCAL,
            dofs_idx_local=np.arange(6)))[:6]
        qs.append(q)
        robot.set_dofs_position(np.concatenate([q, q_saved[6:]]))
    robot.set_dofs_position(q_saved)
    return np.stack(qs)


def _prepare_robot_mjcf():
    src_dir = paths.ROBOTS_DIR
    tmp_dir = tempfile.mkdtemp(prefix="m0609_fw_v2_")
    for root_dir, _, files in os.walk(src_dir):
        rel = os.path.relpath(root_dir, src_dir)
        dst_dir = os.path.join(tmp_dir, rel) if rel != "." else tmp_dir
        os.makedirs(dst_dir, exist_ok=True)
        for f in files:
            shutil.copy2(os.path.join(root_dir, f), os.path.join(dst_dir, f))
    dst = os.path.join(tmp_dir, "m0609_rg2_v2_patched.xml")
    tree = ET.parse(ROBOT_MJCF)
    root = tree.getroot()

    asset = root.find("asset")
    for i, hull_file in enumerate(FLEX_FINGER_HULLS):
        mesh_el = ET.SubElement(asset, "mesh")
        mesh_el.set("name", f"flex_finger_hull_{i:03d}")
        mesh_el.set("file", f"{COACD_DIR_REL}/{hull_file}")

    wb = root.find("worldbody")
    for j in wb.iter("joint"):
        j.attrib.pop("damping", None)
        j.attrib.pop("frictionloss", None)
    for body in wb.iter("body"):
        if body.get("name") in ("f1_flex_finger", "f2_flex_finger"):
            for i in range(len(FLEX_FINGER_HULLS)):
                g = ET.SubElement(body, "geom")
                g.set("type", "mesh")
                g.set("mesh", f"flex_finger_hull_{i:03d}")
                g.set("contype", "1")
                g.set("conaffinity", "1")
                g.set("group", "0")
                g.set("friction", "1.5 0.02 0.001")

    for tag in ("actuator", "equality"):
        el = root.find(tag)
        if el is not None:
            root.remove(el)
    tree.write(dst)
    return dst


def _prepare_seal_colored_bag():
    """실링부(좌우 가장자리)만 다른 색으로 보이는 봉투 mesh 를 준비한다.
    평면 UV 를 구워 OBJ 로 내보내고, 그 UV 에 맞춘 스트라이프 텍스처 이미지를
    함께 반환한다 — (obj_path, texture_rgb_array)."""
    m = tm.load(BAG_STL)  # welded 모델(773 근처, 얼굴 공유) — Genesis 로드와 동일 위상
    v = m.vertices
    u = np.clip((v[:, 0] + BAG_PANEL_HALF_W) / (2 * BAG_PANEL_HALF_W), 0, 1)
    vv = np.clip((v[:, 1] + BAG_PANEL_HALF_H) / (2 * BAG_PANEL_HALF_H), 0, 1)
    m.visual = tm.visual.TextureVisuals(uv=np.stack([u, vv], axis=1))
    obj_path = os.path.join(OUT_DIR, "_bag_seal_uv.obj")
    m.export(obj_path)

    tex_w = 128
    tex = np.tile(np.array(BAG_BODY_COLOR, dtype=np.uint8), (tex_w, tex_w, 1))
    u_axis = np.linspace(0, 1, tex_w)
    seal_frac = SEAL_BAND_WIDTH / (2 * BAG_PANEL_HALF_W)
    seal_cols = (u_axis < seal_frac) | (u_axis > 1 - seal_frac)
    tex[:, seal_cols] = np.array(SEAL_COLOR, dtype=np.uint8)
    return obj_path, tex


def main(use_viewer: bool = False):
    print("=" * 60)
    print(f" Full workflow: tablet drop -> bag catch -> grasp -> lift -> Crusher slot insert (viewer={use_viewer})")
    print("=" * 60)

    crusher_xml = _prepare_crusher_mjcf()
    robot_xml = _prepare_robot_mjcf()
    bag_obj, bag_seal_tex = _prepare_seal_colored_bag()

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning", precision="32")
    patch_fem_vertex_constraints()

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=DT, gravity=(0, 0, -9.81)),
        coupler_options=gs.options.IPCCouplerOptions(
            contact_d_hat=IPC_D_HAT,
            contact_friction_enable=True,
            two_way_coupling=True,
            enable_rigid_rigid_contact=False,
            enable_rigid_ground_contact=False,
            constraint_strength_translation=100.0,
            constraint_strength_rotation=100.0,
        ),
        fem_options=gs.options.FEMOptions(damping=FEM_DAMPING),
        vis_options=gs.options.VisOptions(
            background_color=(0.93, 0.94, 0.96),
            # 1차 조정(ambient 0.35 + key 8.0)이 명암 대비가 거의 없이 밋밋했다는
            # 피드백 반영 — ambient 를 낮추고 key/fill 강도 비율을 벌려 그림자를
            # 다시 살린다(45도 키 라이트는 유지).
            ambient_light=(0.16, 0.16, 0.18),
            lights=[
                {"type": "directional", "dir": (-1, -1, -1), "color": (1.0, 1.0, 1.0), "intensity": 6.0},
                {"type": "directional", "dir": (1, 1, -0.6), "color": (1.0, 1.0, 1.0), "intensity": 1.2},
            ],
        ),
        show_viewer=use_viewer,
    )

    for p in PLATE_POSITIONS:
        scene.add_entity(
            gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
            material=gs.materials.Rigid(coup_type="ipc_only"),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )

    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=True, convexify=True),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
        surface=gs.surfaces.Default(smooth=False),
    )

    # visualization=False: 충돌(안전망)은 유지하되 격자무늬 렌더는 끔 — 씬에는
    # 알루미늄 플레이트만 보이도록(사용자 지시).
    scene.add_entity(gs.morphs.Plane(visualization=False), material=gs.materials.Rigid(coup_type="ipc_only"))
    scene.add_entity(
        gs.morphs.Box(size=SHELF_SIZE, pos=SHELF_POS, fixed=True),
        material=gs.materials.Rigid(coup_type="ipc_only", coup_friction=0.3),
        surface=gs.surfaces.Default(color=(0.75, 0.78, 0.82)),
    )

    scene.add_entity(
        gs.morphs.MJCF(file=FIXTURE_MJCF, pos=FIXTURE_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )
    # 워크셀 정사각형 레이아웃(2026-07-27): 고정장치와 대칭인 나머지 두 플레이트에
    # 회수장치2/석션V1을 정적 소품으로 배치 — 둘 다 아직 로봇과 실제로 상호작용하지
    # 않는 시각 전용 배치(고정장치와 동일한 취급).
    scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY2_MJCF, pos=RECOVERY2_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )
    scene.add_entity(
        gs.morphs.MJCF(file=SUCTION_MJCF, pos=SUCTIONV1_POS, decimate=False),
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint"),
    )

    robot = scene.add_entity(
        gs.morphs.MJCF(file=robot_xml, pos=tuple(ROBOT_OFFSET), decimate=False),
        material=gs.materials.Rigid(
            coup_type="two_way_soft_constraint",
            coup_links=FINGER_LINKS,
            coup_friction=CLOTH_FRICTION,
        ),
    )

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=bag_obj, scale=BAG_SCALE, pos=BAG_POS, euler=BAG_EULER),
        # color= 대신 실링부 스트라이프가 구워진 UV 텍스처를 사용(§_prepare_seal_colored_bag).
        surface=gs.surfaces.Default(opacity=0.55, roughness=0.9, double_sided=True,
                                     diffuse_texture=gs.textures.ImageTexture(image_array=bag_seal_tex)),
    )

    # SKIP_TABLET=1: 정제를 빼고 **봉투(FEM.Cloth) + IPC 커플러**만 격리 검증한다.
    # utills/primitive_tablet_generator.py 의 TetGen 우회 몽키패치가 Genesis 1.3.1
    # 시그니처 변경으로 깨져 있어(docs/DigitalTwin.md 조합11 추가 발견), 정제가
    # 씬 구성 단계에서 죽으면 봉투/IPC 가 멀쩡한지조차 확인할 수 없다. 정제는
    # 이 검증의 대상이 아니므로 분리할 수 있게 한다.
    if SKIP_TABLET:
        tablet = None
        print("[tablet] SKIP_TABLET=1 — 정제 제외, 봉투(FEM.Cloth)+IPC 만 검증")
    else:
        cap_verts_mm, cap_elems = make_capsule_tets_v2(
            radius_mm=CAP_RADIUS_MM, cyl_height_mm=CAP_CYL_H_MM, n_theta=12, n_cap_rings=4, n_cyl_bands=2,
        )
        tablet = add_analytic_fem_entity(
            scene, key=os.path.join(OUT_DIR, "_analytic_capsule_v2.stl"),
            verts_mm=cap_verts_mm, elems=cap_elems,
            material=gs.materials.FEM.Elastic(
                E=TABLET_E, nu=TABLET_NU, rho=TABLET_RHO,
                friction_mu=TABLET_FRICTION, model="stable_neohookean",
            ),
            scale=1e-3, pos=TABLET_POS,
            surface=gs.surfaces.Default(color=(0.9, 0.9, 0.85), roughness=0.6),
        )

    cam_over = scene.add_camera(res=(1280, 960), pos=OVERVIEW_CAM_POS, lookat=OVERVIEW_CAM_LOOK,
                                fov=48, GUI=False)
    cam_bag = scene.add_camera(res=(960, 720), pos=tuple(np.array(BAG_POS) + BAGCAM_OFFSET),
                               lookat=BAG_POS, fov=40, GUI=False)
    # 정확한 pos/lookat 은 gap_cx/gap_cy/wall_center_z 계산 후(Phase 7 직전) set_pose 로 확정 — 지금은 placeholder.
    cam_side = scene.add_camera(res=(960, 720), pos=OVERVIEW_CAM_POS, lookat=OVERVIEW_CAM_LOOK,
                                fov=45, GUI=False)

    # 계측은 full_workflow_rigid.py 와 **같은 경계**로 잰다(빌드 / 스텝+인코딩).
    # 그래야 두 방식의 수치를 같은 자로 잰 값으로 비교할 수 있다 — 예전 "16분
    # 18초"는 스크립트가 아니라 사람이 벽시계로 잰 값이라 build/steps 분리가
    # 없었고 Genesis 버전·캐시 상태도 기록이 없었다.
    import time as _time
    print("\n[build] scene.build() 시작...")
    _t_build = _time.time()
    scene.build(n_envs=0)
    _build_s = _time.time() - _t_build
    _t_steps = _time.time()
    # 스텝 수는 런타임에 센다. N_* 를 더하는 정적 합계는 위상이 하나 늘 때마다
    # (예: N_ABOVE_SETTLE) 조용히 어긋나고, trim 은 애초에 가변 회차다.
    _step_n = [0]
    print(f"[build] 성공 ({_build_s:.1f}s)")

    # ── 봉투 형상 고정: 바닥+양측면(입구는 자유) — §docstring 참고 ───────────
    # BAG_EULER 가 (90,0,90)으로 바뀌면서 폭(측면 고정 대상) 축이 world X->Y
    # 로 이동(높이=Z 는 불변) — by 기준으로 side_mask 를 잡는다.
    bag_pos0 = _npy(bag.get_state().pos).squeeze()
    by, bz = bag_pos0[:, 1], bag_pos0[:, 2]
    bag_bottom_mask = bz < bz.min() + 0.012
    bag_side_mask = (by < by.min() + 0.008) | (by > by.max() - 0.008)
    bag_fixed_idx = np.where(bag_bottom_mask | bag_side_mask)[0]
    bag.set_vertex_constraints(verts_idx_local=bag_fixed_idx.tolist(), is_soft_constraint=False)
    print(f"[bag] shape 고정: {len(bag_fixed_idx)}/{len(bz)} 정점(바닥+양측면), 입구는 자유")

    # ── Crusher 슬롯 위치 계산 ───────────────────────────────────────────────
    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])
    gap_cx = (gap_lo_x + gap_hi_x) / 2.0
    gap_width = gap_hi_x - gap_lo_x
    y_lo = max(wb_lo[1], wl_lo[1]); y_hi = min(wb_hi[1], wl_hi[1])
    gap_cy = (y_lo + y_hi) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    # L1_Wall1_1 = 포켓 바닥 플레이트(Crusher_Samplebag.py 주석: "L1_Wall1_1 은
    # 바닥 플레이트"). gap 슬릿 중심(gap_cx,gap_cy)은 봉투가 "통과하는" 위치일
    # 뿐, 포켓 바닥의 실제 중심과는 다르다(실측: -38mm 가량 로봇 반대쪽으로
    # 치우쳐 있음) — 사용자 지시대로 봉투 하단 목표를 포켓 바닥 중심으로 잡는다.
    w1_lo, w1_hi = crusher_mesh_world_aabb("L1_Wall1_1")
    pocket_cx = (w1_lo[0] + w1_hi[0]) / 2.0
    pocket_cy = (w1_lo[1] + w1_hi[1]) / 2.0
    print(f"[slot] gap_cx={gap_cx:.4f} gap_cy={gap_cy:.4f} gap_width={gap_width*1000:.1f}mm wall_top_z={wall_top_z:.4f}")
    print(f"[slot] pocket(L1_Wall1_1) center=({pocket_cx:.4f},{pocket_cy:.4f})")

    crusher_joints = {j.name: j for j in crusher.joints if j.name}
    def _scalar_dof(name):
        d = crusher_joints[name].dofs_idx_local
        return d[0] if isinstance(d, (list, tuple, np.ndarray)) else d
    crank_dof = _scalar_dof(CRANK_JOINT)
    wall_dof = _scalar_dof(WALL_JOINT)
    crusher.set_dofs_kp(np.array([CRANK_KP]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kv(np.array([CRANK_KV]), dofs_idx_local=[crank_dof])
    crusher.set_dofs_kp(np.array([WALL_KP]), dofs_idx_local=[wall_dof])
    crusher.set_dofs_kv(np.array([WALL_KV]), dofs_idx_local=[wall_dof])

    gripper_link = robot.get_link("gripper_body")
    left_link = robot.get_link(FINGER_LINKS[0])

    q_grasp, q_lift = Q_GRASP, Q_LIFT
    robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))

    vp0 = _npy(bag.get_state().pos).squeeze()
    d_to_mid = np.linalg.norm(vp0 - FINGER_MID, axis=1)
    grip_idx = np.where(d_to_mid < 0.020)[0].astype(int)
    print(f"[bag] grip_strip verts near FINGER_MID: {len(grip_idx)}")

    # Genesis 1.3.1 에서 녹화 API 가 바뀌었다(§docs/DigitalTwin.md 조합11):
    # 파일명/fps 가 start_recording 으로 이동했고 stop_recording() 은 인자를 안 받는다.
    # (조합11 에 기록돼 있던 변경인데 이 파일엔 반영이 안 돼 있어 실행 끝에서 죽었다.)
    cam_over.start_recording(save_to_filename=MP4_OVERVIEW, fps=30)
    cam_bag.start_recording(save_to_filename=MP4_BAGCAM, fps=30)
    cam_side.start_recording(save_to_filename=MP4_SIDE, fps=30)

    def _bag_com():
        p = _npy(bag.get_state().pos).squeeze()
        return p.mean(axis=0)

    def _bag_extent():
        """(width_y, height_z, bottom_z) — Y 스윕 판정용(사용자 지시, 2026-07-23)."""
        p = _npy(bag.get_state().pos).squeeze()
        return float(p[:, 1].max() - p[:, 1].min()), float(p[:, 2].max() - p[:, 2].min()), float(p[:, 2].min())

    def _bag_tilt():
        """봉투 높이축이 world +Z 에서 몇 도 기울었나(2026-08-14).

        AABB 로는 자세를 못 읽는다(§13-7 — 회전이 섞이면 Z-extent 가 실치수를
        넘어버림). rigid 모드는 강체 quat 으로 직접 쟀지만 FEM 은 점군이라,
        **최하단 10% 정점 중심 -> 최상단 10% 정점 중심** 벡터를 높이축으로 쓴다.
        슬롯 진입 실패의 지배적 모드가 "기운 채로 내려가 벽 윗면에 얹힘"이라
        (rigid 실측: above 종료 tilt 28.8deg) FEM 쪽도 같은 지표가 필요하다."""
        p = _npy(bag.get_state().pos).squeeze()
        k = max(1, len(p) // 10)
        order = np.argsort(p[:, 2])
        axis = p[order[-k:]].mean(axis=0) - p[order[:k]].mean(axis=0)
        n = np.linalg.norm(axis)
        if n < 1e-9:
            return float("nan")
        return float(np.degrees(np.arccos(np.clip(axis[2] / n, -1.0, 1.0))))

    def _tablet_z():
        if tablet is None:
            return float("nan")
        p = _npy(tablet.get_state().pos).squeeze()
        return float(p[:, 2].mean())

    def _finger_z():
        return float(_npy(left_link.get_pos()).squeeze()[2])

    def render_cams():
        cam_over.render()
        bc = _bag_com()
        cam_bag.set_pose(pos=tuple(bc + BAGCAM_OFFSET), lookat=tuple(bc), up=(0, 0, 1))
        cam_bag.render()
        cam_side.render()

    def run_arm(name, q0, q1, f0, f1, n, crank_q=None, wall_q=None, trace=False):
        for k in range(n):
            s = ease((k + 1) / n)
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                crusher.control_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
            scene.step()
            _step_n[0] += 1
            render_cams()
            if trace and k % 40 == 0:
                print(f"    [{name} k={k:4d}] tablet_z={_tablet_z()*1e3:+.2f}mm bag_com={_bag_com()}")
        bc = _bag_com()
        print(f"[phase] {name:8s} @done  bag_com={bc}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm  tilt={_bag_tilt():.1f}deg  "
              f"bag_bottom={_bag_extent()[2]:.4f}")

    def run_arm_path(name, q_way, f, n, crank_q=None, wall_q=None, trace=False):
        """웨이포인트 열을 따라 구동 — run_arm 의 카테시안 직선판(§solve_descent_waypoints)."""
        m = len(q_way) - 1
        for k in range(n):
            u = ease((k + 1) / n) * m
            i = min(int(u), m - 1)
            q = q_way[i] + (q_way[i + 1] - q_way[i]) * (u - i)
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                crusher.control_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
            scene.step()
            _step_n[0] += 1
            render_cams()
            if trace and k % 40 == 0:
                print(f"    [{name} k={k:4d}] tablet_z={_tablet_z()*1e3:+.2f}mm bag_com={_bag_com()}")
        bc = _bag_com()
        print(f"[phase] {name:8s} @done  bag_com={bc}  finger_z={_finger_z():.4f}  "
              f"tablet_z={_tablet_z()*1e3:+.2f}mm  tilt={_bag_tilt():.1f}deg  "
              f"bag_bottom={_bag_extent()[2]:.4f}")

    # ── Phase 0: prep — 크랭크 -180도, Left_Wall 개방(슬롯 준비, 사용자 지시) ──
    print(f"\n[phase] 0 prep ({N_PREP*DT:.1f}s) — 크랭크 0->{CRANK_START_Q:+.3f}rad(-180deg), "
          f"Left_Wall 0->{WALL_OFFSET*1000:+.0f}mm(개방)")
    for k in range(N_PREP):
        s = (k + 1) / N_PREP
        crusher.control_dofs_position(np.array([CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET * s]), dofs_idx_local=[wall_dof])
        robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
        scene.step()
        _step_n[0] += 1
        render_cams()
    cq = _npy(crusher.get_dofs_position())[crank_dof]
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] prep     @done  crank={cq:+.3f}rad  wall={wq*1000:+.2f}mm")

    # ── Phase 1-6: 정제 낙하 -> 봉투 파지 -> 리프트 (tablet_bag_grasp_pipeline.py 동일) ──
    run_arm("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # 정제가 안전하게 들어간 뒤 형상 고정 해제 — 이후 grasp/lift 는 순수 마찰로 진행.
    bag.remove_vertex_constraints()
    print("[bag] shape 고정 해제 — 이제부터 순수 마찰 파지")

    run_arm("close", q_grasp, q_grasp, FING_OPEN, FING_CLOSE, N_CLOSE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("grasp", q_grasp, q_grasp, FING_CLOSE, FING_CLOSE, N_GRASP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("lift", q_grasp, q_lift, FING_CLOSE, FING_CLOSE, N_LIFT,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
    run_arm("hold", q_lift, q_lift, FING_CLOSE, FING_CLOSE, N_HOLD,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # Y 스윕 판정 기준선(삽입 시도 전, 사용자 지시 2026-07-23) — 이후 결과와 비교.
    baseline_width_y, baseline_height_z, _ = _bag_extent()

    # ── Phase 7: above — 슬롯 바로 위로 이동 (IK) ──────────────────────────
    # **버그 발견(2026-07-16, 사용자 지적)**: 지금까지 IK 타깃 링크로 썼던
    # `gripper_body`는 실제 손가락(=봉투가 매달린 지점)과 무려 140mm(z),
    # 34mm(x) 어긋나 있었다(FK 실측 확인) — gripper_body 는 손목 브라켓
    # 쪽이고, RG2 내부 링크(moment_arm/truss_arm)를 거쳐 손가락이 그보다 한참
    # 아래에 붙는다. `insert_z=wall_top_z`로 gripper_body 를 명령했더니 실제
    # 손가락은 140mm 아래(z<0, 포켓 바닥까지)로 내려갔던 것 — 이전 라운드들의
    # "목표에 도달 못 함/벽에 막힘"으로 보였던 현상 상당수가 사실 이 링크
    # 오프셋 때문이었다. **IK 타깃 링크를 `left_link`(손가락)로 교체**해
    # 해결(격리 검증: 오차 0.19mm, Crusher 접촉 0건).
    #
    # (2026-07-16 9차: Q_GRASP/Q_LIFT 의 wrist 가 이미 0으로 바뀌어서 — 위
    # BAG_EULER 수정과 함께 — 더 이상 "손목만 0으로 되돌리는" 임시 조회가
    # 필요 없다.)
    # 2026-07-24: 예전엔 여기서 현재 손가락 quat 을 그대로 캡처해 썼는데, 그
    # quat 자체가 13.8도 기울어져 있었다(Q_GRASP/Q_LIFT 가 IK가 아니라
    # 손튜닝값이었기 때문). Q_GRASP/Q_LIFT 를 완전 수직으로 재계산한 지금은
    # VERTICAL_QUAT 상수를 그대로 목표로 쓴다(위 hold 종료 시점 실제 quat과
    # 사실상 동일 - IK가 그 값으로 수렴하도록 풀었으므로).
    q_insert_quat = VERTICAL_QUAT

    # ── 목표 위치(2026-07-16 6차, docs/Crusher.md §5·§11-5 + 사용자 지시) ────
    # X/Y: 로봇은 봉투를 포켓 깊숙이 밀어넣을 필요 없이 gap 근처까지만
    # 옮기면 된다(Left_Wall 이 이후 실제로 클램프하는 기구 — §조합9 후속).
    # Z: "벽 중앙에서 10cm 위"(사용자 지시) — wall_center_z(포켓 상/하단의
    # 세로 중점) + 100mm 를 **손가락** 목표로 직접 사용.
    wall_center_z = (wall_top_z + wb_lo[2]) / 2.0
    above_z = wall_top_z + 0.20
    # 2026-08-14: 목표 깊이를 "핑거-Crusher 충돌 경계"(구 0.052)가 아니라
    # **사용자 목표(봉투 최하단 = wall_center_z)** 에서 직접 역산한다.
    insert_z = wall_center_z + BAG_HANG_BELOW_FINGER
    # Y_OFFSET(사용자 지시, 2026-07-23): gap_cy 추정 위치 근방에서 Y 스윕 검증용.
    # BAG_DY_FROM_FINGER 를 빼는 것이 이번 라운드의 핵심 수정 — 슬롯에 정렬돼야
    # 하는 것은 핑거가 아니라 봉투 몸체다(상수 정의부 주석 참고).
    target_xy = np.array([gap_cx, gap_cy - BAG_DY_FROM_FINGER + Y_OFFSET])
    print(f"[slot] wall_center_z={wall_center_z:.4f}  insert_z(finger)={insert_z:.4f}  "
          f"hang={BAG_HANG_BELOW_FINGER*1000:.0f}mm")
    print(f"[slot] 봉투중심 보정 dy={BAG_DY_FROM_FINGER*1000:+.1f}mm -> finger_y={target_xy[1]:.4f} "
          f"(봉투중심 y={target_xy[1]+BAG_DY_FROM_FINGER:.4f} = gap_cy {gap_cy:.4f})")
    print(f"[sweep] Y_OFFSET_MM={Y_OFFSET_MM:+.1f}mm -> target_xy={target_xy}  (gap_cy={gap_cy:.4f})")

    # 사이드뷰 카메라 확정 — gap(X, "슬롯 두께"=gap_width) vs 삽입 깊이(Z) 단면을
    # -Y 에서 +Y 로 정면으로 잡아, above->insert 하강 중 봉투가 gap 폭 안에서
    # 내려가는지(걸림/충돌) 육안으로 바로 판단 가능하게 한다(사용자 요청, 2026-07-27).
    side_z = (above_z + wall_center_z) / 2.0
    side_cam_pos = (gap_cx, gap_cy + SIDECAM_Y_OFFSET, side_z)
    side_cam_look = (gap_cx, gap_cy, side_z)
    cam_side.set_pose(pos=side_cam_pos, lookat=side_cam_look, up=(0, 0, 1))
    print(f"[cam] side view 고정: pos={side_cam_pos} look={side_cam_look} "
          f"(gap_width={gap_width*1000:.1f}mm, z범위 above={above_z:.4f}~insert={insert_z:.4f})")

    # 2026-07-24: local_point=FINGER_TCP_LOCAL 추가 — 이전엔 f1(left_link)
    # 원점 하나만 타겟이라 f1-f2 진짜 중앙과 20mm 어긋난 채로 목표를 풀고
    # 있었다(위 Q_GRASP/Q_LIFT 주석 참고). 이제 진짜 중앙이 target_xy/above_z/
    # insert_z 에 정확히 도달하도록 푼다.
    target_above = np.array([target_xy[0], target_xy[1], above_z])
    qpos_above = _npy(robot.inverse_kinematics(
        link=left_link, pos=target_above, quat=q_insert_quat, local_point=FINGER_TCP_LOCAL,
        dofs_idx_local=np.arange(6)))[:6]
    print(f"\n[ik] above-slot target={target_above}  arm_q={qpos_above}")
    run_arm("above", q_lift, qpos_above, FING_CLOSE, FING_CLOSE, N_ABOVE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    # 하강 전 스윙 감쇠(2026-08-14) — 팔은 정지, 봉투만 가라앉힌다.
    run_arm("aboveset", qpos_above, qpos_above, FING_CLOSE, FING_CLOSE, N_ABOVE_SETTLE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # ── Phase 8: insert — 슬롯 안까지 카테시안 직선 하강 ─────────────────────
    # 2026-08-14: 조인트각 선형보간은 중간에서 dy=+9.67mm 부푼다(Y 여유 0.5mm/쪽) —
    # z 만 내려가는 웨이포인트 IK 로 바꾼다(§solve_descent_waypoints).
    q_way = solve_descent_waypoints(robot, left_link, target_xy, above_z, insert_z)
    qpos_insert = q_way[-1]
    print(f"[ik] insert target={np.array([target_xy[0], target_xy[1], insert_z])}  "
          f"웨이포인트 {len(q_way)}개(카테시안 직선)  arm_q={qpos_insert}")
    run_arm_path("insert", q_way, FING_CLOSE, N_INSERT,
                 crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle2", qpos_insert, qpos_insert, FING_CLOSE, FING_CLOSE, N_SETTLE2,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # ── Phase 8b: trim — 봉투 최하단을 wall_center_z 에 맞추는 1자유도 폐루프 ──
    # 자유 매달림 길이는 예측이 된다(실측 hold 83.0mm ≈ 설계 82mm). 예측이 안 되는
    # 건 **하강 중 봉투가 슬롯 벽에 스치며 되말려 올라가는 양**이다 — 같은 실행에서
    # above 82.6mm -> settle2 71.2mm 로 11mm 가 사라졌다. 이건 천의 접촉 이력에
    # 달린 값이라 상수로 못 박는다.
    # 그런데 맞춰야 하는 건 스칼라 하나(bag_bottom_z)이고 핑거 z 와 거의 1:1 이므로,
    # 남은 오차만큼 더 내려주는 폐루프로 닫는다 — 정책을 학습할 구조가 아니다.
    # 하한은 §DigitalTwin.md §13-7 의 핑거-Crusher 충돌 경계(wall_center+52mm).
    INSERT_Z_FLOOR = wall_center_z + 0.052
    TRIM_TOL, TRIM_GAIN, N_TRIM = 0.002, 0.8, 80
    # **기본 OFF (2026-08-14 실측).** trim 은 봉투를 슬롯 안으로 더 밀어넣는데,
    # 그 과정에서 IPC 접촉이 퇴화(degenerate)해 **솔버가 기하급수적으로 느려지고
    # 결국 죽는다**: 같은 씬이 trim 없이 4~5분인데 trim 4라운드에서 5시간 17분을
    # 쓰고 libuipc CCD 어서션(`toi > 0.0f failed`, toi=-5.4e-26)으로 abort 했다.
    # 원인은 §14-6 과 같다 — 봉투가 마찰로 걸린 상태에서 계속 누르니 천이 자기
    # 자신과 벽 사이에서 짓눌린다(tilt 12.8 -> 16.5 -> 22.2 -> 23.9deg 단조 증가).
    # 즉 trim 은 "밀어넣기의 한계"를 재는 진단 도구지 상시 켜둘 보정이 아니다.
    # 파지 위치 비교처럼 조건을 맞춰야 하는 실험에서는 반드시 0 으로 둔다.
    TRIM_ROUNDS = int(os.environ.get("TRIM_ROUNDS", "0"))
    # **발산 가드(2026-08-14 실측).** 핑거 z ↔ bag_bottom 커플링은 1:1 이 아니라
    # ~0.36 이다 — 봉투가 슬롯 벽에 마찰로 걸려 있어서, 더 내리면 따라 내려가는
    # 게 아니라 안에서 눌려 접힌다. 실제로 r3 에서 오차가 7.8 -> **28.3mm** 로
    # 튀었다. 그래서 "더 내리면 나아진다"를 가정하지 않고, 나빠지면 즉시 최적점
    # 으로 되돌아가 멈춘다. 남는 오차는 밀어넣기가 아니라 파지 위치/자세 쪽에서
    # 풀어야 한다는 신호다(§14-6, GRIP_OFFSET_MM 스윕).
    TRIM_DIVERGE = 0.002
    cur_z, q_cur = insert_z, qpos_insert
    best_err, best_z, best_q = abs(_bag_extent()[2] - wall_center_z), cur_z, q_cur
    if TRIM_ROUNDS == 0:
        print(f"[trim] TRIM_ROUNDS=0 — 생략(기본값). settle2 시점 bag_bottom 오차 "
              f"{(best_err)*1e3:+.1f}mm 를 그대로 결과로 쓴다.")
    for r in range(TRIM_ROUNDS):
        err = _bag_extent()[2] - wall_center_z
        if abs(err) < best_err:
            best_err, best_z, best_q = abs(err), cur_z, q_cur
        elif abs(err) > best_err + TRIM_DIVERGE:
            print(f"[trim] r{r}: 오차가 {best_err*1e3:.1f} -> {abs(err)*1e3:.1f}mm 로 악화 "
                  f"— 봉투가 슬롯 안에서 눌려 접히는 중. 최적점(finger_z={best_z:.4f})으로 복귀 후 종료")
            run_arm("trimback", q_cur, best_q, FING_CLOSE, FING_CLOSE, N_TRIM,
                    crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
            cur_z, q_cur = best_z, best_q
            break
        if abs(err) < TRIM_TOL:
            print(f"[trim] r{r}: 오차 {err*1e3:+.1f}mm < {TRIM_TOL*1e3:.0f}mm — 수렴, 종료")
            break
        new_z = float(np.clip(cur_z - TRIM_GAIN * err, INSERT_Z_FLOOR, above_z))
        if abs(new_z - cur_z) < 1e-4:
            print(f"[trim] r{r}: 오차 {err*1e3:+.1f}mm 남았으나 핑거가 충돌 하한"
                  f"({INSERT_Z_FLOOR:.4f})에 걸림 — 종료")
            break
        print(f"[trim] r{r}: bag_bottom 오차 {err*1e3:+.1f}mm -> finger_z {cur_z:.4f}->{new_z:.4f}")
        q_next = _npy(robot.inverse_kinematics(
            link=left_link, pos=np.array([target_xy[0], target_xy[1], new_z]),
            quat=q_insert_quat, local_point=FINGER_TCP_LOCAL,
            dofs_idx_local=np.arange(6)))[:6]
        run_arm(f"trim{r}", q_cur, q_next, FING_CLOSE, FING_CLOSE, N_TRIM,
                crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)
        cur_z, q_cur = new_z, q_next
    print(f"[trim] 최종 finger_z={cur_z:.4f}  (초기 {insert_z:.4f}, 하한 {INSERT_Z_FLOOR:.4f})")
    qpos_insert = q_cur

    # ── Phase 9: clamp — Left_Wall(Motor2, Rack&Pinion) 닫아 실링부 고정 ─────
    # docs/Crusher.md §11-5: 래칫/락 없이 모터를 계속 구동해 강하게 고정하는
    # 방식 — Crusher_Samplebag.py 의 CLAMP_TARGET(-5mm) 재사용, WALL_OFFSET
    # (+6mm)에서 총 11mm 이동해 6mm 두께 봉투를 압착.
    print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.1f}s) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
          f"{CLAMP_TARGET*1000:+.1f}mm (실링부 압착)")
    for k in range(N_CLAMP):
        s = (k + 1) / N_CLAMP
        wq = WALL_OFFSET + (CLAMP_TARGET - WALL_OFFSET) * s
        crusher.control_dofs_position(np.array([wq]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        robot.set_dofs_position(np.concatenate([qpos_insert, [FING_CLOSE] * 6]))
        scene.step()
        _step_n[0] += 1
        render_cams()
    wq_final = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm  bag_com={_bag_com()}")

    # ── Phase 10: release — 그리퍼 개방, 고정은 이제 Left_Wall 이 담당 ───────
    run_arm("release", qpos_insert, qpos_insert, FING_CLOSE, FING_OPEN, N_RELEASE,
            crank_q=CRANK_START_Q, wall_q=CLAMP_TARGET, trace=True)

    # ── Y 스윕 PASS/FAIL 판정(사용자 지시, 2026-07-23) ───────────────────────
    # 실제 그리퍼 마찰 파지 경로라 slot_fit_check.py 의 carrier 판정보다 여유를
    # 둔다: 도달 오차 30mm, 폭/높이는 삽입 시도 직전(hold 시점) 값 대비 상대 비교
    # (탄성 흔들림은 정상, 붕괴/과신전만 이상 신호).
    # 2026-08-14: 30mm 는 깊이 72mm 포켓에서 "어디든 들어가면 PASS" 수준이라,
    # 실제로는 봉투가 벽 윗면에 접힌 채 26.0mm 오차로 PASS 가 찍히고 있었다
    # (`_run_reposition_check.log`). 사용자 목표("봉투 최하단 = Wall_1 중간
    # 높이")를 실제로 판정하려면 이 창이 목표 정밀도여야 한다.
    REACH_TOL = 0.015
    WIDTH_TOL_FRAC = 0.6
    HEIGHT_TOL_FRAC = 2.0
    final_width_y, final_height_z, final_bottom_z = _bag_extent()
    reached = abs(final_bottom_z - wall_center_z) < REACH_TOL
    width_ok = final_width_y > baseline_width_y * WIDTH_TOL_FRAC
    height_ok = final_height_z < baseline_height_z * HEIGHT_TOL_FRAC
    verdict = "PASS" if (reached and width_ok and height_ok) else "FAIL"
    reasons = []
    if not reached:
        reasons.append(f"미도달(bag_bottom_z={final_bottom_z:.4f} vs wall_center_z={wall_center_z:.4f}, "
                        f"diff={abs(final_bottom_z-wall_center_z)*1000:.1f}mm>{REACH_TOL*1000:.0f}mm)")
    if not width_ok:
        reasons.append(f"폭 붕괴(현재 {final_width_y*1000:.1f}mm < 기준 {baseline_width_y*1000:.1f}mm의 "
                        f"{WIDTH_TOL_FRAC*100:.0f}%, 걸림/구겨짐 의심)")
    if not height_ok:
        reasons.append(f"비정상 신전(높이 {final_height_z*1000:.1f}mm > 기준 {baseline_height_z*1000:.1f}mm의 "
                        f"{HEIGHT_TOL_FRAC*100:.0f}%)")
    reason_str = "; ".join(reasons) if reasons else "삽입 성공, 걸림/붕괴 없음"
    print(f"\n[RESULT] GRIP_OFFSET_MM={GRIP_OFFSET_MM:+.0f}mm  Y_OFFSET_MM={Y_OFFSET_MM:+.1f}mm  "
          f"verdict={verdict}  ({reason_str})")
    print(f"[RESULT] final_bottom_z={final_bottom_z:.4f}  wall_center_z={wall_center_z:.4f}  "
          f"오차={(final_bottom_z-wall_center_z)*1000:+.1f}mm(+면 얕음/-면 깊음)")
    print(f"[RESULT] width_y={final_width_y*1000:.1f}mm(baseline {baseline_width_y*1000:.1f}mm)  "
          f"height_z={final_height_z*1000:.1f}mm(baseline {baseline_height_z*1000:.1f}mm)")

    cam_over.stop_recording()
    cam_bag.stop_recording()
    cam_side.stop_recording()
    print(f"\n[saved] overview -> {MP4_OVERVIEW}")
    print(f"[saved] bagcam   -> {MP4_BAGCAM}")
    print(f"[saved] sideview -> {MP4_SIDE}")

    _steps_s = _time.time() - _t_steps
    _n = _step_n[0]
    print(f"\n[timing] build={_build_s:.1f}s  steps={_steps_s:.1f}s "
          f"({_n} steps, {_steps_s / _n * 1e3:.1f}ms/step)  "
          f"합계={_build_s + _steps_s:.1f}s")
    print("완료.")


if __name__ == "__main__":
    main(use_viewer=os.environ.get("VIEWER") == "1")
