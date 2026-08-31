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
# CLAMP_ONLY=1: 팔 시퀀스(Phase 1~8b, 2,430 스텝 ≈ 4~5분)를 건너뛰고 봉투를
# 슬롯에 직접 놓은 뒤 Phase 9 만 돌린다. 압착 파라미터를 반복 스윕할 때 매번
# 붙는 앞단을 없애기 위한 것이다(사용자 지시 2026-08-31).
# 그리퍼 파지 대신 **봉투 상단을 정점 구속으로 매단다** — 파지 자체는 §17 에서
# 이미 검증됐고 여기서 재려는 것은 벽의 압착뿐이다. 대신 "그리퍼를 놓으면
# 떨어지는가"는 이 모드로 판정할 수 없다(그건 풀 시퀀스로 봐야 한다).
CLAMP_ONLY = os.environ.get("CLAMP_ONLY", "0") == "1"
CLAMP_ONLY_PIN = os.environ.get("CLAMP_ONLY_PIN", "0") == "1"
# NO_VIDEO=1: 카메라 렌더/녹화를 통째로 끈다. 파라미터 스윕처럼 **수치만** 필요한
# 런에서 카메라 3대를 매 스텝 돌리는 비용을 없앤다. 영상이 결과물인 런에서는 끄면 안 된다.
NO_VIDEO = os.environ.get("NO_VIDEO", "0") == "1"

# ── 파지 위치 스윕(2026-08-14, 사용자 지시) ─────────────────────────────────
# 봉투 로컬 폭축(BAG_EULER 회전으로 world Y 에 매핑) 상에서 그리퍼가 무는 지점:
#     0    = 봉투 폭 정중앙(입구 한가운데를 뭄) — 봉투가 핑거 바로 아래 매달린다
#   -28mm  = 세로 실링 가장자리(§5-2 에서 검증된 현행 파지)
# 이 값이 곧 핑거 TCP ↔ 봉투 중심의 Y 오프셋(부호 반전)이라, 슬롯 정렬 보정
# (BAG_DY_FROM_FINGER)과 above/insert IK 타깃이 전부 자동으로 따라온다.
# 가장자리를 물수록 봉투에 굽힘 모멘트가 걸려 삽입 중 쐐기처럼 끼는 것으로
# 의심돼(§14-6 trim 발산) 정중앙~가장자리를 스윕해 비교한다.
GRIP_OFFSET_MM = float(os.environ.get("GRIP_OFFSET_MM", "-28"))

# OUT_DIR 은 **런마다 분리할 수 있어야 한다.** `_bag_seal_uv.obj`,
# `_analytic_capsule_v2.stl`, `material_0.png` 는 매 런이 새로 굽는 중간 산출물인데
# 경로가 고정이라, 병렬로 두 런을 돌리면 한쪽이 쓰는 중에 다른 쪽이 읽어
# `ValueError: need at least one array to concatenate` 로 죽는다(2026-08-31 실측:
# 스윕 레인 2개에서 전 런이 초 단위로 실패). 프로세스를 중간에 죽이면 잘린 파일이
# 남아 이후 런까지 전부 오염된다. 스윕은 RUN_TAG 로 런마다 다른 디렉터리를 준다.
_RUN_TAG = os.environ.get("RUN_TAG", "")
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       f"RESULT{('_' + _RUN_TAG) if _RUN_TAG else ''}")
os.makedirs(OUT_DIR, exist_ok=True)
# 케이스별 하위 디렉토리(사용자 지시) — 영상이 케이스마다 분리돼 쌓인다.
CASE_DIR = os.path.join(OUT_DIR, f"grip{GRIP_OFFSET_MM:+03.0f}mm")
os.makedirs(CASE_DIR, exist_ok=True)
_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
_TAG = f"grip{GRIP_OFFSET_MM:+.0f}mm_yoff{Y_OFFSET_MM:+.1f}mm_{_TS}"
MP4_OVERVIEW = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_overview.mp4")
MP4_BAGCAM = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_bagcam.mp4")
MP4_SIDE = os.path.join(CASE_DIR, f"full_workflow_{_TAG}_sideview.mp4")

# ── 실설계 배치 (Hardware_setup.step / Base_ver2.step 실측, 2026-08-28) ─────
# STEP 어셈블리를 파싱해 각 조립품의 배치를 얻고, **동일 부품의 bbox 를 MJCF 와
# 대조**해 MJCF 원점 -> 어셈블리 좌표 변환을 실측했다(오프셋 lo/hi 가 일치하면
# 회전 성분 0). 월드 원점은 **알루미늄 판 상면 중심** = 어셈블리 (-375, 400, 30)mm
# 으로 잡는다 — 실물 장착 기준면이라 판 상면이 z=0 이 되어 해석이 깔끔하다.
#
#   조립품      MJCF->어셈블리 평행이동(mm)   회전   검증
#   Crusher     (-115.20,  64.72,  30.57)   없음   1_Wall1/2_Wall3 표준편차 0
#   고정장치     (-315.00,  52.50,  30.00)   없음   MotorDriver     표준편차 0
#   회수장치2    (-378.99,  65.78, 182.52)   없음   M_Top/F_Top/RachetGear 0.002
#   석션V1      (-366.55, 202.50,  30.00)   Z+90   Hbeam_L/B/M, Dummy 로 확인
#   로봇 M0609   (-600.00, 650.00,  30.00)   없음   STEP BASE_M0609 배치 (미대조)
#
# **Crusher 는 EULER 0 이다** — MJCF 의 모든 geom 이 quat="0.5 0.5 0.5 0.5"
# (X->+Y, Y->+Z, Z->+X)를 이미 갖고 있고 이것이 STEP 의 Crusher 자세 (90,0,90)과
# 같은 회전이다. 여기에 (90,0,90)이나 기존 (0,0,90)을 또 주면 이중 회전이 된다.
# 검산: 이 변환에서 MJCF 원점의 z 가 판 상면 +0.57mm — 판 위에 정확히 얹힌다.
LAYOUT_FROM_STEP = os.environ.get("LAYOUT_FROM_STEP", "0") == "1"
LAYOUT_ONLY = os.environ.get("LAYOUT_ONLY", "0") == "1"      # 배치만 렌더하고 종료
if LAYOUT_ONLY:
    LAYOUT_FROM_STEP = True

# ── Crusher + plate (scene_setup.py 동일) ───────────────────────────────────
CRUSHER_SRC_XML = paths.MJCF_MAIN
CRUSHER_POS = (0.2598, -0.3353, 0.00057) if LAYOUT_FROM_STEP else (0.0, 0.0, 0.0)
CRUSHER_EULER = (0.0, 0.0, 0.0) if LAYOUT_FROM_STEP else (0.0, 0.0, 90.0)
PLATE_PATH = paths.ALUMINUM_PLATE
PLATE_POSITIONS = [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]
# 실판: 750 x 800 x 30mm, D6.0 나사구멍 240개(15x16 격자, 피치 50mm, 여백 25mm).
# 구멍은 IPC 충돌비용만 올리고 접촉에 기여하지 않으므로 **충돌은 box primitive**로
# 두고 구멍은 시각/치수 근거로만 남긴다(Twin.md §5: 사전정의 primitive 가 유리).
PLATE_SIZE = (0.750, 0.800, 0.030)
PLATE_HOLE_D, PLATE_HOLE_PITCH, PLATE_HOLE_MARGIN = 0.006, 0.050, 0.025

WALL_GEOMS_TO_ENABLE = {"base_link", "L1_Wall1_1", "L1_Wall2_1", "L2_Wall3_1"}
# Left_Wall 압착면 평탄화 — **기본 꺼짐(2026-08-25, 사용자 지적으로 폐기)**.
# 본체 면이 플랜지보다 5mm 물러난 건 결함이 아니라 **impact plate(L9_PLATE_v3_1)의
# 통로**다. 실측: 슬라이더 q=-15/-10/-5mm 에서 플레이트가 그 5mm 구간을 통과하며
# 겹침이 6244 / 12500 / 6256 mm^3 다. 메우면 분쇄가 불가능해진다.
# 상단 플랜지(z 81.43~86.43)는 플레이트 작동 z(24.43~74.43)보다 위라 분쇄를 막지
# 않는다 — 비산 방지 립이다(사용자 확인). 즉 형상은 설계대로 옳다.
# 실험 재현용으로만 남긴다. 자세한 근거는 _add_leftwall_clamp_face() docstring.
LEFTWALL_CLAMP_FACE = os.environ.get("LEFTWALL_CLAMP_FACE", "0") == "1"
L7_LINK3_COM = "0.006 0 -0.005"

# ── Left_Wall 충돌 형상 분리 — 기본 켜짐(2026-08-26) ────────────────────────
# 사용자가 Fusion 에서 Left_Wall 을 본체/플랜지 두 부품으로 쪼개 다시 내보냈다
# (`Crusher_IsaacSim_mjcf`). 그 둘만 가져와 현재 모델 좌표로 옮기고 본체는 CoACD
# 로 더 쪼갠 것이 이 경로다 — 굽는 쪽은 `assets/MJCF/build_leftwall_split.py`.
#
# **버그 수정이 아니라 개선이다.** 소스인 `Crusher_IsaacSim_colored.xml` 은 이미
# 통짜 메시를 contype=0 시각 geom 으로 두고 CoACD hull 11개(group=3)를 충돌로
# 물려 놨다. 즉 단차는 원래도 정확히 재현되고 있었다. 바뀌는 것은:
#
#     충돌 geom 수   11개        -> 6개 (본체 hull 5 + 플랜지 1)
#     부피 충실도    38,847      -> 38,694 mm^3  (101.9% -> 101.5%)
#     플랜지 표현    hull_000    -> verts 8 정확한 box (100.8% -> 100.0%)
#     위치           예전 CAD    -> 새 CAD (뒤의 LEFTWALL_GAP_* 주석, 0.50mm)
#     §18-5 준비     hull 에 섞임 -> 독립 geom 이라 떼어내기 쉬움
#
# 배선이 성립하는 근거: Crusher 는 `convexify=True` 로 실리는데 Genesis 는 MJCF 를
# robot 으로 보고 `decompose_robot_error_threshold`(기본 **inf**)를 쓴다. 임계값이
# 무한대면 `geoms_must_decompose` 가 영원히 False 라 `must_decompose` 가 False 고,
# 그러면 **융합(fusion) 분기가 통째로 스킵된다**(genesis/utils/mesh.py:671).
# 즉 geom 을 나눠 두면 나뉜 채로 유지되고, 이미 볼록한 hull 은 볼록화해도 자기
# 자신이다. 거꾸로 말하면 **통짜 메시 하나로 두면 hull 하나로 뭉개진다** —
# 실부피의 3.6배(38,123 -> 136,890 mm^3)가 되고 늘어난 98,767mm^3 가 하필 좌우
# 세로부재 사이 55mm 개구부, 즉 impact plate 의 분쇄 통로를 메운다(§18-2).
#
#     본체   L2_Left_Wall_body(시각) + hull 5개(충돌)  = 실부피의 101.8%
#     플랜지 L2_Left_Wall_flange     verts 8 정확한 box = 실부피의 100.0%
#
# 플랜지를 별도 **body** 가 아니라 별도 **geom** 으로 둔 이유: 충돌 결과가
# 동일하고(볼록화는 geom 단위) 링크가 늘지 않아 구조 위험이 없다. §18-5 처럼
# 플랜지를 고정 프레임에 매달려면 그때 진짜 body 로 올려야 한다.
LEFTWALL_SPLIT = os.environ.get("LEFTWALL_SPLIT", "1") == "1"
LEFTWALL_BODY_MESH = "L2_Left_Wall_body"
LEFTWALL_FLANGE_MESH = "L2_Left_Wall_flange"

# ── 플랜지를 접촉에서 뺀다 (2026-08-31, 사용자 지시) ────────────────────────
# §18-5 / §19-4 의 결론: 상단 플랜지(z 81.43~86.43)는 **Wall3 상단(80.43)보다
# 위라 마주보는 면이 없다.** 벽을 닫으면 플랜지는 허공을 누르고, 반력이 안 생기니
# 봉투는 눌리는 게 아니라 옆으로 밀린다(clamp 중 Y +3.7mm 실측). 게다가 진짜
# 압착면인 본체면은 5mm 물러나 있어, 본체가 닿기도 전에 플랜지가 먼저 봉투를
# 뭉갠다. 즉 플랜지는 **압착 경로에서 빠져야 할 부품**이다.
#
# §18-5 는 "접촉만 끄는 것은 실재 부품을 없는 셈 치는 것"이라며 Fusion 재export
# 으로 별도 링크 분리를 처방했지만, 그건 CAD 왕복이 필요하다. 그 전에 접촉만
# 떼어 가설을 검증한다(사용자 지시). **렌더에는 그대로 남는다** — 부품이 사라지는
# 게 아니라 충돌에서만 빠진다.
#
# **배선: contype=0 conaffinity=0 이면 IPC 에서도 빠진다.** §19-5 는 "시각 전용
# geom 도 IPC 안에서는 살아 있다"고 적었는데 **그건 틀렸다**(2026-08-31 소스 재확인):
#
#   genesis/engine/entities/rigid_entity/rigid_entity.py:_postprocess_geoms_info
#       is_col = g_info["contype"] or g_info["conaffinity"]
#       -> is_col 이면 cg_infos(link.geoms), 아니면 vg_infos(link.vgeoms)
#   genesis/engine/couplers/ipc_coupler/coupler.py:392
#       for geom in source_link.geoms:      # ← vgeoms 는 여기 안 온다
#
# 커플러 루프에 마스크 **검사**가 없는 것은 맞다. 하지만 그 루프가 도는 `link.geoms`
# 자체가 이미 충돌 geom 만 담고 있어서, contype/conaffinity 를 **둘 다 0** 으로
# 두면 애초에 커플러에 도달하지 않는다. §9 조합9 후속4 의 "비트마스크로 조 충돌
# 제외 -> 효과 없음"과 모순되지 않는다 — 그건 contype/conaffinity 를 0 이 아닌
# 다른 비트로 바꿔 **쌍**만 어긋내려 한 것이라 geom 은 여전히 충돌 geom 이었다.
# 회수장치2 를 needs_coup=False 로 뺀 근거("모든 geom 이 contype=0 이라 커플러가
# 가져갈 충돌 메시가 0개")와도 같은 이야기다.
#
# 기본값은 LAYOUT_FROM_STEP 를 따라간다 — 실배치에서만 빼고, 예전 검증 배치
# (§18 까지의 -0.7mm 파이프라인)는 건드리지 않는다.
LEFTWALL_FLANGE_CONTACT = os.environ.get(
    "LEFTWALL_FLANGE_CONTACT", "0" if LAYOUT_FROM_STEP else "1") == "1"
LEFTWALL_BODY_HULL_N = 5
LEFTWALL_BODY_HULLS = [f"L2_Left_Wall_body_hull_{i:03d}" for i in range(LEFTWALL_BODY_HULL_N)]
if LEFTWALL_SPLIT and LEFTWALL_CLAMP_FACE:
    raise SystemExit(
        "[config] LEFTWALL_SPLIT 과 LEFTWALL_CLAMP_FACE 는 같이 못 켠다 — "
        "clamp_face box 의 좌표는 예전 통짜 메시 기준이라 분리 형상과 어긋난다."
    )

# 중립(q=0) 압착 간격 — 고정 3벽 면(x=336.28)에서 각 압착면까지, m 단위.
# **분리하면서 0.50mm 넓어졌다**: 새 CAD 의 Left_Wall 이 그만큼 물러나 있다.
# 공통 부품 4개(Wall1/Wall2/Wall3/PLATE)로 평행이동을 교차검증했으므로 변환
# 오차가 아니라 CAD 차이다. build_leftwall_split.py 가 매 빌드마다 이 값을 찍는다.
LEFTWALL_GAP_FLANGE = 0.01250 if LEFTWALL_SPLIT else 0.01200
LEFTWALL_GAP_BODY = 0.01750 if LEFTWALL_SPLIT else (
    0.01200 if LEFTWALL_CLAMP_FACE else 0.01700)

# ── Crusher 슬롯 계산에 필요한 벽 mesh(Crusher_Samplebag.py 동일) ──────────
WALL_BACK_MESH = "L2_Wall3_1"
# 분리 후에는 벽이 메시 두 장이라 슬롯 AABB 도 둘의 합집합으로 잡아야 한다.
# 통짜 메시를 계속 쓰면 슬롯 계산만 예전 위치(0.50mm 앞)에 머물러 충돌 형상과
# 조용히 어긋난다.
WALL_LEFT_MESH = ((LEFTWALL_BODY_MESH, LEFTWALL_FLANGE_MESH) if LEFTWALL_SPLIT
                  else ("L2_Left_Wall1_1",))
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
#
# **-5mm 는 봉투를 압착하지 못한다(2026-08-25 실측, 사용자 지적)**. MJCF 로컬
# 프레임에서 재보면 고정 3벽(L1_Wall1_1 / L1_Wall2_1 / L2_Wall3_1)의 마주보는
# 면이 전부 y=336.28mm 로 같은 평면이고, L2_Left_Wall1_1 의 면이 y=324.28mm —
# **중립(q=0) 간격이 12.00mm** 다. 따라서
#
# [주의] 이 12.00mm 는 Left_Wall **상단 5mm 플랜지**에서만 성립한다. 본체 면은
# y=319.28 로 5mm 더 물러나 있어 간격이 17.00mm 이고, 그래서 벽을 닫아도 플랜지
# 한 줄만 봉투에 닿는다. 단차는 결함이 아니라 impact plate 의 통로다(§18-2).
#
# **LEFTWALL_SPLIT(기본 켜짐, 2026-08-26)이면 두 면 모두 0.50mm 넓다** — 새 CAD
# 의 Left_Wall 이 그만큼 물러나 있다. 실제 값은 LEFTWALL_GAP_* 를 보라.
#     gap_flange(q) = LEFTWALL_GAP_FLANGE + q   (분리 12.50 / 예전 12.00mm)
#     gap_body(q)   = LEFTWALL_GAP_BODY   + q   (분리 17.50 / 예전 17.00mm)
#     (q>0 개방, q<0 폐쇄)
#     q=-5mm  -> 플랜지 7.50mm 잔여 : 6mm 두께 봉투에 여유 → 압착 안 됨
#     q=-12mm -> 플랜지 0.50mm      : 사실상 완전 폐쇄 지령
#
# **이 값은 WALL_FORCE_LIM 과 짝으로만 의미가 있다.** 힘 상한 없이 깊은 목표를
# 주면 위치제어가 IPC 배리어를 무한한 힘으로 밀어 발산한다(실측: -12mm 는 44분
# 미완, -10.4mm 는 한 번은 -9.21mm 균형 · 한 번은 wall=-2.09e9mm 발산 — 안정
# 한계 위에 걸친 값이었다). 힘을 ±100 N 로 묶으면 벽은 봉투 반력과 균형지는
# 곳에서 물리적으로 멈추므로, 깊은 목표를 줘도 안전하고 **그 정지 위치가 곧
# 봉투의 실효 압착 두께 실측값**이 된다. 참고로 압착 지점은 실링부라 정제가 없고
# 필름 두 겹(2 x CLOTH_THICK = 2.00mm)뿐이므로 이론 하한은 잔여 2.00mm 부근이다.
#
# **플랜지를 접촉에서 빼면 기준면이 바뀐다(2026-08-31).** 위 -12.0mm 는 플랜지면
# (12.50mm)에서 잰 값이라 "잔여 0.50mm = 사실상 완전 폐쇄"였다. 플랜지가 빠지면
# 압착면은 본체면 하나뿐인데 그쪽은 17.50mm 에서 출발하므로, 같은 -12.0mm 가
# **잔여 5.50mm** 가 되어 6mm 봉투를 스치지도 못한다. 기준면을 본체로 바꾼다:
#     CLAMP_TARGET = -(LEFTWALL_GAP_BODY - 2.0mm)  ->  -15.5mm
# 2.0mm 는 실링부 필름 두 겹(2 x CLOTH_THICK)의 이론 하한이다. 여기서도 실제
# 정지 위치는 WALL_FORCE_LIM 이 정한다 — 목표는 "끝까지 닫으라"는 지령일 뿐이다.
_CLAMP_DEFAULT_MM = -12.0 if LEFTWALL_FLANGE_CONTACT else -(LEFTWALL_GAP_BODY * 1e3 - 2.0)
CLAMP_TARGET = float(os.environ.get("CLAMP_TARGET_MM", f"{_CLAMP_DEFAULT_MM:.1f}")) * 1e-3

# ── 압착을 위치제어 -> **속도제어**로 (2026-08-31, 사용자 지시) ──────────────
# docs/Crusher.md §11-5 이 실기 거동을 이미 이렇게 적고 있다: "Ratchet 메커니즘
# 등을 통해서 lock 을 거는 메커니즘이 아니라, **모터를 계속해서 구동시킴을 통해서
# 강하게 고정**한다." 실기에는 목표 간격이라는 게 없다 — 랙-피니언이 계속 밀고,
# 봉투 반력이 모터 힘과 균형지는 자리에서 멈출 뿐이다.
#
# 위치제어는 이걸 두 번 왜곡한다:
#   1) 도달 불가능한 목표를 매 스텝 지령하면 그 오차가 그대로 IPC 배리어에
#      실린다 — §13-11 의 24s/step 폭발, 이번 -15.5mm 21분 무진행이 그것이다.
#   2) 목표 간격을 우리가 정해야 하는데 **그 값이 곧 답**(실효 압착 두께)이다.
#      재려는 것을 입력으로 넣는 순환이 된다.
# 속도제어 + 힘 상한이면 둘 다 사라지고, 정지 위치가 그대로 측정값이 된다.
# 크랭크(Phase 11)가 이미 같은 처방을 쓰고 있어 배선도 검증돼 있다.
#
# 닫힘 속도: §11-5 "Motor2 구동부 스피드 ... **6 RPM**". 피니언 피치원 지름이
# 도면에 없어(L4_Motor2_Shaft_1 은 3.75 x 4 x 12mm 샤프트 스텁이라 못 잰다)
# **d=25mm 로 가정**하면 pi * 25mm * 0.1rev/s = 7.85mm/s. 8.0mm/s 로 둔다.
# 피니언 실치수가 확인되면 이 값만 고치면 된다.
CLAMP_MODE = os.environ.get("CLAMP_MODE", "velocity" if LAYOUT_FROM_STEP else "position")
WALL_CLOSE_MMPS = float(os.environ.get("WALL_CLOSE_MMPS", "8.0"))
# 기계적 하드스톱 = 본체면이 고정벽에 닿는 q = -LEFTWALL_GAP_BODY. 금속끼리
# 부딪히기 0.5mm 전에서 지령을 끊는다. **목표가 아니라 가드다** — 여기까지
# 내려왔다는 건 봉투를 못 물었다는 뜻이므로 로그에 그렇게 찍는다.
WALL_Q_FLOOR = -(LEFTWALL_GAP_BODY - 0.0005)

CRANK_KP, CRANK_KV = 2000.0, 100.0
# ── 클램프를 **그리퍼 레시피로** 맞춘다 (2026-08-28, 사용자 지시) ────────────
# 같은 IPC 스택에서 그리퍼는 FEM 봉투를 마찰만으로 안정적으로 파지하는데 벽은
# 못 잡는다. 두 접촉의 설정을 나란히 놓으면 같은 조건이 아니었다:
#
#            그리퍼(성공)              Left_Wall(실패)
#   kp        30                       5000      <- 167배
#   kv        1.5                      500
#   힘상한    2.0 N.m ~= 40 N          100 N
#   마찰      0.8 (명시)               미지정 -> 기본 0.1
#
# kp=30 은 매우 무른 위치제어라 봉투에 닿으면 손가락이 거기서 멈춘다 — 사실상
# 힘제어에 가까운 파지다(m0609_rg2_v2.xml 주석: "OnRobot 데이터시트 gripping
# force 3-40N"). 반면 kp=5000 은 0.1mm 오차에 500N 을 요구해 힘 상한에 즉시
# 포화되고, **매 스텝 최대 힘으로 밀어붙이는 상태**가 되어 IPC 배리어와 정면
# 충돌한다. 천을 눌러 멈추는 접촉이 아니라 뚫으려는 접촉이었다.
# 실측: -8.5mm 는 안정하나 압착 못 함, -10.0/-10.5 는 발산(-104.8mm / +625mm).
#
# 실기 Motor2 는 랙-피니언이라 강성이 높겠지만, 그 강성을 그대로 쓰면 IPC 가
# 버티지 못한다. 그리퍼도 같은 타협(kp=30)을 하고 있으므로 동일하게 간다.
#
# **[정정 2026-08-31] 그리퍼 레시피의 kp=30 은 벽에 그대로 옮길 수 없다 — 단위가
# 다르다.** 두 관절의 타입이 다르다:
#
#   m0609_rg2_v2.xml:152   f1_finger_tip_joint   type="hinge"  -> kp [N.m/rad]
#   Crusher_IsaacSim.xml:114 L1_Guide1_1_...      type="slide"  -> kp [N/m]
#
# 그리퍼의 kp=30 N.m/rad 은 0.067 rad 만 어긋나도 2.0 N.m(≈40N)로 포화한다 —
# 손가락 끝에서 약 3mm 다. 즉 **선형 환산 강성은 ≈12,000 N/m 로 오히려 5000 보다
# 세다.** 그걸 벽에 kp=30 N/m 로 옮기면 11.5mm 를 어긋내도 힘이 0.35N 밖에
# 안 나온다(봉투 무게 0.025N 의 14배). 힘 상한 40N 은 근처도 못 간다 — 압착이
# 안 되는 게 아니라 **아예 밀지를 않는다.**
#
# 그리고 힘을 WALL_FORCE_LIM 으로 자르는 이상 kp 의 크기는 접근 구간에서만
# 의미가 있다. 포화 후에는 kp 5000 이든 12000 이든 똑같이 상한값으로 민다.
# 실제 압착력을 정하는 손잡이는 kp 가 아니라 **WALL_FORCE_LIM** 이다.
#
# 따라서 §19-4 의 "kp 167배가 원인" 진단은 성립하지 않는다. -10.0/-10.5mm 가
# 발산한 진짜 이유는 그 지점부터 플랜지가 봉투를 **마주보는 면 없이** 뭉개기
# 시작해 균형점이 존재하지 않았기 때문이다(§18-5). 플랜지를 접촉에서 빼면
# 본체면 ↔ Wall3 라는 실제 반력면이 생기므로 균형점이 생긴다.
# 그래서 검증된 5000/500 + 100N 으로 되돌린다.
# ── 랙-피니언 반사 관성 (2026-08-31) ───────────────────────────────────────
# MJCF 의 Left_Wall 링크 질량은 **0.312 kg** 뿐이다. 벽 판때기 자체의 질량은
# 맞지만, 실제로 이 축을 움직이려면 랙·피니언·감속기·모터 로터를 전부 같이
# 가속시켜야 한다. 그 반사 관성이 모델에 없으면 축이 터무니없이 가볍다:
#
#     100N / 0.312kg = 321 m/s^2  ->  dt=5ms 한 스텝에 +1,603 mm/s
#
# 그래서 봉투에 처음 닿는 순간의 작은 교란 하나가 그대로 관통이 된다(실측:
# 잔여 8mm 에서 v 가 -8 -> -16 -> -3,237mm/s, wall 이 -373mm 까지 뚫고 나감).
# 수치 안정성도 같은 뿌리다 — 명시적 감쇠의 한계가 kv <~ 2m/dt = 125 인데
# WALL_KV=500 은 그 4배다. 접촉 전에는 안 드러나다가 닿는 순간 발산한다.
#
# MuJoCo 의 `armature` 가 정확히 이 항이다(slide 조인트에서는 kg). Genesis 도
# MJCF 의 per-joint armature 를 읽는다(genesis/utils/mjcf.py:153~).
# 실기 사양(§11-5 "6 RPM" 감속 기어모터)에서 로터 관성 x 감속비^2 는 랙 축에서
# 수십 kg 급이 되므로, 0 보다는 큰 값이 반드시 맞다. 정확한 값은 모터/감속기
# 사양이 있어야 정해지므로 **스윕 대상**으로 두고 기본은 10kg 로 잡는다
# (100N -> 9.7 m/s^2, 스텝당 +48mm/s. kv 한계도 2*10.3/0.005 = 4,120 으로 올라가
# WALL_KV=500 이 안정 영역에 들어온다).
WALL_ARMATURE = float(os.environ.get("WALL_ARMATURE", "10.0"))
# ── 벽을 기구학적으로 구동한다 (2026-08-31) ────────────────────────────────
# 힘제어(control_dofs_position/velocity)로는 첫 접촉에서 벽이 튕겨나간다. 힘·속도
# ·관성·구속강성을 다 흔들어도 같은 지점에서 같은 모양으로 터졌다. 힘으로 미는
# 한 접촉 임펄스가 벽을 움직일 수 있기 때문이다.
# `set_dofs_position` 으로 매 스텝 위치를 **직접 써넣으면** 벽은 지령대로만
# 움직이고 접촉이 되밀 수 없다 — 강성이 무한한 기구학 구동이다. 실기의 랙-피니언
# 감속기가 사실상 이쪽에 가깝기도 하다(6 RPM 저속 대감속이라 봉투 반력으로
# 역구동되지 않는다).
# 대신 힘 상한이라는 정지 기준이 사라지므로 **어디까지 닫을지를 정해야 한다** —
# CLAMP_TARGET 이 다시 의미를 갖는다. 압착력은 그 위치에서 봉투가 얼마나 눌렸는지
# (본체면 잔여)로 읽는다.
WALL_KINEMATIC = os.environ.get("WALL_KINEMATIC", "0") == "1"

WALL_KP = float(os.environ.get("WALL_KP", "5000.0"))
WALL_KV = float(os.environ.get("WALL_KV", "500.0"))
# Motor2(Left_Wall, Rack&Pinion) 힘 상한 — docs/Crusher.md §5 의 액추에이터
# `Motor2_left_wall` ctrlrange ±100 N (MJCF actuatorfrcrange 와 동일).
# 적용 이유는 사용처(_fmin/_fmax) 주석 참고.
# 40N 은 그리퍼 레시피 이식 때 같이 따라온 값인데, 위 [정정]대로 그 이식은
# 단위가 안 맞았다. 실기 Motor2 의 스펙값인 100N 으로 되돌린다 — 압착력을 정하는
# 진짜 손잡이가 이것이므로 근거 없는 값을 쓰면 안 된다.
WALL_FORCE_LIM = float(os.environ.get("WALL_FORCE_LIM_N", "100.0"))

# ── Phase 11: crush — 고정된 봉투를 두고 Crusher 를 실제로 운전(2026-08-26) ──
# 여기까지가 "봉투를 슬롯에 넣고 벽으로 문다" 였고, 그 상태에서 크랭크를 돌려
# impact plate 가 정제를 때리게 한다. 구동 사양·패턴은 Crusher_only.py 를 그대로
# 따른다(그쪽이 크러셔 단독 검증용으로 이미 확립해 둔 것):
#
#   · 8 RPM (docs/Crusher.md §1 "운전 속도 8 RPM", BL4281+PG42 저속 준정적)
#   · **velocity 제어 + 토크 클램프** — position 제어로 각도를 램프시키면 접촉으로
#     크랭크가 밀렸을 때 PD 가 무한 토크로 밀어붙인다. Left_Wall 에 WALL_FORCE_LIM
#     을 건 것과 같은 이유이고, 같은 처방이다.
#   · τ ≤ 12.5 N·m (Crusher.md §2-2 표 — 슬라이더 실측 625 N 과 매칭되는 값)
#   · kv 만 쓰므로 크게 잡아 force_range 한계까지 즉시 saturate 시킨다.
#
# **기본 꺼짐(CRUSH_SECONDS=0)**. 60s 는 DT=5e-3 에서 12,000 스텝이라 기존 2,430
# 스텝짜리 런에 20분 이상을 더한다 — 매 런마다 물릴 값이 아니다.
# 매 스텝 render_cams() 를 부르면 3대 x 12,000 프레임이 되므로 이 구간만
# CRUSH_RENDER_EVERY 로 솎는다(30fps 기준 10 이면 60s -> 40s 클립).
CRUSH_SECONDS = float(os.environ.get("CRUSH_SECONDS", "0"))
CRUSH_RENDER_EVERY = int(os.environ.get("CRUSH_RENDER_EVERY", "10"))
CRANK_RPM = 8.0
CRANK_OMEGA = CRANK_RPM * 2.0 * np.pi / 60.0     # 0.8378 rad/s
CRANK_TORQUE_LIM = float(os.environ.get("CRANK_TORQUE_LIM_NM", "12.5"))
CRANK_KV_SPIN = 5000.0


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
        if WALL_ARMATURE > 0:
            for _j in root.iter("joint"):
                if _j.get("name") == WALL_JOINT:
                    _j.set("armature", f"{WALL_ARMATURE:.6f}")
                    print(f"[mjcf] Left_Wall 반사 관성 armature={WALL_ARMATURE:.1f}kg "
                          f"(링크 질량 0.312kg + 랙/피니언/감속기/로터). "
                          f"100N -> {100.0/(0.312+WALL_ARMATURE):.1f} m/s^2, "
                          f"kv 안정 한계 {2*(0.312+WALL_ARMATURE)/5e-3:.0f}")
        if CLAMP_ONLY:
            # CLAMP_ONLY 는 봉투를 빌드 시점에 이미 슬롯 안에 스폰한다. 그런데
            # 크랭크 q=0 이면 impact plate(L9_PLATE_v3_1)가 그 슬롯을 차지한다 —
            # 실측 world AABB x[0.1865,0.2365] y[-0.0085,0.0015] z[0.025,0.075]
            # 로 봉투와 24,944mm^3 겹치고, IPC 의 build-time 교차 검사에 걸려
            # 늘 나오던 `'NoneType' object has no attribute 'body_count'` 로 죽는다.
            #
            # 풀 시퀀스는 Phase 0 에서 크랭크를 -180deg 로 돌려 플레이트를 빼지만
            # CLAMP_ONLY 는 그 구간을 건너뛴다. MJCF 의 `ref` 로 초기각을 주는
            # 방법은 안 통했다(Genesis 가 qpos0 를 읽기는 하나 실제 자세는 안
            # 바뀐다 — 실측). 그래서 **플레이트를 접촉에서 뺀다**: 플랜지와 같은
            # 처방(contype/conaffinity=0 -> link.vgeoms 로 가서 커플러가 못 봄)이고,
            # 압착만 격리해 재는 모드라 플레이트는 애초에 등장하지 않는다.
            # (Phase 11 crush 를 켜려면 CLAMP_ONLY 를 끄고 풀 시퀀스로 가야 한다.)
            _n_plate = 0
            for _g in wb.iter("geom"):
                if (_g.get("mesh") or "").startswith("L9_PLATE"):
                    _g.set("contype", "0"); _g.set("conaffinity", "0")
                    _n_plate += 1
            print(f"[mjcf] CLAMP_ONLY — impact plate geom {_n_plate}개를 시각 전용으로 "
                  f"내림(빌드 시 봉투와 겹침 회피). 압착 격리 모드라 분쇄는 대상이 아니다.")
        if LEFTWALL_SPLIT:
            _split_leftwall_collision(root, wb)
        if LEFTWALL_CLAMP_FACE:
            _add_leftwall_clamp_face(wb)
        for body in wb.iter("body"):
            if body.get("name") == "L7_Link3_1":
                inertial = body.find("inertial")
                if inertial is not None:
                    inertial.set("pos", L7_LINK3_COM)
    tree.write(dst)


def _split_leftwall_collision(root, wb):
    """Left_Wall 의 시각 메시 + CoACD hull 11개를 "본체 hull 5개 + 플랜지 box" 로 간다.

    소스(`Crusher_IsaacSim_colored.xml`)의 L2_Left_Wall1_1 body 는 이렇게 생겼다:
        geom mesh=L2_Left_Wall1_1            contype=0/conaffinity=0  (시각, material)
        geom mesh=L2_Left_Wall1_1_hull_000~010  group=3               (충돌 11개)
        geom mesh=L3_RackGear_1              contype=0/conaffinity=0  (시각, 남긴다)
    앞의 12개를 통째로 걷어내고 새 6+1개를 넣는다. **옛 hull 을 남기면 새 형상과
    겹쳐 충돌 geom 이 이중으로 깔리고 IPC build 가 죽는다.**

    근거는 LEFTWALL_SPLIT 상수 주석. 여기서는 배선만 한다.

    geom 별 역할 배정은 Genesis 의 MJCF 규칙을 따른다(genesis/utils/mjcf.py):
      - `contype or conaffinity` 가 참이면 충돌 geom 이 된다.
      - 그중 `group` 이 0/1/2 면 시각 geom 으로도 **복제**된다(MuJoCo 규칙).
    그래서 hull 은 group=3 으로 둬 충돌만 시키고(뭉툭한 hull 이 렌더되면 벽이
    덩어리로 보인다), 본체 원형은 contype/conaffinity=0 으로 시각만 맡긴다.
    플랜지는 verts 8개짜리 정확한 box 라 볼록화해도 자기 자신이므로 시각/충돌을
    겸한다 — 렌더와 충돌이 같은 형상이라는 뜻이라 오히려 정직하다.

    **부피 겹침 금지**: IPC 의 build-time 검사는 contype/conaffinity 필터를
    무시하고 순수 메시 교차만 본다. 같은 body 안 형제 geom 이라도 겹치면
    "Intersection detected" 후 `'NoneType' object has no attribute 'body_count'`
    로 죽는다(§9 조합9 후속4 와 같은 서명). 본체와 플랜지는 y=81.43 에서
    **맞닿기만** 하고(겹침 0) CoACD hull 도 그 선을 넘지 않는 것을
    build_leftwall_split.py 가 매 빌드마다 검사한다. 맞닿는 것은 괜찮다.
    """
    asset = root.find("asset")
    if asset is None:
        print("[mjcf] 경고: <asset> 이 없어 Left_Wall 분리를 건너뛴다")
        return
    for name in (LEFTWALL_BODY_MESH, LEFTWALL_FLANGE_MESH, *LEFTWALL_BODY_HULLS):
        ET.SubElement(asset, "mesh", {
            "name": name, "content_type": "model/stl",
            "file": f"{name}.stl", "scale": "0.001 0.001 0.001",
        })

    for body in wb.iter("body"):
        if body.get("name") != "L2_Left_Wall1_1":
            continue
        # 시각 메시와 hull 11개를 한꺼번에 잡는다 — 접두사로 걸러야 옛 hull 이
        # 남아 새 형상과 이중으로 깔리는 일이 없다. L3_RackGear_1 은 안 건드린다.
        old = [g for g in body.findall("geom")
               if (g.get("mesh") or "").startswith("L2_Left_Wall1_1")]
        if not old:
            print("[mjcf] 경고: L2_Left_Wall1_1 geom 을 못 찾아 분리를 건너뛴다")
            return
        # 새 geom 은 옛 geom 과 같은 pos/quat/material 을 그대로 쓴다 — 분리 메시가
        # 애초에 같은 STL 프레임으로 옮겨져 있으므로 프레임 보정이 필요 없고,
        # material 을 빠뜨리면 벽만 기본색으로 렌더돼 눈에 띈다.
        base = next((g for g in old if g.get("mesh") == "L2_Left_Wall1_1"), old[0])
        pos, quat, mat = base.get("pos"), base.get("quat"), base.get("material")
        n_old = len(old)
        for g in old:
            body.remove(g)

        def _geom(name, mesh, visual, **kw):
            attrs = {"name": name, "type": "mesh", "mesh": mesh}
            if pos:
                attrs["pos"] = pos
            if quat:
                attrs["quat"] = quat
            if visual and mat:
                attrs["material"] = mat
            attrs.update(kw)
            ET.SubElement(body, "geom", attrs)

        _geom("L2_Left_Wall_body_vis", LEFTWALL_BODY_MESH, True,
              contype="0", conaffinity="0", group="2")
        for i, h in enumerate(LEFTWALL_BODY_HULLS):
            _geom(f"L2_Left_Wall_body_col_{i:03d}", h, False,
                  contype="1", conaffinity="1", group="3")
        # 플랜지: 접촉을 끄면 시각 전용(group=2)으로 내려 IPC·강체 양쪽에서 뺀다.
        # group 을 0 -> 2 로 바꾸는 이유는 없다(둘 다 시각 그룹) — 본체 시각 geom
        # 과 같은 값으로 맞춰 두면 "시각 전용"이라는 의도가 한눈에 읽힌다.
        if LEFTWALL_FLANGE_CONTACT:
            _geom("L2_Left_Wall_flange", LEFTWALL_FLANGE_MESH, True,
                  contype="1", conaffinity="1", group="0")
        else:
            _geom("L2_Left_Wall_flange_vis", LEFTWALL_FLANGE_MESH, True,
                  contype="0", conaffinity="0", group="2")

        _fl = ("충돌+시각" if LEFTWALL_FLANGE_CONTACT else "**시각 전용(접촉 제외)**")
        print(f"[mjcf] Left_Wall 형상 교체: 옛 geom {n_old}개(시각 1 + hull "
              f"{n_old-1}) -> 본체 hull {len(LEFTWALL_BODY_HULLS)}개 + 플랜지 1개 "
              f"+ 시각 1개. 플랜지 = {_fl}. "
              f"중립 간격 플랜지 {LEFTWALL_GAP_FLANGE*1000:.2f}mm / "
              f"본체 {LEFTWALL_GAP_BODY*1000:.2f}mm")
        return
    print("[mjcf] 경고: L2_Left_Wall1_1 body 를 찾지 못해 분리를 건너뛴다")


def _add_leftwall_clamp_face(wb):
    """Left_Wall 의 압착면을 상단 플랜지와 같은 평면으로 채우는 box geom 추가.

    **문제(2026-08-25 실측, 사용자 지적 "관통하는 듯한 현상")**
    L2_Left_Wall1_1 은 평평한 벽이 아니다. STL 을 재보면 마주보는 면이 두 단이다:

        상단 플랜지  z[81.43, 86.43] (5mm)   면 y=324.28  -> 고정벽과 간격 12.00mm
        본체 면      z[ 9.43, 81.43] (72mm)  면 y=319.28  -> 고정벽과 간격 17.00mm

    (고정 3벽 L1_Wall1_1 / L1_Wall2_1 / L2_Wall3_1 의 면은 전부 y=336.28)

    볼록분해(hull_000~010)는 이 형상을 정확히 재현한다 — hull_000 만 324.28 에
    닿고 나머지 10개는 319.28 에서 끝난다. 즉 **분해 누락이 아니라 부품 형상이다.**

    결과: 벽을 닫으면 **상단 5mm 플랜지만 봉투에 닿고** 본체 72mm 는 17mm 간격이라
    8mm 봉투를 영영 건드리지 못한다. 벽은 플랜지가 봉투에 걸린 지점(q=-4.97mm)에서
    멈추고, 그 아래는 계속 헐렁하다. 봉투가 위 모서리 한 줄로만 눌려 release 때
    tilt 2.5deg -> 6.9deg 로 기울고 x 로 2.9mm 밀린 것이 이 때문이다.
    게다가 플랜지(z 81.43~86.43)는 Wall3 상단(80.43)보다 **위**라 서로 z 가 겹치지
    않는다 — 지금 형상으로는 두 벽이 애초에 맞물릴 수 없다.

    **처방(사용자 지시: "맞물릴 수 있도록 ... 해줘라")**
    본체 면의 5mm 단차를 box 로 메워 압착면을 77mm 전체에서 평평하게 만든다.
    그러면 간격이 전 높이에서 12.00mm 로 균일해지고 벽이 봉투를 면으로 문다.
    box 는 analytic SDF 라 메시 충돌보다 빠르고 터널링에도 강하다(Twin.md §5).

    좌표는 STL 프레임(= 기존 mesh geom 들과 같은 로컬 프레임, x->world y,
    y->world z, z->world x)에서 잡고 mesh geom 과 같은 pos 오프셋을 뺀다.
    """
    # z_hi 는 **플랜지 하단(81.39)까지만** 이다. 처음에 벽 높이 전체(86.43)로 채웠다가
    # hull_000(플랜지, y[306.28,324.28] z[81.39,86.43])과 1638mm^3 겹쳐서 IPC build 가
    # 죽었다 — `AttributeError: 'NoneType' object has no attribute 'body_count'`
    # (coupler.py:722 _init_accessors, 직전에 "Intersection detected"). §9 조합9 후속4
    # 석션V1 조 링크 자기교차와 **정확히 같은 서명**이다. IPC 의 build-time 유효성
    # 검사는 contype/conaffinity 필터링을 무시하고 순수 메시 교차만 보므로, 같은
    # body 안의 형제 geom 이라도 부피가 겹치면 안 된다. 맞닿는 것(겹침 0)은 괜찮다 —
    # 기존 hull 11개가 이미 서로 맞닿아 있고 정상 빌드된다.
    GEOM_OFF = np.array([-0.286278, -0.016542, 0.017802])   # 기존 mesh geom 들의 pos
    face_y, body_y = 0.32428, 0.31928        # 플랜지 면 / 본체 면
    z_lo, z_hi = 0.00943, 0.08139            # 본체 하단 ~ 플랜지 하단(hull_000 회피)
    x_lo, x_hi = -0.07980, -0.01480          # 벽 폭 전체
    half = np.array([(face_y - body_y) / 2, (z_hi - z_lo) / 2, (x_hi - x_lo) / 2])
    ctr = np.array([(face_y + body_y) / 2, (z_hi + z_lo) / 2, (x_hi + x_lo) / 2]) + GEOM_OFF
    for body in wb.iter("body"):
        if body.get("name") != "L2_Left_Wall1_1":
            continue
        ET.SubElement(body, "geom", {
            "name": "L2_Left_Wall1_1_clampface",
            "type": "box",
            "pos": " ".join(f"{v:.6f}" for v in ctr),
            "size": " ".join(f"{v:.6f}" for v in half),
            "rgba": "0.15 0.15 0.18 1",
        })
        print(f"[mjcf] Left_Wall 압착면 box 추가: 단차 {(face_y-body_y)*1000:.2f}mm 를 "
              f"높이 {(z_hi-z_lo)*1000:.1f}mm x 폭 {(x_hi-x_lo)*1000:.1f}mm 전체에 메움 "
              f"-> 압착 간격이 전 높이에서 12.00mm 로 균일")
        return
    print("[mjcf] 경고: L2_Left_Wall1_1 body 를 찾지 못해 압착면 box 를 추가하지 못했다")


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
    """메시의 world AABB. 이름 하나든 여러 개든 받는다 — 여러 개면 합집합.

    LEFTWALL_SPLIT 이후 WALL_LEFT_MESH 가 (본체, 플랜지) 두 장이 됐다. 호출부를
    일일이 고치는 대신 여기서 흡수한다 — full_workflow_rigid / slot_fit_check /
    slot_ik_check / interactive_probe 가 이 함수에 그 상수를 그대로 넘긴다.
    두 조각은 같은 body/geom 프레임을 쓰므로 pos 인자는 공유해도 된다."""
    if not isinstance(mesh_name, str):
        los, his = zip(*(crusher_mesh_world_aabb(n, body_pos, geom_pos) for n in mesh_name))
        return np.min(los, axis=0), np.max(his, axis=0)
    yaw = np.radians(CRUSHER_EULER[2])
    R_e = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                    [np.sin(yaw), np.cos(yaw), 0.],
                    [0., 0., 1.]])
    v = tm.load(os.path.join(paths.MJCF_DIR, f"{mesh_name}.stl")).vertices * 0.001
    local = np.asarray(geom_pos) + v
    in_crusher = np.asarray(body_pos) + (_R_GEOM_HALF @ local.T).T
    w = np.array(CRUSHER_POS) + (R_e @ in_crusher.T).T
    return w.min(axis=0), w.max(axis=0)


def slot_geometry():
    """Crusher 슬롯 기하 — STL + CRUSHER_POS/EULER 만으로 정해지므로 씬 없이 계산된다.

    main() 의 Phase 7 과 CLAMP_ONLY 의 봉투 스폰 위치가 **같은 값**을 써야 해서
    함수로 뺐다. 두 벌로 두면 조용히 어긋난다.

    간격 축은 하드코딩하지 않고 **기하로 판별**한다(2026-08-28). 예전 코드는
    `wb_hi[0]`/`wl_lo[0]` 처럼 간격이 world X 에 있다고 박아 뒀는데 그건
    CRUSHER_EULER=(0,0,90) 일 때만 참이다. STEP 실배치(EULER 0)로 바꾸자 간격이
    world Y 로 옮겨가 gap_width 가 12mm 대신 80mm 로 잡혔다(실측). 두 벽 AABB 가
    겹치지 않는 축이 곧 간격 축이다.
    """
    wb_lo, wb_hi = crusher_mesh_world_aabb(WALL_BACK_MESH)
    wl_lo, wl_hi = crusher_mesh_world_aabb(WALL_LEFT_MESH, LEFTWALL_BODY_POS, LEFTWALL_GEOM_POS)
    ov = [min(wb_hi[i], wl_hi[i]) - max(wb_lo[i], wl_lo[i]) for i in range(2)]
    gap_ax = int(np.argmin(ov))           # 떨어진 축 = 봉투 두께가 들어갈 방향
    oth_ax = 1 - gap_ax                   # 겹치는 축 = 슬롯 길이 방향(봉투 폭)
    g_lo = min(wb_hi[gap_ax], wl_hi[gap_ax])
    g_hi = max(wb_lo[gap_ax], wl_lo[gap_ax])
    g_c = (g_lo + g_hi) / 2.0
    o_c = (max(wb_lo[oth_ax], wl_lo[oth_ax]) + min(wb_hi[oth_ax], wl_hi[oth_ax])) / 2.0
    wall_top_z = max(wb_hi[2], wl_hi[2])
    return {
        "wb_lo": wb_lo, "wb_hi": wb_hi, "overlap": ov,
        "gap_ax": gap_ax, "oth_ax": oth_ax, "gap_width": g_hi - g_lo,
        "gap_cx": g_c if gap_ax == 0 else o_c,
        "gap_cy": g_c if gap_ax == 1 else o_c,
        "wall_top_z": wall_top_z,
        # insert_z 계산과 판정 기준이 되는 벽 중앙 높이(main 의 Phase 7 과 동일식).
        "wall_center_z": (wall_top_z + wb_lo[2]) / 2.0,
    }


# ── M0609+RG2(v2) + 정제 + 샘플백 ───────────────────────────────────────────
ROBOT_MJCF = os.path.join(paths.ROBOTS_DIR, "m0609_rg2_v2.xml")
COACD_DIR_REL = "rg2/reference_onrobot_ros/meshes/rg2_v1/coacd"
FLEX_FINGER_HULLS = [f"flex_finger_hull_{i:03d}.stl" for i in range(7)]
# 실링부 1mm 판으로 전환(2026-08-25, 사용자 지시). 기존 `Samplebag_seal_pouch3.stl`
# 은 X 컬럼 17개가 전부 두께 6.000mm 인 **실링부 없는** 균일 파우치라, Phase 9 가
# "실링부 압착"이라고 찍으면서 실제로는 파우치 몸통을 누르고 있었다.
# `_seal1mm` 은 양측 가장자리 3컬럼(|x|=24,28,32mm)이 1.000mm 로 눌린 판이지만
# **IPC build 를 통과하지 못한다** — 측면/바닥 패널의 z=0 중간선이 +-0.5mm 면 사이
# 정가운데 남아 자기간격이 0.500mm 로 반토막 나기 때문이다(IPC 요구 2*CLOTH_THICK).
#   Object[cloth_0_0] is too close (distance=0.000398, thickness=0.002) to itself
# 그래서 `_sealslab1mm` 을 새로 생성했다(§16-7 "등두께 슬래브"):
#   1) z=0 중간선 57정점을 제거하고 둘레(좌21/바닥17/우21)를 front-back 직결 quad 로
#      재삼각화 -> 중간선발 근접이 원천 소멸 (정점 771->714, 면 1504->1392)
#   2) 실링 대역 |x|>=24mm 를 z=+-0.5mm **등두께 슬래브**로 (비례축소 아님)
#   3) 테이퍼를 x 20->24(4mm)에서 **16->24(8mm)** 로 늘림 — 4mm 테이퍼는 비틀린 quad
#      대각이 반대편 모서리에 0.759mm 까지 접근해 여전히 부족했다
# 실측 최소 자기거리: 원본 2.500 / _seal1mm 0.500 / _sealslab1mm 1.000mm
#
# **실링 두께는 CLOTH_THICK 의 2배 이상이어야 한다.** IPC 요구가 자기간격 >=
# 2*CLOTH_THICK 이고 슬래브 실링의 자기간격이 곧 실링 두께이기 때문이다. 물리적
# 으로도 같다 — 실링부는 앞뒤 필름이 맞붙어 용착된 자리라 두께가 곧 필름 2겹이다.
# 1.0mm 필름으로 1mm 실링은 성립할 수 없다.
# `_sealslab1mm`(실링 1.0mm) 은 build 는 통과하지만 CLOTH_THICK 을 0.4mm 로
# 낮춰야 해서 **파지가 깨졌다**(lift 에서 그리퍼만 +126mm, 봉투 제자리 -> tilt
# 78.3deg -> 바닥 낙하, FAIL 오차 -49.9mm). FING_CLOSE=1.20 이 1.0mm 기준이라
# 0.4mm 천에는 닿지 못한다 — §17-7 의 0.1mm 실패와 같은 서명.
#
# 그래서 **`_sealslab3mm`(실링 3.0mm, 테이퍼 12->24mm)** 을 쓴다. 실측 최소
# 자기거리 2.278mm 로 CLOTH_THICK=1.0mm 요구치 2.000mm 를 278um 여유로 통과한다.
# 실링 폭은 편측 8mm 로 동일, 몸통은 6.0mm 유지.
BAG_STL = os.path.join(paths.ROBOTS_DIR, "Samplebag",
                       os.environ.get("BAG_STL_NAME", "Samplebag_seal_pouch3_sealslab3mm.stl"))
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
FIXTURE_POS = (0.0600, -0.3475, 0.0000) if LAYOUT_FROM_STEP else (0.5, -0.3, 0.12)

# 석션V1 배치(2026-07-29 재작성, 사용자 지시): 더 이상 정사각형 레이아웃의
# 대칭 위치가 아니다 — 실제 작업 구조("회수장치2가 봉투를 고정하고, 석션V1의
# 흡착컵 2개가 봉투를 양옆으로 당겨서 여는 구조")에 맞춰, 회수장치2의
# F_LeftLink_1-F_RightLink_1 중점에 석션V1의 흡착컵 2개(Suction_Cup_M5_
# 0.8mm_15mm_1/2) 중점이 오도록 정렬한다.
#   F_LeftLink_1/F_RightLink_1 중점(회수장치2 원점 기준, mesh bounds 실측):
#     (-0.057502, 0.059201, 0.098) — **recovery2_mjcf 빌드 기준으로 재측정
#     (2026-08-19)**. 구 회수장치2_description 빌드에서는 (-0.08845, 0.06513,
#     0.098) 이었다(§RECOVERY2_MJCF 주석: 두 빌드는 모델 원점이 다르다).
#   흡착컵 중점(석션V1 원점 기준, q=(0,0,0) 기본자세, genesis 충돌 geom 실측):
#     (-0.07790, 0.06750, 0.28594)
#   world_target = RECOVERY2_POS + F링크중점 = (0.378513, -0.227519, 0.367524)
#   SUCTIONV1_POS = world_target - 흡착컵중점 = (0.456413, -0.295019, 0.081584)
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
SUCTIONV1_POS = (0.0085, -0.1975, 0.0000) if LAYOUT_FROM_STEP else (0.675, -0.29501900, 0.08158463)
# 석션V1 만 MJCF 에 회전이 안 들어 있어 Z+90 이 필요하다(Hbeam_L/B/M, Dummy 대조).
SUCTIONV1_EULER = (0.0, 0.0, 90.0) if LAYOUT_FROM_STEP else (0.0, 0.0, 0.0)

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
#
# **에셋 교체(2026-08-19, 사용자 지시)**: fusion2urdf→URDF→MJCF 를 거친
# `회수장치2_description/회수장치2.xml` 대신 fusion2xml(MJCF 직행) 빌드인
# `recovery2_mjcf/recovery2.xml` 을 쓴다(자산 대조표는 그 폴더 README.md).
# 경로가 ASCII 라 `paths.ascii_safe_mjcf` 미러가 필요 없다.
#   - **RECOVERY2_POS 는 그대로 둔다.** 위 정렬점 (-74.015, 59.22, 1.776)mm 는
#     Fusion 조인트 앵커(joint_anchors_.csv) 값인데, 구 빌드는 정작 자기 샤프트
#     힌지축이 그 앵커에서 (+30.92, -6.62, 0)mm 어긋나 있었다(두 빌드를 MuJoCo
#     qpos0 로 올려 부품별 메시 월드 AABB 를 대조 — 샤프트에 꿰인 환형 부품
#     Bearing1/NeedleBearing/Washer/PULLEY/RachetGear 중심이 각 빌드의 자기
#     힌지축과는 0.3mm 안에서 일치하므로 두 빌드 모두 내부적으로는 정합).
#     신 빌드는 힌지축(회전_31)이 앵커와 정확히(x,y 0.01mm 이내) 일치하므로,
#     같은 POS 로 오히려 의도했던 정렬(고정장치 ServoShaft 중심 ↔ 회수장치2
#     샤프트)이 처음으로 실제로 맞는다. 대신 렌더상 회수장치2 전체가 예전
#     영상 대비 x +30.9mm, y -6.6mm 옮겨 보인다.
#   - 신 빌드가 조립 자체도 맞다: 구 빌드는 가동턱(M_*)이 고정턱(F_*) 대비
#     x +35mm / y -5~8mm 어긋나 있었는데(M_Top-F_Top: old +91.87,-5.49 →
#     new +55.07,+0.01mm), 신 빌드는 두 턱이 y 로 0.01mm 안에서 정렬되고
#     qpos0 가 닫힌 상태(F/M 링크 간격 5.0mm)다. 즉 정지 소품의 겉모습도
#     "35mm 더 열린 채 비틀린" 것에서 "닫힌 정상 조립"으로 바뀐다.
#   - 조인트 4개(더미 바디 Crank_1_b 포함) → 3개(슬라이더_35 / 회전_31 /
#     회전_29), 폐루프 connect 가 실제 바디끼리(Crank_1↔M_Top_1)로 바뀌었다.
#     Genesis 1.3.3 은 이 XML 을 그대로 파싱한다(links=33, dofs=3, equality=1
#     확인). 위 석션V1 주석의 "exclude 는 Genesis mjcf.py 파싱이 깨진다"는 메모는 그
#     버전에서는 더 이상 유효하지 않다 — 신 빌드의 <contact><exclude> 200여
#     쌍은 contype/conaffinity 비트마스크로 변환돼 들어간다.
#   - **충돌 geom 이 0개 → 33개로 늘어난다.** 구 빌드는 ShaftHandle_1_hull
#     하나만 contype=1 이고 나머지가 전부 contype/conaffinity=0 이라 Genesis 가
#     시각 geom 으로만 취급했다. 신 빌드는 부품마다 `*_col`(원본 STL, 최대
#     15k face) 을 달고 있어, 정적 소품이라 접촉이 무의미해도 빌드 비용과
#     IPC 교차 검사에는 그대로 들어간다.
RECOVERY2_MJCF = os.path.join(paths.ROBOTS_DIR, "recovery2_mjcf", "recovery2.xml")
RECOVERY2_POS = (-0.0040, -0.3342, 0.1525) if LAYOUT_FROM_STEP else (0.436015, -0.28672, 0.269524)
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
# ── 강체 <-> IPC 프록시 소프트 구속 강성 (2026-08-31 스윕 대상) ─────────────
# Crusher 는 coup_type="two_way_soft_constraint" 로 실린다. 이 커플링은 Genesis
# 강체 솔버가 지령한 자세와 uipc 안의 프록시 사이를 스프링으로 묶는데, 그 강성이
# 이 값이다. 봉투 접촉이 프록시를 밀어내면 이 스프링이 되당기고 그 반력이 다시
# 강체로 들어가는 **양방향 루프**가 된다.
# 벽이 봉투에 처음 닿는 순간 터지는 임펄스가 액추에이터에서 나오지 않는다는 것은
# 역산으로 확인됐다(armature=10 런: m*dv/dt = 10.3 x 2.46 / 0.17 = 149N > 100N
# 상한). 힘(100~800N)·속도(2~8mm/s)·관성(0.312~10.3kg)을 다 바꿔도 같은 지점에서
# 같은 모양으로 터졌으므로 남은 후보가 이 루프다.
IPC_CONSTRAINT_STRENGTH = float(os.environ.get("IPC_CONSTRAINT_STRENGTH", "100.0"))

# 봉투 stiffen(2026-07-15): above/insert 구간에서 봉투가 과하게 흔들려 E/굽힘
# 강성을 올림(1e5->4e5, bend 50->400) — 이동 속도를 늦추는 것(N_ABOVE/N_INSERT)과
# 함께 스윙을 줄이기 위한 조합 처방.
# CLOTH_THICK 스윕(2026-08-25, 사용자 지시 "봉투 두께를 실제와 비슷하게"):
# 기본 1.0mm 는 실측 필름 두께가 아니라 §13-3 프록시 질량 일치 트릭에 맞춘 값이다
# (STL 의 6mm 는 파우치 공동 두께이지 천 두께가 아니며, 실제 필름 실측 기록은 없다).
# recovery2_bag_clamp.py 는 0.1mm 를 쓴다 — 이쪽이 실물 약포지 필름에 가깝다.
# CLOTH_THICK 은 접촉 두께이자 막 강성 계수라 단독으로 낮추면 봉투가 그 비율만큼
# 물러지므로(§15), 막 강성 E*t=400 N/m 를 보존하도록 CLOTH_E 를 함께 준다.
#   기본       : t=1.0mm, E=4.0e5  -> E*t = 400 N/m
#   실물 근사  : t=0.1mm, E=4.0e6  -> E*t = 400 N/m
# 실물 필름은 약 0.2mm 지만 질량/관성이 너무 작아 수치적으로 불안정해질 소지가
# 커서 **1.0mm 로 합의**했다(2026-08-25, 사용자). 0.1mm 를 시도했을 때 파지가
# 아예 성립하지 않은 실측도 있다(§17-7) — 이 값은 FING_CLOSE 캘리브레이션과
# 묶여 있어 단독으로 못 낮춘다.
# _sealslab3mm 의 자기간격 2.278mm 가 CLOTH_THICK <= 1.139mm 를 허용하므로
# 검증된 1.0mm 를 그대로 쓴다(E 도 원래 4.0e5, E*t = 400 N/m).
CLOTH_THICK = float(os.environ.get("CLOTH_THICK_MM", "1.0")) * 1e-3
CLOTH_E = float(os.environ.get("CLOTH_E", "4.0e5"))
CLOTH_NU, CLOTH_RHO = 0.499, 200.0
CLOTH_BEND = 400.0
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
# 손목 트위스트(joint 6) — 2026-07-16 에 BAG_EULER 가 (90,0,90) 이 되면서 0 으로
# 제거됐던 값이다. 실배치에서 봉투를 (90,0,0) 으로 되돌리면 두께축이 world X ->
# Y 로 옮겨가므로 그리퍼 닫힘축도 같이 90도 돌려야 한다.
# 로봇은 삽입 전부터 압착 후까지 봉투를 계속 파지한다(사용자 지시 2026-08-28).
HOLD_THROUGH_CLAMP = os.environ.get("HOLD_THROUGH_CLAMP",
                                    "1" if LAYOUT_FROM_STEP else "0") == "1"
WRIST6_DEG = float(os.environ.get("WRIST6_DEG", "90" if LAYOUT_FROM_STEP else "0"))
if WRIST6_DEG:
    Q_GRASP[5] += np.radians(WRIST6_DEG)
    Q_LIFT[5] += np.radians(WRIST6_DEG)
    # **above/insert 의 IK 목표 자세도 같이 돌려야 한다(2026-08-28)**.
    # `q_insert_quat = VERTICAL_QUAT` 은 고정 상수라, 손목만 돌리면 파지는 ±90도로
    # 해놓고 IK 가 이송 중 손목을 VERTICAL_QUAT 으로 **되돌려** 봉투를 90도 비튼다.
    # 실측: 하강 중 봉투가 X 로 밀리는데 그 **부호가 손목 부호를 따라 뒤집혔다**
    # (+90 -> -9.4mm, -90 -> +10.2mm). 벽 간섭이면 부호가 안 바뀐다.
    # world Z 축 둘레로 같은 각을 곱해 파지 자세와 삽입 자세를 일치시킨다.
    _h = np.radians(WRIST6_DEG) / 2.0
    _qz = np.array([np.cos(_h), 0.0, 0.0, np.sin(_h)])          # (w,x,y,z)
    _a, _b = _qz, VERTICAL_QUAT
    VERTICAL_QUAT = np.array([
        _a[0]*_b[0] - _a[1]*_b[1] - _a[2]*_b[2] - _a[3]*_b[3],
        _a[0]*_b[1] + _a[1]*_b[0] + _a[2]*_b[3] - _a[3]*_b[2],
        _a[0]*_b[2] - _a[1]*_b[3] + _a[2]*_b[0] + _a[3]*_b[1],
        _a[0]*_b[3] + _a[1]*_b[2] - _a[2]*_b[1] + _a[3]*_b[0]])
FING_OPEN, FING_CLOSE = 1.00, 1.20

# 슬롯(약 (-0.33,-0.05,0.09))에서 0.87m 떨어진 원래 위치((0,0.7,0))는 orientation
# 고정 IK 오차가 12cm 까지 났다 — 슬롯에 훨씬 가까운 위치로 재배치(오차 <0.001m
# 확인, 이 스크립트 개발 중 격리 테스트로 검증).
ROBOT_OFFSET = np.array([-0.2250, 0.2500, 0.0]) if LAYOUT_FROM_STEP else np.array([-0.330, -0.65, 0.0])
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
# **2026-08-28, STEP 실배치 전환**: 위 논리는 슬롯 간격축이 world X 일 때(구
# CRUSHER_EULER=(0,0,90)) 성립한다. 실배치(EULER 0)에서는 간격이 world Y(12.5mm),
# 슬롯 길이가 world X(65mm)로 **정확히 뒤바뀌므로** 봉투도 되돌려야 한다:
#   (90,0,90) -> X=6(두께)  Y=64(폭)   : 구 배치용
#   (90,0, 0) -> X=64(폭)   Y=6(두께)  : 실배치용  <- 두께가 간격축 Y 로
# 되돌리지 않으면 폭 64mm 가 12.5mm 틈에 들어가려다 구겨진다(실측: insert tilt
# 22.8deg, 높이 90.1 -> 76.1mm, 오차 +25.0mm FAIL).
BAG_EULER = (90, 0, 0) if LAYOUT_FROM_STEP else (90, 0, 90)
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
# 로컬 폭축이 매핑되는 world 축이 BAG_EULER 에 따라 바뀐다(구: Y / 실배치: X).
_SEAL_ON_X = LAYOUT_FROM_STEP
BAG_POS = (FINGER_MID[0] - (SEAL_LOCAL_X if _SEAL_ON_X else 0.0),
           FINGER_MID[1] - (0.0 if _SEAL_ON_X else SEAL_LOCAL_X),
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
if CLAMP_MODE == "velocity":
    # 열림(+6mm)에서 하드스톱까지 전 구간을 닫고도 남을 시간 + 스톨 관찰 여유(x1.5).
    # 위치제어처럼 램프를 다 쓸 필요가 없고, 봉투에 걸리면 그 앞에서 멈춘다.
    N_CLAMP = int(round((WALL_OFFSET - WALL_Q_FLOOR) * 1e3 / WALL_CLOSE_MMPS * 1.5 / DT))

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
            constraint_strength_translation=IPC_CONSTRAINT_STRENGTH,
            constraint_strength_rotation=IPC_CONSTRAINT_STRENGTH,
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

    if LAYOUT_FROM_STEP:
        # 실판 1장 — 상면이 월드 z=0 이 되도록 중심을 -두께/2 에 둔다.
        scene.add_entity(
            gs.morphs.Box(size=PLATE_SIZE, pos=(0.0, 0.0, -PLATE_SIZE[2] / 2), fixed=True),
            material=gs.materials.Rigid(coup_type="ipc_only"),
            surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
        )
    else:
        for p in PLATE_POSITIONS:
            scene.add_entity(
                gs.morphs.Mesh(file=PLATE_PATH, fixed=True, pos=p),
                material=gs.materials.Rigid(coup_type="ipc_only"),
                surface=gs.surfaces.Default(color=(0.82, 0.82, 0.85), metallic=0.85, roughness=0.3),
            )

    crusher = scene.add_entity(
        gs.morphs.MJCF(file=crusher_xml, pos=CRUSHER_POS, euler=CRUSHER_EULER,
                       decimate=True, convexify=True),
        # coup_friction 미지정이면 기본 0.1(§15). 핑거는 CLOTH_FRICTION=0.8 로
        # 봉투를 잡는데 벽만 0.1 이던 비대칭을 없앤다 — 그리퍼 레시피 정렬의 일부.
        material=gs.materials.Rigid(coup_type="two_way_soft_constraint",
                                    coup_friction=CLOTH_FRICTION),
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
    # 회수장치2 만 IPC 커플러에서 뺀다(needs_coup=False, 2026-08-19). 구 에셋은
    # 모든 geom 이 contype/conaffinity=0 이라 Genesis 가 시각 geom 으로만 만들어
    # 커플러가 가져갈 충돌 메시가 0개였다 → coup_type 을 줘도 결과적으로 커플링
    # 밖이었다. 신 recovery2_mjcf 는 부품마다 `*_col`(원본 STL) 이 있어 그대로
    # 두면 링크 33개가 ABD 강체로 IPC 월드에 들어가는데, 그중 열린(watertight
    # 아닌) 메시가 있어 빌드가 uipc 에서 죽는다 — AffineBodyConstitution ->
    # compute_mesh_volume: "Calculating volume of open trimesh is meaningless"
    # (실측 확인, scene.build() 안에서 즉사). 어차피 로봇/봉투와 상호작용하지
    # 않는 정적 소품이므로 커플러에서 제외하는 게 예전 거동과도 일치한다 —
    # 렌더와 강체 솔버에는 그대로 남는다.
    scene.add_entity(
        gs.morphs.MJCF(file=RECOVERY2_MJCF, pos=RECOVERY2_POS, decimate=False),
        material=gs.materials.Rigid(needs_coup=False),
    )
    scene.add_entity(
        gs.morphs.MJCF(file=SUCTION_MJCF, pos=SUCTIONV1_POS, euler=SUCTIONV1_EULER, decimate=False),
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

    # CLAMP_ONLY 는 봉투를 **처음부터 슬롯 안에** 스폰한다. 나중에 옮기는 것은
    # 불가능하다 — IPC 커플러가 매 스텝 uipc 의 상태를 FEM 엔티티로 되쓰기 때문에
    # (coupler.py:1041 `entity.set_pos(0, geom_positions)`) Genesis 쪽
    # `set_position` 은 다음 스텝에 그대로 덮인다(실측: 이동량 303mm 가 통째로
    # 무시됨). 스폰 좌표는 morph 가 적용하는 변환을 역산해 만든다:
    #     world = R(euler) @ (verts * scale) + pos
    _bag_spawn_pos = BAG_POS
    if CLAMP_ONLY:
        import genesis.utils.geom as gu
        _sg = slot_geometry()
        _bv = tm.load(bag_obj).vertices * BAG_SCALE
        _bw = (gu.quat_to_R(gu.xyz_to_quat(np.array(BAG_EULER), rpy=True, degrees=True))
               @ _bv.T).T
        _lo, _hi = _bw.min(0), _bw.max(0)
        _bag_spawn_pos = (float(_sg["gap_cx"] - (_lo[0] + _hi[0]) / 2),
                          float(_sg["gap_cy"] - (_lo[1] + _hi[1]) / 2),
                          float(_sg["wall_center_z"] - _lo[2]))
        print(f"[clamp_only] 봉투를 슬롯에 직접 스폰: pos={np.round(_bag_spawn_pos, 4)} "
              f"(기본 {BAG_POS}) — 슬롯 중심 ({_sg['gap_cx']:.4f},{_sg['gap_cy']:.4f}), "
              f"bag_bottom 목표 {_sg['wall_center_z']:.4f}")

    bag = scene.add_entity(
        material=gs.materials.FEM.Cloth(
            E=CLOTH_E, nu=CLOTH_NU, rho=CLOTH_RHO,
            thickness=CLOTH_THICK, bending_stiffness=CLOTH_BEND,
            friction_mu=CLOTH_FRICTION,
        ),
        morph=gs.morphs.Mesh(file=bag_obj, scale=BAG_SCALE, pos=_bag_spawn_pos, euler=BAG_EULER),
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

    # 플랜지가 정말 접촉에서 빠졌는지 **빌드 후 실물로** 확인한다. MJCF 를 고쳤다는
    # 것만으로는 부족하다 — Genesis 가 contype/conaffinity 를 어떻게 해석했는지가
    # 답이고, IPC 커플러가 도는 것은 link.geoms(충돌) 뿐이다(coupler.py:392).
    # 기대: 플랜지 접촉 ON = 본체 hull 5 + 플랜지 1 = 6, OFF = 5. LAYOUT_ONLY=1 로
    # 시뮬 없이 이 한 줄만 뽑아 볼 수 있게 종료 분기보다 위에 둔다.
    try:
        _lw = crusher.get_link("L2_Left_Wall1_1")
        print(f"[verify] L2_Left_Wall1_1 충돌 geom {len(_lw.geoms)}개 / 시각 geom "
              f"{len(_lw.vgeoms)}개  (플랜지 접촉="
              f"{'ON' if LEFTWALL_FLANGE_CONTACT else 'OFF'}, "
              f"기대 충돌 {6 if LEFTWALL_FLANGE_CONTACT else 5}개)")
        if not LEFTWALL_FLANGE_CONTACT and len(_lw.geoms) != LEFTWALL_BODY_HULL_N:
            print("[verify] **경고**: 충돌 geom 수가 기대와 다르다 — "
                  "플랜지가 여전히 접촉에 남아 있을 수 있다.")
    except Exception as _e:      # 링크 이름이 바뀌어도 런 자체를 죽이지는 않는다
        print(f"[verify] Left_Wall 링크 확인 실패(무시하고 진행): {_e}")

    if LAYOUT_ONLY:
        # 실설계 배치만 확인하는 모드 — 시뮬 없이 정지 프레임만 뽑고 끝낸다.
        _out = os.path.join(CASE_DIR, f"layout_{_TS}")
        cam_over.set_pose(pos=(1.30, -1.30, 0.95), lookat=(0.0, -0.10, 0.10))
        cam_over.render(rgb=True)[0]
        import PIL.Image as _I
        for _tag, _pos, _look in (
            ("iso",  (1.30, -1.30, 0.95), (0.00, -0.10, 0.10)),
            ("top",  (0.00,  0.00, 1.80), (0.00,  0.00, 0.00)),
            ("front",(0.00, -1.90, 0.45), (0.00,  0.00, 0.15)),
            ("side", (1.90,  0.00, 0.45), (0.00,  0.00, 0.15)),
        ):
            cam_over.set_pose(pos=_pos, lookat=_look)
            _I.fromarray(cam_over.render(rgb=True)[0]).save(f"{_out}_{_tag}.png")
            print(f"[layout] saved {_out}_{_tag}.png")
        print(f"[layout] 판 {PLATE_SIZE[0]*1000:.0f}x{PLATE_SIZE[1]*1000:.0f}x"
              f"{PLATE_SIZE[2]*1000:.0f}mm, 상면 z=0")
        for _n, _p in (("Crusher", CRUSHER_POS), ("고정장치", FIXTURE_POS),
                       ("회수장치2", RECOVERY2_POS), ("석션V1", SUCTIONV1_POS),
                       ("로봇", tuple(ROBOT_OFFSET))):
            print(f"[layout]   {_n:10s} ({_p[0]:+.4f}, {_p[1]:+.4f}, {_p[2]:+.4f})")
        print("[layout] 완료 — 배치 확인 모드라 시뮬은 건너뛴다.")
        return
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
    _slot = slot_geometry()
    wb_lo, wb_hi = _slot["wb_lo"], _slot["wb_hi"]
    GAP_AX, OTH_AX = _slot["gap_ax"], _slot["oth_ax"]
    gap_width, gap_cx, gap_cy = _slot["gap_width"], _slot["gap_cx"], _slot["gap_cy"]
    wall_top_z = _slot["wall_top_z"]
    _ov = _slot["overlap"]
    print(f"[slot] 간격축 = world {'XY'[GAP_AX]} (겹침 {_ov[GAP_AX]*1000:+.1f}mm), "
          f"슬롯 길이축 = world {'XY'[OTH_AX]} (겹침 {_ov[OTH_AX]*1000:+.1f}mm)")
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
    # 반사 관성은 **런타임 API 로 건다**(rigid_entity.py:3982). MJCF 의 armature
    # 속성으로도 걸어 두지만 그게 실제로 솔버까지 갔는지 확인한 적이 없다 —
    # 2026-08-31 밤 스윕의 "관성을 키워도 관통한다"는 결론이 사실은 값이 무시된
    # 결과였을 수 있어서, 무시될 여지가 없는 경로로 한 번 더 건다.
    if WALL_ARMATURE > 0:
        crusher.set_dofs_armature(np.array([WALL_ARMATURE]), dofs_idx_local=[wall_dof])
        print(f"[ctrl] Left_Wall armature={WALL_ARMATURE:.1f}kg (런타임 API) — "
              f"유효 질량 {0.312+WALL_ARMATURE:.2f}kg, {WALL_FORCE_LIM:.0f}N -> "
              f"{WALL_FORCE_LIM/(0.312+WALL_ARMATURE):.1f} m/s^2, "
              f"kv 안정 한계 {2*(0.312+WALL_ARMATURE)/DT:.0f} (현재 {WALL_KV:.0f})")

    # ── Motor2(Left_Wall) 힘 상한 — 발산 방지의 근본 처방(2026-08-25) ────────
    # 힘 제한이 없으면 WALL_KP=5000 위치제어가 IPC 배리어(간격→0 에서 반력이
    # 무한대로 커짐)를 상대로 무한한 힘을 낼 수 있어 되먹임이 발산한다. 실측:
    #   CLAMP_TARGET=-12.0mm -> 배리어 폭발, 44분 미완(스텝당 ~4.7s)
    #   CLAMP_TARGET=-10.4mm -> 1차 런은 -9.21mm 에서 균형(단 release 24s/step),
    #                            2차 런은 clamp 에서 **발산**(wall=-2.09e9 mm).
    #                            같은 설정이 갈린다 = 안정 한계 위에 걸친 값(§13-11).
    # 실기 Motor2 는 그런 힘을 못 낸다 — docs/Crusher.md §5 `Motor2_left_wall`
    # motor, **ctrlrange ±100 N** (MJCF actuatorfrcrange 와 일치). Genesis 의
    # control_dofs_position 경로는 MJCF 의 그 값을 적용하지 않으므로 직접 건다.
    # 힘을 제한하면 벽은 봉투 반력과 균형지는 곳에서 **물리적으로** 멈추고
    # 발산할 수 없다 — §11-5 의 "래칫 lock 없이 모터를 계속 구동해 강하게 고정"이
    # 정확히 이 거동이다. 그래서 CLAMP_TARGET 을 깊게 줘도 안전해지고, 정지
    # 위치가 곧 봉투의 실효 압착 두께 실측값이 된다.
    # (Crusher_only.py 가 크랭크에 대해 같은 처방을 이미 쓴다 — 318~327행)
    _fmin = np.full(crusher.n_dofs, -np.inf)
    _fmax = np.full(crusher.n_dofs,  np.inf)
    _fmin[wall_dof], _fmax[wall_dof] = -WALL_FORCE_LIM, WALL_FORCE_LIM
    crusher.set_dofs_force_range(lower=_fmin, upper=_fmax)
    print(f"[ctrl] Left_Wall force clip: ±{WALL_FORCE_LIM:.0f} N (Motor2 ctrlrange, "
          f"docs/Crusher.md §5) — 벽은 이 힘에서 균형지는 위치에 멈춘다")

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
    if not NO_VIDEO:
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
        if NO_VIDEO:
            return
        cam_over.render()
        bc = _bag_com()
        cam_bag.set_pose(pos=tuple(bc + BAGCAM_OFFSET), lookat=tuple(bc), up=(0, 0, 1))
        cam_bag.render()
        cam_side.render()

    # CLAMP_ONLY 는 Phase 1~8b 의 팔 구동만 건너뛴다 — clamp 뒤의 Phase 10
    # (release/hold2)은 관찰 대상이므로 clamp 직후 이 플래그를 내린다.
    _skip_arm = [CLAMP_ONLY]

    def run_arm(name, q0, q1, f0, f1, n, crank_q=None, wall_q=None, trace=False):
        if _skip_arm[0]:    # 스텝 없이 최종 자세만 확정 — 시뮬 시간을 안 쓴다
            robot.set_dofs_position(np.concatenate([q1, [f1] * 6]))
            print(f"[phase] {name:8s} @skip  (CLAMP_ONLY)")
            return
        for k in range(n):
            s = ease((k + 1) / n)
            q = q0 + (q1 - q0) * s
            f = f0 + (f1 - f0) * s
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                if WALL_KINEMATIC and name in ("release", "hold2"):
                    crusher.set_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
                else:
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
        if _skip_arm[0]:
            robot.set_dofs_position(np.concatenate([q_way[-1], [f] * 6]))
            print(f"[phase] {name:8s} @skip  (CLAMP_ONLY)")
            return
        for k in range(n):
            u = ease((k + 1) / n) * m
            i = min(int(u), m - 1)
            q = q_way[i] + (q_way[i + 1] - q_way[i]) * (u - i)
            robot.set_dofs_position(np.concatenate([q, [f] * 6]))
            if crank_q is not None:
                crusher.control_dofs_position(np.array([crank_q]), dofs_idx_local=[crank_dof])
            if wall_q is not None:
                if WALL_KINEMATIC and name in ("release", "hold2"):
                    crusher.set_dofs_position(np.array([wall_q]), dofs_idx_local=[wall_dof])
                else:
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
    for k in range(0 if CLAMP_ONLY else N_PREP):
        s = (k + 1) / N_PREP
        crusher.control_dofs_position(np.array([CRANK_START_Q * s]), dofs_idx_local=[crank_dof])
        crusher.control_dofs_position(np.array([WALL_OFFSET * s]), dofs_idx_local=[wall_dof])
        robot.set_dofs_position(np.concatenate([q_grasp, [FING_OPEN] * 6]))
        scene.step()
        _step_n[0] += 1
        render_cams()
    if CLAMP_ONLY:      # 램프 없이 슬롯 준비 상태로 바로 놓는다
        crusher.set_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        crusher.set_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
    cq = _npy(crusher.get_dofs_position())[crank_dof]
    wq = _npy(crusher.get_dofs_position())[wall_dof]
    print(f"[phase] prep     @done  crank={cq:+.3f}rad  wall={wq*1000:+.2f}mm")

    # ── Phase 1-6: 정제 낙하 -> 봉투 파지 -> 리프트 (tablet_bag_grasp_pipeline.py 동일) ──
    run_arm("drop", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_DROP,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET, trace=True)
    run_arm("settle", q_grasp, q_grasp, FING_OPEN, FING_OPEN, N_SETTLE,
            crank_q=CRANK_START_Q, wall_q=WALL_OFFSET)

    # 정제가 안전하게 들어간 뒤 형상 고정 해제 — 이후 grasp/lift 는 순수 마찰로 진행.
    if not CLAMP_ONLY:      # CLAMP_ONLY 는 봉투를 슬롯에 놓을 때까지 고정을 유지한다
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

    if CLAMP_ONLY:
        # 봉투는 이미 슬롯에 스폰돼 있다(위 _bag_spawn_pos). 여기서는 형상 고정을
        # 풀고 **상단만** 다시 구속해 그리퍼 대신 매단 뒤 잠깐 안정화한다.
        bag.remove_vertex_constraints()
        vp = _npy(bag.get_state().pos).squeeze()
        if CLAMP_ONLY_PIN:
            # 그리퍼 대신 상단 정점을 매단다. **IPC 커플러와 궁합이 나쁘다** —
            # 슬롯 안에서 걸면 settle 100 스텝이 5분 넘게 진행이 없다(실측
            # 2026-08-31). 커플러가 매 스텝 uipc 상태를 되쓰는 것과 구속이 서로
            # 싸우는 것으로 보인다. 기본은 꺼두고, 봉투는 포켓 바닥에 얹어 둔다.
            top_idx = np.where(vp[:, 2] > vp[:, 2].max() - 0.010)[0].astype(int)
            bag.set_vertex_constraints(verts_idx_local=top_idx.tolist(), is_soft_constraint=False)
            print(f"[clamp_only] 상단 {len(top_idx)}/{len(vp)} 정점 구속(그리퍼 대체)")
        else:
            print("[clamp_only] 정점 구속 없음 — 봉투는 포켓 안에 자유롭게 놓인다")
        for _ in range(N_SETTLE2):
            crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
            crusher.control_dofs_position(np.array([WALL_OFFSET]), dofs_idx_local=[wall_dof])
            scene.step()
            _step_n[0] += 1
            render_cams()
        _w, _h, _b = _bag_extent()
        print(f"[clamp_only] settle 후 bag_com={_bag_com()}  tilt={_bag_tilt():.1f}deg  "
              f"bag_bottom={_b:.4f} (목표 {wall_center_z:.4f}, 오차 "
              f"{(_b-wall_center_z)*1e3:+.1f}mm)  폭 {_w*1e3:.1f}mm 높이 {_h*1e3:.1f}mm")
        if _h > 0.150 or _w < 0.003:
            raise RuntimeError(
                f"[clamp_only] 봉투 형상이 깨졌다 — 폭 {_w*1e3:.1f}mm 높이 {_h*1e3:.1f}mm. "
                f"압착을 재도 의미가 없다.")


    # ── Phase 9: clamp — Left_Wall(Motor2, Rack&Pinion) 닫아 실링부 고정 ─────
    # docs/Crusher.md §11-5: 래칫/락 없이 모터를 계속 구동해 강하게 고정하는
    # 방식 — WALL_OFFSET(+6mm)에서 CLAMP_TARGET(-12mm)까지 완전 폐쇄를 지령하되,
    # 실제 정지 위치는 WALL_FORCE_LIM(±100N)이 결정한다 = 실효 압착 두께 실측.
    # 잔여 간격 = LEFTWALL_GAP_FLANGE + CLAMP_TARGET 이라 -5mm 는 7.5mm 를 남겨
    # 6mm 봉투를 스치기만 했다 — 상단 CLAMP_TARGET 정의부의 실측 근거 참고.
    _wall_q = lambda: float(_npy(crusher.get_dofs_position())[wall_dof])
    _wall_v = lambda: float(_npy(crusher.get_dofs_velocity())[wall_dof])
    if CLAMP_MODE == "velocity":
        # **속도원(velocity source) + 힘 상한을, 지령 램프로 구현한다.**
        #
        # `control_dofs_velocity` + 큰 kv 를 직접 쓰면 터진다(2026-08-31 실측):
        # 자유 구간에서는 -8.00mm/s 를 정확히 따라갔는데, 잔여 6.92mm 에서 봉투에
        # 처음 닿는 순간 v 가 -11.5 -> -27.3 -> -10,003mm/s 로 폭주했다. 명시적
        # 감쇠의 안정 조건이 kv*dt/m <~ 2 인데 kv=5000, dt=5ms, m=0.312kg 이면
        # 80 이라 접촉 충격 한 번에 발산 영역으로 넘어간다. 크랭크의
        # CRANK_KV_SPIN=5000 이 멀쩡한 것은 회전 관성이라 단위가 다르기 때문이다.
        #
        # 대신 검증된 위치제어 경로(kp=5000/kv=500)를 그대로 두고 **지령만**
        # 속도로 움직인다. 핵심은 지령이 실제 위치보다 lead_max 이상 앞서지
        # 못하게 묶는 것이다:
        #     lead_max = WALL_FORCE_LIM / WALL_KP = 100N / 5000 N/m = 20mm
        # 이러면 자유 구간에서는 8mm/s 로 따라가고, 봉투에 막히면 지령이 20mm
        # 앞에서 포화해 **정확히 100N 으로 계속 미는 상태**가 된다 — 실기의
        # "모터를 계속 구동해 강하게 고정"(§11-5)과 같은 거동이고, 도달 불가능한
        # 목표를 배리어에 밀어붙이던 위치 램프의 문제도 사라진다.
        _lead_max = WALL_FORCE_LIM / WALL_KP
        _v_step = WALL_CLOSE_MMPS * 1e-3 * DT
        _q_cmd = WALL_OFFSET
        print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.1f}s) — Left_Wall {WALL_OFFSET*1000:+.1f}mm 에서 "
              f"**속도지령** {WALL_CLOSE_MMPS:.1f}mm/s 로 계속 닫음 (docs §11-5), "
              f"lead {_lead_max*1e3:.1f}mm = 정지 시 {WALL_FORCE_LIM:.0f}N, "
              f"가드 {WALL_Q_FLOOR*1e3:+.1f}mm")
    else:
        print(f"\n[phase] 9 clamp ({N_CLAMP*DT:.1f}s) — Left_Wall {WALL_OFFSET*1000:+.1f}mm -> "
              f"{CLAMP_TARGET*1000:+.1f}mm (실링부 압착)")

    # 무진행 정체와 정상 스톨을 로그에서 구분할 수 있어야 한다 — 예전 런들이
    # clamp 구간에 출력이 없어 20분을 기다린 뒤에야 이상을 알았다.
    _clamp_log = max(1, N_CLAMP // 25)
    for k in range(N_CLAMP):
        if WALL_KINEMATIC:
            # 지령을 v 만큼 전진시키고 그 위치를 그대로 써넣는다. CLAMP_TARGET
            # 에서 멈춘다 — 접촉이 벽을 되밀 수 없으므로 lead 제한이 필요 없다.
            _q_cmd = max(_q_cmd - _v_step, CLAMP_TARGET)
            crusher.set_dofs_position(np.array([_q_cmd]), dofs_idx_local=[wall_dof])
        elif CLAMP_MODE == "velocity":
            # 지령을 v 만큼 전진시키되 (a) 실제 위치보다 lead_max 이상 앞서지 않고
            # (b) 기계적 하드스톱 가드를 넘지 않게 묶는다. max = 덜 음수인 쪽.
            # **단조 감소**로 묶는다 — 벽이 +방향으로 튕기면 lead 항이 그 폭주를
            # 따라 올라가 영영 되돌아오지 못한다(실측: c1_arm10 에서 wall +244mm).
            _q_cmd = min(_q_cmd, max(_q_cmd - _v_step, _wall_q() - _lead_max, WALL_Q_FLOOR))
            crusher.control_dofs_position(np.array([_q_cmd]), dofs_idx_local=[wall_dof])
        else:
            s = (k + 1) / N_CLAMP
            wq = WALL_OFFSET + (CLAMP_TARGET - WALL_OFFSET) * s
            crusher.control_dofs_position(np.array([wq]), dofs_idx_local=[wall_dof])
        crusher.control_dofs_position(np.array([CRANK_START_Q]), dofs_idx_local=[crank_dof])
        robot.set_dofs_position(np.concatenate([qpos_insert, [FING_CLOSE] * 6]))
        scene.step()
        _step_n[0] += 1
        render_cams()
        if k % _clamp_log == 0:
            _q = _wall_q()
            print(f"    [clamp t={k*DT:5.2f}s] wall={_q*1e3:+7.2f}mm "
                  f"v={_wall_v()*1e3:+7.2f}mm/s  본체면 잔여 "
                  f"{(LEFTWALL_GAP_BODY + _q)*1e3:+6.2f}mm  bag_com={_bag_com()}")
    wq_final = _npy(crusher.get_dofs_position())[wall_dof]
    # 발산 검증(2026-08-25) — 2차 -10.4mm 런에서 벽 DOF 가 -2.09e9 mm 로 터졌는데
    # 그 값이 그대로 release 의 wall_q 로 흘러들어가 다음 페이즈까지 망가뜨렸다.
    # 물리적으로 가능한 범위는 [CLAMP_TARGET - 1mm, WALL_OFFSET + 1mm] 뿐이다.
    # 여유 5mm — 1mm 로 잡았더니 PD 오버슛(목표 -10.0 에 -11.36 도달)을 발산으로
    # 오탐했다. 실제 발산은 +625mm / -2.09e9mm 규모라 5mm 로도 충분히 걸러진다.
    _wq_deep = WALL_Q_FLOOR if CLAMP_MODE == "velocity" else CLAMP_TARGET
    _wq_lo, _wq_hi = min(_wq_deep, 0.0) - 0.005, max(WALL_OFFSET, 0.0) + 0.005
    if not (np.isfinite(wq_final) and _wq_lo <= wq_final <= _wq_hi):
        raise RuntimeError(
            f"[clamp] Left_Wall DOF 발산: wall={wq_final*1000:.2f}mm "
            f"(허용 {_wq_lo*1000:+.1f}~{_wq_hi*1000:+.1f}mm). "
            f"WALL_FORCE_LIM={WALL_FORCE_LIM}N 로도 안정화되지 않았다 — "
            + (f"WALL_CLOSE_MMPS({WALL_CLOSE_MMPS:.1f}mm/s)를 낮추거나 힘 상한을 낮출 것."
               if CLAMP_MODE == "velocity" else
               f"CLAMP_TARGET({CLAMP_TARGET*1000:+.1f}mm)을 줄이거나 힘 상한을 낮출 것.")
        )
    # 잔여 간격은 **면마다 다르다**(§18-2): 상단 플랜지와 본체 면이 5mm 어긋나
    # 서로 다른 값에서 출발한다. 하나만 찍으면 다른 쪽을 오해하므로 둘 다 낸다.
    # 출발값은 LEFTWALL_GAP_* 가 쥐고 있다 — LEFTWALL_SPLIT(신 CAD, 12.50/17.50)
    # 인지 LEFTWALL_CLAMP_FACE(본체 면을 플랜지 평면까지 메움)인지에 따라 다르다.
    _g_flange = (LEFTWALL_GAP_FLANGE + wq_final) * 1000
    _g_body = LEFTWALL_GAP_BODY + wq_final
    # 플랜지 접촉을 끄면 플랜지면 잔여는 **참고값**이다 — 그 면은 이제 아무것도
    # 안 누른다. 음수로 내려가도 정상이며(형상이 봉투를 통과해 지나간다) 압착을
    # 판단할 면은 본체면 하나뿐이다.
    _fl_tag = "" if LEFTWALL_FLANGE_CONTACT else "(비접촉)"
    print(f"[phase] clamp    @done  wall={wq_final*1000:+.2f}mm  "
          f"(잔여: 플랜지면{_fl_tag} {_g_flange:.2f}mm / "
          f"본체면 {_g_body*1000:.2f}mm)  bag_com={_bag_com()}")
    if CLAMP_MODE == "velocity":
        # 속도제어에서는 "어디서 멈췄나" 가 곧 측정값이다. 가드까지 내려갔다면
        # 봉투가 힘 상한을 세우지 못한 것 = 못 물었다는 뜻이다.
        _stalled = wq_final > WALL_Q_FLOOR + 1e-4
        print(f"[clamp] {'봉투 반력에 막혀 정지' if _stalled else '**하드스톱까지 내려감 — 봉투를 못 물었다**'}"
              f"  (가드 {WALL_Q_FLOOR*1e3:+.2f}mm, 최종 v={_wall_v()*1e3:+.2f}mm/s)")

    # ── Phase 10: release — 그리퍼 개방, 고정은 이제 Left_Wall 이 담당 ───────
    # **wall_q 를 CLAMP_TARGET 이 아니라 실제 도달 위치로 준다(2026-08-25)**.
    # clamp 는 목표를 램프로 올려 마지막 스텝에만 CLAMP_TARGET 을 지령하지만,
    # release 는 100 스텝 내내 그 값을 유지한다. 벽은 봉투 반력과 균형지는
    # 지점(-9.21mm, 잔여 2.79mm)에서 이미 멈춰 있는데 -10.4mm 를 계속 밀면
    # 도달 불가능한 1.19mm 를 매 스텝 IPC 배리어에 밀어붙이는 꼴이라, 실측에서
    # 스텝당 >24초로 폭발했다(16분에 40스텝 미만). 도달 위치에서 WALL_PRELOAD
    # 만큼만 더 눌러 파지력(kp x 0.5mm = 2.5N)은 유지하되 배리어와 싸우지 않는다.
    WALL_PRELOAD = 0.0005
    # 로봇은 삽입 전부터 압착 후까지 봉투를 계속 파지한다(사용자 지시 2026-08-28).
    # 그리퍼를 열면 봉투가 포켓으로 흘러내린다(실측 -34.9mm) — 클램프가 플랜지
    # 한 줄로만 닿아 사실상 고정력이 없기 때문(플랜지 높이엔 마주보는 고정벽이
    # 없다, §18-2). 놓지 않으면 그 실패모드 자체가 사라진다.
    _bottom_at_clamp = _bag_extent()[2]      # 릴리스 전후 낙하량 비교 기준
    _skip_arm[0] = False        # Phase 10 부터는 CLAMP_ONLY 에서도 실제로 돌린다
    _fing_end = FING_CLOSE if HOLD_THROUGH_CLAMP else FING_OPEN
    # 속도지령 모드면 WALL_PRELOAD 라는 개념이 없다 — clamp 끝의 지령 _q_cmd 가
    # 이미 실제 위치보다 lead_max 앞서 포화해 있으므로, 그대로 유지하면 계속
    # 힘 상한으로 눌러 고정한다(§11-5). 새로 계산할 것이 없다.
    _wall_hold = _q_cmd if CLAMP_MODE == "velocity" else wq_final - WALL_PRELOAD
    if WALL_KINEMATIC:
        _wall_hold = _q_cmd
    _wall_kw = dict(wall_q=_wall_hold)
    run_arm("hold2" if HOLD_THROUGH_CLAMP else "release",
            qpos_insert, qpos_insert, FING_CLOSE, _fing_end, N_RELEASE,
            crank_q=CRANK_START_Q, trace=True, **_wall_kw)

    # ── Phase 11: crush — 벽이 문 상태로 Crusher 를 실제 운전 ─────────────────
    # 근거·사양은 CRUSH_SECONDS 정의부 주석. 기본은 꺼져 있고 CRUSH_SECONDS 로 켠다.
    if CRUSH_SECONDS > 0:
        n_crush = int(round(CRUSH_SECONDS / DT))
        wall_hold = _wall_hold      # 속도지령 모드면 포화된 지령, 아니면 프리로드

        def _tablet_extent():
            """정제 AABB (dx, dy, dz) mm — 분쇄 진행도의 대리 지표."""
            if tablet is None:
                return (float("nan"),) * 3
            p = _npy(tablet.get_state().pos).squeeze()
            return tuple((p.max(0) - p.min(0)) * 1e3)

        # 크랭크를 position -> velocity 제어로 바꾸고 토크를 실기 한계로 묶는다.
        # 벽의 force_range 는 그대로 둬야 하므로 배열을 새로 만들어 둘 다 건다.
        crusher.set_dofs_kv(np.array([CRANK_KV_SPIN]), dofs_idx_local=[crank_dof])
        _fmin2 = np.full(crusher.n_dofs, -np.inf)
        _fmax2 = np.full(crusher.n_dofs,  np.inf)
        _fmin2[wall_dof], _fmax2[wall_dof] = -WALL_FORCE_LIM, WALL_FORCE_LIM
        _fmin2[crank_dof], _fmax2[crank_dof] = -CRANK_TORQUE_LIM, CRANK_TORQUE_LIM
        crusher.set_dofs_force_range(lower=_fmin2, upper=_fmax2)

        q0_crank = float(_npy(crusher.get_dofs_position())[crank_dof])
        e0 = _tablet_extent()
        n_rev = CRUSH_SECONDS * CRANK_OMEGA / (2 * np.pi)
        print(f"\n[phase] 11 crush ({CRUSH_SECONDS:.0f}s, {n_crush} step) — 크랭크 "
              f"{CRANK_RPM:.1f} RPM ({CRANK_OMEGA:.4f} rad/s) 연속 회전 ≈ {n_rev:.1f} 바퀴")
        print(f"[ctrl] crank torque clip: ±{CRANK_TORQUE_LIM:.1f} N·m "
              f"(docs/Crusher.md §2-2), kv={CRANK_KV_SPIN:.0f} — velocity 제어")
        print(f"[crush] 시작 정제 크기 {e0[0]:.2f} x {e0[1]:.2f} x {e0[2]:.2f} mm")

        _log_every = max(1, n_crush // 30)
        for k in range(n_crush):
            # 팔은 release 자세 그대로 유지(핑거 z=0.134 는 impact plate 의 z대역
            # 0.024~0.074 보다 위라 간섭하지 않는다 — 회피 동작을 넣지 않는 이유).
            robot.set_dofs_position(np.concatenate([qpos_insert, [FING_OPEN] * 6]))
            crusher.control_dofs_velocity(np.array([CRANK_OMEGA]), dofs_idx_local=[crank_dof])
            crusher.control_dofs_position(np.array([wall_hold]), dofs_idx_local=[wall_dof])
            scene.step()
            _step_n[0] += 1
            if k % CRUSH_RENDER_EVERY == 0:
                render_cams()
            if k % _log_every == 0:
                qc = float(_npy(crusher.get_dofs_position())[crank_dof])
                vc = float(_npy(crusher.get_dofs_velocity())[crank_dof])
                qw = float(_npy(crusher.get_dofs_position())[wall_dof])
                e = _tablet_extent()
                print(f"    [crush t={k*DT:5.1f}s] crank={np.degrees(qc-q0_crank):+8.1f}deg "
                      f"({vc*60/(2*np.pi):5.2f} RPM)  wall={qw*1e3:+6.2f}mm  "
                      f"정제 {e[0]:5.2f}x{e[1]:5.2f}x{e[2]:5.2f}mm")

        qc = float(_npy(crusher.get_dofs_position())[crank_dof])
        e1 = _tablet_extent()
        print(f"[phase] crush    @done  크랭크 {np.degrees(qc-q0_crank):+.1f}deg "
              f"({(qc-q0_crank)/(2*np.pi):.2f} 바퀴)  wall={_npy(crusher.get_dofs_position())[wall_dof]*1e3:+.2f}mm")
        print(f"[crush] 정제 {e0[0]:.2f}x{e0[1]:.2f}x{e0[2]:.2f} -> "
              f"{e1[0]:.2f}x{e1[1]:.2f}x{e1[2]:.2f} mm  "
              f"(변화 {e1[0]-e0[0]:+.2f}/{e1[1]-e0[1]:+.2f}/{e1[2]-e0[2]:+.2f})")
        print(f"[crush] bag_com={_bag_com()}  tilt={_bag_tilt():.1f}deg  "
              f"bag_bottom={_bag_extent()[2]:.4f}")

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

    # ── 스윕용 한 줄 요약 ─────────────────────────────────────────────────────
    # 파라미터 스윕은 로그 수십 개를 사람이 읽는 게 아니라 이 줄만 긁어서 표로
    # 만든다. **압착이 성립했는지의 핵심 지표는 drop** 이다 — 그리퍼를 놓은 뒤
    # 봉투가 얼마나 흘러내렸는가(§18-4 의 실패 실측 -34.9mm).
    _drop_mm = (final_bottom_z - _bottom_at_clamp) * 1e3
    print("[SUMMARY] " + " ".join([
        f"flange={'ON' if LEFTWALL_FLANGE_CONTACT else 'OFF'}",
        f"mode={CLAMP_MODE}",
        f"v={WALL_CLOSE_MMPS:.1f}mm/s",
        f"F={WALL_FORCE_LIM:.0f}N",
        f"kp={WALL_KP:.0f}",
        f"hold={'1' if HOLD_THROUGH_CLAMP else '0'}",
        f"wall={wq_final*1e3:+.2f}mm",
        f"body_gap={(LEFTWALL_GAP_BODY + wq_final)*1e3:+.2f}mm",
        f"stalled={'Y' if (CLAMP_MODE == 'velocity' and wq_final > WALL_Q_FLOOR + 1e-4) else 'N'}",
        f"drop={_drop_mm:+.1f}mm",
        f"tilt={_bag_tilt():.1f}deg",
        f"width={final_width_y*1e3:.1f}mm",
        f"reach_err={(final_bottom_z - wall_center_z)*1e3:+.1f}mm",
        f"verdict={verdict}",
    ]))

    if not NO_VIDEO:
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
