# Crusher_Genesis — 프로젝트 노트 (2026-06-24 기준)

본 문서는 **Genesis 1.1.0** 위에 Doosan **M0609** + OnRobot **RG2** + Crusher
하드웨어를 디지털 트윈으로 옮기는 작업 기록이다. 새 세션에서 작업을 이어받을
때 이 파일과 `Twin.md`(평행 그리퍼 모델링), `requirements.txt`(env 명세)
세 가지를 먼저 읽고 시작한다.

---

## 1. 환경 (Windows 11 + RTX 5080)

| 항목 | 값 |
|---|---|
| Python | 3.13.14 (conda env `crusher_genesis`) |
| Genesis | 1.1.0 |
| torch | 2.11.0 + **cu128** (Blackwell sm_120 지원) |
| NVIDIA Driver | **595.79 (Studio DCH)** — Linux 의 `595.80-open` 등가 |
| Backend | `gs.init(backend=gs.cuda)` |
| 인터프리터 | `C:\Users\simuser\miniconda3\envs\crusher_genesis\python.exe` |

**중요**: RTX 5080 (Blackwell, sm_120) 은 드라이버 **595.79 이상** 필수. 더 낮은 드라이버에서는 quadrants(=Genesis 의 taichi fork) 커널 단계에서 access violation 으로 사망. Driver 업그레이드 후에야 `gs.cuda` 가 정상 작동.

설치 절차:
```powershell
& "$env:USERPROFILE\miniconda3\Scripts\conda.exe" create -n crusher_genesis python=3.13 -y
& "$env:USERPROFILE\miniconda3\envs\crusher_genesis\python.exe" -m pip install --index-url https://download.pytorch.org/whl/cu128 torch
& "$env:USERPROFILE\miniconda3\envs\crusher_genesis\python.exe" -m pip install genesis-world==1.1.0 numpy trimesh Pillow matplotlib scipy
```

실행 예:
```powershell
$py = "$env:USERPROFILE\miniconda3\envs\crusher_genesis\python.exe"
Set-Location C:\Crusher_isaacsim\Crusher_Genesis
& $py Crusher_Samplebag_headless.py
```

---

## 2. 정식 파일 셋 (사용자 합의)

정식 6+1 파일 (헤드리스는 그냥 `main(use_viewer=False)` 호출하는 thin wrapper):

| 파일 | 역할 | 상태 |
|---|---|---|
| `Crusher_only.py` / `_headless.py` | Crusher 단독, Motor1 PD velocity 회전, 크랭크-슬라이더 패시브 조인트 검증 | ✓ 동작 |
| `Samplebag.py` / `_headless.py` | PBD cloth + Rigid box 만, stretch 검증 | ✓ 동작 |
| `Crushing.py` / `_headless.py` | M0609+RG2 + 봉투 + Crusher 통합 (full pick&place) | ⚠ 봉투 grasp 시각 미흡 (RG2 핑거 mesh 문제) |
| `Crusher_Samplebag.py` / `_headless.py` | 로봇 없이 Crusher + 봉투 단독: carrier 로 봉투 하강 + Motor2 클램프 + Motor1 크랭크 | ✓ 동작 (방금 작업) |
| `RigidGrasp_headless.py` | **격리 테스트용** — Franka Panda + PBD 봉투 grasp 검증 (M0609+RG2 의 grasp 실패 원인 분리) | ✓ 성공 |
| `Twin.md` | 디지털 트윈 기법 노트 (사용자 문서) | 참조 필수 |

**사용자 정책**: 임시/디버그용 스크립트를 새로 만들지 않는다. 위 6+1 파일 안에서 수정·재실행하며 검증한다. 정말 새 파일이 필요할 땐 사용자 허락을 받는다. (`memory/feedback_consolidate_test_files.md`)

### 디렉토리 구조 + 경로 규약 (2026-07-08 재편)

```
Crusher_Genesis/
  config.json          ← 경로 원장 (루트 기준 상대경로, 유일한 경로 정의처)
  paths.py             ← config.json 로더 → 절대경로 상수 (MJCF_MAIN, ROBOT_M0609_RG2, …)
  assets/
    MJCF/              ← Crusher MJCF + STL (구 MJCF/)
    robots/            ← m0609_rg2.xml, rg2/, aluminum_plate 등 (구 robots/)
  Sim_result/          ← 시뮬 산출물 (mp4/png/csv)
  Real_result/         ← 실기 측정
  FEM/                 ← FEM/IPC 검증 스크립트
  legacy/              ← 이전 세션 잔재 (참고용, 실행 보장 없음)
  20260603/            ← 날짜 아카이브
  <정식 6+1 스크립트>  ← 루트
```

**경로 규약**: 스크립트는 asset 경로를 `__file__` 로 직접 계산하지 않는다.
상단에 아래 부트스트랩(위치 독립: config.json 을 상향 탐색)을 넣고 `paths.*` 를 쓴다:

```python
_r = os.path.dirname(os.path.abspath(__file__))
while _r != os.path.dirname(_r) and not os.path.exists(os.path.join(_r, "config.json")):
    _r = os.path.dirname(_r)
sys.path.insert(0, _r)
import paths   # paths.MJCF_MAIN, paths.ROBOT_M0609_RG2, paths.ALUMINUM_PLATE,
               # paths.SIM_RESULT, paths.REAL_RESULT, paths.TABLETS_STL, paths.asset(...)
```

새 asset 을 추가하면 `config.json` 에 항목을 넣고 `paths.py` 에 상수를 추가한다.

이전 세션의 잔재(`medicine_envelope_*.py`, `pbd_bag_test.py`, `vinyl_bag_model.py`)는 현 작업과 직접 관련 없어 **`legacy/` 폴더로 격리**함 (2026-07-08). 삭제 아님, git rename 으로 히스토리 보존. 이 스크립트들은 `__file__` 상대경로가 루트 기준이라 지금 위치에선 그대로 실행되지 않음(참고용 보존).

---

## 3. 시뮬 환경 구성 요소

### 알루미늄 플레이트 (1m × 1m × 2cm)
4장을 2×2 그리드로 배치 (총 2m × 2m 작업면):
```python
for p in [(0.5, -0.5, 0), (0.5, 0.5, 0), (-0.5, -0.5, 0), (-0.5, 0.5, 0)]:
    scene.add_entity(gs.morphs.Mesh(file=paths.ALUMINUM_PLATE, fixed=True, pos=p), ...)
```
플레이트 빠지면 박스 등 rigid 가 z=-∞로 자유낙하해 시뮬 폭주.

### Crusher (`assets/MJCF/Crusher_IsaacSim_colored.xml` = `paths.MJCF_MAIN`)
- `pos=(0.55, 0, 0)`, `euler=(0, 0, 90°)` — 매니퓰레이터 정면, 바닥 위
- 런타임 패치 (`patch_crusher_mjcf`) 필수:
  - `<equality><joint name="lock_crank" polycoef=...>` 제거 (Genesis 1.1.0 미지원)
  - `<weld>` solref/solimp 를 Genesis 양수 강한값으로 교체: `solref="0.0002 50"`, `solimp="0.999 0.99999 1e-5"`
  - `<geom name="ground">` 제거 (알루미늄 plate 와 중복)
  - 벽 geom (`L1_Wall1_1, L1_Wall2_1, L2_Wall3_1`) 의 `contype=0/conaffinity=0` 제거 → 충돌 ON
  - `L7_Link3_1` inertial CoM 을 bbox 중심 `(0.006, 0, -0.005)` 로 교체 (URDF 변환 오류 보정)
- 엔티티 add 시 `decimate=False, convexify=False, surface=gs.surfaces.Default(smooth=False)` — MuJoCo 동일 faceted 정밀 렌더 (smooth=True 면 mesh 정상 normal 깨짐)

### Crusher Motor 조인트
| 조인트 이름 | 역할 | DOF |
|---|---|---|
| `L3_Bevel_GearBox_1_L4_Shaft_1` | **Motor1 크랭크** (revolute) | 0 |
| `L1_Guide1_1_L2_Left_Wall1_1` | **Motor2 Left_Wall 슬라이드** (prismatic) | 2 |
| `L5_Link1_1_L6_Link2_1` | 패시브 (connecting rod) | 4 |
| `L6_Link2_1_L7_Link3_1` | 패시브 (slider arm) | 7 |
| `L2_Linear_bush_1_L8_Link3_Shaft_1` | 패시브 (L8 slider) | 3 |

### Crusher 정적 벽 좌표 (build 불필요, 해석적 계산)
Genesis 가 worldbody 정적 mesh geom 들을 link 당 1개로 "병합" → `link.get_pos()` 와
`entity.geoms` 로 개별 mesh 조회 불가. 해결: STL 정점에 MJCF 변환(quat=(.5,.5,.5,.5),
scale=1e-3) + 엔티티 변환(`CRUSHER_POS`, yaw)을 손으로 적용해 월드 AABB 계산.
함수: `crusher_mesh_world_aabb(mesh_name, body_pos, geom_pos)` 참조 (`Crusher_Samplebag.py`).

### M0609 + RG2 (`assets/robots/m0609_rg2.xml` = `paths.ROBOT_M0609_RG2`)
- 6 DOF arm + 2 DOF 프리즈매틱 핑거 (Panda 식 독립 prismatic, `Twin.md` §1 참조)
- 핑거 link y offset ±0.017 m → DOF=0 (최대 닫힘) 시 finger gap **34 mm**
  → **34 mm 미만 객체는 RG2 로 grip 불가** (구조적 한계)
- 핑거 mesh contype=0 기본 (visual-only); 충돌 ON 하면 PBD 압착 폭주

### PBD 봉투 (5면체 cloth, mouth 상단 개방)
```python
W, H, D = 0.05, 0.08, 0.006        # Crushing.py 기준. D 6mm 가 PBD 하한 (2*particle_size)
PARTICLE_SIZE = 2.83e-3
gs.materials.PBD.Cloth(
    stretch_compliance=1e-3,        # Samplebag 검증값. default 1e-7 은 매우 단단
    bending_compliance=1e-3,
)
```
- D < 2*particle_size 면 앞/뒤 패널 파티클이 즉시 충돌 제약 위반 → 폭주 (`Twin.md` §4 참조)
- `make_bag()` 함수: 5 panels(front, back, bottom, left, right) tri-mesh 생성 → STL → Genesis Mesh morph 로드. `euler=(90, 0, 0)` 으로 mouth 상단 향함

### PBD-Rigid 커플링
```python
scene = gs.Scene(
    pbd_options=gs.options.PBDOptions(max_density_solver_iterations=2, particle_size=PARTICLE_SIZE),
    coupler_options=gs.options.LegacyCouplerOptions(rigid_pbd=True),
    ...
)
```

---

## 4. 핵심 기술 발견 (트러블슈팅 누적 결과)

### 4.1 PD 컨트롤러 — 텔레포트 금지
**`set_dofs_position` (텔레포트) 대신 `control_dofs_position` (PD)** 을 써야 다이내믹스 + 제약(`<weld>`) 이 정상 작동. 텔레포트는 단일 step 안에서 임펄스를 만들어 제약 솔버가 폭주.

Franka grasp_bottle.py 예제 게인 패턴(이 게 정답):
```python
robot.set_dofs_kp([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100])
robot.set_dofs_kv([ 450,  450,  350,  350,  200,  200,  200,  10,  10])
robot.set_dofs_force_range([-87]*7 + [-100]*2, [87]*7 + [100]*2)
```
- 팔: kp 4500/kv 450 — 매우 강함
- **핑거: kp 100/kv 10 — compliance** (박스 끼이지 않고 적절히 추종)
- force_range 명시 (default 너무 작으면 PD torque 가 clip 되어 robot 안 움직임)

### 4.2 Grasp 사이클 — position → force 전환
```python
# reach/close 까진 position control
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_position([0, 0], fingers_dof)
# lift 부터는 핑거를 force control 로
franka.control_dofs_force([-20, -20], fingers_dof)
```
`control_dofs_position` 두 번 호출 시 두 번째가 첫 번째를 덮어쓰는 케이스가 있어 (M0609 에서 관측) **한 번에 8/9 DOF 통합 호출** 권장.

### 4.3 weld vs connect (Crusher 크랭크-슬라이더 폐쇄 루프)
MuJoCo 의 `<weld anchor="...">` 는 anchor 지정 시 **3-DOF position-only** 로 완화되지만, **Genesis 는 anchor 무시하고 무조건 6-DOF 강체 부착**. L7_Link3 ↔ L8_Link3_Shaft 가 6-DOF 로 묶이면 슬라이더 회전 자유도가 없어 슬라이더에 자세 mismatch torque 누적 → 발산.
- 우회 1: `<connect>` 로 교체 (3-DOF)
- 우회 2: **weld 유지 + PD velocity 컨트롤러** (`control_dofs_velocity`) → 텔레포트가 아니라 토크 기반이라 weld 가 정상 작동. **이 방식이 검증됨** (`Crusher_only.py` 참조).

### 4.4 IK
Genesis `inverse_kinematics` 가 M0609 에서 비정상 (target 무관하게 arm-upright pose 반환). Franka 에서는 정상.
- M0609: **brute-force grid scan** 으로 우회 (`Crushing.py`, `Crusher_Samplebag.py`)
  - 2-stage: coarse 21×22×25 → fine ±0.1 11×11×11
  - `j2+j3+j5 = 2.90` invariant 로 EE 수직 자세 보존
  - 에러 ~2-5 mm 까지 수렴
- Franka: Genesis IK + `plan_path()` 그대로 사용 OK

### 4.5 RG2 비대칭 + mesh
- 핑거 4절 링크가 panda 식 독립 prismatic 으로 모델링됨 (Twin.md §1)
- `<equality><joint joint1=rg2_left joint2=rg2_right polycoef="0 1 0 0 0">` 로 동기화 의도지만, Genesis 1.1.0 에서 제대로 안 먹힘 → 좌우 핑거가 비대칭으로 닫힘 (관측됨)
- OnRobot 핑거 OBJ mesh 가 복잡 → convex hull 변환 시 폭주 위험
- **격리 테스트 결과**: Franka + PBD 봉투 grasp 성공 (`RigidGrasp_headless.py`). M0609+RG2 + 봉투 grasp 는 RG2 가 원인.

### 4.6 m0609_rg2.xml 의 `<actuator>` + `damping` + `<equality>`
Genesis `set_dofs_kp`/`set_dofs_kv` 와 MJCF 의 `<position kp=...>` actuator 가 충돌. joint `damping=400` 도 PD 추종 방해. 해결: 패치로 모두 제거 후 set_dofs_kp 만 사용.

### 4.7 mesh normal smoothing
`gs.surfaces.Default(smooth=False)` 안 주면 STL normal smoothing 으로 sharp edge 가 둥글어져 mesh 깨진 듯 보임. MuJoCo 와 동일한 faceted 렌더 원하면 `smooth=False` 필수.

### 4.8 봉투 corner 4개 선정 버그
원본 코드의 corner 4개 선정:
```python
corners = [argmin(x+y), argmax(x-y), argmin(-x+y), argmax(x+y)]
```
`argmin(-x+y) == argmax(x-y)` 라서 사실상 3 unique. **올바른 4 corner**:
```python
corners = [argmin(x+y), argmax(x-y), argmax(y-x), argmax(x+y)]
```

### 4.9 봉투 sag 후 grip particle 선정
4 corners 가 fix 되면 중앙 mouth 가 처져서 z_top 근처에 corners 만 남음. 절대 z 비교 대신 **x_strip 안에서 z 상위 K개** 로 선정:
```python
def _select_mid_top_grip(K=30):
    cur = _pos_of(bag); cx, cz = cur[:, 0], cur[:, 2]
    x_match = np.abs(cx - x_center) < GRIP_X_WIDTH
    cand = np.where(x_match)[0]
    cand = cand[~np.isin(cand, corners)]
    K = min(K, len(cand))
    return cand[np.argpartition(cz[cand], -K)[-K:]]
```
(`RigidGrasp_headless.py` 의 Franka+봉투 케이스에서 적용해 성공)

### 4.10 Bag 운반 — fix_particles_to_link
- 봉투 grip 영역 particle 을 robot link(또는 carrier rigid)에 weld:
  ```python
  bag.fix_particles_to_link(link_idx=link_idx, particles_idx_local=grip_idx)
  bag.release_particle(particles_idx_local=corners)   # 코너 풀어줘야 매달림
  ```
- Crushing.py 에선 `rg2_left` 에 부착 시도. `Crusher_Samplebag.py` 에서는 **invisible kinematic rigid carrier** 에 부착해 `carrier.set_pos(p, zero_velocity=True)` 로 하강.

### 4.11 정적 (fixed=True) rigid 도 set_pos 가능
```python
carrier = scene.add_entity(morph=gs.morphs.Box(..., fixed=True), ...)
# build 후
carrier.set_pos(np.array([x,y,z]), zero_velocity=True)
```
이걸로 매니퓰레이터 없이도 봉투를 임의 궤적으로 운반 가능.

---

## 5. 파일별 상세 (실행 + 검증 노트)

### `Crusher_only.py` (+ headless)
- Crusher 단독, Motor1 PD position 워밍업 (0→-π/2) + PD velocity 스핀 (2π rad/s).
- 패시브 조인트 (L5/L6/L8) 응답 + L8 슬라이더 stroke ±2cm 검증.
- DT=5e-4, SUBSTEPS=10 (weld solref 0.0002 와 짝 맞춤)
- 마지막 검증 (이전 세션): 슬라이더 stroke ratio 0.99 (target 0.04 m, measured 0.0395 m) ✓
- Viewer 모드에서 `show_collision=True` 옵션으로 충돌 mesh 반투명 오버레이 가능.

### `Samplebag.py` (+ headless)
- PBD cloth + Rigid box 만 (로봇/Crusher 없음).
- stretch metric: `scipy.spatial.cKDTree` 로 initial neighbor-pair 추출, 각 step 거리 비율로 계산.
- 디폴트 `stretch_compliance=1e-3, bending_compliance=1e-3` 가 박스 안전 유지 + 유연성 ↑ 의 균형점.

### `Crushing.py` (+ headless)
- Full integrated: M0609+RG2 + 봉투 + 박스 + Crusher.
- 패치된 m0609_rg2.xml 로드 (`patch_robot_mjcf`: RG2 finger 충돌 ON), Crusher 패치본 로드.
- Waypoint Q 5개 (GRASP/LIFT/MOVE/APPROACH/DESCEND) 를 grid scan 으로 동적 계산.
- 시퀀스: dropin → close → csettle → grasp(attach) → lift → move → approach → settle → descend → release → hold
- **현재 이슈**: RG2 핑거 비대칭/mesh 형상으로 봉투 grasp 시각 미흡. 봉투가 핑거 옆에 매달리는 듯 보임.
- Wall_3(L2_Wall3_1) 앞면을 타깃으로 grid scan + 2-step IK (APPROACH→DESCEND) + j6=π/2 봉투 90° 회전.

### `Crusher_Samplebag.py` (+ headless) — 방금 작업
- **로봇 없는 환경.** Crusher + 봉투 + 박스 + plate 4개 + invisible carrier.
- 5-phase 시퀀스:
  1. **dropin (1s)**: 박스가 봉투 mouth 위 2cm 에서 낙하 → 봉투 안 안착
  2. **descend (1.5s)**: carrier 선형 하강 (start_z=0.286 → target_z=0.086), 봉투 corner 4개가 carrier 에 weld 돼 따라옴
  3. **warmup (0.5s)**: Motor1 크랭크 PD position 0 → -π/2 램프
  4. **clamp (1s)**: Motor2 +5 mm/s PD velocity (Left_Wall 닫힘)
  5. **crank (4s)**: Motor1 +π rad/s 등속 회전
- 검증된 결과 (`Crusher_Samplebag_20260624_155514.mp4`):
  - 봉투 carrier 따라 하강 ✓
  - 박스 봉투 안 유지 (descend 끝 box_z=0.044)
  - wall 4.1mm 슬라이드 ✓
  - 크랭크 10.9 rad (~1.7 rev) 안정 회전 ✓

### `RigidGrasp_headless.py` — 격리 테스트
- **변인통제용 일회성 테스트.** Franka Panda + PBD 봉투 grasp.
- 시퀀스: pre-grasp → reach → grasp(close + fix_particles_to_link) → lift(finger force -20N)
- 결과: 봉투 z=0.04 → 0.30, 25cm lift 성공 → **Genesis 제어 패턴 자체는 OK**, M0609+RG2 가 원인 확정.
- 4.7 / 4.8 / 4.9 / 4.10 의 모범 구현이 여기 있음.

---

## 6. 알려진 이슈 / TODO

| 우선순위 | 이슈 | 메모 |
|---|---|---|
| 높음 | `Crushing.py` 의 RG2 봉투 grasp 시각화 미흡 | RG2 mesh + 비대칭 prismatic 가 원인. 대안: (a) Franka 그리퍼로 통째 교체, (b) RG2 mesh 를 단순 box primitive 로 패치 + `<actuator>/<equality>/damping` 제거 |
| 높음 | `Crusher_Samplebag.py` 박스가 봉투에서 일부 빠져나옴 | descend 끝 box_z=0.044 (봉투 mouth z=0.086 보다 4cm 아래 = 봉투 안 OK 이지만 crank 시작 후 box 가 plate 위로 흘러나옴 가능). dropin 더 길게 + 봉투 깊이 ↑ 검토 |
| 중간 | Crusher carrier 의 grip 위치 — corner 가 아닌 mouth 전체 ring 으로? | corner 4점만이라 봉투가 사각형으로 펴진 채 하강. 자연스런 갈색 모양 원하면 carrier 부착점 늘리기 |
| 중간 | weld vs connect — Crusher 크랭크-슬라이더는 weld+PD velocity 로 검증됨 | 다른 closed-loop 메커니즘 추가 시 같은 패턴 적용 |
| 낮음 | 옛 medicine_envelope/pbd_bag_test 등 정리 | 사용자 미요청, 보존 중 |

---

## 7. 빠른 실행 가이드 (새 세션 첫 5분)

```powershell
# 1. 환경 확인
nvidia-smi    # Driver 595.79 이상이어야 함
$py = "$env:USERPROFILE\miniconda3\envs\crusher_genesis\python.exe"
& $py -c "import torch; print('torch', torch.__version__, 'cuda=', torch.cuda.is_available())"
& $py -c "import genesis as gs; gs.init(backend=gs.cuda); print('genesis OK')"

# 2. 동작 검증된 세 시뮬 (각 30초~3분)
Set-Location C:\Crusher_isaacsim\Crusher_Genesis
& $py Crusher_only_headless.py          # 크랭크 회전 + 슬라이더 stroke 검증
& $py Samplebag_headless.py             # PBD cloth + box 안정성
& $py Crusher_Samplebag_headless.py     # 통합 5-phase 시퀀스 (방금)

# 3. 결과
ls Sim_result\*.mp4 | Sort-Object LastWriteTime -Descending | Select-Object -First 5
```

---

## 8. 사용자 지침 (중요)

- 새 테스트 스크립트를 마구잡이로 만들지 않는다. 정식 6+1 파일 셋 안에서 수정·재실행.
- 로그/임시 파일 (`_run.log`, `.vscode/`, `MUJOCO_LOG.TXT`, MuJoCo_PlayGround 산출물, tablets_stl 등) 은 커밋하지 않음.
- 영상은 timestamp (`Crusher_Samplebag_YYYYMMDD_HHMMSS.mp4`) 로 저장해 이전 결과와 나란히 보존.
- `Twin.md` 의 평행 그리퍼 (4절 링크 → 독립 prismatic + equality) 모델링 패턴 우선 참조.
- 권한 allowlist: `.claude/settings.json` 에 git read / grep / find / WebFetch(github,raw.gh) 등록됨.

---

## 9. 인용 (Citation)

본 프로젝트는 **Genesis** 물리 엔진 위에서 개발되었다. 논문·보고서에서 인용 시
Genesis 공식 저장소가 제공하는 BibTeX 를 사용한다 (개별 저자 대신 단체 저자
`{Genesis Authors}` 표기가 공식):

```bibtex
@software{Genesis,
  author  = {Genesis Authors},
  title   = {Genesis: A Universal and Generative Physics Engine for Robotics and Beyond},
  month   = {December},
  year    = {2024},
  url     = {https://github.com/Genesis-Embodied-AI/Genesis}
}
```
