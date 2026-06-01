# Crusher_IsaacSim

**약품 분쇄기(Crusher)** 의 Isaac Sim → MuJoCo 마이그레이션 및 정제(Tablet) 형상 시뮬레이션 통합 저장소입니다.

---

## 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [디렉토리 구조](#디렉토리-구조)
3. [빠른 시작](#빠른-시작)
4. [밀도 기반 접촉 경도 모델](#밀도-기반-접촉-경도-모델)
5. [변경 이력](#변경-이력)
6. [의존성 설치](#의존성-설치)
7. [주의 사항](#주의-사항)

> **✅ 2026-05-29 업데이트 (최신)** — **밀도 기반 접촉 경도 모델** 추가 (`--density` / `--mass`): Hertzian Contact Theory를 기반으로 알약 밀도 → solref 시정수 τ 자동 계산. 경질 알약은 강성 높게, 연질 알약은 강성 낮게 시뮬레이션.  
> → [밀도 기반 접촉 경도 상세 설명](#밀도-기반-접촉-경도-모델) | [시뮬레이션 실행 바로가기](#5-crusher--tablet-통합-시뮬레이션-실행)

---

## 프로젝트 개요

| 구성 요소 | 설명 |
|-----------|------|
| **Crusher_IsaacSim_description** | ROS URDF + Isaac Sim USD 원본 기술 파일 |
| **MuJoCo_PlayGround** | Crusher 모델을 MuJoCo(MJCF)로 변환·시뮬레이션 |
| **tablets_stl** | 경구정(Tablet) 형상 파라미터 기반 STL 생성·시각화·충돌 분해 |

---

## 디렉토리 구조

```
Crusher_IsaacSim/
│
├── README.md                          ← 이 파일
├── requirements.txt                   ← Python 의존성 목록
├── .gitignore                         ← STL 바이너리 제외 설정
│
├── Crusher_IsaacSim_description/      ← ROS 패키지 원본
│   ├── urdf/                          ← URDF / xacro / USD 파일
│   ├── meshes/                        ← 부품별 STL 메시
│   └── Code/                          ← 보조 스크립트 (질량/관성 검사, 재질 등)
│
├── CODES/                             ← 초기 Isaac Sim 디버그 스크립트 모음
│
├── 20260505/ ~ 20260513/              ← 날짜별 Isaac Sim 작업 스크립트
│
├── MuJoCo_PlayGround/                 ← MuJoCo 시뮬레이션 작업 공간
│   ├── MJCF/
│   │   ├── Crusher_IsaacSim.xml       ← 기본 MJCF (색상 없음)
│   │   ├── Crusher_IsaacSim_colored.xml  ← 색상 + Ground + 폐루프 제약 완성본
│   │   ├── Ground.stl                 ← 바닥 메시
│   │   └── L*.stl / base_link.stl    ← 부품 메시 (42개)
│   │
│   ├── 20260514/  view_mjcf.py        ← 초기 MJCF 로더
│   ├── 20260515/  open_crusher.py     ← 기본 XML 뷰어 (경고 억제 포함)
│   ├── 20260518/                      ← 색상 파이프라인 스크립트
│   │   ├── apply_csv_to_mjcf.py       ← mesh_colors.csv → MJCF material 적용
│   │   ├── extract_materials.py       ← USD에서 재질 추출
│   │   ├── make_mesh_csv.py           ← 메시-색상 CSV 생성
│   │   └── fusion360/                 ← Fusion 360 appearances.json 기반 적용
│   │
│   ├── 20260520/
│   │   └── open_crusher_colored.py   ← ★ 색상+Ground XML Python 뷰어 (신규)
│   │
│   ├── 20260527/
│   │   └── crusher_tablet_sim.py     ← ★ Crusher + Tablet 통합 시뮬레이션 (2-Phase)
│   │
│   ├── mujoco.bat                     ← ★ 더블클릭으로 뷰어 실행 (상대경로 수정)
│   └── convert_urdf_to_mjcf.py        ← URDF → MJCF 변환기
│
└── tablets_stl/                       ← 정제 형상 도구 모음
    ├── stl/
    │   ├── _tablet_index.csv          ← 1000개 정제 파라미터 메타데이터
    │   └── tablet_R*.stl              ← 생성된 STL 파일 (git 제외)
    │
    └── Codes/
        ├── viewer_desktop.py          ← ★ PyVista 3-슬라이더 형상 뷰어 + MuJoCo 연동
        ├── launch_mujoco.py           ← ★ STL → MuJoCo 물리 시뮬레이션 런처
        ├── collision_viewer.py        ← ★ 볼록 분해 4-모드 충돌 뷰어
        ├── tablet_generator.py        ← Fusion 360 AddIn (로컬 실행 불가, 참조용)
        ├── viewer.html                ← 웹 기반 STL 브라우저
        ├── start_viewer.bat           ← viewer_desktop.py 실행 배치
        ├── run_collision_viewer.bat   ← collision_viewer.py 실행 배치
        └── install_and_run.bat        ← 의존성 설치 + 뷰어 실행 배치
```

---

## 빠른 시작

### 1. Crusher MuJoCo 뷰어 실행

```bash
# 방법 A — 더블클릭 (권장)
MuJoCo_PlayGround/mujoco.bat

# 방법 B — Python 스크립트
conda activate isaac_sim
python MuJoCo_PlayGround/20260520/open_crusher_colored.py
```

> `mujoco.bat`은 `.py` 파일을 경유하지 않고 `python -m mujoco.viewer --mjcf` 명령으로  
> 설치된 MuJoCo 패키지의 CLI 뷰어를 **직접** 실행합니다.

### 2. 정제 형상 뷰어 실행

```bash
conda activate isaac_sim
python tablets_stl/Codes/viewer_desktop.py
```

- 슬라이더로 `R` / `AR` / `CV` 파라미터 조정 → 형상 실시간 미리보기
- **[MuJoCo에서 열기]** 버튼 → 선택한 STL을 별도 창에서 물리 시뮬레이션

### 3. 충돌 메시 분해 뷰어 실행

```bash
conda activate isaac_sim
python tablets_stl/Codes/collision_viewer.py
```

- 4가지 보기 모드: 원본 메시 / 볼록 껍질 / 겹침 보기 / 면 법선
- 거친(8) / 표준(16) / 정밀(32) / 최정밀(64) hull 프리셋

### 5. Crusher + Tablet 통합 시뮬레이션 실행

```bash
conda activate isaac_sim

# 기본 실행 (밀도 1200 kg/m³ 기본값)
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl>

# 밀도 직접 지정 (경질 알약)
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl> --density 1500

# 실측 질량으로 밀도 자동 계산
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl> --mass 320

# 파일 선택 다이얼로그로 실행
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py
```

#### 배치 시뮬레이션 실행

```bash
# 순차 실행 (기본 밀도)
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py

# 병렬 실행 + 경질 알약 설정
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py --parallel --workers 8 --density 1500

# 빠른 테스트 (10개, plot 저장)
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py --limit 10 --save-plots --density 1200
```

#### 동작 방식

| 단계 | 내용 |
|------|------|
| **Phase 1** (뷰어 없음, 500 스텝) | `lock_crank` equality 활성 (−90° 고정) → Crusher 메커니즘 안정화 |
| **Phase 2 — 대기** (0 ~ 3 s) | 뷰어 오픈, 모터 OFF. 알약은 mocap body로 충돌판 벽면에 고정 |
| **Phase 2 — 압축** (3 s ~) | `lock_crank` 해제 → Motor CCW −0.5 N·m 구동 → 슬라이더 전진, 법선 반력 측정 |
| **Phase 2 — 역전** (stall 감지) | 크랭크 각속도 \|ω\| < 0.05 rad/s 가 STALL_TIME_S(2.0s) 연속 → 방향 전환 (CCW↔CW 반복) |

#### 핵심 구조

- **XML 파일 없음** — `Crusher_IsaacSim_colored.xml` 을 메모리에서 파싱, Tablet body 노드를 동적 삽입 후 `MjModel.from_xml_string()` 으로 직접 로드
- **알약 고정 방식 — mocap body** — `mocap="true"` 선언으로 MuJoCo가 알약을 불가침 강체 벽으로 인식. `data.mocap_pos / mocap_quat` 으로 위치·자세 제어. 관통 없음
- **접촉력 측정** — `data.contact` + `mj_contactForce()` → contact frame → world frame 변환. 알약에 작용하는 법선 반력(F_Y) 수집
- **크랭크 제어** — `lock_crank` equality (polycoef=−π/2) 런타임 해제 (`data.eq_active`), **moving-window stall 감지** 기반 양방향 자동 역전
- **알약 자세** — X축 90° + Y축 90° 합성 쿼터니언 `[0.5, 0.5, 0.5, −0.5]` → 장축이 수직(world-Z) 방향
- **실시간 플롯** — `plt.ion()` 기반 실시간 F_Y 창이 시뮬레이터와 동시 업데이트 (20 step 마다)
- **출력** — 위치·힘·임펄스·각속도 그래프 5종을 PNG로 저장

#### 알약 배치 좌표 (MuJoCo world frame)

| 축 | 값 | 의미 |
|----|----|------|
| X | −47.879 mm | 충돌판 중심 정렬 |
| Y | 336.199 mm | 충돌판(back wall) 면에 밀착 |
| Z | 50.108 mm | 수직 높이 |

---

---

## 밀도 기반 접촉 경도 모델

### 개요

같은 형상(Shape)의 알약이라도 **밀도(density)**가 높을수록 파쇄가 어렵습니다.  
밀도는 실측 무게와 형상(부피)으로 추정할 수 있으며, 이를 MuJoCo 접촉 파라미터(`solref`)에 반영합니다.

### 이론적 근거 (Hertzian Contact Theory)

```
제약공학 사실:
  압착 압력↑  →  밀도(ρ)↑  →  Young's modulus(E)↑  →  경도↑

Hertzian Contact:
  접촉 강성  K_contact  ∝  E*  (등가 탄성계수)
  경험적 관계: E  ∝  ρⁿ   (n ≈ 2~3)

MuJoCo solref 연결:
  K_mujoco  ∝  1/τ²   (τ = solref 시정수)
  K ∝ ρⁿ, K ∝ 1/τ²   →   τ  ∝  ρ^(-n/2)

결론: 밀도 높음 → τ 작음 → 접촉 강성 높음 → 관통 감소
```

### 매핑 함수

두 기준점을 지정하면 **power-law**로 보간합니다:

```
τ(ρ) = τ_soft × (ρ_soft / ρ)^α

        log(τ_hard / τ_soft)
α  = ─────────────────────────  ≈ −3.32
        log(ρ_hard / ρ_soft)
```

| 밀도 (kg/m³) | τ (s) | 상대 강성 (1/τ²) | 대표 알약 |
|---|---|---|---|
| 900 | 0.0200 | 1× | 연질 (저압착 포도당) |
| 1100 | 0.0096 | 4.3× | 표준 하한 |
| **1200** | **0.0080** | **6.3×** | ← 기본값 |
| 1400 | 0.0053 | 14× | 표준 경질 |
| 1600 | 0.0033 | 37× | 고압착 |
| 1800 | 0.0020 | 100× | 초경질 (탄산칼슘) |

### 부피 추정 및 밀도 계산

알약 실측 질량(mg)만 알면 형상 파라미터로 밀도를 자동 계산합니다:

```
V_ellipsoid = (4/3)π × (R×AR) × R × (th/2)
V_biconvex  = V_ellipsoid × 0.82   (biconvex 보정계수)

ρ = mass / V_biconvex
```

```bash
# 실측 질량 320mg → 밀도 자동 계산 → τ 자동 설정
python crusher_tablet_sim.py tablet_R6.0_AR1.50_CV0.20.stl --mass 320
```

### 구현 위치

| 함수 | 설명 |
|---|---|
| `estimate_tablet_volume_mm3(R_mm, AR, CV)` | 형상 파라미터 → 부피 [mm³] 추정 |
| `mass_to_density(mass_mg, R_mm, AR, CV)` | 질량 + 형상 → 밀도 [kg/m³] |
| `density_to_solref_tau(density_kg_m3)` | 밀도 → solref τ [s] (power-law) |
| `_build_model(..., density_kg_m3)` | geom에 `density`, `solref`, `solimp` 적용 |

### 조정 가능한 기준점 (코드 상단 상수)

```python
DENSITY_REF_SOFT  = 900.0    # kg/m³  연질 기준점
DENSITY_REF_HARD  = 1800.0   # kg/m³  경질 기준점
SOLREF_TAU_SOFT   = 0.020    # s      연질 τ (MuJoCo 기본값)
SOLREF_TAU_HARD   = 0.002    # s      경질 τ (실용적 최솟값)
DENSITY_DEFAULT   = 1200.0   # kg/m³  미지정 시 기본값
BICONVEX_VOL_FACTOR = 0.82   # biconvex 부피 보정계수
```

### CSV 출력 컬럼 (신규 추가)

결과 CSV 메타데이터에 밀도 관련 정보가 자동 기록됩니다:

```
# density_kg_m3, vol_estimate_mm3, biconvex_factor
# solref_tau_s, solimp_dmax, DENSITY_REF_SOFT, DENSITY_REF_HARD
```

---

### 4. 정제 STL 생성 (Fusion 360 전용)

`tablets_stl/Codes/tablet_generator.py` 참조.  
로컬에서 직접 실행 불가 — Fusion 360 내부 AddIn으로만 사용 가능합니다.

---

## 변경 이력

### 2026-05-29 (2) — 밀도 기반 접촉 경도 모델 (solref 자동 계산)

#### 핵심 변경

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| **tablet geom solref** | 미지정 (MuJoCo 기본값 τ=0.02s) | **밀도 → τ 자동 계산** (power-law 보간) |
| **tablet geom density** | 1200 kg/m³ 하드코딩 | `--density` 또는 `--mass` 인자로 동적 설정 |
| **_build_model 시그니처** | `(stl_path, R_mm, half_th)` | `(stl_path, R_mm, half_th, density_kg_m3)` |
| **CLI 인자** | 없음 | `--density KG_M3` / `--mass MG` (상호 배타) |
| **CSV 메타데이터** | 없음 | `density_kg_m3`, `solref_tau_s`, `solimp_dmax`, `vol_estimate_mm3` 추가 |
| **batch_tablet_sim.py** | 동일 | `--density` 인자 + task tuple에 밀도 포함 |

#### 신규 함수

| 함수 | 위치 | 설명 |
|------|------|------|
| `estimate_tablet_volume_mm3(R, AR, CV)` | 두 파일 공통 | 형상 파라미터 → biconvex 부피 [mm³] |
| `mass_to_density(mass_mg, R, AR, CV)` | 두 파일 공통 | 질량(mg) + 형상 → 밀도 [kg/m³] |
| `density_to_solref_tau(density_kg_m3)` | 두 파일 공통 | 밀도 → solref τ [s] (power-law) |

#### 이론

```
Hertzian Contact: E ∝ ρⁿ  →  K ∝ 1/τ²  →  τ ∝ ρ^(-n/2)
α = log(τ_hard/τ_soft) / log(ρ_hard/ρ_soft) ≈ −3.32
```

#### 수정 파일

| 파일 | 변경 내용 |
|------|-----------|
| `MuJoCo_PlayGround/20260527/crusher_tablet_sim.py` | v7 — 밀도 기반 solref, --density/--mass CLI |
| `MuJoCo_PlayGround/20260527/batch_tablet_sim.py` | v2 — 동일 로직, --density CLI, task tuple 확장 |
| `README.md` | 밀도 기반 접촉 경도 모델 섹션 신규 추가 |

---

### 2026-05-29 (1) — Moving-window 역전 알고리즘 + 실시간 반력 플롯

#### 핵심 변경

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| **크랭크 역전 방식** | 접촉력 임계(`\|F_Y\| > 5 N`) → CCW→CW 단방향, 복귀 없음 | **Moving-window stall 감지** — 양방향 (CCW↔CW 반복) |
| **역전 트리거 조건** | 접촉력 크기 기반 | 크랭크 각속도 `\|ω\| < 0.05 rad/s` 30 step 연속 유지 |
| **실시간 플롯** | 없음 (시뮬 종료 후 정적 PNG만) | `plt.ion()` 기반 F_Y 실시간 창 (20 step 간격 갱신) |
| **출력 그래프** | 3종 (위치, 힘, XYZ 성분) | **5종** — 위치, 힘+임펄스, XYZ 성분, **실시간 스냅샷**, **크랭크 각속도** |
| **콘솔 출력** | 위치·힘·접촉 수 | 위치·힘·각속도·현재 방향(`[CCW]`/`[CW]`) |

#### 상세 내용

**① Moving-window stall 감지 기반 양방향 역전**
```
STALL_WINDOW  = 30      # 연속 판정 스텝 수 (0.002 s × 30 = 0.06 s)
STALL_VEL_THR = 0.05    # 크랭크 속도 임계 [rad/s]
```
- `deque(maxlen=30)` — 최신 30 스텝의 `|ω| < threshold` 불리언 기록
- `all(stall_buf)` 가 True 되는 순간 `motor_dir = -motor_dir` 전환, deque 초기화
- 부호 규칙: `data.ctrl[act_crank] = motor_dir × MOTOR_CTRL`
  - `motor_dir = +1` (CCW) → ctrl = −0.5 N·m
  - `motor_dir = −1` (CW) → ctrl = +0.5 N·m
- CCW→CW, CW→CCW 모두 자동 처리. 역전 횟수·시각 콘솔 출력 및 플롯 마커 기록

**② 실시간 법선 반력 플롯 (`plt.ion()`)**
- 시뮬레이터 뷰어와 동시에 별도 창으로 F_Y [N] 실시간 표시
- 20 step(0.04 s)마다 `line_fy.set_data()` + `ax_rt.relim()` + `flush_events()`
- 방향 전환 이벤트 수직선(axvline)을 실시간 갱신
- 시뮬 종료 후 `crusher_tablet_realtime_force.png` 로 저장

**③ 크랭크 각속도 플롯 추가 (그림 4)**
- `data.qvel[crank_vadr]` 로그를 시간 축으로 표시
- stall 임계선 `±STALL_VEL_THR` 수평선 표시
- 방향 전환 이벤트 수직선 동기화

#### 수정 파일

| 파일 | 수정 내용 |
|------|-----------|
| `MuJoCo_PlayGround/20260527/crusher_tablet_sim.py` | v5 — moving-window 역전, 실시간 플롯, 각속도 플롯, 다중 역전 이벤트 추적 |

---

### 2026-05-28 — 알약 물리 고정 방식 전환 및 반력 측정 파이프라인 완성

#### 핵심 변경

| 항목 | 변경 전 | 변경 후 |
|------|---------|---------|
| **알약 고정 방식** | freejoint + 매 스텝 qpos/qvel 덮어쓰기 (kinematic hold) | `mocap="true"` body — 관통 없는 불가침 강체 벽 |
| **접촉력 측정** | `cfrc_ext` (부정확, 유령 body 문제) | `data.contact` + `mj_contactForce()` → world frame 변환 |
| **크랭크 초기 각도** | +90° | **−90°** |
| **모터 방향** | CW (+0.5 N·m) | **CCW (−0.5 N·m)**, 3초 지연 후 구동 |
| **크랭크 역전 트리거** | gap 기반 (슬라이더 위치) → 벽 접촉 시 고착으로 동작 안 됨 | **접촉력 기반** `|F_Y| > 5 N` → 신뢰성 있는 역전 |
| **알약 자세** | X축 90° (눕힘) | X축 90° + Y축 90° 합성 → **세로(수직) 방향** |

#### 상세 내용

**① mocap body 전환 (관통 문제 해결)**
- `freejoint` 방식에서는 MuJoCo contact solver가 분리 impulse를 계산해도 매 스텝 qpos를 덮어써서 슬라이더가 알약을 관통하는 문제 발생
- `mocap="true"` body는 물리 엔진이 직접 불가침 강체로 취급 → 슬라이더가 알약을 밀어내고 알약은 고정 유지
- 위치·자세 제어: `data.mocap_pos[mocap_id]`, `data.mocap_quat[mocap_id]`

**② lock_crank equality 런타임 토글**
- `Crusher_IsaacSim_colored.xml` 의 `lock_crank` (polycoef=−1.5708, solref=−100000) 로 Phase 1에서 크랭크를 −90°에 고정
- Phase 2 시작 3초 후 `data.eq_active[eq_lock_id] = 0` 으로 해제 → 모터 CCW 구동

**③ 접촉력 기반 크랭크 역전**
- 기존 gap 기반: 슬라이더가 mocap 벽에 막히면 gap이 최솟값에 고착 → `elif gap > min + 0.5mm` 조건 영원히 불충족
- **수정**: `|F_Y| > REVERSE_F_THRESHOLD (5 N)` 초과 시점에 `ctrl = +0.5 N·m` (CW) 역전
- 역전 시각, 접촉력, gap 값을 콘솔 출력 및 플롯 마커로 기록

**④ 법선 반력 플롯 개선**
- F_Y 그래프 레이블을 "Normal Contact Force (World-Y)" 로 명확화
- 모터 ON 시각, 크랭크 역전 시각을 수직선(axvline)으로 표시
- 압축(양)/인장(음) 구간 fill_between 색 구분

**⑤ Tablet Shape Viewer 단축키 패널 추가**
- `viewer_desktop.py` 좌측 패널 하단에 PyVista 단축키 섹션 추가
- 마우스 조작 / 렌더링(`W` 와이어프레임 ★) / 카메라 / 기타 4개 섹션

#### 수정 파일

| 파일 | 수정 내용 |
|------|-----------|
| `MuJoCo_PlayGround/20260527/crusher_tablet_sim.py` | mocap body, lock_crank 토글, CCW 모터, 역전 로직, 법선 반력 플롯 |
| `MuJoCo_PlayGround/MJCF/Crusher_IsaacSim_colored.xml` | lock_crank polycoef −1.5708 복원 |
| `tablets_stl/Codes/viewer_desktop.py` | PyVista 단축키 패널 추가 |

---

### 2026-05-27 — Crusher + Tablet 통합 시뮬레이션 완성

#### 신규 파일

| 파일 | 내용 |
|------|------|
| `MuJoCo_PlayGround/20260527/crusher_tablet_sim.py` | Crusher XML + Tablet STL을 메모리에서 조합해 2-Phase 통합 시뮬레이션 실행. Phase 1(메커니즘 안정화) → Phase 2(알약 고정 + 접촉력 측정 + 그래프 저장) |

#### 주요 변경 사항

| 항목 | 내용 |
|------|------|
| **알약 고정 방식** | freejoint(6자유도) + 매 스텝 kinematic hold (qpos/qvel 덮어쓰기) → 알약을 공간에 고정하면서 cfrc_ext 접촉 반력 측정 가능 |
| **알약 배치** | Y 좌표를 `WALL_Y_MM`(336.199 mm)으로 설정 — 충돌판 벽면에 중심 밀착 |
| **MJCF 구성** | 별도 XML 파일 없음 — 기존 Crusher XML을 메모리에서 파싱 후 Tablet body / sensor 노드 동적 삽입 |
| **뷰어 site 구 제거** | `sitegroup` 전체 `False` — 중심부 구 형태 gizmo 비표시 |

---

### 2026-05-26 — Crank-Slider 메커니즘 진단 및 좌표계 분석

#### 핵심 발견

| 항목 | 내용 |
|------|------|
| **Equality Constraint 타입 오류** | `connect`는 위치(3-DOF)만 구속 → L7-L8 사이 회전 자유도 미구속으로 진동 발생. `weld`(위치+방향 완전 고정)로 교체 필요 |
| **Constraint Gap 원인 분석** | anchor=0.027 설정 시 L8 body와 10 mm 갭 발생 → 제약력 500 N (중력 12.7 N의 **39배**) → CCW 비정상 회전 유발 |
| **크랭크 회전축 오해** | XML `axis="0 0 1"` (body local Z) + `quat="0.5 0.5 0.5 0.5"` → 실제 world 회전축 = **[1, 0, 0] (world X)**. 중력(-Z)과 수직이므로 최대 토크 발생 |
| **MuJoCo 좌표계 변환** | `quat="0.5 0.5 0.5 0.5"` → R=`[[0,0,1],[1,0,0],[0,1,0]]` : URDF X→MuJoCo Y, URDF Y→MuJoCo Z, URDF Z→MuJoCo X |

#### 신규 파일

| 파일 | 내용 |
|------|------|
| `MuJoCo_PlayGround/20260526/coordinate_gizmo.py` | MuJoCo 월드 좌표계(XYZ 기즈모) 확인 스크립트. 뷰어 원점에 body frame 표시 |
| `MuJoCo_PlayGround/20260526/open_urdf.py` | 원본 URDF를 MuJoCo로 직접 로드해 각 body 월드 위치·joint 목록 출력 및 뷰어 실행 |

#### MJCF 수정 이력 (Crusher_IsaacSim_colored.xml)

| 수정 | 내용 |
|------|------|
| L5-L6, L6-L7 hinge damping 추가 | `damping="0.5"` — 초기 불안정 진동 억제 |
| L8 slide joint damping 추가 | `damping="5"` |
| L8_Link3_Shaft_1 body pos 수정 | Rigid32 접합 위치 = `(-0.048302, 0.236278, 0.049431)` (URDF 실측 검증 완료) |
| Equality Constraint anchor | URDF Rigid32 joint xyz=(0.037, 0, -0.005) 기준. **다음 단계: connect → weld 교체 예정** |
| 뷰어 설정 | `mjVIS_CONVEXHULL=False`, `geomgroup[3]=False` — visual mesh 전용 표시 |

---

### 2026-05-21 — macOS 호환성 수정 및 tablet_generator Fusion 360 등록

| 파일 | 변경 내용 |
|------|-----------|
| `tablets_stl/Codes/viewer_desktop.py` | `subprocess.CREATE_NEW_CONSOLE` (Windows 전용) → `platform.system()` 감지 후 조건부 적용으로 macOS 호환 수정 |
| `tablets_stl/stl/_tablet_index.csv` | Fusion 360 `tablet_generator` AddIn 실행으로 정제 STL 1,000개 생성 및 인덱스 갱신 |

---

### 2026-05-20 — MuJoCo 통합 완성 및 구조 정리

#### MuJoCo_PlayGround

| 파일 | 변경 내용 |
|------|-----------|
| `mujoco.bat` | 하드코딩된 절대경로 → `%~dp0` 기반 상대경로로 전면 수정; conda 활성화 경로도 `%USERPROFILE%` 변수로 포터블화 |
| `20260520/open_crusher_colored.py` | **신규** — `Crusher_IsaacSim_colored.xml`(Ground.stl 포함)을 Python API(`mujoco.viewer.launch`)로 여는 스크립트; `__file__` 기준 상대경로 사용 |

#### tablets_stl

| 파일 | 변경 내용 |
|------|-----------|
| `Codes/viewer_desktop.py` | **[MuJoCo에서 열기]** 버튼 추가; STL 로드 성공 시 활성화, `subprocess.Popen`으로 비차단 실행 |
| `Codes/launch_mujoco.py` | **신규** — 선택된 STL을 메모리 내 MJCF로 변환(`from_xml_string + assets`), 물리 파라미터(밀도 1200 kg/m³, 마찰 0.5) 설정 후 뷰어 실행 |
| `Codes/launch_mujoco.py` | **버그 수정** — `_print_params()` 정규식이 파일명의 `.stl` 확장자까지 캡처하여 `float('0.20.')` 오류 발생 → `os.path.splitext()` 로 확장자 제거 후 정규식 적용 |
| `Codes/collision_viewer.py` | **신규** — 볼록 분해(CoACD → VHACD → single hull fallback) 독립 뷰어; 4-모드 시각화 |

#### 저장소 공통

| 파일 | 변경 내용 |
|------|-----------|
| `requirements.txt` | **신규** — 그룹별 의존성 정리 (수치연산 / 3D시각화 / 메시처리 / MuJoCo / JAX / 이미지처리); 비pip 패키지(Isaac Sim, Fusion 360, Blender) 설치 안내 주석 포함 |
| `.gitignore` | **신규** — `tablets_stl/stl/*.stl`, `tablets_stl/collision/*.stl` 바이너리 제외; Python 캐시 제외 |

---

### 2026-05-18 — Crusher 색상 파이프라인 완성

| 변경 내용 |
|-----------|
| Fusion 360 appearance JSON 추출 스크립트 (`extract_appearance.py`) |
| `mesh_colors.csv` 기반 MJCF material 자동 적용 (`apply_csv_to_mjcf.py`) — RGB 0-255 → 0-1 변환, 조명 3개 자동 추가 |
| `Crusher_IsaacSim_colored.xml` 완성 — 색상·조명·Ground.stl·Crank-slider 폐루프 equality constraint·Motor1_crank 액추에이터·L1_Slider_jig 프리즈매틱 조인트 포함 |

---

### 2026-05-15 — MuJoCo 초기 마이그레이션

| 변경 내용 |
|-----------|
| URDF → MJCF 변환 (`convert_urdf_to_mjcf.py`) |
| 기본 뷰어 스크립트 `20260515/open_crusher.py` (MuJoCo UserWarning 억제) |

---

### 2026-05-05 ~ 2026-05-13 — Isaac Sim 디버그 및 관절 설정

| 작업 |
|------|
| 구동부 관절(Drive, Prismatic, Mimic) 설정 및 진단 스크립트 |
| Rack-and-Pinion 기어 연결 디버그 |
| CoACD 충돌 메시 최적화 |
| 질량/관성 텐서 검사 및 수정 |

---

## 의존성 설치

```bash
conda activate isaac_sim
pip install -r requirements.txt
```

> ⚠️ 아래 패키지는 pip으로 설치되지 않습니다.
> - **Isaac Sim** (`omni.*`, `carb`, `pxr`) — NVIDIA Isaac Sim 설치 시 번들 제공
> - **Fusion 360 API** (`adsk`) — Autodesk Fusion 360 내부 실행 전용
> - **Blender Python** (`bpy`) — Blender 내부 실행 전용

---

## 주의 사항

- **STL 바이너리** (`tablets_stl/stl/*.stl`) 는 `.gitignore`에 의해 Git에서 제외됩니다.  
  STL 파일은 Fusion 360 AddIn(`tablet_generator.py`)으로 로컬에서 생성하세요.
- `tablet_generator.py`는 `adsk` 모듈 의존성으로 인해 **Fusion 360 외부에서 실행 불가**합니다.  
  파일 내 주석의 설치 경로 안내를 따르세요.
- MuJoCo STL 단위는 **mm → m 자동 변환** (scale `0.001`)이 적용됩니다.  
  Fusion 360 기본 STL 내보내기 단위가 mm이기 때문입니다.
