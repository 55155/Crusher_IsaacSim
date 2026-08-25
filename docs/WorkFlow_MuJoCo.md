# Crusher 시뮬레이션 워크플로우 (MuJoCo 정제 분쇄 트랙)

MuJoCo 기반 **정제 분쇄 반력 프로파일** 시뮬레이션의 스크립트 실행 흐름을
단계별로 정리합니다.

> **범위 주의**: 이 문서는 `MuJoCo_PlayGround/` 계통의 실행 가이드입니다.
> Genesis 기반 **공정 시뮬레이션 워크플로우**(파지 → 삽입 → 분쇄 → 이송 →
> 회수)는 [`docs/WorkFlow.md`](WorkFlow.md) 를 보세요. 이 트랙의 방법론적
> 배경은 [`docs/Real2Sim.md`](Real2Sim.md) 에 있습니다.

---

## 전체 흐름 다이어그램

```
[1] 정제 형상 생성
    viewer_desktop.py (R / AR / CV 슬라이더)
         │
         ▼  tablet_R{R}_AR{AR}_CV{CV}.stl
[2] 단일 시뮬레이션
    crusher_velocity_ctrl.py
         │                         ─────────────────────
         ▼                        │ 선택적 경로            │
    CSV + Plot PNG          [2-A] crusher_tablet_sim.py  │  (stall 역전 방식)
         │                  [2-B] crusher_tablet_slidejoint.py │
         │                        ─────────────────────
         ▼
[3] 배치 / 파라미터 스윕
    batch_cv_sweep.py  /  batch_tablet_sim.py
         │
         ▼
    overlay PNG + summary CSV
         │
         ▼
[4] 결과 분석
    force_profile_analysis.py  /  test_tdp_position.py
         │
         ▼
    dF/dt 분류, F_max vs θ_contact 그래프
         │
         ▼
[5] 디버그 (필요 시)
    debug_headless.py
         │
         ▼
    6채널 진단 PNG (NaN / RPM / 솔버 수렴 자동 감지)
```

---

## 단계별 상세

---

### STEP 1 — 정제(Tablet) 형상 생성

**목적**: 분쇄 대상 정제의 3D STL 파일을 파라미터로부터 생성합니다.

#### 파라미터 정의

| 파라미터 | 기호 | 의미 | 단위 |
|---------|------|------|------|
| 반지름 | R | 정제 반경 | mm |
| 종횡비 | AR | 높이 / (2R) | — |
| 왕관비 | CV | 크라운 높이 / R | — |

두께(thickness) 계산:

```python
cd = CV * 2 * R          # crown depth [mm]
th = R * 0.20 + 2 * cd   # 총 두께 [mm]
```

#### 실행

```bash
# PyVista 3-슬라이더 뷰어 (R / AR / CV 실시간 미리보기)
python MuJoCo_PlayGround/20260603/viewer_desktop.py

# 또는 20260527 버전
python tablets_stl/Codes/viewer_desktop.py
```

**[MuJoCo에서 열기]** 버튼 → 선택한 STL을 즉시 물리 시뮬레이션으로 연결.

#### 출력

```
tablets_stl/stl/
└── tablet_R{R}_AR{AR}_CV{CV}.stl   ← 파일명에 파라미터 인코딩
```

> STL 파일명 파싱 정규식: `R([\d.]+)_AR([\d.]+)_CV([\d.]+)`  
> `crusher_velocity_ctrl.py`의 `_parse_params()` 함수가 이 형식에 의존합니다.

---

### STEP 2 — 단일 시뮬레이션

세 가지 시뮬레이션 방식이 있으며, 모두 동일한 MJCF 모델을 사용합니다.

---

#### 2-A. 속도 제어 방식 (권장) — `crusher_velocity_ctrl.py`

**특징**: 실제 모터 스펙(BL4281 + 1:212) 기반, 준정적 조건, 뷰어 없음(headless)

```bash
python MuJoCo_PlayGround/20260603/crusher_velocity_ctrl.py <tablet.stl>
python MuJoCo_PlayGround/20260603/crusher_velocity_ctrl.py <tablet.stl> --rpm 8 --kv 14.9 --density 1200
```

**시뮬레이션 단계:**

| Phase | 조건 | 내용 |
|-------|------|------|
| **Phase 1** (500 step, 약 0.5 s) | `lock_crank` equality 활성 | 크랭크 −90° 고정, 메커니즘 수렴 대기 |
| **Phase 2 — 대기** (0 ~ 2 s) | 모터 OFF | 정제 mocap 위치 확인 |
| **Phase 2 — 압축** (2 s ~) | `lock_crank` 해제, 모터 ON (CCW) | 크랭크 회전 → 슬라이더 전진 → 접촉력 측정 |

**핵심 파라미터:**

```python
TARGET_RPM     = 8.0          # 크랭크 운전 속도
VEL_KV_DEFAULT = 14.9         # velocity actuator 게인 [N·m·s/rad]
MOTOR_FORCELIM = 12.5         # 토크 상한 [N·m]
PHASE1_STEPS   = 500
SIM_DURATION   = 20.0         # [s]
MOTOR_DELAY    = 2.0          # [s] 모터 투입 지연
```

**출력:**

```
MuJoCo_PlayGround/Sim_result/
├── csv/velctrl_{tablet}_{timestamp}.csv   ← 시계열 데이터 (시간, 각도, RPM, 접촉력)
└── plot/velctrl_{tablet}_{timestamp}__result.png
         velctrl_{tablet}_{timestamp}__realtime.png
```

---

#### 2-B. 뷰어 포함 실행 — `crusher_velocity_ctrl_viewer.py`

```bash
python MuJoCo_PlayGround/20260603/crusher_velocity_ctrl_viewer.py <tablet.stl>
```

- MuJoCo 인터랙티브 뷰어와 함께 실시간으로 시뮬레이션 관찰 가능

---

#### 2-C. Slide Joint 방식 — `crusher_tablet_slidejoint.py`

```bash
python MuJoCo_PlayGround/20260603/crusher_tablet_slidejoint.py <tablet.stl>
```

- 정제를 mocap(고정 벽)이 아닌 **slide joint**로 구속 → 정제가 반력으로 밀릴 수 있음
- 접촉 물리가 더 현실적이나, 수치 불안정성이 높음

---

#### 2-D. Stall 감지 역전 방식 — `crusher_tablet_sim.py` (20260527)

```bash
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl>
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl> --density 1500
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl> --mass 320
```

- 크랭크 각속도 |ω| < 0.05 rad/s 가 `STALL_TIME_S` (2 s) 연속 → CCW ↔ CW 자동 역전
- 뷰어 포함 인터랙티브 모드
- 밀도 → solref τ 자동 계산 (Hertzian Contact Theory)

**밀도 → 접촉 경도 변환:**

| 밀도 | solref τ | 경도 분류 |
|------|----------|----------|
| 900 kg/m³ | 0.0200 s | 연질 |
| 1,200 kg/m³ | 0.0080 s | 기본 |
| 1,800 kg/m³ | 0.0010 s | 경질 |

---

### STEP 3 — GUI 실행 (선택적)

```bash
python MuJoCo_PlayGround/20260527/tablet_sim_gui.py
python MuJoCo_PlayGround/20260527/tablet_sim_gui.py --stl tablets_stl/stl/tablet_R8.0_AR0.80_CV0.25.stl
```

**입력:**
- 무게(mg) 또는 밀도(kg/m³) 직접 입력
- R / AR / CV 슬라이더
- STL 파일 경로

**실시간 계산 결과:**
- 추정 부피, 밀도
- solref τ (접촉 시정수)
- 경도 게이지 (0~100%)

**실행 버튼:**
- `▶ 시뮬레이션 실행` → `crusher_tablet_sim.py` 단일 실행
- `📦 배치 시뮬레이션` → `batch_tablet_sim.py` 호출

---

### STEP 4 — 배치 / 파라미터 스윕

#### 4-A. CV 스윕 — `batch_cv_sweep.py`

같은 R / AR에 대해 CV(왕관비)를 변화시키며 힘 프로파일을 비교합니다.

```bash
# 기본 (R=4.0, AR=1.00, CV 전체)
python MuJoCo_PlayGround/20260603/batch_cv_sweep.py

# 특정 R/AR 그룹
python MuJoCo_PlayGround/20260603/batch_cv_sweep.py --R 4.0 --AR 1.17

# 밀도 / RPM 지정
python MuJoCo_PlayGround/20260603/batch_cv_sweep.py --density 1400 --rpm 8

# 기존 CSV만 재플롯 (시뮬 재실행 없음)
python MuJoCo_PlayGround/20260603/batch_cv_sweep.py --csv_dir MuJoCo_PlayGround/Sim_result/csv
```

**출력:**

```
MuJoCo_PlayGround/Sim_result/cv_sweep/
├── cv_sweep_{timestamp}.csv                  ← 요약 테이블
├── cv_sweep_{timestamp}_CV{xx}.png           ← CV별 개별 그래프
├── cv_sweep_{timestamp}_F_timeseries.png     ← 힘 시계열 오버레이
├── cv_sweep_{timestamp}_P_A_timeseries.png   ← 위치/각도 오버레이
├── cv_sweep_{timestamp}_contact_type.png     ← 접촉 유형 분류
└── cv_sweep_{timestamp}_summary_metrics.png  ← F_max / 충격량 요약
```

#### 4-B. 배치 정제 스윕 — `batch_tablet_sim.py`

`tablets_stl/stl/_tablet_index.csv`에 등록된 정제를 순차/병렬 실행합니다.

```bash
# 순차 실행
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py

# 병렬 실행 (8 workers)
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py --parallel --workers 8 --density 1500

# 빠른 테스트 (10개 + 플롯 저장)
python MuJoCo_PlayGround/20260527/batch_tablet_sim.py --limit 10 --save-plots --density 1200
```

---

### STEP 5 — 결과 분석

#### 5-A. 힘 프로파일 분석 — `force_profile_analysis.py`

dF/dt 기반으로 준정적 vs 충격 접촉을 자동 분류합니다.

```bash
# 이론 비교 플롯만 출력
python MuJoCo_PlayGround/20260603/force_profile_analysis.py

# 실제 CSV 로드
python MuJoCo_PlayGround/20260603/force_profile_analysis.py --csv result.csv

# mocap vs slidejoint 비교
python MuJoCo_PlayGround/20260603/force_profile_analysis.py --csv mocap.csv --csv2 slidejoint.csv
```

**분류 기준:**

| 유형 | dF/dt 특성 | 원인 |
|------|-----------|------|
| 준정적(quasi-static) | 완만한 상승 → 정체(plateau) | 속도 제어, 낮은 stiffness |
| 충격(impact) | 접촉 직후 급격한 spike | 높은 solref, 딱딱한 정제 |

#### 5-B. TDP 위치 검증 — `test_tdp_position.py`

접촉 각도(θ_contact)와 F_max의 관계를 검증합니다.

```bash
python MuJoCo_PlayGround/20260603/test_tdp_position.py
python MuJoCo_PlayGround/20260603/test_tdp_position.py --stl tablet_R4.0_AR1.00_CV0.20.stl
python MuJoCo_PlayGround/20260603/test_tdp_position.py --offsets 0 5 10 20 30
```

**가설**: y_offset 증가 → θ_contact가 TDP(상사점, 0°)에서 멀어짐 → F_max 감소

**역산 공식:**

```
F_slider = τ_crank / (r × sin θ_contact)

θ_contact 가 0° 에 가까울수록 sin θ → 0 → F_slider → ∞ (이론적 증폭)
```

**출력:**

```
MuJoCo_PlayGround/Sim_result/tdp_test/
├── tdp_{tablet}_{timestamp}_01_timeseries.png
├── tdp_{tablet}_{timestamp}_02_fmax_vs_offset.png
├── tdp_{tablet}_{timestamp}_03_tdp_verification.png
└── tdp_{tablet}_{timestamp}_04_bar.png
```

#### 5-C. 방식 비교 — `run_comparison_headless.py`

mocap 고정 방식 vs slide joint 방식 반력 프로파일을 한 번에 비교합니다.

```bash
python MuJoCo_PlayGround/20260603/run_comparison_headless.py tablet_R6.0_AR1.50_CV0.20.stl
```

---

### STEP 6 — 디버그 / 진단

**언제 사용**: 비정상적인 움직임, NaN 발생, RPM 불일치, 솔버 발산 의심 시.

```bash
# 15초 헤드리스 진단 (기본 정제)
python MuJoCo_PlayGround/20260603/debug_headless.py --dur 15

# 특정 정제 + 진단 시간
python MuJoCo_PlayGround/20260603/debug_headless.py tablet_R4.0_AR1.00_CV0.08.stl --dur 20
```

**6채널 진단 패널:**

| 패널 | 측정값 | 정상 범위 |
|------|--------|----------|
| ① 크랭크 각도 | qpos[0] [deg] | −180° ~ +180° 연속 회전 |
| ② 크랭크 RPM | qvel [RPM] | target ± 20% |
| ③ 모터 ctrl vs actuator_force | [N·m] | forcelim=12.5 N·m 내 |
| ④ 슬라이더 Y 위치 | data.xpos [mm] | 스트로크 40 mm ± 허용 |
| ⑤ 솔버 반복 횟수 | solver_niter | avg ≤ 5, max < 50 |
| ⑥ efc_force 최대값 | [N] | 정상 구동 시 수백 N 이하 |

**자동 감지 항목:**
- `NaN` 발생 여부
- RPM 목표 대비 ±20% 이탈
- forcelim 포화율 (saturation %)
- 솔버 최대 반복 도달
- efc_force 급등
- 슬라이더 스트로크 오차

**출력:**

```
MuJoCo_PlayGround/Sim_result/debug/
└── diag_{tablet}_{timestamp}.png
```

---

### STEP 7 — 역산 검증 (Back-calculation)

시뮬레이션 출력값으로부터 하드웨어 실측값을 역산하여 모델 타당성을 확인합니다.

```
실측 Ground Truth          시뮬레이션 출력            역산 공식
─────────────────          ──────────────────          ──────────────
슬라이더 반력 600–650 N ←→ actuator_force + θ     F = τ / (r × sin θ)
크랭크 속도 8 RPM       ←→ qvel[crank_dof]
스트로크 40 mm          ←→ sliderY 변위
모터 토크 효율 ~32%     ←→ τ_crank / (τ_motor × n)
```

**검증 통과 기준 (debug_headless.py 실행 결과):**

```
Phase1 크랭크:  −89.94°  ✓  (허용: ±5°)
NaN 발생:           0회  ✓
평균 RPM:          7.74  ✓  (목표 8.0 ± 20%)
스트로크:          40.0 mm ✓  (이론값 2r)
솔버 niter:        avg 1.0, max 2 ✓
실측 토크:    −2.52 ~ 12.50 N·m ✓  (forcelim 내)
```

---

## 스크립트 전체 목록

| 스크립트 | 위치 | 역할 | 방식 |
|---------|------|------|------|
| `viewer_desktop.py` | `20260603/` | 정제 형상 뷰어 + MuJoCo 연동 실행 버튼 | 인터랙티브 |
| `crusher_velocity_ctrl.py` | `20260603/` | 속도 제어 시뮬 (헤드리스) ★ **주력** | headless |
| `crusher_velocity_ctrl_viewer.py` | `20260603/` | 속도 제어 + MuJoCo 뷰어 | 인터랙티브 |
| `crusher_tablet_slidejoint.py` | `20260603/` | slide joint 방식 시뮬 | headless |
| `batch_cv_sweep.py` | `20260603/` | CV 파라미터 스윕 배치 | headless |
| `run_comparison_headless.py` | `20260603/` | mocap vs slidejoint 비교 | headless |
| `force_profile_analysis.py` | `20260603/` | 힘 프로파일 dF/dt 분류 | 분석 |
| `test_tdp_position.py` | `20260603/` | TDP 위치 vs F_max 검증 | headless |
| `debug_headless.py` | `20260603/` | 6채널 진단 (NaN/RPM/솔버) | headless |
| `crusher_tablet_sim.py` | `20260527/` | stall 역전 방식 시뮬 (뷰어 포함) | 인터랙티브 |
| `batch_tablet_sim.py` | `20260527/` | 정제 인덱스 배치 실행 | headless |
| `batch_cv_sweep.py` | `20260527/` | CV 스윕 (구버전) | headless |
| `tablet_sim_gui.py` | `20260527/` | tkinter GUI 프론트엔드 | GUI |
| `check_placement.py` | `20260527/` | 정제 배치 좌표 검증 | 디버그 |

---

## 출력 디렉토리 구조

```
MuJoCo_PlayGround/Sim_result/
├── csv/                      ← 시계열 원시 데이터 (.csv)
│   └── velctrl_{tablet}_{ts}.csv
│
├── plot/                     ← 단일 시뮬 결과 플롯 (.png)
│   ├── velctrl_{tablet}_{ts}__result.png
│   └── velctrl_{tablet}_{ts}__realtime.png
│
├── cv_sweep/                 ← CV 스윕 결과
│   ├── cv_sweep_{ts}.csv
│   └── cv_sweep_{ts}_CV{xx}.png  (개별) + 오버레이 + 요약
│
├── debug/                    ← 헤드리스 진단 PNG
│   └── diag_{tablet}_{ts}.png
│
└── tdp_test/                 ← TDP 위치 검증 그래프
    └── tdp_{tablet}_{ts}_0{n}_*.png
```

---

## 환경 설정

```bash
# Conda 환경
conda activate isaaclab   # (또는 isaac_sim)

# Python 실행 파일 (Windows)
C:\Anaconda3\envs\isaaclab\python.exe

# 주요 의존성 버전
# MuJoCo   3.5.0
# numpy    2.3.1
# matplotlib 3.10.8
```

---

## 관련 문서

| 문서 | 내용 |
|------|------|
| [`docs/Crusher.md`](Crusher.md) | 기구·형상·재질·관성 상세 사양 |
| [`docs/motor_spec.md`](motor_spec.md) | 모터·감속기·액추에이터 파라미터 |
| [`README.md`](../../README.md) | 프로젝트 개요·빠른 시작·변경 이력 |
