# Crusher — 기구·형상·재질 사양서

Crusher 분쇄기의 **기하학적(Geometry)**, **메커니즘(Mechanism)**, **재질(Material)**, **관성(Inertia)** 정보를 MJCF 원본(`Crusher_IsaacSim_colored.xml`) 및 시뮬레이션 코드에서 추출하여 정리합니다.

---

## 목차

1. [전체 구조 개요](#1-전체-구조-개요)
2. [크랭크-슬라이더 메커니즘](#2-크랭크-슬라이더-메커니즘)
3. [링크 Body — 위치·관성 상세](#3-링크-body--위치관성-상세)
4. [슬라이더(Crusher 판)](#4-슬라이더crusher-판)
5. [좌측 벽 (Left Wall) 슬라이드](#5-좌측-벽-left-wall-슬라이드)
6. [구속 조건 (Equality Constraints)](#6-구속-조건-equality-constraints)
7. [재질 및 색상](#7-재질-및-색상)
8. [자유도(DoF) 목록](#8-자유도dof-목록)
9. [정제(Tablet) 배치 좌표](#9-정제tablet-배치-좌표)
10. [MuJoCo 솔버 설정](#10-mujoco-솔버-설정)

---

## 1. 전체 구조 개요

Crusher는 **크랭크-슬라이더(Crank-Slider)** 메커니즘으로 모터 회전운동을 직선 압축력으로 변환하는 정제 분쇄 장치입니다.

```
[Motor1 BLDC]
     │  (GEAR 1:212)
     ▼
[크랭크 L4_Shaft_1]  — hinge(z) — (crank r = 20 mm)
     │
[커넥팅 로드 L6_Link2_1]  — hinge(z) — (link 94 mm)
     │
[커넥팅 로드 L7_Link3_1]  — hinge(z)
     │  (weld equality: crank_slider_loop)
[슬라이더 L8_Link3_Shaft_1]  — slide(Y) — (충돌판 L9_PLATE)
     │
     ▼  ← 정제(Tablet) 충돌
```

- **구동 방향**: 크랭크 CCW(반시계) → 슬라이더 전진(−Y 방향)
- **스트로크**: 2 × r = 2 × 20 mm = **40 mm**
- **운전 속도**: 8 RPM (크랭크 기준)

---

## 2. 크랭크-슬라이더 메커니즘

### 2-1. 핵심 치수

| 파라미터 | 기호 | 값 | 출처 |
|---------|------|----|------|
| 크랭크 반경 | r | **20 mm** | L6_Link2_1 local pos x = 0.02 m |
| 커넥팅 로드 길이 (MJCF) | L_mjcf | **94 mm** | L7_Link3_1 local pos x = 0.094 m |
| 커넥팅 로드 길이 (유효) | L_eff | **80 mm** | 실제 핀 간 거리 (사용자 확인) |
| 슬라이더 스트로크 | S | **40 mm** | 2 × r |

> `L_mjcf = 94 mm`는 body origin 간 거리이며, 실제 크랭크 핀 ~ 슬라이더 핀 유효 길이는 **80 mm**입니다.

### 2-2. 기구학 (Kinematics)

크랭크 각도 θ 기준 슬라이더 위치 (근사, r/L << 1 가정):

```
x_slider(θ) ≈ r·cos θ + L·√(1 - (r/L)²·sin²θ)
```

슬라이더 힘 (준정적, 마찰 무시):

```
F_slider = τ_crank / (r × sin θ)
```

| θ | sin θ | F_slider (τ = 12.5 N·m, r = 0.02 m) | 비고 |
|---|-------|--------------------------------------|------|
| 90° | 1.000 | **625 N** | 실측 600–650 N ✓ |
| 60° | 0.866 | **722 N** | — |
| 45° | 0.707 | **884 N** | — |
| 30° | 0.500 | **1,250 N** | TDP(상사점) 근처, 이론 최대 |

### 2-3. 초기 자세

| 설정 | 값 | 설명 |
|------|----|------|
| 크랭크 초기각 | **−90° (−π/2 rad)** | `lock_crank` equality 고정 |
| 초기 안정화 | 500 step (Phase 1) | 구속 활성 상태에서 MuJoCo 수렴 |
| 모터 투입 | t = 2.0 s 이후 | `MOTOR_DELAY = 2.0 s` |

---

## 3. 링크 Body — 위치·관성 상세

모든 좌표는 **MuJoCo world frame** 기준 (단위: m, kg, kg·m²).

### 3-1. 크랭크 — `L4_Shaft_1`

| 항목 | 값 |
|------|----|
| World pos | `(−0.062002, 0.085278, 0.049431)` |
| Quat | `0.5 0.5 0.5 0.5` |
| 구동 관절 | `L3_Bevel_GearBox_1_L4_Shaft_1` — hinge, axis `(0, 0, 1)` |
| 질량 | 0.0662 kg |
| 관성 COM pos | `(0.0148, 2.15e-7, −0.0135)` |
| diaginertia (원본) | `4.166e-5, 3.160e-5, 1.407e-5` kg·m² |

> **시뮬레이션 주의**: `implicitfast` 적분기 사용 시 환산 관성(J_REFL = 0.324 kg·m²) body 주입 불필요. body 주입하면 Newton 솔버 23,000:1 관성비 → 발산.

### 3-2. 제1 커넥팅 로드 — `L6_Link2_1`

| 항목 | 값 |
|------|----|
| Local pos (L4_Shaft_1 기준) | `(0.02, 0, 0.0187)` → **크랭크 핀 오프셋 = 20 mm** |
| 관절 | `L5_Link1_1_L6_Link2_1` — hinge, axis `(0, 0, −1)`, damping=0.5 |
| 질량 | 0.163 kg |
| COM pos | `(0.047, −3.77e-6, 0.0049)` |
| diaginertia | `1.61e-4, 1.57e-4, 7.0e-6` kg·m² |

### 3-3. 제2 커넥팅 로드 — `L7_Link3_1`

| 항목 | 값 |
|------|----|
| Local pos (L6_Link2_1 기준) | `(0.094, 0, 0)` → **링크 길이 = 94 mm** |
| 관절 | `L6_Link2_1_L7_Link3_1` — hinge, axis `(0, 0, −1)`, damping=0.5 |
| 질량 | 0.287 kg |
| COM pos | `(0.0958, 0.00976, −0.00113)` |
| diaginertia | `9.245e-4, 8.888e-4, 1.271e-4` kg·m² |

---

## 4. 슬라이더(Crusher 판)

### 4-1. Slider Body — `L8_Link3_Shaft_1`

| 항목 | 값 |
|------|----|
| World pos | `(−0.048302, 0.236278, 0.049431)` |
| Quat | `0.5 0.5 0.5 0.5` |
| 관절 | `L2_Linear_bush_1_L8_Link3_Shaft_1` — slide, axis `(1, 0, 0)` (→ world Y), damping=5 |
| 질량 | **0.35 kg** |
| diaginertia | `5e-4, 5e-4, 2e-4` kg·m² |

> axis `(1, 0, 0)`은 body quat `0.5 0.5 0.5 0.5` 변환 후 **world Y 방향** 직선 운동.

### 4-2. Crusher 충돌판 — `L9_PLATE_v3_1`

- `L8_Link3_Shaft_1`의 자식 geom (type=mesh)
- 접촉 활성 (`contype`/`conaffinity` 기본값 → 충돌 검출)
- 정제(Tablet)와의 직접 접촉면
- 색상: 검정 `rgba="0.0 0.0 0.0 1.0"`

---

## 5. 좌측 벽 (Left Wall) 슬라이드

### `L2_Left_Wall1_1`

| 항목 | 값 |
|------|----|
| World pos | `(−0.017802, 0.286278, 0.016542)` |
| Quat | `0.5 0.5 0.5 0.5` |
| 관절 | `L1_Guide1_1_L2_Left_Wall1_1` — slide, axis `(−1, 0, 0)` |
| 구동 액추에이터 | `Motor2_left_wall` — motor, ctrlrange `±100 N` |
| 질량 | 0.312 kg |
| diaginertia | `3.681e-4, 2.653e-4, 1.931e-4` kg·m² |
| 래크기어 | `L3_RackGear_1` (자식 geom, 흰색) |

> 랙-피니언 구동으로 좌측 벽을 이동해 분쇄 간격을 조정합니다 (현재 시뮬에서는 미사용).

---

## 6. 구속 조건 (Equality Constraints)

### 6-1. `crank_slider_loop` — Weld

폐쇄 루프(closed-loop) 크랭크-슬라이더 연결. Rigid 32 조인트를 끊고 등가 weld로 재결합합니다.

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| body1 | `L7_Link3_1` | 커넥팅 로드 끝단 |
| body2 | `L8_Link3_Shaft_1` | 슬라이더 body |
| anchor | `(0.027, 0, −0.005)` m | L7 로컬 프레임에서 슬라이더 핀 위치 |
| solref | `−50000, −500` | stiffness=50,000 N/m, damping=500 N·s/m |
| solimp | `0.99, 0.9999, 0.0001` | constraint impedance 최대화 |

### 6-2. `lock_crank` — Joint Equality

Phase 1 (초기 안정화) 동안 크랭크를 −90°에 고정하는 equality 조건.

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| joint1 | `L3_Bevel_GearBox_1_L4_Shaft_1` | 크랭크 관절 |
| polycoef | `−1.5708, 0, 0, 0, 0` | q = −π/2 rad = −90° |
| solref | `−100000, −1000` | stiffness=100,000 N/rad (강체 구속) |
| solimp | `0.99, 0.9999, 0.0001` | — |

**해제 방법 (Python)**:

```python
eid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "lock_crank")
data.eq_active[eid] = 0   # Phase 2 시작 시
```

---

## 7. 재질 및 색상

MJCF `<material>` 기준 각 부품의 색상 및 표면 특성.

| Body / 부품 | 색상 (RGBA) | 재질 느낌 | specular | shininess |
|-------------|------------|----------|----------|-----------|
| `base_link` | 검정 `(0,0,0)` | 무광 플라스틱 | 0.30 | 0.20 |
| `L1_Slider_jig_1` | 검정 `(0,0,0)` | 무광 플라스틱 | 0.25 | 0.15 |
| `L1_Wall1_1`, `L1_Wall2_1` | 검정 `(0,0,0)` | 무광 플라스틱 | 0.25 | 0.15 |
| `L2_Linear_bush_1` | 실버 `(0.74,0.74,0.74)` | 광택 금속 | 0.90 | 0.75 |
| `L2_Wall3_1` | 검정 `(0,0,0)` | — | 0.25 | 0.15 |
| `L2_Left_Wall1_1` | 검정 `(0,0,0)` | 무광 플라스틱 | 0.30 | 0.20 |
| `L3_Bevel_GearBox_1` | 다크그레이 `(0.36,0.36,0.36)` | 알루미늄 | 0.28 | 0.18 |
| `L3_RackGear_1` | 흰색 `(1,1,1)` | 플라스틱 기어 | 0.30 | 0.40 |
| `L4_Bevel_GearBox2_1` | 골드 `(0.60,0.44,0)` | 황동/도금 | 0.05 | 0.05 |
| `L4_Motor1_FrontBearing_1` | 실버 `(0.74,0.74,0.74)` | 베어링 금속 | 0.65 | 0.40 |
| `L4_Motor1_RearBearing_1` | 실버 `(0.74,0.74,0.74)` | 베어링 금속 | 0.65 | 0.40 |
| `L4_Reducer1_1` | 파랑 `(0,0.2,0.6)` | 감속기 케이스 | 0.30 | 0.40 |
| `L4_Shaft_1` (크랭크) | 실버 `(0.74,0.74,0.74)` | 스테인리스 | 0.45 | 0.35 |
| `L5_Key_1` | 실버 `(0.74,0.74,0.74)` | 키 금속 | 0.45 | 0.35 |
| `L5_Link1_1` | 실버 `(0.74,0.74,0.74)` | 광택 금속 | 0.80 | 0.60 |
| `L5_Reducer2_1` | 실버 `(0.74,0.74,0.74)` | 감속기 부품 | 0.78 | 0.58 |
| `L6_DcutShaft_1` | 다크그레이 `(0.36,0.36,0.36)` | — | 0.28 | 0.18 |
| `L6_Link2_1` (커넥팅 로드 1) | 검정 `(0,0,0)` | 무광 | 0.30 | 0.20 |
| `L6_Motor1_Braket_1` | 검정 `(0,0,0)` | 광택 플라스틱 | 0.70 | 0.45 |
| `L7_Link3_1` (커넥팅 로드 2) | 검정 `(0,0,0)` | 무광 | 0.30 | 0.20 |
| `L7_Motor1_Body_1` | 다크그레이 `(0.36,0.36,0.36)` | 모터 바디 | 0.25 | 0.35 |
| `L9_PLATE_v3_1` (충돌판) | 검정 `(0,0,0)` | 무광 | 0.28 | 0.18 |
| `ground` | 회색 `(0.45,0.45,0.45)` | 콘크리트 | 0.05 | 0.05 |
| `tablet` (정제) | 베이지 `(0.85,0.80,0.72)` | — | — | — |

---

## 8. 자유도(DoF) 목록

MuJoCo `qpos` 배열 순서 (총 8 DoF):

| 인덱스 | 관절명 | 종류 | 역할 |
|--------|--------|------|------|
| [0] | `L3_Bevel_GearBox_1_L4_Shaft_1` | hinge (z) | **크랭크 — Motor1 구동** |
| [1] | `L5_Link1_1_L6_Link2_1` | hinge (z) | 제1 커넥팅 로드 |
| [2] | `L6_Link2_1_L7_Link3_1` | hinge (z) | 제2 커넥팅 로드 |
| [3] | `L3_Motor2_1_L4_Motor2_Shaft_1` | hinge (z) | 피니언 — Motor2 구동 |
| [4] | `L6_DcutShaft_1_L7_Holder_Bearing2_1` | hinge (z) | 베어링 홀더 2 |
| [5] | `L6_DcutShaft_1_L7_Holder_Bearing1_1` | hinge (z) | 베어링 홀더 1 |
| [6] | `L1_Guide1_1_L2_Left_Wall1_1` | slide (−x→Y) | 좌측 벽 선형 이동 |
| [7] | `L2_Linear_bush_1_L8_Link3_Shaft_1` | slide (x→Y) | **슬라이더 — Crusher 충돌판** |

> 초기 keyframe `crank_90deg`: qpos = `[π/2, 0, 0, 0, 0, 0, 0, 0]`  
> 시뮬레이션 초기화 시 크랭크를 **−π/2**로 덮어씌움 (`data.qpos[0] = -π/2`).

---

## 9. 정제(Tablet) 배치 좌표

정제는 **mocap body**로 삽입되어 충돌 벽으로 동작합니다.

| 항목 | 값 (world frame) |
|------|-----------------|
| X 위치 | **−47.879 mm** (충돌판 중심 정렬) |
| Y 위치 | `(WALL_Y_MM − half_thickness)` = 336.199 mm − t/2 |
| Z 위치 | **50.108 mm** (수직 높이) |
| 기본 밀도 | **1,200 kg/m³** |
| 자세 쿼터니언 | `[√2/2, 0, √2/2, 0]` → 장축 world-Y 방향 |

정제 두께 계산 (코드 기준):

```python
cd      = CV * 2 * R_mm          # crown depth [mm]
th      = R_mm * 0.20 + 2 * cd   # 총 두께 [mm]
half_th = th / 2.0
```

---

## 10. MuJoCo 솔버 설정

| 설정 | 원본 XML | 시뮬레이션 런타임 (`crusher_velocity_ctrl.py`) |
|------|---------|----------------------------------------------|
| solver | Newton | Newton (변경 없음) |
| iterations | 50 | 50 |
| timestep | 0.002 s | **0.001 s** (2배 세밀화) |
| integrator | Euler (기본) | **implicitfast** |

### `implicitfast` 선택 이유

velocity actuator의 explicit Euler 안정 조건: `kv × dt / J_eff < 2`

| 조건 | kv | dt | J_eff | 안정 지수 | 판정 |
|------|----|----|-------|----------|------|
| 원본 (크랭크 관성만) | 14.9 | 0.001 s | ~4.2e-5 kg·m² | ≈ 354 | 발산 ✗ |
| J_REFL body 주입 | 14.9 | 0.001 s | 0.324 kg·m² | ≈ 0.046 | 이론 안정 ✓ |
| → Newton 솔버 실패 | — | — | 23,000:1 비율 | — | 실제 발산 ✗ |
| **implicitfast** | 14.9 | 0.001 s | 무관 | 무제한 | **무조건 안정 ✓** |

`implicitfast`는 velocity/position actuator 힘을 암묵적으로 선형화하여 kv 크기와 무관하게 안정을 보장합니다.

---

## 관련 파일

| 파일 | 역할 |
|------|------|
| [`MJCF/Crusher_IsaacSim_colored.xml`](../MJCF/Crusher_IsaacSim_colored.xml) | MJCF 원본 — 전체 body/joint/constraint/actuator 정의 |
| [`20260603/crusher_velocity_ctrl.py`](../20260603/crusher_velocity_ctrl.py) | 속도 제어 시뮬레이션 — implicitfast, forcelim=12.5 N·m |
| [`20260603/debug_headless.py`](../20260603/debug_headless.py) | 헤드리스 진단 스크립트 — 6채널 로그, PNG 출력 |
| [`docs/motor_spec.md`](motor_spec.md) | 모터·감속기·액추에이터 상세 사양 |
