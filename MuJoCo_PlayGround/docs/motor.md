# Motor & Actuator Specification

Crusher 시뮬레이션에 사용된 실제 모터 스펙과 MuJoCo 내 구현 방식을 정리합니다.

---

## 1. 실제 모터 — BL4281 (BLDC)

| 항목 | 값 | 단위 |
|------|-----|------|
| 모델 | **BL4281** | — |
| 종류 | BLDC (Brushless DC) | — |
| 정격(stall) 토크 | **0.185** | N·m |
| 무부하 속도 | **5,800** | RPM |
| 로터 관성 (J_motor) | **7.2 × 10⁻⁶** (`72e-7`) | kg·m² |

코드 상수 (`crusher_velocity_ctrl.py`):

```python
MOTOR_STALL_TORQ = 0.185      # [N·m]
MOTOR_NOLOAD_RPM = 5800.0     # [RPM]
MOTOR_INERTIA_KG = 72e-7      # [kg·m²]
```

---

## 2. 감속기

| 항목 | 값 |
|------|----|
| 감속비 (n) | **1 : 212** |
| 역구동 차단 | 있음 (self-locking) — 부하 시 크랭크 위치 유지 |

```python
GEAR_RATIO = 212.0
```

---

## 3. 크랭크 출력 (감속 후)

| 항목 | 계산식 | 값 | 단위 |
|------|--------|----|------|
| Stall 토크 (τ_crank) | 실측 역산: F_slider × r | **12.5** | N·m |
| 무부하 속도 | 5800 ÷ 212 | **≈ 27.4** | RPM |
| 환산 관성 (J_refl) | J_motor × n² = 72×10⁻⁷ × 212² | **≈ 0.324** | kg·m² |
| 전달 효율 (η) | τ_crank / (τ_motor × n) = 12.5 / (0.185 × 212) | **≈ 32 %** | — |

```python
_J_REFL          = MOTOR_INERTIA_KG * GEAR_RATIO**2   # = 0.3236 kg·m²
_TAU_STALL_CRANK = 12.5                                # [N·m]  실측 역산
```

---

## 4. 크랭크-슬라이더 기구

| 항목 | 값 | 단위 |
|------|----|------|
| 크랭크 반경 (r) | **20** | mm |
| 커넥팅 로드 길이 (L) | **80** | mm |
| 운전 속도 | **8** | RPM |

```python
CRANK_R_M = 0.020   # [m]
ROD_L_M   = 0.080   # [m]
TARGET_RPM = 8.0
```

슬라이더 힘 공식 (크랭크 각도 θ 기준):

```
F_slider = τ_crank / (r × sin θ)
```

| θ | F_slider | 비고 |
|---|----------|------|
| 90° | 12.5 / (0.020 × 1.000) = **625 N** | 실측 600–650 N ✓ |
| 60° | 12.5 / (0.020 × 0.866) = **722 N** | — |
| 30° | 12.5 / (0.020 × 0.500) = **1250 N** | TDP(사점) 근처 증폭 |

---

## 5. MuJoCo 액추에이터 구현

### 5-1. MJCF 원본 정의 (`Crusher_IsaacSim_colored.xml`)

```xml
<actuator>
  <motor name="Motor1_crank"
         joint="L3_Bevel_GearBox_1_L4_Shaft_1"
         gear="1"
         ctrlrange="-50 50" />
</actuator>
```

> 뷰어 단독 실행 시 사용하는 기본 토크 제어 액추에이터 (forcerange 미설정).

### 5-2. 시뮬레이션 런타임 오버라이드 (`crusher_velocity_ctrl.py`)

`_build_model()` 내에서 기존 `Motor1_crank` 요소를 제거하고 **velocity 액추에이터**로 교체합니다:

```xml
<velocity
  name="Motor1_crank"
  joint="L3_Bevel_GearBox_1_L4_Shaft_1"
  kv="14.9"
  gear="1"
  forcerange="-12.5 12.5"
  ctrlrange="-1.6755 1.6755" />
```

| 파라미터 | 값 | 의미 |
|----------|----|------|
| `kv` | **14.9** N·m·s/rad | 속도 게인 = τ_stall / ω_target |
| `forcerange` | −12.5 ~ +12.5 N·m | 크랭크 stall 토크 상한 |
| `ctrlrange` | ±2 × ω_target | 목표 각속도 범위 |
| `gear` | 1 | 감속비는 환산 관성으로 별도 반영 |

kv 계산:

```
ω_target = 8 RPM × (2π / 60) = 0.8378 rad/s
kv = τ_stall / ω_target = 12.5 / 0.8378 ≈ 14.9  N·m·s/rad
```

```python
MOTOR_FORCELIM   = _TAU_STALL_CRANK                                     # 12.5 N·m
VEL_KV_DEFAULT   = _TAU_STALL_CRANK / (TARGET_RPM / 60.0 * 2 * math.pi)  # ≈ 14.9
```

### 5-3. 환산 관성 주입

고감속비 감속기의 반영 관성을 `L4_Shaft_1` body의 `diaginertia`에 더해 수치 안정성을 확보합니다:

```python
# _build_model() 내부
inertial.set("diaginertia",
    f"{orig_i0 + _J_REFL:.6f} {orig_i1:.6f} {orig_i2:.6f}")
```

`_J_REFL = 72×10⁻⁷ × 212² ≈ 0.3236 kg·m²`

---

## 6. 수치 안정성 검증

MuJoCo velocity 액추에이터의 안정 조건: `kv × dt / J_eff < 2`

| 조건 | kv | dt | J_eff | 안정 지수 | 결과 |
|------|----|----|-------|-----------|------|
| 환산 관성 **미적용** | 14.9 | 0.002 s | ~4.2×10⁻⁵ kg·m² | ≈ 710 >> 2 | 발산 ✗ |
| 환산 관성 **적용** | 14.9 | 0.002 s | 0.324 kg·m² | ≈ 0.09 << 2 | 안정 ✓ |

> `timestep=0.001 s` (implicitfast) 사용 시 안정 지수 ≈ 0.046으로 더욱 안정적.

---

## 7. 시뮬레이션 파라미터 요약

| 상수 | 값 | 설명 |
|------|----|------|
| `GEAR_RATIO` | 212 | 감속비 |
| `MOTOR_STALL_TORQ` | 0.185 N·m | BL4281 정격 토크 |
| `MOTOR_NOLOAD_RPM` | 5800 RPM | 무부하 속도 |
| `MOTOR_INERTIA_KG` | 7.2×10⁻⁶ kg·m² | 로터 관성 |
| `_TAU_STALL_CRANK` | 12.5 N·m | 크랭크 stall 토크 |
| `_J_REFL` | 0.3236 kg·m² | 환산 관성 (J × n²) |
| `MOTOR_FORCELIM` | 12.5 N·m | velocity actuator forcerange 상한 |
| `VEL_KV_DEFAULT` | ≈ 14.9 N·m·s/rad | velocity actuator 게인 |
| `TARGET_RPM` | 8 RPM | 운전 속도 |
| `CRANK_R_M` | 0.020 m | 크랭크 반경 |
| `ROD_L_M` | 0.080 m | 커넥팅 로드 길이 |

---

## 8. 관련 파일

| 파일 | 역할 |
|------|------|
| [`MJCF/Crusher_IsaacSim_colored.xml`](../MJCF/Crusher_IsaacSim_colored.xml) | MJCF 원본 (motor 토크 액추에이터) |
| [`20260603/crusher_velocity_ctrl.py`](../20260603/crusher_velocity_ctrl.py) | 실제 모터 스펙 상수 정의 + velocity 액추에이터 오버라이드 + 환산 관성 주입 |
| [`20260527/crusher_tablet_sim.py`](../20260527/crusher_tablet_sim.py) | 인터랙티브 시뮬레이션 (MOTOR_CTRL = −0.5 N·m, stall 역전 포함) |
| [`20260526/crank_acceleration.py`](../20260526/crank_acceleration.py) | 크랭크·슬라이더 가속도 측정 (MOTOR_CTRL = 10 N·m 개루프 테스트) |
