# Digital Twin — 시뮬레이션 전략 노트

시뮬레이션에 대한 디테일한 전략들을 적는 공간.

---

## 목차

1. [개요](#1-개요)
2. [시뮬레이터 선정 전략](#2-시뮬레이터-선정-전략)
3. [Crusher](#3-crusher)
4. [Gripper](#4-gripper)
5. [Sample Bag](#5-sample-bag)
6. [Tablet](#6-tablet)
7. [FEM 방식으로 모델링](#7-fem-방식으로-모델링)
8. [발생할 수 있는 문제 사항](#8-발생할-수-있는-문제-사항)
9. [피드백](#9-피드백)

---

## 1. 개요

**Crusher 압착력 → 봉투 → 알약**: 3-way coupling.

- Crusher는 강체 모터 구동
- 봉투는 PBD
- 알약은 rigid

> **CPU 백엔드에서 안정성 모니터링 필수** (이전 보수적 권고대로).

---

## 2. 시뮬레이터 선정 전략

- 나의 기구는 현재 **Closed Kinematic loop**를 형성하고 있음. 따라서 Tree 구조로 풀거나, Equality constraint 같은 동등성 제약을 지원하지 않는 시뮬레이터는 어려울 수 있다. 비록 mimic joint 등으로 Crank-slider mechanism을 개선하는 방법을 고려해볼 수는 있으나, 이는 다항식(polynomial) 한 조건에서 푸는 것이 아닌 이상 정상적이지 않은 Output이 생성된다.
  - → **이러한 이유로 폐루프 시스템을 구현할 수 있는 시뮬레이터 선정이 필요하다. MJCF를 지원해야 한다.**
- 나의 기구는 현재 슬라이더 끝에 발생하는 **반력 프로파일**이 중요하다. 따라서 물리적 충실도가 높은 시뮬레이터를 선정하는 것이 중요했다. 최근 Nvidia의 동향을 보았을 때, Isaac Sim의 PhysX는 여러 가지 물리적 충실도에 있어 한계가 보인다.
- 개발 환경의 자유도가 높은 시뮬레이터가 좋다. 이를테면 오픈소스이거나, 개발진들과의 소통이 자유로운 시뮬레이터.
- 또한 위의 맥락에서 최근 계속해서 **업데이트를 활발히 진행**하고 있는 시뮬레이터가 좋다.
- Twin인 만큼 **공정 시뮬레이션**에 집중해야 할 것 같다. 사실 물리적 충실도가 높으면 매우 좋다고 생각하지만, 현실과는 괴리가 존재하기 때문에 이를 calibration 하는 과정이 매우 어렵다. 또한 현실에서의 고상 시료를 다루는 문제를 센싱하기가 어렵기 때문에, **공정 절차를 중심으로 시퀀스를 구성하는 것이 좋아 보인다.**
- 물리적 거동을 묘사하는 솔버에 대한 **수학적 모델이 명확**해야 한다. 수학적 모델이 명확하지 않다면, 논문을 작성할 때 문제가 발생할 수 있다.
- 파우더나, 알약이나, 강체나 다양한 재료들에 대한 계산이 이루어져야 한다. 따라서 Digital Twin 논문에서 늘 나오는 **Multi-Scale 문제**라고 볼 수 있고, 이를 해결하기 위해서 다양한 솔버가 지원되어야 한다. 우선 Rigid collision dynamics 계산은 무조건 해야 하고, 그 밖에 **PBD, MPM** 솔버 등이 지원되어야 한다.

> 이와 같은 이유로 **MuJoCo, Isaac Sim, Genesis** 중 **Genesis**를 선택하게 되었다.

---

## 3. Crusher

이 기구는 처음부터 끝까지 직접 설계한 기구이다. Geometry, motor, joint, link 등의 자세한 정보는 [`motor_spec.md`](motor_spec.md)나 [`Crusher.md`](Crusher.md) 파일을 참고하면 될 것이다.

### 3-1. Digital Twin을 위한 Step

1. **Fusion 360**을 통해서 설계를 한다.
2. 설계된 기구에 각 링크를 fixed joint, slide joint, revolute joint 등으로 접합 연결을 한다.
3. **Fusion2URDF** GitHub extension을 통해서 robot description file인 **URDF** 파일로 export 한다.
4. URDF 파일을 **MJCF** 파일로 변환하는 과정이 필요하다.
   1. URDF에서는 내가 사용한 crank-slider(4절 링크 기구)를 묘사하기 어렵다. 더군다나 active joint를 통해 passive joint의 각도를 모방하는 mimic joint는 polynomial한 함수로만 묘사할 수 있기 때문에, 회전운동을 선형운동으로 변환하는 방식의 Crank-slider 메커니즘에는 사용이 어렵다.
   2. 따라서 MJCF를 통해서 이를 묘사할 수 있는데, **동등성 제약 조건(Equality Constraint)**을 토대로 이를 묘사할 수 있다. 이는 제약을 통해서 두 링크를 강제로 붙어있게 만드는 방식이며, 실제로 MuJoCo 논문(**Convex and analytically-invertible dynamics with contacts and constraints: Theory and implementation in MuJoCo**)에서는 이 폐루프에 대해서 설명하고 있다.
   3. 크랭크 슬라이더 구조 상, **Ground–Crank–Connecting rod–Slider–Ground** 이런 식으로 이어져 있다. 나는 Slider를 두 부분으로 나눠, `link3`–`shaft` 식으로 rigid joint로 구성해 놓았다. 그 후 폐루프 구조를 끊어내기 위해 `link3`–`shaft` 부분의 Rigid Joint를 끊어낸 후, **Equality Constraint**를 통해서 묶어 놓았다.
5. MJCF로 변환한 파일을 **Genesis** 시뮬레이터에 업로드한다.
6. **Velocity control**로 제어해야 한다.
   1. Position control로 제어하게 되면 제약을 빠르게 따라가기 위해서 모든 링크가 분해되는 현상이 발생한다.

### 3-2. 디테일한 설명

- 후면에 있는 샘플백을 고정하기 위한 홀더나, Impact plate, Wall_1의 경우에는 충돌 판정이 원활히 되어야 한다. 따라서 이 둘을 제외한 mesh들은 실제로 **self collision이 발생하지 않아야 할 지역**이다. 물론 볼록분해로 분해할 수는 있겠으나, 필요가 없기 때문에 모두 **collision-free한 객체**로 둔다.
- 다만, 샘플백을 고정하기 위한 홀더(**Left_Wall**)의 경우에는, 충돌이 켜져 있어야 하며, impact plate를 통과해야 하기 때문에 **볼록분해를 수행**한 상태이다.
- **사용한 테크닉:** 볼록분해 / self collision을 피하기 위해 엔티티 대부분을 collision-free한 엔티티로 만들기.

---

## 4. Gripper

- 사용하고 있는 제품은 **Onrobot 사의 RG2-v2**로, 현재 description 파일을 찾아보고 있는데, 기본적으로 Gripper는 모두 폐루프 시스템이기 때문에, description file을 작성할 때 여러 테크닉이 필요하다.
  - **폐루프를 사용하면 안 되는 이유:** RNE(Recursive Newton-Euler) 계산 과정에서 각 조인트에서 발생한 운동학 트리가 아니라면 상위 노드로 올라가는 움직임이 무한히 반복되기 때문에, 일종의 모든 노드가 연결되어 있는 그래프를 사용하면 안 되고 **운동학 트리(kinematic tree)**를 사용해야 한다.
- 따라서 이런 시스템에서 다음 두 개의 가이드라인이 권고된다.
  1. Revolute Joint를 통해서 하나의 **Active Joint**를 만들고 다른 조인트들을 **mimic**하는 구조의 description.
  2. Mimic Joint를 사용하지 않고, 어차피 회전운동을 선형운동으로 평행시키는 조인트이기 때문에 **Prismatic Joint**로 description하는 방식.

  → 기존 description 파일(franka emika panda 등)에서는 **2번 방식**을 선택하는 것으로 보인다. 따라서 2번 방식으로 간다.

### 4-1. 상대적으로 차순위 문제 (완벽하게 하려고 하지 말자)

- 그리퍼가 현재는 Revolute Joint–mimic joint가 아니라 **Slide joint**를 사용하여 묘사하고 있기 때문에, 이를 해결해야 실제 환경과 비슷하게 동작시키는 것이 가능해진다. 그렇게 설계할 수 있도록 해야 한다. → 위에서 설명한 **1번 방식의 구현**.
- 이 방식으로 가야 하는 이유는 파지하는 위치도 Rev joint냐 slide joint냐에 따라서 달라진다. 또한 slide joint로 작동시킬 때 **시각적으로 기구가 분리되는 듯한 현상**이 포착된다.

---

## 5. Sample Bag

- **PBD 솔버**로 풀어내려고 한다.
  - PBD 솔버가 가지고 있는 제약 함수를 통해서 비닐봉투 느낌, 혹은 약포지 느낌을 묘사하려고 한다.
  - FEM 솔버로 푸는 것이 정확하긴 하나, 어느 정도 물리적 충실도는 양보하고 **빠른 속도**를 선택.
  - 변형체지만 강성이 강하고 어느 정도 바스락바스락거리는 종이 느낌이 존재하기 때문에, 이를 구현하기 위해서는 **bending에 대한 값**들을 좀 조절하는 것이 좋아 보인다.
- 추가로 파지할 때 디테일이 필요하다고 생각하는데, 파지할 때 PBD 솔버 특성상 **두 particle의 간격이 일정 이상으로 가까워지지 못하게끔 하는 제약 조건**이 따로 명시되어 있다. 따라서 일종의 닫으면서 잡아 올리는 행동을 할 때 이런 제약 조건이 깨질 수 있기 때문에 이에 대한 고려가 필요하다.
  - particle의 거리 최솟값에 대한 조정을 수행(불안정해질 수는 있음. 이는 제약식과 관련)하고, 파지력과 마찰을 통해 잡는 물리적인 행위를 묘사할 수 있는지 고려.
- 생각보다 **stiffness를 작게** 만들어야, Crusher 단계에서 샘플백을 고정하는 홀더가 역할을 잘 수행할 수 있음.
- 마지막으로 위와 연결되는 핵심이라고 생각하는데, 공정 절차가 비닐백에 강체를 담고 이에 충격을 가하게 되면 가루 형태가 만들어진다. 이는 **MPM 솔버, PBD 솔버** 등의 파우더를 표현할 수 있는 솔버를 사용해야 할 듯하다. crushing 공정이 끝난 샘플백을 매니퓰레이터를 통해 이송하고, 회수 장치에 들어간 후 MPM 혹은 PBD 재료의 파우더를 접시에 담는 것까지가 목표이다.
  - 문제는 **MPM → Rigid → PBD** 이 3단 커플링이 잘 될지가 의문이다.
- 샘플백 이송도 문제이다. 현재 파지를 해보고 있는데, 생각보다는 파지가 잘 되질 않는다.

### 5-1. 백엔드 이슈 — *해결됨* (2026-06-30)

이전: Genesis 1.0.0부터 Taichi → **Warp** 전환 시 5080/5090 호환 안 됨.
→ 관련 이슈: https://github.com/Genesis-Embodied-AI/genesis-world/issues/2942

**원인 판명**: GPU 호환 문제가 아니라 **그래픽 드라이버 버그**. 드라이버 업데이트 후
정상 작동 확인.

**현재 표준 (2026-06-30 결정)**:
- Genesis **v1.2.0** 으로 통일 (`C:\genesis-world\` source 기준)
- env: **`isaacsim`** (Python 3.11, Warp backend, uipc 포함)
- Genesis_env (Python 3.10, 0.2.1, Taichi) 는 deprecated — 신규 작업 안 함

> 과거 SAP 결과(`fem_uniaxial_20260629_*` 등)는 Genesis 0.2.1 + Taichi 산물이라
> 새 환경 결과와 직접 비교 시 백엔드 차이 고려 필요.

---

## 6. Tablet

타블렛을 모델링하는 일이 사실상 제일 어렵다고 볼 수 있다.

- Tablet을 모델링하여 샘플백 내에 떨어뜨리는 일이 중요 → **Rigid-PBD coupling**
- 이 문제는 예전에 봉투에 Rigid body 시뮬레이션 시 particle이 찢겨나가거나, rigid body인 알약이 PBD와 닿는 순간 시뮬레이션이 튕겨버리는 현상이 발생했음.
- **따라서 충돌 영역이 사전에 수학적으로 계산되어 있는 방식(예: `Genesis.Entity.box`, `Genesis.Entity.Sphere`)으로 접근하는 것이 좋아 보이는데, 이를 구현하는 방법을 알아야 함.**
- MPM 솔버로 강체 제약을 걸어주는 방식도 생각하고 있어야 할 것 같다. 이전 연구에서는 이런 연구가 존재했나?
- **피로파손 문제** — 압력:

---

## 7. FEM 방식으로 모델링

### 7-1. FEM의 대충 정의

**1. 무엇을 푸는가**

연속체(돌, 고무, 살 등)에 힘을 가하면 **모든 점**이 조금씩 움직인다. 이 "모든 점의 변위"를 정확히 풀려면 편미분방정식(PDE)을 무한 차원에서 풀어야 함 — 해석적으로 불가능. FEM은 이걸 **"잘게 쪼개서 근사"로 푸는 방법**이다.

**2. 핵심 아이디어 — "쪼개고, 보간하고, 합친다"**

**3. 풀이 흐름**

```
1. 형상 → tet 메쉬 분할 (전처리)
2. 각 요소의 "강성 행렬" K_e 계산   ← 재료(E, ν)·기하로 결정
3. 전체 행렬 K = Σ K_e 조립
4. 방정식 푼다:  K · u = f
                 ↑   ↑   ↑
              강성행렬 변위 외력
5. u 로부터 변형률 ε, 응력 σ 계산 (후처리)
```

**4. 왜 FEM이 강력한가**

- **임의 형상 OK** — 정사각형이든 정제든 사람 뼈든 메쉬만 만들면 적용.
- **불균질 재료 OK** — 요소별로 다른 E, ν 부여 가능.
- **다물리 결합** — 열·유체·전자기 등 같은 프레임워크에 통합.

**5. 약점 (Genesis 시뮬에서 의미 있는 것)**

- **메쉬 의존성**: 메쉬 거칠면 부정확, 너무 촘촘하면 느림.
- **큰 변형/분할 어려움** — 표준 FEM은 메쉬 토폴로지가 고정. **분쇄(fracture)는 표준 FEM의 한계** — XFEM, eroded element, 또는 MPM 같은 입자법으로 확장 필요.
- **접촉이 까다로움** — 강체-FEM 접촉은 coupler가 따로 처리 (Genesis의 `coupler_options`).

### 7-2. Input 파라미터

| 분류 | 항목 | 단위 | 설명 |
| --- | --- | --- | --- |
| 형상 | mesh (tet) | — | 사면체 메쉬. 표면 STL만 있으면 tetgen으로 자동 생성 |
| 재료 | E (Young's modulus) | Pa | 강성. 클수록 단단 |
| 재료 | ν (Poisson ratio) | — | 옆면 변형 비율 (0–0.5) |
| 재료 | ρ (density) | kg/m³ | 밀도 |
| 경계조건 | fixed nodes (Dirichlet) | — | 못 움직이게 할 노드 |
| 경계조건 | applied force (Neumann) | N | 직접 가할 외력 (Genesis는 rigid 접촉으로 대체 가능) |
| 솔버 | dt, substep | s | 시간 간격 |
| 솔버 | damping | — | 감쇠 (수치 안정성) |

### 7-3. Output 파라미터

| 범위 | 항목 | 단위 | 설명 |
| --- | --- | --- | --- |
| 노드 | 위치 / 변위 u | m | 각 노드가 얼마나 움직였나 |
| 노드 | 속도 v | m/s | 동적 시뮬에서 |
| 요소(tet) | 변형률 ε (tensor) | — | 얼마나 늘어났나 (3×3) |
| 요소(tet) | 응력 σ (tensor) | Pa | 단위 면적당 힘 (3×3) |
| 요소(tet) | Von Mises 응력 | Pa | σ의 스칼라 → 항복 판정 표준 |
| 요소(tet) | 주응력 (principal) | Pa | 최대 인장/압축 |
| 전체 | 반력 (reaction force) | N | 접촉면/고정점에 가해진 힘 → 분쇄력 측정 |
| 전체 | 변형 에너지 | J | 정제에 쌓인 에너지 |
| 전체 | 부피 변화 | m³ | 압축률 |
| **derived** | **균열 핵 위치 (crack nucleation site)** | (tet idx → world xyz) | **`argmax_e σ_I(e)` — 응력 집중부**. 단일 점이 아니라 *상위 N% percentile field* 로 추출하면 weak surface 후보. **pre-fracture 모델링(FEM.md §4.7)의 직접 입력.** |
| derived | 첫 파쇄 시점 t\* | s | `min{t : σ_I_max(t) ≥ σ_t}` — Regime I 의 F_threshold 도출 |

> 충돌된 힘들을 정제에 쌓인 에너지로 사용하고, 이를 N-time과 매칭.
> 균열 핵 위치는 FEM 가 *가장 잘 푸는* 양 (§7-5) — Twin → pre-fractured composite hand-off 의 핵심 인자.

### 7-4. 활용 — 예측과 캘리브레이션

**(a) 예측 (forward simulation)**

실험 전에 새 알약의 **F\* (파쇄 시점)**을 추정. Crusher 운동이 정확하게 모델링됐다면 ±20% 정확도 기대 가능 → 실험 횟수 절약.

> → 이 문제는 알약의 F\*가 이상적으로 한 번에 깨질 때의 이야기이다. 우리가 구해야 하는 건 **W\* (힘의 총량, 면적분)**이다.

**(b) 역추정 / 캘리브레이션 (inverse problem) — 디지털 트윈의 핵심**

실험 데이터(F, hardness)가 있을 때 FEM 시뮬을 fit 해서 **재료 파라미터(E, fracture stress σ_c, plastic threshold)**를 역으로 추출.

#### W\* (cumulative work to fracture) — Twin 레벨에서 측정

사용자의 800N 반복 타격 Crusher에서 F\*의 자연스러운 재정의는 **"몇 번 때려야 깨지나(N_f) + 그때까지의 누적 일(W\*)"**. 단일 임계력 F_threshold는 *재료 물성 파라미터*로 남고, 사용자의 측정·예측 타깃은 **N_f와 W\***가 되어야 함.

#### F\*의 새 정의 (3 후보)

**(a) F_threshold — "이론적 단일 타격 파쇄력" (재료 물성에 더 가까움)**
> "Crusher 한 번의 충격으로 깨려면 필요한 peak force"
- FEM으로 추정: σ_max(F_peak) = σ_t가 되는 F_peak.
- **800N과 비교**해서 regime 판정에 사용.
- 사용자 실험에서는 **간접적**으로만 측정 (사실상 hardness 시험 결과 ≈ F_threshold).

**(b) N_f (cycles to fracture) — "몇 번 때려야 깨지나" ✅ 반복 타격에 가장 자연스러운 정의**
> "800N peak 기준 첫 균열 발생까지의 stroke 횟수"
- 사용자 실험에서 **직접 측정 가능**: 첫 균열 보일 때까지 stroke 카운트.
- FEM + 손상 누적 모델(S-N curve, Paris' law 등)로 예측.
- 단점: Genesis 기본 FEM에는 손상 모델 없음 → 외부 후처리 필요.

**(c) W\* (cumulative work to fracture) — "누적 에너지" ✅ 강력한 대안**
> "첫 균열까지 Crusher가 알약에 전달한 총 일 [J]"
- 정의: `W* = Σ_strokes F(t) · v(t) dt` (각 stroke의 일 합)
- 사용자 실험 측정: torque × angular velocity × time 적분, 또는 reaction force × displacement.
- 재료 물성과 잘 매핑됨 (단위 부피당 fracture energy G_c와 직결).
- **N_f보다 더 본질적** — stroke별 일량 차이를 자동 흡수.

#### Regime 분류

| Regime | 조건 | 거동 | 의미 있는 출력 |
| --- | --- | --- | --- |
| I. 즉시 파쇄 | F_peak (800N) ≥ F_threshold | 첫 stroke에 파쇄 | F_threshold (전통적 F\*) |
| II. 누적 손상 | F_peak < F_threshold 이지만 σ_max > σ_t·k | 여러 stroke 누적 후 파쇄 | N_f, W\* |
| III. 영구 안 깨짐 | F_peak < critical & σ_max ≪ σ_t | 깨지지 않음 | (해당 알약 분쇄 불가) |

- **Pre-fractured rigid composite** — 정제를 미리 여러 rigid 조각으로 만들고 weak constraint로 묶어, 응력 한계를 넘으면 constraint를 끊기. Genesis 안에서 hack으로 구현 가능.

### 7-5. 왜 1차 파쇄는 FEM이 맞나

- 1차 파쇄 직전까지 정제는 **연속체** — continuum 가정 유효.
- 균열 *핵*의 위치/시점은 **응력 집중부**에 의해 결정 — FEM의 응력장 분해능이 높음.
- 1차 분쇄력(F\*)은 **재료 강도 + 형상**의 함수 — FEM의 변동성(E, ν, σ_t)와 직접 매핑.

**MPM이 1차 파쇄에 약한 이유:**
- 입자 단위라 응력장이 **공간적으로 노이지** — 균열 핵 위치 부정확.
- 강성 계산이 부드러워서(smoothed) **임계 응력 도달 시점**이 모호.
- 큰 변형/유동 시 그 진가 발휘 — 1차 파쇄 전엔 과한 모델.

### 7-6. 워크플로우

```
[새 알약 도착]
   │
   ▼
[측정] shape (3D scan), mass, curvature, hardness
   │
   ├─→ ρ = mass/volume                  (직접)
   ├─→ σ_c ≈ Hardness / 접촉면적          (직접 추정)
   ├─→ literature: E, ν, σ_t 표준값        (정제 종류 알면)
   │
   ▼
[FEM 1차 시뮬] (literature 값으로) → 예측 N_f_sim, W*_sim
   │
   ▼
[Crusher 실험 1회] N_f_meas, W*_meas 측정
   │
   ▼
[캘리브레이션] σ_t·E 조정 → 시뮬 ≈ 실험 매칭
   │
   ▼
[검증된 material card] 저장
   │
   ▼
[새 형상 (같은 처방) 들어옴] → Forward 예측만, 실험 생략 가능
```

### 7-7. 접촉 모델링 주의 (Rigid–FEM)

→ Rigid plate가 FEM tablet을 누를 때, **표면 노드의 위치**만 보고 contact pair를 만들어 force를 인가한다. 내부 노드는 이웃 element의 strain 변화로 *간접* 전달.

**(1) Tunneling (관통) — 가장 흔한 문제**

Rigid가 빠르게 움직이면 한 step 동안 표면 vertex 사이의 공간을 *건너뛰어* 내부로 들어옴. 표면 노드 충돌 없으면 → contact 미감지 → rigid가 FEM 내부로 박힘.
- **예방**: `dt` 작게, rigid 속도 제한.

**(2) Sparse mesh에서 정확도 손실**

표면 vertex가 듬성듬성하면 vertex 사이의 face 중간으로 rigid가 닿아도 충돌 안 잡힘. **소형 rigid + 듬성한 FEM mesh**가 위험.
- **예방**: tet 메쉬 더 촘촘하게(`scale` 줄이기, tetgen 옵션). 정제의 경우 12,892 tet 정도면 일반적으로 OK.

**(3) Rigid plate 구동 — `set_pos` 금지, velocity control 사용**

Rigid plate를 일정 속도로 누르려고 매 step `set_pos`로 위치를 강제 텔레포트하면 **반드시 계산 문제가 생긴다**:

- `set_pos`는 **이산적(hard) 위치 덮어쓰기** — 속도/가속도와 일관성 없음. 솔버 내부 상태(rigid mass, momentum, contact predictor)가 그 점프를 *연속적인 운동*으로 해석하지 못함.
- SAP coupler 입장에서 plate 속도가 0 (또는 정의되지 않음) → contact 임펄스를 보내지 않다가 penetration이 깊어지면 한 번에 큰 임펄스 폭발 → 정제 튕김 → 또 무접촉 → spike만 반복되는 *불연속 contact 거동*.
- 시각적으로는 plate가 FEM mesh를 *통과*하는 것처럼 보임 (Genesis FEM 정제 + 1 μm 두께 compression 케이스에서 확인됨, 2026-06-29).
- 게다가 `set_pos` 직후의 한 step 사이에 외력(중력 등)이 누적되면 의도한 속도의 1000× 노이즈가 끼어 더 망가짐.

**원칙**: rigid plate든 grinder의 driver든, **운동학적으로 움직여야 하는 강체는 control_dofs_velocity (Genesis) 같은 PD-controlled velocity target을 써야** 솔버가 운동을 *연속적인 속도장*으로 인식하고 contact impulse도 매 step 정상적으로 흐른다. position 제어가 정말 필요하면 prismatic joint + velocity control로 우회.

(이 교훈은 §3 Crusher 의 "velocity control" 권고와 같은 맥락 — *position 강제는 closed loop든 single body든 항상 깨진다*.)

---

## 8. 발생할 수 있는 문제 사항

- 실제 운용 속도와 맞을지는 의문.
- 파지할 때 마찰과 파지력으로 집는 것이 아님 → **weld**.
- 샘플백의 파라미터를 튜닝하는 일이 필요.

---

## 9. 피드백

**한 박사님:**
- 중요한 것은 화학자들이 어떤 것을 얻을 수 있을까?
- DT를 통해서 보고 싶은 건, **이걸 얼마만큼 때려야 부서지는가?**이고, 이걸 물리적으로 묘사할 수 있어야 함. 그게 안 된다면 적어도 **Data-driven**하게 풀 수는 있어야 함.
- 그게 안 되면 큰 의미는 없어 보임.

**남은 과제:**
- 피로파손 – 압력.
- 알약의 경도와 피로파손을 어떻게 풀 수 있을까.
