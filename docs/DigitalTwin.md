# Digital Twin — 시뮬레이션 전략 노트

시뮬레이션에 대한 디테일한 전략들을 적는 공간.

> 관련: 실험·트윈·한계 데이터 구분과 S-N 재검토는 [`DataInventory.md`](DataInventory.md) 참고.

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

### 6-1. Primitive object 가설과 FEM 적용성

**가설 (내 생각):**
- Rigid body collision에서 사전 작업한 메쉬 파일(**Primitive object** — 충돌 영역을 수학적으로 사전 정의해 두는 방식)이 굉장히 좋은 성능을 보인다.
- 캡슐도 Primitive object로 만들 수 있다면 충돌 성능이 상당 부분 개선될 것이라는 가설. 다만 이 방식이 **FEM에서도 효과적인지는 불분명**하다.

**분석·전략 (Claude):**

1. **Rigid-rigid에서 primitive가 빠른 이유 — 가설 맞음.** Primitive는 충돌 영역이 닫힌 수식(analytic)으로 정의된다. 예: **캡슐 SDF = "선분까지의 거리 − 반지름"** — 곱셈 몇 번. BVH 트리 순회도, 삼각형-삼각형 테스트도 없다. 거리·법선이 **정확**하고 **C¹ 연속**(부드러움)이라 contact normal이 안 튀어 솔버 수렴도 좋고, 닫힌 SDF라 **연속충돌(CCD)·관통 판정**이 쉬워 터널링에 강하다. 메쉬 충돌은 반대(테셀레이션 정밀도 의존, 스텝 사이 빠른 통과를 놓침). Genesis **rigid solver에서 캡슐은 이미 primitive**라 별도 작업이 필요 없다.

2. **FEM 적용성 — 절반만 맞고, 그게 핵심.**
   - **변형체(정제) 자체는 primitive 불가.** FEM의 본질은 표면이 변형장에 따라 매 스텝 움직이는 것이라, 고정된 캡슐/구 수식으로 표현이 불가능하다. FEM 접촉은 **태생적으로 표면 메쉬 기반**(surface vertex/element).
   - **하지만 접촉 쌍의 *rigid 쪽*(plate·공구)은 primitive면 큰 이득.** FEM 표면 vertex ↔ primitive 거리를 analytic하게 계산하면 테셀레이션 없이 정확·저렴·터널링에 강하다.

3. **그런데 Genesis SAP coupler는 이걸 안 한다 — 이게 §7-7 터널링의 구조적 원인.** SAP coupler 구현(`sap_coupler.py`)은 rigid collider를 primitive로 쓰지 않고 **Box plate조차 trimesh → tet 메쉬로 변환**(`mesh_to_elements`)하며, **primitive plane은 명시적으로 금지**(`GEOM_TYPE.PLANE` → raise). 접촉은 `FEMSurfaceTetLBVH` ↔ `RigidTetLBVH`, 즉 **BVH 기반 메쉬-메쉬 이산 접촉**이다. → plate가 한 스텝에 정제 표면 tet 두께를 넘게 움직이면 BVH가 겹침을 놓쳐 관통.
   - 반대로 **IPC는 이 analytic primitive 접촉을 실제로 가지고 있다**(`IPCVertexHalfPlaneNormalContact` = half-plane primitive vs FEM vertex). IPC가 매력적이었던 또 하나의 이유. (단, 현재 RTX 5070에서 Genesis→IPC 경로가 런타임에 죽는 별도 이슈 존재 — §5-1 환경 노트 참고.)

4. **전략 결론.** SAP에 머무는 동안엔 coupler에 primitive를 쓸 수 없으므로, 터널링은 **이산 메쉬 접촉 아티팩트**로 보고 다음으로 누른다: **dt 축소 / substeps 증가**(스텝당 plate 변위 < 표면 tet 크기), **plate 속도 ↓·ramp**, **접촉부 메쉬 세분화**, **plate tet 두께 확보**. primitive contact의 "정답형"은 결국 IPC였다는 점은 기록해 둔다.

### 6-2. Crusher와의 상호작용 — 실측 반력 프로파일 구동 FEM (2026-07-04)

정제가 시뮬 안에서 받아야 하는 힘은 임의 값이 아니라 **실기 Crusher가 실제로 가하는 힘**이다.
이 절은 "Impact plate 접촉이 모델링됐는가?"라는 질문에 대한 답이자, 정제 하중을
실측으로 앵커링하는 현재 방법을 정리한다.

**1) 실기에서 정제가 받는 힘의 정체 — 지령이 아니라 창발.**
크랭크-슬라이더에서 슬라이더가 정제(또는 벽)를 누르는 힘은 `F = τ_crank/(r·sinθ)` 로
기구가 stall 하며 *창발*한다 ([Real2Sim.md](Real2Sim.md) §1). v3 FSM 은
STRIKE(접촉→stall) → RETRACT 사이클을 반복하고, 그 결과가 ForceGage 실측 반력
프로파일(`Crusher_Genesis/Real_result/반력프로파일/{θ}deg.txt`, 5 Hz,
0 → 상승 → stall plateau(~515 N @ 60°) → 해제 → 0)이다. **정제를 두면 부서지기
전까지는 특정 접촉각에서 이 프로파일에 준하는 하중을 받는다**는 것이 대전제.

**2) Impact plate 접촉은 직접 모델링하지 않는다 (못 한다).**
SAP soft contact 는 2 GPa 정제에 하중을 싣지 못하고 plate 가 관통한다(§6-1, §7-7).
그래서 plate–정제 접촉 대신 **타격면 노드를 kinematic Dirichlet 로 구동**한다.
단, 임의 변위 펄스를 주는 것은 물리적으로 무의미하므로 **하중의 크기·시간형상을
실측 반력 프로파일로 강제**한다:

```
① 프로브: 작은 변위 d_probe 압입 → 벽면 σ·n 적분 반력 → 정제 강성 k = F/d 측정
② 실측 프로파일에서 대표 타격 사이클 1개 추출 (peak가 중앙값인 사이클)
③ 피드포워드 d(t) = F_meas(t)/k 로 타격면 구동   ← linear corotated 준정적 → F ∝ d
④ 매 스텝 벽면 반력 F_sim(t) 후처리 → F_meas(t) 와 겹쳐 추종 검증
```

구현: [`Crusher_Genesis/FEM/fem_stroke_real_profile.py`](../Crusher_Genesis/FEM/fem_stroke_real_profile.py).
60° 프로파일 기준 결과: k = 4.41×10⁶ N/m (R4.0/AR1.00/CV0.20, E=2 GPa 가정),
F 추종 RMS 오차 1.4% (peak 515 N). stall plateau 동안 σ_III 압축 기둥과 σ_I
측면 인장 링(균열 핵 후보, §7-8 연계)이 유지되다가 RETRACT 에서 소멸하는
주응력장 시계열을 컷어웨이로 출력한다
(`Sim_result/fem_real_profile_{θ}deg_*_sigI/sigIII_cut.png`).

**3) 가정과 한계 — 해석 시 주의.**
- **프로파일 전이 가정**: 실측은 ForceGage(강체 벽)를 누른 값. 정제가 있으면
  접촉 강성이 낮아져 stall 각도·plateau 힘이 다소 달라질 수 있다. "부서지기 전
  준정적 구간에서는 같은 프로파일을 받는다"는 근사가 대전제.
- **elastic-only**: 파단이 없으므로 515 N 을 끝까지 받는다. 실제 정제는 도중에
  깨진다 — 이 필드는 "깨지기 직전까지의 응력 발달 + 균열 개시 위치(σ_I max)"
  예측용이고, 파단 판정은 §7-4/§7-8 Weibull 캘리브레이션이 담당.
- **E 미캘리브레이션**: 힘은 실측으로 고정되지만 변위(d_max=117 μm @ E=2 GPa)는
  모델 의존. E 를 캘리브레이션하면(§7-8) 변위·강성이 같이 맞는다.
- **접촉 patch 근사**: 타격면 = 상단 10% 노드 band 고정 구속. 렌즈형 정제라
  실제 plate 접촉 patch 와 비슷하지만, Hertz 식 접촉 확장(하중↑ → patch↑)은 없다.

**4) real vs sim 구동 오차 — 트윈은 sim 반력으로 돌아야 한다 (2026-07-04).**
디지털 트윈은 매번 ForceGage 실측이 아니라 **시뮬이 만든 반력**(Crusher_8env.py 의
MuJoCo 벽 반력)으로 검증한다. 그래서 같은 FEM 정제를 real F(t) / sim F(t) 두 프로파일로
각각 구동해 주응력장을 비교했다
([`fem_profile_compare.py`](../Crusher_Genesis/FEM/fem_profile_compare.py),
sim 시계열은 [`extract_sim_profile.py`](../Crusher_Genesis/FEM/extract_sim_profile.py) 로 추출).

- **핵심 관찰 — 3D 필드 오차 = 반력 프로파일 오차로 환원.** linear_corotated 준정적이라
  `σ(x,t) = σ_shape(x)·F(t)/k` — 공간패턴 σ_shape 는 구동원과 무관하게 동일. 그래서
  "3D 응력장을 어떻게 비교하나"는 (A) 시간영역 F·σ_max(t) 겹치기, (B) 공간 Δσ 컷어웨이,
  (C) tet별 σ_real vs σ_sim 산점도(R²·기울기) 3장으로 응축된다.
- **60° 결과**: sim peak 533 N vs real 515 N (+3.5%) → σ_I peak 698→733 MPa (+5.1%),
  필드 rel-L2 오차 4.7%, RMSE 2.2 MPa. **산점도 R²=0.998, 기울기 1.046** —
  즉 두 필드는 거의 완전한 **스케일 복제**.
- **결론**: sim/real 반력 오차는 **파쇄 개시 *위치*(σ_I max 지점)에는 거의 영향이 없고
  *크기*만 ~5% 스케일**한다. 위치 예측(균열 핵)은 sim 구동으로도 견고하고, Weibull
  파단확률(σ 크기 민감)만 반력 캘리브레이션 정확도에 영향받는다.
- **형태 차이 하나 — 해소(2026-07-06).** 종전 sim(Crusher_8env)은 stall 감지 즉시 STRIKE 를
  끊어 **짧은 스파이크(~0.25 s)** 라 real 의 stall dwell(plateau ~1 s) 사다리꼴과 형태가
  어긋났다. `_sim_logged` 에 **stall dwell**(접촉 유지 → plateau, `STRIKE_DWELL_S=1.0 s`
  flat + rise settle ≈4τ)과 **접촉력 1차지연**(`CONTACT_TAU=0.25 s`, rigid-wall 스텝응답
  → 완만한 rise/fall = 정제 탄성변형 근사)을 넣어 sim F(t) 도 **천천히 상승 → dwell → 천천히
  하강** 하는 실측형 사다리꼴이 되게 했다. **peak 캘리브값은 보존** — dwell plateau 를 stall
  검출순간 힘 `Fpk`(= 종전 needle peak)에 고정하므로 low-pass 정상상태 과증(rigid-wall
  674 N)로 새지 않는다. rising-edge 정렬 오버레이(`overlay_60deg_pulse.png`)에서 sim/real
  펄스가 상승슬로프·plateau·peak(525 vs 518 N)까지 거의 겹친다. 준정적·elastic 필드
  비교(σ∝F)엔 무영향, 점탄성/피로 확장 시 dwell 정합만 개선.

**5) 결정 노트 — real vs sim, 무엇으로 FEM을 구동하나 (2026-07-04).**

> **질문**: 실측 반력을 그대로 따라가면 되는데, Real2Sim으로 만든 sim 반력을 따라가는 건
> 오차(+5%)만 얹고 무의미하지 않나?

**결정: real과 sim은 경쟁이 아니라 한 파이프라인의 두 단계다. 측정한 조건은 real,
미측정 조건은 sim.**

| 단계 | 구동원 | 용도 | 근거 |
| --- | --- | --- | --- |
| 앵커·검증 | **real** (ForceGage) | 측정한 조건 (정제 1종·8각도·1 RPM) | real이 있으면 엄밀히 우월. 여기서 sim은 손실 복사본이라 **쓰지 않는다** |
| 예측·생산 | **sim** (Crusher_8env) | 미측정 조건 (새 각도/RPM/공구·온라인 폐루프) | real이 **존재하지 않으므로** sim이 유일한 반력원. 단 검증된 오차범위 내 |

- **측정한 조건에서는 real이 정답이 맞다.** sim은 그 real을 근사한 것이라 +5% 오차만
  더한다. 60°처럼 실측이 있으면 real로 구동한다. — 이 지적은 옳다.
- **트윈의 존재 이유는 "측정 안 한 조건"의 예측이다.** 정제 STL 1000+종·임의 각도·
  다른 RPM엔 따라갈 real 프로파일이 없다. 그 순간 반력을 낼 수 있는 건 시뮬레이터뿐이라,
  sim-driving은 "굳이 sim을 쓰는 것"이 아니라 **real이 없는 곳에서 유일하게 응력장을 내는
  경로**다.
- **§6-2의 real↔sim 비교(4번)는 생산이 아니라 검증이었다.** real·sim이 둘 다 있는 유일한
  케이스(60°)에서 **sim 경로의 오차를 값매김**한 것 = "위치 R²=0.998, 크기 +5%". 이게
  real이 없는 조건에서 sim을 믿어도 된다는 **사용 허가증**이다. 검증 없이 sim을 쓰면 근거가
  없고, real만 쓰면 커버리지가 측정한 8점에 갇힌다.
- **정직한 현재 한계 — sim은 아직 *형상*으로는 일반화 못 한다.** Crusher_8env는 정제 대신
  **강체 벽**을 눌러 F를 뽑으므로, sim의 F는 각도/RPM/stall토크로는 외삽되지만 **정제 형상엔
  반응하지 않는다**(벽이 rigid). 따라서 두 일반화 축의 출처가 갈린다:
  - **반력 F 일반화** → 크러셔 sim (각도/RPM/공구 축) *— 형상은 아직 ✗*
  - **형상별 응력장 σ 일반화** → **FEM** (같은 F를 1000개 STL에 주입 → 형상별 σ)
  → 오늘 기준 sim-driving이 실제로 사주는 값은 **각도/RPM/온라인 폐루프** 축이지 형상 축이
  아니다. 형상까지 sim의 F가 반응하게 하려면 크러셔 sim 루프에 정제 compliance를 넣어야
  하는데, 그게 §6-1·§7-7의 SAP 커플링 문제다.

**스코프 의존**: 만약 이 프로젝트가 측정한 8개 조건만 다루면 되면, sim-driving은 빼고 real만
쓰는 게 맞다. 미측정 조건 예측·온라인 트윈이 목표이면 sim 경로가 필수이고, 그 신뢰도는
위 검증(±5%, 위치 견고)이 보증한다.

**6) 로드맵**: θ = 0~120° 프로파일 8개를 스윕해 각도별 σ_I field → 접촉각이 파쇄
위치·확률에 주는 영향 정량화 → Weibull 파단 모델(§7-4)과 결합.

**7) 준정적 F–θ 반력 캘리브레이션 (Crusher_8env, 2026-07-06).**
§(4)의 60° 단일 각도 필드 비교와 별개로, **여러 접촉각에 걸친 sim 벽 반력의 절대 크기**를
실측 robust-mean에 맞추는 스칼라 보정. 준정적(8 RPM) 운전이라 각 각도의 대표값은 peak
반력 1개로 응축되고([`Crusher_8env.py`](../Crusher_Genesis/Crusher_8env.py), MuJoCo FSM
8-env, N_CYCLES=25 strike의 mean peak), 이를 F–θ 곡선으로 실측과 겹친다.

- **보정 방식 — 원점 통과 최소자승 단일 스칼라.** 물리(각도 의존 형상)는 sim이 내고,
  힘 게인만 한 개 상수로 맞춘다:
  ```
  FORCE_SCALE = Σ(F_sim·F_meas) / Σ(F_sim²)      # 원점 통과 LSQ (각도별 아님)
  ```
  코드에선 스트라이크마다 `Fn = F_raw · FORCE_SCALE · sf`(sf=노이즈)로 적용.
- **105° 제외.** 실측 robust-mean이 394 N 으로 인접 각도(90°=503, 120°=563) 대비 급락 —
  sim 대비 −31%로 다른 각도(±수%)와 성격이 다른 **이상치**라, 스케일 재적합·플롯·`ANGLES`·
  `MEASURED`에서 모두 제거(제거 후 6점으로 재적합).
- **결과: `FORCE_SCALE` 0.98 → 0.9453** (105° 제외 재적합). 각도별 정합:

  | θ [°] | sim (보정) [N] | measured [N] | 오차 |
  |---|---|---|---|
  | 30 | 811 | 896 | −9.4% |
  | 45 | 591 | 596 | −0.8% |
  | 60 | 518 | 515 | +0.5% |
  | 75 | 493 | 456 | +8.1% |
  | 90 | 506 | 503 | +0.6% |
  | 120 | 642 | 563 | +14.0% |

  **평균 절대오차 5.6%.** 중간각(45–90°)은 거의 완전 정합, 양 끝(30°↓·120°↑)은 잔차가
  남는다 — **단일 스칼라는 크기만 맞추지 곡선 *형상*은 못 고친다**(§(4)의 "위치는 견고,
  크기만 스케일"과 같은 성질). 형상까지 맞추려면 각도별 보정이나 위상보정
  `PHASE_OFFSET_DEG`·stall 토크 재튜닝이 필요.
- **출력**: [`Sim_result/sim8env/F_vs_theta_sim.png`](../Crusher_Genesis/Sim_result/sim8env/F_vs_theta_sim.png)
  (y축 0 기준 + measured ±10% 밴드 + 평균오차 주석). robust-mean·스케일은
  [`Crusher_8env.py`](../Crusher_Genesis/Crusher_8env.py) 상단 `MEASURED`/`FORCE_SCALE`.
- **§7-8 재료 캘리브레이션과의 관계**: 이 F–θ 보정은 **하중(반력) 스케일**을 고정하는
  단계로, §7-8이 역보정하는 **재료 항(E·σ_t)** 보다 상류다. σ_I 절대값 = (반력 스케일)×
  (E 스케일)이므로, 여기서 반력을 실측에 앵커해 두면 §7-8의 E·Weibull 역보정이
  반력 불확실성과 분리된다.

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

### 7-8. 한 알약 예시 — 캘리브레이션 워크플로우 (worked example)

§7-4·§7-6을 **구체 수치**로 굴린 예. 대상: 원형 biconvex 정제
`tablet_R4.0_AR1.00_CV0.20` (지름 D=8 mm, 전체두께 t=3.5 mm, 밴드 W=2 mm,
≈200 mg, MCC 주성분).

**용어 — "조정 항" vs "맞춤 표적"** (이 구분이 캘리브레이션의 핵심)

| 구분 | 정의 | 예 |
|---|---|---|
| **캘리브레이션 항 (조정)** | 몰라서 돌려 맞추는 미지 파라미터 | **E**¹, ν, (σ_t), Weibull m, ~~G_c~~², **손상규칙 계수**, 접촉/봉투/전류 |
| **표적 (맞춤)** | 실험으로 재서 트윈이 재현해야 하는 출력 | F(δ) 곡선, Brazilian σ_t, **F_peak–N_f**, 균열위치 |

> **N_f·W\* 는 항이 아니라 표적**이다. σ_t·손상계수(항)로부터 *예측*하고, 실측 N_f에
> 맞춰 계수를 조정한다 (DataInventory.md §E-3와 동일 구조).

**¹ E는 "조정 항"이 아니라 장비에 따라 "입력"으로 격상된다.** E를 직접 재는 "E 미터기"는
없다 — 어떤 장비든 F(δ) 곡선을 재고 *모델로* E를 역산할 뿐이다. 따라서 성격이 갈린다:
- **UTM + 규칙 시편**(평면 compact·굽힘 beam): 곡선→E가 닫힌 공식(`σ=Eε`) → **직접
  측정 → 입력으로 고정**(강한 prior). STEP 2 역보정 불필요.
- **통짜 biconvex 정제만**: 곡면 Hertz 접촉 탓에 닫힌 공식 없음 → **FEM back-fit**으로만
  E를 뽑음 → *이때만* E가 "조정 항"으로 남는다(STEP 2).
- **UTM 없음**: E를 문헌값(MCC ~4–9 GPa)으로 고정 → 아래 **저자원 경로**.

**² G_c 는 목록에 있으나 실제로는 W\* 로 대체한다.** G_c 는 *단위 균열면적당* 에너지
[J/m²]라 총량 환산에 균열 면적 A가 필요(`W=G_c·A`). 깨끗한 1개 균열이면 A≈단면적으로
가능하지만, **가루 분쇄는 새 표면적 A가 방대·측정 불가**(분쇄공학 Rittinger/Bond 법칙이
이 때문에 존재). → 트윈은 G_c·A 대신 **바깥에서 잰 일 `W*=∮F·v dt`** 를 직접 표적으로
삼아 우회한다(§7-4).

**주응력장(σ_I)과 σ_t의 역할**
- σ_I = 각 점에서 전단이 0인 3개 주축의 수직응력 중 최대(가장 인장쪽). σ_I 장 =
  **인장 집중 = 균열 개시 위험 지도**.
- 취성 파괴는 최대 인장 주응력이 구동(Rankine): **σ_I ≥ σ_t 이면 그 자리에서 균열**.
- σ_I **방향·상대분포**는 노드 변위만으로(물성 없이) 나온다. **절대값[MPa]** 만
  E(스케일)+ν(텐서 혼합) 필요 → 값이 없으면 실측 힘으로 E를 역보정.
- **파쇄에 직접 관여하는 물성은 σ_t(저항) 하나, 구동자는 σ_I(하중).** E·ν·형상은
  σ_I 를 *만드는* 간접 인자(하중→σ_I "환율")일 뿐 파쇄 판정 자체엔 관여 안 한다.

**STEP 0 — 형상 → 메쉬**: STL → tet 분할 (예: 노드 2745, tet 12005).

**STEP 1 — 실험으로 표적·입력 수집**

| 시험 | 결과 | 용도 |
|---|---|---|
| 단축 압축 F–δ | 힘–변위 초기 기울기 | → E (규칙 시편이면 직접, 통짜면 back-fit 표적) |
| Brazilian(직경압축) = **정제 경도계** | F_break = 80 N | → σ_t (Pitt 식, **E 불필요**) |
| 위 30개 반복 | 파괴하중 분포 | → Weibull m ≈ 8 |
| Crusher 반복 타격 | F_peak별 파쇄 타수 | → F_peak–N_f 표적 (=취득 가능한 핵심 실측) |

볼록면이라 σ_t 는 **Pitt 식**:
`σ_t = 10F/(πD²)·[2.84·t/D − 0.126·t/W + 3.15·W/D + 0.01]⁻¹ = 800/(π·64)/1.82 ≈ 2.2 MPa`

**STEP 2 — E 확정**: *장비에 따라 경로가 갈린다(각주 ¹).*
- **규칙 시편 UTM 있음** → `σ=Eε` 로 E를 직접 읽어 **입력 고정** (이 STEP은 측정 1줄로 끝).
- **통짜 정제만**(이 예시) → **FEM back-fit**: 강제변위 FEM 반력 F(δ)를 실측 기울기에 맞춰
  E 조정. 가정 E=2 GPa → 반력이 실측의 1/3 → **E ≈ 6 GPa** 로 수렴(MCC 현실값), ν=0.25
  문헌 고정.

이제 σ_I 절대값[MPa]이 물리적 의미를 가진다.

**STEP 3 — 단일 타격 σ_I → σ_t 판정**: 보정된 E로 한 타격의 σ_I 장 계산 →
σ_I,max ≥ σ_t(2.2 MPa)이면 그 지점 1타 파쇄, **F_threshold** 산출 + 균열 개시 위치
확정. 미달이면 STEP 4.

**STEP 4 — 손상누적 → N_f 예측 (2순위 항)**:

> **σ_I 는 매 타격 올랐다 내려오는 *순간값*(타격마다 리셋), 누적되는 건 손상 `D`다.**
> σ_I 는 매 타격 D 를 *얼마나 늘릴지* 정하는 입력일 뿐 자신이 쌓이지 않는다.

손상변수 `D ∈ [0,1]` (0=온전, 1=파쇄). 매 stroke i 의 peak σ_I 로 증분을 더한다:

`D = Σ_i ΔD_i,   ΔD_i = (σ_I,i / σ_t)^p,   D ≥ 1 → 파쇄 (그때 타수 = N_f)`

- **σ_I ≥ σ_t** → ΔD ≥ 1 → **1타 파쇄** (Regime I)
- **σ_I ≲ σ_t** → ΔD < 1 누적 → **N_f 타에 파쇄** (Regime II)
- **σ_I ≪ σ_t** → ΔD ≈ 0 → **안 깨짐** (Regime III)

지수 하나(p)로 세 Regime 이 연속적으로 갈린다. **손상규칙 계수 = p**(문턱하 민감도;
subcritical crack growth·Weibull 유래)를 실측 F_peak–N_f 곡선에 맞춰 조정(예: 60 N 예측
5 → 실측 3 에 수렴). Weibull m 은 파쇄 *확률* 산포로 반영.

> **Weibull 파괴확률은 스칼라가 아니라 σ_I *장(field)* 의 부피적분이다** (약한 고리):
> `P_f = 1 − exp[−∫_V (σ_I(x)/σ_0)^m · dV/V_0]` (σ_I>0 인장부만). FEM 에선 **tet 별
> σ_I·부피 합산**(risk of rupture). peak σ_I 하나만 쓰면 유효부피 `V_eff/V_0` 인자를
> 빠뜨린 근사다. σ_0 = 특성강도(63.2% 파괴 응력). 수식 PNG: [`Notation.py`](Notation.py)
> (`weibull_pf`·`weibull_fem_sum`·`weibull_veff`).
- **누적일 `W = Σ∫F·v dt`** (토크×각속도 or 반력×변위) — **실측 가능**. 에너지형 손상
  `D = W/W*` 와 직결(§7-4 W\*). ⇒ 트윈은 σ_I→D 로 *예측*, 실기는 W 로 *관측*해 맞춤.
- **강성 저하**: 손상↑ → 정제 물러짐 → stroke 당 슬라이더 관통량↑ / 반력 기울기↓.
- **AE(음향방출)**: 미세균열 이벤트 카운트.
- **FSM "stall 없이 통과"**(Crusher.md §12-4) = D 가 1 도달한 순간 = **N_f 온라인 검출**.

**STEP 5 — 검증 & 사용**: 보정에 안 쓴 F_peak(예: 75 N)로 N_f 예측 vs 실측 검증.
실기 FSM은 "stall 없이 통과"로 N_f를 온라인 카운트(Crusher.md §12-4) → A열 실측
피드백. 검증되면 **실험 없이** 신규 형상(CV/AR 스윕)·타격 스케줄의
N_f·필요토크·균열패턴을 예측한다.

**한 장 요약**

| 역할 | 항목 |
|---|---|
| 입력(측정) | σ_t=2.2 MPa, Weibull m≈8, ρ·porosity, (**E**: UTM 규칙시편 있으면 여기로) |
| 조정 항 | **손상규칙 계수**(F_peak–N_f로) — 순수 미지항은 사실상 이것뿐. **E**는 통짜 정제일 때만 F–δ back-fit(≈6 GPa). ~~G_c~~→W\* |
| 표적(맞춤) | F(δ), Brazilian σ_t, F_peak–N_f, 균열위치 |
| 출력(예측) | 신규 조건 N_f · 필요토크 · 균열패턴 |

> 한 줄: **손상계수는 "돌려 맞추는 항", σ_t는 "재서 넣는 입력", N_f는 "예측·검증
> 표적"** — E는 장비가 좋을수록 조정 항→입력으로 격상된다. 세 역할이 한 알약을 관통한다.
> 관련: [`fem_stress_field.py`](../../Crusher_Genesis/FEM/fem_stress_field.py), DataInventory.md §E.

**저자원 경로 (경도계 + Crusher 만 있을 때)** — UTM·별도 Brazilian 없이 대부분 커버된다:
- **경도계 = 직경압축 = Brazilian.** 파괴하중 `F_break` 를 재고 → **Pitt 식으로 σ_t 도출
  (E 불필요, 순수 기하 공식)**. 전용 UTM Brazilian 을 따로 할 필요 없다.
- **같은 경도계로 30개 반복** → 파괴하중 분포 → **Weibull m** (취성 강도 산포).
- **Crusher F_peak–N_f** → 손상규칙 계수 fit(STEP 4). F_peak–N_f 자체가 Crusher 로 얻는 값.
- ⇒ **경도계 하나로 σ_t · (F_threshold ≈ 경도) · Weibull m 을 얻고, 순수 조정 항은 손상계수
  하나로 줄어든다.**
- **E 는 언제 필요?** *단일 파쇄력엔 불필요* (F_threshold ≈ 경도). E 는 **균열 *위치* 와
  N_f(반복 손상누적)** 를 풀 때만 추가로 든다 → 없으면 문헌값(MCC ~4–9 GPa)으로 고정.
- **잃는 것**: σ_I 절대값이 문헌 E 의존 → 균열 위치·F_threshold 정밀도↓. 하지만 N_f 경로
  (Regime II)는 fit 이 스케일 오차를 흡수해 **성립**한다.
- 한 박사님 피드백 "Data-driven 하게라도 풀 수 있어야"(§9)에 정확히 대응.

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
