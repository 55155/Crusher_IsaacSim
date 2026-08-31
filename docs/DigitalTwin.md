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
9. [Solver / Coupler 조합 실험 기록](#9-solver--coupler-조합-실험-기록)
10. [피드백](#10-피드백)
11. [Visualization options](#11-visualization-options)
12. [현 시점 우선순위 및 미해결 문제 (2026-08-08)](#12-현-시점-우선순위-및-미해결-문제-2026-08-08)
13. [공정 검증 전략 — 전(全) Rigid 선행 모드](#13-공정-검증-전략--전全-rigid-선행-모드)

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

**[정정] env는 conda `isaacsim`이 아니라 standalone Python (2026-07-15 확인)**:
위 "env: `isaacsim`" 표기는 부정확했다. 실제로 Genesis(v1.2.1, editable,
`C:\Users\user\Desktop\Genesis`)+pyuipc+torch+warp-lang 스택이 설치된
인터프리터는 `C:\Users\user\AppData\Local\Programs\Python\Python311\python.exe`
(standalone Python 3.11.9, conda 아님)이다. conda env 3개(`TabletCrusher`,
`isaac_sim`, `mujoco_env`)에는 genesis-world가 전혀 없다 — `isaac_sim`
conda env는 이름이 비슷해 헷갈리지만 실제로는 NVIDIA Isaac Sim 5.0.0
(별개 제품) 전용이다. Genesis/IPC 스크립트 실행 시
`C:\Users\user\AppData\Local\Programs\Python\Python311\python.exe <script>`
를 직접 호출할 것(conda activate 불필요/무의미).

**[재정정] 현재 인터프리터는 conda env `crusher_genesis` (2026-08-03 확인)**:
위 두 경로(`C:\Users\user\...`, `C:\Users\simuser\...`) 모두 지금은 존재하지
않는다. Genesis 1.3.1 + quadrants 1.2.0 스택이 실제로 설치된 인터프리터는

```
C:\Users\simuser\miniconda3\envs\crusher_genesis\python.exe
```

이다. 실행 로그(`_run_reposition_check.log` 등)의 site-packages 경로로 확인.

### 5-2. 실링부 마찰 파지 — *해결됨* (2026-07-13)

M0609+RG2 로 봉투 옆면 실링부(~1cm)를 순수 접촉+마찰로 파지(weld 없이).

- **PBD.Cloth + 핑거 mesh 접촉 → 즉시 폭발.** 5-1의 우려가 실제로 발생(자기충돌
  제약과 손가락 침투 해석이 충돌). → **IPC 커플러**로 전환(`gs.options.IPCCouplerOptions`).
  단, IPC는 PBD를 못 보므로 봉투 재질을 **`gs.materials.FEM.Cloth`** 로 교체.
- 실측 STL(옆면·바닥이 두께 0으로 핀치된 실링선 포함)을 그대로 넣으면 IPC
  self-intersection sanity check에서 계속 실패. 클리핑/데시메이션 다 시도해도
  실패 지속. **진짜 원인은 위상이 아니라 로그를 올려야 보이는 네이티브 에러**:
  `Geometry(0) in cloth_0_0 is too close (distance=1.2mm, thickness=2mm) to
  Geometry(0) in cloth_0_0` — front/back 패널 간격이 material thickness 대비
  너무 얇아서 생긴 **자기충돌**. 이미 검증된 procedural 5-panel 프록시(간격 6mm)
  로 교체하니 즉시 해결.
- 그리퍼는 6DOF mimic(m0609_rg2_v2.xml) 대신 **단순 2DOF(m0609_rg2.xml)** +
  `set_dofs_position` 텔레포트 사용(IPC의 `two_way_soft_constraint` 가 반력을
  자체 처리, PD 불필요). 결과: `M0609_RG2/grasp_bag_ipc_test.py`, 봉투 Δz +125.8mm.

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

### 6-2. SDF vs fan tetrahedralization — 둘은 다른 층위의 방법 (2026-07-14)

§6-1에서 언급한 **SDF(Signed Distance Function)**와, 캡슐 정제를 위해 실제로
구현한 **fan tetrahedralization**(`utills/primitive_tablet_generator.py`)은
이름이 둘 다 "primitive를 수식으로 정의"라서 헷갈리기 쉽지만 **풀고 있는
문제 자체가 다르다.**

**SDF — Rigid(강체) 충돌 전용, 메쉬가 아예 없음.**
- 형상을 `f(x) = (점 x 에서 표면까지의 부호 있는 거리)` 라는 **암시적(implicit)
  함수**로 정의한다. 예: 캡슐 SDF = "선분까지의 거리 − 반지름".
- 정점(vertex)도 삼각형(face)도 없다 — 어떤 점이 형상 안/밖/표면에 있는지,
  법선이 뭔지를 **그 자리에서 수식으로 계산**한다(BVH 순회·삼각형 테스트 불필요).
- **강체 전용인 이유**: 형상이 절대 변형되지 않는다는 전제가 있어야 "표면까지
  거리"라는 하나의 고정된 수식이 성립한다. 정제(FEM)처럼 매 스텝 표면이
  변형장에 따라 움직이는 대상은 애초에 SDF로 표현할 수 없다(§6-1의 결론).
  Genesis rigid solver는 캡슐을 이미 SDF primitive로 취급하므로 이 문제와는
  무관하다.

**Fan tetrahedralization — FEM(변형체) 전용, 실제 메쉬(정점+사면체)를 만듦.**
- FEM은 물질점(material point)이 있어야 변형을 적분할 수 있으므로, SDF 같은
  암시적 표현이 아니라 **명시적 정점 + 사면체(tet) 요소 목록**이 반드시 필요하다.
  일반적으로 이걸 만드는 도구가 TetGen(Delaunay 기반 사면체화 라이브러리)이다.
- **fan tetrahedralization은 TetGen을 안 쓰고 이 정점+사면체 목록을 직접
  계산하는 방법**이다: ①형상의 표면을 파라메트릭 방정식으로 직접 생성(캡슐이면
  원기둥+반구 방정식) → ②**볼록(convex) 도형**이면 내부의 아무 점(보통
  centroid)에서 모든 표면 삼각형으로 부채꼴을 이으면 `tet(a, b, c, centroid)`
  가 항상 유효(퇴화 없음)하다는 사실을 이용 → 표면 삼각형 N개 → 사면체 N개,
  전부 같은 apex(centroid) 하나를 공유. TetGen의 Delaunay 리파인먼트·Steiner
  point 삽입이 전혀 없는, 100% 폐형식(closed-form) 계산이다.
- **한계**: 모든 tet가 같은 apex를 공유하는 구조라, 표면이 급격히 휘는
  영역(캡슐의 극(pole))에 삼각형이 몰리면 그 근처 tet들이 아주 얇고
  뾰족해진다(sliver tet, 종횡비 나쁨) — Delaunay 리파인먼트가 없어서 TetGen처럼
  이런 tet를 자동으로 개선해주지 않는다. §8 캡슐 얼어붙음 조사에서 이게 바로
  캡슐만 불안정하고 Box(sliver 없음)는 안정적인 이유로 좁혀졌다. 또한 오목
  (non-convex) 형상에는 이 방법이 그대로 적용 불가(부채꼴이 형상 밖으로
  나가 tet가 뒤집힘) — 그런 경우는 형상을 볼록 조각으로 나누거나 TetGen처럼
  더 일반적인 사면체화가 필요하다.

**요약**: SDF는 "형상을 수식으로 정의"하지만 메쉬가 없는 강체 접촉 기법이고,
fan tetrahedralization은 "형상을 수식으로 정의"해서 **TetGen을 대체할 메쉬
(정점+사면체)를 만드는** FEM 전용 기법이다. 이름의 "수식적 정의"라는 표현이
같아서 헷갈렸지만 서로 대체재가 아니라 각각 다른 문제(강체 충돌 vs 변형체
메쉬 생성)를 푼다.

#### `gs.morphs.Box`/`Sphere` 는 FEM 에서 primitive 가 아니다 — 실측 재확인 (2026-08-03)

"정제를 Genesis 의 Box 엔티티처럼 primitive 로 만들면 되지 않나"는 질문이
반복해서 나오는데(파일 이름이 `primitive_tablet_generator.py` 라 더 헷갈린다),
**FEM 경로에서는 아니다.** Genesis 1.3.1 에서 직접 측정:

| morph | 입력 형상 | 실제 FEM 엔티티 |
| --- | --- | --- |
| `gs.morphs.Box(size=4.8mm)` | 정점 8 / 면 12 | **정점 9 / tet 12** |
| `gs.morphs.Sphere(r=2mm)` | — | **정점 966 / tet 3791** |
| 우리 해석적 캡슐(§6-2) | — | 정점 137 / tet 288 |

둘 다 TetGen 산출물이다. **1.3.1 에서는 이게 더 분명해졌다** — `genesis/utils/
element.py` 에 `mesh_to_elements` **하나만** 남았고(예전의 `box_to_elements`/
`sphere_to_elements` 는 아예 없어짐), `FEMEntity.sample()` 이 모든 morph 를
동일하게 처리한다:

```
meshes = gs.Mesh.from_morph_surface(morph, surface)   # 형상 → 표면 메시
surface_trimesh = trimesh.Trimesh(...)
verts, elems = eu.mesh_to_elements(surface_trimesh, tet_cfg)   # → TetGen
```

즉 `gs.morphs.Box` 는 "박스 표면 메시를 만들어주는 편의 함수"이지 해석적
primitive 가 아니다. `primitive_tablet_generator.py` 의 "primitive" 도
**"primitive *형상*"(캡슐·박스)** 이라는 뜻이며, 그 파일이 존재하는 이유
자체가 "Box/Sphere 가 FEM 에선 primitive 가 아니더라"는 발견이다.

**Box 를 쓰면 오히려 나쁘다.** 위의 `정점 9 / tet 12` 가 §조합10 에서 파우더
담기가 무반응이었던 원인이다 — 박스 모서리 8개 + 내부점 1개뿐이라 판 중앙처럼
정점이 없는 넓은 영역은 커플링 보정이 전혀 안 된다.

**진짜 primitive 경로는 강체 쪽에만 있다**: MJCF `type="capsule"` geom
(= SDF, §조합6) — `full_workflow_rigid.py` 가 쓰는 방식이다. 정제는 결국
용도에 따라 두 가지 표현이 공존한다:

| | 파일 | 방식 | 진짜 primitive? |
| --- | --- | --- | --- |
| FEM 정제 | `full_workflow.py` | medial-axis 사면체화 → 몽키패치 주입 | ✗ (메시 — FEM 은 원리상 불가) |
| Rigid 정제 | `full_workflow_rigid.py` | MJCF `type="capsule"` | ✓ SDF primitive |

부수 효과: 1.3.1 에서 `mesh_to_elements` 가 **모든 FEM morph 의 단일 관문**이
되면서, §13-9 의 몽키패치가 예전보다 오히려 견고해졌다 — 형상 종류에 따라
다른 함수로 새는 경로가 없다.

### 6-3. Crusher와의 상호작용 — 실측 반력 프로파일 구동 FEM (2026-07-04)

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
- **§6-3의 real↔sim 비교(4번)는 생산이 아니라 검증이었다.** real·sim이 둘 다 있는 유일한
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
- 한 박사님 피드백 "Data-driven 하게라도 풀 수 있어야"(§10)에 정확히 대응.

---

## 8. 발생할 수 있는 문제 사항

- 실제 운용 속도와 맞을지는 의문.
- 파지할 때 마찰과 파지력으로 집는 것이 아님 → **weld**.
- 샘플백의 파라미터를 튜닝하는 일이 필요.
- **[미해결] implicit FEM 솔버가 정지 상태(v=0)에서 중력을 무시하고 얼어붙음**
  (`FEM/fem_tablet_drop.py`, 2026-07-13 격리 확인). 초기 속도 킥으로 우회
  시도했지만 형상에 따라 필요한 킥 크기가 다르고, 통하는 경우도 비물리적
  거동(찌그러졌다가 중력 반대로 상승)을 보임. Rigid 엔티티가 씬에 있으면
  킥을 줘도 다시 얼어붙음 — 원인 미상, Genesis 쪽 확인 필요.
- **[해결] FEM.Elastic + M0609(IPC) 조합 "낙하 얼어붙음" — 원인 확정 (2026-07-14)**
  (`M0609_RG2/grasp_bag_tablet_ipc_test.py`, `utills/primitive_tablet_generator.py`).
  `fem_options`에 `use_implicit_solver=True`를 안 주면(기본값) 1차 문제는
  해결되나, M0609가 씬에 있으면 FEM.Elastic이 다시 거의 얼어붙는 2차 문제가
  있었다. 처음엔 "정점/tet 개수 임계치" 가설을 세웠으나(`gs.morphs.Box`는
  정상, `Sphere`/정제 STL은 얼어붙음), **박스 적층(5-layer→3-layer, tet 수를
  오히려 줄임)으로 반증됨** — 3-layer가 5-layer보다 더 심하게 얼어붙어 단순
  개수 비례가 아님을 확인.
  **진짜 원인**: 정점/tet 개수가 아니라 **TetGen을 거쳤는지 여부 자체**였다.
  `genesis/utils/element.py`의 `box_to_elements`/`sphere_to_elements`/
  `mesh_to_elements` 는 전부 `mu.tetrahedralize_mesh`(TetGen)를 호출하는데,
  `gs.morphs.Box`만 우연히 안전한 게 아니라 **TetGen이 Box처럼 아주 단순한
  볼록 입력에 한해 "질 좋은/안전한" tet를 내놓고, 조금만 복잡해지면(Sphere,
  실제 메시) M0609+IPC 커플링과 상호작용하는 무언가 다른 내부 상태를 만드는
  것**으로 좁혀졌다.
  **검증**: `utills/primitive_tablet_generator.py` 로 캡슐을 **TetGen을 전혀
  거치지 않고** 순수 수학적으로 사면체화했다 — 캡슐 표면을 파라메트릭
  방정식으로 직접 생성하고, 볼록 도형의 정석 기법인 **centroid fan
  tetrahedralization**(모든 표면 삼각형을 중심점과 이어 사면체 하나씩,
  `tet(a,b,c,centroid)`)으로 verts/elems를 직접 계산한 뒤,
  `genesis.utils.element.mesh_to_elements`를 몽키패치해 TetGen 경로를
  건너뛰고 Genesis에 직접 주입했다(Genesis FEM solver는 `det([a-d,b-d,c-d])
  < 0` 방향성 요구 — 위반 시 `RuntimeError: tet_wrong_order`, 삼각형별로
  부호 확인 후 필요시 정점 swap으로 해결). **이 완전 비-TetGen 캡슐(정점
  83개, tet 160개)은 M0609 앞에서 얼어붙지 않고 정상 낙하 →
  파지→리프트까지 전 구간 성공**(tablet Δz=+123.2mm, Box 근사(+117.6mm)와
  거의 동일) — **TetGen 산출물 자체가 원인이라는 가설이 최종 확인됨**.
  실용적 해결책: Box primitive 근사, 또는 정확한 형상이 필요하면
  `primitive_tablet_generator.py`의 analytic fan-tetrahedralization 사용
  (단, 볼록 도형에만 적용 가능 — 오목 형상은 centroid fan이 무효가 되므로
  별도 분해 필요).

- **[정정 + 신규 발견] "TetGen 산출물이 원인"은 M0609 케이스에 한정 —
  FEM.Elastic(체적) + FEM.Cloth(표면 전용) 조합은 별개의 얼어붙음 버그
  (2026-07-14, `FEM/fem_tablet_drop.py`)**: 위 결론(TetGen 산출물 자체가
  원인)은 M0609 로봇이 있는 씬에서만 검증됐다. 정제를 실제 샘플백
  (FEM.Cloth)에 낙하시키는 씬에서 재테스트한 결과, **TetGen 을 전혀 안 쓴
  analytic 캡슐도 FEM.Cloth 봉투가 씬에 있으면 접촉 여부/거리(멀리 떨어뜨려도
  동일)/속도 킥과 무관하게 거의 완전히 멈췄다** — 심지어 봉투를 Rigid 메시로
  바꿔도 재현됨. 반면 **`gs.morphs.Box` primitive 는 이 조합에서도 예외적으로
  안정적**(격리 테스트: t=50ms 에 120→106.5mm, 이론 자유낙하와 근접). 즉
  "TetGen 산출물"과 "Box 냐 아니냐"는 서로 다른 두 얼어붙음 버그 각각에서
  독립적으로 확인된 회피 조건이며, 근본 원인(체적 FEM 대 표면 전용 FEM
  결합 시 IPC 솔버 내부에서 무엇이 문제인지)은 아직 미상이다 — Box
  primitive 만이 현재까지 발견된 두 버그 모두에서 안정적인 유일한 형상.
  추가로 확인된 특이사항: 봉투 입구 통과 여부가 knife-edge 조건이라
  `contact_d_hat=1mm` 에서는 동일 설정으로도 실행마다(GPU 부동소수점
  비결합성 추정) 통과/걸림이 갈렸고, `contact_d_hat=0.5mm` 로 낮추자
  안정적으로 통과함(실측 2/2).

- **[변인통제 실험] 캡슐 얼어붙음의 원인은 "우리 수식 정의 방식"이 아니라
  "캡슐이라는 형상/토폴로지" — Box 대조군으로 확정(2026-07-14)**: 위 발견에서
  "analytic 캡슐도 FEM.Cloth 봉투 앞에서 얼어붙는다"까지만 알았고, 이게
  ①우리가 만든 TetGen 우회 방식(표면 직접 계산 + centroid fan
  tetrahedralization + `mesh_to_elements` 몽키패치 주입) 자체의 결함인지,
  아니면 ②캡슐이라는 형상 고유의 문제인지가 불분명했다. 변인통제:
  **`Samplebag(FEM.Cloth) + 우리 방식으로 만든 analytic Box(FEM.Elastic,
  정점 9개/tet 12개, `primitive_tablet_generator.make_box_tets`)`** 를
  `Samplebag + gs.morphs.Box`(대조군, TetGen 경유하지만 이미 안정성 확인됨)
  와 비교 — **거의 완전히 일치하는 정상 자유낙하**(t=50/100/200ms 에서
  106.55/68.62/-81.10mm vs 대조군 106.52/68.56/-81.70mm). 즉 **우리의 수식적
  형상 정의 방식 자체는 정상**이고, 캡슐만 얼어붙는 원인은 ②쪽 — 구체적으로는
  **fan tetrahedralization이 극(pole) 근처에서 만드는 다수의 얇은 sliver
  tet**(하나의 centroid 정점을 공유하는 부채꼴 구조상, 표면이 급하게 휘는
  영역에 삼각형이 많이 몰리면 종횡비 나쁜 tet가 다수 생김 — 해상도를 올렸을
  때 실제 좌굴 변형으로 관찰된 것과 동일 메커니즘)로 좁혀졌다. Box(12개,
  종횡비 좋음)는 문제없고 캡슐(160개, 극 근처 sliver 다수)만 얼어붙는 것과
  일치. **SDF(§6-1 참고)와는 무관** — SDF는 애초에 FEM(변형체)에는 적용 불가한
  Rigid 전용 접촉 표현이라, 이번 fan tetrahedralization 접근과는 완전히
  다른 층위의 방법이다(아래 §6-3 참고).

  **참고**: Box와 캡슐은 **둘 다 볼록(convex)** 이라 fan tetrahedralization
  자체(사면체 유효성)는 둘 다 성립한다 — 문제는 볼록성이 아니라 "전역
  centroid 하나로부터의 거리가 표면 전체에서 균일한가"였다. Box는 8개
  꼭짓점이 중심에서 대략 비슷한 거리(반대각선 절반)에 있어 12개 tet의
  종횡비가 고르지만, 캡슐은 극(pole) 쪽 작은 삼각형까지 전부 반지름보다
  훨씬 먼 전역 centroid(예: 반지름 2.5mm인데 극-centroid 거리 6mm)로
  이었던 게 문제였다 — 형상의 "볼록/오목"이 아니라 "부채꼴에 쓰는 apex와
  표면 사이 거리의 균일성"이 sliver 발생 여부를 결정한다.

- **[해결] sliver 없는 캡슐 사면체화 — medial-axis 다중 앵커 방식
  (2026-07-14, `utills/primitive_tablet_generator.make_capsule_tets_v2`)**:
  위 sliver 원인 분석을 그대로 뒤집어 해결책으로 썼다. 캡슐의 수학적 정의
  자체가 "중심 선분(medial axis)까지 거리 ≤ 반지름인 점의 집합"이므로(§6-2
  SDF 설명과 동일한 정의), 전역 centroid 하나 대신 **표면의 각 부분을 그
  지점에서 가장 가까운 축 위의 점에 부채꼴로 잇는다**:
  - 북/남 반구(뚜껑)는 정의상 그 반구 자신의 중심점에서 항상 정확히
    반지름 R 만큼 떨어져 있으므로, 반구 표면 전체(pole 포함)를 그 반구
    중심 하나로 부채꼴 — apex-표면 거리가 어디서나 정확히 R로 균일해짐.
  - 원기둥 몸통은 `n_cyl_bands` 개 층으로 나누고, 각 층 경계를 표준
    "삼각기둥→3-사면체" 분해(널리 쓰이는 프리즘 메싱 기법)로 처리 — 인접
    층/반구가 같은 앵커점을 공유해 내부 경계면이 정확히 상쇄되어(watertight)
    유지된다(직접 검증: 축퇴(0에 가까운 부피) tet 0개, 전체 부피가 이론값에
    근접).
  - **결과(실측)**: 최대 edge-길이비(sliver 정도 지표)가 **15.0 → 6.25로
    개선**(같은 n_theta/n_cap_rings). 바닥 낙하 테스트에서 착지 후 좌굴/찌그러짐
    없음(v1은 해상도를 올리면 좌굴 발생했었음). **더 중요하게, M0609+FEM.Elastic
    얼어붙음과 FEM.Cloth+FEM.Elastic 얼어붙음 둘 다에서 더 이상 얼어붙지
    않고 Box와 거의 동일한 정상 자유낙하를 보임**(Cloth+캡슐 v2:
    t=50/100/150/200ms 에 107.5/70.7/9.5/-76.1mm — Box 대조군 106.5/68.6/6.3/
    -81.1mm 과 거의 일치). 봉투 통과 데모(§Tablet)에서도 실제로 입구를
    통과해 내부까지 안정적으로 낙하함을 확인(contact_d_hat=0.5mm).
  - **함의**: 이번 결과로 앞서 별개로 보였던 두 얼어붙음 버그(M0609+TetGen
    산출 메시, FEM.Cloth+FEM.Elastic)가 사실 **같은 근본 원인(FEM 메시의
    tet 종횡비/조건수 불량, 즉 sliver tet)에서 비롯된 동일 계열의 문제였을
    가능성이 높다** — "TetGen이냐 아니냐", "Cloth가 있냐 없냐" 는 진짜
    원인이 아니라, 그 조건에서 우연히 sliver가 있었는지 없었는지의 대리
    지표(proxy)였던 것으로 보인다. Box가 두 버그 모두에서 예외적으로
    안정적이었던 이유도 이걸로 설명된다(sliver가 없는 형상이라서).

- **[해결(가설)] Rigid(정제, MJCF capsule geom=SDF) + FEM.Cloth(봉투) + IPC
  조합 접촉 시 발산 — `coup_type` 자동선택 문제로 특정(2026-07-14/15)**
  (`FEM/rigid_capsule_tablet_bag_ipc_test.py`): §6-1에서 제안한 "정제를
  sliver 없는 tet mesh 대신 Rigid MJCF capsule(네이티브 SDF primitive)로
  넣으면 어떨까"를 실제로 시도한 첫 버전은 접촉 순간 위치가 수 미터로
  튕겨나가는 발산을 보였다(`constraint_strength` 100→0.5로 낮추면 발산은
  막히지만 감쇠 없는 바운스가 지속 — 원인 미상으로 보류됨, git 3fca8c8).
  **재조사 결과**: `genesis/engine/couplers/ipc_coupler/coupler.py`
  `_setup_coupling_config`(L206-215)의 `coup_type` 자동선택 규칙은
  `entity.n_joints > 0`(관절 있음)이면 `two_way_soft_constraint`(Genesis
  rigid solver의 PD/제어 결과를 IPC가 soft constraint로 따라가게 하는
  방식 — **제어 대상(로봇 팔 등)을 위한 경로**), `n_joints == 0`(무관절
  단일 바디)이면 `ipc_only`(IPC가 중력·동역학을 전부 담당하는 one-way
  경로 — Plane/Shelf가 쓰는 경로)를 쓴다. 이전 시도는 캡슐을 MJCF
  `<freejoint/>` 바디로 만들었으므로 `n_joints=1`→자동으로
  `two_way_soft_constraint`를 탔을 것으로 추정된다. 이 정제는 PD로
  구동되는 제어 대상이 아니라 **순수 낙하하는 수동체**이므로 의미상
  맞지 않는 경로였다는 가설.
  **검증**: 동일한 MJCF capsule(freejoint 유지, `n_joints`는 그대로
  1)에 `material=gs.materials.Rigid(coup_type="ipc_only", ...)`로
  자동선택을 **명시적으로 override** — Plane/Shelf에 이미 검증된 경로를
  강제 적용. 결과: **발산 없이(300 스텝 끝까지 `|pos|<1m` 유지) 정제가
  봉투 입구(105mm)에서 낙하해 봉투 바닥 근처(≈15.9mm)까지 자연스럽게
  안착**(`net_fall=101.28mm`, xy는 봉투 중심 근방 유지) — 영상으로도
  좌굴/폭발 없이 봉투 안에 담기는 모습 확인
  (`rigid_capsule_tablet_bag_ipc_ipc_only_20260714_202712.mp4`).
  **주의**: 이건 "재현 1회 성공"이며 근본 원인이 100% 확정된 건 아니다
  (`two_way_soft_constraint` 자체가 원리적으로 왜 무제어 낙하체에서
  발산하는지는 아직 코드 레벨로 추적 안 됨 — 후속 조사 항목으로 유지).
  다만 실용적으로는 **무관절/무제어 Rigid 낙하체는 `coup_type="ipc_only"`
  를 항상 명시하라**는 지침으로 채택할 만하다.

- **[성공 + 신규 한계 발견] FEM(정제, sliver-free v2 캡슐) + FEM.Cloth(봉투)
  강성 상향 스윕 — 200x(E=1e7)까지 안정, 2000x(E=1e8)에서 새로운 "중력
  무시" 현상 재현(2026-07-15, `FEM/fem_tablet_drop_stiff.py`)**: 위 Rigid
  캡슐 경로는 정제를 강체로 근사해 압축·변형 거동을 볼 수 없다는 한계가
  있어, **FEM(정제)+FEM(봉투)** 조합을 유지한 채 fem_tablet_drop.py의
  "안정성 우선으로 낮춘 E=5e4"보다 강성을 올려 재도전했다.
  **dt 스케일링**: explicit FEM의 안정 dt는 파동속도 c=√(E/ρ)에 반비례하므로
  `dt_new = dt_base·√(E_base/E_new)`로 스케일링(기준: E=5e4, dt=5ms → 검증됨)
  하고, 물리 시간(1.5s)이 유지되도록 스텝 수도 같이 늘렸다.
  - **E=1e6(20x, dt=1.12ms, 1342 스텝)**: 발산 없음. 정제가 봉투 바닥
    근처(z≈19mm)까지 정상 낙하·안착. 형상 유지 양호(z_span 4.438→4.509mm,
    거의 불변 — 무른 재질(E=5e4)보다 훨씬 "딱딱한 정제"다운 거동).
  - **E=1e7(200x, dt=0.354ms, 4243 스텝)**: 마찬가지로 발산 없음, 더 나은
    형상 유지(z_span 4.639→4.675mm). 영상으로 봉투 안 안착 확인
    (`fem_tablet_bag_drop_stiff_E1e+07_*.mp4`).
  - **E=1e8(2000x, dt=0.112ms, 13416 스텝)**: **정제가 사실상 전혀
    낙하하지 않음**(1.5초 시뮬 동안 net_fall=0.007mm, 봉투 입구조차 통과
    못함) — §8 위쪽에 이미 기록된 "[미해결] implicit FEM 솔버가 정지
    상태에서 중력을 무시하고 얼어붙음" 버그와 증상은 동일하지만, 이번엔
    `fem_options`를 전혀 건드리지 않은 **기본(explicit) 솔버**에서
    재현됐다는 점이 다르다 — 즉 원인이 "implicit 솔버 특정"이 아니라
    더 일반적일 가능성. **가설**: 이 정제(mm 스케일, 부피 ~수 mm³)의
    중력(F=mg)은 애초에 극히 작은 값(~1e-7N 수준)인데, E=1e8처럼 강성이
    극단적으로 크면 부동소수점 잔차 수준의 미세한 변형만으로도 이를
    상쇄하는 내부 탄성력이 발생해, 매 스텝 중력이 수치적으로 묻혀버리는
    "강성-질량 불균형" 수치 문제로 추정(코드 레벨 확정은 안 됨, 후속
    조사 필요).
  - **결론**: 이 dt 스케일링 방식으로는 **200x(E=1e7)가 실용적 상한**으로
    보인다 — 문헌값 실제 정제(E~2GPa, 40000x)는 이 스케일링을 적용하면
    dt가 25μs까지 줄어 스텝 수가 비현실적으로 커질뿐더러, 그 전에
    1e8 근방에서 이미 별개의 수치 버그를 만난다(§grasp_bag_tablet_ipc_test.py
    에서도 실측 E=2GPa가 explicit·dt=5e-3 조합에서 발산한다고 별도 보고됨 —
    같은 결론으로 수렴).

- **[해결] 정제 형상이 접촉 전부터 즉시 우그러지는 현상 — 원인은
  `contact_d_hat`이 캡슐 자신의 극(pole) 근처 정점 간격보다 커서 생긴
  self-contact 오탐(2026-07-15, `FEM/fem_tablet_solo_diag.py`)**:
  위 강성 스윕 영상들(E=1e6/1e7)에서 정제가 "우그러진(찌그러진)" 채로
  낙하하는 것처럼 보인다는 사용자 지적으로 시작된 조사.
  **1단계 — 정점 덤프로 실제 형상 확인**: `fem_tablet_drop_stiff.py`에
  `DUMP_VERTS=1` 옵션을 추가해 137개 정점 좌표를 여러 시점에 저장,
  matplotlib으로 XZ/XY 단면을 그려보니 — **XY(원형 단면)는 시종일관
  완벽하게 유지**되고 **Z축(장축)만 t=0 직후 즉시 설계값 5.0mm →
  4.64mm(−7.2%)로 압축된 뒤 이후 1.5초 내내 소수점 3자리까지 완전히
  고정**됨을 확인. 봉투 접촉은 자유낙하 55ms 이후에나 가능한데 이미
  t<1ms에 압축이 끝나 있어 접촉이 원인일 수 없었다.
  **2단계 — 봉투/선반 없이 정제 단독 낙하로 접촉 가능성 배제**
  (`fem_tablet_solo_diag.py`, Plane을 1m 아래 멀리 둬서 200스텝 내 접촉
  불가능하게 설정): 그래도 **k=0(첫 스텝, t=0.35ms)에 동일한 압축이
  즉시 발생**(bbox z: 5.0000→4.6406mm) — 완전히 고립된 상태에서도
  재현되어 "봉투와 무관한 정제 자체/솔버 문제"로 확정.
  **3단계 — 원인 특정**: `make_capsule_tets_v2`가 만드는 137개 정점의
  최근접 이웃 거리를 `scipy.spatial.cKDTree`로 전수 계산한 결과, **극
  바로 아래 링(z=±2.402mm)의 인접 정점 간격이 0.32mm** — 당시 쓰던
  `contact_d_hat=5e-4(0.5mm)`보다 작았다. IPC의 self-contact(자기충돌
  방지) 배리어가 이 근접한(그러나 실제로는 메시 표면상 정상적으로
  가까운) 정점 쌍을 "충돌"로 오인해 첫 스텝부터 극을 안쪽으로 밀어붙인
  것 — 캡슐이 작을수록(mm 스케일) 극 근처 정점 간격도 비례해서 작아지므로
  **d_hat을 조정하지 않으면 작은 형상일수록 이 오탐에 더 취약해진다**는
  점에서 사용자의 "스케일이 너무 작아서인가" 라는 직관이 정확히 들어맞았다.
  **검증**: solo 진단에서 `contact_d_hat=5e-5(0.05mm, 최소 정점간격
  0.32mm보다 확실히 작게)`로 낮추자 200스텝 내내 bbox가 정확히
  4.0000×5.0000mm(설계값)로 고정 — 압축 완전히 사라짐, 자유낙하 물리량
  (com_z 낙하량)도 이론값과 일치. 봉투 포함 시나리오에도
  `contact_d_hat=1e-4(0.1mm)`로 적용해 재실행하니 자유낙하 구간에서는
  설계 형상을 그대로 유지하다가, **봉투 입구 접촉 시점(t≈450ms)부터
  비로소 x≠y인 진짜 비대칭 동적 변형이 나타나기 시작**(이전엔 접촉과
  무관하게 이미 축대칭으로 고정돼 있어 "가짜 형상"이었던 것과 대조적).
  시뮬레이션 시간을 3초로 늘려 완전히 안착할 때까지 실행 확인
  (`fem_tablet_bag_drop_stiff_v2_E1e+07_20260715_005954.mp4`).
  **교훈**: `contact_d_hat`은 오브젝트 간 최소 간극뿐 아니라 **각
  변형체 메시 자신의 최소 정점 간격(특히 곡률이 큰 극/모서리 부근)보다도
  작아야 한다** — 이전 fem_tablet_drop.py의 `d_hat=5e-4`는 "봉투 입구
  통과의 knife-edge 문제"를 기준으로 정해진 값이라 정제 자체의 self-contact
  안전 마진은 검토된 적이 없었다. 앞으로 캡슐/구 등 곡률 큰 FEM 형상을
  쓸 때는 `make_capsule_tets_v2`류 생성자 출력에 대해 최근접 이웃 거리를
  먼저 계산해 `contact_d_hat`이 그보다 충분히(예: 1/3 이하) 작은지 확인하는
  걸 표준 절차로 삼는다.

---

## 9. Solver / Coupler 조합 실험 기록

각 구성요소(Robotarm/Tablet/Samplebag)를 어떤 표현 방식(Rigid/FEM/PBD 등)으로
넣고 어떤 coupler로 묶었는지의 조합별로 무엇을 시도했고 어떻게 풀었는지(또는
아직 못 풀었는지) 정리한다. 상세 조사 과정은 §5~8에 이미 기록돼 있으므로
여기서는 **조합 단위로 인덱싱**하고 핵심만 요약, 자세한 내용은 원 절을 참고.

> **현재(2026-07) 사실상 IPC 커플러만 쓰고 있다** — PBD는 자기충돌 제약과
> 침투 해석이 충돌해 폭발했고(조합 1), SAP는 rigid collider가 전부 mesh 경유라
> 터널링에 취약해서(조합 4) 실전 조합은 거의 IPC로 수렴했다.

### Robotarm(M0609+RG2) 관련 공통 테크닉

이후 조합들에서 반복 사용되는 로봇팔/그리퍼 구현 기법:

- **Mimic joint (4-bar 폐루프 근사)**: RG2 그리퍼는 실물이 4-bar 폐루프 기구라
  RNE 계산이 트리 구조를 요구하는 시뮬레이터에서 그대로 풀 수 없다(§4).
  MJCF `<equality><joint joint1=... joint2="gripper_joint" polycoef="0 1 0 0 0"/>`
  로 능동 관절(`gripper_joint`) 하나만 구동하고 나머지 5개 관절(truss_arm/
  finger_tip×2, mirror)을 1:1 선형 동기화하는 URDF-mimic 방식을 채택
  (`m0609_rg2_v2.xml`). **단, Genesis 1.1.0은 `<equality><joint>`를 제대로
  안 지킨다** — 네이티브 MuJoCo(`mj_step`)는 정상 동작하지만 Genesis에서
  쓸 때는 이 우회가 필요: 6개 gripper DOF를 매 스텝 전부 동일 값으로
  `set_dofs_position`/`control_dofs_position` 직접 호출(폐루프 풀이를
  코드 레벨에서 흉내). 정합 관절/각속도 액추에이터 조합이 기본 Euler
  적분기에서 qacc 발산을 일으켜 `integrator="implicitfast"`로 전환.
- **Convex decomposition (CoACD)**: `f1_flex_finger`/`f2_flex_finger`의
  실제 손가락 패드 메시는 오목(non-convex)이라 충돌 판정에 그대로 쓰기
  까다롭다 — CoACD로 7개의 볼록 조각(`flex_finger_hull_000~006.stl`)으로
  분해해 각각을 독립 collision geom(`friction="1.5 0.02 0.001"`)으로
  MJCF에 주입(`_prepare_robot_mjcf()`, `grasp_bag_tablet_ipc_test.py` 등).
- **Crusher 쪽 별도 기법**: crank-slider 4-bar도 동일한 이유로 폐루프를
  직접 안 풀고, `link3`–`shaft` 연결을 끊은 뒤 `<equality>`로 재접합(§3-1).
  self-collision이 불필요한 대부분의 바디는 collision-free 처리하고, 관통이
  필요한 `Left_Wall`만 별도로 볼록분해(§3-2).

### 조합 1 — Robotarm(Rigid, mimic+convex decomp) + Samplebag(PBD.Cloth) + LegacyCoupler(rigid_pbd) · **실패 → 폐기**

(`M0609_RG2/grasp_bag_test.py`, §5-2)
- PBD 입자 자기충돌 방지 제약과 손가락 mesh 침투 해석이 충돌 → **핑거가
  봉투에 닿는 순간 즉시 폭발**. §5에서 이미 우려했던 "파지 시 particle
  최소 간격 제약이 깨질 수 있다"는 리스크가 실제로 발생한 것.
- 회피책 없이 **IPC 커플러로 전면 전환**하는 것으로 결론(조합 2로 이동).
  PBD 자체는 폐기하지 않고 파우더 표현 등 다른 용도로는 남겨둘 여지 있음
  (§5의 MPM→Rigid→PBD 3단 결합 구상은 아직 미시도).

### 조합 2 — Robotarm(Rigid, mimic+convex decomp) + Samplebag(FEM.Cloth, 2D) + IPC coupler · **성공**

(`M0609_RG2/grasp_bag_ipc_test.py`, `grasp_box_test.py`, `grasp_bag_tablet_ipc_test.py`, §5-2)
- IPC는 PBD를 못 보므로 봉투 재질을 `FEM.Cloth`로 교체. 실측 STL(두께 0
  핀치 실링선 포함)을 그대로 넣으면 self-intersection sanity check 실패 →
  원인은 위상이 아니라 **패널 간격(1.2mm)이 material thickness(2mm)보다
  얇아서 생긴 자기충돌**(로그 레벨을 올려야 보이는 네이티브 에러) — 검증된
  procedural 5-panel 프록시(간격 6mm)로 교체해 해결.
- 초기 버전은 6DOF mimic(v2) 대신 **단순 2DOF(m0609_rg2.xml) +
  `set_dofs_position` 텔레포트**를 썼다 — IPC의 `two_way_soft_constraint`가
  반력을 자체 처리해 PD 게인 튜닝이 불필요했기 때문. 이후 박스 파지
  검증(`grasp_box_test.py`)에서 `control_dofs_position` 통합 호출 +
  `noslip_iterations=20`/`constraint_timeconst=0.005`(정지 시 접촉 소실
  방지) 조합을 확정하면서 v2 6DOF mimic + convex decomp 그리퍼로 이행.
- 결과: 실링부(~1cm) 순수 마찰 파지(weld 없음), 봉투 Δz +125.8mm.

### 조합 3 — Robotarm(Rigid, M0609 복잡 메시) + Tablet(FEM.Elastic) + IPC coupler · **부분 해결 → 최종적으로 조합 5의 근본 원인으로 수렴**

(`M0609_RG2/grasp_bag_tablet_ipc_test.py`, §8)
- 정제가 tetgen 산출 메시(Sphere/STL)면 M0609 앞에서 거의 얼어붙고, `Box`
  primitive만 정상 낙하하는 패턴을 처음 발견한 조합. 당시엔 "TetGen 산출물
  자체가 원인"으로 결론지었으나, 이후 조합 5·6에서 밝혀진 **self-contact
  d_hat 오탐**이 진짜 원인이었을 가능성이 높음(같은 증상, 더 정밀한 원인).

### 조합 4 — Tablet(FEM.Elastic, 실측 2GPa) + Crusher Plate(Rigid) + SAP coupler · **부분 성공(저속) / 실패(고속) → IPC로 대체**

(`FEM/fem_uniaxial_compression.py`, §6-1, §7-7, §7-9)
- Franka 공식 예제 패턴(`sap_coupling/franka_grasp_fem_sphere.py`) 그대로
  quasi-static 압축(plate 속도 0.01mm/s)에는 성공 — 진짜 Rigid-FEM 접촉으로
  σ-ε, F(t), W(t) 실측.
- **터널링 발견**: SAP coupler는 rigid collider를 primitive로 안 쓰고
  **Box plate조차 trimesh→tet 메쉬로 변환**하며 primitive plane은 명시적
  금지(`GEOM_TYPE.PLANE`→raise). 접촉이 `FEMSurfaceTetLBVH`↔`RigidTetLBVH`
  BVH 이산 접촉이라, plate가 한 스텝에 정제 표면 tet 두께를 넘게 움직이면
  겹침을 놓쳐 관통 — plate 속도를 올리면(비-quasi-static) 재현됨.
- **추가 발견**: plate 속도가 0/미정의면 SAP가 contact impulse를 안 보내다가
  penetration이 깊어지면 한 번에 큰 임펄스로 튕기는 불연속 spike 거동(§7-9).
- 결론: primitive contact이 진짜 필요한 조합(빠른 충돌)엔 IPC가 맞고, SAP는
  quasi-static 압축처럼 느린 케이스에만 신뢰. dt 축소/substeps 증가/plate
  속도 ramp로 완화는 가능하나 근본 해결은 아님.

### 조합 5 — Tablet(3D FEM.Elastic, fan tetrahedralization) + Samplebag(2D FEM.Cloth) + IPC coupler · **해결**

(`utills/primitive_tablet_generator.py`, `FEM/fem_tablet_drop.py`,
`FEM/fem_tablet_drop_stiff.py`, `FEM/fem_tablet_solo_diag.py`, §8)

TetGen을 아예 안 쓰고 표면을 파라메트릭 방정식으로 생성 + 사면체는 순수
기하학적으로 계산하는 방식(§8) 자체가 핵심 아이디어. 이 조합에서 겪은
문제가 가장 복잡해 3단계로 풀렸다:

1. **얼어붙음(1차, v1 fan tet)**: 전역 centroid 하나로 부채꼴 이으면 극(pole)
   근처에 종횡비 나쁜 sliver tet가 다수 생김(반지름 2.5mm인데 극-centroid
   거리 6mm) → M0609/FEM.Cloth 양쪽 모두에서 얼어붙음. `make_capsule_tets_v2`
   (medial-axis 다중 앵커: 반구는 자기 중심점, 원기둥은 축 위 국소 앵커 +
   표준 프리즘→3-사면체 분해)로 sliver를 대폭 줄여(edge-비율 15.0→6.25)
   해소 — Box와 거의 동일한 정상 낙하 회복.
2. **강성 상향 스윕**: 위 해결로 확보한 v2 캡슐 + explicit dt를 파동속도
   스케일링(`dt ∝ 1/√E`)해가며 E를 5e4(안정화용)→1e6(20x)→1e7(200x)까지
   올려도 발산 없이 낙하·안착(`fem_tablet_drop_stiff.py`). E=1e8(2000x)에서는
   "중력을 무시하고 정지"하는 별개의 새 현상 재현 — 강성-질량 수치 불균형
   가설, 원인 미확정.
3. **"우그러짐"의 진짜 근본 원인(최종)**: 위 1·2단계 내내 정제가 접촉 전부터
   미세하게 압축된 채로 보였던 게 사실은 **`contact_d_hat`이 캡슐 극 근처
   정점 간격(0.32mm)보다 컸던 것**(당시 값 5e-4=0.5mm) — IPC self-contact
   배리어가 정상적으로 가까운 인접 정점을 충돌로 오인해 t=0(접촉 불가능한
   자유낙하 중)부터 극을 짓누름. 봉투/선반 없는 정제 단독 낙하로 재현해
   접촉 무관함을 확정하고, `scipy.cKDTree`로 최근접 이웃 거리를 실측해
   d_hat을 그보다 충분히 작게(1e-4~5e-5) 낮추자 완전 해소 — 자유낙하 중
   설계 형상 그대로 유지, 봉투 접촉부터 비로소 진짜 동적 변형 시작.
   **교훈(표준 절차화)**: 곡률 큰 FEM 메시는 `contact_d_hat`을 정할 때
   오브젝트 간 간극뿐 아니라 **메시 자신의 최소 정점 간격**부터 확인할 것.
4. **위 3가지가 사실 하나로 수렴**: "강성을 올릴수록 얼어붙음이 심해진다"
   (앞서 관찰)와 "self-contact 오탐이 근본 원인"(나중 발견)을 합치면, 극의
   가짜 self-contact 고정력이 재질이 뻣뻣할수록 몸체 전체 움직임을 더 강하게
   묶어버리는 것으로 설명됨 — 작은 정제(정점 간격이 더 촘촘)일수록, 강성이
   높을수록 같은 d_hat 버그가 "약간의 압축"에서 "완전한 정지"까지 다양한
   심각도로 나타난다.

### 조합 6 — Tablet(Rigid Body, MJCF capsule geom = SDF) + Samplebag(FEM.Cloth) + IPC coupler · **해결**

(`FEM/rigid_capsule_tablet_bag_ipc_test.py`, `FEM/fem_tablet_drop.py`
`TABLET_MODE=rigid_sdf`, §8)
- 정제를 변형체가 아니라 **강체 SDF**(Genesis에 `Capsule` 모프 클래스는
  없지만 MJCF `<geom type="capsule">`는 `GEOM_TYPE.CAPSULE`, 즉 메시가 아닌
  radius+height 파라미터의 analytic 표현으로 처리됨)로 넣으면 형상 왜곡이
  원천적으로 불가능해진다는 아이디어.
- **1차 시도 실패**: `coup_type="two_way_soft_constraint"`로 접촉 순간 위치가
  수 미터로 발산. `constraint_strength`를 100→0.5로 낮추면 발산은 막히지만
  감쇠 없는 바운스가 계속됨 — 근본 원인 미상으로 잠정 보류.
- **원인 특정**: `coupler.py`의 `_setup_coupling_config`가 `coup_type=None`
  일 때 **`entity.n_joints>0`(관절 있음, freejoint 포함)이면 자동으로
  `two_way_soft_constraint`**를 선택함 — 이건 PD로 계속 구동되는 제어
  대상(로봇 팔 등)을 위한 경로다. 캡슐의 MJCF `<freejoint/>`가 "관절 1개"로
  카운트되어, 아무 명령도 받지 않는 순수 낙하체에 잘못된 경로가 자동
  적용된 것.
- **해결**: `material=gs.materials.Rigid(coup_type="ipc_only", ...)`로
  자동선택을 명시적으로 override(Plane/Shelf가 이미 쓰던, 검증된 one-way
  경로 — "IPC가 중력·동역학 전부 담당"). `constraint_strength`는 기본값
  100 그대로 두고도 발산 없이 낙하→봉투 입구 통과→바닥 안착까지 성공.
- **한계**: 강체라 정제의 압축·파손 거동은 볼 수 없음 — 그런 관찰이
  필요하면 조합 5(FEM+FEM)를 쓴다.

### 조합 7 — Tablet(FEM.Elastic, Box primitive vs 우리 정의 analytic Box) + Samplebag(FEM.Cloth) + IPC coupler · **변인통제 실험(조합 5 진단용)**

(`utills/primitive_tablet_generator.make_box_tets`, §8)
- 조합 5의 sliver 가설을 검증하기 위한 대조군: `gs.morphs.Box`(TetGen 경유)
  vs 우리가 fan tetrahedralization으로 직접 만든 analytic Box(정점 9개/
  tet 12개, TetGen 미사용) — 둘 다 FEM.Cloth 앞에서 거의 완전히 동일한
  정상 자유낙하(106.5/68.6/-81.1mm vs 106.55/68.62/-81.10mm, t=50/100/200ms).
  **결론**: "TetGen 경유 여부"는 무관했고, 우리 수식 정의 방식 자체도
  정상 — 문제는 항상 캡슐의 sliver/self-contact 쪽이었다.

### 조합 8 — Tablet(Rigid+SDF) + Samplebag(FEM.Cloth) + Robotarm(Rigid, mimic+convex decomp) + IPC coupler · **미해결 → Tablet을 FEM으로 우회해 파이프라인은 완성**

(`M0609_RG2_Tablet_Samplebag/tablet_bag_grasp_pipeline.py`, 2026-07-15)

조합 2(로봇+봉투)와 조합 6(정제+봉투)는 각각 따로는 검증됐으니, 셋을 합쳐
"정제 낙하 → 봉투가 받음 → 로봇이 봉투(+정제)를 파지·리프트"하는 최종
워크플로우를 그대로 합쳐서 시도했다.

**증상**: 셋을 한 씬에 합치자 로봇을 전혀 구동하지 않아도(정지 상태로 두기만
해도) 정제 위치가 단 1~2 스텝만에 지수적으로 발산했다(k=1에 2.9m, k=11에
10²³mm 스케일). `constraint_strength_translation/rotation`을 100(로봇 단독
검증값)→30→10 으로 낮춰도 15~30스텝 내 동일하게 발산. **1.0까지 낮추면
발산은 멈추지만, 이번엔 정제가 봉투를 그대로 뚫고 바닥까지 가속 낙하**해
버렸다(접촉 해석 자체가 무력화됨) — 즉 "로봇 파지에 필요한 강한 결합"과
"정제의 정상적인 접촉 해석"을 동시에 만족하는 `constraint_strength` 값을
찾지 못했다.

**원인 조사**: Genesis 공식 예제(`examples/IPC_Solver/*.py`) 4개를 전수
확인한 결과, `ipc_robot_cloth_teleop.py`에 Rigid(Franka, `two_way_soft_
constraint`) + FEM.Cloth + Rigid(`ipc_only`)의 **같은 조합이 존재**하지만,
그 `ipc_only` 엔티티(16개 박스)는 전부 **`fixed=True`(정적 소품)**였다.
그 예제의 커플러 설정(`enable_rigid_rigid_contact=True`,
`newton_semi_implicit_enable=False`, `contact_resistance=1e7`,
`newton_tolerance=1e-1` 등, 우리보다 훨씬 느슨함)을 그대로 우리 씬에
적용해도 발산은 재현됐다 — 즉 "설정 미스매치"가 아니라, **동적(freejoint)
`ipc_only` 강체가 `two_way_soft_constraint` 강체와 같은 IPC 시스템에
공존하는 조합 자체가 검증된 바 없는 조합**으로 판단된다(`ipc_objects_
falling.py`는 동적 `ipc_only` 박스가 있지만 `two_way_soft_constraint`
엔티티가 아예 없는 씬이라 이 조합을 커버하지 않음). 코드 레벨 근본 원인은
미확정(후속 조사 항목).

**우회(채택)**: 정제를 Rigid+SDF 대신 **FEM(조합5, sliver-free 캡슐)**으로
바꿨다 — 그러면 씬의 Rigid 엔티티가 로봇 하나뿐이라 `ipc_only`+
`two_way_soft_constraint` mismatch 자체가 사라진다. `constraint_strength`
를 로봇 단독 검증값(100.0) 그대로 쓸 수 있었고, `contact_d_hat=1e-4`(조합5
self-contact 교훈 반영)로 정제도 낙하 중 형상 왜곡 없이 정상 동작.
**결과: 발산 없이 전체 파이프라인 완주** — 정제 낙하(492.97→426.57mm,
봉투 안 안착) → 그리퍼 파지 → 리프트(finger Δz=+126.1mm, bag grip
Δz=+125.9mm, tablet Δz=+89.6mm, 봉투/정제 동반 상승 둘 다 OK).

**교훈**: `coup_type`을 엔티티별로 다르게 섞을 때(`ipc_only` + `two_way_
soft_constraint` 등), 최소 하나가 **동적(freejoint)** 이면 공식 예제로
검증되지 않은 조합일 수 있다 — 강도 파라미터를 아무리 튜닝해도 "발산 아니면
접촉 무력화"라는 양자택일만 나올 수 있으니, 이런 조합을 새로 시도할 땐
먼저 공식 예제에 정확히 같은 조합(정적/동적 여부까지)이 있는지 확인하고,
없으면 파라미터 튜닝보다 **한쪽을 같은 도메인(FEM 등)으로 통일**하는 우회를
먼저 고려하는 게 시간 대비 효율적이다.

### 조합 9 — Crusher(Rigid) + 알루미늄 플레이트 4개(Rigid, 정적) + Tablet(FEM) + Samplebag(FEM) + Robotarm(Rigid, mimic+convex decomp) + IPC coupler · **해결, 전체 시퀀스(파지→리프트→슬롯 삽입) 성공**

(`Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py`, 2026-07-15)

조합 8 파이프라인에 Crusher 본체와 알루미늄 플레이트 4개(작업대)를 추가해
"정제 낙하 → 봉투가 받음 → 로봇이 봉투를 파지·리프트 → Crusher 슬롯까지
이동 → 슬롯에 삽입"까지 전체 워크플로우를 완성했다.

**CUDA 크래시 2건 (모두 `coup_type` 미지정이 원인)**:
1. Crusher 추가 시 `cudaErrorInvalidDevice`(scene.build() 내부 advance()에서
   발생) — 자동선택이 `external_articulation`을 골랐는데, 이는 모든 링크가
   충돌 지오메트리를 가져야 하는 조건이라 Crusher의 장식용 링크(충돌 지오메트리
   없음)에서 `Rigid link has no collision geometry` 예외로 이어짐. **해결**:
   `material=gs.materials.Rigid(coup_type="two_way_soft_constraint")`를
   Crusher에 명시(관절이 있는 구동체이므로 로봇과 동일 취급).
2. 알루미늄 플레이트 4개 추가 시 재발 — 원인은 플레이트가
   `gs.materials.Rigid()`로 `coup_type` 미지정(자동선택) 상태였던 것.
   **해결**: 플레이트도 `coup_type="ipc_only"` 명시(Plane/Shelf와 동일
   패턴 — 무관절 정적 소품).
   조합 8 교훈("엔티티별 coup_type을 섞을 때 최소 하나가 동적이면 위험")의
   실전 재확인: 이번엔 반대로 **모든 엔티티에 coup_type을 명시적으로
   지정**하는 것 자체가 근본 해결책이었다(자동선택에 맡기지 않음).

**봉투 형상 고정 — `set_vertex_constraints`의 IPC 커플러 버그 발견**:
로봇을 Crusher 슬롯 근처(거리 0.87m)에 두자 IK 오차가 8~12cm로 컸다.
로봇을 슬롯에서 ~0.65m 거리로 재배치하니 오차가 <0.001m로 줄었다(고정
자세 제약 하에서 IK 수렴은 로봇 베이스로부터의 거리에 매우 민감함).

첫 실행에서 봉투가 아무 지지 없이 중력만으로 버티다 정제가 채 들어가기도
전에 처져버렸고(§5), 이후 그리퍼가 126mm 올라가는 동안 봉투는 제자리에
그대로 남아 파지가 완전히 실패했다. 조사 결과 봉투를 고정하는 함수가
프로젝트 어디에도 없었다 — grasp 자체는 순수 마찰 접촉으로 설계돼 있었지만
(치트 없음), 그 이전에 형상 자체가 무너져 파지 대상이 사라진 것.

Genesis `FEMEntity.set_vertex_constraints()`가 정확히 이 용도(PBD의
`fix_particles_to_link` 대응)지만, 소스 확인 결과 버그가 있었다
(`fem_entity.py` 900행 부근):
```python
if isinstance(self.sim.coupler, IPCCoupler):
    gs.raise_exception("This method is only supported by IPC coupler.")
```
조건이 뒤집혀 있어 "IPC 커플러에서만 지원됨"이라는 메시지와 반대로 **IPC
커플러를 쓸 때 정확히 예외를 던진다** — 우리 씬은 전부 IPC 커플러이므로
이 메서드가 사실상 항상 막혀 있었다(실측 확인:
`bag.set_vertex_constraints(...)` → `GenesisException`). `update_constraint_
targets()`/`remove_vertex_constraints()`는 이 체크가 없어 정상 동작.

**우회**: Genesis 설치본(site-packages)은 건드리지 않고, 원본 로직을 그대로
복제하되 버그인 `isinstance` 체크 한 줄만 제거한 함수로 런타임에
`FEMEntity.set_vertex_constraints`를 몽키패치(`utills/fem_ipc_workarounds.
patch_fem_vertex_constraints()`, `primitive_tablet_generator.py`의
`mesh_to_elements` 패치와 동일한 기법).

고정 정점 범위는 3회 반복 튜닝: 정점 77%(입구 제외 전부)를 고정하니 자유
상태인 입구가 오히려 비현실적으로 늘어남(50스텝 내 -390mm) → 바닥 밴드
(12mm)만 고정하니 150스텝까지는 버티다 이후 붕괴(z-span 88.7→31.8mm) →
**바닥(12mm) + 양 측면(각 8mm), 입구만 완전히 자유** = 정점 303/771(39%)
고정이 격리 테스트에서 설계 치수(90mm z-span)를 200+ 스텝(1초+) 안정
유지함을 확인. `close`(그리퍼 닫기) 직전에 `remove_vertex_constraints()`로
전부 해제, 이후 `grasp`/`lift`는 순수 마찰로 진행.

**결과 (`_full_workflow_run2.log`)**: 고정 적용 후 `prep`/`drop`/`settle` 동안
봉투 COM이 사실상 고정(0.4321→0.4322, 5번째 소수점 드리프트)된 채 정제가
정상적으로 낙하(456.83→431.44mm)했고, 해제 후 `close`→`grasp`→`lift`에서
봉투 COM_z가 0.4315→0.5560(+125.9mm)로 상승해 finger_z 상승분(0.4360→
0.5621, +126.1mm)과 거의 정확히 일치 — **순수 마찰 파지가 실제로 성공**한
것을 수치로 확인(이전 실행에선 finger_z는 동일하게 126mm 올라갔지만
bag_com은 완전히 그대로였음 — 완전한 반전). `above`/`insert` 구간에서는
봉투·정제가 팔의 이동을 따라 흔들리며 이동했고(스윙 있음), 최종적으로
봉투가 Crusher 슬롯 가이드 벽 틈 사이로 내려간 것을 `bagcam` 영상 프레임
(`insert_end`, `end`)으로 시각 확인함 — 그리퍼에 물린 흰색 봉투 메쉬가
슬롯의 검은 가이드 벽 사이에 걸쳐 있는 모습.

영상: `RESULT/full_workflow_20260715_162834_overview.mp4`,
`RESULT/full_workflow_20260715_162834_bagcam.mp4`.

**교훈**: (1) 복잡한 다중 엔티티 IPC 씬에서는 **모든** Rigid 엔티티에
`coup_type`을 명시하는 편이 안전하다(자동선택 규칙이 장식용 충돌-지오메트리
없는 링크나 무관절 소품에서 예상 밖의 타입을 고를 수 있음). (2) Genesis API
문서/타입힌트만 믿지 말고 실제 예외가 발생하면 소스를 직접 읽을 것 —
`set_vertex_constraints`의 `isinstance` 체크는 정확히 반대로 뒤집힌 버그였고
메시지만 봐서는 알아챌 수 없었다. (3) FEM 형상을 grasp 전 안정화할 때는
"입구/파지 대상 부위는 자유, 나머지 골격(바닥+측면)만 고정"하는 최소 고정이
과다 고정(전체 고정)보다 결과가 좋다 — 과다 고정은 해제 안 된 자유 영역에
비현실적 응력 집중을 유발한다.

### 조합 9 후속 — 씬 튜닝 · 슬롯 삽입 위치 재조사 (2026-07-15 2~5차)

(`Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py`, run3~run7)

조합 9 1차 성공 이후 사용자 시각 검수 피드백을 반영해 5차례 재실행하며
장면 품질·삽입 위치·기구 이해를 다듬었다.

**1) 조명/배경 튜닝**: 기본 조명(ambient 0.1, key light 1개 intensity 5.0)이
너무 어두워 45도 키 라이트(intensity 8.0) + 필 라이트(3.0) + ambient 0.35 로
올렸더니 이번엔 명암 대비가 거의 없이 밋밋(washed-out)해짐 → ambient 0.16,
key 6.0, fill 1.2 로 재조정해 대비를 되살림. 격자무늬 Ground 는
`gs.morphs.Plane(visualization=False)`로 충돌(안전망)은 유지하되 렌더만 꺼서
알루미늄 플레이트만 보이게 함.

**2) `contact_d_hat` 축소 시도 → 성능 폭증 → 원복**: 정제 우그러짐 재발
우려로 `IPC_D_HAT` 1e-4→5e-5 로 낮췄더니 `scene.build()` 내부 warm-start
솔브가 30분+ 로 폭증(직전 1e-4 실행은 빌드+전체스텝+인코딩 합쳐 16분18초).
원인: `contact_d_hat`은 정제 하나의 self-contact 뿐 아니라 **씬 전체(플레이트
4개+Crusher+로봇+봉투+정제)의 모든 접촉쌍**에 적용되는 커플러 전역 설정이라,
민감도를 2배 높이면 이 복잡한 다중 엔티티 씬 전체의 접촉 해석 비용이
크게 늘어난다 — 격리 테스트(정제 단독)에서는 이 비용이 안 보였던 것. 정제
극 근처 최소 정점 간격(0.32mm) 대비 1e-4 의 비율은 0.31 로 이미 문서 기준
"1/3 이하" 안전 마진 안이었으므로 원복(1e-4).
**교훈**: 씬 전역 파라미터를 국소 문제(특정 엔티티의 self-contact) 해결
목적으로 낮출 땐, 그 파라미터가 정말 국소적인지부터 확인할 것 — 아니라면
전체 씬 규모에서 비용을 먼저 가늠해야 한다.

**3) 실링부 색칠 — vertex color 는 무시됨, UV+텍스처만 동작**: Genesis 는
mesh 에 구운 `vertex_colors`(PLY 로 저장해도)를 렌더에 반영하지 않는다(격리
테스트로 확인 — 단색으로만 나옴; `utils/mesh.py`의 `_get_texture`가
`ColorTexture`/`ImageTexture` 두 경우만 처리하고 mesh 자체의 vertex color
경로가 없음). 대안으로 봉투 로컬 좌표(폭 ±32mm, 높이 ±45mm, 이미 원점 중심)에
평면 UV(`u=x/64mm+0.5, v=y/90mm+0.5`)를 구워 OBJ 로 내보내고, 좌우 가장자리
10mm 대역(`|local_x|>22mm`)만 다른 색인 128×128 스트라이프 텍스처를
`surface=gs.surfaces.Default(diffuse_texture=gs.textures.ImageTexture(...))`
로 입혀 해결 — 격리 렌더 테스트로 확인.

**4) 파지 위치를 봉투 최상단(입구)으로 이동**: `TOP_GRIP_MARGIN=8mm` 추가,
`BAG_POS_z = FINGER_MID_z - BAG_HALF_H + TOP_GRIP_MARGIN`(이전엔 봉투
중앙 높이가 FINGER_MID 와 일치) — 그리퍼가 입구에서 8mm 안쪽을 물고, 봉투
대부분이 그 아래로 늘어지는 구조로 변경.

**5) 슬롯 삽입 위치 — 3단계 시행착오**:
   - *5-1 (경험적 보정)*: gap 슬릿 중심(gap_cx,gap_cy)을 목표로 주면 파지된
     봉투가 관성/처짐으로 목표에 못 미쳐(벽 쪽 -42/-28mm 부족) 도착 — 실측
     shortfall 만큼 목표를 벽 쪽으로 미리 밀어 보정(1차 완화, 완전 해결은
     아님).
   - *5-2 (포켓 바닥판 오인)*: "봉투 하단이 벽 쪽에 더 붙어야 한다"는 지적을
     "`L1_Wall1_1`(포켓 바닥 플레이트) 중심까지 가야 한다"로 확대 해석해
     목표를 그쪽으로 옮김. 하지만 이 바닥판은 12mm 폭 gap 슬릿보다 더
     안쪽(-x 방향)에 있어 단순 수직 하강으로는 도달 불가능한 자리였다 —
     실측 로그에서 `insert`/`settle2` 내내 bag_com.x 가 목표로 수렴하지 않고
     **오히려 멀어지는** 패턴으로 이를 확인(벽 구조물에 막힘).
   - *5-3 (근본 수정, docs/Crusher.md 재확인)*: `Crusher.md` §5·§11-5 를 보면
     `Left_Wall`(`L1_Guide1_1_L2_Left_Wall1_1`, Motor2, Rack&Pinion)이 바로
     "샘플백 홀더" — "모터를 계속해서 구동시킴을 통해서 강하게 고정"한다고
     명시돼 있다. 즉 **봉투를 포켓 깊숙이 옮기는 건 로봇의 역할이 아니고,
     로봇은 gap 근처까지만 옮기면 Left_Wall 이 닫히며 실링부를 Wall3 에
     눌러 고정**하는 게 실제 기구 설계다. 목표를 `Crusher_Samplebag.py` 원안
     (`target_x=gap_cx, target_z=wall_top_z`)으로 되돌림 — 파지점이 이제
     봉투 최상단이라 `gripper_z ≈ 입구 높이`가 `wall_top_z` 목표와 자연스럽게
     대응된다.
   - **저비용 사전검증 스크립트**(`slot_ik_check.py`, 신규): above/insert
     목표에서 로봇팔이 Crusher 본체와 충돌하는지를, 무거운 IPC+FEM 풀 씬을
     빌드하지 않고 **Rigid-only(Crusher+Robot 만, coupler 기본값)** 로 빠르게
     검증(빌드 수십 초 vs 풀 파이프라인 4~16분). `entity.get_contacts
     (with_entity=...)` 로 손가락 제외 링크가 Crusher 와 접촉하는지 확인 —
     above/insert 모두 접촉 0건으로 "로봇팔이 슬롯에 같이 들어가는" 문제는
     Crusher 본체와는 무관함을 확인(사용자가 우려했던 지점). 참고로 같은
     검증에서 팔 아래쪽 링크가 **바닥 plane 과는 접촉**함을 발견했는데,
     본 파이프라인은 로봇을 `set_dofs_position`(운동학적 텔레포트)으로
     구동해 이 접촉이 물리적으로 강제되지 않으므로(렌더상 살짝 파고드는
     정도) 봉투/정제 물리에는 영향 없음 — 미해결 코스메틱 이슈로 남겨둠.

**6) `clamp`/`release` phase 신규 추가 — 미해결 이상 발견**: `insert`/
`settle2` 이후 `Left_Wall` 을 `WALL_OFFSET`(+6mm)→`CLAMP_TARGET`(-5mm, 2.0s)
로 램프하는 `clamp` phase 와, 그 뒤 그리퍼를 여는 `release` phase 를
추가(`Crusher_Samplebag.py` 의 `CLAMP_TARGET` 재사용). **증상**: 명령한
목표는 -5mm 인데 `crusher.get_dofs_position()` 실측값이 **-159.42mm** 로
찍힘 — 물리적으로 불가능한 수치. 그런데 `overview` 카메라로는 clamp 진행
중 벽이 시각적으로 거의 안 움직였고, `bagcam` 에는 봉투(실링부 색 포함)가
가이드 블록과 모터 하우징 사이에 눌린 모습만 보인다. **추정 원인**:
Crusher 가 로봇과 동일하게 `coup_type="two_way_soft_constraint"`(rigid
솔버가 계산한 목표를 스프링으로 IPC 쪽 실제 강체에 당기는 방식)로 처리되는데,
벽이 봉투와 접촉해 저항을 받으면 `constraint_strength=100.0` 스프링이 그
반력을 못 이겨 **rigid 솔버 쪽 DOF 값만 목표를 향해 계속 헛돌며 발산**하고,
IPC 가 실제로 시뮬레이션하는 강체 자체는 접촉에 막혀 거의 못 움직이는
것으로 보인다 — 로봇 손가락-봉투 결합에서 이미 확인된 "soft constraint 가
약해 반력을 못 버틴다" 문제와 같은 계열이, 이번엔 Crusher 자체의 벽-봉투
접촉에서 재현된 것. **미해결** — Crusher 전용으로 `constraint_strength`를
올려보거나, clamp 구간만 다른 결합 방식을 쓰는 등의 후속 조사가 필요하다.

영상: `RESULT/full_workflow_20260715_202218_overview.mp4`,
`RESULT/full_workflow_20260715_202218_bagcam.mp4`(run7, clamp/release
포함 — clamp 수치 이상 있는 채로 저장됨).

**교훈**: (1) "위치가 부정확하다"는 피드백을 받으면 목표를 더 밀어넣기 전에
그 방향이 애초에 기구적으로 맞는 요구인지(문서·실측 로그의 수렴 여부)부터
확인할 것 — 5-2 처럼 도달 불가능한 목표를 밀어붙이면 로그상 "수렴 실패"
패턴(목표에서 멀어짐)으로 드러난다. (2) `two_way_soft_constraint` 의
`constraint_strength` 부족 문제는 로봇-봉투 접촉에만 국한된 게 아니라
**두 way-soft-constraint 로 묶인 어떤 rigid 엔티티든 강한 접촉 저항을 받으면
동일하게 재현될 수 있는 일반적인 실패 모드**로 간주해야 한다. (3) 비싼
풀 IPC+FEM 빌드 전에 rigid-only 저비용 씬으로 IK 목표의 충돌 여부만 먼저
검증하는 방식(`slot_ik_check.py`)이 반복 튜닝 비용을 크게 줄여준다.

### 조합 9 후속 2 — IK 링크 오프셋 버그, 봉투 축 뒤바뀜 버그, 인터랙티브 프로브 (2026-07-16)

(`Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py`, `interactive_probe.py` 신규)

**버그 A — IK 타깃 링크가 손가락이 아니라 손목 브라켓(`gripper_body`)이었다**:
above/insert IK 를 지금까지 `gripper_body` 링크에 대해 풀고 있었는데, FK
실측 결과 이 링크는 실제 손가락(봉투가 매달린 지점, `f1_flex_finger`)과
**world z 로 140mm, x 로 34mm** 나 어긋나 있었다(RG2 내부 링크(moment_arm/
truss_arm)를 거쳐 손가락이 그만큼 아래에 붙는 구조). `insert_z=wall_top_z`
로 `gripper_body` 를 명령하면 실제 손가락은 140mm 아래(포켓 바닥 근처, 심지어
z<0)까지 내려갔다 — 이전 라운드들에서 "목표에 도달 못 함/벽에 막힘"으로
보였던 현상 상당수가 사실 이 링크 오프셋 때문이었다. **해결**: IK 타깃
링크를 `left_link`(`f1_flex_finger`)로 교체 — 격리 검증 결과 오차 0.19mm,
Crusher 접촉 0건.

**버그 B — `BAG_EULER` 축이 뒤바뀌어 있어 봉투의 넓은 면이 좁은 gap 과
부딪히고 있었다**: `BAG_EULER=(90,0,0)` 에서 봉투의 world 좌표계 크기는
X=64mm(폭+실링), Y=6mm(두께), Z=90mm(높이) 였는데, 슬롯 gap 은 X=12mm(좁음),
Y=65mm(여유) — **봉투의 넓은 면(64mm)이 좁은 12mm gap 과, 얇은 면(6mm)이
여유로운 65mm 쪽과 부딪히는 축이 뒤바뀐 상태**였다(trimesh 로 직접 회전행렬
적용해 검증). 원래 참고했던 `Crusher_Samplebag.py` 는
`BAG_EULER=(90,0,**90**)` 을 썼는데(마지막 Z축 90도가 우리 코드에서 누락돼
있었음) 이걸 추가하면 X=6mm(두께, gap 12mm 대비 여유 3mm씩), Y=64mm(폭, gap
65mm 대비 여유 0.5mm씩 — 타이트하지만 통과 가능)로 정확히 뒤바뀐다. 높이
(local Y→world Z)는 이 변경과 무관하게 그대로 유지된다.
**연쇄 수정**: 그리퍼 닫힘축도 봉투 두께축을 따라가야 하므로 손목(joint 6)
트위스트(+90°, "봉투 두께=world Y 시절"에만 필요했던 우회)를 제거(0°) —
FK 실측으로 wrist=0 일 때 핑거가 world X축으로, wrist=+90°일 때 Y축으로
벌어짐을 확인. `FINGER_MID`(그리퍼 중심 기준점), `SEAL_LOCAL_X` 적용 축
(로컬 폭축이 이제 world Y 로 매핑), 봉투 형상 고정용 `bag_side_mask`(이제
`by` 기준)까지 전부 재측정/수정.

**insert 목표 재확정**: 저비용 rigid-only 스윕(`gap 근처, "벽 중앙+margin"`
z 목표에서 margin 을 100mm→10mm 단위로 낮춰가며 손가락-Crusher 접촉 검사)으로
**margin=52mm(wall_top_z+16mm) 가 접촉 0건인 가장 타이트한 값**임을 확인
(margin=45mm 부터 접촉 발생 — 핑거 폭이 12mm gap 보다 넓어 핑거 자체는 gap
을 통과할 수 없고, 봉투만 통과해야 함).

**인터랙티브 프로브 도구(`interactive_probe.py`, 신규)**: "위치를 계속
추측→실행→영상확인 하는 루프가 너무 느리다"는 피드백으로, 로봇 없이 봉투
치수(6×64×90mm)의 rigid 프로브 박스를 라이브 뷰어에서 마우스로 직접 드래그해
슬롯에 넣어볼 수 있는 도구를 만들었다. Genesis 뷰어의
`MouseInteractionPlugin`(`gs.vis.viewer_plugins`)을 사용.
- **1차 시도(GPU 백엔드 + `use_force=True`(스프링힘) + Crusher 관절을 매
  스텝 PD 로 계속 붙잡음)**: 사용자가 실수로 Crusher 의 크랭크/벽 링크를
  잡으면 스프링힘과 PD 힘이 충돌해 `Invalid constraint forces causing
  'nan'` 발산. 픽업 자체는 되고 있었는데(디버그 로그로 `[pick] OK` 확인)
  이 충돌 때문에 몇 초 안에 항상 죽었다.
  print 출력이 안 보이는 것도 원인 중 하나였다 — stdout 을 파일로
  리다이렉트하면 파이썬이 완전 버퍼링돼서 프로세스가 끝나야 로그가
  한꺼번에 flush 됨(`python -u` 로 해결).
- **해결(공식 예제 `genesis-world/examples/viewer_plugin/mouse_interaction.py`
  그대로 참고)**: CPU 백엔드 + `use_force=False`(스프링 대신 위치를 직접
  set — 발산 자체가 불가능) + Crusher 관절은 루프 진입 전 딱 한 번만
  세팅(이후 매 스텝 PD 로 안 붙잡음, 드래그와 충돌할 대상이 없어짐). 이
  패턴으로 바꾸자 크래시 없이 정상적으로 드래그 가능함을 확인.
- **결과**: 사용자가 직접 프로브를 슬롯에 넣어보고 "슬롯에 삽입하는 문제는
  여유가 있는 것으로 보인다"고 확인 — 버그 A/B 를 고치고 나니 애초에
  걱정했던 "12mm/64mm 타이트 핏" 문제가 실제로는 크게 문제 되지 않음이
  인터랙티브 테스트로 검증됨.

**교훈**: (1) IK 타깃 링크를 고를 때 "엔드이펙터에 가까워 보이는 링크"가
아니라 **실제 작업점(여기선 손가락)과의 FK 오프셋을 먼저 실측**할 것 — 140mm
차이는 육안으로도 눈치채기 쉬웠어야 했는데, 여러 라운드 동안 "위치가 조금
안 맞는다" 정도로만 보여서 놓쳤다. (2) 좌표축 매핑(euler 회전)을 다른
스크립트에서 그대로 가져올 땐 **모든 축의 각도를 다 옮겨왔는지** 확인할 것
— 하나(마지막 Z축 90°)를 빠뜨린 게 몇 라운드에 걸친 "위치가 안 맞는다"
디버깅의 근본 원인이었다. (3) 반복 위치 튜닝이 막히면, 무거운 물리 파이프라인
전체를 반복 실행하기보다 **가벼운 인터랙티브 도구로 사람이 직접 확인**하는
쪽이 훨씬 빠르게 답을 준다 — Genesis 의 `MouseInteractionPlugin` 이 정확히
이 용도로 이미 만들어져 있었다.

---

### 조합 9 후속 3 — Q_GRASP/Q_LIFT 오리엔테이션 기울어짐 (2026-07-24)

(`Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py`)

`Q_GRASP`/`Q_LIFT`는 IK로 푼 값이 아니라 테스트하며 손으로 고른 조인트각
하드코딩 상수다. FK로 실측하니 손가락 진행축이 완전 수직에서 약 13.8° 기울어져
있었고(파지 시 봉투 비틀림의 유력 원인), 같은 위치에서 quat을 수직으로 강제한
IK는 오차 0.002mm로 수렴하고 조인트 여유도 충분했다 — 로봇 링크 근접에 의한
기구학적 한계가 아니라, 튜닝 당시 엔드이펙터 오리엔테이션을 제약 조건으로
넣지 않아서 생긴 어긋남이다.

### 조합 9 후속 4 — 석션V1 조(jaw) 자기교차, IPC build crash **[미해결, 작업 중]** (2026-07-29)

(`assets/robots/석션V1_description/석션V1.xml`, `full_workflow.py`,
`Fixture_only/fixture_recovery2_stack_sim.py`)

**작업 배경**: 회수장치2+고정장치 위치를 원상 복구(Jig-회수장치2 근접은
설계 의도, +25mm 인위적 Z-갭이 오히려 버그였음을 확인)한 뒤, 석션V1을
`full_workflow.py`에 배치 — 회수장치2의 `F_LeftLink_1`/`F_RightLink_1`
중점에 석션V1의 두 `Suction_Cup_M5_0.8mm` 링크 중점이 오도록 정렬(회수장치2가
봉투를 고정하고 석션V1 석션컵이 양옆으로 당기는 구조). 정렬 후 석션V1
베이스와 고정장치 베이스 간 실제 겹침(link-level AABB)이 발견되어 X를
+0.25m 시프트해 해소했다.

**증상**: 이 겹침을 전부 없앤 뒤에도 고정장치+석션V1, 회수장치2+석션V1,
셋 다 포함 씬 모두 동일하게 `scene.build()` 단계에서
`AttributeError: 'NoneType' object has no attribute 'body_count'`로 크래시
(직전에 IPC가 "Intersection detected" 로그를 찍음).

**진단**: "그동안 잘 됐었는데 왜 지금 죽나?"는 질문에서 출발 — 크로스-엔티티
겹침이 원인이라는 가설로 T1-Jig 케이스처럼 접근했으나 재현 안 됨. 최소
재현 조합(고정장치+석션V1 단 둘만)으로 좁힌 뒤, `entity.links` 순서를 직접
출력해 크래시 로그의 `Object[rigid_link_4_0(2)]` / `Object[rigid_link_5_0(3)]`
인덱스를 링크 이름에 정확히 매핑했다(이전 시도에서 인덱스 산수를 암산으로
하다가 다른 2-엔티티 조합에서 같은 인덱스 4/5가 나와 오판했던 적이 있어,
이번엔 실제 출력으로 확정): 인덱스 4/5 = **석션V1 자신의**
`L_E-SMLG9H-100-ES10_2_1` / `R_E-SMLG9H-100-ES10_2_1` (좌우 조 링크).
즉 크로스-엔티티 문제가 아니라 **석션V1 조 자체의 자기교차**였다.

**왜 "전에는 문제없었나"**: 두 조 링크는 기본 배치(`pos` y=+0.00345 /
-0.00345, 간격 약 6.9mm)에서 이미 항상 살짝 겹쳐 있었던 것으로 보인다.
IPC의 build-time 자기교차 유효성 검사(`coupler.build()` →
`_init_accessors()`)는 **공간적 근접성에 의존하는 것으로 추정**된다 —
석션V1이 씬에 혼자/다른 바디들과 멀리 떨어져 있을 땐 이 자기교차를
용인하다가, 다른 리지드 바디(고정장치/회수장치2)와 가까이 배치되자 같은
자기교차를 엄격하게 걸러낸 것으로 보인다(broad-phase 공간 분할/pruning
휴리스틱 추정, Genesis 내부 확인은 못 함).

**시도했다가 안 통한 것**: `contype`/`conaffinity` 비트마스크로 조끼리
충돌 제외 → 효과 없음(IPC 자기교차 검사는 MuJoCo 스타일 필터링을 무시하는
것으로 보임). MJCF `<contact><exclude body1=".." body2=".."/>` 직접 추가 →
ElementTree로 넣으니 무관한 파서 버그(`KeyError: 'vmesh'`) 유발, 이 경로는
포기.

**작동한 것(확인됨, 아직 실파일엔 미적용)**: 두 조 링크의 y-오프셋을
바깥쪽으로 대칭 확장(`extra_offset`, 원래 `±0.00345`에 가산) — 조 간격을
넓혀 자기교차 자체를 없애면 build 성공. 이진 탐색 결과:
- 편측 20~29mm 추가 오프셋(총 간격 ~46.9~64.9mm) → 전부 FAIL(여전히 crash)
- 편측 30mm 추가 오프셋(총 간격 ~66.9mm) → OK(build 성공)
- 29~30mm 사이 정밀 임계값은 미확정(세밀 이진 탐색이 도구 오류로 중단됨,
  다음 세션에서 이어서 진행 예정)

**현재 상태**: `석션V1.xml`의 조 링크 `pos`는 아직 원래 값
(`0.00345`/`-0.00345`) 그대로다 — 조 간격을 넓히는 실제 수정은 다음
작업에서 진행. 이 문제가 풀리기 전까지 고정장치+회수장치2+석션V1을 모두
포함하는 `full_workflow.py`의 최종 영상 재생성은 보류 상태.

### 조합 10 — 파우더 담기: MPM(과립) + {FEM/PBD}(봉투) + Legacy 커플러 **부분 해결**
(2026-07-28~30)

(`Powder_flip_test/powder_containment_test.py`)

**목표**: 회수장치로 파우더를 봉투에 담는 실험(§8 목표)의 사전 단계로, 로봇팔/
회수장치는 배제하고 "MPM.Sand 파우더가 봉투 내부 공간에 실제로 들어가는가"만
격리 검증. `BAG_BACKEND`(pbd/fem) × `TEST_MODE`(pour/rigid_probe/legacy_sanity)
env var로 조합 전환.

**시도 1 — FEM.Cloth(봉투) + MPM.Sand, `fem_mpm` 커플러 → build crash**
(`_fem_run1.log`, BAG_BACKEND=fem). `scene.build()` 중
`QuadrantsSyntaxError: ... Unsupported node "Raise"`
(`FEM/base.py:_update_stress_noop`에서 발생). 원인은 커플러 옵션이 아니라
**재질 자체의 설계 제약**이었다 — `genesis/engine/materials/FEM/cloth.py`의
`Cloth` 클래스 docstring에 "Only works with IPCCoupler enabled"라고 명시돼
있고, `update_stress`를 오버라이드하지 않는다. Legacy 커플러는 명시적
substep(`fem_solver.compute_vel` → `_mats_update_stress[mat_idx]` dispatch)를
타는데 Cloth는 이 경로용 함수가 없어 base의 noop(raise)이 그대로 호출된다 —
Cloth는 애초에 IPC 전용 설계라 Legacy와는 재질 수준에서 호환 불가.
(Genesis 공식 GitHub `examples/coupling/`에도 fem_mpm 조합 예제는 없음 —
cloth_on_rigid / cloth_attached_to_rigid / fem_cube_linked_with_arm /
sand_wheel / flush_cubes / cut_dragon / water_wheel / sph_mpm / sph_rigid /
grasp_soft_cube 전부 rigid↔fem 또는 rigid↔mpm 조합뿐.)

**시도 2 — FEM.Elastic 평판 + MPM.Sand, `fem_mpm` 커플러(TEST_MODE=legacy_sanity)
→ build 성공, 그러나 커플링 완전 무반응**. Cloth 대신 TetGen이 안전하게
다루는 두께 있는 Box(FEM.Elastic)를 얹어 `fem_mpm` 코드 경로 자체가 동작하는지
격리 검증. `scene.build()`는 성공했지만, 파우더 94.5~94.6%가 평판을 완전히
무시하고 자유낙하 — 낙하한 입자의 `min_z`가 정확히
`MPM_LOWER.z + 3*dx = -0.03 + 3*(1/128) ≈ -0.0066`(MPM 도메인의 3*dx
세이프티 패딩을 뺀 유효 바닥)로, 평판 두께를 6mm→30mm(5배)로 늘려도 **토씨 하나
안 틀리고 동일** — "판이 얇아서 새는" 게 아니라 **커플링 자체가 전혀 발동하지
않고 있음**을 의미. 유력한 원인: `gs.morphs.Box`를 TetGen 기본 설정으로
사면체화하면 극도로 거친 메시(정점 9개, tet 12개 — 사실상 박스 모서리
8개뿐)가 나오는데, `fem_mpm` 커플링은 FEM 표면 정점 단위로 주변 MPM 그리드를
보정하는 방식이라(§조합9후속4 인접 섹션 아님, `legacy_coupler.py` 581~635줄)
평판 중앙처럼 정점이 없는 넓은 영역은 전혀 보정되지 않는 것으로 추정 —
확정 검증은 보류.

**시도 3 — PBD.Cloth(실제 봉투 메시) + MPM.Sand, `mpm_pbd` 커플러(TEST_MODE=pour,
기본 모드) → 부분 작동, 경계선상 누출**. `_pbd_run4_openmouth.log`: 누출
141/8523(1.7%) → CONTAINED. `_pbd_run5_taper.log`: 544/8477(6.4%) → LEAK.
재현 실행(2026-07-30): 407/8534(4.8%) → CONTAINED(임계값 5% 바로 아래). 세 번
모두 실제로 새는 입자는 시도 2와 동일하게 `min_z≈-0.0066`(도메인 유효
바닥)까지 정확히 떨어짐.

**원인 분석(사용자 관찰 — "컨택은 발생하는 것 같은데 내부로 못 들어가고 상단에
쌓인다" — 코드 확인으로 확정, 2026-07-30)**: `legacy_coupler.py` 452~498줄의
`mpm_pbd` 커플링은 **표면/법선 개념이 전혀 없는 순수 근접-기반 속도평균
스킴**이다 — MPM 그리드 노드가 PBD 입자(=봉투 정점) 중 `|Δpos|_inf <
mpm_solver.dx*0.5` 안에 있는 것들을 찾아 그 속도의 평균으로 강제 스냅하고,
반작용 momentum을 그 정점들에 되돌려준다. 안/밖 구분이나 표면 방향 판정이
전혀 없다(`fem_mpm`은 최소한 signed-distance+normal 기반이라 이 문제가 덜함,
단 시도2처럼 메시가 너무 거칠면 그마저도 무용지물). 수치 확인:
봉투 STL(`Samplebag_seal_pouch3.stl`, 4512 vertices, 64×90×6mm)의 실측 정점
간격은 약 1.7~2mm인데, 이 커플링의 포착 반경은 `dx*0.5 ≈ 3.9mm`(그리드
dx=7.8mm 기준) — 정점 간격이 포착 반경보다 훨씬 촘촘해 입구 테두리를 따라
포착 영역이 전부 겹쳐서, **실제로는 뚜껑 없는 열린 튜브 구멍인데도 입구
평면 전체가 방향 무관 "속도-감쇠 커튼"으로 작동**한다. 낟알이 입구를
향해 내려가도 이 커튼 근처에서 속도가 정점 속도(≈0)로 스냅되어 감속·정지하고,
뒤따르는 낟알이 이미 멈춘 낟알 위에 일반 MPM 입자-입자 역학으로 계속
쌓인다(관찰된 "상단에 쌓임"과 정확히 일치). 소수(1~6%)가 새는 건 더미가
쌓이며 정점 배치가 국소적으로 밀리거나 포착망의 우연한 틈으로 빠지는
경우로 추정.

**§조합5/6/9(FEM 정제+FEM 봉투+IPC)가 성공했던 이유와의 결정적 차이**: 그
조합은 IPC의 CCD(연속충돌감지) 기반이라 실제 메시 표면에 대해 진짜
geometric 침투 판정을 하고, 무엇보다 정제가 **하나의 일관된 물체를 IK로
미리 계산한 경로를 따라 입구를 정확히 겨냥해** 내려보낸 것이었다 — "다수의
자유낙하 입자가 좁은 틈을 확률적으로 찾아 들어가야 하는" 지금 상황과
근본적으로 다른 문제다. `mpm_pbd`는 방향 정보가 없는 구조적 한계상 다수
입자 스트림 담기 용도로는 부적합.

**결론**: FEM.Cloth+MPM(재질 자체 비호환) / FEM.Elastic+MPM(무반응) 둘 다
탈락. `PBD.Cloth+MPM`만 실질적으로 작동하나 누출률이 임계값(5%) 바로 근처를
오르내려(1.7%~6.4%) 안정적이라 보기 어렵다. 커플러 옵션은 필요한 것 하나만
켜고 나머지 전부 False로 끄는 최적화(계산비용 절감) 적용 완료. 다음 단계
후보: 마우스 테이퍼 형상 조정으로 누출을 임계값 아래로 확실히 낮추거나,
이 파우더 담기 용도는 이미 검증된 IPC 커플러(정제처럼 "담긴 알갱이 여러 개
= 작은 FEM.Elastic 엔티티"로 표현, `main_ipc()` 함수 참고)로 가는 방안.

### 조합 11 — pyuipc 네이티브 Particle 커플러 자작 + IPC 바닥면 위치 버그 발견 (2026-07-31)

(`Powder_flip_test/ipc_grain_coupler.py`, `Powder_flip_test/sand_wheel_repro.py`)

**배경**: 조합10에서 Legacy 계열(`fem_mpm`/`mpm_pbd`)이 전부 봉투 담기에 부적합함을
확인한 뒤, "Genesis의 IPC 커플러는 FEM-Rigid만 알고 다른 재질 커플링은 아예
없는데, 그 밑단의 pyuipc(libuipc) 자체는 더 다양한 재질을 지원하지 않느냐"는
질문에서 출발. 실제로 `uipc.constitution.Particle`("point-mass simulation")
+ `uipc.geometry.pointcloud()`(dim()==0 SimplicialComplex)가 존재하고,
`Particle.apply_to(sc, mass_density, thickness)`로 접촉용 "두께"(유효 반경)를
가진 점질량을 만들 수 있으며, libuipc의 GUI/SceneVisitor 코드도 pointcloud를
tetmesh(FEM 체적)/trimesh(FEM.Cloth)/linemesh(rod)와 나란히 1급 지오메트리로
다룬다 — 즉 FEM.Cloth와 **같은 IPC Scene 안에서 동일한 CCD 기반 접촉**으로
알갱이를 표현할 길이 있었다.

**구현**: Genesis 설치 패키지를 직접 고치지 않고, `genesis.engine.couplers.
ipc_coupler.coupler.IPCCoupler`를 상속하는 `GrainIPCCoupler`를 별도 스크립트
(`ipc_grain_coupler.py`)로 작성 — `_add_fem_entities_to_ipc`/`_register_
contact_pairs`/`_retrieve_fem_states` 패턴을 그대로 따라 `_add_grain_
entities_to_ipc`(pointcloud+Particle 등록)/오버라이드한 `_register_contact_
pairs`(알갱이-cloth/rigid/ground/알갱이끼리 접촉쌍 추가)/`_retrieve_grain_
states`(dim()==0 지오메트리 상태 회수)를 추가했다. Genesis는 `coupler_options`
타입만 보고 내부적으로 `IPCCoupler`를 직접 생성하므로(`simulator.py`), `gs.
Scene()` 생성 직후 `scene.build()` 전에 `scene._sim._coupler`를 이 서브클래스
인스턴스로 바꿔치기하는 방식으로 끼워 넣었다.

**시행착오 2건(둘 다 코드 레벨에서 원인 확정)**:
1. 첫 sanity 테스트(강체 바닥+알갱이)가 crash 없이 통과했지만 알갱이가
   바닥을 완전히 무시하고 자유낙하 — 원인은 다른 모든 지오메트리(FEM 314줄,
   Rigid 462줄)와 달리 pointcloud에 `uipc.geometry.label_surface(mesh)`를
   빠뜨렸기 때문(직접 재현: `label_surface` 호출 전엔 `is_surf` 속성이
   없어 접촉 판정 대상에서 아예 빠짐, 호출 후엔 정상). 추가 후 알갱이가
   정확히 반지름 위치에서 멈춤 확인.
2. 영상에 알갱이가 하나도 안 보임 — 알갱이는 Genesis 엔티티가 아니라
   `scene.draw_debug_spheres()`로 그린 마커뿐이라, `genesis/vis/
   rasterizer.py:76`의 `skip_markers = not camera.debug`에 걸려 기본
   카메라 렌더에서 통째로 스킵됐다. `scene.add_camera(..., debug=True)`로
   해결(직접 최소 재현으로 확인).

**성공 신호**: 실제 봉투(FEM.Cloth)+알갱이(Particle) 조합에서 낙하한 알갱이
대부분이 봉투 내부 바닥 쪽에 정상적으로 쌓였다(영상 프레임으로 시각 확인) —
mpm_pbd처럼 입구 위에 쌓이는 현상과 다름. 다만 누출률이 3.3~11.7% 사이에서
런마다 흔들려(GPU 부동소수점 비결정성 추정) 조합10과 마찬가지로 임계값
근처를 오갔다.

**별개의 근본 원인 추가 발견(사용자 관찰 "봉투가 바닥까지 늘어나며 찢어지는
것 같다"에서 출발)**: `BAG_CONSTRAINT_MODE` 변인통제(바닥/옆면/입구 고정을
"기존대로"/"입구 벌림 생략하고 원위치만 고정"/"아예 제약 없음" 세 가지로
전환 가능하게 계측 추가)로 확인한 결과, **제약(vertex constraint)이 원인이
아니었다** — 제약을 하나도 안 걸어도 봉투 전체가 똑같이 붕괴했다. 대신
직접 속성 조회로 확인한 진짜 원인: `genesis/engine/couplers/ipc_coupler/
coupler.py`의 `_add_rigid_geoms_to_ipc()`가 바닥면(Plane) 높이를
`height = np.dot(geom.init_pos, normal)`로 계산하는데, `geom.init_pos`는
링크-로컬 좌표라 항상 `[0,0,0]`이다(링크의 실제 월드 위치 `link.pos`는 이
계산에 안 들어감 — 다른 일반 메시 지오메트리는 `trans_view[0] = link_T`로
월드 변환을 따로 적용하는데 Plane 전용 분기만 이 단계가 빠짐). 결과적으로
**`gs.morphs.Plane(pos=...)`에 뭘 넣든 IPC 안에서는 항상 world z=0에 바닥이
생긴다** — 봉투(바닥이 로컬로 약 z=0.065)가 진짜 바닥(z=0)까지 6.5cm를
허공에서 그냥 떨어져 늘어진 뒤에야 멈춘 것이었다. 이건 mpm_pbd/fem_mpm/
Particle 등 커플링 방식과 완전히 무관하게 조합10·11 실험 전체에 공통으로
영향을 준 원인이었다.

**Genesis 버전 업그레이드로 재확인(2026-07-31)**: 사용자가 "이 버그가 Genesis
1.2.3에서 이미 발견됐다"고 언급해 `genesis-world` 1.1.0 → 1.3.1(quadrants
0.8.0 → 1.2.0 동반)로 업그레이드했으나, **`coupler.py:424`의 해당 줄은
1.3.1에서도 완전히 동일**하고 재현 실험 결과도 소수점 4자리까지 이전과
일치 — 이 특정 버그는 아직 안 고쳐진 것으로 확인. 업그레이드 과정에서
`Camera.start_recording`/`stop_recording` API가 바뀐 것도 확인(파일명/fps가
`start_recording`으로 이동, `stop_recording()`은 인자 없음) — 스크립트
수정 완료.

> **[추가 발견 2026-08-03] 이 업그레이드가 `full_workflow.py` FEM 경로를
> 깨뜨려 놨었다 (§13-6 rigid 모드 작업 중 회귀 검증에서 발견).** 두 군데:
> (1) 위 녹화 API 변경이 `full_workflow.py`에는 반영이 안 돼 있어
> `stop_recording(save_to_filename=...)` 에서 `TypeError` — **수정 완료**.
> (2) `utills/primitive_tablet_generator.py`의 `add_analytic_fem_entity`
> (TetGen 우회용 `mesh_to_elements` 몽키패치)가 **0바이트 더미 STL**을
> 만들어 두고 `gs.morphs.Mesh(file=...)`의 파일 존재 검사만 통과시키는
> 구조인데, Genesis 1.3.1의 `FEMEntity.sample()`이 새로 추가한
> `gs.Mesh.from_morph_surface(morph, surface)`(**시각 geom**을 morph 파일에서
> 직접 로드)가 그 빈 파일을 읽어 빈 리스트를 돌려주고
> `merge_submeshes` 가 `ValueError: need at least one array to concatenate`
> 로 죽는다. 1.1.0에서는 sample()이 패치된 `mesh_to_elements` 경로만 탔기
> 때문에 빈 더미로도 통과했었다. **마지막 성공 실행(2026-07-29, b2ba7cb)
> 이후 업그레이드(07-31)가 있었고 그 사이 FEM 모드를 아무도 안 돌려서
> 지금까지 안 보였던 것.**
>
> **[정정] "더미 STL만 채우면 된다"는 진단은 불완전했다.** 실제로 실물
> 캡슐 표면 메시(watertight, 264면)를 그 자리에 써 넣고 재실행하니 한 줄
> 더 가서 **다른 지점**에서 죽었다 — `fem_entity.py:531`:
> ```
> verts, elems = eu.mesh_to_elements(surface_trimesh, tet_cfg=self.tet_cfg)
>   → primitive_tablet_generator.py:325  _orig(file, pos=pos, scale=scale, ...)
> TypeError: mesh_to_elements() got an unexpected keyword argument 'pos'
> ```
> **`mesh_to_elements` 의 시그니처가 바뀌었다**: 1.1.0 `(file, pos, scale,
> tet_cfg)` → 1.3.1 **`(mesh, tet_cfg)`**. 파일 경로가 아니라 trimesh 객체를
> 받는다. 그런데 몽키패치는 `if file in _ANALYTIC_CACHE` 로 **경로 문자열을
> 캐시 키**로 쓰므로 이제 캐시가 절대 안 맞고, 폴백 호출도 없어진 인자를
> 넘겨 죽는다. 즉 **TetGen 우회 메커니즘 자체를 1.3.1 시그니처에 맞춰 다시
> 써야 한다**(캐시 키를 morph 파일 경로가 아닌 다른 것으로 잡거나, 우회
> 지점을 `FEMEntity` 쪽으로 옮기는 등). 한 줄 수정이 아니다.
> 이 유틸은 `add_analytic_fem_entity` 를 쓰는 13개 스크립트가 공유한다
> (`FEM/fem_tablet_*.py` · `Legacy_vs_IPC/pipeline_ipc.py` ·
> `Recovery2_only/recovery2_tablet_drop.py` · `Powder_flip_test/` ·
> `scene_setup.py` · `slot_fit_check.py` 등).
>
> **단, 이건 정제(FEM.Elastic) 한정 문제다.** 봉투(`FEM.Cloth`)는 이 유틸을
> 안 쓰고 일반 `gs.morphs.Mesh` 경로라 무관하다 — 봉투+IPC 자체의 건전성은
> `SKIP_TABLET=1` 로 분리 검증했다(§13-8).
>
> **→ 해결 (2026-08-03), §13-9 참고.**

**Rigid-MPM 베이스라인 재확인**: Genesis 공식 예제 `examples/coupling/
sand_wheel.py`를 headless/GPU 환경으로 이식해 재현(`sand_wheel_repro.py`) —
모래가 각 바퀴 십자 날개에 정확히 부딪혀 튕기며 흘러내림을 영상으로 확인,
crash/관통 없음. `rigid_mpm` 커플링 자체는 안정적이라는 근거 확보 — 문제는
"양쪽 다 변형체/입자"인 조합(`mpm_pbd`, `fem_mpm`)에 국한됨을 뒷받침.

**결론 및 다음 단계**: (1) Particle 기반 접근은 방향성이 맞다(FEM.Cloth와
같은 CCD 경로를 타므로 mpm_pbd의 근본적 한계가 없음) — 다만 바닥면 버그
때문에 이번 실험만으로는 완전한 결론을 못 냄. (2) 바닥면 버그는 Genesis에
직접 보고 예정(GitHub enhancement 문의 초안 작성 완료 — Legacy 커플러의
재질 조합 확장 계획 문의에 포함). (3) 바닥면 버그를 피하려면 봉투를 실제
바닥(항상 world z=0) 바로 위에 오도록 배치하거나, 버그가 고쳐진 이후 버전을
기다려야 함.

### 조합 12 — 접촉/커플링 pair 제어 수단과 MPM 입자 크기의 실질 한계 · **코드 레벨 조사(실행 검증 없음)** (2026-08-11)

**성격 먼저**: 이건 새로 돌린 실험이 아니라 **Genesis 1.3.1 설치본 소스를 직접
읽어 확인한 조사 기록**이다. 아래 표의 비용 수치는 코드의 공식에서 유도한
값이지 실측이 아니다(그렇게 표시해 둠). 조합 10·11 을 재시도하거나 새 조합을
설계하기 전에 "무엇이 되고 무엇이 안 되는가"를 먼저 못박아 두려고 남긴다.

#### 12-A. MPM 입자 크기는 자유롭게 못 줄인다 — 실질 제약은 격자 dx

`MPMOptions.particle_size` 자체엔 하한이 **없다**(`PositiveFloat`, 그냥 >0 —
`options/solvers.py:645`). 안 주면 자동으로 잡히는데, 그 공식이 핵심이다:

```
particle_size = 0.01 * 64 / grid_density     # solvers.py:663-665
dx            = 1 / grid_density             # mpm_solver.py:53
→ particle_size = 0.64 * dx  (항상)
```

즉 Genesis 가 권장하는 건 절대 크기가 아니라 **입자경 ≈ 0.64 × 격자간격**이라는
*비율*이다. 그리고 `particle_size` 가 솔버에서 실제로 하는 일은 딱 둘뿐이다
(전수 확인) — **입자 질량**(`volume = particle_size**3`, `mpm_solver.py:48-50`)과
**샘플링 간격**(`particle_entity.py:257-270`). **충돌 반경으로는 어디서도 안
쓰인다.** 접촉·커플링 판정은 전부 `mpm_solver.dx` 기준이다
(`legacy_coupler.py:386,433,574` — 574줄엔 `NOTE: use dx as minimal unit for
collision` 이라고 명시돼 있다).

**→ 조합 10 에 대한 함의**: 알갱이만 줄이는 건 누출 대책이 될 수 없다. 조합 10
에서 "포착 반경 `dx*0.5 ≈ 3.9mm`" 로 이미 짚은 것과 같은 얘기인데, 한 단계 더
나아가면 — **dx=7.8mm 격자로는 6mm 입구 슬롯이 셀 하나 안에 통째로 들어가
애초에 표현되지 않는다.** 슬롯을 격자에 보이게 하려면 dx ≲ 2mm, 즉
`grid_density ≥ 512` 가 필요하다.

**그 비용**(Powder_flip_test 도메인 0.24×0.24×0.33m 기준, 코드 공식에서 유도 —
셀 수는 `mpm_solver.py:55-57`, substep 하한은 `:238` 의 `2e-2 * dx`):

| grid_density | dx | 기본 입자경 | 격자 셀 | 필요 substeps(DT=1e-3) | 상대 비용 |
| --- | --- | --- | --- | --- | --- |
| 128 (조합10 사용값) | 7.8mm | 5mm | 41k | 10 ✓ | 1× |
| 256 | 3.9mm | 2.5mm | 341k | 13 | ~16× |
| **512** | **2.0mm** | **1.25mm** | **2.6M** | **26** | **~256×** |
| 1024 | 0.98mm | 0.63mm | 20.7M (~0.7GB) | 52 | ~4,100× |
| 2048 | 0.49mm | 0.31mm | 164M (~5.9GB) | 103 | ~66,000× |
| ~3700 | 0.27mm | — | 1e9 초과 → **raise** | — | — |

비용이 **1/dx⁴** 로 간다(격자 세제곱 × substep 수). 하드 캡은 셀 총량 1e9
(`mpm_solver.py:59-63`)지만 메모리가 훨씬 먼저 죽는다. 이 도메인의 현실선은
**grid_density 1024, dx≈1mm, 알갱이≈0.6mm** 정도로 보이고, 그 아래로 가려면
도메인을 더 좁혀야 한다.

**결론적 관점**: 실제 의약품 파우더 입도(10~100µm)를 맞추는 건 목표가 아니고
가능하지도 않다(100µm → 이 도메인에서 1.9e10 셀, 하드 캡 초과). MPM 입자는
물리적 알갱이가 아니라 **연속체의 적분점**이고, 과립 거동은
`MPM.Sand` 의 Drucker-Prager 소성모델(`friction_angle`)이 담당한다.
**`particle_size` 는 물성이 아니라 이산화 파라미터다.**

#### 12-B. 접촉/커플링을 특정 pair 만 켤 수 있는가 — 축마다 다르다

| 축 | 수단 | 필터 단위 | 코드 |
| --- | --- | --- | --- |
| Rigid ↔ Rigid | `contype`/`conaffinity` 비트마스크 | **geom** | `collider.py:408-410` |
| Rigid ↔ MPM/SPH/PBD/FEM | `material.needs_coup` | **geom** | `legacy_coupler.py:143,479,680,715` |
| 〃 (링크로 더 좁히기) | `material.coup_links` | **link** | `rigid_entity.py:2542` |
| 솔버 종류 간 | `LegacyCouplerOptions` 7 플래그 | 솔버 **종류** | `options/solvers.py:104-110` |
| IPC 커플러 | `coup_type` / `enable_coup_collision` / `coup_collision_links` | 엔티티·링크 | `materials/rigid.py:62-85` |
| **MPM ↔ PBD/SPH/FEM** | **없음** | — | 근접판정에 geom 개념 자체가 없음 |
| **MPM 입자끼리** | **없음** | — | 격자 매개라 "pair" 가 존재하지 않음 |

**핵심 — Rigid↔MPM 은 geom 단위로 끌 수 있다.** 커플링 커널이 모든 rigid geom
을 돌면서 geom 별로 플래그를 본다:

```python
for i_p, i_b in qd.ndrange(mpm_solver.n_particles, ...):   # legacy_coupler.py:476
    for i_g in range(rigid_solver.n_geoms):
        if geoms_info.needs_coup[i_g]:                      # ← 여기 (:479)
            sdf_normal = sdf.sdf_func_normal_world(...)
```

이 플래그는 `needs_coup = material.needs_coup and (coup_links is None or
link.name in coup_links)` 로 geom 마다 채워진다(`rigid_entity.py:2542`). 그래서
`gs.materials.Rigid(needs_coup=False)` 인 강체는 루프에서 스킵된다 — "body1 만
모래와 접촉, body2 는 무시" 가 성립한다. **건너뛰는 게 입자×geom 마다 도는 SDF
질의라 계산량도 같이 준다.**

**주의 4가지(전부 코드 확인)**:
1. `needs_coup=False` 는 **모든 파티클 솔버**와의 접촉을 한꺼번에 끈다 —
   MPM(`:479`)·SPH(`:680`)·PBD(`:715`)·FEM 이 전부 같은 플래그 하나를 본다.
   "MPM 하고만 끊고 PBD 봉투와는 닿게" 는 불가능.
2. `needs_coup` 은 **Rigid↔Rigid 충돌엔 영향이 없다.** 그쪽은 collider 의
   contype/conaffinity 로 완전히 별개 경로다(보통 이게 원하는 동작).
3. `contype=0 && conaffinity=0` 으로 두면 그 geom 은 **비주얼 전용으로 강등**
   된다(`utils/collision.py:18-20`). 최소 한 비트는 남겨야 한다.
4. MJCF/USD 로 로드한 엔티티는 마스크가 **엔티티 내부에서만** 유효하다
   (`rigid_entity.py:2625-2629`, `is_local_collision_mask`). 엔티티끼리 끄려면
   Mesh/URDF morph 를 쓰거나 `needs_coup` 쪽을 쓴다.

**끄고 싶은 pair 가 많으면 손으로 비트를 짜지 말 것** — `genesis/utils/
collision.py:6` 의 `solve_contype_conaffinity(n, invalid_pairs)` 가 z3 로
마스크를 역산해준다(MJCF `<contact><exclude>` / USD FilteredPairsAPI 임포터가
쓰는 바로 그 함수). 풀 수 없으면 `None` 을 돌려준다.

**§13(전 Rigid 모드)에 대한 함의**: rigid 공정 검증 모드에서 불필요한 geom
pair 를 contype/conaffinity 로 끄면 broadphase 후보가 줄어 추가 가속 여지가
있다 — 조합 10 결론에 적힌 "커플러 옵션은 필요한 것만 켠다" 최적화의 geom 판
확장. **미적용/미측정.**

---

## 10. 피드백

**한 박사님:**
- 중요한 것은 화학자들이 어떤 것을 얻을 수 있을까?
- DT를 통해서 보고 싶은 건, **이걸 얼마만큼 때려야 부서지는가?**이고, 이걸 물리적으로 묘사할 수 있어야 함. 그게 안 된다면 적어도 **Data-driven**하게 풀 수는 있어야 함.
- 그게 안 되면 큰 의미는 없어 보임.

**남은 과제:**
- 피로파손 – 압력.
- 알약의 경도와 피로파손을 어떻게 풀 수 있을까.

---

## 11. Visualization options

**고정장치/M0609 렌더링 스파이크 아티팩트 — 해결(2026-07-21)**: `smooth=True`(기본값)에서 볼트머리 등 작은 디테일 부근에 별모양 스파이크 발생, `smooth=False`는 형상 자체가 깨짐, `decimate`는 무영향 — 같은 STL을 MuJoCo 자체 렌더러로 찍으면 정상이라 Genesis 쪽 문제로 특정. 원인: `pyrender/mesh.py`의 smooth 셰이딩이 `trimesh.vertex_normals`를 크리스 각도 구분 없이 그대로 씀(hard-edge 개념 없음), MuJoCo는 날카로운 모서리에서 정점을 분리해 평균을 안 냄. 해결: `trimesh` `mesh.smooth_shaded`(크리스 각도 기준 정점 분리 후 스무싱)로 재수출한 `_ss.obj`를 시각 전용 geom에만 적용(충돌 geom은 원본 STL 유지, non-watertight라 충돌엔 부적합) — 고정장치 12개, M0609 링크 10개 메시에 적용 완료.

---

## 12. 현 시점 우선순위 및 미해결 문제 (2026-08-08)

### 12-1. Digital Twin 우선순위

> **[갱신 2026-08-14, 사용자 지시] 전(全) Rigid 는 더 이상 1순위가 아니다.**
> 목표 조합이 **Samplebag(FEM.Cloth) + Tablet(FEM.Elastic, 3D) + 그 외 전부 Rigid**
> 로 바뀌었다 — 즉 `full_workflow.py` 의 현재 구성이 곧 타깃이고,
> `full_workflow_rigid.py`(§13)는 산출물이 아니라 **저비용 기하 검증 하네스**로
> 격하된다(빌드+실행 90초 vs 16분). 아래 원문은 2026-08-08 시점 기록으로 남겨둔다.
> 현재 집중 과제는 §14 — 봉투를 파지해 슬롯에 넣는 공정.

지금 시점에서의 Digital Twin 우선순위는, **"모든 바디를 Rigid body로 사용하고, 공정 검증을 하는 게 1순위이다."**

이 Digital Twin이 가지는 의미는 실험자가 별도의 Teaching을 하거나 이동정책을 수정하지 않고, 이 모듈을 사용할 때 Path planning과 다양한 모터의 제어전략을 재사용 가능하다는 점과 공정에 대한 검증이 가능하다는 결론이 나온다. 샘플백과 정제를 FEM 재료로 묘사하게 되면, 너무 큰 계산비용이 나오므로 이런 식의 검증을 하는 것이다.

내가 할 실제 실험은 정제가 이렇게 했을 때, powder 사이즈 분포가 어떻게 되는지, 그리고 DigitalTwin을 통해서 옮긴 공정이 얼마만큼 실제 세계와 맞는지 총 두 가지 정도의 플랏을 그리면 될 것 같다.

### 12-2. 2026.08.08 시점에서의 문제

현재 회수장치의 폐루프 문제가 안 풀린다. 이미 Crusher에서 하나의 링크를 두 개로 나눠서 해결한 문제이나, 생각보다는 잘 안 풀리는 듯하다. Pulley와 Crank가 연결되어 있는데, 내 시스템에서 "Crank"로 Description되어 있는 링크(현재 시스템에서는 Crank의 역할을 Pulley가 수행하고 있음)는 사실 기구적인 관점에서는 connecting rod이다. 이 커넥팅로드를 두 개로 분할하고 모든 자유도를 weld로 빼앗아야 할지 의문이다. 혹은 M_top을 두 개의 링크로 분할하는 방식이다. 추가로 지금 방식은 Connecting rod를 F_top에 연결지어 놓고, 한쪽을 Equality constraint의 weld가 아닌, connect로 연결하여 3자유도를 살릴 생각이었으나, connect 자체가 가지고 갈 수 있는 자유도가 planar joint(2 Prismatic + 1 Rotate)가 아니라 3 Prismatic Joint인 것으로 알고 있다. 쉽게 생각하면 칠판에서의 거동이 필요한 것인데, 실제로는 볼조인트가 가지는 거동만이 존재하는 것이다. 따라서 이를 해결할 방법을 생각해봐야 한다.

### 12-3. §12-2의 원인 규명 — **Fusion 조인트를 0으로 초기화하지 않고 export** (2026-08-12)

**결론부터: 폐루프 모델링 방식의 문제가 아니었다.** `connect`도, 링크 분할도
건드릴 필요가 없었다. **Fusion에서 크랭크를 사점(180°)으로 돌려놓은 상태로
export**해서 조인트 원점이 그 자세로 굳어 들어간 것이 원인이다.

#### 증상

MJCF를 MuJoCo에 물려 보니 풀리와 커넥팅로드가 서로 반대로 도는 것처럼 보였다.
접합점에 마커를 찍고 메시 정점을 원 피팅해 실제 핀 구멍을 찾아 비교했다.

```
축 중심 (회전_31)                x = -0.074015
로드 보어 A (메시 실측, r=3.4mm)  x = -0.091515      편심 -17.5 mm
로드 보어 B (메시 실측, r=3.4mm)  x = -0.031515      보어 간 60.00 mm
M_Top 아랫면 구멍 (r=2.1mm)       x = -0.031515      보어 B와 0.00 mm

XML 의 회전_29 (크랭크핀)         x = -0.056515      +35.0 mm 어긋남
XML 의 equality anchor            x = +0.003485      +35.0 mm 어긋남
```

두 조인트가 **같은 방향으로 정확히 같은 크기(35.0 mm = 2×편심)** 어긋났다.

#### 사점 export 가설의 검증

로드 보어 A를 축 중심 기준으로 180° 회전시키면:

```
rot(보어 A, 축중심, 180°)  = [-0.056515, 0.05922]
XML 의 회전_29             = [-0.056515, 0.05922]     차이 0.0000 mm

사점에서 로드 타단(= 핀 + 60mm) = [+0.003485, 0.05922]
XML 의 anchor                    = [+0.003485, 0.05922]  차이 0.0000 mm
```

**두 점 모두 0.0000 mm로 일치.** 사점에서는 크랭크와 로드가 일직선이라 y가
변하지 않는데, 실제로 오차가 x에만 나타났다는 점도 이 가설과 맞는다.

#### 왜 물리가 망가지나

로드의 반대쪽 끝은 M_Top 구멍에 물려 있으므로 위치가 고정이다. 핀만 35mm
밀리면 로드의 **유효 길이**가 바뀐다.

```
정상 : |-0.031515 - (-0.091515)| = 60.0 mm     (= 실제 보어 간격)
export: |-0.031515 - (-0.056515)| = 25.0 mm     (35mm 짧아짐)
```

커넥팅로드는 회전하지 않고 요동하며, 요동폭은 `2·asin(r/L)`이다.

| | 로드 길이 | L/r | 요동폭(실측) | 이론값 |
|---|---|---|---|---|
| 사점 export 상태 | 25.71 mm | 1.47 | 89.1° | 85.8° |
| 정상 | 60.30 mm | 3.45 | 33.9° | 33.7° |

L/r ≈ 1.5면 로드가 크랭크와 길이가 비슷해져 ±45°씩 홱홱 젖혀진다. 풀리가 한
방향으로 도는 동안 로드가 반대로 넘어가는 구간이 생기고, 그게 "반대로 도는
것처럼" 보인 정체였다.

#### 이 오류가 오래 안 잡힌 이유

**행정 = 2 × 편심**이라 편심의 방향이 반대여도 행정은 똑같이 35 mm가 나온다.
행정만 확인하면 정상으로 보인다. 실제로 잘못된 상태와 정상 상태 모두
슬라이더 행정이 35.01 mm로 측정됐다. **로드 길이를 같이 확인해야 잡힌다.**

#### 파급 — Slide joint와 리밋

`슬라이더_35`도 기하에서 126 mm 떨어진 곳에 있는데, 이건 **무해**하다. slide
조인트는 anchor 위치가 운동에 영향을 주지 않고, 이 익스포터 규약상
`<geom pos> = -링크원점`이 정확히 상쇄하기 때문이다.

문제가 되는 건 **리밋**이다. MuJoCo의 `qpos=0`은 export 시점의 자세이므로,
Fusion 조인트 zero가 아닌 자세로 내보내면 Fusion이 보고하는 리밋 값이 그
변위만큼 통째로 밀린다. 이번 모델은 리밋이 전부 꺼져 있어 드러나지 않았을
뿐이다. 회전 조인트도 마찬가지다.

#### 규칙

> **fusion2xml(및 fusion2urdf)로 export하기 전에 모든 조인트를 0으로
> 초기화한다.** Fusion에서 `Joints` 폴더 우클릭 → 모든 조인트의 애니메이션/
> 드래그 자세를 원위치로 되돌린 뒤 export.

이걸 지키지 않으면 조인트 원점·리밋이 그 자세 기준으로 굳어 들어가고,
메시는 별개로 배치되어 **기구 치수가 조용히 틀어진다**. 로딩도 되고
움직이기까지 하므로 발견이 늦다.

#### §12-2에 대한 답

`connect`가 남기는 자유도가 볼조인트 3회전이라는 지적은 맞지만, 이 기구에서는
**트리 쪽이 이미 면외 자유도를 막고 있어 문제가 되지 않는다** — 모든 힌지축이
z로 평행하고 슬라이더가 x이므로 평면성이 보장된다. 링크를 분할하거나 weld로
바꿀 필요가 없었다. 수정 후 실측: 슬라이더 행정 35.01 mm(이론 2×17.5),
로드 요동폭 33.9°(이론 33.7°), 무부하 드리프트 0.

남는 건 `connect`가 소프트 제약이라 접촉력이 걸리면 핀이 벌어진다는 점이다
(토크 0.10/0.20/0.50 N·m에서 388/749/1817 µm). 물림력을 정량적으로 볼
단계에서는 `solref`/`solimp`를 단단하게 잡고 `timestep`을 줄여야 한다.

---

## 13. 공정 검증 전략 — 전(全) Rigid 선행 모드

*(2026-08-03, 사용자 전략 지시)*

### 13-1. 왜 바꾸나 — 재료 충실도와 공정 검증의 분리

§9 조합9까지 오면서 `full_workflow.py`(정제 낙하 → 봉투 파지 → 리프트 → 슬롯
삽입)는 물리적으로는 성공했지만 **한 번 돌리는 비용이 크다**. 당시 실측:
IPC + FEM.Cloth(봉투) + FEM.Elastic(정제) 조합에서 빌드 + 전체 스텝 + 인코딩
합쳐 **16분 18초**(`IPC_D_HAT=1e-4` 기준), `d_hat`을 5e-5로 낮추면 build 내부
warm-start 솔브만 **30분+** 로 폭증(§full_workflow.py `IPC_D_HAT` 주석).
기구 배치를 1mm 옮길 때마다, IK 타깃을 한 번 바꿀 때마다 이 비용을 낸다.

> **[정정 2026-08-14] 이 "16분 18초"는 재현되지 않는다.** 같은 씬·같은 2230
> 스텝을 Genesis 1.3.3 에서 계측 코드로 다시 재니 **273.5초**다(§13-10).
> 그 값은 스크립트가 아니라 사람이 벽시계로 잰 것이라 Genesis 버전도 캐시
> 상태도 기록이 없었다. 전(全) Rigid 로 가는 **동기 자체는 그대로 유효**하지만
> (스텝당 117.3ms → 34.2ms), 배율은 12배가 아니라 3.2배다. `d_hat=5e-5` 의
> "30분+" 도 같은 출처라 아직 미검증이다.

그런데 지금 반복적으로 검증하고 있는 것들(§조합9 후속 1~4의 대부분)은
**재료 물성과 무관한 항목**이다:

- 기구(고정장치/회수장치2/석션V1/Crusher)의 배치와 상호 간섭
- IK 도달성 — 목표 자세가 조인트 한계·특이점 안에 있는가
- 매니퓰레이터 조작정책 — 어느 순서로, 어느 경로로, 어느 속도로 움직이는가
- 그리퍼-Crusher 충돌 여유(핑거 폭 vs gap 12mm 같은 기하 제약)

**전략: 공정 검증(rigid)과 재료 충실도(FEM/IPC)를 두 단계로 분리한다.**

| 단계 | 재료 | 검증 대상 | 비용 |
| --- | --- | --- | --- |
| 1. 공정 검증 | **전부 rigid** | 배치·IK 도달성·조작정책·간섭·시퀀스 | 낮음(IPC/FEM 없음) |
| 2. 재료 검증 | FEM.Cloth + FEM.Elastic + IPC | 파지 성립성·봉투 변형·정제 응력 | 높음 |

1단계에서 확정된 궤적·자세·타이밍은 **그대로** 2단계로 넘어간다. 이를 위해
`DT`와 페이즈 스텝수(`N_*`)를 rigid 모드에서도 동일하게 유지한다 — 같은
스텝에서 같은 관절각을 명령하므로 두 모드의 궤적이 1:1 대응한다.

### 13-2. 이 전략이 트윈에서 갖는 의미

기존 산업용 로봇 도입 절차는 (a) 기구를 물리적으로 배치하고 → (b) 사람이
로봇을 직접 티칭하고 → (c) 배치가 바뀌면 티칭을 다시 하는 순환이다. 위
2단 파이프라인이 성립하면 **(a)~(c) 전부를 시뮬레이션 안에서 끝내고, 검증된
결과를 그대로 실기에 올린다**:

- 별도의 로봇 티칭 절차가 필요 없다 — 조작정책이 시뮬에서 나온다.
- 기구 배치가 바뀔 때마다 사람이 다시 해야 하는 일이 없다 — 배치 상수만
  고치고 1단계를 다시 돌린다(싸므로 반복 가능).
- §2에서 Genesis를 고른 이유("Twin인 만큼 공정 시뮬레이션에 집중")와 §10
  피드백("화학자들이 무엇을 얻는가")에 직접 대응하는 축이다 — 재료 파쇄
  물리(§6~7)와는 **다른 축의 산출물**이다.

### 13-3. 봉투 애셋은 그대로 rigid 로 못 쓴다 — 실측과 프록시 설계

**실측** (`Samplebag_seal_pouch3.stl`, trimesh):

| 항목 | 값 |
| --- | --- |
| 정점 / 면 | 771 / 1504 |
| 크기(로컬) | X 64mm(폭) × Y 90mm(높이) × Z 6mm(두께), 원점 중심 |
| watertight | **False** — 경계 엣지 36개 = 입구(로컬 y=+45mm) |
| 부피 | 계산 불가(닫힌 볼륨 아님) |
| 표면적 | 12,984 mm² |
| convex hull | 정점 8 / 면 12 = **속 꽉 찬 64×90×6mm 슬래브** |

이 애셋은 §5-2에서 IPC self-intersection 을 피하려고 만든 **두께 0의 5-panel
cloth 표면 메시**다(front/back 11520mm² + 좌우 1080mm² + 바닥 384mm², 상단만
open — 면 법선 그룹으로 확인). FEM.Cloth 는 두께를 재료 파라미터
(`CLOTH_THICK=1mm`)로 받으므로 표면만 있으면 되지만, rigid body 는 성립하지
않는다:

1. **질량/관성 미정의** — 닫힌 볼륨이 없어 density 기반 추정이 불가
2. **`convexify=True` 는 입구를 막는다** — hull 이 속 찬 슬래브라 정제가
   안으로 못 들어감(drop 페이즈가 죽음)
3. **볼록분해도 무의미** — 두께 0 조각은 부피가 0이라 퇴화

**채택: 같은 치수에서 절차적으로 생성한 5-panel box primitive 프록시**
(한 MJCF body 안에 `type="box"` geom 5개 + freejoint). box 는 §6-1의 analytic
SDF primitive 라 메시 충돌보다 빠르고 터널링에도 강해 rigid 모드의 목적과
정확히 맞는다.

**벽 두께 성장 방향 — 슬롯 여유가 결정한다.** 벽에 두께 `t`를 줄 때 안/밖
어느 쪽으로 키우냐가 자유롭지 않다:

| 패널 | 성장 방향 | 근거 |
| --- | --- | --- |
| front/back (로컬 Z=두께축 → world X) | **바깥** | 안쪽으로 키우면 공동이 6→4mm 인데 정제 캡슐 지름이 정확히 4mm 라 여유 0. 바깥으로 키우면 외형 8mm 이나 슬롯 gap 12mm 대비 여전히 2mm/쪽 여유 |
| 좌우 (로컬 X=폭축 → world Y) | **안쪽** | 폭 64mm 가 슬롯 Y 여유 65mm 대비 **0.5mm/쪽**밖에 안 됨(§full_workflow.py `BAG_EULER` 주석). 바깥으로 1mm 키우면 66mm > 65mm 로 **삽입 자체가 불가능**해진다 |
| 바닥 (로컬 Y=높이축) | 바깥 | 공동 높이 90mm 보존, 슬롯 폭/두께 여유와 무관 |

**질량 일치 트릭.** 5개 박스의 총 부피는 `t × 12984mm²` = `t × A`(A = cloth
표면적)이고, FEM 봉투의 질량은 `CLOTH_RHO × CLOTH_THICK × A` 다. 따라서

```
t = CLOTH_THICK  이면  density = CLOTH_RHO × CLOTH_THICK / t = CLOTH_RHO
```

즉 **벽 두께를 `CLOTH_THICK`(1mm), density 를 `CLOTH_RHO`(200)로 두면 rigid
프록시 질량이 FEM 봉투와 자동으로 정확히 일치**한다(2.597 g). 관성도 MuJoCo
가 5개 박스에서 셸 관성으로 제대로 계산한다 — 슬래브 근사가 아니다. 상수를
따로 관리할 필요가 없어지므로 이 방식을 쓴다.

**정제**: FEM 캡슐(`make_capsule_tets_v2`, R=2mm/실린더 1mm) 대신 MJCF
`type="capsule"` geom — §조합6에서 이미 검증된 SDF primitive 경로. `density`
는 `TABLET_RHO`(1300) 그대로.

### 13-4. 구동 방식 전환 — 텔레포트 금지, PD 로

현재 `full_workflow.py`는 로봇을 매 스텝 `set_dofs_position`(하드 텔레포트)로
움직인다. IPC 모드에서 이게 통했던 건 `two_way_soft_constraint`가 반력을 자체
처리했기 때문이다(§5-2). **rigid 모드에서는 그대로 쓰면 안 된다** — §7-7이
정확히 이 경우를 다룬다: 위치 강제는 속도/가속도와 일관되지 않아 접촉
임펄스가 정상적으로 흐르지 않고, 핑거가 봉투를 뚫거나 봉투를 튕겨낸다.

→ rigid 모드에서는 팔·그리퍼 전부 **`control_dofs_position`(PD)** 로 구동하고,
로봇 재료에 `gravity_compensation=1.0`을 준다(산업용 서보의 중력보상에 대응 —
안 주면 PD 정상상태 오차로 팔이 처져 IK 목표에서 벗어난다).

두 가지 주의:
- **마찰 항이 바뀐다.** IPC 모드의 `coup_friction`은 커플러 전용이라 rigid
  모드에서는 아무 효과가 없다 — 리지드 솔버가 쓰는 `friction`을 봉투/핑거
  양쪽에 명시해야 파지가 성립한다.
- **RG2 mimic 은 여전히 수동.** MJCF 의 `<equality><joint>` 5개를 Genesis 가
  안 지키는 문제(§`m0609_rg2_v2.xml` 주석)는 rigid 모드에서도 같으므로,
  6개 핑거 DOF 에 같은 목표각을 넣는 기존 방식을 그대로 유지한다.

**봉투 자립 문제.** FEM 모드는 `set_vertex_constraints`로 봉투 형상을
붙잡아두다 `close` 직전에 풀었다. rigid 봉투는 64×8mm 바닥면에 90mm 높이로
서 있어 그대로 두면 넘어진다 — 같은 역할로 **prep~close 구간 동안 매 스텝
pose 를 고정**하고, 핑거가 이미 닫힌 뒤(`grasp` 시작)에 놓아준다(FEM 모드보다
한 페이즈 늦게 푸는 것 — 붙잡을 게 이미 있는 상태에서 풀기 위함).

### 13-5. 알려진 이슈 — Left_Wall 충돌이 꺼져 있다

`Crusher_IsaacSim_colored.xml` 의 `L2_Left_Wall1_1` geom 은
`contype="0" conaffinity="0"`(충돌 OFF)이고 `WALL_GEOMS_TO_ENABLE`
(= base_link / L1_Wall1_1 / L1_Wall2_1 / L2_Wall3_1)에도 들어있지 않다. IPC
모드에서는 IPC 가 MuJoCo 스타일 필터링을 무시하는 것으로 보여(§조합9 후속4)
클램프가 동작했지만, **rigid 모드에서는 리지드 솔버가 이 필터를 지키므로
clamp 페이즈에서 벽이 봉투를 그냥 통과한다.** rigid 모드에서만
`L2_Left_Wall1_1` 을 활성 집합에 추가해 우회한다(§3-2 의 "Left_Wall 은 충돌이
켜져 있어야 한다"와도 일치). 다만 비볼록 메시라 `convexify=True` 의 hull 로
클램프하게 되므로 접촉 위치가 실제보다 거칠 수 있다 — 정밀 클램프력이
필요해지면 §3-2 처럼 볼록분해가 필요하다.

> **[정정 2026-08-03] 이 우회는 기구를 잼시킨다 — 기본 OFF 로 되돌렸다.**
> "거칠 수 있다"는 정도가 아니라, 부풀린 hull 이 다른 크러셔 부품과 간섭해
> 슬라이드 조인트를 -10mm 에 고정시켜 **슬롯이 아예 안 열린다**. 상세는
> §13-7. 켜려면 `RB_LEFTWALL_COLLISION=1` 이되 **볼록분해가 선행돼야 한다.**

### 13-6. 1차 구현 결과 (2026-08-03)

**파일 분리(사용자 지시)**: 처음엔 `full_workflow.py` 안에 `RIGID_MODE=1` env
스위치로 넣었으나, FEM 실험과 섞이지 않도록 분리했다.

| | 파일 | 출력 |
| --- | --- | --- |
| FEM + IPC (재료 검증) | `full_workflow.py` | `RESULT/` |
| **RIGID (공정 검증)** | **`full_workflow_rigid.py`** | **`Result_NoCoupling_OnlyRigidbody/`** |

`full_workflow_rigid.py` 는 `import full_workflow as fw` 로 **배치·자세·페이즈
상수를 전부 공유**하고(궤적 1:1 대응의 전제), 자기 파일에는 "rigid 로 바꾸느라
달라진 것"만 둔다 — `slot_fit_check.py` 가 쓰는 것과 같은 패턴.
`full_workflow.py` 에 남긴 변경은 **녹화 API 수정 한 건뿐**이다(아래 1번,
FEM 모드도 그것 때문에 죽고 있었음).

env: `WALL_OPEN_MM`(슬롯 개방 mm, 기본 6) / `Y_OFFSET_MM` /
`RB_TABLET_CARGO`(기본 1) / `RB_LEFTWALL_COLLISION`(기본 0) / `VIEWER`.

**속도 — 목표 달성.** (아래는 2026-08-03 당시 기록이다. **배율은 §13-10 에서
정정됐다** — 실제로는 약 12배가 아니라 3.2배다. 이 표의 RIGID 수치 자체는
유효하고, 틀린 건 비교 상대였다.)

| | build | steps(2230) | 합계 |
| --- | --- | --- | --- |
| FEM + IPC | — | — | **16분 18초**(기존 실측) |
| **RIGID** | **7.4s** | **72.8s** (32.7ms/step) | **약 80초** |

당시 계산으로 **약 12배**. 배치 상수를 고치고 다시 돌리는 반복이 실용적인
영역에 들어왔다. (1회차는 build 가 154.9s 였는데 커널 컴파일 캐시가 없어서였고,
2회차부터 7.4s 로 떨어졌다.)

**시행착오 3건 — 전부 코드/로그로 원인 확정.**

1. **Genesis 1.3.1 녹화 API 미반영으로 크래시.** `stop_recording(
   save_to_filename=..., fps=...)` → `TypeError`. 조합11 에서 이미 발견해
   기록해 둔 변경(파일명/fps 가 `start_recording` 으로 이동,
   `stop_recording()` 은 인자 없음)이 `full_workflow.py` 에는 반영이 안 돼
   있었다. **이건 rigid 모드만의 문제가 아니라 FEM 모드도 같이 죽는
   상태였다** — 양쪽 다 고쳐졌다.

   → 회귀 검증으로 FEM 모드를 돌려보니 **같은 업그레이드가 깨뜨린 두 번째
   지점**이 더 있었다(`add_analytic_fem_entity` 의 0바이트 더미 STL을 1.3.1
   의 새 시각 geom 로더가 읽으려다 실패). 상세·수정 방향은 §조합11 의
   "추가 발견 2026-08-03" 노트 참고 — **아직 미적용이라 FEM 모드는 현재
   실행 불가**다(공유 유틸이라 별도 작업으로 분리).

2. **정제가 hold 해제 순간 봉투를 뚫고 튀어나감.** 1차엔 바닥 패널이 1mm 라
   두께 문제로 보고 4mm(안쪽 성장)로 키웠는데 증상이 그대로였다. 봉투 로컬
   프레임 진단(`tablet_local`)을 넣으니 원인이 정확히 보였다 — 정제는
   prep~close 내내 `(+0.0,-39.5,+0.0)mm IN` 으로 공동 바닥에 안정적으로
   있다가, hold 해제 직후 **로컬 z 가 0.2초 만에 0 → +59mm**(공동 두께는
   ±3mm뿐)로 튀었다. 즉 바닥 관통이 아니라 **두께 방향 측면 사출**이고,
   기여 요인은 (a) 공동 두께 6mm 에 캡슐 지름 4mm — 여유 1mm/쪽, (b) 그리퍼가
   *고정된*(텔레포트로 pin 된) 봉투를 누르며 쌓인 접촉 에너지가 해제 순간
   한꺼번에 풀림.
   → **정제를 화물(cargo)로 봉투에 종속**시켜 해결(`RB_TABLET_CARGO`,
   기본 1). "정제가 봉투에 담기는가"는 §조합5/6/9 에서 이미 검증된 항목이라
   rigid 모드에서 다시 풀 이유가 없다. 담기 물리를 다시 보려면 0 으로 끈다.

3. **PD 추종 지연.** `kp=4500` 에서 above/lift 같은 빠른 구간의 추종 오차가
   74~82mrad(4~4.7°)까지 벌어졌고, above 종료 시점에도 안 가라앉아 슬롯
   정렬(gap 12mm 대비 봉투 8mm = 여유 2mm/쪽)에 그대로 실렸다.
   `kp=20000/kv=1200` 으로 올려 종료 시점 오차 1.65~3.4mrad 로 수렴.

### 13-7. 슬롯 삽입 — 잼 오진 → 개방 상태 검증 (2026-08-03, 사용자 지시)

**처음엔 "봉투가 회전해 벽 윗면에 얹혔다"로 오진했다.** 1차 결과가
`bag_bottom_z=0.0858` ≈ `wall_top_z=0.0864` 였고 width/height 가 뒤바뀌어
있어(90.3 / 64.8) 마찰 파지가 자세를 못 잡는 문제로 결론지었다. 사용자가
**"슬롯을 완전히 연 상태에서 들어가는지부터 테스트해야 한다"** 고 지시해
개방 상태를 직접 계측하니, **애초에 슬롯이 열리지 않고 있었다.**

```
[phase] prep @done  wall=-10.16mm      ← 지령은 +6.00mm(개방)
```

지령과 부호부터 반대다. 마지막 정상 FEM 실행(2026-07-29)의 같은 지점은
`wall=+5.43mm` 로 정상이었다. **원인은 §13-5 에서 내가 켠
`L2_Left_Wall1_1` 충돌 자체**였다 — 비볼록 메시가 `convexify` 되며 부풀린
hull 이 다른 크러셔 부품과 간섭해, 슬라이드 조인트를 -10mm 에 물리적으로
잼시켰다. 액추에이터가 `actuatorfrcrange="-100 100"` 으로 제한돼 이 접촉을
못 이긴다. 즉 §13-5 에서 "거칠 수 있다"고만 적어둔 위험이 실제로는
**기구를 통째로 멈추는** 수준이었다. → `RB_LEFTWALL_COLLISION` 기본 **OFF**.

**끄자마자 봉투가 슬롯에 들어간다.** 개방량을 스윕한 결과:

| `WALL_OPEN_MM` 지령 | 실제 wall | 실제 통로 | 최종 tilt | width_y | height_z | bag_bottom_z |
| --- | --- | --- | --- | --- | --- | --- |
| +6 | +5.43mm | 17.4mm | — | 78.0mm | 97.9mm | 0.0120 |
| +20 | **+7.14mm** | 19.1mm | 8.6° | 74.7mm | 97.6mm | 0.0104 |
| +10 | **+7.03mm** | 19.0mm | **4.3°** | **64.5mm** | **90.7mm** | 0.0099 |

- **슬롯 최대 개방은 +7.1mm 다 — 하드 스톱.** +10 을 줘도 +20 을 줘도 같은
  +7.0~7.1mm 에서 멈춘다(조인트에 `range` 속성이 없으니 조인트 한계가 아니라
  기구 간섭이다). 따라서 **이 설계에서 슬롯 통로의 물리적 상한은 약 19.1mm**
  (정지 12mm + 개방 7.1mm)다. 봉투 8mm 대비 여유 5.5mm/쪽.
- **완전 개방 시 봉투는 거의 완벽히 수직으로 들어간다.** +10 실행의 최종
  `width_y=64.5mm`/`height_z=90.7mm` 는 봉투 실치수(64 / 91mm)와 사실상
  일치한다 = 기울지 않았다는 뜻. tilt 4.3° / twist 4.3°.
- **AABB 폭·높이는 자세 판정에 쓰면 안 된다.** 회전이 섞이면 Z-extent 가
  실제 높이 91mm 를 넘어버려(97.9mm) 해석이 안 된다. 봉투 높이축(로컬 +Y)과
  world +Z 사이 각(`tilt`), 두께축(로컬 +Z)과 world X 사이 각(`twist`)을
  직접 재는 지표를 추가했다.
- 삽입 중 tilt 가 1.6°→26.3° 로 단조 증가했다가 정착하며 4~11° 로 풀리는
  패턴이 보인다 — 하강 중 어딘가에 끌리다가 놓이는 것으로, 통로 여유가
  5.5mm/쪽이나 되는데도 남는 현상이라 파지 자세 구속의 여지는 여전히 있다.

**판정 기준 — rigid 전용으로 새로 정의.** `full_workflow.py` 의 판정식은
`abs(final_bottom_z - wall_center_z) < 30mm` 라는 **양방향** 창이라, 봉투가
포켓 바닥까지 제대로 내려가면 오히려 FAIL 이 뜬다("못 미쳤다"가 아니라 "더
깊이 들어갔다"). 봉투가 매달려 있던 FEM 시절 설계다. 파일이 분리됐으므로
`full_workflow_rigid.py` 는 **FEM 쪽을 건드리지 않고** 자기 기준을 쓴다:

```
entered = bottom_z < wall_top_z - 20mm      # 벽 위에 얹히지 않았나
seated  = bottom_z < wall_center_z + 10mm   # 포켓 안까지 내려왔나
upright = tilt < 15deg and twist < 15deg    # 서 있나
```

**분리 후 재검증(`WALL_OPEN_MM=10`, 즉 완전 개방):**

```
[RESULT] verdict=PASS  (삽입 성공, 자세 정상)
[RESULT] wall_open 지령=+10.0mm 실제=+7.03mm  통로=19.0mm
[RESULT] bag_bottom_z=0.0146  (wall_top=0.0864 wall_center=0.0504 pocket_floor=0.0144)
[RESULT] tilt=4.8deg  twist=5.5deg
```

봉투 바닥이 **포켓 바닥(0.0144)의 0.2mm 위**에 안착했다 — 완전 착저. 벽 하드
스톱(+7.03mm)도 그대로 재현. build 7.5s + steps 75.3s.

### 13-8. FEM 봉투 + IPC 커플러 격리 검증 — **정상 확인** (2026-08-03)

rigid 작업 중 FEM 모드가 안 도는 것을 발견했는데, 실패 지점이 **정제**여서
"FEM 모드 전체가 죽었다"로 뭉뚱그려 보고했다가 혼선이 있었다. 실제로 확인이
필요했던 건 **봉투(FEM.Cloth) + IPC 커플러가 이 환경에서 건전한가** 였고,
정제는 그 검증의 대상이 아니다. `SKIP_TABLET=1`(`full_workflow.py`)로 정제만
빼고 분리 검증했다.

**결과 — 전 구간 정상, `verdict=PASS`.**

```
[tablet] SKIP_TABLET=1 — 정제 제외, 봉투(FEM.Cloth)+IPC 만 검증
[build] 성공
[bag] shape 고정: 339/771 정점(바닥+양측면), 입구는 자유
[phase] prep @done  crank=-3.000rad  wall=+5.43mm
... drop → settle → close → grasp → lift → hold → above → insert → settle2 → clamp → release
[RESULT] verdict=PASS  (삽입 성공, 걸림/붕괴 없음)
[RESULT] final_bottom_z=0.0761  wall_center_z=0.0504
         width_y=51.7mm(baseline 64.1mm)  height_z=78.2mm(baseline 91.2mm)
```

세 가지가 한 번에 확정됐다:

1. **봉투 FEM.Cloth + IPC 커플러는 Genesis 1.3.1 에서 정상이다.** `339/771 정점`은
   마지막 정상 실행(2026-07-29 `_run_reposition_check.log`)과 **정확히 같은
   수치**다. IPC 코드도 실제로 돈다(`ipc_coupler/utils.py` 의
   NumbaPerformanceWarning 이 그 증거 — 경고일 뿐 오류 아님).
2. **§조합9 후속4 의 석션V1 조 자기교차는 이 씬에 영향이 없다.** `build()` 를
   통과했고 `Intersection detected` 도 `body_count` AttributeError 도 없다.
   1.3.1 에서도 07-29 와 같은 거동 — 그 이슈는 `Fixture_only` 격리 씬 한정이다.
   (§조합9 후속4 의 "최종 영상 재생성 보류" 상태는 이로써 해제해도 된다.)
3. **`full_workflow.py` 를 막는 것은 정제 하나뿐이다** — 위 조합11 추가 발견 노트의
   `mesh_to_elements` 시그니처 변경. 봉투/IPC/석션과 무관한 별개 부품이다.

**FEM vs rigid 비교(같은 궤적, 같은 스텝수)** — 삽입 깊이가 크게 다르다:

| | bag_bottom_z | 폭 변화 | 높이 변화 |
| --- | --- | --- | --- |
| FEM.Cloth | 0.0761 (벽 윗면 0.0864 아래 10mm) | 64.1 → **51.7mm** | 91.2 → **78.2mm** |
| rigid 프록시 | **0.0146** (포켓 바닥 0.0144) | 64.0 유지 | 91 유지 |

FEM 쪽은 천이 구겨지며 폭·높이가 20% 가까이 줄고 그만큼 얕게 들어간다. rigid
프록시는 안 구겨지니 끝까지 내려간다. **공정 검증(도달성·간섭)에는 rigid 가,
실제 삽입 깊이·구김 예측에는 FEM 이 맞다** — §13-1 의 2단 분리가 의도한 그대로다.

### 13-9. 정제 TetGen 우회 유틸 — Genesis 1.3.1 대응 재작성 (2026-08-03)

`utills/primitive_tablet_generator.py`. §조합11 추가 발견의 블로커를 해소했다.
**세 군데**를 고쳐야 했는데, 세 번째는 실행해보기 전에는 안 보였다.

| # | 무엇이 깨졌나 | 고친 방법 |
| --- | --- | --- |
| 1 | `sample()` 이 `gs.Mesh.from_morph_surface()` 로 **morph 파일을 실제로 읽어** 시각 geom 을 만들게 바뀜 → 0바이트 더미로는 `merge_submeshes` 가 죽음 | tet 메시의 경계면을 뽑아(`_boundary_faces`) **실제 표면 STL 을 쓴다**. 시뮬용 tet 은 여전히 패치가 주입하므로 TetGen 은 안 탄다 |
| 2 | `mesh_to_elements` 시그니처 `(file, pos, scale, tet_cfg)` → **`(mesh, tet_cfg)`** — 경로가 아니라 trimesh 객체. 경로를 캐시 키로 쓰던 패치가 무력화 | 캐시 키를 경로 문자열 → **일회성 큐**(`_ANALYTIC_PENDING`)로 교체. `scene.add_entity` 가 동기적으로 `sample()` → `mesh_to_elements` 를 부르므로 안전하고, `finally` 로 실패 시 큐를 비운다. `pos` 는 1.3.1 이 사면체화 **이후에** 더하므로 패치에서 더하지 않는다 |
| 3 | **정점 순서 계약** — 1.3.1 은 `FEMVisGeom.sim_verts_idx` 가 "표면 정점이 앞에, 입력 순서 그대로"를 가정(`sample()` docstring). 그런데 `make_capsule_tets_v2` 는 medial-axis 앵커(반구 중심·원기둥 축점) 같은 **내부 정점을 표면 정점 사이에 섞어** 만든다 | `_align_to_surface()` — Genesis 가 읽은 표면 정점과 우리 정점을 **위치로 매칭**(STL 왕복의 float32 오차 대비 형상 크기 비례 tol)해, 표면을 Genesis 순서 그대로 앞에 놓고 내부 앵커를 뒤에 붙인 뒤 elems 를 remap |

시그니처가 또 바뀌면 조용히 깨지지 않도록 `_ensure_patched()` 에 **버전 가드**를
넣었다(첫 인자명이 `mesh` 가 아니면 명시적 에러).

**격리 검증** (캡슐 R=2mm/실린더 1mm, pos=(0.10,-0.20,0.35)):

```
[ours]   verts=137 tets=288
[entity] n_vertices=137 n_elements=288      ← 우리 메시와 정확히 일치(TetGen 리파인먼트 없음)
[entity] size_mm=[4.0 4.0 5.0]  center=[0.1 -0.2 0.35]   → 크기·중심 OK
20 step 후 낙하 Δz=-51.5mm (자유낙하 기대 -49.1mm)        → 얼어붙음/폭발 없음
```

**전체 파이프라인 복구 확인** — `full_workflow.py` 를 정제 포함으로 완주,
`verdict=PASS`. 정제가 전 구간 봉투 안에 남는다:

| 페이즈 | tablet_z | bag_com z |
| --- | --- | --- |
| drop | 375.6mm | 398.5mm |
| grasp | 364.0mm | 398.1mm |
| **lift** | **483.3mm (+119)** | **523.6mm (+125)** |
| insert | 92.8mm | 91.8mm |

lift 에서 정제가 봉투와 **함께 119mm 상승** — 담긴 채로 운반된다.

호출부 13곳은 **시그니처를 안 바꿨으므로 자동 복구**된다(개별 검증은 안 함).
커밋돼 있던 0바이트 `RESULT/_analytic_capsule_v2.stl` 은 이제 실제 표면
메시(13,284 B)로 갱신된다.

**부수 효과 — 정제 렌더 깨짐 해결(2026-08-04).** 1.1.0 시절 정제 표면의 검은
지그재그 균열(`FEM/Result/fem_tablet_bag_suction_20260721_191211.mp4`)이
같은 스크립트 재실행에서 사라졌다. 원인은 **예전 패치가 반환값 3개 중
세 번째(표면 메시)를 `None` 으로 지어낸 것** — winding(468/468 바깥 일관)과
앵커 혼입(STL 왕복이 자동 제거)은 실측으로 배제했다. `full_workflow.py` 의
"우그러짐 → d_hat 축소" 대응은 오진이었다.

**가드 추가.** 몽키패치는 계약 위반이 조용히 지나갈 수 있어(위 건이 3개월간
안 보인 이유) `add_analytic_fem_entity` 끝에 사후 검사를 넣었다 — 시각 메시
**면 수**가 STL 과 같은지 + 시뮬 tet 이 주입값과 같은지. 정점 수는 안 본다
(Genesis 가 면마다 정점을 분리해 `vverts = 3 × vfaces` 가 정상 — 1차 구현이
여기서 오탐해 확인).

### 13-10. 계산비용 재측정 — **"약 12배"는 3.2배로 정정** (2026-08-14)

§13-6 의 배율은 **통제되지 않은 비교**였다. RIGID 쪽은 갓 계측한 값인데 FEM 쪽은
문서에 "기존 실측"이라고만 적힌 16분 18초를 그대로 가져다 썼다. 그 값의 정체를
추적해 보니 `full_workflow.py` 에는 **타이밍 코드가 한 줄도 없었다** — 사람이
벽시계로 잰 값이고, Genesis 버전도 커널 캐시 상태도 build/steps 분리도 기록이
없다. 반면 RIGID 80초는 스크립트가 찍은 warm 값(cold 154.9초는 명시적으로 제외).
자를 다르게 대고 잰 두 수를 나눈 셈이다.

#### 통제한 것

- **Genesis 1.3.3 + quadrants 1.3.0**, 같은 머신, 연속 실행.
- **2230 스텝** — `full_workflow_rigid.py` 가 `DT` 와 페이즈 `N_*` 를 전부
  `full_workflow.py` 에서 import 하므로 우연이 아니라 **구조적으로** 같다.
- **카메라 3대 동일**(1280x960, 960x720, 960x720), 같은 자세, 셋 다 fps=30 녹화.
  양쪽 **318 프레임** 인코딩(Genesis 녹화는 실시간 페이스라
  `round(realtime_factor/(fps*dt))=7` 스텝당 1프레임만 담는다 — §13-13).
- **같은 계측 경계** — `full_workflow.py` 에 `full_workflow_rigid.py` 와 동일한
  `time.time()` 펜스를 넣어 build / steps+encode 를 나눠 찍게 했다.
- **양쪽 2회 실행 후 warm 값 사용.**

#### 결과

| | build | steps(2230) | ms/step | 합계 | verdict |
| --- | --- | --- | --- | --- | --- |
| FEM + IPC (cold) | 77.1s | 262.2s | 117.6 | 339.3s | PASS |
| **FEM + IPC (warm)** | **12.0s** | **261.5s** | **117.3** | **273.5s** | PASS |
| RIGID (cold) | 111.6s | 79.4s | 35.6 | 191.0s | FAIL |
| **RIGID (warm)** | **9.1s** | **76.2s** | **34.2** | **85.3s** | PASS |

**총 3.2배, 스텝당 3.4배.** 옛 FEM 값 978초는 재측정값의 3.6배로, 재현되지 않는다.

#### 읽는 법 세 가지

**1. 스텝당 비용이 진짜 수치다.** cold→warm 에서 FEM 117.6→117.3, RIGID
35.6→34.2 로 거의 안 움직인다. 반면 build 는 커널 컴파일에 좌우돼 cold 에서는
**RIGID 가 오히려 느리다**(111.6 vs 77.1). 논문에 쓸 숫자는 스텝당 3.4배다.

**2. build 를 합치면 배율이 희석된다.** 총합 3.2배는 build 를 포함한 값이고,
반복 실행에서 build 는 한 번만 내는 비용이 아니라 매번 낸다(프로세스가 새로
뜨므로). 그래서 "반복 실험 밀도" 관점에서는 총합이 맞는 수치다 — 시간당
FEM 13.2회 vs RIGID 42.2회.

**3. RIGID 는 속도를 얻고 재현성을 잃었다.** 위 표에서 FEM 은 2회 모두 PASS 인데
RIGID 는 FAIL/PASS 로 갈렸다(§13-11). 3.4배 가속의 대가다.

`d_hat=5e-5` 의 "30분+" 도 같은 출처(코드 주석)라 **아직 미검증**이고, 이번
그림에서는 뺐다.

그림: `docs/rigid_vs_fem_cost.{png,pdf}` — 생성 `utills/plot_rigid_vs_fem_cost.py`.

### 13-11. **판정이 비결정적이다** — 같은 설정에서 FAIL/PASS 가 갈린다 (2026-08-14)

Genesis 1.3.3, `RB_BAG_SOLID=0`, `WALL_OPEN_MM=6`, 나머지 기본값. 연속 2회:

| run | above | insert | settle2 | release | verdict |
| --- | --- | --- | --- | --- | --- |
| 1 | 38.7° | 34.7° | 35.6° | **31.6°** | **FAIL** |
| 2 | 40.4° | 25.2° | 22.1° | **7.3°** | **PASS** |

**`above` 의 스윙은 재현된다**(38.7 vs 40.4°). 재현되지 않는 건 그 뒤로 봉투가
자세를 되찾느냐다. 판정 임계 15° 가 그 산포 한복판에 있어 **판정이 사실상 동전
던지기**가 된다. 시드 고정이 없고, 결과가 한계 파지의 접촉 해결 순서에 걸려
있어 미세한 수치 차이가 빠른 `above` 구간에서 증폭된다.

**`above` 스윙 자체는 1.2.1 대비 커졌다** — 같은 설정에서 1.2.1 은 14.0° 였다.
이건 재현되는 변화라 따로 쫓을 가치가 있다(미해결).

#### 이 절이 다른 결론에 미치는 영향

**A/B 를 1회 실행으로 판단하면 안 된다.** 최종 verdict 대신 재현되는 중간값
(`above` tilt, jitter RMS)을 보거나, N회 돌려 분포를 봐야 한다. 단일 FAIL 은
회귀의 증거가 아니고 단일 PASS 는 수정의 증거가 아니다.

특히 **§13-12 의 solid 박스 비교(각 1회)는 정황일 뿐 확정이 아니다.**

### 13-12. 봉투 프록시를 단일 box primitive 로 — 떨림 대책 (2026-08-14, 사용자 지시)

**동기(사용자 관찰)**: rigid 봉투가 심하게 떨린다. 5-panel 쉘 때문인 것 같다.

쉘이 떨림을 만들 경로는 실제로 셋이다 — (a) 1mm 판 5장이 한 바디에 붙어 접촉점이
판마다 따로 생기고, (b) 그리퍼가 얇은 판을 물면 접촉이 켜졌다 꺼졌다 하고,
(c) 공동 6mm 에 캡슐 지름 4mm 인 정제가 여유 1mm/쪽으로 front/back 을 간헐적으로
때린다. 볼록 geom 하나면 셋 다 사라진다.

**구현**(`RB_BAG_SOLID=1`, 기본): 공동 없는 box primitive 한 장. 외형은 5-panel
쉘의 AABB 와 **정확히 같은 64×90×8mm** 로 잡아 §13-7 슬롯 여유 수치를 승계한다.
질량도 같은 `RB_TARGET_MASS`(FEM cloth 등가 2.597g)로 맞추고, 부피가 14.1→46.1cm³
로 3.26배 커지므로 density 를 184→56 kg/m³ 로 역산한다.

**부작용 — 정제를 낙하로 담을 수 없다.** 공동이 없으니 정제는 박스 윗면에 얹혔다
굴러떨어진다. 그래서 solid + cargo 조합에서는 정제를 **첫 스텝부터** 태우고
(로컬 y = −39.0mm, 5-panel 공동 바닥 착지점과 동일하게 계산) **정제 충돌을 끈다**
(`contype=0`). 안 끄면 봉투 solid 와의 겹침이 관통 임펄스로 터지고, cargo 는 정제
pose 를 덮어써도 반작용이 봉투로 가 새 떨림원이 된다. 담기 자체는 §조합5/6/9
(FEM+IPC)에서 검증 완료라 rigid 모드가 다시 풀 문제가 아니다.

**1회 실행 결과 — 오히려 나빴다.** (Genesis 1.2.1 에서, 각 1회)

| | grasp | lift | hold | 결과 |
| --- | --- | --- | --- | --- |
| 5-panel | 686°/s | 642 | 674 | PASS(삽입 성공) |
| solid | 821°/s | 916 | 886 | above 에서 봉투 낙하(z=0.004, tilt 90°) |

떨림(각속도 RMS)이 준 게 아니라 늘었고 파지까지 깨졌다. 관성 분포가 유력한
용의자다 — 같은 질량에서 쉘은 질량이 앞뒤 판에 몰려 있고 solid 는 고르게 퍼져
있어 solid 가 기울기 쉽다(계산상 I는 20% 안팎 차이라 이것만으로 전부 설명되진
않는다).

**다만 §13-11 때문에 이 비교는 확정이 아니다** — arm 당 1회 실행이라 판정 산포
안에 들어간다. `RB_BAG_SOLID=0` 으로 예전 쉘을 남겨 뒀으니 N회 반복으로 다시
확인해야 한다. **미해결.**

### 13-13. Genesis 녹화는 실시간 페이스다 — 영상이 짧게 끝나는 이유 (2026-08-14)

`sand_wheel_repro_20260731_170221.mp4` 가 300스텝을 돌고도 **27프레임 0.89초**로
끝났다. 시뮬이 짧아서가 아니다. `genesis/vis/camera.py` 가 프레임을 고르는 식:

```
steps_per_frame = max(1, round(realtime_factor / (fps * dt)))
```

기본값(`realtime_factor=1.0`, `fps=30`, `dt=3e-3`)이면 11 — 11스텝당 1프레임만
남는다. 즉 **영상 길이 = 시뮬 시간 / realtime_factor** 이고, `N_STEPS` 를 늘리면
시뮬 시간과 함께 영상도 늘지만 결코 **느려지지는** 않는다.

원하는 길이를 얻으려면 역산한다: `realtime_factor = N_STEPS * dt / video_sec`.
`sand_wheel_repro.py` 에 `VIDEO_SEC`(기본 10초)로 넣었다 — `N_STEPS=900` 에서
300프레임 10.00초(3.7배 슬로모션), 모래 9,000→27,000 입자.

함정 둘:

- **`show_viewer=False` 여도 페이스는 `viewer_options.realtime_factor` 를 읽는다**
  (viewer 가 None 이면 그리로 폴백). 헤드리스에서 viewer 옵션이 무의미하다고
  넘기기 쉽다.
- **녹화 중에는 루프에서 `cam.render()` 를 직접 부르지 말 것.** `scene.step()` 이
  페이스에 맞춰 알아서 렌더/인코딩하므로, 수동 호출은 인코딩되지도 않는 프레임을
  그리며 GPU 시간만 쓴다.

---

## 14. 봉투 슬롯 삽입 — 근본 원인 규명 (2026-08-14)

*(사용자 지시로 §12-1 우선순위 변경. 타깃 조합 = **Samplebag(FEM.Cloth) +
Tablet(FEM.Elastic 3D) + 그 외 전부 Rigid + IPC 커플러** = `full_workflow.py`
현재 구성 그대로. §13 의 전-rigid 모드는 저비용 검증 하네스로 격하.)*

증상(사용자): "알약을 담아 슬롯으로 이송해 담는 워크플로우에서 비닐이 계속
휘어져 원하는 위치에 안 들어간다." 목표: **봉투의 Z 최하단이 Wall_1 의 중간
높이(`wall_center_z`)에 오는 것.**

§9 조합9 후속 1~4 에서 여러 라운드를 "위치가 조금 안 맞는다"로 튜닝했는데,
**재료 물성 문제가 아니라 조준이 틀려 있었다.** 원인 세 가지 전부 기하/기구학
이고 CAD 치수에서 결정론적으로 계산된다.

### 14-1. 원인 1 — 봉투 몸체가 슬롯에서 28mm 벗어난 채 내려가고 있었다

그리퍼는 봉투의 **세로 실링 가장자리**(`SEAL_LOCAL_X=-28mm`, `BAG_EULER`
회전으로 world +Y 에 매핑)를 문다(§5-2 에서 검증된 파지). 종이를 왼쪽 끝만
집어 든 것과 같아서 64mm 폭 몸체가 핑거 한쪽으로 통째로 뻗는다:

```
BAG_POS[1] = FINGER_MID[1] - SEAL_LOCAL_X      # 봉투 중심 = 핑거 + 28mm
```

그런데 `above`/`insert` IK 는 **핑거 TCP** 를 `gap_cy` 에 맞추고 있었다. trimesh
실측(Genesis 없이 계산 가능):

| | 값 |
| --- | --- |
| 봉투 Y 스팬 | `[-0.0513, +0.0127]` |
| gap Y 창 | `[-0.0798, -0.0148]` |
| 결과 | 폭 64mm 중 **+27.5mm 가 Wall3/Left_Wall 상면 위** |

봉투 절반이 벽 윗면에 얹힌 채 눌리니 천이 그대로 접힌다. 로그도 정확히 일치
한다 — `_run_reposition_check.log` 의 `above` 시점 `bag_com.y=-0.0239` vs
`gap_cy=-0.0473`(예측 -0.0193, 차이는 처짐/스윙).

**이 값 하나가 §13-7 에서 설명 안 되던 것도 같이 설명한다** — "통로 여유가
5.5mm/쪽이나 되는데도 남는 tilt 26.3°"는 봉투의 튀어나온 절반이 벽 모서리에
걸려 회전한 것이었다. rigid 프록시는 강체라 걸렸다가 미끄러져 들어갔고(그래서
§13-7 이 PASS 로 보였다), 천은 그냥 접힌다.

### 14-2. 원인 2 — 하강 경로가 카테시안 직선이 아니었다

`above`→`insert` 를 두 IK 해 사이의 **조인트각 선형보간**으로 내려갔다. 이건
카테시안 직선이 아니다. `slot_ik_check.py` 로 FK 실측:

```
최대 이탈 s=0.50 에서 dx=+0.10mm  dy=+9.67mm      (봉투 Y 여유 0.50mm/쪽)
```

**19배 초과**다. 양 끝점을 아무리 정확히 맞춰도 그 사이에서 벽에 긁힌다.
→ z 만 균등하게 내려가는 웨이포인트 41점마다 IK 를 풀고 그 사이만 조인트
보간한다(`fw.solve_descent_waypoints`). 간격 4.6mm 라 구간 내 부풂은 0.01mm 수준.

### 14-3. 원인 3 — `insert_z` 가 목표가 아니라 충돌 한계에서 나온 값이었다

`INSERT_MARGIN_ABOVE_CENTER=0.052` 는 §9 조합9 후속2 의 rigid 스윕에서 나온
"핑거-Crusher 충돌이 안 나는 최소 여유"였을 뿐, **원하는 삽입 깊이와 아무
관계가 없다.** 사용자 목표에서 역산하면:

```
BAG_HANG_BELOW_FINGER = 2*BAG_HALF_H - TOP_GRIP_MARGIN = 90 - 8 = 82mm
insert_z = wall_center_z + 82mm                       # 0.1324 (구 0.1024)
```

82mm 는 충돌 경계(52mm)보다 30mm **더 높아** 충돌 여유가 오히려 늘어난다.
실측 자유 매달림 길이는 83.0mm(`hold` 시점)로 설계값과 1mm 이내 일치.

### 14-4. 부수 처방 — 스윙 억제

- **5차 최소저크 프로파일**(`fw.ease`). 기존 `s=(k+1)/n` 선형 램프는 구간
  양끝에서 속도가 계단으로 튀어(= 무한대 가속 임펄스) 매달린 봉투를 때린다.
  총 스텝수는 그대로라 §13-1 의 궤적 1:1 대응은 유지된다.
- **`aboveset` 페이즈 신설**(`N_ABOVE_SETTLE=200`, 1.0s) — 하강 전 팔은 정지,
  봉투만 가라앉힌다.

### 14-5. 검증

**(a) 저비용 rigid 하네스** (`full_workflow_rigid.py`, 90초):

| | 수정 전 | 수정 후 |
| --- | --- | --- |
| insert 시점 tilt | 4.3~8.6° (§13-7) | **0.2°** |
| Crusher 충돌 | — | 0건 (`slot_ik_check.py`) |
| IK 오차 | — | 0.005mm |

다만 rigid 프록시는 삽입에 실패했다 — 64×8mm 박스를 마찰로만 잡다 보니 전송
중 아래로 24mm 미끄러지고 충돌 시 28mm 되밀려 올라간다. FEM 봉투는 같은
구간에서 미끄러짐이 없어(`hold`/`above` 내내 핑거 아래 82~83mm 유지) **프록시
한정 이슈**로 판단하고 넘어갔다. §13 이 산출물이 아니라 하네스로 격하됐으므로
이 파지 품질은 지금 고칠 이유가 없다.

**(b) 타깃 조합 FEM 실행** (`_fw_bagcenter_run1.log`):

| | 수정 전(07-29) | 수정 후 |
| --- | --- | --- |
| `bag_bottom_z` 오차 | **+26.0mm** | **+9.7mm** |
| `height_z` (baseline 91.2) | 80.5mm (**12% 수축**) | **94.4mm** (수축 없음) |
| insert 시점 봉투 COM ↔ 핑거 | 11.4mm | 25.9mm |
| 판정 | PASS(창 30mm) | PASS(창 15mm) |

수축이 사라진 게 핵심이다 — 이전엔 봉투가 벽 위에서 구겨지며 높이가 12%
줄어든 것이고, 지금은 실제로 슬롯 안으로 내려간다. `sideview` 영상 프레임으로
육안 확인: 수정 전은 봉투가 벽 상면에 팬케이크처럼 눌려 있고, 수정 후는 벽
블록 사이 채널로 몸체가 내려가 있다.

**판정창을 30mm→15mm 로 조였다.** 깊이 72mm 포켓에서 30mm 는 "어디든 들어가면
PASS"라, 실제로는 봉투가 벽 위에 접힌 채 26.0mm 오차로 PASS 가 찍히고 있었다.

### 14-6. 잔차와 폐루프 trim — 강화학습이 필요한 구조가 아니다

남은 +9.7mm 의 출처는 분해된다:

| 구간 | 핑거 아래 매달림 |
| --- | --- |
| `hold`(자유) | 83.0mm |
| `above` | 82.6mm |
| `aboveset` | 82.3mm |
| **`insert`** | **69.7mm** |
| `settle2` | 71.2mm |

자유 매달림은 설계값과 일치하고, **하강 중 슬롯 벽에 스치며 되말려 올라가는
11mm 만 예측이 안 된다** — 천의 접촉 이력에 달린 값이라 상수로 못 박는다.
그런데 맞춰야 하는 건 스칼라 하나(`bag_bottom_z`)이고 핑거 z 와 거의 1:1 이다.
→ `settle2` 뒤에 **1자유도 폐루프 `trim` 페이즈**를 넣어 남은 오차만큼 더
내려준다(gain 0.8, tol 2mm, 최대 4라운드, 하한은 충돌 경계 `wall_center+52mm`).

**강화학습을 쓸 문제가 아니다:**
1. 자유 파라미터가 사실상 3개(정렬 dy / 삽입 깊이 / 경로 형태)이고 앞의 둘은
   CAD 치수에서 바로 나온다. 나머지 하나는 위 폐루프가 닫는다.
2. 비용이 안 맞는다 — FEM+IPC 롤아웃 1회 ~16분이라 1,000 에피소드면 11일이다.
3. 배울 만한 확률적 구조가 없다. 같은 궤적이면 같은 결과다.

RL 이 정당해지는 국면은 "천의 구김을 예측해 파지점을 매번 다르게 잡아야 한다"
같은 데인데, 거기까지 가기 전에 조준이 틀려 있었다.

### 14-7. 교훈

1. **"위치가 안 맞는다" 를 물성/솔버 문제로 읽기 전에, 작업점(TCP)과 실제
   제어 대상(여기선 봉투 몸체) 사이의 고정 오프셋부터 실측할 것.** §9 후속2 의
   "IK 타깃 링크가 손가락이 아니라 손목 브라켓이었다"(140mm)와 **같은 계열의
   실수**를 파지 대상 쪽에서 반복한 것이다 — 그때는 로봇 쪽 오프셋, 이번엔
   물체 쪽 오프셋.
2. **조인트각 선형보간을 "직선 이동"으로 착각하지 말 것.** 여유가 mm 단위인
   삽입 공정에서는 중간 이탈이 양 끝점 정확도보다 훨씬 크게 작동한다.
3. **판정창이 목표 정밀도보다 헐거우면 실패가 PASS 로 위장된다.** 26.0mm 오차가
   30mm 창을 통과하며 여러 라운드 동안 "성공"으로 기록됐다.
4. 목표 상수는 **원하는 결과에서 역산**해 정의할 것 — `INSERT_MARGIN_ABOVE_
   CENTER` 처럼 제약(충돌 한계)에서 나온 값을 목표로 재사용하면 근거가 끊긴다.

### 14-8. 파지 위치 스윕 — 절벽은 "가장자리 4mm" 에 있다 (2026-08-14, 사용자 지시)

§14-6 의 trim 발산이 "밀어넣기로는 못 푼다"를 보여줬으므로, 남은 오차를 **파지
위치**에서 푸는지 확인했다. `GRIP_OFFSET_MM` 신설 — 봉투 로컬 폭축(world Y) 상에서
그리퍼가 무는 지점이다. 이 값 하나가 `BAG_DY_FROM_FINGER` 를 통해 슬롯 정렬 보정과
above/insert IK 타깃까지 전부 끌고 가므로, **어느 케이스든 봉투 몸체는 항상 슬롯
중심에 정렬된 상태**로 비교된다 — 변인은 "봉투의 어디를 무느냐" 하나뿐이다.

조건 통제를 위해 `TRIM_ROUNDS=0`(폐루프 OFF)으로 `settle2` 시점 값을 비교했다.
출력은 `RESULT/grip<±NN>mm/` 로 케이스마다 분리(영상 3종 + `run.log`).

| `GRIP_OFFSET_MM` | 물린 정점 | tilt @hold | @above | **@insert** | `bag_bottom` 오차 | width_y(→final) |
| --- | --- | --- | --- | --- | --- | --- |
| **0** (폭 정중앙) | 118 | 2.6° | 2.3° | **2.2°** | **-0.9mm** | 64.1 → 66.9 |
| -7 | 116 | 3.3° | 2.7° | **2.6°** | **-1.2mm** | 64.3 → 66.8 |
| -14 | 121 | 2.5° | 2.5° | **2.7°** | **-1.1mm** | 64.3 → 66.8 |
| -21 | 108 | 2.5° | 2.0° | **2.6°** | **-1.2mm** | 67.2 → 66.2 |
| **-28** (구 기본값) | **91** | 4.7° | 7.6° | **14.8°** | **+9.9mm** | 64.2 → **91.1** |

경계를 좁히려 추가한 **-24mm 는 삽입 이전에 파지 자체가 실패**했다 — 삽입 성능
비교 대상이 아니므로 위 표에서 뺀다. 2회 모두 실패했고 **실패 모드가 서로 달랐다**:

| 시도 | 증상 | 지표 |
| --- | --- | --- |
| 1 | `close`~`grasp` 중 봉투가 넘어짐 — 그리퍼가 빈손으로 올라감 | `hold` tilt **82.9°**(=드러누움), `height_z` 8.9mm(설계 90mm), `bag_bottom` 이 above/insert 내내 0.3512 로 불변(선반 위) |
| 2 | 파지는 됐으나 `above` 이송 중 놓침 | `hold` tilt 4.4° 이지만 baseline `height_z` **108.9mm**(설계 90mm — 모서리로만 매달려 늘어난 상태) → `above` 에서 tilt **74.3°**, `bag_bottom` **0.0011**(바닥) |

물린 정점 수는 108(-21) → 102(-24) → 91(-28) 로 단조 감소하는데 결과는 단조가
아니다(-21 정상 / -24 파지 실패 / -28 파지는 되나 얕게 삽입). 즉 -24mm 실패는
"잡은 양이 적어서"만으로는 설명되지 않는다 — 봉투가 5-panel 파우치(front/back
대면 + 좌우 좁은 거싯 + 바닥, §13-3)라 이 근처가 **평평한 2겹이 아니라 거싯이
접히는 자리**일 가능성이 있다. 확인하려면 파지 지점의 로컬 패널 소속을 찍어봐야
한다. **미해결 — 다만 -24mm 가 쓸 수 없는 파지점이라는 결론은 2회 재현으로 충분하다.**

> **부수 확인 — 이 파이프라인은 run-to-run 재현성이 없다.** 같은 커밋·같은 env 로
> 돌린 -24mm 두 번이 `drop` 부터 갈렸다(tilt 5.4 vs 2.4°, `close` 15.0 vs 8.6°).
> GPU float32 + IPC 접촉 순서의 비결정성으로 보인다. **단일 실행 결과를 그대로
> 결론으로 쓰면 안 되고**, 위 표처럼 케이스 간 분리가 큰 경우(1mm 대 9.9mm)에만
> 1회 실행으로 판단할 수 있다. 경계 근처를 다룰 땐 반복 실행이 필요하다.

**결과는 완만한 곡선이 아니라 절벽이다.** 정중앙부터 -21mm 까지는 전부 목표의
**1mm 이내**(오차 -0.9 ~ -1.2mm)에 안착하고 tilt 도 2~3° 로 평평하다. -28mm 하나만
+9.9mm 로 튀고 tilt 가 4.7 → 7.6 → 9.3 → **14.8°** 로 단조 증가한다.

**메커니즘 — 그리퍼가 봉투 가장자리를 물다 만다.** 봉투 반폭이 32mm 이므로
-28mm 는 가장자리에서 **4mm** 지점이다. 물린 정점 수가 108(-21mm) → **91**(-28mm)
로 16% 떨어진다 = 핑거 패드의 상당 부분이 봉투 밖으로 나가 있다. 잡은 천이 적으니
봉투가 파지점에서 돌아가고(hold 시점부터 이미 4.7°, 다른 케이스는 2.5°), 기운 채로
내려가면 슬롯 입구에서 쐐기처럼 껴서 10mm 를 남기고 멈춘다. `width_y` 가 최종
91.1mm(설계 64mm)까지 벌어지는 것도 이 케이스뿐 — 나머지는 66~67mm 로 사실상
변형이 없다. 영상(`sideview`)으로도 정중앙 파지는 얇은 수직 판이 그대로 슬릿에
내려가고, -28mm 는 입구에서 접혀 옆으로 퍼진다.

**즉 §14-1~14-3 의 기하 수정으로 남았던 +9.7mm 잔차는 파지 위치가 원인이었다.**
조준(§14-1)과 경로(§14-2)와 깊이(§14-3)를 다 고친 뒤에도 남던 마지막 항이고,
폐루프로 밀어넣어 지우려던 것(§14-6)이 사실은 파지 한 줄로 사라진다.

**권고: `GRIP_OFFSET_MM = -21` (기본값 후보).** 정중앙(0)이 수치상 가장 좋지만
입구 한가운데를 물어 **봉투 입구가 닫힌다** — 이후 석션V1 이 흡착컵 2개로 봉투를
양옆으로 당겨 여는 공정(§full_workflow.py SUCTIONV1_POS 주석)과 상충한다. -21mm 는
삽입 성능이 정중앙과 구분 불가(-1.2 vs -0.9mm)면서 파지점이 가장자리에서 11mm
안쪽이라 핑거가 천을 온전히 문다. 실링 밴드(|x|>22mm)와는 1mm 차이로 살짝 안쪽이니,
실링부 마찰 파지(§5-2)를 유지해야 한다면 -24mm 케이스가 경계 판단용이다.

**교훈**: "물체를 어디로 옮기나"만 보고 "물체를 어디를 잡나"를 상수로 놔뒀다.
파지점은 조작 성능의 자유 변수이고, 여기서는 **다른 모든 튜닝을 합친 것보다 큰
효과**를 냈다(잔차 9.7mm → 1mm). §14-7 의 교훈 1과 같은 축이다 — 작업점과 물체의
관계를 기하로 먼저 확인할 것.

## 15. 회수장치2 실링부 압착 고정 — 시퀀스는 성립, 파지는 실패 (2026-08-19)

> **[정정 2026-08-21 — §16 참조]** 이 절의 판정 지표(`D. 해제 후 낙하량`)는
> 파지를 재지 못한다. 봉투는 압착 여부와 무관하게 상판에 얹혀 멈추고, 아래에
> 비교한 10.93 / 11.18 / 12.23 / 14.37mm 는 전부 **해제 첫 스텝의 탄성 스냅과
> 초기 관통량**의 함수였다(턱을 전혀 닫지 않은 대조군도 같은 값이 나온다).
> 원인 후보로 지목한 `set_dofs_position` 마찰 앵커 리셋도 기각됐다 — 성공 사례인
> RG2 파지가 같은 API 를 쓴다. 아래 기록은 당시 판단으로 남겨둔다.

`Recovery2_only/recovery2_bag_clamp.py` 신설. 샤프트 180°(슬라이더 +35mm, 열림) →
봉투 투입(실링부 하단 = F_Top/M_Top 상면 58.00mm, 전 정점 hard constraint) → 샤프트
340°(틈새 0.8mm)로 잠금 → 구속 해제, 라쳇은 압착각을 매 스텝 재지령해 대신한다.
봉투는 실링부만 1mm 로 얇게 만든 `Samplebag_seal_pouch3_seal1mm.stl` 을 쓴다.

**시퀀스·형상은 목표대로 나왔지만 압착이 봉투를 잡지 못한다.** 대조군(턱 미폐쇄)
낙하 10.93mm 대비 클램프 11.18(틈새 0.8/마찰 0.1) · 12.23(0.8/0.8) · 14.37mm(0.4/0.8)
로, 압축 3배·마찰 8배를 줘도 **개선이 없고 오히려 악화**한다. 충돌 설정은 정상이다
(`_col` 33개 전부 활성, 고정턱 마스크 1 / 가동턱 2 로 갈려 서로만 비충돌; 봉투는 FEM
이라 IPC 가 마스크와 무관하게 접촉 — 가동턱이 실링부를 20mm 밀고 상판이 봉투를
59.10mm = 58.00+천두께1.00+d_hat0.10 에서 세우는 것으로 양쪽 다 실측 확인).
법선 접촉은 작동하는데 마찰만 듣지 않으므로, **샤프트를 매 스텝 `set_dofs_position`
으로 위치 지령하는 구동이 IPC 마찰 앵커를 리셋시키는 것**이 유력하다 — 검증하려면
토크/액추에이터 구동으로 바꿔야 한다(미실시).

부수 확인: IPC 는 초기 배치의 교차뿐 아니라 **근접도 거부**한다(봉투-상판 여유 3mm
거부 / 5mm 통과, 천 자기근접은 두께의 2배 요구). `CLOTH_THICK` 은 접촉 두께이자 막
강성 계수라 1.0→0.1mm 로 낮추면 봉투가 10배 물러진다(E 를 4.0e5→4.0e6 으로 상쇄).
`Rigid.coup_friction` 기본값은 0.1 이다.

## 16. 회수장치2 압착 재검증 — §15 의 판정 지표가 무효였다 (2026-08-21)

§15 는 "압착이 봉투를 잡지 못한다, 법선 접촉은 되는데 마찰만 안 듣는다"로 끝났고,
원인 후보로 **매 스텝 `set_dofs_position` 위치 지령이 IPC 마찰 앵커를 리셋한다**를
지목했다. 둘 다 틀렸다.

### 16-1. `set_dofs_position` 가설의 기각

성공 사례인 `full_workflow.py` 의 RG2 파지도 **똑같이 매 스텝
`robot.set_dofs_position(...)` 으로 손가락을 구동한다**(805/827/1029행). 같은 API,
같은 `coup_type="two_way_soft_constraint"`, 같은 `coup_friction=0.8` 인데 한쪽은
순수 마찰 파지가 성립한다(파일 헤더: "grasp는 처음부터 끝까지 순수 마찰 접촉이었는데
(치트 없음)"). 따라서 구동 방식은 원인이 아니다.

### 16-2. 진짜 차이 — 구속 해제 시점, 그리고 **지지**

| | full_workflow (성공) | recovery2_bag_clamp §15 |
|---|---|---|
| 구속 해제 | `close` **앞**(864행 -> 867행) | 다 닫은 **뒤** |
| 닫는 동안 천 | 자유 | 전 정점 hard(운동학적) |
| 닫는 동안 지지 | **선반 위에 놓여 있음** | 허공(M_Top 이 +35mm 후퇴) |

§15 는 닫는 내내 천이 운동학적이라 IPC 가 접촉력을 실을 대상이 없었다. 스크립트
주석이 이미 인정하고 있었다 — "압착은 해제 직후 IPC 가 겹침을 풀며 만들어낸다".

### 16-3. 판정 지표(`D. 해제 후 낙하량`)가 파지를 재지 못한다

해제 구간을 스텝별로 보면 **모든 설정에서 첫 스텝에만 떨어지고 그 뒤 700스텝 동안
전혀 움직이지 않는다.** 즉 9~11mm 는 미끄러짐이 아니라 얼어 있던 형상이 풀리는
**탄성 스냅**이다. 게다가 압착이 끝나면 M_Top 이 봉투 밑으로 되돌아와 봉투가
**상판에 그냥 얹혀** 멈춘다.

결정적 증거: **턱을 전혀 닫지 않은 대조군도 똑같이 멈춘다.**

| 실행 | 압착 | 낙하(D) | 탄성 스냅 | 크리프 |
|---|---|---|---|---|
| B1 대조군(lift 0) | 없음(35.00mm 열림) | +8.80mm | +8.80 | **+0.00mm** |
| A1 조건1+2(lift 0) | 0.80mm | +10.32mm | +10.33 | **-0.01mm** |
| A4 물림창(lift 0) | 0.80mm | +9.06mm | +9.17 | **-0.11mm** |

크리프가 전부 0 이다 — 어떤 설정에서도 봉투는 미끄러진 적이 없다. §15 가 비교한
10.93 / 11.18 / 12.23 / 14.37mm 는 전부 스냅과 초기 관통량의 함수였다.

### 16-4. 지지를 없앤 재실험 — 그래도 클램프는 못 잡는다

봉투를 상판 위 15mm 로 띄워(`BAG_LIFT_MM=15`) 상판 지지를 걷어내고 다시 쟀다.

| 실행 | 압착 | 탄성 스냅 | 크리프 |
|---|---|---|---|
| E1 압착 0.8mm | 있음 | +23.59mm | -0.02mm |
| E3 압착 0.8mm (반복) | 있음 | +23.58mm | +0.01mm |
| G1 압착 0.4mm | 있음 | +23.61mm | +0.01mm |
| F1 압착 0.8mm + 소프트구속 | 있음 | +23.57mm | -0.01mm |
| E2 대조군 | 없음 | +20.88mm | +0.00mm |
| E4 대조군 (반복) | 없음 | +20.88mm | +0.00mm |

압착 쪽이 오히려 2.7mm 더 떨어지고, 봉투는 15mm 리프트를 통째로 잃고 상판에
안착한다. **클램프는 실제로 봉투를 못 잡는다** — 다만 이유는 마찰이 아니다.

부수 확인 1 — 이 계측은 **완전히 재현된다**(압착 23.57/23.58/23.59/23.61,
대조군 20.88/20.88). §13-11 의 비결정성은 full_workflow 의 tilt 판정에 국한된
현상으로 보이며, 이 씬에는 해당하지 않는다.

부수 확인 2 — **틈새를 0.4mm 로 조여도 결과가 같다**(23.61 vs 23.59mm). §15 에서
0.4mm 가 가장 나빴던 것(14.37mm)은 조건2 이전의 초기 관통 때문이었고, 관통을
없애자 그 병리가 사라졌다. 조건2 의 효과가 여기서 분리되어 확인된다.

부수 확인 3 — **`set_vertex_constraints(is_soft_constraint=True, stiffness=...)` 는
이 스택에서 무효다.** k=1000 을 준 F1 이 완전 자유인 E1 과 안정화 후 실링부 하단
60.56mm · 두께 20.03mm 까지 **소수점 자리까지 동일**했다. `utills/
fem_ipc_workarounds.patch_fem_vertex_constraints()` 가 hard 경로만 여는 것으로
보인다(미확인). 소프트 구속은 이 씬의 도구로 쓸 수 없다.

### 16-5. 실제 원인 — 실링부가 틈새에 들어가질 못한다

물림창을 자유로 두면 봉투 밑에 받쳐줄 것이 없어(턱이 열려 있는 동안 M_Top 이
x[-55,-20] 을 비운다) 자유 천이 처지고 이음매가 벌어진다:

- A1(실링부 212정점 전부 자유): 실링부 하단 58.00 -> **47.06mm**, 두께 1.00 -> **20.03mm**
- A4(물림창 150정점): 하단 **58.00mm 유지**(처짐 해결), 그러나 두께는 여전히 20.03mm
- E1(리프트 15): 하단 73.00 -> 60.56mm, 압착 후 실링부 x[-69.37,-53.13] = **16.23mm**

틈새 0.77mm 에 두께 16~20mm 의 벌어진 천을 밀어 넣는 꼴이라, 턱은 실링부를 **무는
게 아니라 옆으로 뭉갠다**. 압축량이 A4 12.77mm / E1 3.81mm 로 나오지만 이건
"벌어진 걸 되접은 양"이지 물린 양이 아니다.

### 16-6. 유효했던 수정 (조건 1·2)

- **조건 2(관통 0)**: 배치 기준을 "최종 틈새 중심"에서 "고정턱 앞면 + 접촉
  오프셋(천두께+d_hat=0.20mm)"으로 바꿨다. 실링부 1.00mm 가 0.8mm 틈새 중심에
  놓이면 시작부터 양쪽 0.10mm 씩(틈새 0.4mm 면 0.30mm 씩) 파고든 상태였고,
  봉투는 `build()` 이후 `set_position()` 으로 투입돼 IPC 초기 검사를 타지 않아
  그게 그대로 통과했다. 지금은 첫 접촉 슬라이더 1.40mm -> 목표 0.80mm 로,
  0.60mm 를 **턱이 일해서** 누른다.
- **조건 1(물림창)**: 실링부 전부가 아니라 턱이 실제로 겹치는 창
  (y<=31.70 or y>=86.70, z 는 상·하 12mm 를 고정으로 남김 -> 150정점)만 자유로
  둔다. 자유 패치가 3면에서 물려 처짐이 사라졌다(47.06 -> 58.00mm).
  MJCF 실측: F/M_LeftLink y[11.70,31.70], F/M_RightLink y[86.70,106.70],
  네 링크 모두 z[48.00,148.00]. 봉투 폭 64mm -> 겹침은 양쪽 4.5mm 뿐이다.

### 16-7. 남은 일

`D` 대신 **크리프**를 판정에 쓴다(스크립트에 분리 출력 추가). 남은 병목은 하나로
좁혀졌다 — **실링부가 벌어진 채로는 0.8mm 틈새에 들어갈 수 없다.** 소프트 구속은
무효로 판명됐으므로(16-4 부수확인 3) 남은 수단은:

1. **실링부 메시 재생성 + `CLOTH_THICK` 복원**(조건3). 현재 생성식 `V[:,2] *= f`
   가 두께를 비례 축소해 파우치 둥근 가장자리가 0.398mm 까지 좁아졌고, 그 탓에
   `CLOTH_THICK` 을 성공 사례(full_workflow 1.0mm)의 1/10 인 0.1mm 로 낮춰야 했다.
   실링 대역을 중립면 기준 **±0.5mm 등두께 슬래브**로 다시 만들면 최소 자기간격이
   1.0mm 로 균일해져 `CLOTH_THICK=0.4mm` 가 가능하고(요구 0.8mm < 1.0mm),
   막 강성 `E*t=400 N/m` 유지를 위해 `CLOTH_E=1.0e6` 으로 맞춘다.
2. **`coup_links` 를 네 링크 + 두 상판으로 한정**(조건4). 지금은 33링크 전부가
   ABD 로 IPC 에 들어가고 그중 설계상 자기교차가 57쌍이다. full_workflow 의 로봇은
   `coup_links=FINGER_LINKS` 로 손가락 2개만 넣는다 — 성공 레시피와 같은 형태로
   맞춘다. E1 에서 자유 천이 고정턱 뒤(x=-69.37, F링크는 x[-60,-55])까지 밀려
   들어간 것은 이 씬의 rigid-cloth 접촉이 온전치 않다는 신호이기도 하다.
3. **지지 조건**. RG2 가 성공한 결정적 조건은 "자유롭되 **선반에 지지된** 천"이었다.
   회수장치2 는 턱이 열려 있는 동안 봉투 밑이 허공이라 같은 조건이 아니다. 시퀀스를
   유지하려면 로봇 파지를 구속이 아니라 **실제 그리퍼 링크**로 모델링하거나, 투입을
   턱이 거의 닫힌 뒤로 옮기는 순서 변경을 검토해야 한다.

## 17. 봉투–정제 재질 조합의 선택지 — 1번 폐기, 2번으로 간다 (2026-08-25)

이송 구간에서 봉투와 정제를 어떤 재질로 둘 것인가. 세 갈래가 있고, **1번을 놓고
2번으로 간다.** 트레이드오프는 모든 선택지에 존재한다.

### 17-1. 조합 1 — Samplebag(FEM) + 정제(Rigid) · **폐기**

IPC 커플러 상에서 freejoint 관련 계산 불가 에러가 있었다는 건 **사실이 아니다** —
그건 §9 조합 6 에서 이미 해결됐다(`coup_type="ipc_only"` 명시 override).

**진짜 문제는 같은 씬에 7자유도 매니퓰레이터가 함께 있을 때다.** 그러면 커플링
옵션을 `two_way_soft_constraint`(로봇 구동 + IPC 환경 혼용)와 freejoint 를 계산하는
`ipc_only`(구동을 놓겠다)의 **조합**으로 가져가야 하는데, 이 둘을 한 씬에 혼용하면
계산이 불가한 현상이 발생한다(§9 조합 8 — 로봇을 구동하지 않아도 정제가 1~2 스텝만에
지수 발산, `constraint_strength` 를 100→30→10→1.0 어디로 옮겨도 "발산 아니면 접촉
무력화"의 양자택일).

따라서 커플러의 **다른 옵션(`EXTERNAL_ARTICULATION`)** 을 고려했으나, 이는 **모든
링크를 collision body 로 설정해야 하므로 계산비용이 올라간다.** 이는 애초에 조합 1을
고려했던 철학(정제를 Rigid 로 내려 비용을 아낀다)과 맞지 않는다. → **놓는다.**

### 17-2. `EXTERNAL_ARTICULATION` 을 놓는 근거 (소스 확인, Genesis 1.3.3)

`genesis/engine/couplers/ipc_coupler/` 를 직접 읽어 확인한 사실을 남긴다.

**(a) 커플링 타입은 3개가 아니라 4개다** (`data.py:19`):

```python
class COUPLING_TYPE(IntEnum):
    TWO_WAY_SOFT_CONSTRAINT = 0
    EXTERNAL_ARTICULATION   = 1
    IPC_ONLY                = 2
    NONE                    = 3
```

**(b) Genesis 의 기본 자동선택은 고정베이스 로봇에 대해 `external_articulation` 이다**
(`utils.py:31`). 즉 `full_workflow.py` 가 로봇에 `coup_type="two_way_soft_constraint"`
를 명시한 것은 **기본값을 덮어쓴 선택**이다:

```python
def default_coup_type(entity):
    if has_articulation_dofs(entity):        # n_dofs > 0
        if entity.base_link.is_fixed:
            return "external_articulation"
        return "two_way_soft_constraint"
    return "ipc_only"
```

**(c) 두 경로의 성격이 다르다.** `two_way_soft_constraint` 는 링크마다
`SoftTransformConstraint` **페널티**를 걸고 `constraint_strength_translation/rotation`
로 세기를 조절한다. `external_articulation` 은 IPC 안에 **실제 관절 구속**
(`AffineBodyRevoluteJoint` / `AffineBodyPrismaticJoint`)을 만들고 Genesis 질량행렬을
IPC 로 넘긴다 — 강도 파라미터 자체가 없다. **조합 8 의 발산은 페널티 경로의 병리이고
`external_articulation` 은 그 축을 갖고 있지 않다.**

**(d) 그런데 `coup_links` 가 조용히 무시된다** (`coupler.py:156, 388`). 필드 주석이
`# Used for "two_way_soft_constraint"` 이고, 필터는 그 타입일 때만 걸린다:

```python
if coup_type == COUPLING_TYPE.TWO_WAY_SOFT_CONSTRAINT:
    link_filter = self._coup_links.get(entity)
    if link_filter is not None and link not in link_filter:
        continue
```

지금 `coup_links=FINGER_LINKS` 로 손가락 2개만 IPC 에 넣고 있는데, 전환하면 **로봇 전
링크가 ABD 로 들어간다.** 이게 (a) 비용 증가이자 (b) §16-7 이 지적한 "33링크 전부
ABD, 자기교차 57쌍"과 같은 형태다.

**(e) 모든 관절 링크에 충돌 지오메트리가 필요하다** (`coupler.py:592`) — 없으면
`gs.raise_exception("Rigid link has no collision geometry. Coupling type
'external_articulation' is not supported.")`.

> **판단**: (c)만 보면 시도 가치가 있고 전환도 한 줄이며 실패 시 build 단계에서 즉시
> 드러난다. 그러나 (d)+(e) 때문에 **비용을 아끼려고 정제를 Rigid 로 내리는 목적 자체가
> 상쇄된다.** 그래서 지금은 놓는다. 나중에 비용이 아니라 **정제의 파손 거동을 보지
> 않아도 되는** 다른 이유로 Rigid 가 필요해지면 그때 꺼낸다.

### 17-3. `ipc_only` 로 전부 통일하는 안 — **불가능** (소스 확인)

"`two_way_soft_constraint` 가 문제라면 전부 `ipc_only` 로 맞추면 되지 않나"는 성립하지
않는다. 결정적인 건 `external_kinetic` 플래그다(`coupler.py:525`):

```python
# external_kinetic: 1 = driven by rigid solver, 0 = IPC-only or IPC-driven free base
uipc.view(external_kinetic_attr)[:] = int(not is_free_base_ipc_driven and not is_ipc_only)
```

`ipc_only` 는 **rigid solver 가 구동을 놓는다**는 선언이다. 로봇에 걸면 셋이 동시에
깨진다:

1. `control_dofs_position` / `set_dofs_position` 이 무의미해진다(rigid solver 가 안 굴림).
2. **관절 구속이 IPC 안에 존재하지 않는다** — 관절 constitution 은
   `external_articulation` 경로에서만 생성되므로 링크들이 자유 ABD 강체로 흩어진다.
3. write-back `_post_advance_ipc_only` 는 base 7-qpos 만 쓰고, **고정베이스면
   `continue` 로 건너뛴다** — 아무것도 돌아오지 않는다.

### 17-4. 조합 6 의 freejoint 자동선택 버그는 아직 살아 있다

`has_articulation_dofs` 가 `n_dofs > 0` 인데 freejoint 는 6 DOF 라 그대로 통과하고,
베이스가 안 고정이니 `two_way_soft_constraint` 가 잡힌다. docstring 이 `n_joints` →
`n_dofs` 로 바뀐 건 **고정 Plane/Box**(`n_joints==1, n_dofs==0`)를 걸러내려는 수정이지
freejoint 케이스를 고친 게 아니다. **정제에 `coup_type="ipc_only"` 명시는 계속
필요하다.**

### 17-5. 조합 2 — Samplebag(FEM) + 정제(FEM) · **채택, 다음 검증 대상**

비용이 올라가나, **Rigidbody 와 비교해봐야 한다.** §13-10 의 3.4배는 *전부 rigid* vs
*전부 FEM* 비교라 여기 그대로 쓸 수 없다 — 봉투가 FEM 인 이상 IPC 는 계속 돌아야
하므로, **정제만 재질을 바꿨을 때의 실제 차이는 측정된 적이 없다.**

### 17-6. 조합 3 — Samplebag 만 + 정제–Crusher 는 따로 확인

분쇄 구간의 정제↔크러싱헤드 접촉은 봉투와 분리해 별도로 검증한다
(`Crusher_Pill.py` 가 8 RPM 실기 사양 + STRIKE/RETRACT FSM 으로 이미 이 pair 를 다룬다).

### 17-7. 조합 2 1차 실행 — 중앙 파지는 성립, `CLOTH_THICK` 은 못 낮춘다 (2026-08-25)

§17-5 의 조합 2(Samplebag FEM + 정제 FEM)를 **중앙 파지 + 실물 근사 두께**로 돌렸다.
`full_workflow.py` 의 `CLOTH_THICK`/`CLOTH_E` 를 기존 `GRIP_OFFSET_MM` 패턴에 맞춰
환경변수(`CLOTH_THICK_MM`, `CLOTH_E`)로 승격시켰다 — 기본값은 그대로 1.0mm/4.0e5.

| 런 | 파지 | 두께 | E | 판정 | 오차 |
|---|---|---|---|---|---|
| `_fw_gripcenter_t0.1mm.log` | 중앙(0mm) | **0.1mm** | 4.0e6 | **FAIL** | +302.5mm |
| `_fw_gripcenter_t1.0mm.log` | 중앙(0mm) | 1.0mm | 4.0e5 | **PASS** | **−1.1mm** |
| (2026-08-14 기존) | 중앙(0mm) | 1.0mm | 4.0e5 | PASS | −0.9mm |
| (2026-08-14 기존, §14-5) | 가장자리(−28mm) | 1.0mm | 4.0e5 | PASS | +9.7mm |

**1. `CLOTH_THICK` 을 실물값으로 낮추면 파지가 아예 성립하지 않는다.**
미끄러진 게 아니라 접촉이 안 잡힌다 — `lift` 에서 그리퍼만 +126mm 올라가고 봉투는
−0.03mm, 이후 `width_y`/`height_z` 가 baseline 과 **정확히 동일**(64.1/90.0mm)해
변형이 0 이다. 봉투는 선반 위에 그대로 서 있고 팔만 슬롯으로 내려갔다. 2026-07-15
실패("그리퍼는 126mm 올라갔는데 봉투는 그 자리에 그대로", 파일 헤더)와 같은 서명.

원인: **`CLOTH_THICK` 은 막 강성 계수이자 동시에 IPC 접촉 두께다.** `E*t=400 N/m`
보존만으로는 부족하다 — 1.0→0.1mm 는 천이 제시하는 접촉 껍질을 양면 합쳐 약 1.8mm
줄이는데, `FING_CLOSE=1.20` 은 1.0mm 두께에서 물도록 맞춰진 값이라 핑거가 천에
닿지 못한다(`[bag] grip_strip verts near FINGER_MID: 118` — 기하학적으로는 핑거가
봉투 위치에 있었다). **두께를 낮추려면 `FING_CLOSE` 재캘리브레이션이 함께 가야 한다.**

**2. 중앙 파지가 가장자리 파지보다 10배 정확하다** (오차 −1.1mm vs +9.7mm).
§14-6 의 trim 발산이 가장자리 파지의 굽힘 모멘트 탓이라는 가설과 일치한다
(§14-8 스윕이 "절벽은 가장자리 4mm" 라고 본 것의 반대편 끝). 삽입 판정창 15mm 기준
중앙 파지는 여유가 13.9mm 다.

**3. 실물 필름 두께 실측 기록이 프로젝트에 없다.** §13-3 의 STL 실측 6mm 는 파우치
공동 두께이지 천 두께가 아니고, 기본값 1.0mm 는 rigid 프록시 질량 일치 트릭
(`t=CLOTH_THICK` 이면 `density=CLOTH_RHO`)에 맞춘 값이다. `recovery2_bag_clamp.py`
의 0.1mm 도 실측이 아니라 실링부 메시 형상에 밀려 낮춘 값이다(§16-7). **두께를
논문 수치로 쓰려면 실측이 선행돼야 한다.**

**계측**: build 11.1s + steps 223.8s (2430 스텝, 92.1 ms/step) = 234.9s.
§13-10 warm 값(117.3 ms/step)보다 빠른데, 그 표는 `TRIM_ROUNDS` 포함 조건이었다.

## 18. 압착 공정 — 힘 상한 부재, 벽 형상 오독, 실링 두께 제약 (2026-08-25)

`full_workflow.py` 로 "중앙 파지 → 슬롯 삽입 → 실링부 압착"을 돌리며 세 건이 풀렸다.
셋 다 다시 밟기 쉬운 함정이라 근거째로 남긴다.

### 18-1. Motor2 힘 상한이 없어 위치제어가 IPC 배리어와 싸운다

`CLAMP_TARGET` 을 깊게 줬더니 세 번 연속 터졌다.

| CLAMP_TARGET | 결과 |
|---|---|
| −5mm | 정상 224초. 단 잔여 7.03mm 로 8mm 봉투를 스치기만 함 |
| −12mm | clamp 에서 **배리어 폭발**, 2651초까지 미완(스텝당 ~4.7s, 평소 0.09s) |
| −10.4mm (1차) | clamp 는 −9.21mm 에서 균형. 단 **release 가 24초/스텝** |
| −10.4mm (2차) | clamp 에서 **발산** — `wall=-2092357760.00mm` |

**같은 −10.4mm 가 한 번은 균형, 한 번은 발산했다**(§13-11 비결정성). 즉 안정 한계
위에 걸친 값이었고, 1차의 균형은 운이었다.

원인은 **벽에 힘 상한이 없다**는 것. `WALL_KP=5000` 위치제어가 무한한 힘을 낼 수
있는데 IPC 배리어는 간격→0 에서 반력이 무한대로 커진다. 둘이 서로를 키워 발산한다.

처방: `docs/Crusher.md §5` 의 `Motor2_left_wall` **ctrlrange ±100 N**(MJCF
`actuatorfrcrange` 와 동일)을 직접 건다. Genesis 의 `control_dofs_position` 경로는
MJCF 의 그 값을 적용하지 않는다.

```python
_fmin[wall_dof], _fmax[wall_dof] = -WALL_FORCE_LIM, WALL_FORCE_LIM
crusher.set_dofs_force_range(lower=_fmin, upper=_fmax)
```

힘이 제한되면 벽은 봉투 반력과 균형지는 곳에서 **물리적으로** 멈추므로 발산이 구조적
으로 불가능하고, 목표를 깊게 줘도 안전하다 — §11-5 의 "래칫 lock 없이 모터를 계속
구동해 강하게 고정"이 정확히 이 거동이다. **그 정지 위치가 곧 봉투의 실효 압착
두께 실측값이 된다.** (`Crusher_only.py:318~327` 이 크랭크에 같은 처방을 이미 쓴다.)

부수 처방 — `release` 는 `wall_q` 를 `CLAMP_TARGET` 이 아니라 clamp 종료 실측
위치에서 `WALL_PRELOAD`(0.5mm)만 더 누른 값으로 준다. 도달 불가능한 목표를 100스텝
내내 미는 것이 24초/스텝의 원인이었다. 그리고 `wq_final` 은 범위 검증 후 쓴다 —
발산값이 다음 페이즈로 전파돼 원인 파악을 늦췄다.

### 18-2. Left_Wall 은 평평한 벽이 아니다 — 5mm 플랜지는 **비산 방지 설계**

`L2_Left_Wall1_1` 의 마주보는 면은 두 단이다(STL 실측).

| 부위 | z 범위 | 면 y | 고정 3벽(y=336.28)과 간격 |
|---|---|---|---|
| 상단 플랜지 | 81.43 ~ 86.43 (5mm) | 324.28 | **12.00mm** |
| 본체 면 | 9.43 ~ 81.43 (72mm) | 319.28 | **17.00mm** |

**AABB 만 보면 12.00mm 로 보이는데 그건 플랜지에만 해당한다.** 이 오독이 §18-1 의
`CLAMP_TARGET` 판단을 계속 오염시켰다. 벽을 닫으면 상단 5mm 플랜지만 봉투에 닿고
본체 72mm 는 8mm 봉투를 영영 건드리지 못한다 — 봉투가 위 모서리 한 줄로만 눌려
release 때 tilt 2.5→6.9deg, x 드리프트 2.9mm 가 나온 이유다.

볼록분해(hull_000~010)는 이 형상을 **정확히** 재현한다 — hull_000 만 y=324.28 에
닿고 나머지 10개는 319.28 에서 끝나며, 시각 메시도 y>=324 인 정점이 z[81.43,86.43]
20개뿐이다. **분해 누락이 아니라 부품 형상이고, 볼록분해로 고칠 수 있는 문제가
아니다**(분해는 형상을 근사할 뿐 표면을 옮기지 못한다).

**단차를 box 로 메우는 처방은 폐기했다(사용자 지적).** 본체 면이 물러난 것은 결함이
아니라 **impact plate(`L9_PLATE_v3_1`)의 통로**다. 슬라이더 스트로크 실측:

| 슬라이더 q | 플레이트 y범위 | 메운 box 와 겹침 |
|---|---|---|
| −20mm | 306.78 ~ 316.78 | 0 |
| −15mm | 311.78 ~ 321.78 | **6,244 mm³** |
| −10mm | 316.78 ~ 326.78 | **12,500 mm³** |
| −5mm | 321.78 ~ 331.78 | **6,256 mm³** |
| 0mm | 326.78 ~ 336.78 (Wall3 면 도달) | 0 |

**[정정 2026-08-25, 사용자 지적]** 위 표의 겹침은 맞지만 **원인 해석이 틀렸다.**
물러난 5mm 단차가 플레이트의 통로인 것이 아니다 — `L2_Left_Wall1_1` 은 **ㅁ 형태의
프레임**이고 가운데가 뚫려 있다. 플레이트 z대역(24.43~74.43)에 존재하는 hull 은
단 두 개다:

    hull_001  x[-79.80,-74.80]  (좌측 세로부재 5mm)
    hull_003  x[-19.80,-14.80]  (우측 세로부재 5mm)
    개구부    x[-74.80,-19.80]  = 55.00mm  (시각 메시 정점 0개 = 완전 관통)

플레이트 x[-73.30,-23.30]=50mm 가 좌 +1.50 / 우 +3.50mm 여유로 이 개구부를 통과한다.
즉 **프레임이 봉투를 테두리로 물고 그 가운데로 플레이트가 지나가 분쇄한다**는 설계다.
`_add_leftwall_clamp_face()` 가 막은 것은 단차가 아니라 **이 개구부 55mm 를 통째로
메운 것**이었다. 상단 플랜지(z 81.43~86.43)는 플레이트 작동 z 보다 위라 무관하며
비산 방지 립이다(사용자 확인). **형상은 설계대로 옳다.**

따라서 압착면을 평탄화하려면 **프레임 부재 위에만** 단차를 메워야 한다(좌/우 세로부재
+ 하단 부재), 개구부는 비워둔 채로. 현재 `LEFTWALL_CLAMP_FACE` 구현은 개구부까지
메우므로 **쓰면 안 된다** — 기본 꺼짐으로 두고, 부재 한정 버전으로 다시 만들어야 한다.

> **남은 문제**: 프레임 부재(y=319.28)가 봉투에 닿으려면 벽이 17mm 를 닫아야 하는데,
> 5mm 앞선 상단 플랜지(y=324.28)가 12mm 에서 봉투에 먼저 걸려 벽을 세운다. 즉
> **플랜지가 세로부재의 압착을 가로막는다.** 프레임이 테두리로 무는 설계가 성립하려면
> 플랜지가 봉투 접촉에서 빠지거나 부재와 같은 평면이어야 한다.

### 18-3. 실링 두께는 `CLOTH_THICK` 의 2배 이상이어야 한다

`full_workflow.py` 가 쓰던 `Samplebag_seal_pouch3.stl` 은 X 컬럼 17개가 전부 6.000mm
인 **실링부 없는** 균일 파우치였다 — Phase 9 가 "실링부 압착"이라고 찍으면서 실제로는
파우치 몸통을 누르고 있었다.

recovery2 의 `_seal1mm`(가장자리 1.0mm)로 바꾸자 IPC build 가 죽었다:

```
Object[cloth_0_0] is too close (distance=0.000398, thickness=0.002) to itself
```

정점-삼각형 거리 실측으로 범인을 특정했다 — **측면·바닥 패널의 z=0 중간선**이다.
실링을 1mm 로 눌러도 중간선이 ±0.5mm 면 사이 정가운데 남아 자기간격이 **0.500mm**
로 반토막 난다(원본은 2.500mm).

`_sealslab*` 생성 절차(§16-7 "등두께 슬래브"의 실제 구현):

1. **z=0 중간선 57정점 제거** — 둘레(좌 21 / 바닥 17 / 우 21)를 front↔back 직결
   quad 로 재삼각화. 정점 771→714, 면 1504→1392.
2. 실링 대역을 **등두께 슬래브**로(비례축소 `V[:,2] *= f` 아님).
3. **테이퍼 연장** — 중간선을 없애고도 x 20→24(4mm) 테이퍼에서 0.759mm 에 막혔다.
   6mm→1mm 를 4mm 만에 좁히면 비틀린 quad 대각이 반대편 모서리에 접근한다.

| 실링 두께 | 테이퍼 시작 | 최소 자기거리 | 허용 `CLOTH_THICK` |
|---|---|---|---|
| 1.0mm | x=16 | 1.000mm | ≤ 0.500 |
| 2.0mm | x=12 | 1.803mm | ≤ 0.901 |
| **3.0mm** | **x=12** | **2.278mm** | **≤ 1.139** |

**제약**: IPC 요구가 자기간격 ≥ 2×`CLOTH_THICK` 이고 슬래브 실링의 자기간격이 곧
실링 두께다. **물리적으로도 같다 — 실링부는 앞뒤 필름이 용착된 자리라 두께가 곧
필름 2겹이다. 1.0mm 필름으로 1mm 실링은 성립할 수 없다.**

`_sealslab1mm`(실링 1.0mm)은 build 는 통과하지만 `CLOTH_THICK` 을 0.4mm 로 낮춰야
해서 **파지가 깨졌다** — lift 에서 그리퍼만 +126mm, 봉투 제자리 → tilt 78.3deg →
바닥 낙하(FAIL, 오차 −49.9mm). `FING_CLOSE=1.20` 이 1.0mm 기준이라 0.4mm 천에는
닿지 못한다(§17-7 의 0.1mm 실패와 같은 서명). 채택은 **`_sealslab3mm`**.

### 18-4. 결과

`_sealslab3mm` + `CLOTH_THICK=1.0mm` + 힘 상한 100N + (당시) 평탄 압착면:

| 지표 | 평탄화 전 · 실링 없음 | 이번 |
|---|---|---|
| 판정 | PASS | PASS |
| 압착 잔여 간격 | 7.03mm | **4.69mm** |
| release 후 tilt | 6.9deg | **0.7deg** |
| release 드리프트 | 2.9mm | **1.1mm** |
| 폭 / 높이 | 67.5 / 90.1mm | 64.5 / 92.1mm (baseline 64.3 / 90.1) |
| 계측 | 267.4s | 269.0s (2430스텝, 106.5 ms/step) |

box 를 끈(= 분쇄와 양립하는) 조건으로 재측정한 결과가 아래다. **트레이드오프가
그대로 드러난다** — 삽입 정확도는 좋아지고 고정은 나빠진다.

| 지표 | box ON (분쇄 불가) | **box OFF (분쇄 가능)** |
|---|---|---|
| 판정 | PASS | PASS |
| 삽입 오차 | −2.0mm | **−0.7mm** (세션 최고) |
| 벽 정지 위치 | −7.31mm | −7.88mm |
| 잔여(플랜지면 / 본체면) | 4.69 / 4.69mm | 4.12 / **9.12mm** |
| release 후 tilt | **0.7deg** | 5.8deg |
| release 드리프트 | **1.1mm** | 2.9mm |
| 폭 / 높이 | 64.5 / 92.1mm | 64.8 / 90.1mm (baseline 64.3 / 90.1) |
| 계측 | 269.0s | 289.8s (2430스텝, 115.2 ms/step) |

box 를 끄면 본체 면 잔여가 9.12mm 로 8mm 봉투에 닿지 않아 **고정은 다시 상단
플랜지 한 줄로만 이뤄진다**(tilt 5.8deg, 드리프트 2.9mm — box 이전 수치로 복귀).
삽입 오차가 −0.7mm 로 좋아진 것은 벽이 봉투를 덜 밀기 때문이지 고정이 나아져서가
아니다.

**이것이 §18-2 가 지적한 구조적 모순의 정량이다** — 봉투를 면으로 물면 분쇄가 막히고,
분쇄를 열면 고정이 한 줄로 돌아간다. Left_Wall 의 역할이 확정되기 전에는 이 둘을
동시에 만족시킬 수 없다.

부수 수정: clamp 로그가 잔여 간격을 `0.012 + wq_final` 하나로만 찍고 있었다. 면마다
출발 간격이 다르므로(플랜지 12.00 / 본체 17.00mm) 둘 다 출력하도록 고쳤다 — 이
하드코딩이 §18-2 의 AABB 오독을 로그에서까지 재생산하고 있었다.

### 18-5. 상단 플랜지 — 비산 방지 립이 세로부재의 압착을 가로막는다

§18-2 의 정정으로 `L2_Left_Wall1_1` 이 **ㅁ 프레임**임이 확인됐다. 그러면 압착 구조는
이렇게 읽힌다 — **프레임이 봉투를 테두리로 물고, 그 가운데 55mm 개구부로 impact
plate 가 지나가 분쇄한다.** 접촉을 끄거나 개구부를 메우는 처방은 전부 불필요하다.

남은 것은 **높이 방향 단차** 하나다.

| 부재 | z 범위 | 면 y | 고정 3벽(336.28)까지 |
|---|---|---|---|
| 상단 플랜지 (비산 방지 립) | 81.43 ~ 86.43 | **324.28** | **12.00mm** |
| 좌/우 세로부재 | 24.16 ~ 81.39 | 319.28 | 17.00mm |

세로부재가 8mm 봉투에 닿으려면 벽이 17mm 를 닫아야 하는데, 5mm 앞선 플랜지가
12mm 지점에서 봉투에 먼저 걸려 벽을 세운다. **플랜지가 세로부재의 압착을 가로막는
것**이 압착이 한 줄로만 걸리는 진짜 이유다(box OFF 실측: tilt 5.8deg, 드리프트 2.9mm).

플랜지 자체는 **비산 방지 설계**이고(사용자 확인) 플레이트 작동 z(24.43~74.43)보다
위라 분쇄와는 무관하다. 즉 없애야 할 부품이 아니라 **압착 경로에서 빠져야 할
부품**이다.

**처방 — `coup_collision_links` 제외가 아니라 별도 링크로 분리한다(사용자 지시).**
접촉만 끄는 것은 실재하는 부품을 없는 셈 치는 것이라 트윈 취지에 어긋난다. Fusion
에서 플랜지를 독립 컴포넌트로 잘라 별도 링크로 export 하는 것이 옳다.

다만 **분리 자체가 해를 주지는 않는다** — 분리한 링크를 어디에 붙이느냐가 결정한다:

| 부모 | 결과 |
|---|---|
| 고정 프레임 | 플랜지가 제자리에 머물고 벽만 닫힌다. 세로부재가 17mm 를 다 닫아 봉투를 테두리로 물고, 플랜지는 슬롯 상단에서 비산 방지만 한다. **해결** |
| Left_Wall (weld) | 링크만 나뉠 뿐 기구학이 동일해 간섭이 그대로 남는다 |

**실기에서 플랜지가 벽과 함께 움직이는 부품인지 프레임에 고정된 부품인지가 갈림길
이다** — 이것이 확정돼야 Phase 9 를 마무리할 수 있다. 비산 방지가 목적이라면 슬롯
상단에 고정된 쪽이 자연스러워 보이나 CAD 확인이 필요하다. 재export 시 플랜지를
독립 링크 이름으로 빼주면 시뮬 쪽 배선은 그에 맞춘다.

## 19. 실설계 배치 정합 — STEP 실측으로 좌표 재수립 (2026-08-28)

`Hardware_setup.step`(56MB, AP214, 부품 184 / 배치 319)과 `Base_ver2.step` 을 파싱해
sim 배치를 실설계로 맞췄다. 기존 좌표는 전부 "정렬점 빼기"로 손 역산한 값이라 실제
설계와 수백 mm 어긋나 있었다(부품 쌍 거리 비교: 석션V1~로봇 504.7 vs 1069.0mm 등).

### 19-1. 방법 — 부품 bbox 대조로 프레임 변환을 실측한다

STEP 컴포넌트 배치만으로는 부족하다. MJCF 애셋의 원점이 STEP 컴포넌트 원점과 다르고,
MJCF 가 자세를 이미 품고 있을 수도 있기 때문이다. 그래서 **같은 부품의 bbox 를 STEP
어셈블리 좌표와 MJCF 로컬 좌표에서 각각 재고 lo/hi 오프셋을 비교**했다 — 둘이 일치
하면 회전 성분이 0(순수 평행이동)이라는 뜻이다.

| 조립품 | MJCF→어셈블리 평행이동(mm) | 회전 | 대조 부품 / 편차 |
|---|---|---|---|
| Crusher | (−115.20, 64.72, 30.57) | 없음 | `1_Wall1`·`2_Wall3` 표준편차 **0** |
| 고정장치 | (−315.00, 52.50, 30.00) | 없음 | `MotorDriver` 표준편차 **0** |
| 회수장치2 | (−378.99, 65.78, 182.52) | 없음 | `M_Top`·`F_Top`·`RachetGear` 0.002mm |
| 석션V1 | (−366.55, 202.50, 30.00) | **Z+90** | `Hbeam_L/B/M`·`Dummy` — lo/hi 가 X↔Y 스왑 |
| 로봇 M0609 | (−600.01, 650.00, 30.00) | 없음 | `BASE_M0609` **+X·+Y·+Z 세 면 일치** |

**Crusher 의 EULER 은 0 이다.** MJCF 의 모든 geom 이 `quat="0.5 0.5 0.5 0.5"`
(X→+Y, Y→+Z, Z→+X)를 갖고 있고 이것이 STEP 의 Crusher 자세 (90,0,90)과 같은 회전
이다. 기존 `(0,0,90)` 이나 STEP 값 `(90,0,90)` 을 주면 이중 회전이 된다.
검산: 이 변환에서 MJCF 원점의 z 가 판 상면 **+0.57mm** — 판 위에 정확히 얹힌다.

로봇은 bbox **치수**가 안 맞아(206×431×97 vs 182×182×52) 처음엔 회전이 있는 것처럼
보였다. 실제로는 MJCF 베이스 메시에 후면 케이블 박스가 포함(−X 24.07 / −Y 248.56 /
−Z 45.25mm)돼 있을 뿐이고, **hi 코너 오프셋이 STEP 배치값과 소수점까지 일치**한다.
그 케이블 박스는 시각 전용 geom(`contype=0 conaffinity=0`)이라 물리엔 무해하나
렌더에서 판을 45mm 관통한다.

### 19-2. 알루미늄 판 — 750 × 800 × 30mm, D6.0 구멍 240개

`Base_ver2.step` 실측: 판 로컬 X[0,750] Y[0,800] Z[0,30], 원통면 240개가 전부
반지름 3.000mm · 축 +Z. 구멍 XY 는 **15 × 16 격자, 피치 50mm, 가장자리 여백 25mm**.

어셈블리에서 `Base` 배치가 (−750, 0, 0)이라 판이 X[−750,0] Y[0,800] Z[0,30] 을
차지하고 **상면이 정확히 Z=30** — 고정장치·석션V1·로봇 BASE·Crusher base_link 가
전부 Z=30 에 있는 것과 일치한다. 판 두께가 곧 그 30mm 였다.

**월드 원점을 판 상면 중심**(어셈블리 (−375, 400, 30))으로 잡는다. 실물 장착
기준면이라 판 상면이 z=0 이 되어 해석이 깔끔하다.

| 조립품 | POS (m) | EULER |
|---|---|---|
| Crusher | (+0.2598, −0.3353, +0.00057) | (0, 0, 0) |
| 고정장치 | (+0.0600, −0.3475, 0) | (0, 0, 0) |
| 회수장치2 | (−0.0040, −0.3342, +0.1525) | (0, 0, 0) |
| 석션V1 | (+0.0085, −0.1975, 0) | (0, 0, 90) |
| 로봇 M0609 | (−0.2250, +0.2500, 0) | (0, 0, 0) |

판은 **충돌을 구멍 없는 box primitive** 로 둔다(사용자 합의) — 구멍 240개는 IPC
비용만 올리고 접촉에 기여하지 않는다. 시각용 실메시는 STEP 테셀레이터(cadquery /
gmsh) 미설치로 보류.

### 19-3. 슬롯 간격 축이 하드코딩돼 있었다

실배치로 바꾸자 `gap_width` 가 12mm 대신 **80mm** 로 잡혔다. 원인은

```python
gap_lo_x, gap_hi_x = sorted([wb_hi[0], wl_lo[0]])   # [0] = world X 고정
```

간격이 world X 에 있다는 전제인데, 그건 `CRUSHER_EULER=(0,0,90)` 일 때만 참이다.
실배치(EULER 0)에서는 간격이 **world Y**, 슬롯 길이가 **world X** 로 뒤바뀐다.
두 벽 AABB 가 겹치지 않는 축을 찾아 판별하도록 고쳤다 — 자세에 의존하지 않는다.

### 19-4. 봉투 자세도 축을 따라 뒤집어야 한다

§9(2026-07-16)에 정확히 반대 방향으로 한 번 겪은 문제다.

```
BAG_EULER (90,0,90) -> X=6(두께)  Y=64(폭)   : 구 배치(간격축 X)
BAG_EULER (90,0, 0) -> X=64(폭)   Y=6(두께)  : 실배치(간격축 Y)
```

되돌리지 않으면 폭 64mm 가 12.5mm 틈에 들어가려다 구겨진다(실측: `baseline
width_y=64.3mm`, insert tilt 22.8deg, 높이 90.1→76.1mm, FAIL +25.0mm).
`BAG_EULER=(90,0,0)` + 손목 트위스트(joint 6) +90도 복원 후 `baseline width_y` 가
**6.4mm** 로 바뀌어 두께가 간격축을 향하는 것을 확인했다.

### 19-5. 삽입 실패의 원인 — IK 목표 자세가 손목을 되돌리고 있었다 **[해결]**

봉투 자세를 고쳐도 삽입은 계속 실패했다(insert tilt 22.8 -> 25.2deg). 손목 부호를
의심해 `WRIST6_DEG=-90` 을 돌렸더니 **더 나빠졌는데(tilt 30.5deg), 그게 원인을
알려줬다** — 하강 중 봉투가 X 로 밀리는 **부호가 손목 부호를 따라 뒤집혔다**
(+90 -> -9.4mm, -90 -> +10.2mm). 벽 간섭이면 부호가 안 바뀐다.

범인은 `q_insert_quat = VERTICAL_QUAT` 이라는 **고정 상수**였다. `WRIST6_DEG` 를
`Q_GRASP`/`Q_LIFT` 에만 더했더니:

```
grasp/lift   : 손목 +90deg  -> 봉투를 두께면으로 제대로 뭄
above/insert : IK 가 VERTICAL_QUAT 지령 -> 손목을 0deg 로 되돌림
               -> 봉투가 이송 중 90deg 비틀리며 끌려감
```

파지·리프트까지 tilt 1deg 이하로 완벽했다가 `above` 이후 무너진 것이 이걸로 설명된다.
`VERTICAL_QUAT` 을 world Z 둘레로 `WRIST6_DEG` 만큼 함께 돌려 파지 자세와 삽입
자세를 일치시켰다.

**결과: insert tilt 25.2 -> 2.4deg, 오차 -0.5mm.** 네 런 연속 재현됐고, 구 배치의
검증값(-0.7mm)보다도 좋다. 실배치에서 삽입은 확정으로 본다.

### 19-6. 클램프는 힘·마찰 부족이 아니라 **강성 과다**다 **[미해결]**

삽입이 정확해지자 클램프가 실제로 일을 해야 하는 상황이 처음 만들어졌고, 거기서
막혔다. `CLAMP_TARGET` 스윕(간격: 플랜지 12.50 / 본체 17.50mm):

| CLAMP_TARGET | 플랜지 잔여 | 본체 잔여 | 결과 |
|---|---|---|---|
| −8.5mm | 4.08mm | 9.08mm | 안정. 단 **본체면 미접촉** → 그리퍼 열면 봉투 −34.9mm 낙하 |
| −10.0mm | 2.50mm | 7.50mm | **발산** `wall=-104.77mm` |
| −10.5mm | 2.00mm | 7.00mm | **발산** `wall=+625.33mm` |
| −12.0mm | 0.50mm | 5.50mm | 배리어 폭주, 78분 미완 |

본체면이 8mm 봉투에 닿으려면 `q < -9.5mm` 인데, 그 지점부터 플랜지 잔여가 3mm
아래로 내려가며 터진다. **두 조건이 양립하는 창이 없다.**

**힘도 마찰도 원인이 아니다.** 봉투 질량은 `CLOTH_RHO 200 x 1.0mm x 12394mm^2`
= 약 2.5g, 무게 0.025N 이다. 100N 은 그 4000배다.

두 접촉의 설정을 나란히 놓으면 **같은 조건이 아니었다**:

| | 그리퍼(성공) | Left_Wall(실패) |
|---|---|---|
| kp | **30** | **5000** (167배) |
| kv | 1.5 | 500 |
| 힘 상한 | 2.0 N·m ≈ 40 N | 100 N |
| `coup_friction` | 0.8 (명시) | 미지정 → 기본 **0.1** |
| 접촉 | 양쪽 손가락이 대칭 압착 | 한쪽 벽, **플랜지 높이엔 마주보는 면 없음** |

`kp=30` 은 매우 무른 위치제어라 봉투에 닿으면 거기서 멈춘다 — 사실상 힘제어에 가까운
파지다(`m0609_rg2_v2.xml` 주석: "OnRobot 데이터시트 gripping force 3-40N"). 반면
`kp=5000` 은 0.1mm 오차에 500N 을 요구해 힘 상한에 즉시 포화되고, **매 스텝 최대
힘으로 밀어붙이는 상태**가 되어 IPC 배리어와 정면 충돌한다. 천을 눌러 멈추는 접촉이
아니라 뚫으려는 접촉이었다.

**그리퍼 레시피 이식 시도 — 이것만으로는 부족했다.** `WALL_KP` 30 / `WALL_KV` 1.5 /
힘 40N / `coup_friction` 0.8 로 맞추고 `CLAMP_TARGET=-10.5mm` 로 돌렸으나 clamp 에서
**19분 무진행**으로 중단했다(CPU 1345s). 삽입은 정상(tilt 2.9deg, bag_bottom 0.0505).
강성만 낮춰서는 해결되지 않는다.

남은 것은 결국 §18-2 의 기하다 — **플랜지가 허공을 누른다.**

```
Wall3(고정벽)  z 14.43 ~ 80.43mm
플랜지         z 81.43 ~ 86.43mm   <- Wall3 상단보다 위, 마주보는 면이 없음
본체면         z  9.43 ~ 81.43mm   <- Wall3 와 겹침 = 진짜 압착면
```

플랜지 높이에 반대편이 없으니 눌러도 반력이 안 생기고 봉투가 옆으로 밀린다(clamp 중
Y 로 +3.7mm 이동 실측). 그리고 진짜 압착면인 본체면은 5mm 물러나 있어 닿기 전에
플랜지가 먼저 봉투를 뭉갠다. **플랜지를 봉투 접촉에서 빼거나 고정 프레임에 붙이는
것이 선행돼야 한다**(§18-5).

### 19-7. 구 배치의 "PASS" 는 클램프가 잡아서가 아니었다

중요한 정정이다. §18-4 의 box OFF 런(오차 -0.7mm, PASS)도 **클램프가 봉투를 잡은
적이 없다** — 그때 기록이 "고정이 다시 상단 플랜지 한 줄로만 이뤄진다(tilt 5.8deg,
드리프트 2.9mm)"였다. 봉투가 안 떨어진 건 삽입이 얕아 **벽 위에 걸쳐 있었기**
때문이다.

```
구 배치  insert tilt 15~25deg, 높이 90 -> 76mm  -> 구겨진 채 끼임 -> 안 떨어짐
실배치   insert tilt 2.4deg,  높이 유지         -> 슬롯에 제대로 들어감 -> 놓으면 흘러내림
```

구 배치의 실패는 항상 **+방향(얕음)** 이었는데 실배치는 **-34.9mm(깊음)** 다. 삽입을
고치면서 "벽에 끼여 안 떨어지던" 눈속임이 사라졌고, 클램프가 실제로 일해야 하는
상황이 처음 만들어진 것이다. 판정 기준(`bag_bottom_z` 도달 + 붕괴 없음)이 클램프
고정력을 재지 않는다는 한계도 함께 드러났다.

### 19-8. IPC 는 `contype`/`conaffinity` 를 무시한다

`coup_links` 를 논하며 확인한 사실. Crusher MJCF 는 body 8개 / geom 53개인데 그중
**36개가 `contype=0` 시각 전용**이다. MuJoCo 리지드 솔버는 그 마스크를 지키지만
**IPC 는 안 본다** — 링크가 IPC 에 들어가는 루프에 마스크 검사가 아예 없고 유일한
필터가 `coup_links` 다(`coupler.py` 소스 확인).

```python
for link in self.rigid_solver.links:
    if coup_type == COUPLING_TYPE.TWO_WAY_SOFT_CONSTRAINT:
        link_filter = self._coup_links.get(entity)
        if link_filter is not None and link not in link_filter:
            continue
    ...   # geom 수집 — contype/conaffinity 검사 없음
```

§9 조합9 후속4 의 실측과도 일치한다 — 석션V1 조 링크를 비트마스크로 빼려다
"효과 없음"으로 끝났던 그 건이다. 즉 **시각 전용 geom 36개도 IPC 안에서는 살아
있다.** 비용을 줄이려면 `coup_links` 경로밖에 없다(§16-7).

다만 Crusher 에 `coup_links` 를 걸면 분쇄 단계에서 `L9_PLATE` 가 정제를 못 때릴
위험이 있으므로, 압착이 성립한 뒤에 다뤄야 한다.

### 19-6. 안전장치

기존 검증 배치를 깨지 않도록 **`LAYOUT_FROM_STEP` / `LAYOUT_ONLY`** 환경변수로
분리했다. 기본값은 예전 그대로라 `full_workflow.py` 를 그냥 돌리면 §18 까지의
검증 파이프라인(중앙 파지 −0.7mm)이 유지된다. `LAYOUT_ONLY=1` 은 시뮬 없이 배치
정지 프레임(iso/top/front/side)만 뽑고 끝낸다.

## 20. 압착 성립 — 원인은 기하가 아니라 **구동 방식**이었다 (2026-08-31)

Phase 9 클램프가 처음으로 성립했다. 판정 지표는 **`drop`** — 그리퍼를 놓은 뒤
봉투가 흘러내린 양이다(§18-4 의 실패 실측 −34.9mm). 이 값이 0 이면 벽이 봉투를
잡고 있다는 뜻이다.

### 20-1. 결과 — 힘제어 7/7 실패, 기구학 구동 10/10 성공

야간 스윕(레인 2개, 런당 25분 타임아웃, `NO_VIDEO=1`). clamp 까지 도달한 런만.

**힘제어(`control_dofs_position`) — 전부 실패, 그리고 전부 같은 값**

| 힘 상한 | 닫힘 속도 | wall 도달 | 본체면 잔여 | drop |
|---|---|---|---|---|
| 100 N | 8 mm/s | −16.99mm | 0.51mm | −27.5mm |
| 200 N | 8 mm/s | −16.99mm | 0.51mm | −26.7mm |
| 400 N | 8 mm/s | −16.99mm | 0.51mm | −26.6mm |
| 800 N | 8 mm/s | −16.99mm | 0.51mm | −20.3mm |
| 100 N | 2 mm/s | −16.99mm | 0.51mm | −27.1mm |
| 200 N | 4 mm/s | −16.99mm | 0.51mm | −26.6mm |

**힘이 부족한 게 아니다 — 반대다.** 넷 다 하드스톱(−17.0mm)까지 완전히 닫혔다.
힘이 모자랐다면 중간에 섰어야 하는데, 봉투를 누르는 대신 **뚫고 지나가서** 끝까지
닫혔다. 힘을 8배로 올려도 결과가 소수점까지 같다.

**기구학 구동(`set_dofs_position`) — 전 깊이 성공**

| CLAMP_TARGET | 플랜지 | wall 도달 | 본체면 잔여 | drop | reach_err | tilt |
|---|---|---|---|---|---|---|
| −11.5mm | OFF | −11.34mm | 6.16mm | −0.0mm | −2.0 / −1.9mm | 9.4 / 9.2° |
| −12.5mm | OFF | −12.33mm | 5.17mm | −0.0mm | −1.9mm | 9.6 / 9.7° |
| **−12.5mm** | **ON** | −13.04mm | 4.46mm | **+0.0mm** | **+0.3mm** | **7.5 / 7.3°** |
| −13.5mm | OFF | −13.48mm | 4.02mm | −0.0mm | −2.4mm | 9.0 / 9.1° |
| −14.5mm | OFF | −14.94mm | 2.56mm | −0.0mm | −2.0mm | 7.2° |
| −15.5mm | OFF | −16.53mm | 0.97mm | −0.0mm | −1.9mm | 9.3° |

### 20-2. 관통의 정체 — 임펄스가 액추에이터에서 오지 않는다

첫 접촉 순간의 궤적(`f1` 이전 힘제어 런):

```
잔여 +9.24mm   v =    -8.16 mm/s     지령(-8.0)대로 추종
잔여 +7.74mm   v =   -16.17 mm/s     봉투에 닿자마자 가속
wall  -373mm   v = -3236.91 mm/s     관통
```

이 가속에 필요한 힘을 역산하면 **149 N** 인데 상한은 100 N 이었다. 즉 벽을
밀어붙인 것은 액추에이터가 아니라 **접촉 솔버가 주입한 임펄스**다. 힘 상한을
올려도 그 임펄스의 크기는 안 변하므로 800N 이 100N 과 같은 결과를 낸 것이다.

기여 조건 두 가지도 같이 나왔다.

- **벽 링크 질량이 0.312 kg 뿐이다.** 100 N 이 321 m/s² 이라 dt=5ms 한 스텝에
  +1,603 mm/s 다. 작은 교란 하나가 그대로 관통이 된다.
- **`WALL_KV=500` 은 명시적 감쇠의 안정 한계 `2m/dt = 125` 의 4배다.** 접촉
  전에는 드러나지 않다가 닿는 순간 발산한다.

### 20-3. 플랜지 제거는 필요조건이 아니었다 [§18-5 정정]

`flange=ON` 대조군도 통과했고 **수치는 오히려 더 좋다**(reach_err +0.3mm vs
−1.9mm, tilt 7.5° vs 9.6°, 2회 재현). §18-5 의 "플랜지가 허공을 눌러 균형점이
없다"는 분석은 **힘제어에서 왜 발산했는지**로는 맞지만, 해결책은 기하가 아니라
구동 방식이었다. 벽이 되밀릴 수 없으면 플랜지 높이에 마주보는 면이 없다는 사실
자체가 무의미해진다. 실기 형상을 그대로 두는 편이 낫다.

`LEFTWALL_FLANGE_CONTACT` 는 환경변수로 남아 있다(기본: 실배치면 꺼짐). 위
결과에 따르면 **기본값을 켬으로 되돌리는 것이 맞다.**

### 20-4. [미해결] `set_dofs_position` 은 임시방편이다 — 힘제어로 돌아가야 한다

**이것이 이 절의 가장 중요한 남은 과제다.** `WALL_KINEMATIC=1` 은 DOF 좌표를 매
스텝 덮어쓰므로 다음을 잃는다:

- **벽에 반력이 안 걸린다.** 봉투가 벽을 되미는 힘이 어디에도 반영되지 않는다.
- **접촉력을 못 읽는다.** 힘 상한이 정지 기준이 아니게 되므로 "정지 위치 = 실효
  압착 두께 실측값"이라는 §11-5 의 측정 논리가 통째로 사라진다. 압착력을 재려면
  힘제어여야 한다.
- **어디까지 닫을지를 사람이 정해야 한다.** 재려던 값(압착 깊이)을 입력으로
  넣는 순환이 남는다.

실기 정당화는 있다 — Motor2 는 6 RPM 대감속 랙-피니언이라 봉투 반력으로
역구동되지 않으므로 기구학 경계조건이 아주 틀린 모델은 아니다. 그래도 **압착력을
수치로 내야 하는 이상 `control_dofs_position` 으로 돌아가는 것이 목표다.**

되돌리기 위해 확인해야 할 축 두 개가 **아직 결과가 없다**:

1. **반사 관성이 실제로 솔버에 적용됐는지.** 야간 스윕은 MJCF 의 `armature`
   속성으로만 걸었고 그 값이 솔버까지 갔는지 확인하지 않았다. Genesis 에는
   `entity.set_dofs_armature()` 런타임 API 가 따로 있다(rigid_entity.py:3982).
   MJCF 경로가 무시됐다면 "관성을 키워도 관통한다"는 결론 자체가 무효다.
   현재는 런타임 API 로도 걸도록 고쳐 뒀고 로그에 유효 질량을 찍는다.
   랙-피니언 감속기의 반사 관성은 실제로 10²~10³ kg 스케일이라, 1,000 kg 이면
   149 N·0.17s 임펄스가 속도를 0.7 mm/s 밖에 못 바꾼다.
2. **IPC 소프트 구속 강성(`IPC_CONSTRAINT_STRENGTH`).** Crusher 는
   `two_way_soft_constraint` 로 실리는데, 강체 솔버의 지령 자세와 uipc 프록시를
   묶는 이 스프링이 양방향 되먹임 루프를 만든다. 레인 E 가 이 축을 맡았으나
   병렬 런의 파일 경합(§20-5)으로 **전부 죽어 데이터가 0건이다.**

#### 20-4-1. 반사 관성은 실제로 효과가 있었다 [야간 결론 무효]

런타임 API 로 다시 걸어 확인했다(`h1`: armature=100kg, 유효 질량 100.31kg).

| | 관통 속도 | 도달 위치 |
|---|---|---|
| 관성 없음(0.312kg) | −3,236.91 mm/s | −373 mm |
| **armature=100kg** | **−143.39 mm/s** | **−17.63 mm** |

**23배 줄었다.** 야간 스윕의 "관성을 키워도 관통한다"는 결론은 MJCF `armature`
속성이 솔버에 반영되지 않은 결과였을 가능성이 크다 — 그때는 검증하지 않았다.

#### 20-4-2. [핵심 가설] 힘 상한 100 N 은 봉투가 만들 수 없는 힘이다

그런데 `h1` 은 여전히 넘어갔고, 이번에는 역산이 다르게 나온다.

    100.31 kg 을 −9.5 → −143 mm/s 로 가속하는 데 필요한 힘 = **79 N** < 100 N 상한

즉 이번 오버슛은 접촉 솔버가 아니라 **액추에이터가 낸 것**이다. 그리고 여기서
전제 자체의 오류가 드러난다 — §11-5 의 "벽이 봉투 반력과 균형지는 곳에서
멈춘다"가 성립하려면 봉투가 그 힘을 만들어야 하는데, **2.5 g 짜리 필름은 어떤
압착 두께에서도 100 N 을 만들지 못한다.** 균형점이 존재하지 않으니 벽은 항상
끝까지 닫히고, 그 뒤 잔여 −0.13mm(관통 상태)에서 배리어가 갈린다(실측: 그 자세로
14분 무진행).

**±100 N 은 Motor2 의 최대 스펙이지 운전 힘이 아니다.** 그런데 §13-11 이래로
그것을 운전값으로 써 왔고, 야간 스윕은 100 → 800 N 으로 **올리기만** 했다.
100 N **아래는 한 번도 보지 않았다.** 참고로 같은 IPC 스택에서 성공하는
그리퍼는 **40 N** 으로 봉투를 잡는다.

**다음 작업 — `sweep_force.sh` (아직 결과 없음).** armature=100 고정, 플랜지 ON,
`control_dofs_position` 힘제어로 `WALL_FORCE_LIM_N` 을 **2 / 5 / 10 / 20 / 40 N**
훑는다. `stalled=Y`(하드스톱 전에 스스로 멈춤) + `drop≈0` 이 나오는 구간이 있으면
`set` 없이 힘제어로 성립하는 것이고, **그 정지 위치가 곧 실효 압착 두께이자
필요 압착력의 첫 측정값**이 된다.

### 20-5. 스윕 운영에서 나온 것

- **병렬 런이 `RESULT/` 중간 산출물을 공유해 서로 죽인다.** `_bag_seal_uv.obj`,
  `_analytic_capsule_v2.stl`, `material_0.png` 는 매 런이 새로 굽는데 경로가
  고정이라, 두 런이 겹치면 `ValueError: need at least one array to concatenate`
  로 죽는다. 프로세스를 중간에 죽이면 잘린 파일이 남아 이후 런까지 전부 오염된다.
  → **`RUN_TAG`** 로 런마다 `RESULT_<tag>/` 를 분리했다.
- **이 기계의 병렬 상한은 2개다**(RAM 16.8GB, 런당 3.6GB / VRAM 12.2GB, 런당
  약 3.5GB). 3~4개로 과다구독하면 `above`(이송) 구간이 정체해 clamp 까지 가지
  못한다. 야간 스윕의 STALL 17건은 전부 이것이며 압착 실패가 아니다 — 단일
  레인으로 다시 돌리자 같은 설정이 **런당 205초**에 완주했다(타임아웃 1,500초).
- **`NO_VIDEO=1`** 로 카메라 3대 렌더를 끄면 수치만 필요한 스윕이 크게 빨라진다.
- **`CLAMP_ONLY`(팔 시퀀스 생략)는 실패했다.** 봉투를 슬롯에 스폰하면 정점 구속을
  걸든 안 걸든 settle 구간에서 정체한다. 빌드 후 FEM 봉투를 옮기는 것도 불가능
  하다 — IPC 커플러가 매 스텝 uipc 상태를 FEM 엔티티로 되쓰기 때문에
  (`coupler.py:1041 entity.set_pos(0, geom_positions)`) Genesis 쪽
  `set_position` 은 다음 스텝에 그대로 덮인다(실측: 이동량 303mm 가 통째로 무시).
  코드는 남아 있으나 기본 꺼짐이다.

### 20-6. 현재 권고 설정

```
LAYOUT_FROM_STEP=1 WALL_KINEMATIC=1 CLAMP_TARGET_MM=-12.5 LEFTWALL_FLANGE_CONTACT=1
```

단, §20-4 대로 `WALL_KINEMATIC` 은 임시방편이다. 힘제어 복귀가 확인되면 이
줄에서 그 항목이 빠져야 한다.
