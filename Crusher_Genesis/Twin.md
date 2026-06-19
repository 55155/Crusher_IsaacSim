# Digital Twin 기법 노트 — M0609 + OnRobot RG2

실제 하드웨어(Doosan **M0609** + OnRobot **RG2-v2**)를 Genesis(PBD/Rigid) 상에
디지털 트윈으로 옮기면서 사용한 모델링 기법을 정리한다.

---

## 1. 평행 그리퍼의 개·폐 자유도: open-loop vs closed-loop

### 실제 RG2 (closed-loop, 1-DOF)
실물 RG2 는 **모터 1개**가 **4절 링크(four-bar linkage)**를 구동해 두 핑거가
항상 **평행을 유지하며 대칭으로** 열리고 닫힌다. 즉 기구학적으로 **닫힌 루프(closed
kinematic loop)**이고, 제어 자유도는 **1개**(폭 하나)다.

### 단순화 트윈 (open-loop tree + 제약)
시뮬에서 4절 링크를 그대로 모델링하면 닫힌 루프라 안정성·속도에서 불리하다.
그래서 핑거를 **독립된 prismatic 조인트 2개**로 단순화한다:

```xml
<body name="rg2_left"  pos="0.105  0.017 0">
  <joint name="rg2_left_joint"  type="slide" axis="0 1 0"  range="0 0.045"/>
  <geom mesh="rg2_finger" .../>
</body>
<body name="rg2_right" pos="0.105 -0.017 0">
  <joint name="rg2_right_joint" type="slide" axis="0 -1 0" range="0 0.045"/>
  <geom mesh="rg2_finger" euler="180 0 0" .../>   <!-- mesh 미러 -->
</body>
```

이 상태는 **두 개의 독립 DOF**를 가진 **open-loop tree**(닫힌 루프 없음)다.
좌·우가 따로 노는 셈이라, 그대로 두면 한쪽만 움직이거나 비대칭으로 닫힐 수 있다.
실제 1-DOF 거동을 복원하려면 **두 조인트를 묶는 제약**이 필요하다.

---

## 2. Equality constraint — 어디에, 어떻게 걸었나

MJCF 최상위의 `<equality>` 블록에 **joint–joint equality**를 건다:

```xml
<equality>
  <joint joint1="rg2_left_joint" joint2="rg2_right_joint" polycoef="0 1 0 0 0"/>
</equality>
```

### 의미 (polycoef 다항식)
MuJoCo 의 joint equality 는 다음을 강제한다:

```
left = a0 + a1·right + a2·right² + a3·right³ + a4·right⁴
       (polycoef = [a0, a1, a2, a3, a4])
```

`polycoef="0 1 0 0 0"` → **left = right**. 두 핑거 변위가 항상 같다.
(만약 비대칭 기어비라면 a1 을 바꾸고, 오프셋이 있으면 a0 를 준다.)

### 솔버 관점
- 이 제약은 **운동학 트리(kinematic tree)에 박혀 있는 게 아니라**, 매 스텝
  **constraint solver** 가 Lagrange-multiplier(soft, `solimp`/`solref`)로 푼다.
- 효과적으로 2-DOF 핑거계를 **1 effective DOF** 로 줄인다 → 실물 1-DOF 와 동일.
- 그래서 **단일 액추에이터**(아래 Panda 패턴)로 폭 하나만 명령해도 양쪽이 대칭으로
  움직인다. 접촉/외력이 비대칭으로 들어와도 제약이 좌우를 맞춰준다.

### "open-loop 인데 왜 제약?" 한 줄 요약
> prismatic 2-조인트는 **기구학적으로 열린 구조**라 좌우가 독립이다.
> equality constraint 가 그 두 DOF 를 **솔버 레벨에서 1-DOF 로 묶어**,
> 실물의 닫힌-루프 1-DOF 대칭 거동을 복원하는 장치다.

---

## 3. 참조한 표준 패턴 — Franka Panda 그리퍼

Genesis `assets/xml/franka_emika_panda/panda.xml` 의 그리퍼가 정확히 이 패턴:

```xml
<!-- 핑거 = slide(prismatic) -->
<default class="finger"><joint axis="0 1 0" type="slide" range="0 0.04"/></default>
...
<tendon>                                   <!-- ① split tendon: 두 조인트 평균 -->
  <fixed name="split">
    <joint joint="finger_joint1" coef="0.5"/>
    <joint joint="finger_joint2" coef="0.5"/>
  </fixed>
</tendon>
<equality>                                 <!-- ② equality: 좌우 동기화 -->
  <joint joint1="finger_joint1" joint2="finger_joint2"/>
</equality>
<actuator>                                 <!-- ③ 단일 actuator 가 tendon 구동 -->
  <general name="actuator8" tendon="split" ctrlrange="0 255" .../>
</actuator>
```

- ① **tendon `split`** : 두 조인트를 가중합으로 묶어 **하나의 제어 변수**로 노출.
- ② **equality** : 좌우 변위를 같게(대칭) 유지.
- ③ **단일 actuator** : tendon 을 구동 → 폭 하나만 제어.

RG2 트윈은 ②(equality)를 적용했고, 운동학(set_dofs_position) 제어에서는 두 핑거
DOF 에 같은 값을 직접 넣는다(①③의 단일-액추에이터는 force 제어로 갈 때 추가).
`inertial` 은 단순화값을 유지(요청대로 건드리지 않음).

---

## 4. PBD 천(봉투) 잡기 — weld(attach) 기법과 타이밍

PBD cloth 를 그리퍼로 "잡는" 방식은 두 가지:

| 방식 | 구현 | 특징 |
|---|---|---|
| **마찰 파지** | 핑거 collision + `rigid_pbd` 접촉 마찰 | 사실적이나 얇은 이중벽에서 **2.1 압착 불안정**(아래) |
| **attach(weld)** | `bag.fix_particles_to_link(link_idx, idx)` | 안정적. grip 파티클을 링크 프레임에 강체 고정 |

### attach 내부 동작 (Genesis LegacyCoupler)
- `kernel_attach_pbd_to_rigid_link`: 호출 시점에 각 파티클의 **링크 로컬 오프셋**을
  저장(`local_pos`).
- 매 스텝 `kernel_pbd_rigid_solve_animate_particles_by_link`:
  `target = link_T · local_pos`, `corrective_vel = (target − pos)·clamped_inv_dt`,
  `clamped_inv_dt = min(1/dt, 50)`.
- 즉 **weld 순간의 상대 자세가 그대로 고정**되고, 이후 링크를 따라간다.

### "잡을 때 봉투가 밀린다" — dt 냐 substep 이냐 (타이밍 분석)
밀림의 원인은 **weld 순간의 상대 속도/오차**다(`corrective_vel ∝ pos_error·50`,
그리고 `+ link_vel`). 따라서:

- **dt 축소**: 스텝당 변위·보정속도 overshoot 감소 → 도움은 되나 **증상 완화**.
- **substep 증가**: 스텝 내 제약 수렴 향상 → 접촉/보정 잔차 감소, 역시 보조적.
- **핵심(weld 타이밍)**: ① 그리퍼를 봉투에 정렬 → ② **봉투·그리퍼를 정지(≈0 속도)
  까지 안정화** → ③ 그 순간 weld(저장 오프셋 = 정지 자세, link_vel≈0 → `pos_error≈0`)
  → ④ 이후 이동. 이러면 kick 이 원천적으로 사라진다.

> 결론: **dt/substep 은 안정성 보조 수단**이고, 밀림의 근본 해법은 **"정지 상태에서
> weld" 타이밍**이다. 구현상 close 후 **settle 단계**를 넣어 속도가 가라앉은 뒤 attach,
> 그리고 들어올림은 그 후 시작한다. 마찰 파지로 갈 경우엔 dt↓·substep↑·solver iter↑
> 가 본질적으로 필요(2.1 참고).

### 2.1 압착 불안정 (참고)
얇은 이중벽(앞·뒤 패널 1cm)을 마찰로 누르면 두 층 파티클이 `particle_size` 이내로
접근 → PBD 충돌제약(`target_dist=particle_size`)과 그리퍼 침투해소가 충돌 →
폭발/튕김. 그래서 렌더에서는 핑거를 visual-only 로 두고 attach 로 잡는다.

---

## 5. 사전정의 Primitive(Box) 가 mesh 보다 충돌계산에 유리한 이유

| | Primitive (Box/Sphere/Cylinder/Capsule) | 임의 Mesh |
|---|---|---|
| 형상 정의 | **해석적**(중심+half-extent+자세) | 삼각형 수천 개 |
| 충돌 질의 | box-box/plane/sphere **closed-form**, GJK support 정확 | BVH 순회 + 근사 |
| SDF (PBD/MPM 커플링) | **해석적 exact SDF** | 복셀/삼각형 근사(해상도 오차) |
| 볼록성 | **항상 convex**(솔버 요구 충족) | 비볼록 → **convex 분해 필요**(CoACD/V-HACD) |
| 접촉 manifold | 면-면 = 안정적 4점, 노이즈 없음 | 다수 경쟁 접촉 → jitter |
| 비용 | O(1)/pair | O(faces), 분해 오버헤드 |

요지: **primitive = 정확·볼록·저비용·안정**. mesh 는 충돌용으로 쓰려면 **볼록 근사
(convexify/decompose)**가 들어가 근사·고비용·jitter 위험. 그래서 봉투 안 내용물은
`gs.morphs.Box` 로 넣었다(안정적 파지/적재).

### 내가 원하는 모양을 "사전정의" 하는 3가지 방법

**(A) Primitive 조합 — 가장 안정적**
- 원하는 형상을 box/sphere/cylinder/capsule **여러 개로 근사**해 한 body 에 묶는다.
  (L자=box2, 알약=capsule, 병=cylinder+sphere). MJCF 한 body 안에 `<geom>` 여러 개:
```xml
<body name="my_part">
  <geom type="box"      size="0.03 0.02 0.01" pos="0 0 0"/>
  <geom type="cylinder" size="0.01 0.04"      pos="0 0 0.05"/>
</body>
```
  Genesis: `gs.morphs.MJCF(file=...)` 또는 단일은 `gs.morphs.Box/Cylinder/Sphere(...)`.

**(B) Mesh + convex 분해**
- 임의 mesh 를 주고 Genesis 가 볼록 분해:
  `gs.morphs.Mesh(file=..., convexify=True)`(단일 볼록껍질) 또는 CoACD 옵션
  (`decompose_nonconvex`, `coacd_options`)로 다중 볼록 조각. 임의형상 OK, 근사.

**(C) visual ↔ collision 분리 — 로보틱스 표준**
- 정밀 mesh 는 **시각**(`contype=0 conaffinity=0`), 충돌은 **primitive**(box/cylinder).
  RG2 핑거·로봇 링크가 쓰는 방식. 보기 좋고 충돌 안정.

> 본 프로젝트: 봉투 내용물 = (A)의 단일 Box. 그리퍼 핑거 = (C). 알루미늄 플레이트는
> 슬랩이라 mesh 를 그대로 fixed rigid 로 써도 convexify 시 사실상 box 라 안정적.

---

## 6. 좌표/마운트 메모
- RG2 hand 의 **+x(공구축)** 을 link_6 **+z(플랜지 법선)** 에 정렬: `euler="0 -90 0"`.
  (`<compiler angle="degree"/>` 명시 — 안 하면 MuJoCo 기본 degree 로 오해석.)
- link_6 메쉬 bbox z=[−0.058, 0] → **플랜지 면 = link_6 원점(z=0)**, 공구는 +z.
- 오른쪽 핑거 mesh 는 `euler="180 0 0"` 으로 미러(AndrejOrsula 원본 구조).
