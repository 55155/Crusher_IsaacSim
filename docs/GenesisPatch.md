# Genesis 몽키패치 대장

Genesis 설치본(`site-packages`)은 **절대 수정하지 않는다.** 런타임에 메서드를
교체하는 방식만 쓰고, 그 내역을 전부 이 문서에 남긴다.

패치 코드는 전부 `utills/fem_ipc_workarounds.py` 한 곳에 모은다. 스크립트는
필요한 패치 함수를 **명시적으로 호출**해야 적용되므로, 부르지 않은 스크립트의
거동은 바뀌지 않는다.

| # | 패치 함수 | 대상 | 상태 | 확인일 |
|---|---|---|---|---|
| 1 | `patch_fem_vertex_constraints()` | `FEMEntity.set_vertex_constraints` | 적용 중 | 2026-07-15 |
| 2 | `patch_ipc_vertex_attach()` | `IPCCoupler._add_fem_entities_to_ipc` | **적용 중** | 2026-09-02 |

환경: Genesis 1.3.3 / libuipc 0.0.25 / quadrants 1.3.0 / Windows / CUDA

---

## 패치 1 — `set_vertex_constraints` 의 뒤집힌 가드

**대상**
```
site-packages/genesis/engine/entities/fem_entity.py:955  FEMEntity.set_vertex_constraints
```

**결함**
```python
if isinstance(self.sim.coupler, IPCCoupler):
    gs.raise_exception("This method is only supported by IPC coupler.")
```
메시지는 "IPC 커플러에서만 지원된다"인데 조건은 **IPC 커플러일 때 예외를 던진다.**
조건이 뒤집혀 있다. 우리 씬은 전부 IPC 커플러라 이 메서드가 사실상 항상 막혀 있었다.

**우회** — 원본 로직을 그대로 복제하고 `isinstance` 체크 한 줄만 빼서 몽키패치.

**사용처 (9개 파일)**
```
Crusher_M0609_RG2_Tablet_Samplebag/full_workflow.py
FEM/fem_tablet_drop_bag_open.py          FEM/fem_tablet_drop_bag_suction.py
Legacy_vs_IPC/pipeline_ipc.py            Recovery2_only/recovery2_bag_clamp.py
Recovery2_only/recovery2_tablet_drop.py  Powder_flip_test/ipc_grain_coupler.py
Powder_flip_test/powder_containment_test.py
SuctionV1_only/suction_bagopen_20260901.py
```

**한계 (2026-09-02 규명)** — 이 패치는 **예외를 뚫을 뿐, 구속을 동작시키지 않는다.**
아래 패치 2의 배경이다.

---

## 패치 2 — IPC 커플러에 정점 부착 배선

### 왜 필요한가

IPC 커플러를 켜면 천은 **uipc 가 스텝**한다. `set_vertex_constraints` 는 Genesis
자체 FEM 솔버의 버퍼에만 쓰는데, `ipc_coupler/coupler.py` 전체에 **FEM 정점 구속을
참조하는 코드가 한 줄도 없다.** 거기 있는 constraint 는 전부 강체 링크용이다
(`SoftTransformConstraint`, `two_way_soft_constraint`, `aim_transform`).

즉 Genesis 에 구현이 없는 게 아니라, **FEM 정점 구속의 IPC 판이 없다.**

### 실측 근거 (2026-09-02, `SuctionV1_only/suction_bagopen_20260901.py`)

봉투(714정점 FEM.Cloth)를 컵 사이에 놓고 하단 190정점을 고정한 뒤 정착시킨 결과.

**소프트 구속은 완전 무반응** — 강성을 1만 배 올려도 소수점까지 동일하다.

| `stiffness` | 고정정점 실제 z | 목표 z | 편차 |
|---|---|---|---|
| 1e4 | 3.6mm | 517.3mm | 545.2mm |
| 1e6 | 3.6mm | 517.3mm | 545.2mm |
| 1e8 | 3.6mm | 517.3mm | 545.2mm |
| 구속 없음(대조군) | 3.6mm | — | — |

**하드 구속은 자기 정점만 옮긴다** — 편차 0.0mm 로 정확히 도달하지만 이웃이
안 따라와 메시가 찢긴 것처럼 늘어난다.

| 조건 | 봉투 bbox z (원래 90mm) |
|---|---|
| 하드 핀 | 470 ~ 545mm |
| 구속 없음 | 6.0mm (바닥에 납작 — 형상 유지) |

대조군이 형상을 유지하므로 **셸 재료는 정상**이다. 파라미터도 아니다 —
`CLOTH_E` 4e5→4e8(1000배), `dt` 5e-3/2e-3, `d_hat` 1e-4/1e-2(100배),
`bending` 50/400 을 다 훑어도 신장이 안 변한다.

이것이 "FEM weld 는 정말 한 점만 당겨지는 느낌"의 정체다. `fem_tablet_drop_bag_suction.py`
가 `SUCTION_N_VERTS = 1  # 1=진짜 한 점` 으로 간 것도 현실성 판단이 아니라
이 메커니즘이 낼 수 있는 유일한 거동이었기 때문이다.

### 무엇을 쓸 것인가

**새 물리를 쓰는 게 아니다.** uipc 에 이미 있는 구성식을 Genesis 가 만드는 씬에
배선할 뿐이다. Genesis 메인테이너도 흡착에 대해 *"Not yet. Suction cup requires
adding an additional constraint, which should be easy to add"* 라고 답했다
([Discussion #87](https://github.com/Genesis-Embodied-AI/Genesis/discussions/87)).
그 constraint 가 이것이다.

```python
uipc.constitution.SoftPositionConstraint.apply_to(sc, strength_rate=100.0)
uipc.constitution.SoftVertexStitch.create_geometry(
    aim_geo_slots=(sc_a, sc_b), stitched_vert_ids, kappa=1e6, rest_length=0.0)
uipc.builtin.aim_position        # 정점별 목표 위치
uipc.builtin.is_constrained      # 정점별 on/off — 런타임 토글
```

Genesis 는 **강체 링크에 이미 같은 메커니즘을 쓰고 있다** (`coupler.py:79-89`):
```python
uipc.view(is_constrained_attr)[0] = 1
uipc.view(aim_transform_attr)[:] = coupler._abd_transforms_by_link[link][env_idx]
```
천 정점판(`aim_position`)이 uipc 에 있는데 연결이 안 돼 있을 뿐이다.

### 붙일 지점

```
coupler.py:292   _add_fem_entities_to_ipc()     ← 패치 대상
coupler.py:365     fem_obj.geometries().create(mesh)   슬롯을 버린다 → 붙잡아야 함
coupler.py:545     _abd_slots_by_link[link]            강체 슬롯은 저장돼 있음
coupler.py:137     _ipc_animator = _ipc_scene.animator()
coupler.py:518     _ipc_animator.insert(obj, callback) 매 스텝 콜백
coupler.py:704   _ipc_world.init(_ipc_scene)    ← 이 이후로는 씬 변경 불가
```

### 계획

```
1. _add_fem_entities_to_ipc 몽키패치
   - 천 메시에 SoftPositionConstraint.apply_to() 를 build 때 항상 적용
   - 버려지는 천 geometry slot 을 붙잡아 저장
   verify: world.init() 통과, 기존 시퀀스가 그대로 완주
2. 애니메이터 콜백 등록 — 매 스텝 aim_position 기록
   verify: is_constrained 전부 0 이면 자유낙하 대조군과 동일
3. 파지 시점에 컵 반경 안 정점만 is_constrained=1, aim_position 을 컵 따라 이동
   verify: bbox z 가 90mm 유지 (지금은 470~545mm)
4. 개구 100mm
   verify: follow = 개구폭/컵이동 ≈ 1.0
```

### 위험

1. `SoftPositionConstraint` 는 `world.init()` **전에만** 붙일 수 있다. 항상 걸어두고
   정점별 `is_constrained` 로 켜고 꺼야 한다.
2. 천(FiniteElement) 정점이 강체 인스턴스와 **같은 방식으로** `is_constrained` 를
   존중하는지 미확인. 단계 1에서 드러난다.
3. private 메서드 몽키패치라 Genesis 업그레이드 시 깨진다. 버전을 위에 못박아 둔다.

---

## 패치 2 는 무엇을 한 것인가 — 메모지 비유

한 줄로: **Genesis 에는 "특정 정점 집합을 당기는(흡착)" 기능이 IPC 씬에서 구현돼
있지 않았고, 우리가 몽키패치로 그걸 가볍게 구현했다.**

### 봉투를 보는 일꾼이 둘이다

IPC 커플러를 켜면 천은 **uipc 가 전담**하고 Genesis 의 FEM 솔버는 논다.

`set_vertex_constraints` 는 **Genesis 의 메모지에 쓰는 것**이다. uipc 는 그 메모지를
안 본다. 그래서 이렇게 갈린다.

| | 무슨 일이 벌어지나 | 결과 |
|---|---|---|
| **소프트** | "살살 당겨줘"라고 적어둔다. 아무도 안 읽는다 | 아무 일도 안 일어남 |
| **하드** | Genesis 가 uipc 몰래 손을 뻗어 그 정점만 직접 옮긴다 | 옮겨지긴 하나 uipc 의 천 모델은 그 사실을 모른다 → 이웃이 안 따라옴 → 찢어진 것처럼 보임 |

### 패치는 uipc 자기 메모지에 쓴다

uipc 에는 그 용도의 양식이 있다 — `SoftPositionConstraint`. 붙이면 **모든 정점**에
칸 두 개가 생긴다.

```
is_constrained   0/1 스위치
aim_position     어디로 끌어당길지
```

**단, 양식은 공장 가동 전에만 붙일 수 있다.** `_ipc_world.init()` 에서 uipc 가 솔버를
컴파일해버리기 때문이다. 그 뒤로는 칸을 새로 못 만든다 — 대신 **이미 있는 칸의
값은 바꿀 수 있다.**

그래서 패치는 빌드 때 양식을 붙이되 **스위치를 전부 0, 목표를 각자 제자리**로 둔다.
켜도 아무 일이 안 일어나므로 기존 거동이 안 바뀐다. 그리고 나중에 쓸 수 있게
**종이 손잡이(geometry slot)를 챙겨둔다.**

걸림돌 두 개를 넘었다.

```
(a) Genesis 빌드가 양식을 안 붙임   -> 빌드 함수를 감싸서 끝난 직후에 붙임
(b) Genesis 가 손잡이를 버림(365행) -> 이름으로 천을 찾아 손잡이 회수
```

### 왜 강체는 이미 되는가

강체의 `is_constrained` + `aim_transform` 은 **사용자용 고정 기능이 아니라 두 솔버를
동기화하는 장치**다. 로봇은 Genesis 강체 솔버가 소유하는데(IK, PD, 관절) 접촉을
풀려면 uipc 에도 같은 로봇이 있어야 한다. 한 물체를 두 솔버가 가지니 한쪽이 따라
가야 하고, 그게 `two_way_soft_constraint` 다(`coupler.py:87` 이 매 스텝 Genesis
자세를 써넣고, `coupler.py:1108` 이 그 구속력을 되읽는다).

천에는 이 문제가 없다. Genesis 쪽에 천을 모는 주체가 없고 uipc 가 단독 소유한다.
그래서 동기화 장치가 필요 없었고 안 만들어졌다. 우리는 **그 장치를 부착 수단으로
전용**한 것이다.

### 실측 (2026-09-02)

봉투 714정점을 컵 사이에 세우고 하단 190정점을 고정한 결과.

| 방식 | 봉투 bbox (원래 64 x 6 x 90mm) |
|---|---|
| 하드 핀 (`set_vertex_constraints`) | z 470~545mm — 찢김 |
| 소프트 (`set_vertex_constraints`) | 무반응, 바닥에 낙하 |
| **`SoftPositionConstraint` 슬롯 직접 쓰기** | **64.0 x 6.1 x 90.0mm, 중심 오차 0.0mm** |

애니메이터 콜백도 필요 없었다 — 슬롯에 직접 쓴 값이 uipc 솔브까지 전달된다.

입구 개구도 성립했다.

```
갈아끼우기 전  개구 +0.0mm   follow 0.000   (파지 정점이 옛 무반응 경로에 있었음)
갈아끼운 후    개구 +55.9mm  follow 0.614   strength_rate=100
strength_rate 1e4            follow 0.761
```

### 같이 고친 두 가지

**1. 측면 필터.** 봉투 두께가 6mm(면이 중심에서 ±3mm)인데 컵 반경은 7.5mm 다.
반경만으로 고르면 각 컵의 구가 봉투를 관통해 반대쪽 면 정점까지 잡고, 두 컵이 같은
정점을 반대로 당겨 상쇄된다.

```
컵당 16개(최근접 1.6mm) -> 개구  2.6mm
컵당  7개(최근접 3.4mm) -> 개구 56.1mm
```
접촉면 기준 자기 쪽 정점만 고르도록 바꾸니 런마다 널뛰던 결과가 재현된다.

**2. 개루프 -> 폐루프.** 종전에는 조인트 명령값 `dy` 만큼 기억한 좌표를 y 로
평행이동했다(`side` 부호를 손으로 지정). 컵이 명령대로 안 가면 목표가 틀리고 회전은
아예 반영이 안 된다. 흡착은 "컵에 붙는" 것이므로 파지 순간의 **컵 로컬 좌표**를
저장하고 매 스텝 컵의 실제 자세로 되돌린다. `side` 도 `dy` 도 필요 없어진다.

```python
loc = (vp[idx] - p_cup) @ R(q_cup)                  # 파지 순간 1회
tgt = loc @ R(q_cup_now).T + p_cup_now              # 매 스텝
```

---

## 규칙

- 설치본 수정 금지. 런타임 교체만.
- 패치는 `utills/fem_ipc_workarounds.py` 에만 둔다.
- 새 패치는 **새 함수**로 추가한다. 기존 함수를 고치면 사용처 9개가 전부 영향받는다.
- 패치를 추가하면 위 표와 절을 같이 갱신한다. 근거는 실측 수치로 남긴다.
