# FEM — 정제(Tablet) 유한요소 해석 진행 노트

Crusher 디지털 트윈에서 **정제의 1차 파쇄력 / 누적 일(W\*)** 을 추정하기 위한 FEM
작업 기록. 이론·전략 배경은 [`../../MuJoCo_PlayGround/docs/DigitalTwin.md`](../../MuJoCo_PlayGround/docs/DigitalTwin.md)
의 **§7 FEM 방식으로 모델링** 을 먼저 참고한다. 이 문서는 그 전략을 실제 Genesis
코드로 옮긴 **구현 현황**을 다룬다.

---

## 1. 디렉토리 구성

| 파일 | 역할 | 상태 |
|---|---|---|
| `fem_uniaxial_compression.py` | 정제 **단축 압축(uniaxial compression)** 테스트 — vertex constraint 구동 | ✓ 1차 동작 |
| `FEM.md` | (이 문서) FEM 진행 노트 | — |

정제 STL 은 repo 루트 `tablets_stl/stl/` 에 **1,000개**가 파라미터 스윕으로 생성돼
있다 (`tablet_R{반경}_AR{종횡비}_CV{곡률}.stl`, 생성기:
`tablets_stl/Codes/tablet_generator*.py`). 현재 테스트는 그중
`tablet_R4.0_AR1.00_CV0.20.stl` 한 개를 사용한다.

---

## 2. `fem_uniaxial_compression.py` — 무엇을 하나

정제를 **위·아래 평판 사이에서 두께 방향(Z)으로 압축**하며 공칭 응력-변형률
선도를 뽑는 가장 단순한 검증 시뮬. 실제 Crusher 타격이 아니라, **FEM 솔버와
재료 거동을 격리 검증**하는 단계다.

### 2-1. 핵심 설계 결정 (공식 예제 패턴 채택)

Genesis 공식 예제 `fem_hard_and_soft_constraint.py` 의 표준 패턴을 그대로 따른다.

- `FEMOptions(use_implicit_solver=True, enable_vertex_constraints=True)` — **coupler 없음**
  - `enable_vertex_constraints` 는 디폴트 `False` → **반드시 켜야** vertex 구속이 동작.
  - implicit(backward Euler) 솔버라 `dt=1e-3, substeps=1` 같은 큰 스텝도 안정.
- **압축을 rigid 접촉이 아니라 vertex constraint(hard Dirichlet)로 구동**:
  - 정제 **top 노드**: 매 step `update_constraint_targets` 로 target 을 −z 로 이동.
  - 정제 **bot 노드**: 초기 위치 고정.
  - 내부 노드: 자유(FEM).
  - 평판(plate)은 **시각 보조일 뿐** — `set_pos` 텔레포트로 압축면만 추적, 접촉 계산
    안 함. (Rigid–FEM 접촉의 tunneling/발산 회피, DigitalTwin.md §7-7 참고)

### 2-2. 축 정렬 주의

사용한 STL 은 **두께(4mm)가 Y축**, 지름(8mm)이 X·Z 에 있다. 압축축을 Z 로 맞추기
위해 로드 직후 **X축 +90° 회전(Y→Z)** 한다. 이후 bbox·배치·BC 모두 Z 두께 기준.

### 2-3. 파라미터 (테스트값)

| 분류 | 값 | 비고 |
|---|---|---|
| 재료 E / ν / ρ | 2.0 GPa / 0.25 / 1300 kg/m³ | literature 정제 일반값, `linear_corotated` |
| dt / substeps | 1e-3 / 1 | implicit |
| duration | 0.3 s | |
| plate 속도 | 0.04 mm/s | ε≈0.3% (4mm 두께)까지만 — 파단 ε≈0.15%(σ≈3MPa) 부근 |
| mesh decimation | 50 face 목표 | 노드 수 대폭 축소, 테스트 반복용 |
| backend | `gs.gpu` (mac=Metal) | |

### 2-4. 출력

- `../Sim_result/fem_uniaxial_<ts>.mp4` — 측면 압축 영상 (fov 6°, 정제 높이 측면 촬영)
- `../Sim_result/fem_uniaxial_<ts>.png` — 4분할 플롯:
  (a) plate vs 정제 압축량, (b) 공칭 변형률 ε(t), (c) top/bot 노드 z 추적,
  (d) 공칭 응력-변형률 σ=E·ε (파단 ~2–3 MPa 밴드 표시)

> 현재 σ 는 **선형 탄성 가정의 공칭값(σ = E·ε)** 이다. Genesis 기본 FEM 에는
> 손상/파단 모델이 없으므로 실제 von Mises·주응력·파단 판정은 후처리 또는 별도
> 모델이 필요하다 (아래 TODO).

---

## 3. 진행 이력 (commit)

- `3c85e57` Add FEM/fem_uniaxial_compression.py — first working FEM uniaxial test
- `0edc4c9` 응력 출력·압축축 수정, 예제 패턴으로 단순화

요약: Rigid–FEM 접촉으로 직접 누르려던 초기 시도에서 발산/접촉 문제로 막혀,
**공식 예제의 vertex-constraint 구동 방식으로 단순화**해 1차 동작을 확보했다.

---

## 4. 알려진 한계 / TODO

| 우선순위 | 항목 | 메모 |
|---|---|---|
| 높음 | **파단/손상 모델 부재** | Genesis 기본 FEM 에 없음 → von Mises·S-N·Paris 등 외부 후처리, 또는 pre-fractured rigid composite hack (DigitalTwin.md §7-4) |
| 높음 | **W\* / N_f 측정** | 디지털 트윈 타깃은 단일 F\* 가 아니라 누적 일 W\*·파쇄 사이클 N_f (DigitalTwin.md §7-4) |
| 중간 | **실제 Rigid–FEM 접촉** | 현재는 constraint 구동(접촉 미계산). 실제 Crusher 타격력 전달엔 coupler 접촉 필요 — tunneling/sparse-mesh 주의 (§7-7) |
| 중간 | **mesh 수렴성** | 50 face 는 테스트용. 정제 ~12,892 tet 수준 권장(§7-7), 메쉬 의존성 점검 |
| 중간 | **반력(reaction force) 추출** | 분쇄력 측정의 핵심. 현재 σ=E·ε 공칭값만 — 구속 노드 반력으로 실제 force–displacement 확보 |
| 낮음 | **형상 스윕** | `tablets_stl/stl/` 1,000개(R/AR/CV) 일괄 해석 → material card 구축 |

---

## 4.5. `elements_el_energy` 단위 의심 (추론 메모)

`fem_solver.elements_el_energy.energy` 합으로 출력한 $W_{int}$ 가 단순 추정치
($\tfrac{1}{2}\sigma\varepsilon V \approx 1$ J) 대비 **~10⁹ 배** 부풀어 나옴
(ε=8.09% 시점에 3 × 10¹² mJ ≈ 3 GJ).

검증 단서:
- 변위·strain·node trajectory 는 모두 SI(m, %) 로 **정확**.
- 부풀음이 **에너지 field 하나만** — 다른 양은 정상.
- 비율이 정확히 **m³ ↔ mm³ (10⁹)** — 다른 단위 변환은 이 자릿수가 안 나옴.

가설 (확률 순):
1. **솔버 내부 raw buffer** — `elements_el_energy` 가 Newton step 보조용 (J 가
   아닌 unscaled quantity). trend 는 옳지만 절대값 J 해석 불가.
2. **부피 단위 mismatch** — element V_tet 가 m³ 가 아닌 mm³ 로 곱해진 자리가
   있을 가능성. (1) 의 한 형태.

함의:
- **trend (∝ ε²) 는 신뢰**, **절대값 [J] 는 신뢰 불가**.
- 정량 W·σ 가 필요하면 **node 변위로 ∇u 직접 계산 → constitutive law 후처리**
  또는 **구속 노드 반력 ∫F·v dt** 가 안전.

(자세한 자릿수 분석은 commit 메시지/대화 기록 참고. 정확한 위치 확정은
`elements_i.V_tet` 등 internal field 추출 필요 — 현 단계 우선순위 낮음.)

---

## 4.6. 현재 막힌 지점 (2026-06-29) — Hertz point contact

velocity control(`control_dofs_velocity` + 무거운 plate ram, DigitalTwin.md §7-7 (3))
도입 후 F(t)는 **부드럽게 연속 증가**(spike 사라짐) — SAP 연결 자체는 정상.
다만 d_max=20 μm 에서 F=0.022 N — 이론 elastic (E·A·ε=500 N) 대비 **~30,000×**,
Hertz 점접촉 (~16 N) 대비도 **~700× 부족**. mesh 1000→5000 face 올려도 1.5×만 늘음.
→ **CV=0.20 convex dome 꼭대기 점접촉이 본질**. 처방: d_max ≈ 1 mm 까지 깊게
눌러서 dome→cylindrical band 면접촉 영역으로 진입시키는 것 (PLATE_VEL 10×↑ 또는
DURATION ↑). 그래도 안 되면 SAP `hydroelastic_stiffness` 조정 검토.

## 4.7. Pre-fracture modeling (다음 단계 계획)

elastic FEM이 σ_I_max 위치(균열 핵)를 정확히 예측 → 그 위치를 따라 정제를
**rigid 조각 + weak equality constraint** 로 미리 메쉬 분할 → constraint reaction
> σ_t 도달 시 끊김 = 균열 발생. 4단계 워크플로우:

1. **Elastic FEM scan** (현재 셋업) → tet 별 σ_I field → 상위 5% 위치 추출.
2. **Pre-fragmentation mesh** (Python/trimesh) — FEM 예측 weak surface(적도면 +
   방사 N개)를 따라 정제 chunk 분할.
3. **Genesis 재구성** — 각 chunk 를 rigid entity, 인접면에 equality constraint;
   매 step Python 에서 reaction force 모니터링.
4. **Break threshold** — `|F_constr/A_face| > σ_t` (Brazilian/Pitt) 면 `remove_constraint`
   → 균열 시퀀스·N_f·W\* 자연스럽게 출력.

→ contact 문제 해결 직후의 자연스러운 다음 작업. MVP는 적도면 1개 + constraint 1개
검증부터 (DigitalTwin.md §7-4 pre-fractured composite hack 의 구체화).

---

## 5. 실행

```bash
# mac (framework python 3.13 + Genesis 1.1.0 + Metal)
cd Crusher_Genesis/FEM
/Library/Frameworks/Python.framework/Versions/3.13/bin/python3 fem_uniaxial_compression.py
# 결과: ../Sim_result/fem_uniaxial_<ts>.{mp4,png}
```
