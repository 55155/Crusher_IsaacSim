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

## 5. 실행

```bash
# mac (framework python 3.13 + Genesis 1.1.0 + Metal)
cd Crusher_Genesis/FEM
/Library/Frameworks/Python.framework/Versions/3.13/bin/python3 fem_uniaxial_compression.py
# 결과: ../Sim_result/fem_uniaxial_<ts>.{mp4,png}
```
