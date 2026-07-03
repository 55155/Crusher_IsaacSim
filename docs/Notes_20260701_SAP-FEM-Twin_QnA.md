# 대화 정리 — SAP · FEM 응력장 · 디지털 트윈 캘리브레이션 (2026-07-01)

> Claude Code 세션 정리본. 원본 로그(JSONL):
> `~/.claude/projects/C--Crusher-isaacsim/65a787d2-fec7-4493-a9a5-1e9ded2c591d.jsonl`
> 관련 소스: [`fem_stress_field.py`](../../Crusher_Genesis/FEM/fem_stress_field.py),
> [`DigitalTwin.md`](DigitalTwin.md) §6~8, [`DataInventory.md`](DataInventory.md),
> [`Crusher.md`](Crusher.md) §12.

---

## 0. git pull 요약 (`caeda9d..49c8afd`)

- **b555e5d** Crusher strike-retract FSM 제어 + DataInventory 문서.
- **49c8afd** FEM von Mises + 최대주응력 응력장 시각화(`fem_stress_field.py`).
- 총 +1,415줄 / 10파일.

---

## 1. SAP 란

- **SAP = Semi-Analytic Primal** (Drake, Castro et al. 2022). 접촉을 primal 공간의
  **볼록 최적화**로 풀고 마찰콘을 해석적으로 근사. Genesis가 rigid↔FEM **커플러**로 이식.
- ⚠️ 약자 충돌: 충돌감지 문헌의 **SAP = Sweep and Prune**(broad-phase)와 전혀 다름.
  우리 맥락은 전자.
- Genesis 구현의 실제: rigid collider도 **tet 메쉬로 변환**(`mesh_to_elements`),
  무한평면(`GEOM_TYPE.PLANE`) 금지, 접촉은 `FEMSurfaceTetLBVH ↔ RigidTetLBVH`
  = **BVH 기반 tet-tet 이산 접촉**.

## 2. Soft contact vs 무한강성 상보성 (LCP/NCP)

- **상보성 조건**: `g·λ = 0, g ≥ 0, λ ≥ 0` (안 닿으면 힘 0, 닿을 때만 힘).
- 강체 = g<0 절대 불허 → 힘–관통 곡선이 g=0에서 **수직 벽**(무한강성). 이걸 풀려면
  매 스텝 **LCP**(마찰 없음)/**NCP**(쿨롱 마찰) 최적화 → GPU 병렬 어려움.
- **Soft contact**: 약간 관통 허용, `λ = k·max(0,−g) + c·v_n`. 수직 벽 → **경사면**.
  상보성 자동 성립, LCP 안 풀고 힘만 더해 **ODE 적분** → GPU 쉬움.
- **SAP**의 semi-analytic = soft 위에 **implicit 적분 + 마찰콘 해석적 smoothing(볼록)**.

## 3. ODE (두 뜻)

- **Ordinary Differential Equation** (상미분방정식): soft contact는 힘을 우변에 얹고
  일반 시간적분기(Euler/RK4 등)로 진행 = "ODE 적분".
- **Open Dynamics Engine**: LCP 기반 고전 강체엔진(Bullet·ODE·초기 MuJoCo 계열)의 이름.

## 4. LCP/NCP "볼록성이 애매하다"

- 볼록 = 국소해=전역해, 해 유일/존재 보장, 뉴턴 수렴 보장.
- 마찰 없는 접촉(Signorini)은 PSD → 볼록. **마찰콘이 들어오면 비볼록** →
  해 여러 개/없음(Painlevé paradox), pivoting solver 실패·사이클링, 병렬화 곤란.
- 그래서 MuJoCo·Drake SAP·Genesis SAP는 **약간의 물리 근사(convex relaxation)**로
  강제 볼록화 → 항상 해 있고 수렴, GPU 병렬.

## 5. 왜 "접촉 solver"가 "커플러"인가 (monolithic)

- Genesis엔 Rigid/FEM/MPM/PBD 등 **독립 솔버**가 병존. 서로 다른 솔버 물체가 닿을 때
  힘을 계산해 줄 **커플러**가 필요.
- 두 시스템을 잇는 유일한 물리적 끈 = **경계면 접촉 구속 λ**. gap·상대속도가 두 시스템
  상태를 동시에 품으므로, λ를 풀려면 **두 운동방정식을 union으로 합쳐** 함께 풀어야 함.
- 그 "함께 푸는 계산"이 곧 접촉 solver → **monolithic 접촉 solver = 커플러**.
- 대가: 통일 표현 위해 rigid를 tet화 → 이산 메쉬 접촉 → **터널링**의 구조적 원인.
- IPC는 vertex–primitive **analytic** 접촉이라 tet화 불필요 (정답형이나 RTX5070 크래시).

## 6. "2 GPa 응력이 적당?" — 범주 오류 주의

- 코드의 `2 GPa`는 **응력이 아니라 영률 E**(`E_TABLET=2.0e9`). 출력 응력장은 **MPa** 단위.
- 정제 겉보기 E: MCC ~4–9 GPa → **2 GPa는 하한(무른 편)**, 허용 범위.
- **의미 있는 응력 상한 = 인장강도 ~1–5 MPa**. 파쇄 모델이 없어 σ는 변형률에 선형 증가
  (1% → 20 MPa 등)하므로, **σ_I가 ~수 MPa에 닿는 지점까지만 물리적으로 유효**.

## 7. tet화 해도 왜 관통(tunneling)이 생기나

tet화는 "같은 표현으로 한 최적화에 넣기"만 해결. 비관통 보장 아님. 원인 2가지:

- **① 이산 시간(DCD, CCD 없음)**: BVH가 "현재 스냅샷"만 봄. `v·dt > 표면 tet 두께`면
  스텝 사이를 건너뛰어 관통. (준정적 0.1 μm/step에선 꺼짐 → 이번 run 원인 아님.)
- **② soft 접촉(유한 강성)** ← 이번 run 진범: `f=k_c·δ`, k_c 상한(implicit 조건수 때문)
  + plate는 속도구동(안 멈춤) + 재료가 2 GPa로 훨씬 stiff → 변위가 접촉 관통 δ로 흡수,
  하중이 정제에 전달 안 됨(F≈0) → 관통. 그래서 Dirichlet BC로 우회.
- IPC는 CCD(스윕) + barrier(δ→0에서 힘 ∞)로 둘 다 차단.

**표기 메모**: `tet화` = **사면체화(tet화)** 권장. 소리대로면 "테트화"(텟화는 비표준).

## 8. 노드 위치 → 응력 (후처리 파이프라인 + 수식)

핵심: linear tet는 4 꼭짓점이 변형을 완전히 결정(constant strain). 순수 기하 + 재료법칙.

1. 모서리행렬: `Dm=[X1−X0,X2−X0,X3−X0]`(기준), `Ds`(현재).
2. **변형구배** `F = Ds·Dm⁻¹` — 옛 모서리→새 모서리 매핑. F=I면 변형 없음.
3. **회전 제거**: polar/SVD `F=R·S`, `F̂ = RᵀF`. (회전은 응력 아님.)
4. **변형률** `ε = ½(F̂+F̂ᵀ) − I` (corotated 소변형).
5. **응력** `σ = 2μ·ε + λ·tr(ε)·I` (솔버와 같은 linear_corotated, μ·λ는 E·ν에서).
6. **스칼라**: von Mises `σvm=√(1.5·dev:dev)`, 최대주응력 `σ_I = max eig(σ)`.

숫자 예: z로 1% 압축(옆 자유, ν=0.25) → `σ_zz = E·ε = −20 MPa`(= 앞의 "1%→20MPa" 확인).

> figure의 색은 솔버가 준 응력이 아니라, **노드 이동(기하)에서 같은 corotated 법칙으로
> 재계산**한 일관된 값. (Genesis FEM은 per-tet 응력을 노출 안 함.)

## 9. 그 응력장 figure는 실제로 어떻게 나왔나

- STL → tet(노드 2745·tet 12005), 중력 0, precision 64.
- **접촉이 아니라 Dirichlet 강제변위**: 상단 10% 밴드 노드를 매 스텝 `−z`로 이동,
  하단 10% 고정. (SAP 접촉이 2 GPa 정제를 못 실어 관통 → 우회.) plate는 사실상 잔재.
- 20스텝마다 노드 위치 스냅샷 → §8 후처리로 σ 재구성 → 5단계 패널(표면 vM / 단면 vM / 단면 σ_I).

## 10. 접촉부(파랑)에 응력이 없는 이유

- 밴드 노드가 **전부 같은 벡터로 이동 = 강체 병진 → F=I → ε=0 → σvm=0 → 파랑**.
- 변형은 자유로운 내부로 몰려 **중앙 빨강**. (von Mises는 국소 왜곡을 재는데 밴드는 안 찌그러짐.)
- 보강: 구속면은 삼축압축(정수압 큼, 편차 작음)이라 vM 낮음 + biconvex 렌즈라 하중이 중앙 기둥으로.
- **주의**: 이는 접촉역학이 아니라 **두꺼운 Dirichlet 밴드를 강체로 움직인 BC 아티팩트**.
  실제 Hertz 접촉이면 압력=표면, **최대 전단=subsurface**, **최대 인장=접촉 가장자리/내부**.

## 11. 인장강도 σ_t 의 역할 (Crusher 관점)

취성 정제는 압축이 아니라 **인장으로 깨진다**. σ_t는 파쇄 물리의 중심 파라미터:

1. **파괴 판정**(Rankine): `σ_I ≥ σ_t` 이면 균열 개시. → σ_I 장을 뽑는 이유.
2. **실험(A)↔트윈(B) 캘리브레이션 접점**: 파괴하중 F → (Fell-Newton/Pitt) → σ_t.
3. **필요 타격력·모터 토크 사이징** 근거.
4. **접촉/파쇄(N_f) 문턱**: σ_I<σ_t 접촉(stall), σ_I≥σ_t 파쇄(통과).
5. **분포**(Weibull): 취성 강도 산포 → "몇 타에 깨지나" 확률.

## 12. E·ν / 주응력장 / N_f / 캘리브레이션 항

- **E·ν 없이 주응력장?** 방향·상대분포는 노드 변위만으로(물성 없이) 나온다
  (주응력 방향=주변형 방향, E·ν 무관). **절대값[Pa]만** E(스케일)+ν(혼합) 필요
  → 실측 힘으로 E 역보정.
- **주응력장** = 각 점에서 전단 0인 3주축의 수직응력 중 최대(σ_I, +인장/−압축)를 공간에
  매핑한 **인장 위험 지도**. (vM=왜곡 스칼라(항복), σ_I=인장 파괴 구동자.)
- **N_f** = 재료상수 아님, **세어서 얻는 관측량**. 실험(직접 카운트) + 트윈(FSM "stall 없이
  통과" 또는 손상누적 D≥1)으로 예측 → 서로 맞춤. **입력이 아니라 캘리브레이션 표적**.
- **캘리브레이션 항**(조정) = **E**, ν, (σ_t), Weibull m, G_c, **손상규칙 계수**, 접촉/봉투/전류.
  **표적**(맞춤) = F(δ), Brazilian σ_t, F_peak–N_f, 균열위치.

## 13. 한 알약 예시 워크플로우 (→ DigitalTwin.md §7-8에 반영)

대상: `tablet_R4.0_AR1.00_CV0.20` (D=8mm, t=3.5mm, W=2mm, ~200mg, MCC).

- STEP0 형상→tet.
- STEP1 실험: F–δ(→E), Brazilian F=80N →(Pitt) **σ_t≈2.2 MPa**, 30개→Weibull m≈8, 반복타격→F_peak–N_f.
- STEP2 **E 보정**: 강제변위 반력을 실측 F–δ에 맞춤 → E≈6 GPa, ν=0.25.
- STEP3 단일타격 σ_I ≥ σ_t? → F_threshold + 균열위치.
- STEP4 미달 시 **손상누적** `D+=f(σ), D≥1→N_f`, 계수를 F_peak–N_f에 맞춤 (Regime II).
- STEP5 검증(held-out F_peak) → 실험 없이 신규 형상 N_f·토크·균열패턴 예측.

한 줄: **E·손상계수=돌려 맞추는 항, σ_t=재서 넣는 입력, N_f=예측·검증 표적.**

---

## 부수 작업 (이 세션에서 실행)

- **Pretendard 폰트 설치**: GitHub v1.3.9 static 9굵기(Thin~Black) OTF를 사용자 폰트로
  등록. (Illustrator에 안 뜬 원인 = 미설치. Variable본 대신 static 권장, 앱 재시작 필요.)
- **git push** (`dec64b3`): DigitalTwin.md §7-8 추가 + `.gitignore`에서 `Sim_result/`
  해제 → Sim_result 폴더 전체(~55MB, FEM/grasp/crushing 영상 포함) 추적 시작.
  (대용량 미디어라 추후 git-lfs/Release 분리 고려 가능.)
