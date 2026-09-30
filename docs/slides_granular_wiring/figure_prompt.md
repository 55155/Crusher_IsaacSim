발표용 다이어그램 5장을 새로 그려 주세요. 주제는 로봇 크러셔 디지털 트윈에서 파우더(과립재)를 어떻게 시뮬레이션하느냐입니다. 첨부한 SVG 5장은 초안입니다. **내용과 수치는 그대로 두고 시각 품질만** 올려 주세요.

## 공통 요구사항
- 형식: 슬라이드 16:9 한 장을 꽉 채우는 SVG (viewBox 1600×900). 각 그림을 따로 된 아티팩트로 주세요.
- 용도: 학회·연구실 발표용 PowerPoint. 흰 배경, 인쇄해도 읽혀야 합니다.
- 글꼴: 한글은 "Pretendard", "Malgun Gothic", sans-serif 순. 코드와 식별자는 "JetBrains Mono", Consolas, monospace.
- 글자 크기: 본문 22px 이상, 제목 30px 이상. 발표장 뒷자리에서도 읽혀야 하니, 초안보다 글자는 줄이고 크기는 키워 주세요.
- 색: 의미별로 3~4색만 씁니다. 강체/Genesis = 회색, 커플러 = 보라, uipc/IPC = 청록, 문제/경고 = 빨강, MPM 파우더 = 황토. 그림자, 그라데이션, 이모지는 쓰지 마세요.
- 박스 안 문장은 짧게. 자세한 설명은 발표 멘트로 돌리므로, 한 박스는 제목 1줄 + 부제 1줄을 넘기지 마세요.
- 화살표가 박스를 관통하거나 글자와 겹치지 않게 해 주세요.
- 수치, 파일명, 함수명은 아래 표기와 한 글자도 다르지 않게 써 주세요.

## 그림 1 — 현재 배선 구조 (fig_wiring_current)
세 개의 큰 영역이 왼쪽에서 오른쪽으로 이어집니다.
1. **Genesis 강체 솔버 (RBD)**: M0609 로봇팔 + RG2 그리퍼 / Crusher 크랭크–슬라이더 (8 RPM, 변위 구동) / 고정장치·회수장치·판 (정적 소품)
2. **GrainIPCCoupler** (IPCCoupler 상속, build() 전에 교체): 결합 방식 라벨 `two_way_soft_constraint` (로봇, Crusher), `ipc_only` (정적 소품), `add_grains / SPC hold·release` (낟알)
3. **uipc IPC World (libuipc)**: 강체 프록시 (ABD) / 샘플백 FEM.Cloth (IPC 전용 박막) / 낟알 Particle (pointcloud, dim=0)
- 낟알 Particle은 "Genesis 재료 목록에 없음 — uipc에서 직접 배선"으로 강조 색을 줍니다.
- IPC World 아래에 띠 하나: "모든 접촉 = IPC 배리어 · d_hat = 0.1 mm (씬 전역)"
- 되돌아오는 화살표 하나: 커플러가 매 스텝 알별 법선력을 회수해 "출력: 크랭크 토크 · 벽 반력 · 알별 법선력"으로 보냅니다.

## 그림 2 — Genesis에 없는 재료를 배선한 CS 기법 (fig_coupler_injection)
좌우 2단 구성입니다.
- **왼쪽 ① 상속**: UML 클래스 다이어그램. `IPCCoupler` (Genesis 원본: `_add_objects_to_ipc()`, `_register_contact_pairs()`, `couple(f)`) ◁— `GrainIPCCoupler` (우리 코드). 새 메서드 `add_grains(pos, radius, ρ)`, `hold_grains / release_grains`, 오버라이드 3개(각각 super() 호출 뒤 확장): `_add_objects_to_ipc` → `_add_grain_entities_to_ipc()`, `_register_contact_pairs` → 낟알×{천, 강체, 바닥, 낟알}, `couple` → `_retrieve_grain_states()`. 부제: "설치 패키지는 한 줄도 수정하지 않음".
- **오른쪽 ② 교체 주입 (hot-swap)**: 위에서 아래로 5단계.
  - 문제: `simulator.py`가 옵션 타입만 보고 IPCCoupler를 직접 생성
  - 1 `scene = gs.Scene(coupler_options=IPCCouplerOptions(...))`
  - 2 `scene._sim._coupler = GrainIPCCoupler(scene._sim, opts)` ← 강조 (build 전 바꿔치기)
  - 3 `coupler.add_grains(positions, R, ρ)`
  - 4 `scene.build()`
  - 5 uipc 내부: `pointcloud(pos)` → `label_surface(mesh)` → `Particle().apply_to(mesh, ρ, thickness=R)` → `contact_tabular.insert(...)`
- 하단 각주: "함정: label_surface 누락 시 접촉 0 / 낟알은 debug 마커라 camera(debug=True) 필요"
- 느낌: "라이브러리 내부의 생성자 자리에 우리 부품을 끼워 넣는다"가 한눈에 보이게. 원본 클래스의 자리에 서브클래스가 끼워지는 소켓·플러그 비유도 좋습니다.

## 그림 3 — 문제 1: 힘을 d_hat이 정한다 (fig_ipc_barrier)
- 왼쪽: 그래프. x축 "두 표면 사이 틈 d", y축 "배리어 힘". 곡선은 IPC 배리어 b(d) = −(d−d̂)² ln(d/d̂)에서 나온 힘 f = −b′(d) (d < d̂에서만 양수, d→0에서 발산, d ≥ d̂에서 0). d̂ 위치에 수직 점선을 긋고, d < d̂ 구간을 옅게 칠합니다.
- 곡선 위 한 점에 주석: "변위 구동 → 틈이 여기 잠김, 힘 = κ·b′(d), κ = contact_resistance"
- 오른쪽 카드 3장:
  - 실측 1: 최근접 간격 = 2R + d_hat (2.098–2.099 mm), NEWTON_TOL을 100배 조여도 불변
  - 실측 2: 알별 하중 443 mN (p95 2.8 N) = "수렴한 배리어력" — 운동량 역산과 비 1.00, cos +1.000. 값을 정하는 건 물성이 아니라 d_hat·κ
  - 결과: 배리어는 비관통용 수치 장치이지 재료 법칙이 아님 → 로드셀로 캘리브레이션 필수

## 그림 4 — 문제 2: 파쇄하면 입자 수가 폭증한다 (fig_particle_count)
- 왼쪽: 그래프. x축 로그 "입자 수 증가 N/N₀ (같은 질량)" 1×, 10×, 100×, 1000×. y축 "직경 d/d₀" 0~1. 곡선 d/d₀ = (N/N₀)^(−1/3).
- 강조점 2개: 10× → 0.46배 / 381× → 0.14배 (725 → 100 µm)
- 오른쪽 카드 3장:
  - 예: 100알 → 1,000알이어도 직경은 약 1/2 (0.46배). 725 µm에서 시작하면 풀을 다 써도 약 340 µm
  - 실제 시료 NaCl 2 g: 725 µm 약 4,630알 → 100 µm까지 약 176만 알 (381배)
  - 현재 트윈 규모: 332–500알, 96–332 ms/스텝. d_hat이 가장 작은 알에 묶여 크기 비도 비용

## 그림 5 — 결론: MPM 격자 + Legacy 커플러, 봉투는 경계조건 (fig_proposed_mpm_legacy)
- 왼쪽: 크러셔 타격 구간의 두께 방향 단면 일러스트.
  - 격자(dx = 0.5 mm)가 깔린 사각 도메인 안 아래쪽에 파우더(MPM 물질점)가 채워져 있습니다.
  - 왼쪽에 좌우로 움직이는 충돌판(RBD, 크랭크 구동)이 있고 양방향 화살표를 붙입니다.
  - 도메인 테두리는 굵은 점선이고 라벨은 "MPM 도메인 경계 = 봉투 내면 (폭·높이 고정)"입니다.
  - 각주: "IPC 런 실측: 타격 중 봉투 내면 폭 변동 0.3–0.7% → 사실상 고정, 움직이는 건 두께 방향 한 면"
- 오른쪽: 세로 흐름 카드 4장
  - ① MPM 격자법 — 과립재: 알 개수와 무관, 비용 = 격자 해상도
  - ② Legacy 커플러 — 힘 계산: MPM↔Rigid SDF 격자 속도 보정, 배리어(d_hat) 없음, coup_restitution 0.8 (관통 0/107,911)
  - 출력: 물질점별 응력 σ (F → 구성식), 판·벽 반력 → 스트로크 일 W (로드셀 비교), 압밀 Jp
  - 치르는 것 (빨강): 봉투는 엔티티가 아니라 경계조건 / 알별 힘사슬 → 셀 평균 응력으로 평활화 / 파쇄·PSD 채널은 재료 확장 필요

## 전달물
- 그림별 SVG 아티팩트 5개
- 끝에 한 줄씩: 초안에서 무엇을 바꿨는지 (배치, 강조, 생략한 문구)
