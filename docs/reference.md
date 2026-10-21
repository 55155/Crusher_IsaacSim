# Reference — 분말 압밀·분쇄 선행연구 정리

분말 압축 시 분쇄(particle breakage)를 멀티솔버 / DEM / 신경망(PINN 포함)으로 다룬
선행연구와, 우리 Crusher 연구 대비 노벨티 판단을 모아 둔다.

> 작성: 2026-10-21. 관련: [`Paper.md`](Paper.md), [`DigitalTwin.md`](DigitalTwin.md), [`Real2Sim.md`](Real2Sim.md)
>
> **주의**: ★ 표시는 PDF 전문을 직접 읽은 논문. 나머지는 웹 검색 초록·요약 기준이라
> 서지(연도·권호)와 세부 내용은 원문으로 재확인할 것.

---

## 목차

1. [핵심 비교 논문 (전문 확인)](#1-핵심-비교-논문-전문-확인)
2. [분말 압밀 + 파쇄 — 멀티솔버 / 멀티스케일](#2-분말-압밀--파쇄--멀티솔버--멀티스케일)
3. [DEM 파쇄 모델](#3-dem-파쇄-모델)
4. [연속체 파쇄 구성식](#4-연속체-파쇄-구성식)
5. [신경망 · PINN 결합](#5-신경망--pinn-결합)
6. [비전 기반 입도 분포 측정](#6-비전-기반-입도-분포-측정)
7. [우리 연구 대비 노벨티 판단](#7-우리-연구-대비-노벨티-판단)

---

## 1. 핵심 비교 논문 (전문 확인)

### 1-1. ★ Chen et al. 2026 — DEM 파쇄 + 신경망 대리모델 + 역보정 (가장 가까운 선행연구)

- J. Chen, M. Asachi, A. Hassanpour, M. Babaie, M. Jabbari (Univ. of Leeds),
  **"Data-driven surrogate modelling and calibration of particle breakage in Li-ion electrode calendering"**,
  *Energy Storage Materials* 86 (2026) 105001. DOI: [10.1016/j.ensm.2026.105001](https://doi.org/10.1016/j.ensm.2026.105001)
  ([White Rose eprint](https://eprints.whiterose.ac.uk/id/eprint/242355/), open access CC BY)
- **워크플로우**
  1. EDEM: NMC622 + CB, 100×100×200 µm, XY 주기경계, 상판 **변위 지정** 19.5 µm 압축. 접촉 EEPA, 바인더는 초기 CB 주변 BPM 본드.
  2. 파쇄: Tavares UFRJ (구형 입자 교체, 크기 의존 파쇄에너지 E50(d), 손상누적 γ, t10).
  3. 데이터: 파쇄 파라미터 8개 LHS 280세트 → 압력곡선 196점 + (파쇄 직전 하중 × 원 직경) 30×30 히스토그램.
  4. 2-branch MLP (공유 8→256→64, 압력 196 / 분포 900 softmax), 손실 = MSE + KL. 테스트 MSE 0.0003, KL 0.33.
  5. 역최적화: MLP 고정, 입력 8개만 Adam→L-BFGS, Huber(압력) + 분포 무게중심 + 경계 패널티.
  6. Sobol 민감도(d0 지배, φ 는 파쇄 직경, **γ·Rτ 무시 가능**), PSD 강건성, TauFactor 굴곡도·확산도.
- **입도 분포**: N(7.74, 1.35²) µm 를 Zhang et al. 2022 실험값에서 계산해 입력(직접 측정 아님).
  PSD 표준편차(1.08–1.59)·평균(9.9, 11.9 µm) 변화 실험 포함. 압축 후 조각 PSD 실측 없음.
- **하지 않은 것**: RBD/MBD 결합 없음(판은 형상 + 변위 지정), 실시간 반력 피드백 없음(반력은 사후 학습 데이터),
  반복 하중 없음(단조 1회), 전단 없음, PINN 아님(데이터 손실만).
- **저자 명시 한계**: 구형 조각, 학습 범위 밖 일반화 미검증, 준정적 일축만, Tavares 모델이 **구속 효과 누락 → E∞ 과대추정 가능**.

### 1-2. ★ Xu et al. 2023 — BPM 2차 입자 파쇄 DEM + nano-XCT

- J. Xu, B. Paredes-Goyes, Z. Su, M. Scheel, T. Weitkamp, A. Demortière, A.A. Franco (LRCS, Amiens),
  **"Computational Model for Predicting Particle Fracture During Electrode Calendering"**,
  arXiv [2306.02155](https://arxiv.org/abs/2306.02155) / *Batteries & Supercaps* 6(12) (2023).
- NMC111 전극, nano-XCT(41 nm) 로 150 MPa 에서 2차 입자 균열 관찰. LIGGGHTS([LIGGGHTS-WITH-BONDS](https://github.com/richti83/LIGGGHTS-WITH-BONDS)),
  XCT 실형상 2차 입자 = 1.59 µm 1차 구 + BPM 본드, Hertz + SJKR.
- 공극률-압력 실험 정합. 본드 파단율 20 MPa 6.8%, 90 MPa 17.8%, 146 MPa 20.3%. "8–12% 파단 ≈ 균열" 기준 제안.
- 한계: 본드 강도 σ_Bc 미보정(파라메트릭만), 1회 1–4일(28코어). 신경망 없음.

---

## 2. 분말 압밀 + 파쇄 — 멀티솔버 / 멀티스케일

| 연구 | 방법 | 비고 |
|---|---|---|
| LANL (APS Shock 2022–2023) | DDMP(MPM) 로 rock sugar 구 압밀·분쇄, X-ray CT 비교, 밀도-압력 곡선 | MPM 단일 솔버. [2023](https://meetings-archive.aps.org/shock/2023/v02/1), [2022](https://archive.aps.org/shock/2022/w04/4), [μCT](https://archive.aps.org/shock/2023/j02/1) |
| Guo & Zhao | MPM×DEM 계층형 멀티스케일 (물질점마다 DEM RVE) | [PDF](https://jzhao.people.ust.hk/home/PDFs/nme4702.pdf) |
| Mason sand 1D MPM-DEM | 입자 파쇄 모델 포함, SHPB 재현 | 동적. [core.ac.uk](https://core.ac.uk/works/152064066) |
| HKUST 학위논문 | 입상체 grain crushing 멀티스케일 | [link](https://researchportal.hkust.edu.hk/en/studentTheses/multiscale-modeling-of-grain-crushing-in-granular-media/) |
| Nezamabadi, Radjai et al. (2017–2025) | implicit MPM + Contact Dynamics, 연성 입자 압밀 | 파괴 없음. [EPJ 2017](https://www.epj-conferences.org/articles/epjconf/abs/2017/09/epjconf170045/epjconf170045.html), [IFPRI](https://ifpri.net/system/files/attached/P3%20-%20Radjai%20-%20Predictive%20Framework%20for%20Composite%20Compact%20Strength.pdf) |
| Ku, Zhao et al., *Powder Technol.* 421 (2023) 118455 | MPFEM + 응집 접촉 파단, 응집성 분말 압밀 | [PDF](https://jzhao.people.ust.hk/home/PDFs/2023-PT-Quan.pdf) |
| Mazor et al., *Powder Technol.* (2018) | DEM(공급부) + FEM(롤 사이) 롤 압축 | [HAL](https://hal-mines-albi.archives-ouvertes.fr/hal-01848201/file/combined-dem.pdf) |
| Procopio & Zavaliangos, JMPS (2005) | MPFEM 고밀도 압밀 | 기억 기반, 재확인 필요 |
| Li et al., CMAME (≈2023) BFEMP | MPM–FEM IPC 배리어 비관통 결합 | 우리 GrainIPCCoupler 와 가장 가까움. 재확인 필요 |
| Wolper et al. 2019/2020 CD-MPM, AnisoMPM | 손상 MPM | 그래픽스 |
| Yue et al. 2018 Hybrid Grains | DEM↔MPM 적응 전환 | 그래픽스 |

## 3. DEM 파쇄 모델

| 연구 | 내용 |
|---|---|
| Potyondy & Cundall, *Int. J. Rock Mech. Min. Sci.* 41 (2004) 1329 — [doi](https://doi.org/10.1016/j.ijrmms.2004.09.011) | BPM (bonded-particle model) 원전 |
| Tavares et al., *Powder Technol.* 392 (2021) 473–488 | sphere-based breakage (UFRJ) — Leeds 가 사용 |
| Tavares & Chagas, *Powder Technol.* 377 (2021) 222–232 | stochastic particle replacement |
| **Campos, Powell, Tavares, *Powder Technol.* 460 (2025) 120949** | **단일 입자 → 구속 입자층 파쇄 전이 (느린 압축)** — 구속 효과, 봉투 구속 논의에 직접 관련 |
| Orefice & Khinast, *Powder Technol.* 368 (2020) 90–104 | 변형·파쇄 가능 DEM 클러스터 압축 |
| Schreiner et al., *Procedia CIRP* 93 (2020) 149 | 캘린더링 DEM 파라미터화 |
| Stasiak et al. | 중공 튜브 입자 고압축 3D DEM |
| Nader et al. — [arXiv 2203.13679](https://arxiv.org/pdf/2203.13679) | oedometer 압축 파쇄 에너지 DEM |
| Hanley, O'Sullivan, Huang | 등방/삼축 압축 DEM 파쇄 |
| DEM–PBM (텀블링 밀) — [UCT](https://open.uct.ac.za/handle/11427/5521), [Springer](https://link.springer.com/article/10.1007/BF03402260), [Edinburgh](https://eng.ed.ac.uk/node/10127) | DEM 충돌 에너지 → PBM 입도 예측 |

## 4. 연속체 파쇄 구성식

| 연구 | 내용 |
|---|---|
| Einav, Breakage mechanics Part I/II (JMPS, 2007) | 파쇄 변수 B(0–1) 로 PSD 진화 — MPM 커스텀 재료 후보. [저자 페이지](https://typeset.io/authors/itai-einav-5fxb3eq0g6) |
| Breakage mechanics in large granular flow (USyd) | [link](https://ses.library.usyd.edu.au/handle/2123/34948?show=full) |
| Ulloa, Ortiz et al. | data-driven breakage mechanics. [COMPLAS 2023](https://complas2023.cimne.com/event/contribution/2ed96723-af2f-11ed-b019-000c29ddfc0c) |
| DPC (Drucker–Prager Cap) FEM | 제약 압밀 표준. Han et al. 2008, Sinha et al. 2010; [Cambridge dDPC](https://www.repository.cam.ac.uk/items/79eee568-10b5-45b7-ae42-47ad88c7d1f0) |

## 5. 신경망 · PINN 결합

| 연구 | 내용 | PINN? |
|---|---|---|
| Ma et al., ICML 2023 NCLaw — [arXiv 2304.14369](https://arxiv.org/abs/2304.14369) | 미분가능 MLS-MPM 안에서 신경망 구성식 학습 | 물리 prior 내장, 엄밀 PINN 아님 |
| PC-NCLaws — [arXiv 2510.21404](https://arxiv.org/abs/2510.21404) | 조건부·다재료 탄소성 확장 | 〃 |
| Chen et al., Iowa — [MPM-ANN](https://iro.uiowa.edu/esploro/outputs/journalArticle/Hierarchical-MPM-ANN-Multiscale-Terrain-Model-for/9984413071602771) | MPM×DEM 의 DEM RVE 를 ANN 으로 대체 | 아님 |
| Rangel, Oñate et al., *Comput. Geotech.* 176 (2024) — [link](https://scipedia.com/public/Rangel_et_al_2024c) | FVM + DEM-학습 ANN 대리모델 | 아님 |
| Chen et al., *Chem. Eng. J.* (2021) — [link](https://cora.ucc.ie/items/37bb3117-68c5-4cc6-bec2-e482bb8da017) | 응집·파쇄 PBE 를 손실에 넣은 PINN | **예** (역학 없음) |
| TAML (2024) — [link](https://doaj.org/article/83482e9e866e46499e8eb18de188282b) | 파쇄 동역학 multi-head PINN | **예** |
| MDPI (2026) — [PDF](https://mdpi-res.com/d_attachment/mca/mca-31-00142/article_deploy/mca-31-00142.pdf) | 물리제약 NN 으로 파쇄 법칙 역동정 | 부분 |
| Darlik, Univ. Luxembourg 학위논문 — [link](https://orbilu.uni.lu/handle/10993/60405) | PINN 을 DEM 대리모델로 (바이오매스 이동층) | **예** |
| PINNSR-DA — [arXiv 2509.14518](https://arxiv.org/html/2509.14518v2) | DEM 전단 데이터에서 PINN+희소회귀로 마찰법칙 발견 | **예** |
| FINN, *Géotechnique* (2025) — [link](https://www.citedrive.com/en/discovery/finn-fabric-informed-neural-network-for-data-driven-discovery-of-constitutive-relations-for-granular-materials/) | fabric 제약 DEM 학습 구성식 | 부분 |
| Tejada & Antolin — [arXiv 2006.05711](https://arxiv.org/abs/2006.05711) | DEM 9만 팩킹 → NN 으로 PSD-역학 상관 | 아님 |
| NeuralDEM, ICLR 2025 — [link](https://iclr.cc/virtual/2025/32956) | DEM 종단간 대리 | 아님 |
| Vijay et al., *Energy Storage Mater.* 75 (2025) 103883 | 전극 제조 물리-딥러닝 하이브리드 (Franco 그룹) | 아님 |
| DPC 파라미터 ANN 역추정 — [Shahrood](https://shahroodut.ac.ir/en/thesis/thesis.php?thid=TJ597), [Al7075](https://research.basu.ac.ir/_Pages/Research.aspx?Id=1043272), [KAIST Shima-Oyane](https://koasas.kaist.ac.kr/handle/10203/312525) | 압밀 곡선 → 재료상수 | 아님 |

> 결론: **분말 압밀·분쇄 + DEM + PINN(물리 손실)** 조합은 찾지 못함. 가장 가까운 것은 1-1(Leeds, 데이터 손실 MLP).

## 6. 비전 기반 입도 분포 측정

| 분야 | 방법 | 출처 |
|---|---|---|
| 제약 분말(밀링 포함) | 스페클 자기상관 → 딥러닝 PSD | [MIT Myerson](https://myersongroup.mit.edu/control-of-powder-based-pharmaceutical-unit-operations-employing-imaging-and-machine-learning/), [MIT 특허](https://tlo.mit.edu/industry-entrepreneurs/available-technologies/system-and-method-real-time-determination-particle) |
| 제약 펠릿 in-line | 내시경 + 고속카메라 + CNN 인스턴스 분할 | [PMC 2025](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12491717/) |
| 유동층 과립·밀링 | 직접 영상 in/at-line | [PharmTech 2015](https://www.pharmtech.com/view/monitoring-fluid-bed-granulation-and-milling-processes-line-real-time-imaging), [SPIE 2025](https://astronomicaltelescopes.spiedigitallibrary.org/conference-proceedings-of-spie/13486/134861H/Machine-vision-and-image-processing-for-intelligent-online-monitoring-and/10.1117/12.3055987.full), [Strathclyde](https://strathprints.strath.ac.uk/64640) |
| 미분탄 | Mask R-CNN | [link](https://www.researching.cn/ArticlePdf/m00018/2021/50/10/2021G004.pdf) |
| 금속 분말(AM) | Pix2Pix 분할, 레이저회절 대비 ~7% | [AM Letters 2024](https://doaj.org/article/69527d68ac904c258ac3e1cfbdb376f2) |
| 광산 파쇄기·발파 | 비접촉 PSD, 체가름 보정 | [Leoben](https://pure.unileoben.ac.at/en/publications/ber%C3%BChrungslose-korngr%C3%B6%C3%9Fenverteilungserkennung-im-mobilen-brecherb/), [발파 절차](https://repository.geologyscience.ru/items/a2ed83a8-bcaf-4d98-8a1d-f282aa1a075d/full), [MST](https://web.mst.edu/norbert/pdf/caljour.pdf) |
| Rocky DEM HPGR 검증 | 롤 힘·동력만 정합 (PSD 아님) | [Brunel](https://bura.brunel.ac.uk/bitstream/2438/18073/2/FullText.pdf) |

> 공백: **비전으로 잰 분쇄 후 PSD 를 DEM 파쇄 모델 보정 타깃으로 쓴 사례**는 찾지 못함.
> 공통 약점: 2D 투영 → 3D 부피 변환 오차(방법 간 평균크기 최대 2배 차), 해상도 이하 미분.

---

## 7. 우리 연구 대비 노벨티 판단

현 상태 (2026-10-21): Newton_sim `crusher-scene` — 500 µm iron 3 g, PTFE 봉투, 크랭크-슬라이더 판을
**토크 상한 F_cap(θ)=τ/(r sinθ)** 로 구동(stall), DEM(비결합·단분산, **파쇄 없음**) vs MPM 반력·비용 비교.
Crusher_IsaacSim — 실기 ForceGage F(θ) MuJoCo 재현(30–90° ±10%), 전 공정 트윈.

| 축 | Leeds 2026 | 우리 | 판정 |
|---|---|---|---|
| 하중 형태 | 평판 일축, 전단 없음 | 판–앤빌 일축, 전단 없음 | **같음** |
| DEM + 대리모델 + 역보정 | 있음 | 계획 | **같음** |
| 입도 분포 | 정규분포 + 변동 실험 | 단분산 | Leeds 우위 |
| 구동 경계 | 변위 지정 | 토크 상한 기구 동역학, stall | **다름** |
| 반력 사용 | 사후 학습 데이터 | 제어 주기 stall 판정(실기 FSM 동일) | **다름** |
| 반복 하중 | 단조 1회 | STRIKE/RETRACT 반복 (N_f, W*) | **다름 — 가장 강함** |
| 구속 | 강체 박스 | 유연 봉투(부분 구속, 측면 유출) | 다름 |
| 검증 | 문헌 실험(Zhang 2022) | 자작 실기 ForceGage | 다름 |
| 솔버 | DEM | DEM vs MPM + 봉투 FEM/VBD | 다름 |

**가장 강한 노벨티 후보 — 반복 타격 손상누적**: Leeds 는 γ(손상누적)를 "무시 가능"으로 결론냈으나
단조 1회 압축이라 손상이 누적될 기회가 없다. 반복 하중에서 γ 가 지배 파라미터가 됨을 실측 반력 + DEM 으로 보이면
Leeds 결론의 적용 한계를 직접 반박. 여기에 (1) 동역학 경계 + 실시간 반력 순차 모델, (2) 봉투 부분 구속
(Leeds 의 E∞ 과대추정 한계, Campos et al. 2025 와 연결), (3) 스트로크별 비전 PSD 측정을 묶는다.

**전제 확인 필요**: 위 1순위는 대상 분말이 실제로 파쇄되는 재료일 때만 성립. 500 µm iron 은 이 하중에서
사실상 깨지지 않으므로, 압밀만 다룬다면 노벨티를 "동역학 경계 + 실시간 반력 + 봉투 구속 + 멀티솔버 트윈"으로 재구성해야 한다.
