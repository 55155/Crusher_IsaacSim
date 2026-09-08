# probe/ — 변인통제 실험 기록 (2026-09-04 ~ 09-07)

정식 파이프라인이 아니라 **"무엇이 되고 무엇이 안 되는지"를 가르기 위해 돌린 격리
실험**들이다. 각 스크립트는 가설 하나를 검증하고 버려지는 대신 여기 남는다 —
나중에 "이걸 왜 이렇게 했더라"를 되짚기 위한 기록이다.

전부 GPU 씬을 직접 띄우므로 실행 전 결과를 다시 얻고 싶을 때만 돌리면 된다.
공통 파이썬: `C:/Users/simuser/miniconda3/envs/crusher_genesis/python.exe`

---

## 1. MPM 을 쓸 수 있는가 — 결론: 못 쓴다

| 스크립트 | 물은 것 | 답 |
|---|---|---|
| `probe_mpm_ipc.py` | MPM 엔티티가 IPC 커플러에서 스텝되는가 | **아니오.** `is_active=True` 인데 120스텝 동안 z 300.8→300.8mm. 한 번도 전진하지 않는다 |
| `probe_cloth_mpm.py` | FEM.Cloth + MPM.Sand 를 legacy 커플러에 같이 | 천이 100스텝 내 NaN |
| `probe_cloth_legacy.py` | 위가 타임스텝 문제인가 (substeps 1/50/200) | **아니다.** substep_dt 를 200배 줄여도 동일하게 NaN. `Cloth` 는 IPC 전용이고 legacy FEM 솔버가 보는 질량이 placeholder(`mass=1.0, mass_over_dt2=0.0`) |
| `probe_cpic.py` | `enable_CPIC` 로 얇은 벽 관통을 막을 수 있는가 | CPIC 단독으로는 효과 없음. **`SimOptions.substeps` 가 지배 인자** — 없으면 모래가 0.5초 만에 도메인 경계까지 터진다 |
| `probe_fill.py` | 위 레시피로 실제로 담기는가 | substeps=50 + grid_density=200 + CPIC 로 용기 안 100% |
| `probe_sand_container.py` | 봉투 STL 을 강체 용기로 쓰면 담기는가 | 흩어짐. 얇은 셸의 SDF 가 MPM 격자 해상도에 진다 |
| `probe_bagfill.py` | 입구가 얼마나 넓어야 모래가 들어가는가 | **입구/입자 비가 지배한다.** 1.9 → 22% 담김, 10.9 → 100% |

**요약**: IPC 커플러에는 MPM 코드가 한 줄도 없고, `FEM.Cloth` 는 IPC 전용이다.
둘은 배타적 커플러를 요구하므로 한 씬에 못 넣는다.

## 2. uipc 네이티브 Particle 로 우회 — 결론: 된다

| 스크립트 | 물은 것 | 답 |
|---|---|---|
| `probe_abd_particle.py` | `Particle` 과 `AffineBodyConstitution`(움직이는 기구물)이 공존하는가 | **된다.** 처음엔 실패했는데 원인은 공존이 아니라 **초기 겹침**이었다 |
| `probe_abd_grains.py` | 낟알을 ABD 강체 구로 만들면 어떤가 | 같은 실패 — 용기 벽끼리 모서리에서 겹쳐 있었다 |
| `probe_order.py` | 등록 순서(낟알 먼저)를 바꾸면 풀리는가 | 아니다. 순서와 무관 |
| `probe_placement_vid.py` | 격자 / 난수 / 기각표집 배치 비교 (영상) | 아래 §3 |
| `probe_grain_crusher.py` | 낟알↔실제 Crusher 벽 커플링 | **성립.** 벽 지연이 −0.76(대조군) → −3.66mm 로 자란다 = 낟알이 벽을 되민다. 이후 `ipc_grain_coupler.py` 의 `TEST_MODE=crusher` 로 이식 |

### 가장 중요한 함정 — 초기 겹침

libuipc 는 빌드 때 표면 거리 검사(`SimplicialSurfaceDistanceCheck`)를 하고, 겹친
지오메트리가 있으면 **월드를 통째로 무효화한다**:

```
[error] World is not valid, skipping init.
[error] World is not valid, skipping advance.   ← 매 스텝 반복
```

증상이 두 가지로 갈리는데, 둘 다 같은 원인이다:

| 상황 | 겉보기 |
|---|---|
| 겹침 + ABD 있음 | `AttributeError: 'NoneType' object has no attribute 'body_count'` — 시끄럽게 실패 |
| 겹침 + ABD 없음 | `build()` 가 "성공"하고 **매 스텝 아무 일도 없이 지나간다** (조용한 정지) |

**파이썬 예외만 보지 말고 libuipc 의 `[error]` 줄을 반드시 확인할 것.** 이 함정에
두 번 걸려 잘못된 결론(“Particle × ABD 는 구조적으로 불가”)을 냈다가 뒤집었다.

## 3. 초기 배치 방법

`probe_placement_vid.py lattice|random|poisson` — 같은 낟알 64개, 배치만 다르다.

| 배치 | 최근접 | 겹친 쌍 | 결과 | 안착 후 최근접 표준편차 |
|---|---|---|---|---|
| 격자 | 8.00mm | 0 | 빌드 OK | **0.00mm** (결정처럼 쌓임 — 비현실적) |
| 난수 `uniform()` | 1.50mm | 5 | **조용히 정지** | — |
| **기각표집** | 4.04mm | 0 | 빌드 OK | **1.57mm** (무질서한 더미) |

기각표집(dart throwing) = 무작위로 뽑되 최소간격(2R+여유)을 못 지키면 버리고 다시
뽑는 것. **격자는 대칭이 안 깨져서 실제 분체 더미가 안 나온다** — 무작위성이
필요하면 기각표집을 써야 한다.

## 4. 분석 도구

| 스크립트 | 용도 |
|---|---|
| `plot_moment2.py <npz글롭> <출력png>` | 무게별 파우더 무게중심·모멘트 3D 플롯. **모멘트는 수평 오프셋에만 비례**한다(M = r × W, W 연직이라 연직 성분은 외적에서 사라짐) |
| `parse_step.py <step파일>` | STEP 파싱 — `Base.step` 에서 실판 규격(750×800×30, D6.0 구멍 240개) 추출에 사용 |
| `track_check.py <mp4>` | bagcam 이 봉투를 추적하는지 검증. 실링 스트라이프 무게중심의 화면중심 오차를 잰다 |

## 5. 드라이버 (`drivers/`)

| 스크립트 | 스윕 내용 |
|---|---|
| `grain_budget.sh` | 낟알 수/크기 예산. 60/200/500 담김, **1000 에서 9.4% 누출**. 비용 ~O(N^1.17) |
| `mass_sweep.sh` | 무게 0.25/0.5/1/2 g (1mm 소금 221/442/884/1768알). 전부 0% 누출 |
| `ecc_sweep.sh` | 편심 투입 0 vs +20mm. **모멘트 0.0129 → 0.2281 mN·m (17.7배)** |
| `crusher_test.sh` / `crush_ab.sh` | 낟알↔Crusher, 반력 계측 채널 확인 |
| `grain_fullwf_sweep.sh` | **full_workflow 환경** 낟알 병목 7조 — 장입량/newton_tol/정착시간/반지름 (2026-09-08) |

> `grain_fullwf_sweep.sh` 만 여기 다른 것들과 성격이 다르다. 나머지는 격리 씬을
> 돌리지만 이건 `full_workflow.py` 전 구간을 돌린다 — probe 씬에는 Genesis 강체가
> 아예 없어서 IPC 와 Genesis 두 솔버가 같이 도는 조건을 대변하지 못하기 때문이다
> (사용자 지시, 2026-09-08). 낟알 관련 판단은 이제 이 드라이버 결과로 한다.
> 배경은 `docs/DigitalTwin.md` §24.

### 반력은 `get_dofs_control_force()` 로 읽는다

`get_dofs_force()` 는 `dofs.force` 를 돌려주는데 그건 `control_dofs_force()` 가 쓰는
**외력 명령 버퍼**다. 위치제어로 모는 벽에서는 항상 ~0 이다. 실측 대조:

| k=524 | 낟알 OFF | 낟알 ON |
|---|---|---|
| 지연 | −0.76mm (일정) | −5.67mm |
| `get_dofs_control_force` | +0.20N | **−26.43N** |
| `get_dofs_force` | −0.00N | −0.21N |

`control_force` 는 POSITION 모드에서 `kp*(cmd−pos)+kv*(vel−cmd_vel)` 을 `force_range`
로 클램프해 돌려준다 = 액추에이터 실제 출력. 검산 kp×지연 = 28.4N vs 실측 26.4N.

## 6. 미해결

- **봉투를 슬롯에 넣은 격리 씬이 초기 겹침으로 안 뜬다.** 낟알과 무관하고(봉투만
  넣어도 동일), impact plate 를 빼도, 위치를 ±4mm 흔들어도 실패한다. 그런데
  `full_workflow.py STAGE=clamp` 는 **완전히 같은 좌표**로 정상 빌드된다 —
  차이는 씬 엔티티 구성에 있는데 아직 못 갈랐다.
- `ContactSystemFeature.contact_gradient()` 반환 형식 미확인 (Love–Weber 응력텐서용).
- 낟알 1000개에서 9.4% 샌 원인 미진단(구식 column 배치의 낙하속도 의심).
