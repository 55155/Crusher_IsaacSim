# recovery2_mjcf — 회수장치2 MJCF (fusion2xml 직행 빌드)

Fusion360 → MJCF 를 URDF 경유 없이 직접 내보낸 빌드(`utils/fusion2xml`).
2026-08-12 빌드분을 그대로 가져왔다.

## 회수장치2_description 과의 관계 — **둘은 다른 자산이다**

| | `회수장치2_description/회수장치2.xml` | `recovery2_mjcf/recovery2.xml` (이것) |
|---|---|---|
| 파이프라인 | fusion2urdf → URDF → MJCF | fusion2xml (MJCF 직행) |
| 폐루프 | `connect crank_loop_close`<br>`body1="Crank_1_b"` (더미 바디) ↔ `Crank_1` | `connect connect_Crank_1_M_Top_1`<br>`body1="Crank_1"` ↔ `body2="M_Top_1"` (둘 다 실제 바디) |
| 메시 | 별도 재exported | **33개 전부 바이트 다름** |

메시 파일명이 서로 겹치지만 **내용이 다르다**. 두 `meshes/` 를 섞지 말 것.
**2026-08-19: 시뮬레이터 쪽 참조를 전부 이 빌드로 옮겼다** — `full_workflow.py`
(`RECOVERY2_MJCF`, 따라서 `full_workflow_rigid.py` 도), `Recovery2_only/` 3종,
`Fixture_only/` 스택 2종. 구 `회수장치2_description` 은 대조용으로만 남긴다.

경로를 ASCII 로 잡은 것은 의도적이다 — 한글 경로면
`paths.ascii_safe_mjcf` 미러를 타야 하고, 그 미러가 부분 복사 상태에서
영구히 막히는 버그가 있었다(커밋 320ffee).

## 시각 메시 전처리 `*_ss.obj` (2026-08-19 추가)

Genesis(pyrender) 의 smooth 셰이딩은 `trimesh.vertex_normals` 를 크리스 각도
구분 없이 그대로 써서, 볼트머리 같은 작은 디테일 부근에 **별모양 스파이크**를
만든다(원인·경위는 docs/DigitalTwin.md "고정장치/M0609 렌더링 스파이크
아티팩트 — 해결(2026-07-21)"). fusion2xml 빌드는 시각/충돌 geom 이 같은 STL 을
보고 있어서 이 아티팩트가 그대로 나온다.

그래서 구 `회수장치2_description` 과 같은 처방을 적용했다:

- `utills/smooth_shade_meshes.py` 로 33개 STL 을 `trimesh.smooth_shaded`
  (크리스 각도 기준 정점 분리 후 스무싱)로 재수출 → `meshes/<이름>_ss.obj`
- `recovery2.xml` 의 `<mesh>` 자산에 `<이름>_ss` 를 추가하고, **`*_vis` geom만**
  그것을 보게 했다. `*_col` geom 은 원본 STL 그대로다 — `_ss` 는 정점을 쪼갠
  결과라 non-watertight 이고 충돌/SDF 에 쓰면 깨진다.

재생성:

```
python utills/smooth_shade_meshes.py Crusher_Genesis/assets/robots/recovery2_mjcf/meshes --force
```

## 검증 (2026-08-18, MuJoCo)

`recovery2.xml` 단독 로드 → bodies=34, joints=3, eq=1, geoms=66, meshes=33
(`_ss.obj` 추가 후로는 meshes=66 — 시각 33 + 충돌 33).
크랭크 힌지에 0.5N·m 를 걸고 6초(3000스텝) 구동:

- 크랭크 +14.9rad(약 2.4회전) 연속 회전 — 사점에 걸리지 않는다
- 슬라이더 0 ~ +35.0mm 왕복 (스트로크 35.0mm) — 폐루프가 회전을 직선운동으로 전달
- 구속 잔차 최대 1.8mm 로 **유계**, 회전을 거듭해도 증가하지 않음
- 발산 없음

> 잔차를 볼 때 `qvel` 을 매 스텝 덮어써서 구동하면 구속 솔버의 보정을
> 무력화해 잔차가 단조 증가하는 것처럼 보인다. 토크로 구동해야 실제
> 거동이 나온다.

## 알려진 거친 부분

- 조인트 이름이 Fusion 자동생성 한글이다 — `슬라이더_35`, `회전_31`, `회전_29`.
- 원 빌드에 있던 렌더 확인용 변형(`recovery2_jaws.xml`, `recovery2_marked.xml`)과
  애니메이션 GIF(`views/`, 34MB)는 자산이 아니라 검증 기록이라 넣지 않았다.
