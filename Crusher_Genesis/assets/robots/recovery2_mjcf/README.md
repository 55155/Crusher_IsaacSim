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
기존 `회수장치2_description` 은 `full_workflow.py` 의 `RECOVERY2_MJCF` 가
계속 참조하므로 그대로 둔다.

경로를 ASCII 로 잡은 것은 의도적이다 — 한글 경로면
`paths.ascii_safe_mjcf` 미러를 타야 하고, 그 미러가 부분 복사 상태에서
영구히 막히는 버그가 있었다(커밋 320ffee).

## 검증 (2026-08-18, MuJoCo)

`recovery2.xml` 단독 로드 → bodies=34, joints=3, eq=1, geoms=66, meshes=33.
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
