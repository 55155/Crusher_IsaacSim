# fusion2xml — Fusion 360 → MuJoCo MJCF

Fusion 디자인을 MuJoCo `.xml`(MJCF)로 바로 내보내는 Add-In.

## 왜 fusion2urdf 로는 안 되나

URDF 에는 equality constraint 가 없다. 링크마다 부모가 **정확히 하나**인 트리만
표현할 수 있어서, 커넥팅로드-크랭크 같은 폐루프 기구는 원리적으로 담기지 않는다.
fusion2urdf 는 루프를 닫는 조인트를 URDF 에서 빼 사이드카 파일
(`*.constraints.mjcf`)에 좌표만 적어 두고, 사람이 MJCF 로 손으로 옮겨야 했다.

fusion2xml 은 처음부터 MJCF 를 쓴다. `<body>` 중첩은 여전히 트리지만, 루프를 닫는
조인트는 **같은 XML 안의 `<equality>`** 로 닫히므로 옮길 게 없다.

## 설치

폴더 통째로 Fusion 의 Add-Ins 폴더에 넣거나, `Utilities > ADD-INS >
Scripts and Add-Ins > Add-Ins` 탭에서 이 폴더를 추가하고 Run.

버튼: `Utilities` 탭 > `ADD-INS` 패널 > **Fusion2MJCF 내보내기**

## 어느 두 링크를 equality 로 묶을지

대화상자에서 고른다.

- **자동 감지** — 무향 그래프에서 사이클을 찾아 루프를 닫는 조인트를 자동으로
  빼서 equality 로 돌린다. Fusion 에서 루프를 이미 닫아 뒀다면 이것만으로 끝난다.
- **수동 지정** — `equality body1` / `equality body2` 드롭다운에서 두 링크를
  직접 고른다. 아직 Fusion 에서 루프를 안 닫아 뒀다면(=사이클이 없으면) 감지될
  게 없으므로 이 경로가 필요하다.
- **추가 쌍** — 여러 개를 묶으려면 `A:B:connect, C:D:weld` 형식으로 적는다.

### anchor 는 어디서 오나

1. 두 링크 사이에 **Fusion 조인트가 있으면 그 원점**을 쓴다. 이게 정상 경로다 —
   Fusion 에서 루프를 닫아 두면 위치가 정확히 나온다.
2. 조인트가 없으면 `anchor` 칸에 root 기준 좌표(m)를 직접 적는다. 비워 두면
   내보내기가 막힌다(조용히 틀린 위치로 나가는 것보다 낫다).

### connect 냐 weld 냐

끊는 조인트의 자유도에 맞춰 자동으로 고른다.

| 끊은 조인트 | equality | 정확한가 |
|---|---|---|
| `fixed` (0 DOF) | `weld` | 정확 (6자유도 전부 구속) |
| `Ball` (회전 3) | `connect` | 정확 (병진 3만 구속) |
| `revolute` (회전 1) | `connect` | 면외 회전 2자유도가 남는다. 루프의 회전축이 전부 평행한 **평면 기구면** 트리가 그 2를 이미 막고 있어 결과적으로 정확하고, 아니면 과소구속 |
| `prismatic` | `connect` | **과대구속** — 미끄럼 방향까지 막힌다. 루프 안의 다른 조인트를 끊는 편이 낫다 |

`connect` 가 남기는 자유도는 3 Prismatic 이 아니라 **구면(볼조인트) 3회전**이다.
"칠판 위 거동"(2병진+1회전)이 필요하면 `connect` 하나로는 안 되고, 평면성이
트리 쪽에서 보장돼야 한다.

## fusion2urdf 와 공유하는 좌표 규약

- 조인트 `xyz` 는 **root 기준 절대 위치**(m).
- 링크 프레임 원점 = 그 링크를 자식으로 갖는 조인트의 위치. base_link 는 원점.
- 모든 프레임이 root 와 축정렬(회전 없음) → 프레임 간 변환이 순수 평행이동.
- STL 은 `copy_occs` 로 컴포넌트를 복제해 **root 좌표**로 내보낸다.

덕분에 MJCF 변환이 뺄셈만으로 끝난다:

```
<body pos>  = 링크원점(자신) - 링크원점(부모)
<joint pos> = "0 0 0"
<geom pos>  = -링크원점(자신)
connect anchor = anchor(root) - 링크원점(body1)
weld relpose   = 링크원점(body2) - 링크원점(body1),  회전은 단위 사원수
```

## fusion2urdf 와 다르게 처리하는 것

- **하위 컴포넌트 안의 조인트까지 훑는다.** `make_joints_dict` 는 `root.joints`
  만 봐서, Fusion 에서는 붙여놨는데 트리가 끊기는 일이 있었다. 폐루프를 닫는
  조인트가 하필 그런 위치에 있으면 루프 자체가 안 보인다.
  As-built Joint(한글 UI 의 "현재 위치에서 접합")는 다루지 않는다 — 조인트
  원점이 없어 equality anchor 를 뽑을 근거가 없다.
- **`fullinertia` 순서를 바꾼다.** URDF 는 `ixx iyy izz ixy iyz ixz`, MJCF 는
  `ixx iyy izz ixy ixz iyz` — 뒤 두 개가 뒤바뀐다. 그대로 옮기면 관성이 조용히
  틀어진다.
- **관성을 미리 검사한다.** MuJoCo 는 관성 텐서가 양정치가 아니거나 주모멘트가
  삼각부등식을 어기면 로딩을 거부한다. URDF/Gazebo 는 그냥 넘어가므로
  fusion2urdf 로는 안 드러나던 문제가 여기서 처음 터진다. 어느 링크가 문제인지
  리포트에 적고, `compiler balanceinertia="true"` 로 로딩 자체는 되게 해 둔다.

## 출력

```
<모델명>_mjcf/
  <모델명>.xml           MuJoCo 가 바로 읽는 MJCF
  <모델명>.report.txt    equality 목록 / 경고 / 메시 내보내기 결과
  meshes/*.stl
```

```bash
python -m mujoco.viewer --mjcf=<모델명>_mjcf/<모델명>.xml
```

## 주의

- **메시를 내보내면 디자인이 바뀐다.** `copy_occs` 가 모든 컴포넌트를 복제하고
  원본 이름을 `old_component` 로 바꾼다(fusion2urdf 와 같은 방식). 저장하지 말고
  `Ctrl+Z` 로 되돌릴 것.
- 저장 경로에 한글이 있으면 MuJoCo 가 메시를 못 읽는 경우가 있다. 내보내기 후
  경고로 알려준다.

## 검증 범위

`core/Tree.py` 와 `core/Mjcf.py` 는 `adsk` 에 의존하지 않아 Fusion 밖에서 테스트
가능하다. 4절 링크 목(mock)으로 다음을 확인했다:

- 폐루프 자동 감지, 수동 지정, anchor 누락 시 차단
- 뒤집힌 조인트의 축 반전
- body pos 가 부모 상대 / geom pos 가 `-링크원점` / anchor 가 body1 로컬
- 관성 고유값·삼각부등식 검사, `fullinertia` 재배열
- **MuJoCo 실제 로딩 + 시뮬레이션**: 크랭크에만 토크를 걸었을 때 equality 가
  있는 쪽에서만 로커암까지 동력이 전달되고(2.4 rad 차이), 핀 벌어짐이
  1.8e-5 m 에 머무름

`core/Extract.py`, `core/Mesh.py`, 대화상자는 **Fusion 실기 미검증**이다.
