# Crusher_IsaacSim

**약품 분쇄기(Crusher)** 의 Isaac Sim → MuJoCo 마이그레이션 및 정제(Tablet) 형상 시뮬레이션 통합 저장소입니다.

---

## 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [디렉토리 구조](#디렉토리-구조)
3. [빠른 시작](#빠른-시작)
4. [변경 이력](#변경-이력)
5. [의존성 설치](#의존성-설치)
6. [주의 사항](#주의-사항)

> **✅ 2026-05-27 신규** — Crusher + Tablet 통합 시뮬레이션 환경 완성.  
> STL 파일 하나를 지정하면 알약을 Crusher 내부에 배치하고 접촉력을 자동 측정합니다. → [바로가기](#5-crusher--tablet-통합-시뮬레이션-실행)

---

## 프로젝트 개요

| 구성 요소 | 설명 |
|-----------|------|
| **Crusher_IsaacSim_description** | ROS URDF + Isaac Sim USD 원본 기술 파일 |
| **MuJoCo_PlayGround** | Crusher 모델을 MuJoCo(MJCF)로 변환·시뮬레이션 |
| **tablets_stl** | 경구정(Tablet) 형상 파라미터 기반 STL 생성·시각화·충돌 분해 |

---

## 디렉토리 구조

```
Crusher_IsaacSim/
│
├── README.md                          ← 이 파일
├── requirements.txt                   ← Python 의존성 목록
├── .gitignore                         ← STL 바이너리 제외 설정
│
├── Crusher_IsaacSim_description/      ← ROS 패키지 원본
│   ├── urdf/                          ← URDF / xacro / USD 파일
│   ├── meshes/                        ← 부품별 STL 메시
│   └── Code/                          ← 보조 스크립트 (질량/관성 검사, 재질 등)
│
├── CODES/                             ← 초기 Isaac Sim 디버그 스크립트 모음
│
├── 20260505/ ~ 20260513/              ← 날짜별 Isaac Sim 작업 스크립트
│
├── MuJoCo_PlayGround/                 ← MuJoCo 시뮬레이션 작업 공간
│   ├── MJCF/
│   │   ├── Crusher_IsaacSim.xml       ← 기본 MJCF (색상 없음)
│   │   ├── Crusher_IsaacSim_colored.xml  ← 색상 + Ground + 폐루프 제약 완성본
│   │   ├── Ground.stl                 ← 바닥 메시
│   │   └── L*.stl / base_link.stl    ← 부품 메시 (42개)
│   │
│   ├── 20260514/  view_mjcf.py        ← 초기 MJCF 로더
│   ├── 20260515/  open_crusher.py     ← 기본 XML 뷰어 (경고 억제 포함)
│   ├── 20260518/                      ← 색상 파이프라인 스크립트
│   │   ├── apply_csv_to_mjcf.py       ← mesh_colors.csv → MJCF material 적용
│   │   ├── extract_materials.py       ← USD에서 재질 추출
│   │   ├── make_mesh_csv.py           ← 메시-색상 CSV 생성
│   │   └── fusion360/                 ← Fusion 360 appearances.json 기반 적용
│   │
│   ├── 20260520/
│   │   └── open_crusher_colored.py   ← ★ 색상+Ground XML Python 뷰어 (신규)
│   │
│   ├── 20260527/
│   │   └── crusher_tablet_sim.py     ← ★ Crusher + Tablet 통합 시뮬레이션 (2-Phase)
│   │
│   ├── mujoco.bat                     ← ★ 더블클릭으로 뷰어 실행 (상대경로 수정)
│   └── convert_urdf_to_mjcf.py        ← URDF → MJCF 변환기
│
└── tablets_stl/                       ← 정제 형상 도구 모음
    ├── stl/
    │   ├── _tablet_index.csv          ← 1000개 정제 파라미터 메타데이터
    │   └── tablet_R*.stl              ← 생성된 STL 파일 (git 제외)
    │
    └── Codes/
        ├── viewer_desktop.py          ← ★ PyVista 3-슬라이더 형상 뷰어 + MuJoCo 연동
        ├── launch_mujoco.py           ← ★ STL → MuJoCo 물리 시뮬레이션 런처
        ├── collision_viewer.py        ← ★ 볼록 분해 4-모드 충돌 뷰어
        ├── tablet_generator.py        ← Fusion 360 AddIn (로컬 실행 불가, 참조용)
        ├── viewer.html                ← 웹 기반 STL 브라우저
        ├── start_viewer.bat           ← viewer_desktop.py 실행 배치
        ├── run_collision_viewer.bat   ← collision_viewer.py 실행 배치
        └── install_and_run.bat        ← 의존성 설치 + 뷰어 실행 배치
```

---

## 빠른 시작

### 1. Crusher MuJoCo 뷰어 실행

```bash
# 방법 A — 더블클릭 (권장)
MuJoCo_PlayGround/mujoco.bat

# 방법 B — Python 스크립트
conda activate isaac_sim
python MuJoCo_PlayGround/20260520/open_crusher_colored.py
```

> `mujoco.bat`은 `.py` 파일을 경유하지 않고 `python -m mujoco.viewer --mjcf` 명령으로  
> 설치된 MuJoCo 패키지의 CLI 뷰어를 **직접** 실행합니다.

### 2. 정제 형상 뷰어 실행

```bash
conda activate isaac_sim
python tablets_stl/Codes/viewer_desktop.py
```

- 슬라이더로 `R` / `AR` / `CV` 파라미터 조정 → 형상 실시간 미리보기
- **[MuJoCo에서 열기]** 버튼 → 선택한 STL을 별도 창에서 물리 시뮬레이션

### 3. 충돌 메시 분해 뷰어 실행

```bash
conda activate isaac_sim
python tablets_stl/Codes/collision_viewer.py
```

- 4가지 보기 모드: 원본 메시 / 볼록 껍질 / 겹침 보기 / 면 법선
- 거친(8) / 표준(16) / 정밀(32) / 최정밀(64) hull 프리셋

### 5. Crusher + Tablet 통합 시뮬레이션 실행

```bash
conda activate isaac_sim
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py <tablet.stl>

# 파일 선택 다이얼로그로 실행
python MuJoCo_PlayGround/20260527/crusher_tablet_sim.py
```

#### 동작 방식

| 단계 | 내용 |
|------|------|
| **Phase 1** (뷰어 없음, 500 스텝) | 크랭크 90° 세팅 + 알약 위치 고정 → Crusher 메커니즘 안정화 |
| **Phase 2** (뷰어 오픈) | 알약을 충돌판(impact plate) 벽면에 고정한 채 모터 구동 → 접촉 반력 측정 |

#### 핵심 구조

- **XML 파일 없음** — `Crusher_IsaacSim_colored.xml` 을 메모리에서 파싱 후 Tablet body/sensor 노드를 추가, `MjModel.from_xml_string(xml_str, assets={"tablet.stl": bytes})` 으로 직접 로드
- **알약 고정 방식** — `freejoint`(6자유도)를 달고 매 스텝 `qpos`·`qvel` 을 초기값으로 덮어씀 (kinematic hold). 물리 엔진은 계속 동작하므로 `cfrc_ext` 접촉 반력이 정상 측정됨
- **출력** — 위치·힘·임펄스 그래프 3종을 PNG로 저장

#### 알약 배치 좌표 (MuJoCo world frame)

| 축 | 값 | 의미 |
|----|----|------|
| X | −47.879 mm | 충돌판 중심 정렬 |
| Y | 336.199 mm | 충돌판(back wall) 면에 밀착 |
| Z | 50.108 mm | 수직 높이 |

---

### 4. 정제 STL 생성 (Fusion 360 전용)

`tablets_stl/Codes/tablet_generator.py` 참조.  
로컬에서 직접 실행 불가 — Fusion 360 내부 AddIn으로만 사용 가능합니다.

---

## 변경 이력

### 2026-05-27 — Crusher + Tablet 통합 시뮬레이션 완성

#### 신규 파일

| 파일 | 내용 |
|------|------|
| `MuJoCo_PlayGround/20260527/crusher_tablet_sim.py` | Crusher XML + Tablet STL을 메모리에서 조합해 2-Phase 통합 시뮬레이션 실행. Phase 1(메커니즘 안정화) → Phase 2(알약 고정 + 접촉력 측정 + 그래프 저장) |

#### 주요 변경 사항

| 항목 | 내용 |
|------|------|
| **알약 고정 방식** | freejoint(6자유도) + 매 스텝 kinematic hold (qpos/qvel 덮어쓰기) → 알약을 공간에 고정하면서 cfrc_ext 접촉 반력 측정 가능 |
| **알약 배치** | Y 좌표를 `WALL_Y_MM`(336.199 mm)으로 설정 — 충돌판 벽면에 중심 밀착 |
| **MJCF 구성** | 별도 XML 파일 없음 — 기존 Crusher XML을 메모리에서 파싱 후 Tablet body / sensor 노드 동적 삽입 |
| **뷰어 site 구 제거** | `sitegroup` 전체 `False` — 중심부 구 형태 gizmo 비표시 |

---

### 2026-05-26 — Crank-Slider 메커니즘 진단 및 좌표계 분석

#### 핵심 발견

| 항목 | 내용 |
|------|------|
| **Equality Constraint 타입 오류** | `connect`는 위치(3-DOF)만 구속 → L7-L8 사이 회전 자유도 미구속으로 진동 발생. `weld`(위치+방향 완전 고정)로 교체 필요 |
| **Constraint Gap 원인 분석** | anchor=0.027 설정 시 L8 body와 10 mm 갭 발생 → 제약력 500 N (중력 12.7 N의 **39배**) → CCW 비정상 회전 유발 |
| **크랭크 회전축 오해** | XML `axis="0 0 1"` (body local Z) + `quat="0.5 0.5 0.5 0.5"` → 실제 world 회전축 = **[1, 0, 0] (world X)**. 중력(-Z)과 수직이므로 최대 토크 발생 |
| **MuJoCo 좌표계 변환** | `quat="0.5 0.5 0.5 0.5"` → R=`[[0,0,1],[1,0,0],[0,1,0]]` : URDF X→MuJoCo Y, URDF Y→MuJoCo Z, URDF Z→MuJoCo X |

#### 신규 파일

| 파일 | 내용 |
|------|------|
| `MuJoCo_PlayGround/20260526/coordinate_gizmo.py` | MuJoCo 월드 좌표계(XYZ 기즈모) 확인 스크립트. 뷰어 원점에 body frame 표시 |
| `MuJoCo_PlayGround/20260526/open_urdf.py` | 원본 URDF를 MuJoCo로 직접 로드해 각 body 월드 위치·joint 목록 출력 및 뷰어 실행 |

#### MJCF 수정 이력 (Crusher_IsaacSim_colored.xml)

| 수정 | 내용 |
|------|------|
| L5-L6, L6-L7 hinge damping 추가 | `damping="0.5"` — 초기 불안정 진동 억제 |
| L8 slide joint damping 추가 | `damping="5"` |
| L8_Link3_Shaft_1 body pos 수정 | Rigid32 접합 위치 = `(-0.048302, 0.236278, 0.049431)` (URDF 실측 검증 완료) |
| Equality Constraint anchor | URDF Rigid32 joint xyz=(0.037, 0, -0.005) 기준. **다음 단계: connect → weld 교체 예정** |
| 뷰어 설정 | `mjVIS_CONVEXHULL=False`, `geomgroup[3]=False` — visual mesh 전용 표시 |

---

### 2026-05-21 — macOS 호환성 수정 및 tablet_generator Fusion 360 등록

| 파일 | 변경 내용 |
|------|-----------|
| `tablets_stl/Codes/viewer_desktop.py` | `subprocess.CREATE_NEW_CONSOLE` (Windows 전용) → `platform.system()` 감지 후 조건부 적용으로 macOS 호환 수정 |
| `tablets_stl/stl/_tablet_index.csv` | Fusion 360 `tablet_generator` AddIn 실행으로 정제 STL 1,000개 생성 및 인덱스 갱신 |

---

### 2026-05-20 — MuJoCo 통합 완성 및 구조 정리

#### MuJoCo_PlayGround

| 파일 | 변경 내용 |
|------|-----------|
| `mujoco.bat` | 하드코딩된 절대경로 → `%~dp0` 기반 상대경로로 전면 수정; conda 활성화 경로도 `%USERPROFILE%` 변수로 포터블화 |
| `20260520/open_crusher_colored.py` | **신규** — `Crusher_IsaacSim_colored.xml`(Ground.stl 포함)을 Python API(`mujoco.viewer.launch`)로 여는 스크립트; `__file__` 기준 상대경로 사용 |

#### tablets_stl

| 파일 | 변경 내용 |
|------|-----------|
| `Codes/viewer_desktop.py` | **[MuJoCo에서 열기]** 버튼 추가; STL 로드 성공 시 활성화, `subprocess.Popen`으로 비차단 실행 |
| `Codes/launch_mujoco.py` | **신규** — 선택된 STL을 메모리 내 MJCF로 변환(`from_xml_string + assets`), 물리 파라미터(밀도 1200 kg/m³, 마찰 0.5) 설정 후 뷰어 실행 |
| `Codes/launch_mujoco.py` | **버그 수정** — `_print_params()` 정규식이 파일명의 `.stl` 확장자까지 캡처하여 `float('0.20.')` 오류 발생 → `os.path.splitext()` 로 확장자 제거 후 정규식 적용 |
| `Codes/collision_viewer.py` | **신규** — 볼록 분해(CoACD → VHACD → single hull fallback) 독립 뷰어; 4-모드 시각화 |

#### 저장소 공통

| 파일 | 변경 내용 |
|------|-----------|
| `requirements.txt` | **신규** — 그룹별 의존성 정리 (수치연산 / 3D시각화 / 메시처리 / MuJoCo / JAX / 이미지처리); 비pip 패키지(Isaac Sim, Fusion 360, Blender) 설치 안내 주석 포함 |
| `.gitignore` | **신규** — `tablets_stl/stl/*.stl`, `tablets_stl/collision/*.stl` 바이너리 제외; Python 캐시 제외 |

---

### 2026-05-18 — Crusher 색상 파이프라인 완성

| 변경 내용 |
|-----------|
| Fusion 360 appearance JSON 추출 스크립트 (`extract_appearance.py`) |
| `mesh_colors.csv` 기반 MJCF material 자동 적용 (`apply_csv_to_mjcf.py`) — RGB 0-255 → 0-1 변환, 조명 3개 자동 추가 |
| `Crusher_IsaacSim_colored.xml` 완성 — 색상·조명·Ground.stl·Crank-slider 폐루프 equality constraint·Motor1_crank 액추에이터·L1_Slider_jig 프리즈매틱 조인트 포함 |

---

### 2026-05-15 — MuJoCo 초기 마이그레이션

| 변경 내용 |
|-----------|
| URDF → MJCF 변환 (`convert_urdf_to_mjcf.py`) |
| 기본 뷰어 스크립트 `20260515/open_crusher.py` (MuJoCo UserWarning 억제) |

---

### 2026-05-05 ~ 2026-05-13 — Isaac Sim 디버그 및 관절 설정

| 작업 |
|------|
| 구동부 관절(Drive, Prismatic, Mimic) 설정 및 진단 스크립트 |
| Rack-and-Pinion 기어 연결 디버그 |
| CoACD 충돌 메시 최적화 |
| 질량/관성 텐서 검사 및 수정 |

---

## 의존성 설치

```bash
conda activate isaac_sim
pip install -r requirements.txt
```

> ⚠️ 아래 패키지는 pip으로 설치되지 않습니다.
> - **Isaac Sim** (`omni.*`, `carb`, `pxr`) — NVIDIA Isaac Sim 설치 시 번들 제공
> - **Fusion 360 API** (`adsk`) — Autodesk Fusion 360 내부 실행 전용
> - **Blender Python** (`bpy`) — Blender 내부 실행 전용

---

## 주의 사항

- **STL 바이너리** (`tablets_stl/stl/*.stl`) 는 `.gitignore`에 의해 Git에서 제외됩니다.  
  STL 파일은 Fusion 360 AddIn(`tablet_generator.py`)으로 로컬에서 생성하세요.
- `tablet_generator.py`는 `adsk` 모듈 의존성으로 인해 **Fusion 360 외부에서 실행 불가**합니다.  
  파일 내 주석의 설치 경로 안내를 따르세요.
- MuJoCo STL 단위는 **mm → m 자동 변환** (scale `0.001`)이 적용됩니다.  
  Fusion 360 기본 STL 내보내기 단위가 mm이기 때문입니다.
