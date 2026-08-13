"""
primitive_tablet_generator.py — 정제(알약)를 IPC+FEM.Elastic 파이프라인에서
쓸 캡슐/양볼록(biconvex) 형상을 **TetGen을 전혀 거치지 않고** 순수 수학적으로
(해석적으로) 사면체화(tetrahedralize)한다.

배경
----
M0609_RG2/grasp_bag_tablet_ipc_test.py 실험(2026-07-14, docs/DigitalTwin.md
§8)에서 FEM.Elastic + M0609(IPC) 조합의 "낙하 얼어붙음" 버그를 조사하다가,
`gs.morphs.Box`/`Sphere` 조차도 내부적으로는 TetGen(`genesis.utils.element.
box_to_elements`/`sphere_to_elements` → `mesh.tetrahedralize_mesh`)을 거친다는
걸 확인했다 — "primitive"라고 부르지만 결국 전부 TetGen 산출 mesh다. 박스
적층 + boolean union(TetGen 경유)으로 정점 수를 줄여봐도 얼어붙음이 재현됐고,
정점/tet 개수와 단순 비례 관계도 아니었다(레이어를 줄였더니 더 심해짐) —
즉 "TetGen 산출물이라는 것 자체"가 변수일 가능성이 있다.

이 스크립트의 접근 — **TetGen 완전 배제**
--------------------------------------
캡슐(원기둥 몸통 + 반구 뚜껑 2개)은 **볼록(convex) 도형**이다. 볼록 도형은
내부의 아무 점(예: 중심점)에서 모든 표면 삼각형을 향해 부채꼴로 이으면
(fan tetrahedralization) **항상 유효한(퇴화 없는) 사면체 분해**가 수학적으로
보장된다 — TetGen 같은 품질 개선/Steiner point 삽입이 전혀 필요 없다:

    각 표면 삼각형 (a, b, c) → 사면체 (center, a, b, c)

표면 자체도 원기둥/구 파라메트릭 방정식으로 직접 생성한다(trimesh.creation
호출조차 안 씀 — 순수 numpy). 이 두 단계 다 TetGen을 거치지 않으므로, 결과
verts/elems 를 `genesis.utils.element.mesh_to_elements` 를 몽키패치해 **직접
주입**한다(Genesis 공개 API 에는 "raw tet mesh 를 그대로 써라"라는 경로가
없어서, 이 방법이 TetGen을 완전히 우회하는 유일한 방법이다).

사용법
------
    from primitive_tablet_generator import make_capsule_tets, add_analytic_fem_entity

    verts, elems = make_capsule_tets(radius_mm=4.0, cyl_height_mm=1.0,
                                      n_theta=10, n_cap_rings=3)

    tablet = add_analytic_fem_entity(
        scene, key="tablet1",
        verts_mm=verts, elems=elems,
        material=gs.materials.FEM.Elastic(E=..., nu=..., rho=..., model="stable_neohookean"),
        scale=1e-3, pos=(0.2, 0.0, 0.5),
    )

직접 실행하면(`python primitive_tablet_generator.py`) 캡슐을 만들어 STL로
내보내고(시각 확인용) 정점/사면체 수를 출력한다.
"""
import os

import numpy as np


# ── 1) 캡슐 표면을 순수 파라메트릭 방정식으로 생성 (TetGen 없음) ─────────────
def _capsule_surface(radius, cyl_height, n_theta=10, n_cap_rings=3):
    """캡슐(원기둥 + 반구 뚜껑 2개) 표면 (verts, faces) 를 직접 계산.

    n_theta   : 둘레 분할 수(적을수록 각진 근사, 많을수록 매끈)
    n_cap_rings: 반구 하나당 위도 링 개수(극점 제외)
    축은 Z. 원기둥 몸통은 z ∈ [-cyl_height/2, +cyl_height/2], 반구는 그 바깥쪽.
    """
    thetas = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    verts = []
    rings = []  # 각 원소 = 해당 높이의 정점 인덱스 리스트(둘레) 또는 단일 극점 인덱스

    def add_ring(z, r):
        idx0 = len(verts)
        for th in thetas:
            verts.append([r * np.cos(th), r * np.sin(th), z])
        rings.append(list(range(idx0, idx0 + n_theta)))

    def add_pole(z):
        verts.append([0.0, 0.0, z])
        rings.append([len(verts) - 1])

    half_h = cyl_height / 2.0
    # 상단 반구: 극점 -> 위도 링(半径 커짐) -> 적도(=원기둥 상단)
    add_pole(half_h + radius)
    for i in range(1, n_cap_rings + 1):
        phi = (np.pi / 2) * (1 - i / (n_cap_rings + 1))  # 극(pi/2)->적도(0) 방향
        r = radius * np.cos(phi)
        z = half_h + radius * np.sin(phi)
        add_ring(z, r)
    add_ring(half_h, radius)          # 원기둥 상단 적도
    add_ring(-half_h, radius)         # 원기둥 하단 적도
    for i in range(1, n_cap_rings + 1):
        phi = (np.pi / 2) * (i / (n_cap_rings + 1))
        r = radius * np.cos(phi)
        z = -half_h - radius * np.sin(phi)
        add_ring(z, r)
    add_pole(-half_h - radius)

    verts = np.array(verts, dtype=np.float64)

    faces = []
    for k in range(len(rings) - 1):
        a, b = rings[k], rings[k + 1]
        if len(a) == 1 and len(b) > 1:            # 극 -> 링 (부채꼴)
            p = a[0]
            for j in range(len(b)):
                faces.append([p, b[j], b[(j + 1) % len(b)]])
        elif len(a) > 1 and len(b) == 1:           # 링 -> 극
            p = b[0]
            for j in range(len(a)):
                faces.append([a[j], p, a[(j + 1) % len(a)]])
        else:                                       # 링 -> 링 (사각형 2분할)
            for j in range(len(a)):
                j2 = (j + 1) % len(a)
                faces.append([a[j], b[j], b[j2]])
                faces.append([a[j], b[j2], a[j2]])
    faces = np.array(faces, dtype=np.int64)
    return verts, faces


# ── 2) 볼록 도형 → centroid fan 사면체화 (TetGen 없음, 수학적으로 항상 유효) ──
def _fan_tetrahedralize(verts, faces):
    """각 표면 삼각형 + centroid → 사면체. Genesis FEM solver 는 tet(a,b,c,d)
    에 대해 det([a-d, b-d, c-d]) < 0 을 요구(`fem_solver.py` build() 의
    `tet_wrong_order` 체크) — 부호가 안 맞으면 a,b 를 swap 해 방향을 뒤집는다.
    """
    centroid = verts.mean(axis=0)
    c_idx = len(verts)
    verts2 = np.vstack([verts, centroid[None, :]])
    elems = []
    for a, b, c in faces:
        d = centroid
        det = np.linalg.det(np.column_stack([verts[a] - d, verts[b] - d, verts[c] - d]))
        if det >= 0.0:
            a, b = b, a
        elems.append([a, b, c, c_idx])
    elems = np.array(elems, dtype=np.int64)
    return verts2, elems


def make_capsule_tets(radius_mm=4.0, cyl_height_mm=1.0, n_theta=10, n_cap_rings=3):
    """캡슐 형태의 (verts[mm], elems) 를 100% 해석적으로(TetGen 없이) 생성."""
    surf_v, surf_f = _capsule_surface(radius_mm, cyl_height_mm, n_theta, n_cap_rings)
    verts, elems = _fan_tetrahedralize(surf_v, surf_f)
    return verts, elems


# ── 2b) sliver 없는 캡슐 사면체화 — "의료 축(medial axis)" 다중 앵커 ─────────
#
# v1(위 make_capsule_tets)의 문제: 모든 삼각형을 하나의 전역 centroid 로만
# 부채꼴로 이었다. 캡슐은 볼록이라 이래도 사면체 자체는 항상 유효(퇴화 없음)
# 하지만, 극(pole) 근처의 작은 삼각형들까지 전부 "먼" centroid(반지름보다
# 훨씬 큰 거리, 예: 반지름 2.5mm인데 극-centroid 거리는 6mm)로 이으면 밑변은
# 작고 높이(지렛대)는 큰 얇은 sliver tet 가 대량 발생한다(실측: 해상도를
# 올릴수록 더 심해짐 — 좌굴 재현됨).
#
# 캡슐의 진짜 수학적 정의는 "중심 선분(medial axis, A→B)까지의 거리 ≤ 반지름
# 인 점들의 집합" 이다(§DigitalTwin.md §6-2 SDF 설명과 동일한 정의). 이
# 정의를 그대로 사면체화에 이용한다 — 전역 centroid 대신, **표면의 각
# 부분을 그 지점에서 가장 가까운 축 위의 점(반구는 자기 중심점, 원기둥은
# 그 높이의 축점)에 부채꼴로 잇는다.** 그러면 apex-표면 거리가 어디서나
# 정확히 반지름 R 로 균일해져 sliver 가 원천적으로 안 생긴다:
#   - 북/남 반구(뚜껑) 표면 전체는 정확히 그 반구의 중심점에서 반지름 R
#     떨어져 있다(구의 정의 그 자체) → 반구 중심 하나로 부채꼴.
#   - 원기둥 몸통은 여러 층(band)으로 나누고, 각 층은 자신의 높이의 축점
#     (0,0,z) 을 앵커로 쓰는 "쐐기(wedge)" 사면체화(표준 삼각기둥→3-사면체
#     분해)를 적용 — 인접 band 끼리 같은 앵커를 공유해 내부 경계면이 정확히
#     상쇄되어(watertight) 유지된다.
def _fan_band_local(verts_list, ring_a, ring_b, apex_idx, tets_out):
    """ring_a/ring_b: 정점 인덱스 리스트(길이 1 이면 극). apex_idx 하나로 부채꼴."""
    def add_tet(a, b, c, d):
        pa, pb, pc, pd = (np.array(verts_list[i]) for i in (a, b, c, d))
        det = np.linalg.det(np.column_stack([pa - pd, pb - pd, pc - pd]))
        if det >= 0.0:
            a, b = b, a
        tets_out.append([a, b, c, d])

    if len(ring_a) == 1 and len(ring_b) > 1:
        p = ring_a[0]
        for j in range(len(ring_b)):
            add_tet(p, ring_b[j], ring_b[(j + 1) % len(ring_b)], apex_idx)
    elif len(ring_a) > 1 and len(ring_b) == 1:
        p = ring_b[0]
        for j in range(len(ring_a)):
            add_tet(ring_a[j], p, ring_a[(j + 1) % len(ring_a)], apex_idx)
    else:
        for j in range(len(ring_a)):
            j2 = (j + 1) % len(ring_a)
            add_tet(ring_a[j], ring_b[j], ring_b[j2], apex_idx)
            add_tet(ring_a[j], ring_b[j2], ring_a[j2], apex_idx)


def _wedge_band(verts_list, ring_a, ring_b, anchor_a, anchor_b, tets_out):
    """두 링 사이를 "축 위의 서로 다른 두 앵커"(anchor_a at ring_a 높이,
    anchor_b at ring_b 높이) 로 쐐기(wedge) 분해. 표준 삼각기둥→3-사면체
    분해(P0P1P2P3 / P1P2P3P4 / P2P3P4P5, 여기서 0-3,1-4,2-5 가 세로 변)를
    각 원주 방향 셀에 적용 — n_theta 개 셀 × 3 사면체.
    """
    n = len(ring_a)

    def add_tet(a, b, c, d):
        pa, pb, pc, pd = (np.array(verts_list[i]) for i in (a, b, c, d))
        det = np.linalg.det(np.column_stack([pa - pd, pb - pd, pc - pd]))
        if det >= 0.0:
            a, b = b, a
        tets_out.append([a, b, c, d])

    for j in range(n):
        j2 = (j + 1) % n
        p0, p1, p2 = anchor_a, ring_a[j], ring_a[j2]
        p3, p4, p5 = anchor_b, ring_b[j], ring_b[j2]
        add_tet(p0, p1, p2, p3)
        add_tet(p1, p2, p3, p4)
        add_tet(p2, p3, p4, p5)


def make_capsule_tets_v2(radius_mm=4.0, cyl_height_mm=7.0, n_theta=12, n_cap_rings=4, n_cyl_bands=2):
    """sliver 없는 캡슐 사면체화 — medial-axis 다중 앵커 방식(위 설명 참고).
    v1(make_capsule_tets)과 인터페이스 동일, 내부 구현만 다르다.
    """
    R, H = radius_mm, cyl_height_mm
    half_h = H / 2.0
    thetas = np.linspace(0, 2 * np.pi, n_theta, endpoint=False)
    verts = []
    tets = []

    def add_ring(z, r):
        idx0 = len(verts)
        for th in thetas:
            verts.append([r * np.cos(th), r * np.sin(th), z])
        return list(range(idx0, idx0 + n_theta))

    def add_pt(p):
        verts.append(list(p))
        return len(verts) - 1

    # ── 북반구: pole -> cap 링들 -> 적도, 전부 북반구 중심(0,0,+half_h) 로 부채꼴
    north_c = add_pt([0.0, 0.0, half_h])
    pole_n = add_pt([0.0, 0.0, half_h + R])
    prev = [pole_n]
    for i in range(1, n_cap_rings + 1):
        phi = (np.pi / 2) * (1 - i / (n_cap_rings + 1))
        ring = add_ring(half_h + R * np.sin(phi), R * np.cos(phi))
        _fan_band_local(verts, prev, ring, north_c, tets)
        prev = ring
    ring_topEq = add_ring(half_h, R)
    _fan_band_local(verts, prev, ring_topEq, north_c, tets)

    # ── 남반구: 적도 -> cap 링들 -> pole, 전부 남반구 중심(0,0,-half_h) 로 부채꼴
    south_c = add_pt([0.0, 0.0, -half_h])
    ring_botEq = add_ring(-half_h, R)
    prev = ring_botEq
    for i in range(1, n_cap_rings + 1):
        phi = (np.pi / 2) * (i / (n_cap_rings + 1))
        ring = add_ring(-half_h - R * np.sin(phi), R * np.cos(phi))
        _fan_band_local(verts, prev, ring, south_c, tets)
        prev = ring
    pole_s = add_pt([0.0, 0.0, -half_h - R])
    _fan_band_local(verts, prev, [pole_s], south_c, tets)

    # ── 원기둥 몸통: 적도(top)~적도(bottom) 사이를 n_cyl_bands 로 세분해
    # 각 band 를 "그 높이의 축점" 앵커로 쐐기(wedge) 분해 — band 높이가
    # 반지름과 비슷한 규모가 되도록 나눠 sliver 를 방지한다. 양 끝 앵커는
    # 반구 중심(north_c/south_c)과 정확히 일치해 경계면이 상쇄된다.
    cyl_rings = [ring_topEq]
    cyl_anchors = [north_c]
    for i in range(1, n_cyl_bands):
        z = half_h - i * (H / n_cyl_bands)
        cyl_rings.append(add_ring(z, R))
        cyl_anchors.append(add_pt([0.0, 0.0, z]))
    cyl_rings.append(ring_botEq)
    cyl_anchors.append(south_c)

    for i in range(len(cyl_rings) - 1):
        _wedge_band(verts, cyl_rings[i], cyl_rings[i + 1], cyl_anchors[i], cyl_anchors[i + 1], tets)

    return np.array(verts, dtype=np.float64), np.array(tets, dtype=np.int64)


def _box_surface(size_mm):
    """직육면체 표면(8 verts, 12 faces)을 순수 좌표 계산으로 직접 생성
    (trimesh.creation.box 등 라이브러리 호출 없음).
    """
    sx, sy, sz = (s / 2.0 for s in size_mm)
    verts = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
    ], dtype=np.float64)
    faces = np.array([
        [0, 1, 2], [0, 2, 3],  # bottom
        [4, 6, 5], [4, 7, 6],  # top
        [0, 5, 1], [0, 4, 5],  # front (y=-sy)
        [1, 6, 2], [1, 5, 6],  # right (x=+sx)
        [2, 7, 3], [2, 6, 7],  # back (y=+sy)
        [3, 4, 0], [3, 7, 4],  # left (x=-sx)
    ], dtype=np.int64)
    return verts, faces


def make_box_tets(size_mm=(4.8, 4.8, 4.8)):
    """`gs.morphs.Box`(TetGen 경유)와 대조하기 위한 대조군: 동일한 상자를
    우리 방식(표면 직접 계산 + centroid fan tetrahedralization)으로
    100% 해석적으로(TetGen 없이) 생성한다. 캡슐이 얼려붙었을 때, 이 상자도
    얼어붙는지 비교하면 "우리 수식 정의 방식 자체의 문제"인지 "캡슐이라는
    형상(다수 삼각형이 하나의 centroid 정점을 공유하는 fan 토폴로지) 고유의
    문제"인지를 변인통제로 가를 수 있다.
    """
    surf_v, surf_f = _box_surface(size_mm)
    verts, elems = _fan_tetrahedralize(surf_v, surf_f)
    return verts, elems


# ── 3) Genesis 주입: mesh_to_elements 몽키패치로 TetGen 경로 완전 우회 ───────
#
# **Genesis 1.3.1 대응 재작성 (2026-08-03)** — docs/DigitalTwin.md 조합11 추가 발견.
# 1.1.0 → 1.3.1 업그레이드가 이 우회 경로를 두 군데 깨뜨렸다:
#
#   1. `FEMEntity.sample()` 이 `gs.Mesh.from_morph_surface(morph, surface)` 로
#      **morph 파일에서 표면 메시를 직접 읽어** 시각 geom 을 만들도록 바뀌었다.
#      예전엔 파일을 안 읽어서 0바이트 더미로 통과했지만, 이제 빈 파일이면
#      `merge_submeshes` 가 `ValueError: need at least one array to concatenate`
#      로 죽는다. → **실제 표면 메시를 STL 로 써야 한다.**
#
#   2. `mesh_to_elements` 시그니처가 `(file, pos, scale, tet_cfg)` →
#      **`(mesh, tet_cfg)`** 로 바뀌었다. 파일 경로가 아니라 (이미 scale 이
#      적용되고 아직 translate 는 안 된) trimesh 객체를 받는다. 예전 패치는
#      경로 문자열을 캐시 키로 썼기 때문에 캐시가 절대 안 맞고, 폴백 호출도
#      없어진 인자를 넘겨 `TypeError` 로 죽었다. → **캐시 키를 경로가 아니라
#      "직전에 등록한 엔티티"라는 일회성 큐로 바꾼다** (`scene.add_entity` 가
#      동기적으로 `sample()` → `mesh_to_elements` 를 부르므로 안전하다).
#
# 추가로 1.3.1 은 정점 순서에 계약을 건다 — `FEMVisGeom.sim_verts_idx` 가
# "표면 정점이 앞에, 입력 순서 그대로 유지된다"고 가정한다(sample() docstring).
# 그런데 `make_capsule_tets_v2` 는 medial-axis 앵커(반구 중심·원기둥 축점) 같은
# **내부 정점을 표면 정점 사이에 섞어서** 만든다. 그래서 반환 직전에
# `_align_to_surface()` 로 재배열한다 — 표면 정점을 Genesis 가 읽은 순서 그대로
# 앞에 놓고, 내부 앵커를 뒤에 붙인 뒤 elems 인덱스를 remap.
#
# 지원 버전: Genesis 1.3.x (`mesh_to_elements(mesh, tet_cfg)`). 시그니처가 또
# 바뀌면 _ensure_patched() 가 명시적으로 에러를 낸다.
_ANALYTIC_PENDING = []
_patched = False


def _boundary_faces(elems):
    """tet 집합의 경계면(한 번만 나타나는 삼각면)을 추출한다."""
    import collections

    FACES = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    cnt = collections.Counter()
    rep = {}
    for t in elems:
        for f in FACES:
            tri = (int(t[f[0]]), int(t[f[1]]), int(t[f[2]]))
            key = tuple(sorted(tri))
            cnt[key] += 1
            rep.setdefault(key, tri)
    return np.array([rep[k] for k, c in cnt.items() if c == 1], dtype=np.int64)


def _align_to_surface(mesh, verts, elems):
    """우리 tet 메시를 Genesis 가 읽은 표면 정점 순서에 맞춰 재배열한다.

    Genesis 1.3.1 은 표면 정점이 앞에 그 순서대로 온다고 가정한다. 우리 정점
    배열은 내부 앵커가 섞여 있고, STL 왕복(float32 + trimesh 의 정점 병합)에서
    순서도 달라진다 — 그래서 **위치로 매칭**해 다시 세운다.

    반환: (verts_aligned, elems_remapped) — 앞 len(mesh.vertices) 개가 표면.
    """
    mv = np.asarray(mesh.vertices, dtype=np.float64)
    ov = np.asarray(verts, dtype=np.float64)

    # 형상 크기에 비례한 허용오차(STL 은 float32 라 상대오차가 들어간다).
    span = float(np.linalg.norm(mv.max(axis=0) - mv.min(axis=0)))
    tol = max(span * 1e-5, 1e-12)

    d = np.linalg.norm(ov[:, None, :] - mv[None, :, :], axis=2)
    nearest = d.argmin(axis=1)
    dmin = d[np.arange(len(ov)), nearest]
    is_surf = dmin < tol

    if not np.array_equal(np.sort(nearest[is_surf]), np.arange(len(mv))):
        raise RuntimeError(
            f"analytic tet mesh 의 표면 정점과 Genesis 가 읽은 표면 정점이 1:1 로 "
            f"대응하지 않는다 (ours_surf={int(is_surf.sum())}, genesis={len(mv)}, tol={tol:.3e}). "
            f"STL 왕복에서 정점이 병합/분리됐을 수 있다."
        )

    # 새 인덱스: 표면은 Genesis 순서, 내부는 그 뒤에 등장 순으로.
    remap = np.empty(len(ov), dtype=np.int64)
    remap[is_surf] = nearest[is_surf]
    interior = np.flatnonzero(~is_surf)
    remap[interior] = len(mv) + np.arange(len(interior))

    verts_aligned = np.vstack([mv, ov[interior]]) if len(interior) else mv.copy()
    return verts_aligned, remap[np.asarray(elems, dtype=np.int64)]


def _ensure_patched():
    global _patched
    if _patched:
        return
    import inspect

    import genesis.utils.element as eu

    params = list(inspect.signature(eu.mesh_to_elements).parameters)
    if params[:1] != ["mesh"]:
        raise RuntimeError(
            f"genesis.utils.element.mesh_to_elements 시그니처가 예상과 다르다: {params}. "
            f"이 몽키패치는 Genesis 1.3.x 의 (mesh, tet_cfg) 를 전제로 한다 — "
            f"docs/DigitalTwin.md 조합11 참고."
        )

    _orig = eu.mesh_to_elements

    def _patched_mesh_to_elements(mesh, tet_cfg=None, **kw):
        if _ANALYTIC_PENDING:
            verts, elems = _ANALYTIC_PENDING.pop()
            return _align_to_surface(mesh, verts, elems)
        return _orig(mesh, tet_cfg=tet_cfg or {}, **kw)

    eu.mesh_to_elements = _patched_mesh_to_elements
    _patched = True


def add_analytic_fem_entity(scene, key, verts_mm, elems, material, scale=1e-3, pos=(0, 0, 0),
                            surface=None):
    """TetGen을 거치지 않고 직접 계산한 (verts_mm, elems) 로 FEM.Elastic 엔티티를 추가한다.

    key 는 표면 메시를 쓸 STL 경로. Genesis 1.3.1 부터는 이 파일을 **실제로
    읽어** 시각 geom 과 표면 정점을 만들므로(예전의 0바이트 더미는 안 됨),
    tet 메시의 경계면을 뽑아 여기에 써 준다. 시뮬레이션용 (verts, elems) 는
    여전히 몽키패치가 가로채 우리 값을 그대로 주입하므로 TetGen 은 안 탄다.
    """
    import genesis as gs
    import trimesh as tm

    _ensure_patched()
    verts_mm = np.asarray(verts_mm, dtype=np.float64)
    elems = np.asarray(elems, dtype=np.int64)

    # ── 표면 STL 작성 (mm 단위 — morph 의 scale 이 적용된다) ──────────────────
    faces = _boundary_faces(elems)
    surf = tm.Trimesh(vertices=verts_mm, faces=faces, process=False)
    surf.remove_unreferenced_vertices()   # 내부 앵커 제거 — 표면 정점만 남긴다
    surf.fix_normals()                    # STL 은 바깥 법선을 기대한다
    os.makedirs(os.path.dirname(key) or ".", exist_ok=True)
    surf.export(key)

    # ── 시뮬용 tet 메시를 일회성 큐에 올린다 ─────────────────────────────────
    # scale 은 Genesis 가 morph 파일에 이미 적용해 넘겨주므로 여기서도 맞춰 둔다.
    # pos 는 Genesis 가 사면체화 **이후에** 더하므로(sample(): is_mesh_morph 분기)
    # 여기서는 더하지 않는다 — 예전 패치가 pos 를 더했던 것과 달라진 지점.
    _ANALYTIC_PENDING.append((verts_mm * scale, elems))
    try:
        kwargs = dict(material=material, morph=gs.morphs.Mesh(file=key, scale=scale, pos=pos))
        if surface is not None:
            kwargs["surface"] = surface
        entity = scene.add_entity(**kwargs)
    finally:
        # add_entity 가 중간에 실패하면 큐가 남아 다음 엔티티를 오염시킨다.
        if _ANALYTIC_PENDING:
            _ANALYTIC_PENDING.clear()

    # ── 사후 가드: "조용한 실패" 방지 ────────────────────────────────────────
    # 몽키패치는 라이브러리 계약을 우리가 지켜야 하는데, 어긋나도 **안 죽고 잘못
    # 동작할 수** 있다. 실제로 1.1.0 시절 패치가 반환값 3개 중 세 번째를 None 으로
    # 지어냈고, 크래시 없이 정제만 깨져 렌더돼 3개월 가까이 못 잡았다(§7-15 오진).
    # 시뮬 tet 과 시각 메시가 둘 다 기대치와 맞는지 여기서 즉시 확인한다.
    # 정점 수는 비교 대상이 아니다 — Genesis 는 시각 메시를 면마다 정점 분리해
    # 만들어서(실측: vverts = 3 x vfaces) STL 정점 수와 다른 게 정상이다. 면 수로 본다.
    n_surf_f = len(surf.faces)
    if entity.n_vfaces != n_surf_f or entity.n_vverts == 0:
        raise RuntimeError(
            f"시각 메시가 표면 STL 과 다르다 (vfaces {entity.n_vfaces} vs {n_surf_f}, "
            f"vverts {entity.n_vverts}) — Genesis 가 morph 파일을 제대로 못 읽었거나 "
            f"시각 geom 경로가 바뀐 것이다."
        )
    if entity.n_vertices != len(verts_mm) or entity.n_elements != len(elems):
        raise RuntimeError(
            f"시뮬 tet 메시가 주입값과 다르다 (verts {entity.n_vertices} vs {len(verts_mm)}, "
            f"elems {entity.n_elements} vs {len(elems)}) — 몽키패치가 안 먹었거나 "
            f"Genesis 가 사면체를 추가로 손댔다(TetGen 경유 의심)."
        )
    return entity


def main():
    verts, elems = make_capsule_tets(radius_mm=4.0, cyl_height_mm=1.0, n_theta=10, n_cap_rings=3)
    print(f"[capsule] verts={len(verts)} tets={len(elems)}  (TetGen 미사용, 100% 해석적)")

    out_dir = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(out_dir, "tablet_capsule_analytic.stl")
    try:
        import trimesh
        surf_v, surf_f = _capsule_surface(4.0, 1.0, 10, 3)
        trimesh.Trimesh(vertices=surf_v, faces=surf_f, process=False).export(out_path)
        print(f"[saved] {out_path} (시각 확인용 표면 STL)")
    except ImportError:
        pass


if __name__ == "__main__":
    main()
