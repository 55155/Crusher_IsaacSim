# -*- coding: utf-8 -*-
"""Fusion 360 디자인 전체를 배율 k 로 축척하는 핵심 로직 (UI 없음).

같은 로직을 두 군데서 쓴다.
  - utills/scale_design/scale_design.py            : 단발성 Script (inputBox UI)
  - Crusher_Genesis/fusion_addin/scale_design/     : Add-In (툴바 버튼 + 다이얼로그)

Add-In 폴더에는 이 파일의 사본이 들어간다(add-in 은 어디에 설치되든 동작해야
하므로 프로젝트 경로에 의존하면 안 됨). 로직을 고칠 때는 이 파일을 고치고
add-in 폴더로 복사한다.

두 가지 모드:

  [G] 지오메트리 스케일 (기본, 조인트 어셈블리 권장)
      - 디자인 안의 모든 컴포넌트에 대해, 그 컴포넌트가 직접 소유한 BRep
        바디들을 '컴포넌트 자기 원점' 기준으로 k배 Scale feature 로 키운다.
        컴포넌트마다 딱 한 번씩만 적용되므로, 같은 컴포넌트를 여러 번 배치한
        (occurrence 가 여러 개인) 어셈블리에서도 k^2, k^3 로 중복 확대되지
        않는다.
      - 조인트는 컴포넌트 안의 형상(면/모서리/조인트 원점)을 참조하므로
        형상이 커지면 조인트 위치도 같이 따라와 어셈블리가 스스로 재정렬된다.
        따라서 조인트로 묶인 occurrence 의 transform 은 건드리지 않는다.
      - 대신 조인트에 숫자로 박혀 있는 offset 값과, 슬라이더/핀슬롯 계열의
        이동 한계(slide limits)는 형상과 함께 자동으로 커지지 않으므로
        직접 k배 해준다(각도 한계는 배율과 무관하므로 그대로 둠).
      - 조인트에 물려있지도 않고 ground 도 아닌, 즉 transform 으로만 놓여 있는
        부품은 위치 벡터를 k배 해서 어셈블리 전체 배치를 함께 키운다. 이때
        반드시 '컴포넌트가 직접 담고 있는 자식(comp.occurrences)'만, 부모 기준
        상대 위치로 한 번씩 옮긴다 — root.allOccurrences 의 프록시를 옮기면
        부모 이동이 자식에게 중복 적용돼 배치가 무너진다.
      - 스케일 후 실제 bounding box 가 '원래 크기 x k' 에서 1% 이상 벗어나면
        결과 요약 맨 위에 경고를 띄운다(배치가 깨졌거나, 링크 부품이 원본
        크기로 남아 있는 경우를 바로 알아채기 위해).

메시(STL) 바디:
  Scale feature 는 BRep 전용이라 STL 로 들여온 메시 바디에는 쓸 수 없다. 대신
  삼각형 데이터를 읽어 노드 좌표를 k배 한 새 메시 바디를 만들고 원본을 지운다
  (scale_mesh_bodies). 기준이 컴포넌트 원점이라 솔리드와 정확히 같은 방식으로
  스케일된다. scale_meshes=False 로 끄면 개수만 세고 건너뛴다.

내보내기:
  apply_scale(..., step_path=..., stl_path=...) 로 스케일 후 디자인 전체를
  내보낸다. STEP 은 BRep 전용이라 메시 바디가 빠지므로, 메시가 있는 디자인은
  STL 을 함께 쓴다.

  [P] 파라미터 스케일
      - 사용자 파라미터(User Parameters) 중 '길이 단위'이면서 '다른 파라미터를
        참조하지 않는 순수 상수'인 것만 골라 expression 에 * k 를 붙인다.
      - 다른 파라미터를 참조하는 종속 파라미터는 참조 대상이 커지면 자동으로
        따라 커지므로 건드리지 않는다(건드리면 k^2 로 이중 확대됨).
      - 형상이 파라미터로 완전히 구동되는 디자인에서만 의미가 있다. 스케치에
        직접 박아둔 치수는 파라미터가 아니므로 커지지 않는다.

주의:
  - 파라메트릭 디자인이면 이번 실행으로 생긴 Scale feature 들을 타임라인 그룹
    하나로 묶어, 접거나 한 번에 지울 수 있게 한다.
  - 메시 바디(Mesh Body)는 Scale feature 대상이 아니라 건너뛰고, 몇 개를
    건너뛰었는지 결과 요약에 표시한다.
  - 외부 링크(Linked/Referenced) 컴포넌트는 이 디자인에서 편집할 수 없으므로
    건너뛴다. 링크를 끊거나(Break Link) 원본 디자인에서 직접 실행해야 한다.
"""

import re

import adsk.core
import adsk.fusion

MODE_GEOMETRY = "G"
MODE_PARAMETER = "P"

# 길이 차원인지 판정할 때 기준으로 쓰는 내부 단위(Fusion 내부 길이 단위는 cm)
INTERNAL_LENGTH_UNIT = "cm"

# "12.5 mm", "3", "-0.4in" 처럼 숫자 + (선택)단위 만으로 된 순수 상수 표현식.
# 다른 파라미터 이름이나 연산자가 섞이면 매칭되지 않는다.
CONST_EXPR_RE = re.compile(r"^\s*[-+]?(\d+\.?\d*|\.\d+)\s*([a-zA-Z]+)?\s*$")


class ScaleReport:
    """실행 결과 집계 — 요약 메시지를 만들기 위한 단순 카운터 모음."""

    def __init__(self):
        self.scaled_components = []      # (컴포넌트 이름, 바디 수)
        self.skipped_referenced = []     # 외부 링크라 건너뛴 컴포넌트 이름
        self.skipped_empty = 0           # 스케일할 바디가 없는 컴포넌트 수
        self.failed_components = []      # (컴포넌트 이름, 실패 사유)
        self.mesh_bodies = 0             # 발견한 메시(STL) 바디 수
        self.mesh_scaled = 0             # 실제로 k배 한 메시 바디 수
        self.mesh_failed = []            # (메시 이름, 실패 사유)
        self.joint_offsets = 0           # 배율 적용한 조인트 offset 수
        self.joint_limits = 0            # 배율 적용한 이동 한계 수
        self.joint_failed = 0            # 조인트 값 수정 실패 수
        self.moved_occurrences = 0       # 위치를 k배 한 occurrence 수
        self.moved_failed = 0
        self.skipped_jointed = 0         # 조인트가 재정렬해주므로 안 옮긴 수
        self.skipped_grounded = 0        # ground 라 못 옮긴 수
        self.export_path = None          # STEP 내보내기 성공 경로
        self.export_error = None         # STEP 내보내기 실패 사유
        self.stl_path = None             # STL 내보내기 성공 경로
        self.stl_error = None            # STL 내보내기 실패 사유
        self.scale_suspect = False       # 결과 크기가 예상과 어긋남(내보내기 경고용)
        self.params_scaled = []          # (파라미터 이름, 이전 expression)
        self.params_dependent = []       # 종속식이라 건드리지 않은 파라미터
        self.params_non_length = []      # 길이 단위가 아니라 제외한 파라미터

    @property
    def body_count(self):
        return sum(n for _, n in self.scaled_components)


# ─────────────────────────────────────────────────────────────
#  공통 유틸
# ─────────────────────────────────────────────────────────────

def is_length_unit(units_mgr, unit):
    """unit 이 길이 차원인지 — cm 로 환산이 되면 길이로 본다."""
    if not unit:
        return False
    try:
        units_mgr.convert(1.0, unit, INTERNAL_LENGTH_UNIT)
        return True
    except Exception:
        return False


def referenced_component_names(root_comp):
    """외부 링크로 들어온(이 디자인에서 편집 불가) 컴포넌트 이름 집합."""
    names = set()
    for occ in root_comp.allOccurrences:
        try:
            if occ.isReferencedComponent:
                names.add(occ.component.name)
        except Exception:
            continue
    return names


def design_bounding_box(root_comp):
    """루트 좌표계 기준 디자인 전체 bounding box (없으면 None).

    솔리드(BRep)와 메시(STL) 바디를 모두 포함한다 — 메시만 있는 디자인에서도
    '원래 크기 x k' 검증이 동작해야 하므로.
    """
    total = None
    body_lists = [root_comp.bRepBodies]
    try:
        body_lists.append(root_comp.meshBodies)
    except Exception:
        pass
    for occ in root_comp.allOccurrences:
        for attr in ("bRepBodies", "meshBodies"):
            try:
                body_lists.append(getattr(occ, attr))
            except Exception:
                continue

    for bodies in body_lists:
        for body in bodies:
            try:
                box = body.boundingBox
            except Exception:
                continue
            if box is None:
                continue
            if total is None:
                total = adsk.core.BoundingBox3D.create(box.minPoint, box.maxPoint)
            else:
                total.combine(box)
    return total


def format_bbox(box, factor=1.0):
    """bounding box 크기를 mm 문자열로 (내부 단위 cm → mm 이므로 ×10).

    factor 를 주면 그 배율을 적용한 '예상 크기'를 보여준다.
    """
    if box is None:
        return "(측정할 바디 없음)"
    scale = 10.0 * factor
    dx = (box.maxPoint.x - box.minPoint.x) * scale
    dy = (box.maxPoint.y - box.minPoint.y) * scale
    dz = (box.maxPoint.z - box.minPoint.z) * scale
    return "{0:.3f} x {1:.3f} x {2:.3f} mm".format(dx, dy, dz)


# ─────────────────────────────────────────────────────────────
#  [G] 지오메트리 스케일
# ─────────────────────────────────────────────────────────────

# MeshBody.mesh 가 돌려주는 객체는 Fusion 버전에 따라 PolygonMesh 이거나
# TriangleMesh 라서 삼각형 데이터 속성 이름이 다르다(2026-08-06: PolygonMesh 에
# nodeIndices 가 없어 메시 스케일이 전부 실패). 후보를 순서대로 시도한다.
MESH_COORD_ATTRS = ("nodeCoordinatesAsDouble",)
MESH_INDEX_ATTRS = ("triangleNodeIndices", "nodeIndices")
MESH_NORMAL_ATTRS = ("normalVectorsAsDouble",)
MESH_NORMAL_INDEX_ATTRS = ("triangleNormalIndices", "normalIndices")


def _first_attr(obj, names):
    """names 중 먼저 존재하는 속성을 리스트로 돌려준다(없으면 None)."""
    for name in names:
        try:
            value = getattr(obj, name)
        except Exception:
            continue
        if value is not None:
            return list(value)
    return None


def _available_attrs(obj):
    """실패 메시지에 실제 속성 목록을 실어 보내 다음 수정을 쉽게 한다."""
    try:
        return ", ".join(sorted(a for a in dir(obj) if not a.startswith("_")))[:240]
    except Exception:
        return "(속성 목록을 읽을 수 없음)"


def read_mesh_triangles(mesh):
    """메시에서 (좌표, 삼각형 인덱스, 법선, 법선 인덱스) 를 뽑는다.

    읽을 수 없으면 ValueError — 사유에 실제 속성 목록을 함께 담는다.
    """
    coords = _first_attr(mesh, MESH_COORD_ATTRS)
    if not coords:
        points = getattr(mesh, "nodeCoordinates", None)
        if points:
            coords = [c for p in points for c in (p.x, p.y, p.z)]
    if not coords:
        raise ValueError("노드 좌표 없음 (가능한 속성: {0})".format(_available_attrs(mesh)))

    node_idx = _first_attr(mesh, MESH_INDEX_ATTRS)
    if not node_idx:
        # 인덱스 배열이 아예 없는 메시도 있다. 삼각형마다 정점 3개를 따로
        # 들고 있는 형태(노드 수 == 삼각형 수 x 3)라면 인덱스가 순서 그대로다.
        # triangleCount 로 교차 확인될 때만 그렇게 본다 — 정점을 공유하는
        # 메시에 순번을 붙이면 형상이 망가지므로.
        node_count = getattr(mesh, "nodeCount", 0) or len(coords) // 3
        triangle_count = getattr(mesh, "triangleCount", 0)
        if triangle_count and node_count == triangle_count * 3:
            node_idx = list(range(node_count))
        else:
            raise ValueError("삼각형 인덱스 없음 (가능한 속성: {0})".format(
                _available_attrs(mesh)))

    normals = _first_attr(mesh, MESH_NORMAL_ATTRS) or []
    normal_idx = _first_attr(mesh, MESH_NORMAL_INDEX_ATTRS)
    if normal_idx is None:
        # PolygonMesh 는 노드마다 법선 1개라 법선 인덱스가 노드 인덱스와 같다.
        normal_idx = list(node_idx) if len(normals) == len(coords) else []
    if not normals or not normal_idx:
        # 법선을 못 맞추면 아예 넘기지 않는다 — Fusion 이 새로 계산한다.
        normals, normal_idx = [], []

    return coords, node_idx, normals, normal_idx


def is_parametric(design):
    try:
        return design.designType == adsk.fusion.DesignTypes.ParametricDesignType
    except Exception:
        return False


def scale_mesh_bodies(design, comp, factor, report):
    """메시(STL) 바디를 k배 한다.

    Scale feature 는 BRep 전용이라 메시 바디에는 쓸 수 없다. 대신 삼각형 데이터를
    직접 읽어 노드 좌표를 k배 한 새 메시 바디를 만들고 원본을 지운다. 메시 노드
    좌표는 그 컴포넌트의 좌표계(내부 단위 cm) 값이라, BRep 바디를 컴포넌트 원점
    기준으로 스케일하는 것과 정확히 같은 기준이 된다.

    파라메트릭 디자인에서는 메시 바디를 Base Feature 안에서만 만들 수 있으므로
    새 Base Feature 를 열어 담는다. 원본이 다른 Base Feature 소속이면 그쪽을
    편집 모드로 열어 지운다. 원본 삭제가 실패하면 새로 만든 바디를 도로 지워서
    같은 메시가 두 개 남는 일이 없게 한다.
    """
    try:
        mesh_bodies = list(comp.meshBodies)
    except Exception:
        return
    if not mesh_bodies:
        return

    report.mesh_bodies += len(mesh_bodies)
    parametric = is_parametric(design)

    for mesh_body in mesh_bodies:
        try:
            name = mesh_body.name
        except Exception:
            name = "(이름 없음)"

        # MeshBody.mesh 로 못 읽으면 displayMesh(표시용 삼각형 메시)로도 시도한다.
        # 버전에 따라 둘 중 하나만 삼각형 인덱스를 들고 있다.
        data = None
        read_errors = []
        for source in ("mesh", "displayMesh"):
            candidate = getattr(mesh_body, source, None)
            if candidate is None:
                continue
            try:
                data = read_mesh_triangles(candidate)
                break
            except Exception as exc:
                read_errors.append("{0} -> {1}".format(source, str(exc).strip()))

        if data is None:
            report.mesh_failed.append((name, "메시 데이터 읽기 실패: {0}".format(
                " / ".join(read_errors)[:320])))
            continue

        coords, node_idx, normals, normal_idx = data
        coords = [v * factor for v in coords]

        if not hasattr(comp.meshBodies, "addByTriangleMeshData"):
            report.mesh_failed.append((
                name,
                "이 Fusion 버전의 MeshBodies 에 addByTriangleMeshData 가 없습니다. "
                "MESH 탭 > Modify > Scale 로 수동 처리하세요."))
            continue

        base = None
        new_body = None
        try:
            if parametric:
                base = comp.features.baseFeatures.add()
                base.startEdit()
            created = comp.meshBodies.addByTriangleMeshData(
                coords, node_idx, normals, normal_idx, base)
            # 버전에 따라 MeshBody 하나 또는 리스트를 돌려준다.
            new_body = created.item(0) if hasattr(created, "count") else created
        except Exception as exc:
            report.mesh_failed.append((name, "새 메시 생성 실패: {0}".format(
                str(exc).strip()[:120])))
        finally:
            if base is not None:
                try:
                    base.finishEdit()
                except Exception:
                    pass

        if new_body is None:
            continue

        try:
            owner = mesh_body.baseFeature
        except Exception:
            owner = None
        try:
            if owner is not None:
                owner.startEdit()
                mesh_body.deleteMe()
                owner.finishEdit()
            else:
                mesh_body.deleteMe()
        except Exception as exc:
            # 원본을 못 지웠으면 같은 메시가 두 벌 남으므로 새 것을 되돌린다.
            try:
                new_body.deleteMe()
            except Exception:
                pass
            report.mesh_failed.append((name, "원본 메시 삭제 실패: {0}".format(
                str(exc).strip()[:120])))
            continue

        try:
            new_body.name = name
        except Exception:
            pass
        report.mesh_scaled += 1


def scale_component_bodies(design, factor, report, scale_meshes=True):
    """모든 컴포넌트의 자기 소유 BRep 바디를 컴포넌트 원점 기준으로 k배 한다.

    scale_meshes 가 True 면 같은 컴포넌트의 메시(STL) 바디도 함께 처리한다.
    """
    skip_names = referenced_component_names(design.rootComponent)
    value = adsk.core.ValueInput.createByReal(factor)

    for comp in design.allComponents:
        if comp.name in skip_names:
            report.skipped_referenced.append(comp.name)
            continue

        if scale_meshes:
            scale_mesh_bodies(design, comp, factor, report)
        else:
            try:
                report.mesh_bodies += comp.meshBodies.count
            except Exception:
                pass

        bodies = adsk.core.ObjectCollection.create()
        for body in comp.bRepBodies:
            bodies.add(body)

        if bodies.count == 0:
            # 메시만 있는 컴포넌트는 위에서 이미 처리했으므로 '빈 컴포넌트'가 아니다.
            if not scale_meshes or not comp.meshBodies.count:
                report.skipped_empty += 1
            continue

        try:
            scale_features = comp.features.scaleFeatures
            scale_input = scale_features.createInput(
                bodies, comp.originConstructionPoint, value)
            scale_features.add(scale_input)
            report.scaled_components.append((comp.name, bodies.count))
        except Exception as exc:
            report.failed_components.append((comp.name, str(exc).strip()[:200]))


def scale_value_input(value_input, factor):
    """ValueInput(실수 또는 수식 문자열)에 배율을 곱한 새 ValueInput 을 만든다."""
    if value_input is None:
        return None
    try:
        if value_input.valueType == adsk.core.ValueTypes.StringValueType:
            text = value_input.stringValue
            if not text:
                return None
            return adsk.core.ValueInput.createByString(
                "({0}) * {1}".format(text, factor))
        real = value_input.realValue
        if real == 0.0:
            return None
        return adsk.core.ValueInput.createByReal(real * factor)
    except Exception:
        return None


def scale_joint_values(design, factor, report):
    """조인트의 숫자 offset 과 이동(직선) 한계값을 k배 한다.

    회전 한계는 각도라서 배율과 무관하므로 손대지 않는다.
    """
    for comp in design.allComponents:
        joint_lists = [comp.joints]
        if hasattr(comp, "asBuiltJoints"):
            joint_lists.append(comp.asBuiltJoints)

        for joints in joint_lists:
            for joint in joints:
                try:
                    new_offset = scale_value_input(getattr(joint, "offset", None), factor)
                    if new_offset is not None:
                        joint.offset = new_offset
                        report.joint_offsets += 1
                except Exception:
                    report.joint_failed += 1

                try:
                    motion = joint.jointMotion
                except Exception:
                    continue

                limits = getattr(motion, "slideLimits", None)
                if limits is None:
                    continue
                try:
                    if limits.isMinimumValueEnabled:
                        limits.minimumValue *= factor
                        report.joint_limits += 1
                    if limits.isMaximumValueEnabled:
                        limits.maximumValue *= factor
                        report.joint_limits += 1
                    if limits.isRestValueEnabled:
                        limits.restValue *= factor
                except Exception:
                    report.joint_failed += 1


def jointed_child_names(comp):
    """comp 안에서 조인트에 물려 있는 '자식 occurrence 이름' 집합.

    한 컴포넌트 안의 occurrence 이름은 유일하므로(예: "Plate:1") 이름으로
    비교해도 안전하다.
    """
    names = set()
    joint_lists = [comp.joints]
    if hasattr(comp, "asBuiltJoints"):
        joint_lists.append(comp.asBuiltJoints)

    for joints in joint_lists:
        for joint in joints:
            for attr in ("occurrenceOne", "occurrenceTwo"):
                try:
                    occ = getattr(joint, attr)
                except Exception:
                    occ = None
                if occ is None:
                    continue
                try:
                    names.add(occ.name)
                except Exception:
                    continue
    return names


def scale_child_occurrence_positions(design, factor, report):
    """각 컴포넌트가 직접 담고 있는 자식 occurrence 의 '부모 기준 상대 위치'를 k배.

    반드시 comp.occurrences(=그 컴포넌트에 native 한 자식)만 건드린다.
    root.allOccurrences 로 돌면 루트 어셈블리 컨텍스트의 '프록시'가 나오는데,
    프록시의 transform 은 부모까지 합성된 값이라 부모를 옮기면 자식이 딸려
    가고 그 뒤에 자식을 또 옮기면서 배치가 이중으로 어긋난다(2026-08-06 버그:
    0.3배 스케일에 전체 크기가 오히려 커짐). 컴포넌트 단위로 한 번씩만
    적용하면 바디 스케일과 정확히 같은 기준이 되어 어셈블리가 균일하게
    축척된다.

    조인트로 묶인 자식은 형상이 커/작아지면서 조인트가 알아서 재정렬해주므로
    제외한다. ground 된 자식도 옮길 수 없으므로 제외하고 개수만 센다.
    """
    skip_names = referenced_component_names(design.rootComponent)

    for comp in design.allComponents:
        # 외부 링크 컴포넌트의 내부는 편집 불가 — 내부 배치도 건드리지 않는다.
        if comp.name in skip_names:
            continue

        jointed = jointed_child_names(comp)

        for occ in comp.occurrences:
            try:
                if occ.name in jointed:
                    report.skipped_jointed += 1
                    continue
                if occ.isGrounded:
                    report.skipped_grounded += 1
                    continue
            except Exception:
                report.moved_failed += 1
                continue

            try:
                matrix = occ.transform2 if hasattr(occ, "transform2") else occ.transform
                translation = matrix.translation
                if translation.length == 0.0:
                    # 부모 원점에 그대로 놓인 부품 — 옮길 것이 없다.
                    continue
                translation.scaleBy(factor)
                matrix.translation = translation
                if hasattr(occ, "transform2"):
                    occ.transform2 = matrix
                else:
                    occ.transform = matrix
                report.moved_occurrences += 1
            except Exception:
                report.moved_failed += 1

    try:
        if design.snapshots.hasPendingSnapshot:
            design.snapshots.add()
    except Exception:
        pass


def run_geometry_mode(design, factor, report, scale_joints=True, move_free=True,
                      scale_meshes=True):
    timeline = None
    start_index = None
    if is_parametric(design):
        try:
            timeline = design.timeline
            start_index = timeline.count
        except Exception:
            timeline = None

    scale_component_bodies(design, factor, report, scale_meshes=scale_meshes)
    if scale_joints:
        scale_joint_values(design, factor, report)
    if move_free:
        scale_child_occurrence_positions(design, factor, report)

    # 이번 실행으로 생긴 feature 들을 타임라인 그룹 하나로 묶어 정리/삭제가
    # 쉽도록 한다(그룹화 실패는 결과에 영향 없으므로 조용히 넘어감).
    if timeline is not None and start_index is not None:
        try:
            end_index = timeline.count - 1
            if end_index > start_index:
                group = timeline.timelineGroups.add(start_index, end_index)
                group.name = "Scale x{0:g}".format(factor)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────
#  [P] 파라미터 스케일
# ─────────────────────────────────────────────────────────────

def run_parameter_mode(design, factor, report):
    units_mgr = design.unitsManager

    for param in design.userParameters:
        try:
            unit = param.unit
            expression = param.expression
        except Exception:
            continue

        if not is_length_unit(units_mgr, unit):
            report.params_non_length.append(param.name)
            continue

        if not CONST_EXPR_RE.match(expression or ""):
            # 다른 파라미터를 참조하는 종속식 — 참조 대상이 커지면 같이 커진다.
            report.params_dependent.append((param.name, expression))
            continue

        try:
            param.expression = "({0}) * {1:g}".format(expression, factor)
            report.params_scaled.append((param.name, expression))
        except Exception:
            report.params_dependent.append((param.name, expression))


# ─────────────────────────────────────────────────────────────
#  실행 + 결과 요약
# ─────────────────────────────────────────────────────────────

def export_step(design, filepath, report):
    """현재 디자인 전체(루트 컴포넌트)를 STEP 으로 내보낸다.

    STEP 은 BRep 포맷이라 메시(STL) 바디는 담기지 않는다 — 메시가 있는 디자인은
    STL 로도 함께 내보내야 한다.
    """
    try:
        export_mgr = design.exportManager
        options = export_mgr.createSTEPExportOptions(filepath, design.rootComponent)
        if export_mgr.execute(options):
            report.export_path = filepath
        else:
            report.export_error = "exportManager.execute() 가 False 를 반환했습니다."
    except Exception as exc:
        report.export_error = str(exc).strip()[:300]
    return report.export_path is not None


def export_stl(design, filepath, report):
    """디자인 전체를 STL 하나로 내보낸다(솔리드 + 메시 모두 삼각형으로).

    sendToPrintUtility 를 끄지 않으면 내보낸 뒤 3D 프린트 유틸리티가 뜬다.
    """
    try:
        export_mgr = design.exportManager
        options = export_mgr.createSTLExportOptions(design.rootComponent, filepath)
        options.sendToPrintUtility = False
        options.isBinaryFormat = True
        if export_mgr.execute(options):
            report.stl_path = filepath
        else:
            report.stl_error = "exportManager.execute() 가 False 를 반환했습니다."
    except Exception as exc:
        report.stl_error = str(exc).strip()[:300]
    return report.stl_path is not None


def bbox_deviation(before, after, factor):
    """실제 결과 크기가 '원래 크기 x k' 에서 얼마나 벗어났는지(최대 상대오차).

    배치가 깨지면 이 값이 크게 튄다. 판정할 수 없으면 None.
    """
    if before is None or after is None:
        return None
    worst = 0.0
    for axis in ("x", "y", "z"):
        span_before = getattr(before.maxPoint, axis) - getattr(before.minPoint, axis)
        span_after = getattr(after.maxPoint, axis) - getattr(after.minPoint, axis)
        expected = span_before * factor
        if expected <= 1e-9:
            continue
        worst = max(worst, abs(span_after - expected) / expected)
    return worst


def apply_scale(design, factor, mode, scale_joints=True, move_free=True,
                scale_meshes=True, step_path=None, stl_path=None):
    """모드에 맞춰 스케일을 적용하고 (ScaleReport, 요약문자열) 을 돌려준다.

    step_path / stl_path 를 주면 스케일 적용 후 그 경로로 내보낸다.
    """
    report = ScaleReport()
    if mode == MODE_GEOMETRY:
        before = design_bounding_box(design.rootComponent)
        run_geometry_mode(design, factor, report,
                          scale_joints=scale_joints, move_free=move_free,
                          scale_meshes=scale_meshes)
        after = design_bounding_box(design.rootComponent)
        run_exports(design, report, step_path, stl_path)
        return report, geometry_summary(report, factor, before, after)

    run_parameter_mode(design, factor, report)
    run_exports(design, report, step_path, stl_path)
    return report, parameter_summary(report, factor)


def run_exports(design, report, step_path, stl_path):
    if step_path:
        export_step(design, step_path, report)
    if stl_path:
        export_stl(design, stl_path, report)


def export_summary(report):
    lines = []
    if report.scale_suspect and (report.export_path or report.stl_path):
        lines += ["", "!! 스케일이 제대로 적용되지 않은 상태로 내보내졌습니다 — "
                      "위 경고를 해결한 뒤 다시 내보내세요."]
    if report.export_path:
        lines += ["", "STEP 저장 완료:", "  {0}".format(report.export_path)]
        if report.mesh_bodies:
            lines.append("  (STEP 은 BRep 전용이라 메시 바디 {0}개는 빠집니다 — "
                         "메시까지 필요하면 STL 로 내보내세요)".format(report.mesh_bodies))
    elif report.export_error:
        lines += ["", "STEP 저장 실패: {0}".format(report.export_error)]

    if report.stl_path:
        lines += ["", "STL 저장 완료:", "  {0}".format(report.stl_path)]
    elif report.stl_error:
        lines += ["", "STL 저장 실패: {0}".format(report.stl_error)]
    return lines


def geometry_summary(report, factor, before, after):
    lines = []
    lines.append("[지오메트리 스케일 x{0:g} 완료]".format(factor))
    lines.append("")
    lines.append("전체 크기: {0}".format(format_bbox(before)))
    lines.append("      →   {0}".format(format_bbox(after)))
    lines.append("")
    # 배치가 깨지면 결과 크기가 '원래 x k' 에서 벗어난다 — 가장 먼저 알려준다.
    deviation = bbox_deviation(before, after, factor)
    if deviation is not None and deviation > 0.01:
        report.scale_suspect = True
        lines.append("!! 결과 크기가 예상({0})과 {1:.1f}% 어긋났습니다.".format(
            format_bbox(before, factor), deviation * 100.0))
        if report.skipped_referenced:
            lines.append("   외부 링크 컴포넌트가 원본 크기 그대로 남아 있어서일 "
                         "가능성이 큽니다(아래 목록).")
        lines.append("   의도한 결과가 아니면 저장하지 말고 Ctrl+Z 로 되돌리세요.")
        lines.append("")

    lines.append("스케일한 컴포넌트 {0}개 / 바디 {1}개".format(
        len(report.scaled_components), report.body_count))
    lines.append("조인트 offset {0}개, 이동 한계값 {1}개 배율 적용".format(
        report.joint_offsets, report.joint_limits))
    lines.append("부품 배치 {0}개 위치 이동 (조인트가 재정렬할 {1}개, "
                 "ground {2}개는 제외)".format(
                     report.moved_occurrences, report.skipped_jointed,
                     report.skipped_grounded))

    if report.mesh_bodies:
        lines.append("메시(STL) 바디 {0}개 중 {1}개 스케일 완료".format(
            report.mesh_bodies, report.mesh_scaled))
    if report.skipped_empty:
        lines.append("바디가 없어 건너뛴 컴포넌트 {0}개".format(report.skipped_empty))
    if report.mesh_failed:
        lines.append("")
        lines.append("[메시 스케일 실패 {0}개]".format(len(report.mesh_failed)))
        for name, reason in report.mesh_failed[:10]:
            lines.append("  - {0}: {1}".format(name, reason))
    if report.skipped_referenced:
        lines.append("")
        lines.append("[외부 링크라 건너뛴 컴포넌트 {0}개]".format(
            len(report.skipped_referenced)))
        for name in report.skipped_referenced[:10]:
            lines.append("  - {0}".format(name))
        lines.append("  (원본 디자인에서 실행하거나 Break Link 후 다시 시도)")
    if report.joint_failed or report.moved_failed:
        lines.append("")
        lines.append("조인트 값 수정 실패 {0}건, 위치 이동 실패 {1}건".format(
            report.joint_failed, report.moved_failed))
    if report.failed_components:
        lines.append("")
        lines.append("[스케일 실패 컴포넌트 {0}개]".format(len(report.failed_components)))
        for name, reason in report.failed_components[:10]:
            lines.append("  - {0}: {1}".format(name, reason))

    lines.extend(export_summary(report))

    lines.append("")
    lines.append("되돌리려면 타임라인 끝의 'Scale x{0:g}' 그룹을 지우거나 "
                 "Ctrl+Z 를 사용하세요.".format(factor))
    return "\n".join(lines)


def parameter_summary(report, factor):
    lines = []
    lines.append("[파라미터 스케일 x{0:g} 완료]".format(factor))
    lines.append("")
    lines.append("배율 적용한 길이 파라미터 {0}개".format(len(report.params_scaled)))
    for name, old in report.params_scaled[:20]:
        lines.append("  - {0}: {1} → ({1}) * {2:g}".format(name, old, factor))
    if len(report.params_scaled) > 20:
        lines.append("  ... 외 {0}개 더".format(len(report.params_scaled) - 20))

    if report.params_dependent:
        lines.append("")
        lines.append("[다른 파라미터를 참조해서 건드리지 않은 것 {0}개]".format(
            len(report.params_dependent)))
        lines.append("  (참조 대상이 커지면 자동으로 같이 커짐)")
        for name, expr in report.params_dependent[:10]:
            lines.append("  - {0} = {1}".format(name, expr))

    if report.params_non_length:
        lines.append("")
        lines.append("길이 단위가 아니라 제외한 파라미터 {0}개".format(
            len(report.params_non_length)))

    if not report.params_scaled:
        lines.append("")
        lines.append("바꾼 게 없습니다. 이 디자인은 사용자 파라미터로 구동되지 않는 것 "
                     "같으니 [G] 지오메트리 모드를 쓰세요.")

    lines.extend(export_summary(report))
    return "\n".join(lines)
