# -*- coding: utf-8 -*-
"""Fusion 360 script — 디자인 안의 구성요소를 하나씩 개별 STL 파일로 내보낸다.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 utills/export_components_stl 폴더(폴더명과 파일명이 같아야 Fusion이
  Script로 인식함)를 선택 > 목록에서 'export_components_stl' 선택 > Run.

어느 단계를 '하나'로 볼지 세 가지 중에 고른다.

  [T] 최상위 부품별 (기본)
      루트 바로 아래 부품 하나가 STL 하나. 그 안의 하위 컴포넌트는 전부 합쳐서
      한 파일에 담긴다 — 예를 들어 하위 50개짜리 RG2 는 RG2.stl 하나로 나온다.
      좌표는 어셈블리에 놓인 위치 그대로다.

  [L] 말단 부품별
      자기 바디를 가진 컴포넌트마다 하나씩. 같은 부품을 여러 번 배치했어도
      파일은 하나고, 좌표는 그 부품 자신의 원점 기준이라 프린트용으로 좋다.
      필요한 개수는 매니페스트 CSV 의 '인스턴스' 열에 있다.

  [S] 선택한 것만
      실행 전에 브라우저나 화면에서 골라둔 부품만 각각 하나씩. 고른 부품의
      하위 컴포넌트는 [T] 와 마찬가지로 합쳐서 담긴다.

치수 정확도 (2026-08-07 추가):
  - 품질을 '곡면 허용 편차(mm)'로 직접 지정한다(기본 0.01mm). STL 은 곡면을
    평평한 삼각형으로 근사하므로 삼각형 면이 곡면 안쪽으로 들어가는 만큼
    치수가 작게 나오는데, 이 편차가 그 최대치다. 프리셋(H/M/L)은 부품 크기에
    따라 편차가 달라지지만 이 값은 절대값이라 작은 부품에서 특히 유리하다.
    평면만 있는 부품은 편차와 무관하게 정확하다.
  - 내보낸 뒤 저장된 파일을 '다시 읽어' 삼각형 좌표에서 크기를 재고, Fusion 이
    보고하는 솔리드 치수와 비교한다. 그 비율을 매니페스트와 결과창에 적어주므로
    단위 문제(10배·25.4배 …)인지 근사 오차(1% 이내)인지 추측할 필요가 없다.
  - '좌표 배율' 을 주면 저장된 STL 의 정점 좌표를 직접 다시 써서 단위를 맞춘다
    (Fusion 의 STL 내보내기에는 단위 옵션이 없다).

내보낸 뒤 같은 폴더에 매니페스트 CSV(파일명 / 이름 / 인스턴스 수 / 합쳐진
구성요소 수 / 솔리드 치수 / STL 실측 치수 / 비율)를 같이 써준다.

주의:
  - Fusion 의 STL 내보내기는 밀리미터 단위 좌표로 나간다. STL 포맷 자체에는
    단위 정보가 없어서, 받는 프로그램이 cm/inch 로 가정하면 크기가 어긋난다.
  - 바디가 하나도 없는 구성요소(스케치·조인트만 있는 것)는 건너뛴다.
  - 메시(STL) 바디만 있는 컴포넌트도 그대로 내보내진다.
  - 루트에 컴포넌트 없이 직접 놓인 바디는 바디 하나당 파일 하나로 내보낸다
    (루트 컴포넌트를 통째로 넘기면 디자인 전체가 한 파일로 나가버리므로).
"""

import csv
import os
import re
import struct
import traceback

import adsk.core
import adsk.fusion

TARGET_TOP = "T"
TARGET_LEAF = "L"
TARGET_SELECTION = "S"

TARGET_LABELS = {
    TARGET_TOP: "최상위 부품별",
    TARGET_LEAF: "말단 부품별",
    TARGET_SELECTION: "선택한 것만",
}

REFINEMENT_LABELS = {
    "H": ("High", "MeshRefinementHigh"),
    "M": ("Medium", "MeshRefinementMedium"),
    "L": ("Low", "MeshRefinementLow"),
}

CSV_COLUMNS = ["file", "name", "instances", "parts",
               "solid_dx_mm", "solid_dy_mm", "solid_dz_mm",
               "stl_dx", "stl_dy", "stl_dz", "ratio"]

INVALID_FILENAME_RE = re.compile(r'[\\/:*?"<>|]+')


# ─────────────────────────────────────────────────────────────
#  파일 이름
# ─────────────────────────────────────────────────────────────

def sanitize(name):
    """파일명으로 쓸 수 없는 문자를 정리한다(한글은 그대로 둔다)."""
    cleaned = INVALID_FILENAME_RE.sub("_", name or "")
    cleaned = cleaned.replace(" ", "_").strip("._")
    return cleaned or "unnamed"


def unique_filename(folder, base, used):
    """같은 이름이 겹치면 _2, _3 을 붙여 유일하게 만든다."""
    name = base
    index = 2
    while name.lower() in used or os.path.exists(os.path.join(folder, name + ".stl")):
        name = "{0}_{1}".format(base, index)
        index += 1
    used.add(name.lower())
    return os.path.join(folder, name + ".stl")


# ─────────────────────────────────────────────────────────────
#  구성요소 훑기
# ─────────────────────────────────────────────────────────────

def body_lists(entity):
    """entity 가 '직접' 가진 BRep + 메시 바디 목록."""
    lists = []
    for attr in ("bRepBodies", "meshBodies"):
        try:
            lists.append(getattr(entity, attr))
        except Exception:
            continue
    return lists


def own_bodies(entity):
    for bodies in body_lists(entity):
        for body in bodies:
            yield body


def child_occurrences(entity):
    """Occurrence 면 childOccurrences, Component 면 occurrences."""
    for attr in ("childOccurrences", "occurrences"):
        try:
            children = getattr(entity, attr)
        except Exception:
            continue
        if children is not None:
            return children
    return []


def iter_bodies_deep(entity):
    """entity 와 그 하위 컴포넌트 전부의 바디."""
    for body in own_bodies(entity):
        yield body
    for child in child_occurrences(entity):
        for body in iter_bodies_deep(child):
            yield body


def has_bodies_deep(entity):
    """하위까지 뒤져서 바디가 하나라도 있는지(첫 개를 찾으면 바로 멈춘다)."""
    for _ in iter_bodies_deep(entity):
        return True
    return False


def count_parts(entity):
    """한 파일에 합쳐지는 구성요소 개수(자기 자신 포함)."""
    total = 1
    for child in child_occurrences(entity):
        total += count_parts(child)
    return total


def size_mm(bodies):
    """바디들의 bounding box 크기를 (dx, dy, dz) mm 로. 못 재면 None."""
    total = None
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
    if total is None:
        return None
    return tuple((getattr(total.maxPoint, axis) - getattr(total.minPoint, axis)) * 10.0
                 for axis in ("x", "y", "z"))


def make_target(entity, name, instances, parts, bodies):
    return {"entity": entity, "name": name, "instances": instances,
            "parts": parts, "size": size_mm(bodies)}


def loose_root_body_targets(root):
    """루트에 컴포넌트 없이 직접 놓인 바디 — 바디 하나당 파일 하나.

    루트 컴포넌트를 그대로 내보내면 디자인 전체가 한 덩어리로 나가므로
    이것만 따로 바디 단위로 처리한다.
    """
    targets = []
    for body in own_bodies(root):
        try:
            name = body.name
        except Exception:
            name = "body"
        targets.append(make_target(body, name, 1, 1, [body]))
    return targets


def collect_top_level(design):
    """[T] 루트 바로 아래 부품 하나 = 파일 하나(하위는 전부 합쳐짐)."""
    root = design.rootComponent
    targets = []
    for occ in root.occurrences:
        if has_bodies_deep(occ):
            targets.append(make_target(occ, occ.name, 1, count_parts(occ),
                                       iter_bodies_deep(occ)))
    targets.extend(loose_root_body_targets(root))
    return targets


def collect_leaf_components(design):
    """[L] 자기 바디를 가진 고유 컴포넌트마다 하나씩(같은 부품은 한 번만)."""
    root = design.rootComponent

    counts = {}
    for occ in root.allOccurrences:
        try:
            comp = occ.component
        except Exception:
            continue
        entry = counts.setdefault(comp.name, [comp, 0])
        entry[1] += 1

    targets = []
    for comp, instances in counts.values():
        bodies = list(own_bodies(comp))
        if not bodies:
            continue
        # 자기 바디를 가지면서 하위 컴포넌트도 있으면, 내보낸 STL 에 하위까지
        # 딸려 들어간다(=다른 파일과 내용이 겹친다). parts 열로 알 수 있게 한다.
        targets.append(make_target(comp, comp.name, instances, count_parts(comp),
                                   iter_bodies_deep(comp)))
    targets.extend(loose_root_body_targets(root))
    return targets


def collect_selection(ui):
    """[S] 지금 선택돼 있는 부품들 각각 하나씩."""
    targets = []
    for index in range(ui.activeSelections.count):
        entity = ui.activeSelections.item(index).entity
        try:
            name = entity.name
        except Exception:
            continue
        if not has_bodies_deep(entity) and not list(own_bodies(entity)):
            continue
        targets.append(make_target(entity, name, 1, count_parts(entity),
                                   iter_bodies_deep(entity)))
    return targets


# ─────────────────────────────────────────────────────────────
#  내보내기
# ─────────────────────────────────────────────────────────────

def export_one(export_mgr, entity, filepath, quality):
    """구성요소 하나를 STL 로 내보낸다.

    quality 는 ask_quality() 가 만든 (라벨, 적용함수) 쌍의 적용함수 — 프리셋
    정밀도 또는 '곡면 허용 편차 mm' 를 옵션에 반영한다.
    """
    options = export_mgr.createSTLExportOptions(entity, filepath)
    options.sendToPrintUtility = False   # 안 끄면 내보낼 때마다 프린트 유틸이 뜬다
    options.isBinaryFormat = True
    quality(options)
    try:
        # 컴포넌트를 넘기면 기본값이 '바디마다 한 파일' 인 버전이 있어 명시적으로 끈다.
        options.isOneFilePerBody = False
    except Exception:
        pass
    return export_mgr.execute(options)


# ─────────────────────────────────────────────────────────────
#  저장된 STL 파일 검증 / 보정
#
#  Fusion 이 뭘 썼는지 추측하지 않고, 파일을 직접 열어 삼각형 좌표에서 크기를
#  다시 잰다. 솔리드 치수와 이 값을 비교하면 단위 문제(10배, 25.4배 …)인지
#  곡면 근사 오차(1% 이내)인지 숫자로 바로 구분된다.
# ─────────────────────────────────────────────────────────────

BINARY_HEADER = 84          # 헤더 80 바이트 + 삼각형 개수 uint32
BINARY_TRIANGLE = 50        # 법선 3 + 정점 9 = 12 float + 속성 2 바이트


def is_binary_stl(path):
    """파일 크기가 바이너리 STL 규격과 정확히 맞는지로 판정한다."""
    size = os.path.getsize(path)
    if size < BINARY_HEADER:
        return False
    with open(path, "rb") as f:
        f.seek(80)
        count = struct.unpack("<I", f.read(4))[0]
    return size == BINARY_HEADER + count * BINARY_TRIANGLE


def read_stl_vertices(path):
    """STL 파일의 정점 좌표를 (x, y, z) 로 하나씩 내놓는다(바이너리/ASCII 둘 다)."""
    if is_binary_stl(path):
        with open(path, "rb") as f:
            f.seek(80)
            count = struct.unpack("<I", f.read(4))[0]
            for _ in range(count):
                chunk = f.read(BINARY_TRIANGLE)
                if len(chunk) < BINARY_TRIANGLE:
                    return
                values = struct.unpack("<12f", chunk[:48])
                yield values[3:6]
                yield values[6:9]
                yield values[9:12]
        return

    with open(path, "r", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                try:
                    yield (float(parts[1]), float(parts[2]), float(parts[3]))
                except ValueError:
                    continue


def stl_bbox(path):
    """저장된 STL 의 실제 크기 (dx, dy, dz). 정점이 없으면 None."""
    lo = [None, None, None]
    hi = [None, None, None]
    for vertex in read_stl_vertices(path):
        for axis in range(3):
            value = vertex[axis]
            if lo[axis] is None or value < lo[axis]:
                lo[axis] = value
            if hi[axis] is None or value > hi[axis]:
                hi[axis] = value
    if lo[0] is None:
        return None
    return tuple(hi[axis] - lo[axis] for axis in range(3))


def rescale_stl(path, factor):
    """저장된 STL 의 정점 좌표를 factor 배로 다시 쓴다(법선은 방향뿐이라 그대로).

    Fusion 의 STL 내보내기에는 단위 옵션이 없으므로, 받는 쪽이 다른 단위를
    기대할 때 여기서 좌표 자체를 바꿔준다.
    """
    if is_binary_stl(path):
        with open(path, "rb") as f:
            data = bytearray(f.read())
        count = struct.unpack("<I", bytes(data[80:84]))[0]
        for i in range(count):
            base = BINARY_HEADER + i * BINARY_TRIANGLE
            for offset in range(12, 48, 4):          # 법선(0~11) 건너뛰고 정점만
                position = base + offset
                value = struct.unpack("<f", bytes(data[position:position + 4]))[0]
                data[position:position + 4] = struct.pack("<f", value * factor)
        with open(path, "wb") as f:
            f.write(bytes(data))
        return True

    out = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                try:
                    scaled = [float(p) * factor for p in parts[1:]]
                except ValueError:
                    out.append(line)
                    continue
                indent = line[:len(line) - len(line.lstrip())]
                out.append("{0}vertex {1:.6e} {2:.6e} {3:.6e}\n".format(indent, *scaled))
            else:
                out.append(line)
    with open(path, "w") as f:
        f.writelines(out)
    return True


# 단위 착오로 흔히 나오는 비율 — 검증 결과를 사람 말로 설명하기 위한 표
KNOWN_UNIT_RATIOS = [
    (1.0, "mm (정상)"),
    (0.1, "cm 로 저장됨"),
    (0.001, "m 로 저장됨"),
    (1.0 / 25.4, "inch 로 저장됨"),
    (10.0, "mm 값을 cm 로 읽은 셈 (10배)"),
    (25.4, "inch 값을 mm 로 읽은 셈"),
]


def explain_ratio(ratio):
    """실측 비율이 알려진 단위 비율에 가까우면 그 설명을, 아니면 None."""
    if ratio is None:
        return None
    for value, label in KNOWN_UNIT_RATIOS:
        if abs(ratio - value) <= value * 0.002:
            return label
    return None


def hide_progress(progress):
    """진행률 창을 닫는다 — 버전에 따라 hide() 이거나 hideDialog() 다."""
    for name in ("hide", "hideDialog"):
        method = getattr(progress, name, None)
        if method is None:
            continue
        try:
            method()
            return True
        except Exception:
            continue
    return False


def verification_lines(rows):
    """저장된 파일에서 다시 잰 크기와 솔리드 치수를 비교해 요약 문장을 만든다."""
    ratios = [float(r["ratio"]) for r in rows if r["ratio"]]
    if not ratios:
        return ["", "[치수 검증] 저장된 파일을 읽지 못해 비교하지 못했습니다."]

    worst = max(ratios, key=lambda value: abs(value - 1.0))
    worst_row = next(r for r in rows if r["ratio"] and float(r["ratio"]) == worst)
    error_pct = (worst - 1.0) * 100.0

    lines = ["", "[치수 검증 — 저장된 STL 을 다시 읽어 잰 값]"]
    lines.append("  솔리드 대비 최대 오차: {0:+.3f}%  ({1})".format(
        error_pct, worst_row["name"]))
    lines.append("    솔리드 {0} x {1} x {2} mm".format(
        worst_row["solid_dx_mm"], worst_row["solid_dy_mm"], worst_row["solid_dz_mm"]))
    lines.append("    STL    {0} x {1} x {2}".format(
        worst_row["stl_dx"], worst_row["stl_dy"], worst_row["stl_dz"]))

    known = explain_ratio(worst)
    if known and abs(worst - 1.0) > 0.002:
        lines.append("  -> 비율 {0:.4f} = {1}".format(worst, known))
        lines.append("     받는 프로그램의 import 단위를 mm 로 맞추거나, "
                     "좌표 배율로 보정하세요.")
    elif abs(error_pct) <= 0.1:
        lines.append("  -> 실측과 일치합니다(0.1% 이내). 파일 좌표는 mm 단위입니다.")
    else:
        lines.append("  -> 곡면을 삼각형으로 근사하면서 생긴 오차입니다. "
                     "편차를 더 작게(예: 0.001) 주면 줄어듭니다.")
        lines.append("     평면만 있는 부품은 편차와 무관하게 정확합니다.")
    return lines


def write_manifest(rows, filepath):
    with open(filepath, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


# ─────────────────────────────────────────────────────────────
#  입력
# ─────────────────────────────────────────────────────────────

def prompt_text(ui, prompt, title, default):
    """inputBox 는 (입력문자열, 취소여부) 를 돌려준다 — 순서를 헷갈리지 않게 감싼다.

    취소하면 None.
    """
    text, cancelled = ui.inputBox(prompt, title, default)
    return None if cancelled else text


def ask_target(ui):
    text = prompt_text(
        ui,
        "어느 단계를 STL 하나로 볼까요?\n\n"
        "  T = 최상위 부품별 (기본)\n"
        "      루트 바로 아래 부품 하나 = 파일 하나.\n"
        "      하위 컴포넌트는 전부 합쳐서 담김 (예: RG2 하위 50개 → RG2.stl 1개).\n\n"
        "  L = 말단 부품별\n"
        "      자기 바디를 가진 컴포넌트마다 하나씩. 같은 부품은 한 번만,\n"
        "      좌표는 부품 자기 원점 기준 — 부품별 프린트용.\n\n"
        "  S = 선택한 것만\n"
        "      지금 브라우저/화면에서 골라둔 부품만 각각 하나씩.",
        "Export Components STL — 단계", "T")
    if text is None:
        return None
    target = text.strip().upper()[:1]
    if target not in TARGET_LABELS:
        ui.messageBox("T, L, S 중 하나만 입력할 수 있습니다: '{0}'".format(text))
        return None
    return target


def preset_quality(ui, key):
    """H/M/L 프리셋 → (라벨, 옵션 적용 함수)."""
    label, enum_name = REFINEMENT_LABELS[key]
    try:
        refinement = getattr(adsk.fusion.MeshRefinementSettings, enum_name)
    except Exception:
        ui.messageBox("이 Fusion 버전에서 정밀도 '{0}' 을 찾을 수 없습니다.".format(label))
        return None

    def apply(options):
        options.meshRefinement = refinement

    return label, apply


def deviation_quality(ui, deviation_mm):
    """곡면 허용 편차(mm) → (라벨, 옵션 적용 함수).

    STL 은 곡면을 평평한 삼각형으로 근사하므로, 삼각형 면이 실제 곡면보다
    안쪽으로 들어가는 만큼 치수가 작게 나온다. 이 편차를 직접 지정하면
    '실측과 몇 mm 까지 맞출지'를 정하는 셈이다. 프리셋(H/M/L)은 부품 크기에
    따라 편차가 달라지지만 이 값은 절대값이라 작은 부품에서 특히 유리하다.
    """
    try:
        custom = adsk.fusion.MeshRefinementSettings.MeshRefinementCustom
    except Exception:
        ui.messageBox("이 Fusion 버전은 사용자 지정 편차를 지원하지 않습니다.\n"
                      "H / M / L 중 하나를 입력해주세요.")
        return None

    def apply(options):
        options.meshRefinement = custom
        # Fusion 내부 길이 단위는 cm — mm 로 받은 값을 cm 로 바꿔 넣는다.
        options.surfaceDeviation = deviation_mm / 10.0
        for name, value in (("normalDeviation", 5.0), ("maxEdgeLength", 0.0),
                            ("aspectRatio", 0.0)):
            try:
                setattr(options, name, value)
            except Exception:
                continue

    return "편차 {0:g}mm".format(deviation_mm), apply


def ask_quality(ui):
    text = prompt_text(
        ui,
        "메시 품질을 정하세요.\n\n"
        "  숫자 = 곡면 허용 편차(mm). 기본 0.01\n"
        "         작을수록 실제 치수에 가깝고 파일이 커집니다.\n"
        "         평면만 있는 부품은 편차와 무관하게 정확합니다.\n\n"
        "  H / M / L = Fusion 프리셋 (High / Medium / Low)",
        "Export Components STL — 품질", "0.01")
    if text is None:
        return None

    key = text.strip().upper()
    if key[:1] in REFINEMENT_LABELS and not key[:1].isdigit():
        return preset_quality(ui, key[:1])

    try:
        deviation = float(key)
    except ValueError:
        ui.messageBox("숫자(mm) 또는 H / M / L 만 입력할 수 있습니다: '{0}'".format(text))
        return None
    if deviation <= 0.0:
        ui.messageBox("편차는 0보다 커야 합니다: {0}".format(deviation))
        return None
    return deviation_quality(ui, deviation)


def ask_output_scale(ui):
    """저장된 STL 좌표에 곱할 배율 — 받는 쪽이 다른 단위를 기대할 때 쓴다."""
    text = prompt_text(
        ui,
        "저장된 STL 좌표에 곱할 배율을 입력하세요.\n\n"
        "  1      = Fusion 기본 그대로 (mm 단위 좌표, 보통 이걸 씁니다)\n"
        "  0.1    = cm 로 읽는 프로그램용\n"
        "  0.03937 = inch 로 읽는 프로그램용\n\n"
        "내보낸 뒤 파일을 다시 읽어 실제 크기를 검증하고, 솔리드 치수와\n"
        "어긋나면 결과창에 원인을 알려줍니다.",
        "Export Components STL — 좌표 배율", "1")
    if text is None:
        return None
    try:
        scale = float(text.strip())
    except ValueError:
        ui.messageBox("숫자를 입력해야 합니다: '{0}'".format(text))
        return None
    if scale <= 0.0:
        ui.messageBox("배율은 0보다 커야 합니다: {0}".format(scale))
        return None
    return scale


def ask_folder(ui):
    dlg = ui.createFolderDialog()
    dlg.title = "STL 파일들을 저장할 폴더 선택"
    if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.folder


# ─────────────────────────────────────────────────────────────
#  진입점
# ─────────────────────────────────────────────────────────────

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("활성 문서가 Fusion 360 디자인이 아닙니다.\n"
                          "Design 워크스페이스에서 실행해주세요.")
            return

        # 대상 선택 모드는 '실행 전에 골라둔 선택'을 써야 하므로, 다이얼로그를
        # 띄우기 전에 미리 읽어둔다(대화상자를 열면 선택이 풀릴 수 있음).
        preselected = collect_selection(ui)

        target = ask_target(ui)
        if target is None:
            return

        if target == TARGET_SELECTION:
            targets = preselected
            if not targets:
                ui.messageBox(
                    "선택된 부품이 없습니다.\n"
                    "스크립트를 실행하기 전에 브라우저나 화면에서 내보낼 부품을 "
                    "먼저 골라주세요.")
                return
        elif target == TARGET_TOP:
            targets = collect_top_level(design)
        else:
            targets = collect_leaf_components(design)

        if not targets:
            ui.messageBox("내보낼 바디가 있는 구성요소를 찾지 못했습니다.")
            return

        quality = ask_quality(ui)
        if quality is None:
            return
        quality_label, quality_apply = quality

        output_scale = ask_output_scale(ui)
        if output_scale is None:
            return

        folder = ask_folder(ui)
        if folder is None:
            return

        export_mgr = design.exportManager
        progress = ui.createProgressDialog()
        progress.isCancelButtonShown = True
        progress.cancelButtonText = "취소"
        progress.show("STL 내보내기 ({0})".format(TARGET_LABELS[target]),
                      "%v / %m  내보내는 중...", 0, len(targets), 0)

        rows = []
        failed = []
        cancelled = False
        used_names = set()

        # 내보내기 도중 무슨 일이 나든 진행률 창은 반드시 닫고, 그때까지의
        # 결과(매니페스트·요약)는 살려서 보여준다.
        try:
            for index, item in enumerate(targets, start=1):
                if progress.wasCancelled:
                    cancelled = True
                    break
                progress.progressValue = index

                filepath = unique_filename(folder, sanitize(item["name"]), used_names)
                try:
                    if not export_one(export_mgr, item["entity"], filepath,
                                      quality_apply):
                        failed.append((item["name"], "execute() 가 False 를 반환"))
                        continue
                except Exception as exc:
                    failed.append((item["name"], str(exc).strip()[:160]))
                    continue

                if output_scale != 1.0:
                    try:
                        rescale_stl(filepath, output_scale)
                    except Exception as exc:
                        failed.append((item["name"],
                                       "좌표 배율 적용 실패: {0}".format(str(exc)[:120])))

                # 추측하지 않고 파일을 도로 읽어 실제로 저장된 크기를 잰다.
                try:
                    written = stl_bbox(filepath)
                except Exception:
                    written = None

                solid = item["size"]
                ratio = None
                if solid and written and max(solid) > 0.0:
                    ratio = max(written) / max(solid)

                rows.append({
                    "file": os.path.basename(filepath),
                    "name": item["name"],
                    "instances": item["instances"],
                    "parts": item["parts"],
                    "solid_dx_mm": "{0:.4f}".format(solid[0]) if solid else "",
                    "solid_dy_mm": "{0:.4f}".format(solid[1]) if solid else "",
                    "solid_dz_mm": "{0:.4f}".format(solid[2]) if solid else "",
                    "stl_dx": "{0:.4f}".format(written[0]) if written else "",
                    "stl_dy": "{0:.4f}".format(written[1]) if written else "",
                    "stl_dz": "{0:.4f}".format(written[2]) if written else "",
                    "ratio": "{0:.6f}".format(ratio) if ratio else "",
                })
        finally:
            hide_progress(progress)

        manifest_note = ""
        if rows:
            manifest_path = os.path.join(folder, "components_manifest.csv")
            try:
                write_manifest(rows, manifest_path)
                manifest_note = "매니페스트: {0}".format(manifest_path)
            except Exception as exc:
                manifest_note = "매니페스트 저장 실패: {0}".format(str(exc)[:120])

        lines = []
        if cancelled:
            lines.append("[취소됨 — 그때까지 내보낸 파일은 그대로 남아 있습니다]")
        lines.append("STL {0}개 저장 완료 (단계: {1}, 품질: {2}, 좌표 배율 x{3:g})".format(
            len(rows), TARGET_LABELS[target], quality_label, output_scale))
        lines.append(folder)
        if manifest_note:
            lines.append(manifest_note)

        lines.extend(verification_lines(rows))

        merged = [r for r in rows if r["parts"] > 1]
        if merged:
            lines.append("")
            lines.append("[하위 컴포넌트가 합쳐진 파일 {0}개]".format(len(merged)))
            for row in sorted(merged, key=lambda r: r["parts"], reverse=True)[:5]:
                lines.append("  - {0}: 구성요소 {1}개가 한 파일로".format(
                    row["name"], row["parts"]))

        if failed:
            lines.append("")
            lines.append("[실패 {0}개]".format(len(failed)))
            for name, reason in failed[:10]:
                lines.append("  - {0}: {1}".format(name, reason))

        summary = "\n".join(lines)
        ui.messageBox(summary, "Export Components STL 결과")

        # 참고용: Text Commands 팔레트에도 남겨둔다(실패해도 무시).
        try:
            palette = ui.palettes.itemById("TextCommands")
            if palette:
                palette.isVisible = True
                palette.writeText(summary)
        except Exception:
            pass

    except Exception:
        if ui:
            ui.messageBox("오류 발생:\n{0}".format(traceback.format_exc()))
