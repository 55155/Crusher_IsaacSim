"""Fusion 360 script — 조인트(Joint) 연결 관계를 트리 다이어그램 SVG로 저장.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 파일이 있는 폴더를 추가 > 'JointTreeSVG' 선택 > Run

동작:
  - root 컴포넌트 직속 occurrence 들을 노드로, Joint / As-built Joint 를
    간선으로 삼아 그래프를 만든다.
  - 이름이 'base_link' 인 컴포넌트를 찾아 그 지점부터 BFS 로 트리를 구성해
    base_link 가 항상 트리의 시작점에 오도록 한다.
  - base_link 와 연결되지 않은 부품들은 별도의 작은 트리(또는 독립 노드)로
    같은 그림에 나란히 그려진다.
  - 폐루프(4절 링크 등)로 인해 이미 방문한 노드에 다시 연결되는 조인트는
    대시선(점선)으로 표시한다.
  - 트리는 왼쪽(부모) → 오른쪽(자식)으로 자라고 형제 노드는 위아래로 쌓인다.
  - 인터랙티브 창 없이 SVG 파일 하나로 저장하며, 저장 후 기본 뷰어로 연다.

방향(URDF 부모/자식) 표기:
  Fusion 조인트는 occurrenceTwo=component2 가 URDF 의 parent, occurrenceOne=
  component1 이 child 로 내보내진다(fusion2urdf 의 Joint.make_joints_dict 기준).
  이 스크립트는 그 방향을 간선 위 화살표로 그린다.
  - base_link 에서 뻗어나가는 방향과 조인트 방향이 같으면  →  (정상)
  - 반대로 작성돼 있으면                                    ←  (빨간색, 뒤집힘)
  방향이 뒤집힌 조인트는 그 자식 쪽 링크를 '부모 2개', 부모 쪽 링크를
  '부모 0개' 로 만들어 URDF 내보내기를 깨뜨린다. 그런 링크는 노드 테두리를
  빨간색으로 칠해 함께 표시한다.

가동(active) 조인트 표기:
  - 실선 = 자유도가 있는 가동 조인트 (Revolute / Slider / Ball ...)
  - 점선 = Rigid(고정) 조인트, 또는 suppress 된 조인트
  - 파란 점선 = As-built Joint. fusion2urdf 는 root.joints 만 읽으므로
    이 간선은 URDF 로 전혀 내보내지지 않는다.
  간선 옆 라벨에 조인트 타입을 적고, 같은 부품 쌍에 조인트가 여러 개면 'xN'
  을 덧붙인다.
"""

import adsk.core, adsk.fusion, traceback
import os, re, sys, subprocess, collections

NODE_W, NODE_H = 150, 40
SIB_GAP = 16      # 같은 부모의 자식(형제) 사이 세로 간격
LEVEL_GAP = 100   # 부모 → 자식 사이 가로(깊이) 간격 (간선 라벨 자리 포함)
FOREST_GAP = 40   # 서로 다른 트리(포레스트) 사이 세로 간격
PADDING = 50
LEGEND_ROW_H = 20
LEGEND_MAX_W = 1000

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
MUTED = '#898781'
BASELINE = '#c3c2b7'
ACCENT = '#2a78d6'
CRITICAL = '#d03b3b'

# adsk.fusion.JointTypes 순서. fusion2urdf 의 joint_type_list 와 같은 순서다.
JOINT_TYPE_NAMES = ['Rigid', 'Revolute', 'Slider', 'Cylindrical',
                    'PinSlot', 'Planar', 'Ball']


def sanitize(name):
    return re.sub(r'[ :()]', '_', name)


def node_name(occ):
    return 'base_link' if occ.component.name == 'base_link' else sanitize(occ.name)


def xml_escape(s):
    return (s.replace('&', '&amp;').replace('<', '&lt;')
             .replace('>', '&gt;').replace('"', '&quot;'))


def truncate(s, max_len=22):
    return s if len(s) <= max_len else s[:max_len - 1] + '…'


def text_width_estimate(s, size=11):
    """CJK 문자는 라틴 문자보다 넓게 잡아 범례 폭을 대략 추정한다."""
    unit = size / 11.0
    return sum((13 if ord(ch) > 0x2000 else 7) * unit for ch in s)


def pair_key(a, b):
    return tuple(sorted((a, b)))


# ─────────────────────────────────────────────────────────────
#  조인트 연결 그래프 구성
# ─────────────────────────────────────────────────────────────

def build_adjacency(root_comp):
    """
    occurrence 이름을 노드로, 조인트를 간선으로 갖는 그래프를 만든다.

    Returns
    ----------
    adj    : {node: [이웃, ...]}                  BFS 용 무방향 인접 리스트
    names  : {node, ...}                          root 직속 occurrence 전체
    edges  : {(a, b): [rec, ...]}                 부품 쌍별 조인트 기록
             rec = {name, type, active, as_built, parent, child}
             parent/child 는 URDF 기준(component2=parent, component1=child)
    """
    adj = collections.defaultdict(list)
    names = set()
    edges = collections.defaultdict(list)

    for occ in root_comp.occurrences:
        names.add(node_name(occ))

    joint_lists = [(root_comp.joints, False)]
    if hasattr(root_comp, 'asBuiltJoints'):
        joint_lists.append((root_comp.asBuiltJoints, True))

    for joints, as_built in joint_lists:
        for joint in joints:
            try:
                occ1, occ2 = joint.occurrenceOne, joint.occurrenceTwo
                if occ1 is None or occ2 is None:
                    continue
                child, parent = node_name(occ1), node_name(occ2)
                names.add(child)
                names.add(parent)
                adj[child].append(parent)
                adj[parent].append(child)

                try:
                    type_idx = joint.jointMotion.jointType
                    type_name = JOINT_TYPE_NAMES[type_idx]
                except Exception:
                    type_idx, type_name = -1, '?'
                suppressed = bool(getattr(joint, 'isSuppressed', False))

                edges[pair_key(child, parent)].append({
                    'name': getattr(joint, 'name', '?'),
                    'type': type_name,
                    'active': (type_idx > 0) and not suppressed,
                    'as_built': as_built,
                    'parent': parent,
                    'child': child,
                })
            except Exception:
                continue

    return adj, names, edges


def count_urdf_parents(edges):
    """
    URDF 내보내기가 실제로 보는 부모 수 = root.joints 에서 그 링크가 child 로
    등장한 횟수. As-built Joint 는 fusion2urdf 가 읽지 않으므로 세지 않는다.
    """
    parent_count = collections.Counter()
    for recs in edges.values():
        for rec in recs:
            if not rec['as_built']:
                parent_count[rec['child']] += 1
    return parent_count


def bfs_tree(adj, start):
    """
    start 로부터 BFS 하여 {parent: [child, ...]} 트리를 만들고,
    이미 방문한 노드로 다시 이어지는 '폐루프' 간선 목록을 함께 반환한다.
    """
    visited = {start}
    children_map = collections.defaultdict(list)
    extra_edges = []
    seen_pairs = set()
    queue = collections.deque([start])

    while queue:
        cur = queue.popleft()
        for nb in adj.get(cur, []):
            pair = pair_key(cur, nb)
            if nb not in visited:
                visited.add(nb)
                children_map[cur].append(nb)
                seen_pairs.add(pair)
                queue.append(nb)
            elif pair not in seen_pairs:
                seen_pairs.add(pair)
                extra_edges.append(pair)

    return visited, children_map, extra_edges


def build_node(name, children_map):
    return {
        'id': name,
        'children': [build_node(c, children_map) for c in children_map.get(name, [])],
    }


def build_forest(root_comp):
    adj, names, edges = build_adjacency(root_comp)
    base_name = 'base_link' if 'base_link' in names else None

    visited_all = set()
    roots = []
    all_extra_edges = []

    if base_name:
        visited, children_map, extra = bfs_tree(adj, base_name)
        roots.append(build_node(base_name, children_map))
        visited_all |= visited
        all_extra_edges += extra

    # base_link 와 연결되지 않은 나머지 부품들 (분리된 서브어셈블리 / 미연결 부품)
    remaining = sorted(names - visited_all)
    while remaining:
        start = remaining[0]
        visited, children_map, extra = bfs_tree(adj, start)
        roots.append(build_node(start, children_map))
        visited_all |= visited
        all_extra_edges += extra
        remaining = [n for n in remaining if n not in visited]

    return roots, all_extra_edges, base_name, adj, edges


# ─────────────────────────────────────────────────────────────
#  트리 레이아웃 (naive tidy-tree: 하위 트리 높이 합산 방식, 왼쪽→오른쪽 성장)
# ─────────────────────────────────────────────────────────────

def layout_extent(node):
    """하위 트리가 세로(형제 방향)로 차지하는 높이를 재귀적으로 합산한다."""
    kids = node['children']
    if not kids:
        node['_rely'] = 0.0
        node['_h'] = float(NODE_H)
        return node['_h']
    heights = [layout_extent(k) for k in kids]
    total = sum(heights) + SIB_GAP * (len(kids) - 1)
    cursor = -total / 2.0
    for k, h in zip(kids, heights):
        k['_rely'] = cursor + h / 2.0
        cursor += h + SIB_GAP
    node['_h'] = max(total, float(NODE_H))
    return node['_h']


def assign_abs(node, cy, depth):
    node['_cx'] = depth * (NODE_W + LEVEL_GAP) + NODE_W / 2.0
    node['_cy'] = cy
    for k in node['children']:
        assign_abs(k, cy + k['_rely'], depth + 1)


def flatten(node, nodes, edges, parent=None):
    nodes.append(node)
    if parent is not None:
        edges.append((parent, node))
    for k in node['children']:
        flatten(k, nodes, edges, node)


def layout_forest(roots):
    heights = [layout_extent(r) for r in roots]
    total = sum(heights) + FOREST_GAP * (len(roots) - 1)
    cursor = -total / 2.0
    for r, h in zip(roots, heights):
        cy = cursor + h / 2.0
        assign_abs(r, cy, 0)
        cursor += h + FOREST_GAP

    nodes, tree_edges = [], []
    for r in roots:
        flatten(r, nodes, tree_edges)
    return nodes, tree_edges


# ─────────────────────────────────────────────────────────────
#  SVG 생성
# ─────────────────────────────────────────────────────────────

def r2(v):
    return round(v, 2)


def node_style(name, base_name, adj, parent_count):
    """(테두리색, 굵기, 점선여부) — 문제 있는 링크를 빨간색으로."""
    if name == base_name:
        return ACCENT, 3, False
    if not adj.get(name):
        return MUTED, 1.5, True          # 조인트가 하나도 없는 독립 부품
    if parent_count.get(name, 0) != 1:
        return CRITICAL, 2.5, False      # 부모 0개(=KeyError) 또는 2개 이상(=링크 중복)
    return BASELINE, 1.5, False


def edge_summary(recs, bfs_parent, bfs_child):
    """
    간선에 걸린 조인트들을 한 줄 정보로 요약한다.

    reversed 는 'BFS 트리에서의 부모' 와 'URDF 조인트가 말하는 부모' 가
    어긋났다는 뜻 — 이 경우 bfs_child 쪽이 부모 2개, bfs_parent 쪽이 부모 0개가
    된다. 같은 쌍에 조인트가 여러 개면 하나라도 정방향이면 정상으로 본다.
    """
    regular = [r for r in recs if not r['as_built']]
    use = regular if regular else recs
    forward = any(r['parent'] == bfs_parent for r in use)
    label = '/'.join(sorted({r['type'] for r in use}))
    if len(use) > 1:
        label += ' x%d' % len(use)
    return {
        'reversed': not forward,
        'active': any(r['active'] for r in use),
        'as_built_only': not regular,
        'label': label,
    }


def arrow_head(x, y, direction, color):
    """끝단이 수평 구간이므로 삼각형을 직접 그린다. direction: +1 오른쪽, -1 왼쪽."""
    w, h = 7.0, 4.5
    return ('<path d="M {tip} {y} L {back} {up} L {back} {dn} Z" fill="{c}"/>'.format(
        tip=r2(x), y=r2(y), back=r2(x - direction * w),
        up=r2(y - h), dn=r2(y + h), c=color))


def layout_legend(items, max_width):
    """범례를 max_width 안에서 줄바꿈해 [[(item, x), ...], ...] 로 배치한다."""
    def item_w(it):
        return 20 + text_width_estimate(it[3]) + 24

    rows, row, x = [], [], 0.0
    for it in items:
        w = item_w(it)
        if row and x + w > max_width:
            rows.append(row)
            row, x = [], 0.0
        row.append((it, x))
        x += w
    if row:
        rows.append(row)
    row_widths = [sum(item_w(it) for it, _ in r) for r in rows]
    return rows, (max(row_widths) if row_widths else 0.0)


def build_svg(roots, extra_edges, base_name, adj, edges):
    nodes, tree_edges = layout_forest(roots)
    by_id = {n['id']: n for n in nodes}
    parent_count = count_urdf_parents(edges)

    legend_items = [
        ('node', ACCENT, False, 'base_link (루트)'),
        ('node', CRITICAL, False, '부모 0개 또는 2개 이상 — URDF 불가'),
        ('node', MUTED, True, '조인트 없는 독립 부품'),
        ('edge', BASELINE, False, '가동 조인트 (active)'),
        ('edge', BASELINE, True, '고정 Rigid / suppressed'),
        ('edge', CRITICAL, False, '방향 뒤집힘 (parent/child 반대)'),
        ('edge', ACCENT, True, 'As-built — 익스포터가 무시'),
        ('edge', CRITICAL, True, '폐루프(순환) 연결'),
    ]
    legend_rows, legend_width = layout_legend(legend_items, LEGEND_MAX_W)

    broken = sorted(n for n in by_id
                    if n != base_name and adj.get(n) and parent_count.get(n, 0) != 1)
    stats = [
        '링크 %d개 (조인트 없는 독립 부품 %d개 포함)  ·  조인트 %d개  ·  가동 %d개'
        % (len(by_id),
           sum(1 for n in by_id if not adj.get(n)),
           sum(len(v) for v in edges.values()),
           sum(1 for v in edges.values() for r in v if r['active'])),
    ]
    if broken:
        stats.append('부모 수가 1이 아닌 링크: ' + ', '.join(broken))
    else:
        stats.append('모든 링크가 부모 정확히 1개 — URDF 트리 조건 충족')
    header_h = LEGEND_ROW_H * len(legend_rows) + 18 * len(stats) + 24

    xs = [n['_cx'] for n in nodes]
    ys = [n['_cy'] for n in nodes]
    min_x = min(xs) - NODE_W / 2 - PADDING
    max_x = max(xs) + NODE_W / 2 + PADDING
    max_x = max(max_x, min_x + PADDING + legend_width)
    min_y = min(ys) - NODE_H / 2 - PADDING - header_h
    max_y = max(ys) + NODE_H / 2 + PADDING
    width = max_x - min_x
    height = max_y - min_y

    parts = []
    parts.append(
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'viewBox="{minx} {miny} {w} {h}" width="{w}" height="{h}" '
        'font-family="Segoe UI, system-ui, sans-serif">'.format(
            minx=r2(min_x), miny=r2(min_y), w=r2(width), h=r2(height)))
    parts.append('<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="{4}"/>'
                 .format(r2(min_x), r2(min_y), r2(width), r2(height), SURFACE))

    # 트리 간선 (왼쪽 부모 → 오른쪽 자식 꺾은선 + 방향 화살표 + 조인트 라벨)
    for parent, child in tree_edges:
        recs = edges.get(pair_key(parent['id'], child['id']), [])
        info = edge_summary(recs, parent['id'], child['id']) if recs else None

        x1 = parent['_cx'] + NODE_W / 2
        x2 = child['_cx'] - NODE_W / 2
        mid_x = (x1 + x2) / 2

        if info is None:
            color, dashed, label = BASELINE, True, ''
        elif info['as_built_only']:
            color, dashed, label = ACCENT, True, info['label'] + ' AB'
        elif info['reversed']:
            color, dashed, label = CRITICAL, not info['active'], info['label']
        else:
            color, dashed, label = BASELINE, not info['active'], info['label']

        dash_attr = ' stroke-dasharray="5 4"' if dashed else ''
        d = 'M {x1} {py} H {mid} V {cy} H {x2}'.format(
            x1=r2(x1), py=r2(parent['_cy']), mid=r2(mid_x),
            cy=r2(child['_cy']), x2=r2(x2))
        parts.append('<path d="{0}" fill="none" stroke="{1}" stroke-width="{2}"{3}/>'
                     .format(d, color, 2 if color == CRITICAL else 1.5, dash_attr))

        # 방향 화살표: 정상이면 자식 쪽(오른쪽), 뒤집혔으면 부모 쪽(왼쪽)
        if info is not None and info['reversed']:
            parts.append(arrow_head(x1, parent['_cy'], -1, color))
        else:
            parts.append(arrow_head(x2, child['_cy'], +1, color))

        if label:
            parts.append(
                '<text x="{x}" y="{y}" text-anchor="end" font-size="9" fill="{c}">{t}</text>'
                .format(x=r2(x2 - 10), y=r2(child['_cy'] - 5),
                        c=color if color != BASELINE else MUTED, t=xml_escape(label)))

    # 폐루프 간선 (BFS 트리에 포함되지 못한 추가 연결)
    for a, b in extra_edges:
        na, nb = by_id.get(a), by_id.get(b)
        if not na or not nb:
            continue
        parts.append(
            '<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="{4}" '
            'stroke-width="2" stroke-dasharray="5 4"/>'.format(
                r2(na['_cx']), r2(na['_cy']), r2(nb['_cx']), r2(nb['_cy']), CRITICAL))
        recs = edges.get(pair_key(a, b), [])
        if recs:
            parts.append(
                '<text x="{x}" y="{y}" text-anchor="middle" font-size="9" fill="{c}">{t}</text>'
                .format(x=r2((na['_cx'] + nb['_cx']) / 2),
                        y=r2((na['_cy'] + nb['_cy']) / 2 - 5),
                        c=CRITICAL,
                        t=xml_escape('/'.join(sorted({r['type'] for r in recs})) + ' loop')))

    # 노드
    for n in nodes:
        stroke, sw, dashed = node_style(n['id'], base_name, adj, parent_count)
        x, y = n['_cx'] - NODE_W / 2, n['_cy'] - NODE_H / 2
        dash_attr = ' stroke-dasharray="4 3"' if dashed else ''
        parts.append(
            '<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" ry="8" '
            'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash}/>'.format(
                x=r2(x), y=r2(y), w=NODE_W, h=NODE_H, fill=SURFACE,
                stroke=stroke, sw=sw, dash=dash_attr))
        parts.append(
            '<text x="{x}" y="{y}" text-anchor="middle" dominant-baseline="middle" '
            'font-size="12" fill="{ink}">{label}</text>'.format(
                x=r2(n['_cx']), y=r2(n['_cy']), ink=INK,
                label=xml_escape(truncate(n['id']))))
        # 부모 수가 1이 아닌 링크는 그 개수를 배지로 덧붙인다
        pc = parent_count.get(n['id'], 0)
        if n['id'] != base_name and adj.get(n['id']) and pc != 1:
            parts.append(
                '<text x="{x}" y="{y}" text-anchor="middle" font-size="9" fill="{c}">'
                '부모 {pc}개</text>'.format(
                    x=r2(n['_cx']), y=r2(n['_cy'] + NODE_H / 2 + 10), c=CRITICAL, pc=pc))

    # 범례
    legend_y = min_y + 22
    for row in legend_rows:
        for (kind, color, dashed, label), offset in row:
            lx = min_x + PADDING + offset
            dash_attr = ' stroke-dasharray="4 3"' if dashed else ''
            if kind == 'node':
                parts.append('<rect x="{0}" y="{1}" width="14" height="14" rx="3" ry="3" '
                             'fill="{2}" stroke="{3}" stroke-width="1.5"{4}/>'.format(
                                 r2(lx), r2(legend_y - 11), SURFACE, color, dash_attr))
            else:
                parts.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" '
                             'stroke="{3}" stroke-width="1.8"{4}/>'.format(
                                 r2(lx), r2(legend_y - 4), r2(lx + 11), color, dash_attr))
                parts.append(arrow_head(lx + 14, legend_y - 4, +1, color))
            parts.append('<text x="{0}" y="{1}" font-size="11" fill="{2}">{3}</text>'.format(
                r2(lx + 20), r2(legend_y), INK_SECONDARY, xml_escape(label)))
        legend_y += LEGEND_ROW_H

    # 요약 통계
    legend_y += 6
    for line in stats:
        parts.append('<text x="{0}" y="{1}" font-size="11" fill="{2}">{3}</text>'.format(
            r2(min_x + PADDING), r2(legend_y), INK, xml_escape(line)))
        legend_y += 18

    parts.append('</svg>')
    return '\n'.join(parts)


def open_in_viewer(path):
    """저장한 SVG 를 OS 기본 뷰어로 연다 (실패해도 무시)."""
    try:
        if sys.platform == 'darwin':
            subprocess.Popen(['open', path])
        elif sys.platform.startswith('win'):
            os.startfile(path)   # noqa: B606  (Windows 전용)
        else:
            subprocess.Popen(['xdg-open', path])
    except Exception:
        pass


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
            ui.messageBox('Design 워크스페이스에서 실행해주세요.')
            return

        root_comp = design.rootComponent
        roots, extra_edges, base_name, adj, edges = build_forest(root_comp)

        if not roots:
            ui.messageBox('디자인에 부품(occurrence)이 없습니다.')
            return

        if not base_name:
            ui.messageBox(
                "'base_link' 이름의 컴포넌트를 찾지 못했습니다.\n"
                "루트 컴포넌트를 base_link 로 지정한 뒤 다시 실행해주세요.\n\n"
                "(base_link 없이도 연결된 부품들은 트리로 표시되지만,\n"
                "요청하신 대로 base_link 를 트리 꼭대기에 둘 수는 없습니다.)")

        svg = build_svg(roots, extra_edges, base_name, adj, edges)

        folder_dlg = ui.createFolderDialog()
        folder_dlg.title = '조인트 트리 SVG 저장 위치 선택'
        if folder_dlg.showDialog() != adsk.core.DialogResults.DialogOK:
            return

        name_parts = root_comp.name.split() if root_comp.name else []
        base = sanitize(name_parts[0]) if name_parts else 'assembly'
        file_name = base + '_joint_tree.svg'
        file_path = os.path.join(folder_dlg.folder, file_name)
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(svg)

        open_in_viewer(file_path)

        parent_count = count_urdf_parents(edges)
        broken = sorted(n for n in adj
                        if n != base_name and adj.get(n) and parent_count.get(n, 0) != 1)
        detail = ''
        if broken:
            detail = '\n\n부모 수가 1이 아닌 링크 (URDF 내보내기 실패 원인):\n' + \
                     '\n'.join('  %s — 부모 %d개' % (n, parent_count.get(n, 0)) for n in broken)
        ui.messageBox('조인트 트리 SVG 저장 완료:\n{0}{1}'.format(file_path, detail))

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
