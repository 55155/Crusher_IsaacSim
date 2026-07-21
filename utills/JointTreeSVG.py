"""Fusion 360 script — 조인트(Joint) 연결 관계를 트리 다이어그램 SVG로 저장.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 파일이 있는 폴더를 추가 > 'JointTreeSVG' 선택 > Run

동작:
  - root 컴포넌트 직속 occurrence 들을 노드로, Joint / As-built Joint 를
    무방향 간선으로 삼아 그래프를 만든다.
  - 이름이 'base_link' 인 컴포넌트를 찾아 그 지점부터 BFS 로 트리를 구성해
    base_link 가 항상 트리의 시작점에 오도록 한다.
  - base_link 와 연결되지 않은 부품들은 별도의 작은 트리(또는 독립 노드)로
    같은 그림에 나란히 그려진다.
  - 폐루프(4절 링크 등)로 인해 이미 방문한 노드에 다시 연결되는 조인트는
    대시선(점선)으로 표시한다.
  - 트리는 왼쪽(부모) → 오른쪽(자식)으로 자라고 형제 노드는 위아래로 쌓인다.
    깊이는 얕고 형제가 많은 어셈블리 트리 특성상, 옆으로 넓어지는 대신
    세로로 길고 좁은(포트레이트) 이미지가 되어 스크롤하며 보기 편하다.
  - 인터랙티브 창 없이 SVG 파일 하나로 저장하며, 저장 후 기본 뷰어로 연다.
"""

import adsk.core, adsk.fusion, traceback
import os, re, collections

NODE_W, NODE_H = 150, 40
SIB_GAP = 16      # 같은 부모의 자식(형제) 사이 세로 간격
LEVEL_GAP = 80    # 부모 → 자식 사이 가로(깊이) 간격
FOREST_GAP = 40   # 서로 다른 트리(포레스트) 사이 세로 간격
PADDING = 50

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
MUTED = '#898781'
BASELINE = '#c3c2b7'
ACCENT = '#2a78d6'
CRITICAL = '#d03b3b'


def sanitize(name):
    return re.sub(r'[ :()]', '_', name)


def node_name(occ):
    return 'base_link' if occ.component.name == 'base_link' else sanitize(occ.name)


def xml_escape(s):
    return (s.replace('&', '&amp;').replace('<', '&lt;')
             .replace('>', '&gt;').replace('"', '&quot;'))


def truncate(s, max_len=22):
    return s if len(s) <= max_len else s[:max_len - 1] + '…'


def text_width_estimate(s):
    """CJK 문자는 라틴 문자보다 넓게 잡아 범례 폭을 대략 추정한다."""
    return sum(13 if ord(ch) > 0x2000 else 7 for ch in s)


# ─────────────────────────────────────────────────────────────
#  조인트 연결 그래프 구성
# ─────────────────────────────────────────────────────────────

def build_adjacency(root_comp):
    """occurrence 이름을 노드로, 조인트를 무방향 간선으로 갖는 인접 리스트."""
    adj = collections.defaultdict(list)
    names = set()

    for occ in root_comp.occurrences:
        names.add(node_name(occ))

    joint_lists = [root_comp.joints]
    if hasattr(root_comp, 'asBuiltJoints'):
        joint_lists.append(root_comp.asBuiltJoints)

    for joints in joint_lists:
        for joint in joints:
            try:
                occ1, occ2 = joint.occurrenceOne, joint.occurrenceTwo
                if occ1 is None or occ2 is None:
                    continue
                n1, n2 = node_name(occ1), node_name(occ2)
                names.add(n1)
                names.add(n2)
                adj[n1].append(n2)
                adj[n2].append(n1)
            except Exception:
                continue

    return adj, names


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
            pair = tuple(sorted((cur, nb)))
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
    adj, names = build_adjacency(root_comp)
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

    return roots, all_extra_edges, base_name, adj


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

    nodes, edges = [], []
    for r in roots:
        flatten(r, nodes, edges)
    return nodes, edges


# ─────────────────────────────────────────────────────────────
#  SVG 생성
# ─────────────────────────────────────────────────────────────

def r2(v):
    return round(v, 2)


def node_style(name, base_name, adj):
    if name == base_name:
        return ACCENT, 3, False
    if not adj.get(name):
        return MUTED, 1.5, True   # 조인트가 하나도 없는 독립 부품
    return BASELINE, 1.5, False


def build_svg(roots, extra_edges, base_name, adj):
    nodes, edges = layout_forest(roots)
    by_id = {n['id']: n for n in nodes}

    legend_items = [
        (ACCENT, False, 'base_link (루트)'),
        (MUTED, True, '조인트 없는 독립 부품'),
        (CRITICAL, True, '폐루프(순환) 연결'),
    ]
    legend_width = sum(20 + text_width_estimate(label) + 24 for _, _, label in legend_items)

    xs = [n['_cx'] for n in nodes]
    ys = [n['_cy'] for n in nodes]
    min_x = min(xs) - NODE_W / 2 - PADDING
    max_x = max(xs) + NODE_W / 2 + PADDING
    max_x = max(max_x, min_x + PADDING + legend_width)
    min_y = min(ys) - NODE_H / 2 - PADDING - 40   # 상단 범례 공간
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

    # 트리 간선 (부모→자식, 왼쪽에서 오른쪽으로 꺾은선)
    for parent, child in edges:
        x1 = parent['_cx'] + NODE_W / 2
        x2 = child['_cx'] - NODE_W / 2
        mid_x = (x1 + x2) / 2
        d = 'M {x1} {py} H {mid} V {cy} H {x2}'.format(
            x1=r2(x1), py=r2(parent['_cy']), mid=r2(mid_x), cy=r2(child['_cy']), x2=r2(x2))
        parts.append('<path d="{0}" fill="none" stroke="{1}" stroke-width="1.5"/>'
                      .format(d, BASELINE))

    # 폐루프 간선 (BFS 트리에 포함되지 못한 추가 연결)
    for a, b in extra_edges:
        na, nb = by_id.get(a), by_id.get(b)
        if not na or not nb:
            continue
        parts.append(
            '<line x1="{0}" y1="{1}" x2="{2}" y2="{3}" stroke="{4}" '
            'stroke-width="2" stroke-dasharray="5 4"/>'.format(
                r2(na['_cx']), r2(na['_cy']), r2(nb['_cx']), r2(nb['_cy']), CRITICAL))

    # 노드
    for n in nodes:
        stroke, sw, dashed = node_style(n['id'], base_name, adj)
        x, y = n['_cx'] - NODE_W / 2, n['_cy'] - NODE_H / 2
        dash_attr = ' stroke-dasharray="4 3"' if dashed else ''
        parts.append(
            '<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" ry="8" '
            'fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash}/>'.format(
                x=r2(x), y=r2(y), w=NODE_W, h=NODE_H, fill=SURFACE, stroke=stroke, sw=sw, dash=dash_attr))
        parts.append(
            '<text x="{x}" y="{y}" text-anchor="middle" dominant-baseline="middle" '
            'font-size="12" fill="{ink}">{label}</text>'.format(
                x=r2(n['_cx']), y=r2(n['_cy']), ink=INK, label=xml_escape(truncate(n['id']))))

    # 범례
    legend_y = min_y + 22
    lx = min_x + PADDING
    for color, dashed, label in legend_items:
        dash_attr = ' stroke-dasharray="4 3"' if dashed else ''
        parts.append('<rect x="{0}" y="{1}" width="14" height="14" rx="3" ry="3" '
                      'fill="{2}" stroke="{3}" stroke-width="1.5"{4}/>'.format(
                          r2(lx), r2(legend_y - 11), SURFACE, color, dash_attr))
        parts.append('<text x="{0}" y="{1}" font-size="11" fill="{2}">{3}</text>'.format(
            r2(lx + 20), r2(legend_y), INK_SECONDARY, xml_escape(label)))
        lx += 20 + text_width_estimate(label) + 24

    parts.append('</svg>')
    return '\n'.join(parts)


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
        roots, extra_edges, base_name, adj = build_forest(root_comp)

        if not roots:
            ui.messageBox('디자인에 부품(occurrence)이 없습니다.')
            return

        if not base_name:
            ui.messageBox(
                "'base_link' 이름의 컴포넌트를 찾지 못했습니다.\n"
                "루트 컴포넌트를 base_link 로 지정한 뒤 다시 실행해주세요.\n\n"
                "(base_link 없이도 연결된 부품들은 트리로 표시되지만,\n"
                "요청하신 대로 base_link 를 트리 꼭대기에 둘 수는 없습니다.)")

        svg = build_svg(roots, extra_edges, base_name, adj)

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

        try:
            os.startfile(file_path)
        except Exception:
            pass

        ui.messageBox('조인트 트리 SVG 저장 완료:\n{0}'.format(file_path))

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
