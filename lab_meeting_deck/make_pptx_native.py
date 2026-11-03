"""슬라이드 HTML → 편집 가능한 pptx (2026-11-03 요청: 이미지 pptx 는 편집도 GIF 재생도 안 된다).
헤드리스 Edge 로 각 슬라이드를 1920x1080 에 그리고, 페이지 안 스크립트가 요소마다 위치·글꼴·테두리를 재서
DOM 에 적어 두면(--dump-dom) 그걸 읽어 PowerPoint 도형으로 다시 놓는다.
  글자 → 텍스트 상자(Pretendard, 굵게/색/위·아래첨자 유지) · 테두리/배경 → 사각형·선 · 그림 → 그림(GIF 는 재생됨)
  · 분수 같은 수식 줄 → render/<id>.png 에서 그 영역만 잘라 그림으로 · <aside> → 발표자 노트
먼저 render_slides.py 로 render/*.png 를 갱신할 것. 실행: base anaconda (python-pptx, PIL).
출력: lab_meeting_deck.pptx (편집용 본판)"""
import base64, html, json, os, re, subprocess
from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Pt

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serve import BLOB  # noqa: E402

S = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(S, 'assets').replace('\\', '/')
SL = os.path.join(S, 'deck2', 'project', 'slides')
OUT = os.path.join(S, 'render')
EDGE = r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'
E = 6350                       # 1 px = 12192000 / 1920 EMU (슬라이드 1920 px 기준)

with open(os.path.join(S, 'render_slides.py'), encoding='utf-8') as f:   # 렌더와 같은 기본 스타일을 쓴다
    HEAD = re.search(r'head = f"""(.*?)"""', f.read(), re.S).group(1).replace('{{', '{').replace('}}', '}').replace('{A}', A)

JS = r"""<script>
addEventListener('load', async () => {
  await document.fonts.ready;
  const S = document.querySelector('section'), out = {items: [], note: ''};
  const px = v => parseFloat(v) || 0;
  const col = c => { const m = /rgba?\(([^)]+)\)/.exec(c || ''); if (!m) return null;
    const p = m[1].split(',').map(parseFloat); if (p.length == 4 && p[3] < 0.05) return null;
    return '#' + p.slice(0, 3).map(v => Math.round(v).toString(16).padStart(2, '0')).join(''); };
  const R = el => { const r = el.getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; };
  const isBlock = c => { const d = getComputedStyle(c).display;
    return !d.startsWith('inline') || c.tagName == 'IMG' || c.tagName == 'X-ICON'; };
  const raster = el => el.querySelector('span[style*="inline-flex"]') ||
    (/Times/.test(getComputedStyle(el).fontFamily) && el.querySelector('i,sub,sup'));
  function runs(el) {
    const rs = [];
    (function rec(n, st) {
      if (n.nodeType == 3) { const t = n.textContent.replace(/[ \t\r\n]+/g, ' '); if (t) rs.push([t, ...st]); return; }
      if (n.nodeType != 1) return;
      if (n.tagName == 'BR') { rs.push(['\n', ...st]); return; }
      const cs = getComputedStyle(n);
      const bold = /Bold/.test(cs.fontFamily) || px(cs.fontWeight) >= 600;
      const va = n.tagName == 'SUB' ? -1 : n.tagName == 'SUP' ? 1 : st[4];
      const s2 = [bold, col(cs.color), px(cs.fontSize), cs.fontStyle == 'italic', va];
      for (const c of n.childNodes) rec(c, s2);
    })(el, [false, '#404040', 16, false, 0]);
    return rs;
  }
  function walk(el) {
    if (el.tagName == 'ASIDE') { out.note = el.textContent.trim(); return; }
    const cs = getComputedStyle(el);
    if (cs.display == 'none' || cs.visibility == 'hidden') return;
    const r = R(el), bg = col(cs.backgroundColor), b = {};
    for (const s of ['Top', 'Right', 'Bottom', 'Left']) {
      const w = px(cs['border' + s + 'Width']);
      if (w > 0 && cs['border' + s + 'Style'] != 'none') b[s[0].toLowerCase()] = [w, col(cs['border' + s + 'Color'])];
    }
    if (el.tagName == 'X-ICON') { out.items.push({t: 'icon', r, name: el.getAttribute('name'), color: col(cs.color)}); return; }
    if ((bg && el !== S && bg != '#ffffff') || Object.keys(b).length || (bg == '#ffffff' && cs.position == 'absolute'))
      out.items.push({t: 'box', r, bg, b});
    if (el.tagName == 'IMG') {
      const nw = el.naturalWidth || r[2], nh = el.naturalHeight || r[3], s = Math.min(r[2] / nw, r[3] / nh);
      const w = nw * s, h = nh * s;
      out.items.push({t: 'img', r: [r[0] + (r[2] - w) / 2, r[1] + (r[3] - h) / 2, w, h], src: el.getAttribute('src')});
      return;
    }
    if (el.tagName == 'X-ICON') { out.items.push({t: 'icon', r, name: el.getAttribute('name'), color: col(cs.color)}); return; }
    const kids = [...el.children];
    if (el.textContent.trim() && !kids.some(isBlock)) {
      if (raster(el)) { out.items.push({t: 'raster', r}); return; }
      const pad = ['Left', 'Top', 'Right', 'Bottom'].map(s => px(cs['padding' + s]) + px(cs['border' + s + 'Width']));
      const lh = cs.lineHeight == 'normal' ? px(cs.fontSize) * 1.2 : px(cs.lineHeight);
      out.items.push({t: 'text', r, pad, align: cs.textAlign, lh, fs: px(cs.fontSize), nowrap: cs.whiteSpace == 'nowrap',
                      rot: cs.transform, runs: runs(el)});
      return;
    }
    for (const c of kids) walk(c);
  }
  walk(S);
  document.body.setAttribute('data-out', btoa(unescape(encodeURIComponent(JSON.stringify(out)))));
});
</script>"""


def measure(sid):
    h = open(os.path.join(SL, sid + '.html'), encoding='utf-8').read()
    h = re.sub(r'/_blob/([0-9a-f]{32})', lambda m: f'file:///{A}/{BLOB.get(m.group(1), "missing")}', h)
    p = os.path.join(OUT, sid + '_dom.html')
    open(p, 'w', encoding='utf-8').write(HEAD + h + JS)
    r = subprocess.run([EDGE, '--headless=new', '--disable-gpu', '--allow-file-access-from-files', '--window-size=1920,1080',
                        '--virtual-time-budget=8000', '--dump-dom', 'file:///' + p.replace('\\', '/')],
                       capture_output=True, timeout=120)
    m = re.search(rb'data-out="([^"]+)"', r.stdout)
    if not m:
        raise SystemExit(f'{sid}: 측정 실패')
    return json.loads(base64.b64decode(m.group(1)).decode('utf-8'))


def rgb(h):
    return RGBColor.from_string(h[1:].upper()) if h else None


def add_line(sl, x0, y0, x1, y1, w, color, arrow=False):
    ln = sl.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Emu(int(x0 * E)), Emu(int(y0 * E)), Emu(int(x1 * E)), Emu(int(y1 * E)))
    ln.line.width = Emu(int(max(w, 0.75) * E))
    ln.line.color.rgb = rgb(color or '#262626')
    if arrow:
        ln.line._get_or_add_ln().append(ln.line._get_or_add_ln().makeelement(qn('a:tailEnd'), {'type': 'triangle', 'w': 'med', 'len': 'med'}))
    return ln


def add_box(sl, it):
    x, y, w, h = it['r']
    b = it['b']
    full = len(b) == 4 and len({(round(v[0], 1), v[1]) for v in b.values()}) == 1
    if it['bg'] or full:
        sh = sl.shapes.add_shape(MSO_SHAPE.RECTANGLE, Emu(int(x * E)), Emu(int(y * E)), Emu(int(w * E)), Emu(int(h * E)))
        sh.shadow.inherit = False
        if it['bg']:
            sh.fill.solid(); sh.fill.fore_color.rgb = rgb(it['bg'])
        else:
            sh.fill.background()
        if full:
            bw, bc = b['t']
            sh.line.width = Emu(int(bw * E)); sh.line.color.rgb = rgb(bc)
        else:
            sh.line.fill.background()
    if not full:                                  # 한쪽 변만 있는 테두리(표 줄, 제목 아래 선) → 선
        for side, (bw, bc) in b.items():
            o = bw / 2
            seg = {'t': (x, y + o, x + w, y + o), 'b': (x, y + h - o, x + w, y + h - o),
                   'l': (x + o, y, x + o, y + h), 'r': (x + w - o, y, x + w - o, y + h)}[side]
            add_line(sl, *seg, bw, bc)


def add_text(sl, it):
    x, y, w, h = it['r']
    pl, pt, pr, pb = it['pad']
    lines = max(1, round((h - pt - pb) / it['lh'])) if it['lh'] else 1
    tb = sl.shapes.add_textbox(Emu(int(x * E)), Emu(int(y * E)), Emu(int((w * 1.04 + 6) * E)), Emu(int(h * E)))
    tf = tb.text_frame
    tf.margin_left, tf.margin_top, tf.margin_right, tf.margin_bottom = (Emu(int(v * E)) for v in (pl, pt, pr, pb))
    tf.word_wrap = lines > 1 and not it['nowrap']
    tf.auto_size = None
    tf.vertical_anchor = MSO_ANCHOR.TOP
    al = {'center': PP_ALIGN.CENTER, 'right': PP_ALIGN.RIGHT}.get(it['align'], PP_ALIGN.LEFT)
    p = tf.paragraphs[0]
    p.alignment = al
    if it['fs']:
        p.line_spacing = max(0.8, it['lh'] / (it['fs'] * 1.2))   # CSS 줄높이 → PowerPoint 배수(한 줄 ≈ 1.2 em)
    runs = it['runs']
    while runs and runs[0][0].strip() == '':
        runs = runs[1:]
    first = True
    for t, bold, color, size, ital, va in runs:
        parts = t.split('\n')
        for k, part in enumerate(parts):
            if k > 0:
                p = tf.add_paragraph(); p.alignment = al
                if it['fs']:
                    p.line_spacing = max(0.8, it['lh'] / (it['fs'] * 1.2))
                first = True
            if part == '' or (first and part.strip() == ''):
                continue
            r = p.add_run()
            r.text = part.lstrip() if first else part
            first = False
            f = r.font
            f.name = 'Pretendard'
            f.size = Pt(size * 0.5)               # 1 px = 0.5 pt (1920 px = 13.333 in)
            f.bold = bool(bold)
            f.italic = bool(ital)
            if color:
                f.color.rgb = rgb(color)
            if va:
                r._r.get_or_add_rPr().set('baseline', '30000' if va > 0 else '-25000')
    if it.get('rot') and 'matrix(0, -1' in it['rot']:
        tb.rotation = -90


def build():
    order = json.load(open(os.path.join(S, 'deck2', 'project', 'deck.json'), encoding='utf-8'))['order']
    prs = Presentation()
    prs.slide_width, prs.slide_height = Emu(12192000), Emu(6858000)
    for sid in order:
        d = measure(sid)
        sl = prs.slides.add_slide(prs.slide_layouts[6])
        shot = Image.open(os.path.join(OUT, sid + '.png'))
        n = {}
        for it in d['items']:
            n[it['t']] = n.get(it['t'], 0) + 1
            x, y, w, h = it['r']
            if w < 0.5 or h < 0.5:
                continue
            if it['t'] == 'box':
                add_box(sl, it)
            elif it['t'] == 'text':
                add_text(sl, it)
            elif it['t'] == 'img':
                src = (it['src'] or '').replace('file:///', '')
                if not os.path.isfile(src):                      # 사진 자리표시(src 없음) 등은 건너뛴다
                    print(f'  {sid}: 그림 없음 {it["src"]}')
                    continue
                sl.shapes.add_picture(src, Emu(int(x * E)), Emu(int(y * E)), Emu(int(w * E)), Emu(int(h * E)))
            elif it['t'] == 'raster':
                p = os.path.join(OUT, f'{sid}_eq{n["raster"]}.png')
                shot.crop((int(x), int(y), int(x + w + 1), int(y + h + 1))).save(p)
                sl.shapes.add_picture(p, Emu(int(x * E)), Emu(int(y * E)), Emu(int(w * E)), Emu(int(h * E)))
            elif it['t'] == 'icon':
                if it['name'] == 'ArrowDown':
                    add_line(sl, x + w / 2, y + h * 0.15, x + w / 2, y + h * 0.85, 2, it['color'], arrow=True)
                else:
                    add_line(sl, x + w * 0.15, y + h / 2, x + w * 0.85, y + h / 2, 2, it['color'], arrow=True)
        if d['note']:
            sl.notes_slide.notes_text_frame.text = html.unescape(d['note'])
        print(sid, n)
    out = os.path.join(S, 'lab_meeting_deck.pptx')
    prs.save(out)
    print('wrote', out)


if __name__ == '__main__':
    build()
