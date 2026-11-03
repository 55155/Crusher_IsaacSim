"""deck.json 순서대로 render/<id>.png 를 한 장씩 붙여 lab_meeting_deck_image.pptx 를 만든다 (이미지 슬라이드 + 발표자 노트, 보관용 — 본판은 make_pptx_native.py).
먼저 render_slides.py 로 PNG 를 갱신할 것. 실행: anaconda base python (python-pptx, PIL)."""
import json, os, re, html
from PIL import Image
from pptx import Presentation
from pptx.util import Emu

S = os.path.dirname(os.path.abspath(__file__))
SL = os.path.join(S, 'deck2', 'project', 'slides')
order = json.load(open(os.path.join(S, 'deck2', 'project', 'deck.json'), encoding='utf-8'))['order']

prs = Presentation()
prs.slide_width, prs.slide_height = Emu(12192000), Emu(6858000)  # 16:9
for sid in order:
    png = os.path.join(S, 'render', sid + '.png')
    im = Image.open(png)
    if im.size != (1920, 1080):  # 헤드리스 창이 1180 높이라 아래를 잘라낸다
        im = im.crop((0, 0, 1920, 1080)); png = os.path.join(S, 'render', sid + '_1080.png'); im.save(png)
    s = prs.slides.add_slide(prs.slide_layouts[6])
    s.shapes.add_picture(png, 0, 0, prs.slide_width, prs.slide_height)
    m = re.search(r'<aside[^>]*>(.*?)</aside>', open(os.path.join(SL, sid + '.html'), encoding='utf-8').read(), re.S)
    if m:
        s.notes_slide.notes_text_frame.text = html.unescape(re.sub(r'<[^>]+>', '', m.group(1))).strip()
out = os.path.join(S, 'lab_meeting_deck_image.pptx')
prs.save(out)
print('wrote', out, len(order), 'slides')
