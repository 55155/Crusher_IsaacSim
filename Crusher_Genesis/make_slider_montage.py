"""
make_slider_montage.py — impact plate(슬라이더) 각도별 스틸을 2×3 라벨 몽타주로 합성.

입력 : Sim_result/crusher_slider_angles/slider_{deg}deg.png  (Left Wall 제거·plate 포커스)
출력 : Sim_result/_crank_angles_6_stroke.png  (빨강 각도 라벨, 밝은 배경, 2행×3열)

Crusher_only.py 의 SLIDER_VIEW 렌더( HIDE_LEFT_WALL=1 SLIDER_VIEW=1 CRANK_ANGLES=... )
로 만든 6장을 대상으로 한다. 각도/크롭/그리드는 아래 상수로 조정.
"""
import os
from PIL import Image, ImageDraw, ImageFont

HERE   = os.path.dirname(os.path.abspath(__file__))
SRCDIR = os.path.join(HERE, "Sim_result", "crusher_slider_angles")
OUT    = os.path.join(HERE, "Sim_result", "_crank_angles_6_stroke.png")

ANGLES = [15, 30, 45, 60, 75, 90]
COLS, ROWS = 3, 2
CROP   = (980, 300, 2180, 1500)        # 원본 2400×1600 → 벽(앵커)쪽 정사각 1200×1200
CANVAS = (3508, 2376)                  # 크랭크 몽타주(_crank_angles_6_crank.png)와 동일 A4
BG     = (233, 238, 247)               # 렌더 배경(0.92,0.94,0.97) 과 동일 톤
LABEL  = (206, 0, 0)                    # 빨강
MARGIN, GAP = 44, 30
FONT_PATHS = ["C:/Windows/Fonts/arialbd.ttf", "C:/Windows/Fonts/arial.ttf"]


def _font(sz):
    for p in FONT_PATHS:
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


def main():
    tiles = []
    for a in ANGLES:
        p = os.path.join(SRCDIR, f"slider_{a:03d}deg.png")
        tiles.append(Image.open(p).convert("RGB").crop(CROP))
    tw, th = tiles[0].size                             # 크롭 후 타일 원본 크기(정사각)

    canvas_w, canvas_h = CANVAS
    cell_w = (canvas_w - 2 * MARGIN - (COLS - 1) * GAP) // COLS
    scale  = cell_w / tw
    cell_h = int(th * scale)                            # 정사각 크롭 → cell_h ≈ cell_w
    grid_h = ROWS * cell_h + (ROWS - 1) * GAP
    y_off  = (canvas_h - grid_h) // 2                   # 세로 중앙정렬(A4 높이 맞춤)

    canvas = Image.new("RGB", (canvas_w, canvas_h), BG)
    draw = ImageDraw.Draw(canvas)
    font = _font(int(cell_h * 0.16))
    pad = int(cell_h * 0.04)

    for i, (a, tile) in enumerate(zip(ANGLES, tiles)):
        r, c = divmod(i, COLS)
        x = MARGIN + c * (cell_w + GAP)
        y = y_off + r * (cell_h + GAP)
        canvas.paste(tile.resize((cell_w, cell_h), Image.LANCZOS), (x, y))
        draw.text((x + pad, y + pad), f"{a}°", fill=LABEL, font=font)

    canvas.save(OUT)
    print(f"[montage] {COLS}x{ROWS}  cell={cell_w}x{cell_h}  -> {OUT}")


if __name__ == "__main__":
    main()
