"""샘플백에서 공극률을 못 쓰는 이유 — 판 아래 알이 옆·위로 밀려나는 장면을 Newton 뷰어로 렌더 (2026-11-03).
알 색: 시작 때 판 투영(x 186.5–236.5 mm) 안 → 끝까지 남음 = 녹색, 밖으로 밀려남 = 주황, 처음부터 밖 = 회색.
newton env 로 실행:  python -I fig_escape_render.py [--labels] [--test]
출력: assets/escape_raw_<view>_<frame>.png (합성은 fig_escape_compose.py)"""
import argparse
import json
import os
import sys

import numpy as np
import warp as wp

SCENE = r"C:\Newton_sim\crusher_scene"
sys.path.insert(0, SCENE)
import newton  # noqa: E402
from render_crush_real import build_scene, make_viewer  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FR = os.path.join(SCENE, "crush_out", "dem_frames.npz")
REC = os.path.join(SCENE, "genesis_rec", "full_workflow_rec.npz")
PLATE_X = (186.5e-3, 236.5e-3)
GREEN, ORANGE, GREY = (0.11, 0.50, 0.23), (0.93, 0.47, 0.05), (0.62, 0.63, 0.66)
# 이름: (cam x y z pitch yaw, 숨길 바디 부분문자열)
VIEWS = {
    "front": ((0.2115, 0.11, 0.064, -3.0, -90.0), ["*"]),   # 앤빌 쪽(+y)에서 판을 정면으로 (판만 남김)
    "side": ((0.120, -0.003, 0.066, -6.0, 0.0), ["*"]),      # 옆(-x)에서 y–z 단면
}

ap = argparse.ArgumentParser()
ap.add_argument("--labels", action="store_true")
ap.add_argument("--test", action="store_true")
ap.add_argument("--views", nargs="+", default=list(VIEWS))
ap.add_argument("--hide", nargs="*", default=None)
ap.add_argument("--cam", type=float, nargs=5, default=None)
ap.add_argument("--w", type=int, default=1200)
ap.add_argument("--h", type=int, default=900)
a = ap.parse_args()
ARGV = sys.argv[1:]

with open(os.path.join(SCENE, "manifest.json"), encoding="utf-8") as f:
    man = json.load(f)
if not os.path.exists(man["crusher"]):
    # 가공 MJCF 는 %TEMP% 에 생겨 지워진다 — manifest.json 은 건드리지 않고 이번 실행용으로만 다시 만든다
    sys.argv = sys.argv[:1]
    sys.path.insert(0, r"C:\Crusher_isaacsim\Crusher_Genesis\Crusher_M0609_RG2_Tablet_Samplebag")
    import full_workflow as fw  # noqa: E402
    man["crusher"] = fw._prepare_crusher_mjcf()
    sys.argv = [sys.argv[0]] + ARGV
rec = dict(np.load(REC))
model, body_rec, n0, plate_k = build_scene(man, [str(x) for x in rec["link_names"]])
if a.labels:
    for i in range(model.body_count):
        print(i, model.body_label[i])
    raise SystemExit

d = dict(np.load(FR))
x = d["x"]
lo, hi = d["bag"].min(1, keepdims=True) - 2e-3, d["bag"].max(1, keepdims=True) + 2e-3
inb = np.all((x >= lo) & (x <= hi), axis=2)          # 판-앤빌에 끼여 튄 알 4개는 그리지 않는다
fp = lambda p: (p[:, 0] > PLATE_X[0]) & (p[:, 0] < PLATE_X[1])
m0 = fp(x[0])
frames = [0, len(d["t"]) - 1]

viewer = make_viewer(a.w, a.h)
viewer.set_model(model)
flags0 = model.shape_flags.numpy().copy()
faces = wp.array(np.asarray(d["bag_faces"], np.int32).ravel(), dtype=wp.int32)
state = model.state()
bq = state.body_q.numpy()
P, Q = rec["link_pos"], rec["link_quat_wxyz"]
from PIL import Image  # noqa: E402

for name in a.views:
    cam, hide = VIEWS[name]
    cam = a.cam or cam
    hide = a.hide if a.hide is not None else hide
    fl = flags0.copy()
    if hide:
        sb = model.shape_body.numpy()
        if hide == ["*"]:   # 판(충격판 링크)만 남기고 크러셔·바닥판 전부 숨김 — 앤빌이 시야를 막는다
            hid_s = np.where(sb != n0 + plate_k)[0]
        else:
            hid = [i for i in range(model.body_count) if any(h in model.body_label[i] for h in hide)]
            hid_s = np.where(np.isin(sb, hid))[0]
        for si in hid_s:
            fl[si] &= ~int(newton.ShapeFlags.VISIBLE)
    model.shape_flags.assign(fl)
    viewer.set_model(model)   # 뷰어는 set_model 때 형상 목록을 굳힌다 — 숨김을 바꾸면 다시 넘겨야 한다
    for i in (frames[:1] if a.test else frames):
        s = int(round(float(d["s_rec"][i])))
        for k, col in enumerate(body_rec):
            if col < 0:
                continue
            q = Q[s, col]
            bq[n0 + k] = np.array([*P[s, col], q[1], q[2], q[3], q[0]], np.float32)
            if k == plate_k:
                bq[n0 + k, 1] += float(d["plate_calib_y"])
        state.body_q.assign(bq)
        ok = inb[i]
        p = x[i][ok]
        c = np.tile(np.array(GREY, np.float32), (len(p), 1))
        c[m0[ok] & fp(p)] = GREEN
        c[m0[ok] & ~fp(p)] = ORANGE
        viewer.set_camera(wp.vec3(*cam[:3]), cam[3], cam[4])
        viewer.begin_frame(float(d["t"][i]))
        viewer.log_state(state)
        viewer.log_mesh("/bag", wp.array(d["bag"][i].astype(np.float32), dtype=wp.vec3), faces,
                        color=(0.90, 0.91, 0.95), opacity=0.18, backface_culling=False, dynamic=True, roughness=0.6)
        viewer.log_points("/iron", wp.array(p.astype(np.float32), dtype=wp.vec3),
                          radii=wp.full(len(p), 0.25e-3, dtype=wp.float32), colors=wp.array(c, dtype=wp.vec3))
        viewer.end_frame()
        out = os.path.join(HERE, "assets", f"escape_raw_{name}_{i}.png")
        Image.fromarray(viewer.get_frame().numpy()).save(out)
        print("wrote", out, f"t={d['t'][i]*1e3:.0f} ms, green {int(np.isclose(c, GREEN).all(1).sum())}, orange {int(np.isclose(c, ORANGE).all(1).sum())}")
