"""probe_crusher_render_diff.py — 같은 Crusher MJCF 를 MuJoCo / Genesis 로 각각
렌더해 나란히 둔다 (2026-10-15, 사용자 지적 "Crusher 외형이 이상하다").

MuJoCo 가 기준이다 — MJCF 를 만든 쪽의 렌더러이므로 "설계대로"가 무엇인지 보여준다.
Genesis 는 geom 을 contype/conaffinity 로 `link.geoms`(충돌)/`link.vgeoms`(시각)로
가르고 **시각 쪽만 그리므로**, 시각 쌍둥이가 없는 geom 은 화면에서 사라진다.
어느 부품이 그런지 눈으로 가려내려는 용도다.

두 렌더는 같은 카메라(방위각/고도각/거리)를 쓴다. 입력 MJCF 는 full_workflow 가
실제로 쓰는 패치본(`_prepare_crusher_mjcf()`)이다 — 소스 XML 이 아니라.

사용법:  python probe_crusher_render_diff.py [--az -60] [--el 20] [--dist 0.55]
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE),
                                "Crusher_M0609_RG2_Tablet_Samplebag"))
import full_workflow as F                                    # noqa: E402

OUT = os.path.join(_HERE, "_crusher_render")


def cam_pos(center, az, el, dist):
    a, e = np.radians(az), np.radians(el)
    return np.array([center[0] + dist * np.cos(e) * np.cos(a),
                     center[1] + dist * np.cos(e) * np.sin(a),
                     center[2] + dist * np.sin(e)])


def _xml_with_offscreen(xml, w, h):
    """MuJoCo 오프스크린 프레임버퍼 기본값은 640x480 이라 그보다 큰 렌더가 거부된다.
    원본은 안 건드리고 <visual><global offwidth/offheight> 를 넣은 사본을 쓴다."""
    import xml.etree.ElementTree as ET
    t = ET.parse(xml); r = t.getroot()
    v = r.find("visual")
    if v is None:
        v = ET.SubElement(r, "visual")
    g = v.find("global")
    if g is None:
        g = ET.SubElement(v, "global")
    g.set("offwidth", str(w)); g.set("offheight", str(h))
    dst = os.path.join(os.path.dirname(xml), "_offscreen_" + os.path.basename(xml))
    t.write(dst)
    return dst


def render_mujoco(xml, center, pos, w, h):
    import mujoco
    m = mujoco.MjModel.from_xml_path(_xml_with_offscreen(xml, w, h))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = center
    cam.distance = float(np.linalg.norm(pos - np.asarray(center)))
    d_ = pos - np.asarray(center)
    cam.azimuth = float(np.degrees(np.arctan2(d_[1], d_[0])))
    cam.elevation = float(np.degrees(np.arcsin(d_[2] / np.linalg.norm(d_))))
    with mujoco.Renderer(m, height=h, width=w) as r:
        r.update_scene(d, camera=cam)
        return r.render()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--az", type=float, default=-60.0)
    ap.add_argument("--el", type=float, default=20.0)
    ap.add_argument("--dist", type=float, default=0.55)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    W, H = 960, 720

    xml = F._prepare_crusher_mjcf()
    print("[xml]", xml)

    # MuJoCo 는 MJCF 를 원점에 둔다 — 중심/카메라도 MJCF 로컬 기준으로 잡는다.
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml)
    lo, hi = m.stat.center - m.stat.extent, m.stat.center + m.stat.extent
    center_local = np.asarray(m.stat.center, float)
    dist = a.dist if a.dist > 0 else float(m.stat.extent) * 2.0
    pos_local = cam_pos(center_local, a.az, a.el, dist)

    from PIL import Image
    img_mj = render_mujoco(xml, center_local, pos_local, W, H)
    Image.fromarray(img_mj).save(os.path.join(OUT, "mujoco.png"))
    print("[mujoco] saved  center", np.round(center_local, 3), "dist", round(dist, 3))

    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning")
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.MJCF(file=xml, pos=(0, 0, 0), euler=(0, 0, 0),
                                    decimate=True, convexify=True))
    cam = scene.add_camera(res=(W, H), pos=tuple(pos_local), lookat=tuple(center_local),
                           fov=45, GUI=False)
    scene.build()
    Image.fromarray(cam.render(rgb=True)[0]).save(os.path.join(OUT, "genesis.png"))
    print("[genesis] saved")
    print(f"\n두 장을 비교해라: {OUT}")


if __name__ == "__main__":
    main()
