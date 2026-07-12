"""
coacd_fingers.py — RG2 flex_finger / finger_tip 볼록분해 (CoACD).

핑거 충돌 인식이 부실한 문제 대응: 기존 rg2_v1 collision/*.stl 은 단순화된
로우폴리 shape 라 정밀 컨택엔 부족. visual/*.stl(실측 형상)을 CoACD 로 다중
볼록껍질 분해해서 실제 곡면 형상에 가까운 collision geom 을 만든다.

파라미터는 프로젝트 기존 관례(assets/MJCF/run_coacd_leftwall.py) 그대로:
threshold=0.05, max_convex_hull=32, preprocess_mode=auto, mcts_iterations=150.

출력: .../meshes/rg2_v1/coacd/{flex_finger,finger_tip}_hull_NNN.stl
"""
import os
import trimesh
import coacd

RG2_DIR = r"C:\Crusher_isaacsim\Crusher_Genesis\assets\robots\rg2\reference_onrobot_ros\meshes\rg2_v1"
VISUAL_DIR = os.path.join(RG2_DIR, "visual")
OUT_DIR = os.path.join(RG2_DIR, "coacd")
os.makedirs(OUT_DIR, exist_ok=True)

TARGETS = ["flex_finger", "finger_tip"]


def decompose(name):
    src = os.path.join(VISUAL_DIR, f"{name}.stl")
    print(f"[CoACD] {name}: loading {src}")
    mesh = trimesh.load(src)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(mesh.dump())
    print(f"[CoACD] {name}: {len(mesh.vertices)} verts, {len(mesh.faces)} faces (input)")

    coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
    parts = coacd.run_coacd(
        coacd_mesh,
        threshold=0.05,
        max_convex_hull=32,
        preprocess_mode="auto",
        mcts_iterations=150,
    )
    print(f"[CoACD] {name}: {len(parts)} convex hulls generated")

    hull_names = []
    for i, (verts, faces) in enumerate(parts):
        out_name = f"{name}_hull_{i:03d}.stl"
        out_path = os.path.join(OUT_DIR, out_name)
        trimesh.Trimesh(vertices=verts, faces=faces).export(out_path)
        hull_names.append(out_name)
        print(f"  saved {out_name}  ({len(verts)} verts, {len(faces)} faces)")
    return hull_names


if __name__ == "__main__":
    all_hulls = {}
    for t in TARGETS:
        all_hulls[t] = decompose(t)
    print("\n[CoACD] done:")
    for t, hulls in all_hulls.items():
        print(f"  {t}: {len(hulls)} hulls -> {OUT_DIR}")
