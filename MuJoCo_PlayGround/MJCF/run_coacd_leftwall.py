"""
run_coacd_leftwall.py
L2_Left_Wall1_1.stl 을 CoACD 로 볼록 분해하여
L2_Left_Wall1_1_hull_000.stl, _hull_001.stl ... 로 저장합니다.
"""
import os
import numpy as np
import trimesh
import coacd

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT_STL = os.path.join(HERE, "L2_Left_Wall1_1.stl")

print(f"[CoACD] Loading {INPUT_STL} ...")
mesh = trimesh.load(INPUT_STL)
if isinstance(mesh, trimesh.Scene):
    mesh = trimesh.util.concatenate(mesh.dump())

# trimesh → coacd.Mesh
coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)

print("[CoACD] Running decomposition  (threshold=0.05) ...")
parts = coacd.run_coacd(
    coacd_mesh,
    threshold=0.05,          # 0.01 ~ 1.0 : 낮을수록 정밀, 높을수록 hull 수 감소
    max_convex_hull=32,
    preprocess_mode="auto",
    mcts_iterations=150,
)

print(f"[CoACD] {len(parts)} convex hulls generated.")
hull_names = []
for i, (verts, faces) in enumerate(parts):
    out_name = f"L2_Left_Wall1_1_hull_{i:03d}.stl"
    out_path = os.path.join(HERE, out_name)
    hull_mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    hull_mesh.export(out_path)
    hull_names.append(out_name)
    print(f"  saved {out_name}  ({len(verts)} verts, {len(faces)} faces)")

print("\n[CoACD] Done. Hull files:")
for n in hull_names:
    print(f"  {n}")
