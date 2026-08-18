"""smooth_shade_meshes.py — Genesis(pyrender) 렌더링 스파이크 제거용 시각 메시 전처리.

docs/DigitalTwin.md "고정장치/M0609 렌더링 스파이크 아티팩트 — 해결(2026-07-21)"
에서 고정장치 12개 / M0609 링크 10개 메시에 적용했던 것과 **같은 처리**를 임의의
메시 폴더에 적용한다.

증상과 원인(위 문서 요약): `smooth=True`(Genesis 기본값)에서 볼트머리 같은 작은
디테일 부근에 별모양 스파이크가 생긴다. `pyrender/mesh.py` 의 smooth 셰이딩이
`trimesh.vertex_normals` 를 **크리스(crease) 각도 구분 없이** 그대로 쓰기 때문
(hard-edge 개념이 없다). MuJoCo 자체 렌더러는 날카로운 모서리에서 정점을 분리해
법선을 평균내지 않으므로 같은 STL 이어도 멀쩡하게 나온다.

처리: `trimesh` 의 `mesh.smooth_shaded`(크리스 각도 기준으로 정점을 분리한 뒤
스무싱)로 다시 내보낸 `<이름>_ss.obj` 를 만든다.

**시각 geom 에만 쓸 것.** 정점을 분리한 결과물이라 non-watertight 이고, 충돌
geom 으로 쓰면 SDF/부피 계산이 깨진다 — 충돌은 원본 STL 을 그대로 유지한다.

사용법:
    python smooth_shade_meshes.py <mesh_dir>              # *.stl -> *_ss.obj
    python smooth_shade_meshes.py <mesh_dir> --force      # 이미 있어도 다시 만듦
    python smooth_shade_meshes.py <mesh_dir> --pattern "*.obj"
"""
import argparse
import glob
import os
import sys

import trimesh as tm

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass


def smooth_shade_dir(mesh_dir, pattern="*.stl", force=False):
    """mesh_dir 안의 pattern 메시들을 <stem>_ss.obj 로 재수출. (생성목록, 건너뛴목록) 반환."""
    made, skipped = [], []
    for src in sorted(glob.glob(os.path.join(mesh_dir, pattern))):
        stem = os.path.splitext(os.path.basename(src))[0]
        if stem.endswith("_ss"):
            continue
        dst = os.path.join(mesh_dir, stem + "_ss.obj")
        if os.path.isfile(dst) and not force:
            skipped.append(dst)
            continue
        mesh = tm.load(src, force="mesh")
        ss = mesh.smooth_shaded
        # OBJ 로 내보낼 때 재질 파일(.mtl)은 만들지 않는다 — MJCF 는 mesh 의
        # 지오메트리만 쓰고 색은 geom rgba 로 준다.
        ss.export(dst, include_texture=False)
        made.append(dst)
        print(f"  {os.path.basename(src):<28} verts {len(mesh.vertices):>6} -> {len(ss.vertices):>6}"
              f"  faces {len(mesh.faces):>6}  -> {os.path.basename(dst)}")
    return made, skipped


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh_dir")
    ap.add_argument("--pattern", default="*.stl")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    if not os.path.isdir(args.mesh_dir):
        sys.exit(f"디렉토리가 없다: {args.mesh_dir}")

    print(f"[smooth_shade] {args.mesh_dir}  pattern={args.pattern}")
    made, skipped = smooth_shade_dir(args.mesh_dir, args.pattern, args.force)
    print(f"[smooth_shade] 생성 {len(made)}개, 건너뜀 {len(skipped)}개(--force 로 재생성)")


if __name__ == "__main__":
    main()
