# -*- coding: utf-8 -*-
"""mesh_geometry_extract.py — 폴더 안의 모든 STL(.stl)에서 기하정보를 뽑아 CSV로 저장.

Fusion 360이 아니라 일반 파이썬(trimesh)으로 동작 -- 이미 export된 meshes/
폴더를 대상으로 길이/부피/무게중심 등을 빠르게 훑어보거나, Fusion에서 뽑은
joint anchor 거리(dump_joint_anchors)와 대조 검증할 때 쓴다.

사용법
------
    python mesh_geometry_extract.py <meshes 폴더 경로> [--out OUT.csv] [--pattern *.stl]

예:
    python mesh_geometry_extract.py \
        "C:\\Crusher_isaacsim\\Crusher_Genesis\\assets\\robots\\회수장치2_description\\meshes"

출력 CSV 컬럼
-------------
    filename          : 파일명
    n_vertices, n_faces
    bbox_min_x/y/z, bbox_max_x/y/z   : 축정렬 바운딩박스 (raw 단위, 보통 mm)
    extent_x/y/z                     : 바운딩박스 각 축 길이
    bbox_diagonal                    : 바운딩박스 대각선 길이(축정렬 기준 최대 길이 근사)
    principal_length                : PCA로 구한 "가장 긴 축" 방향 실제 스팬
                                        (막대/샤프트처럼 기울어진 부품의 실제
                                        길이를 축정렬 바운딩박스보다 더 정확히 반영)
    centroid_x/y/z                   : 정점 평균(무게중심 근사, 균일밀도 가정
                                        아님 -- 정확한 질량중심은 Fusion에서
                                        physicalProperties로 뽑는 게 정확함)
    is_watertight, volume            : 워터타이트(닫힌 솔리드)인 경우만 부피 유효
"""

import argparse
import glob
import os
import sys

import numpy as np
import trimesh


def analyze_mesh(path):
    m = trimesh.load(path, force="mesh", process=False)
    verts = np.asarray(m.vertices)

    bbox_min = verts.min(axis=0)
    bbox_max = verts.max(axis=0)
    extent = bbox_max - bbox_min
    bbox_diag = float(np.linalg.norm(extent))

    centroid = verts.mean(axis=0)
    vc = verts - centroid
    cov = vc.T @ vc
    eigval, eigvec = np.linalg.eigh(cov)
    long_axis = eigvec[:, np.argmax(eigval)]
    proj = vc @ long_axis
    principal_length = float(proj.max() - proj.min())

    is_watertight = False
    volume = ""
    try:
        m_check = m.copy()
        m_check.merge_vertices()
        is_watertight = bool(m_check.is_watertight)
        if is_watertight:
            volume = float(m_check.volume)
    except Exception:
        pass

    return {
        "filename": os.path.basename(path),
        "n_vertices": len(m.vertices),
        "n_faces": len(m.faces),
        "bbox_min_x": bbox_min[0], "bbox_min_y": bbox_min[1], "bbox_min_z": bbox_min[2],
        "bbox_max_x": bbox_max[0], "bbox_max_y": bbox_max[1], "bbox_max_z": bbox_max[2],
        "extent_x": extent[0], "extent_y": extent[1], "extent_z": extent[2],
        "bbox_diagonal": bbox_diag,
        "principal_length": principal_length,
        "centroid_x": centroid[0], "centroid_y": centroid[1], "centroid_z": centroid[2],
        "is_watertight": is_watertight,
        "volume": volume,
    }


CSV_COLUMNS = [
    "filename", "n_vertices", "n_faces",
    "bbox_min_x", "bbox_min_y", "bbox_min_z",
    "bbox_max_x", "bbox_max_y", "bbox_max_z",
    "extent_x", "extent_y", "extent_z",
    "bbox_diagonal", "principal_length",
    "centroid_x", "centroid_y", "centroid_z",
    "is_watertight", "volume",
]


def main():
    ap = argparse.ArgumentParser(description="폴더 내 STL들의 기하정보를 CSV로 추출")
    ap.add_argument("folder", help="STL 파일들이 있는 폴더 경로")
    ap.add_argument("--out", default=None, help="출력 CSV 경로 (기본: <folder>/mesh_geometry.csv)")
    ap.add_argument("--pattern", default="*.stl", help="glob 패턴 (기본 *.stl)")
    args = ap.parse_args()

    folder = args.folder
    out_path = args.out or os.path.join(folder, "mesh_geometry.csv")
    files = sorted(glob.glob(os.path.join(folder, args.pattern)))

    if not files:
        print(f"[경고] '{folder}'에서 '{args.pattern}' 패턴에 맞는 파일을 찾지 못했습니다.")
        sys.exit(1)

    rows = []
    for f in files:
        try:
            rows.append(analyze_mesh(f))
            print(f"  OK   {os.path.basename(f)}")
        except Exception as e:
            print(f"  실패  {os.path.basename(f)}  ({e})")

    import csv
    with open(out_path, "w", newline="", encoding="utf-8-sig") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    print(f"\n총 {len(rows)}/{len(files)}개 처리 완료. 저장: {out_path}")


if __name__ == "__main__":
    main()
