# -*- coding: utf-8 -*-
"""
Mesh.py — 링크 STL 을 `<save_dir>/meshes/` 에 내보낸다. (`adsk` 필요)

fusion2urdf `utils.copy_occs` + `utils.export_stl` 과 같은 방식이다. 왜 굳이
컴포넌트를 복제하느냐면, occurrence 를 그대로 내보내면 메시가 **그 컴포넌트의
로컬 좌표**로 나오기 때문이다. 새 컴포넌트를 단위행렬 변환으로 만들어 body 를
복사해 넣으면 조립된 위치 그대로, 즉 **root 좌표**로 나온다. MJCF 쪽 `<geom pos>`
가 `-링크원점` 인 것은 이 전제 위에서만 맞다.

경고: `copy_occs` 는 **디자인을 실제로 바꾼다**(모든 컴포넌트를 복제하고 원본
이름을 'old_component' 로 바꾼다). 그래서 MJCF 를 다 쓴 뒤 맨 마지막에 부르고,
호출한 쪽에서 사용자에게 "저장하지 말고 Undo 하라" 고 알려야 한다.
"""

import adsk
import adsk.core
import adsk.fusion
import os

from .Extract import sanitize


def copy_occs(root):
    """모든 컴포넌트를 root 좌표에 놓인 새 컴포넌트로 복제한다.

    새 컴포넌트 이름은 `Extract.sanitize` 로 짓는다 — 링크 이름을 짓는 것과
    같은 함수여야 `export_stl` 이 낸 파일명과 MJCF 의 링크 이름이 맞는다.
    """
    def copy_body(all_occs, occs):
        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()
        new_occs = all_occs.addNewComponent(transform)
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = sanitize(occs.name)
        new_occs = all_occs.item(all_occs.count - 1)
        for i in range(bodies.count):
            bodies.item(i).copyToComponent(new_occs)

    all_occs = root.occurrences
    old = []
    for occs in [o for o in all_occs]:
        if occs.bRepBodies.count > 0:
            copy_body(all_occs, occs)
            old.append(occs)
    for occs in old:
        occs.component.name = 'old_component'


def export_stl(design, root, save_dir, wanted=None):
    """`save_dir/meshes/<링크>.stl` 로 내보낸다. (exported, failed)

    **최상위 occurrence 만 본다.** 예전에는 `design.allComponents` 를 훑고
    그 안에서 다시 `allOccurrences` 를 돌아, `copy_occs` 가 손대지 않은
    하위 컴포넌트 원본까지 전부 내보내려 들었다. 그 결과:

    - 아무 body 도 안 쓰는 mesh 가 `<asset>` 에 쌓인다(실측 40개 중 7개,
      그중 하나가 1MB 짜리 '제목 없음').
    - 그중 이름에 한글이 섞이면 MuJoCo 가 **모델 전체의 로딩을 거부**한다.
    - 베어링 볼처럼 body 가 없는 컴포넌트에서 실패가 쏟아진다(실측 115건).

    `wanted` 를 주면 그 링크만 내보낸다 — body 트리에 실제로 들어간 링크
    집합을 넘기면 `<asset>` 과 body 가 정확히 일대일이 된다.
    """
    mesh_dir = os.path.join(save_dir, 'meshes')
    try:
        os.mkdir(mesh_dir)
    except Exception:
        pass

    export_mgr = design.exportManager
    exported, failed, skipped = set(), [], []
    for occ in root.occurrences:
        name = occ.component.name
        if 'old_component' in name:
            continue
        if wanted is not None and name not in wanted:
            skipped.append(name)
            continue
        if occ.bRepBodies.count == 0:
            skipped.append(name)
            continue
        try:
            opts = export_mgr.createSTLExportOptions(
                occ, os.path.join(mesh_dir, name))
            opts.sendToPrintUtility = False
            opts.isBinaryFormat = True
            opts.meshRefinement = \
                adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            export_mgr.execute(opts)
            exported.add(name)
        except Exception:
            failed.append(name)
    return exported, failed
