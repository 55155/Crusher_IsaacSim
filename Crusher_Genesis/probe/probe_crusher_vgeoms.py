"""probe_crusher_vgeoms.py — Crusher 링크별 충돌/시각 geom 수를 센다 (2026-10-15).

사용자 지적: "Crusher 외형이 이상하다, 일부 geom 이 visualization 이 안 된다 (예 Wall_1)".

가설: Genesis 는 MJCF geom 을 contype/conaffinity 로 갈라 `link.geoms`(충돌)와
`link.vgeoms`(시각)에 넣는데, 소스 MJCF `Crusher_IsaacSim_colored.xml` 의 geom 53개
중 36개만 `contype=0 conaffinity=0`(=시각)이고 나머지는 contype 속성이 아예 없어
MuJoCo 기본값 1/1 = 충돌로 들어간다. 그 15개 중 Left_Wall hull 11개는 따로 시각
geom 을 받지만, **L1_Wall1_1 / L1_Wall2_1 / L2_Wall3_1 / L9_PLATE_v3_1 은 시각
쌍둥이가 없다** -> 충돌에만 존재해서 화면에 안 그려진다.

이 스크립트는 그 가설을 빌드 결과로 확인한다. 씬은 Crusher 하나만 올린다
(full_workflow 와 같은 `_prepare_crusher_mjcf()` 산출물을 쓴다).

사용법:  python probe_crusher_vgeoms.py
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE),
                                "Crusher_M0609_RG2_Tablet_Samplebag"))
import full_workflow as F                                    # noqa: E402


def main():
    import genesis as gs
    gs.init(backend=gs.gpu, logging_level="warning")
    scene = gs.Scene(show_viewer=False)
    crusher = scene.add_entity(
        gs.morphs.MJCF(file=F._prepare_crusher_mjcf(), pos=F.CRUSHER_POS,
                       euler=F.CRUSHER_EULER, decimate=True, convexify=True),
    )
    scene.build()

    print(f"\n{'link':34s} {'충돌':>5s} {'시각':>5s}   비고")
    bad = []
    for lk in crusher.links:
        ng, nv = len(lk.geoms), len(lk.vgeoms)
        if ng == 0 and nv == 0:
            continue
        note = ""
        if nv == 0:
            note = "<-- 시각 geom 0개: 화면에 안 나온다"
            bad.append(lk.name)
        print(f"{lk.name:34s} {ng:5d} {nv:5d}   {note}")
    print(f"\n[probe] 시각 geom 이 없는 링크 {len(bad)}개: {bad}")


if __name__ == "__main__":
    main()
