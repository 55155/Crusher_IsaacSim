"""Fusion 360 script — 약포지형 샘플백 서페이스 모델 자동 생성.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 파일이 있는 폴더를 추가 > 'fusion_sample_bag_surface' 선택 > Run

생성 결과 (새 컴포넌트 'SampleBag_70x100'):
  - Seal  : 하단 1cm + 양옆 1cm 실링 마진을 합친 U자형 단일 패치 서페이스 (평면, face 1개)
  - Front : 실링 안쪽 경계에서 시작해 상단 입구가 +Z로 벌어지는 로프트 서페이스
  - Back  : 동일하게 -Z로 벌어지는 로프트 서페이스
  상단은 실링 마진이 없으므로 입구가 열린 상태이며, 앞/뒤판의 옆·아래 모서리는
  실링 서페이스의 안쪽 U자 경계와 정확히 일치한다 (스티치는 하지 않음 —
  실링 안쪽 모서리에서 face 3장이 만나는 비다양체라 Fusion 스티치 불가).

주의: Fusion API 내부 단위는 cm이므로 아래 파라미터는 전부 cm 단위다.
"""

import traceback

import adsk.core
import adsk.fusion

# ---- 파라미터 (cm) -------------------------------------------------------
BAG_W = 7.0    # 가로 (X)
BAG_H = 10.0   # 세로 (Y)
SEAL = 1.0     # 실링 마진: 하단 + 양옆
MOUTH = 1.0    # 입구 벌어짐 깊이 (앞뒤 각각 최대 Z 변위) — 0에 가깝게 주면 거의 평평한 봉투
# --------------------------------------------------------------------------


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('활성화된 Fusion 디자인이 없습니다. 디자인 문서를 연 뒤 실행하세요.')
            return

        root = design.rootComponent
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = 'SampleBag_{:.0f}x{:.0f}'.format(BAG_W * 10, BAG_H * 10)  # mm 표기

        half_w = BAG_W / 2.0
        inner_half_w = half_w - SEAL          # 안쪽 폭의 절반 (2.5cm)
        P = adsk.core.Point3D.create

        # ---- 1) 실링 스케치: 바깥 사각형 + 안쪽 U자 분할선 (XY 평면, z=0) ----
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = 'seal_outline'
        lines = sk.sketchCurves.sketchLines

        lines.addTwoPointRectangle(P(-half_w, 0, 0), P(half_w, BAG_H, 0))
        l_left = lines.addByTwoPoints(P(-inner_half_w, BAG_H, 0), P(-inner_half_w, SEAL, 0))
        l_bottom = lines.addByTwoPoints(l_left.endSketchPoint, P(inner_half_w, SEAL, 0))
        lines.addByTwoPoints(l_bottom.endSketchPoint, P(inner_half_w, BAG_H, 0))

        # U자(실링) 프로파일 선택: 면적이 실링 면적과 가장 가까운 프로파일
        seal_area = BAG_W * BAG_H - (BAG_W - 2 * SEAL) * (BAG_H - SEAL)
        profiles = [sk.profiles.item(i) for i in range(sk.profiles.count)]
        prof_seal = min(profiles, key=lambda p: abs(p.areaProperties().area - seal_area))

        # ---- 2) 실링부: U자 프로파일 하나를 패치 → 단일 서페이스 ----
        patches = comp.features.patchFeatures
        patch_in = patches.createInput(prof_seal,
                                       adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        seal_feat = patches.add(patch_in)
        seal_feat.bodies.item(0).name = 'Seal'

        # ---- 3) 상단(y=BAG_H) 오프셋 평면에 입구 아치 2개 스케치 ----
        planes = comp.constructionPlanes
        plane_in = planes.createInput()
        plane_in.setByOffset(comp.xZConstructionPlane,
                             adsk.core.ValueInput.createByReal(BAG_H))
        top_plane = planes.add(plane_in)
        if abs(top_plane.geometry.origin.y - BAG_H) > 1e-6:
            # XZ 평면 법선 방향에 따라 오프셋 부호가 반대로 적용된 경우 뒤집는다
            top_plane.deleteMe()
            plane_in = planes.createInput()
            plane_in.setByOffset(comp.xZConstructionPlane,
                                 adsk.core.ValueInput.createByReal(-BAG_H))
            top_plane = planes.add(plane_in)

        sk_top = comp.sketches.add(top_plane)
        sk_top.name = 'mouth_arcs'

        def on_top(x, z):
            return sk_top.modelToSketchSpace(P(x, BAG_H, z))

        arcs = sk_top.sketchCurves.sketchArcs
        arc_front = arcs.addByThreePoints(on_top(-inner_half_w, 0),
                                          on_top(0, MOUTH),
                                          on_top(inner_half_w, 0))
        arc_back = arcs.addByThreePoints(on_top(-inner_half_w, 0),
                                         on_top(0, -MOUTH),
                                         on_top(inner_half_w, 0))

        # ---- 4) 앞판/뒷판: 실링 안쪽 하단 라인 → 상단 아치 서페이스 로프트 ----
        # 2단면 로프트라 옆 모서리는 아치 끝점(z=0)과 하단 라인 끝점을 잇는
        # 직선이 되어 실링 안쪽 세로 경계와 정확히 일치한다.
        feats = comp.features
        for arc, name in ((arc_front, 'Front'), (arc_back, 'Back')):
            loft_in = feats.loftFeatures.createInput(
                adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            loft_in.isSolid = False
            loft_in.loftSections.add(feats.createPath(l_bottom, False))
            loft_in.loftSections.add(feats.createPath(arc, False))
            loft = feats.loftFeatures.add(loft_in)
            loft.bodies.item(0).name = name

        ui.messageBox(
            'SampleBag 생성 완료\n'
            '  전체: {:.0f} x {:.0f} mm, 실링 마진 {:.0f} mm (하단+양옆)\n'
            '  바디: Seal(단일 서페이스) / Front / Back\n'
            '  상단 입구 개방, 벌어짐 ±{:.0f} mm'.format(
                BAG_W * 10, BAG_H * 10, SEAL * 10, MOUTH * 10))

    except Exception:
        if ui:
            ui.messageBox('스크립트 실패:\n{}'.format(traceback.format_exc()))
