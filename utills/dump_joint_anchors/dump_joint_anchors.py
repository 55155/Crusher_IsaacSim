# -*- coding: utf-8 -*-
"""Fusion 360 script — 폐루프에서 중복 export되는 부품의 joint anchor 좌표 추출.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 utills/dump_joint_anchors 폴더(파일명과 폴더명이 같아야 Fusion이 Script로
  인식함 — utills 폴더 자체를 추가하면 안 됨)를 선택 > 목록에서
  'dump_joint_anchors' 선택 > Run.

동작:
  - 대상 컴포넌트 이름 패턴(부분 문자열, 예: "Crank")을 입력받는다. 비워두거나
    "*"/"all"을 입력하면 패턴 필터 없이 디자인 전체의 모든 컴포넌트/joint를
    대상으로 한다(폐루프로 중복 export되는 부품이 여러 개 있어도 한 번에
    전부 찾아줌).
  - 어셈블리 전체 joint를 훑어서, 이름이 매칭되는 컴포넌트가 물려있는 joint를
    전부 찾아 각 joint 원점을 월드 좌표와 해당 컴포넌트 자신의 로컬 좌표(=MJCF
    <connect anchor="..."/> 에 바로 쓸 값) 양쪽으로 계산한다.
  - fusion2urdf는 하나의 부품이 폐루프 안에서 두 joint에 물려 있으면 URDF
    트리 제약 때문에 링크를 2개로 쪼개 export한다. 이 스크립트는 그 원래
    부품(Fusion 상에서는 여전히 occurrence 1개)에 물린 두 joint의 로컬 anchor를
    모두 뽑아, 둘 사이 거리(=커넥팅로드/크랭크의 실제 핀-핀 길이)까지 계산한다.
  - 결과 요약은 항상 messageBox(대화상자)로 먼저 뜬다 -- Text Commands
    팔레트는 환경에 따라 안 뜨는 경우가 있어 messageBox를 1차 출력으로 씀.
    이어서 CSV 저장 경로를 고르는 다이얼로그가 뜬다(취소해도 이미 messageBox로
    핵심 결과는 본 상태).

fusion_addin/dump_joint_anchors/ 아래에 같은 로직의 Add-In(툴바 버튼) 버전도
있다 — 매번 Scripts 탭에서 파일을 추가하지 않고 Fusion 시작 시 자동 로드하고
싶으면 그쪽을 쓴다.
"""

import adsk.core, adsk.fusion, traceback

from dump_joint_anchors_core import collect_records, write_csv


def _summary_text(records, name_pattern, target_occs, n_joints):
    anchor_rows = [r for r in records if r["record_type"] == "anchor"]
    dist_rows = [r for r in records if r["record_type"] == "distance"]

    lines = []
    lines.append(f"'{name_pattern}' 매칭 occurrence {len(target_occs)}개 / 전체 joint {n_joints}개")
    lines.append(f"anchor 레코드 {len(anchor_rows)}개, distance 레코드 {len(dist_rows)}개")
    lines.append("")

    if dist_rows:
        lines.append("[핀-핀 거리 / MJCF anchor 벡터]")
        for r in dist_rows[:30]:
            lines.append(
                f"  [{r['component']}] {r['joint']}\n"
                f"    거리={r['distance_m']:.6f} m\n"
                f"    anchor=({r['anchor_vec_x_m']:.6f}, {r['anchor_vec_y_m']:.6f}, {r['anchor_vec_z_m']:.6f})"
            )
        if len(dist_rows) > 30:
            lines.append(f"  ... 외 {len(dist_rows) - 30}개 더 (전체 내용은 CSV 참고)")
    else:
        lines.append("(같은 컴포넌트에 joint가 2개 이상 물린 경우가 없어 거리 계산 결과 없음)")
        lines.append("")
        lines.append("[개별 anchor 좌표(최대 10개까지 표시)]")
        for r in anchor_rows[:10]:
            local = (f"local=({r['local_x_m']:.6f}, {r['local_y_m']:.6f}, {r['local_z_m']:.6f})"
                     if r["local_x_m"] != "" else "local=(대상 컴포넌트 아님)")
            lines.append(f"  joint={r['joint']} tag={r['tag']} {local}")

    return "\n".join(lines)


def run(context):
    ui = None
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("활성 문서가 Fusion 360 디자인이 아닙니다.")
            return

        ok, name_pattern = ui.inputBox(
            "joint 원점을 뽑을 대상 컴포넌트 이름(부분 문자열, 예: Crank)을 입력하세요.\n"
            "비워두거나 '*'/'all' 입력 시 디자인 전체의 모든 joint를 대상으로 합니다.",
            "Dump Joint Anchors", "")
        if not ok:
            return

        records, target_occs, n_joints = collect_records(design, name_pattern)

        if not records:
            ui.messageBox(
                f"'{name_pattern}' 을 이름에 포함하는 컴포넌트에 물린 joint를 "
                f"찾지 못했습니다.\n\n매칭된 occurrence 수: {len(target_occs)}\n"
                f"디자인 전체 joint 수: {n_joints}\n\n"
                "컴포넌트 이름이 정확한지, 그 부품에 joint가 실제로 연결돼 있는지 확인하세요."
            )
            return

        # 1차 출력: 항상 뜨는 messageBox (Text Commands 팔레트는 환경에 따라
        # 숨겨져 있거나 다른 창 뒤에 있어서 못 볼 수 있어 이걸 우선함)
        summary = _summary_text(records, name_pattern, target_occs, n_joints)
        ui.messageBox(summary, "Joint Anchor 결과 요약")

        # 참고용: Text Commands 팔레트에도 시도해서 띄워둠(실패해도 무시)
        try:
            text_palette = ui.palettes.itemById("TextCommands")
            if text_palette:
                text_palette.isVisible = True
                text_palette.writeText(summary)
        except Exception:
            pass

        save_dlg = ui.createFileDialog()
        save_dlg.title = "joint_anchors_report.csv 저장 위치 선택"
        save_dlg.filter = "CSV files (*.csv)"
        save_dlg.initialFilename = f"joint_anchors_{name_pattern.strip()}.csv"
        result = save_dlg.showSave()
        if result == adsk.core.DialogResults.DialogOK:
            write_csv(records, save_dlg.filename)
            ui.messageBox(f"CSV 저장 완료: {save_dlg.filename}\n레코드 {len(records)}개")
        else:
            ui.messageBox("저장 취소됨 — CSV 파일은 생성되지 않았습니다.\n"
                           "(위에서 본 요약 대화상자에 핵심 결과는 이미 표시됨)")

    except Exception:
        if ui:
            ui.messageBox(f"오류 발생:\n{traceback.format_exc()}")
