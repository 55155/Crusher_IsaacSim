# -*- coding: utf-8 -*-
"""Fusion 360 script — 활성 디자인 전체를 지정한 배율(k)로 축척(scale)한다.

실행 방법 (Fusion 360 안에서):
  Utilities > ADD-INS > Scripts and Add-Ins > Scripts 탭 > '+' 버튼으로
  이 utills/scale_design 폴더(폴더명과 파일명이 같아야 Fusion이 Script로
  인식함 — utills 폴더 자체를 추가하면 안 됨)를 선택 > 목록에서
  'scale_design' 선택 > Run.

실제 스케일 로직은 scale_design_core.py 에 있고, 이 파일은 배율/모드를 물어보는
UI 껍데기다. 모드([G] 지오메트리 / [P] 파라미터)와 각 모드가 무엇을 어떻게
바꾸는지는 scale_design_core.py 의 모듈 docstring 참고.

매번 Scripts 탭에서 폴더를 추가하기 싫고 항상 떠 있는 툴바 버튼이 필요하면
Crusher_Genesis/fusion_addin/scale_design/ 의 Add-In 버전을 쓴다 — 그쪽은
배율/모드/옵션을 한 다이얼로그에서 받고, 배율을 바꾸면 예상 크기가 바로
갱신된다.
"""

import importlib.util
import os
import sys
import traceback

import adsk.core
import adsk.fusion

_HERE = os.path.dirname(os.path.abspath(__file__))
_CORE_PATH = os.path.join(_HERE, "scale_design_core.py")

core = None   # load_core() 가 run() 에서 채운다


def load_core():
    """옆의 scale_design_core.py 를 매번 파일에서 새로 읽는다.

    'from scale_design_core import ...' 로 쓰면 Fusion 세션 동안 sys.modules 에
    캐시돼서, 파일을 고쳐도 Fusion 을 껐다 켜기 전까지 예전 코드가 계속 돈다
    (2026-08-06: 코어를 고쳤는데도 같은 증상이 반복되던 원인). Add-In 쪽과
    같은 방식으로 파일 경로에서 직접 실행해 항상 최신 코드를 쓴다.
    """
    spec = importlib.util.spec_from_file_location("scale_design_core_script", _CORE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def prompt_text(ui, prompt, title, default):
    """inputBox 는 (입력문자열, 취소여부) 를 돌려준다 — 순서를 헷갈리지 않게 감싼다.

    취소하면 None.
    """
    text, cancelled = ui.inputBox(prompt, title, default)
    return None if cancelled else text


def ask_factor(ui):
    text = prompt_text(
        ui,
        "현재 크기의 몇 배로 축척할지 배율을 입력하세요.\n"
        "예) 2 = 2배로 확대, 0.5 = 절반으로 축소",
        "Scale Design — 배율", "2.0")
    if text is None:
        return None
    try:
        factor = float(text.strip())
    except ValueError:
        ui.messageBox("숫자를 입력해야 합니다: '{0}'".format(text))
        return None
    if factor <= 0.0:
        ui.messageBox("배율은 0보다 커야 합니다: {0}".format(factor))
        return None
    if factor == 1.0:
        ui.messageBox("배율 1.0 은 아무것도 바꾸지 않습니다.")
        return None
    return factor


def ask_mode(ui):
    text = prompt_text(
        ui,
        "스케일 방식을 고르세요.\n\n"
        "  G = 지오메트리 스케일 (기본)\n"
        "      모든 컴포넌트의 바디를 Scale feature 로 키우고,\n"
        "      조인트 offset/이동 한계와 부품 배치까지 같이 배율 적용.\n\n"
        "  P = 파라미터 스케일\n"
        "      사용자 파라미터 중 길이 상수만 배율 적용.\n"
        "      형상이 파라미터로 완전히 구동되는 디자인에서만 유효.",
        "Scale Design — 방식", "G")
    if text is None:
        return None
    mode = text.strip().upper()[:1]
    if mode not in (core.MODE_GEOMETRY, core.MODE_PARAMETER):
        ui.messageBox("G 또는 P 만 입력할 수 있습니다: '{0}'".format(text))
        return None
    return mode


def ask_export_path(ui, design, factor, kind):
    """kind('step'/'stl') 로 내보낼지 묻고, 예이면 저장 경로를 받는다(아니면 None)."""
    answer = ui.messageBox(
        "스케일 적용 후 결과를 {0} 파일로 내보낼까요?".format(kind.upper()),
        "Scale Design — {0} 내보내기".format(kind.upper()),
        adsk.core.MessageBoxButtonTypes.YesNoButtonType,
        adsk.core.MessageBoxIconTypes.QuestionIconType)
    if answer != adsk.core.DialogResults.DialogYes:
        return None

    base = design.rootComponent.name.split()[0] if design.rootComponent.name else "design"
    extensions = (".step", ".stp") if kind == "step" else (".stl",)

    dlg = ui.createFileDialog()
    dlg.title = "스케일 결과를 저장할 {0} 파일 경로".format(kind.upper())
    dlg.filter = ("STEP files (*.step);;STEP files (*.stp)" if kind == "step"
                  else "STL files (*.stl)")
    dlg.initialFilename = "{0}_x{1:g}{2}".format(base, factor, extensions[0])
    if dlg.showSave() != adsk.core.DialogResults.DialogOK:
        return None

    path = dlg.filename
    if not path.lower().endswith(extensions):
        path += extensions[0]
    return path


def run(context):
    global core
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        core = load_core()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("활성 문서가 Fusion 360 디자인이 아닙니다.\n"
                          "Design 워크스페이스에서 실행해주세요.")
            return

        factor = ask_factor(ui)
        if factor is None:
            return

        mode = ask_mode(ui)
        if mode is None:
            return

        root_comp = design.rootComponent

        # 외부 링크 부품은 스케일이 안 되므로 실행 전에 경고한다 — 링크 부품만
        # 원본 크기로 남으면 결과가 통째로 어긋난다.
        linked = sorted(core.referenced_component_names(root_comp))
        linked_warning = ""
        if linked:
            linked_warning = (
                "\n!! 외부 링크 컴포넌트 {0}개는 스케일할 수 없습니다:\n   {1}\n"
                "   Break Link 후 실행해야 전체가 같은 배율이 됩니다.\n"
            ).format(len(linked), ", ".join(linked[:5]))

        if mode == core.MODE_GEOMETRY:
            before = core.design_bounding_box(root_comp)
            confirm = (
                "'{0}' 디자인 전체를 {1:g}배로 축척합니다.\n\n"
                "현재 크기: {2}\n"
                "예상 크기: {3}\n\n"
                "컴포넌트 {4}개의 바디가 각 컴포넌트 원점 기준으로 스케일되고,\n"
                "조인트 offset/이동 한계도 함께 {1:g}배 됩니다.\n"
                "{5}\n"
                "진행할까요?"
            ).format(
                root_comp.name, factor, core.format_bbox(before),
                core.format_bbox(before, factor), design.allComponents.count,
                linked_warning)
        else:
            confirm = (
                "'{0}' 디자인의 길이 사용자 파라미터를 {1:g}배 합니다.\n"
                "(사용자 파라미터 {2}개 검사 예정)\n{3}\n진행할까요?"
            ).format(root_comp.name, factor, design.userParameters.count,
                     linked_warning)

        answer = ui.messageBox(confirm, "Scale Design 확인",
                               adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                               adsk.core.MessageBoxIconTypes.QuestionIconType)
        if answer != adsk.core.DialogResults.DialogYes:
            ui.messageBox("취소되었습니다 — 디자인은 변경되지 않았습니다.")
            return

        # 저장 경로는 디자인을 건드리기 전에 물어본다 — 스케일을 다 해놓고
        # 저장 다이얼로그에서 취소하면 헛수고가 되므로.
        step_path = ask_export_path(ui, design, factor, "step")
        stl_path = ask_export_path(ui, design, factor, "stl")

        _report, summary = core.apply_scale(design, factor, mode,
                                            step_path=step_path, stl_path=stl_path)
        ui.messageBox(summary, "Scale Design 결과")

        # 참고용: Text Commands 팔레트에도 남겨둔다(실패해도 무시).
        try:
            palette = ui.palettes.itemById("TextCommands")
            if palette:
                palette.isVisible = True
                palette.writeText(summary)
        except Exception:
            pass

    except Exception:
        if ui:
            ui.messageBox("오류 발생:\n{0}".format(traceback.format_exc()))
