"""
extract_appearance.py  -  Fusion 360 Script
===========================================
Fusion 360 의 모든 Component/Body Appearance 를 추출해서 JSON 으로 저장.

실행 방법:
  Fusion 360 → Tools → Scripts and Add-Ins → Scripts → [+] → 이 파일 선택 → Run
"""

import adsk.core
import adsk.fusion
import json
import traceback
from pathlib import Path

OUT_PATH = r"C:\Crusher_isaacsim\MuJoCo_PlayGround\20260518\fusion360\appearances.json"


def get_color_property(props):
    """Appearance properties 에서 diffuse color(RGBA) 추출."""
    for i in range(props.count):
        prop = props.item(i)
        if isinstance(prop, adsk.core.ColorProperty):
            if any(k in prop.name.lower() for k in ["color", "diffuse", "albedo"]):
                c = prop.value
                return [
                    round(c.red   / 255, 4),
                    round(c.green / 255, 4),
                    round(c.blue  / 255, 4),
                    round(c.opacity / 255, 4),
                ]
    return None


def get_float_property(props, keywords):
    """Appearance properties 에서 float 속성 추출."""
    for i in range(props.count):
        prop = props.item(i)
        if isinstance(prop, adsk.core.FloatProperty):
            if any(k in prop.name.lower() for k in keywords):
                return round(prop.value, 4)
    return None


def extract_body_appearance(body):
    """BRepBody 의 Appearance 정보 딕셔너리로 반환."""
    app = body.appearance
    if not app:
        return None

    props = app.appearanceProperties
    info  = {"appearance_name": app.name}

    color = get_color_property(props)
    if color:
        info["rgba"] = color

    roughness = get_float_property(props, ["roughness"])
    if roughness is not None:
        info["roughness"] = roughness

    metallic = get_float_property(props, ["metallic", "metal"])
    if metallic is not None:
        info["metallic"] = metallic

    opacity = get_float_property(props, ["opacity", "transparent"])
    if opacity is not None:
        info["opacity"] = opacity

    return info


def run(context):
    ui = None
    try:
        app    = adsk.core.Application.get()
        ui     = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)

        results = {}  # component_name → {body_name → appearance_info}

        for comp in design.allComponents:
            comp_name = comp.name
            comp_data = {}

            for body in comp.bRepBodies:
                info = extract_body_appearance(body)
                if info:
                    comp_data[body.name] = info

            if comp_data:
                results[comp_name] = comp_data

        # JSON 저장
        Path(OUT_PATH).parent.mkdir(parents=True, exist_ok=True)
        with open(OUT_PATH, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)

        count = sum(len(v) for v in results.values())
        ui.messageBox(
            f"추출 완료\n"
            f"  Component: {len(results)}개\n"
            f"  Body: {count}개\n"
            f"  저장: {OUT_PATH}"
        )

    except Exception:
        if ui:
            ui.messageBox(f"오류:\n{traceback.format_exc()}")
