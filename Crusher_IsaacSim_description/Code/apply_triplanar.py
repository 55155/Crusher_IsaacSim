"""
Crusher_IsaacSim_base.usd 에 그룹별 OmniPBR Triplanar 재질 적용

- project_uvw = True  →  UV 없이 월드 공간 기준 Triplanar 투영
- 텍스처 소스: C:/TEMP/Crusher_IsaacSim_description/textures/ (PolyHaven CC0)
- 수정 파일: Crusher_IsaacSim_base.usd  (원본은 .triplanar.bak 으로 백업)

실행:
    python apply_triplanar.py
"""

import shutil
from pathlib import Path
from pxr import Usd, UsdShade, Sdf, Gf

BASE_USD = Path(r"C:\TEMP\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim\configuration\Crusher_IsaacSim_base.usd")
TEX_DIR  = Path(r"C:\TEMP\Crusher_IsaacSim_description\textures")
LOOKS    = "/Crusher_IsaacSim/Looks"

# ──────────────────────────────────────────────────────────────
# 텍스처 파일 맵  (PolyHaven 2K PNG)
# ──────────────────────────────────────────────────────────────
T = TEX_DIR
TEXTURES = {
    # 산업용 마모 강판 — diff / rough / metal / normal 모두 있음
    "metal_plate_02": {
        "diff":   T / "metal_plate_02_diff_2k.png",
        "rough":  T / "metal_plate_02_rough_2k.png",
        "metal":  T / "metal_plate_02_metal_2k.png",
        "normal": T / "metal_plate_02_nor_gl_2k.png",
    },
    # 도색 금속 (모터) — ARM 팩 맵: R=AO, G=Roughness, B=Metallic
    "rusty_painted_metal": {
        "diff":   T / "rusty_painted_metal_diff_2k.png",
        "orm":    T / "rusty_painted_metal_arm_2k.png",
        "normal": T / "rusty_painted_metal_nor_gl_2k.png",
    },
    # 마모된 강철 (크러셔) — diff / rough / normal
    "rusty_metal_02": {
        "diff":   T / "rusty_metal_02_diff_2k.png",
        "rough":  T / "rusty_metal_02_rough_2k.png",
        "normal": T / "rusty_metal_02_nor_gl_2k.png",
    },
}

# ──────────────────────────────────────────────────────────────
# 그룹별 재질 파라미터
#   texture_scale : Triplanar 타일 배율 (높을수록 작은 타일)
#   bump_factor   : 노말맵 강도
# ──────────────────────────────────────────────────────────────
GROUPS = {
    "frame": {
        "parts": {
            "base_link",
            "L1_Braket1_1", "L1_Wall1_1", "L1_Wall2_1",
            "L1_Guide1_1", "L1_Guide2_1", "L1_Slider_jig_1",
            "L2_Braket2_1", "L2_Left_Wall1_1", "L2_Linear_bush_1",
            "L2_Wall3_1", "L2_GearCase2_1", "L2_GearCase3_1",
        },
        "texture":   "metal_plate_02",
        "color":     Gf.Vec3f(0.55, 0.55, 0.58),
        "metallic":  0.90,
        "roughness": 0.35,
        "scale":     Gf.Vec2f(3.0, 3.0),
        "bump":      1.0,
    },
    "shaft": {
        "parts": {
            "L4_Shaft_1", "L4_Motor2_Shaft_1",
            "L5_Coupler_1", "L5_Key_1",
            "L6_DcutShaft_1", "L8_Link3_Shaft_1",
        },
        "texture":   "metal_plate_02",
        "color":     Gf.Vec3f(0.75, 0.75, 0.78),
        "metallic":  1.00,
        "roughness": 0.12,
        "scale":     Gf.Vec2f(6.0, 6.0),
        "bump":      0.5,
    },
    "gear": {
        "parts": {
            "L3_Bevel_GearBox_1", "L3_GearCase1_1", "L3_RackGear_1",
            "L4_Bevel_GearBox2_1", "L4_Reducer1_1", "L5_Reducer2_1",
            "L7_SpurGear_1",
        },
        "texture":   "metal_plate_02",
        "color":     Gf.Vec3f(0.40, 0.40, 0.42),
        "metallic":  0.95,
        "roughness": 0.25,
        "scale":     Gf.Vec2f(4.0, 4.0),
        "bump":      1.2,
    },
    "motor": {
        "parts": {
            "L3_Motor2_1", "L2__Motor2_Braket_1",
            "L6_Motor1_Bolt_1", "L6_Motor1_Braket_1", "L7_Motor1_Body_1",
        },
        "texture":   "rusty_painted_metal",
        "color":     Gf.Vec3f(0.20, 0.20, 0.22),
        "metallic":  0.70,
        "roughness": 0.60,
        "scale":     Gf.Vec2f(3.0, 3.0),
        "bump":      0.8,
    },
    "crusher": {
        "parts": {
            "L5_Link1_1", "L6_Link2_1", "L7_Link3_1", "L9_PLATE_v3_1",
        },
        "texture":   "rusty_metal_02",
        "color":     Gf.Vec3f(0.35, 0.33, 0.30),
        "metallic":  0.85,
        "roughness": 0.45,
        "scale":     Gf.Vec2f(4.0, 4.0),
        "bump":      1.5,
    },
    "bearing": {
        "parts": {
            "L4_Motor1_FrontBearing_1", "L4_Motor1_RearBearing_1",
            "L5_Motor1_FrontSnapring_1", "L5_Motor1_RearSnapring_1",
            "L7_Holder_Bearing1_1", "L7_Holder_Bearing2_1",
        },
        "texture":   "metal_plate_02",
        "color":     Gf.Vec3f(0.65, 0.63, 0.58),
        "metallic":  0.95,
        "roughness": 0.20,
        "scale":     Gf.Vec2f(5.0, 5.0),
        "bump":      0.6,
    },
}

# part_name → group_name 역방향 맵
PART_TO_GROUP = {
    part: gname
    for gname, ginfo in GROUPS.items()
    for part in ginfo["parts"]
}


def ap(path: Path) -> Sdf.AssetPath:
    return Sdf.AssetPath(path.as_posix())


def create_material(stage: Usd.Stage, gname: str, g: dict) -> UsdShade.Material:
    mat_path    = f"{LOOKS}/{gname}"
    shader_path = f"{mat_path}/Shader"

    if stage.GetPrimAtPath(mat_path).IsValid():
        stage.RemovePrim(mat_path)

    mat    = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, shader_path)

    shader.SetSourceAsset("OmniPBR.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)

    si = shader.CreateInput  # shorthand

    # 기본 물성 상수
    si("diffuse_color_constant",        Sdf.ValueTypeNames.Color3f).Set(g["color"])
    si("metallic_constant",             Sdf.ValueTypeNames.Float).Set(g["metallic"])
    si("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(g["roughness"])
    si("enable_opacity",                Sdf.ValueTypeNames.Bool).Set(False)

    # Triplanar 월드 공간 투영
    si("project_uvw",     Sdf.ValueTypeNames.Bool).Set(True)
    si("world_or_object", Sdf.ValueTypeNames.Bool).Set(False)  # False = world space
    si("texture_scale",   Sdf.ValueTypeNames.Float2).Set(g["scale"])

    tex = TEXTURES[g["texture"]]

    # Diffuse / Albedo 텍스처
    if "diff" in tex and tex["diff"].exists():
        si("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(ap(tex["diff"]))

    # Normal map
    if "normal" in tex and tex["normal"].exists():
        si("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(ap(tex["normal"]))
        si("bump_factor",       Sdf.ValueTypeNames.Float).Set(g["bump"])

    # Roughness / Metallic: ORM 팩 또는 개별 맵
    if "orm" in tex and tex["orm"].exists():
        # ARM 맵 = OmniPBR ORM (R=AO, G=Roughness, B=Metallic)
        si("enable_ORM_texture", Sdf.ValueTypeNames.Bool).Set(True)
        si("ORM_texture",        Sdf.ValueTypeNames.Asset).Set(ap(tex["orm"]))
    else:
        if "rough" in tex and tex["rough"].exists():
            si("reflectionroughness_texture", Sdf.ValueTypeNames.Asset).Set(ap(tex["rough"]))
        if "metal" in tex and tex["metal"].exists():
            si("metallic_texture", Sdf.ValueTypeNames.Asset).Set(ap(tex["metal"]))

    # MDL surface / displacement / volume 출력 연결
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    api = shader.ConnectableAPI()
    mat.CreateSurfaceOutput("mdl").ConnectToSource(api, "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(api, "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(api, "out")

    return mat


def main():
    # 백업
    bak = BASE_USD.with_suffix(".triplanar.bak")
    if not bak.exists():
        shutil.copy2(BASE_USD, bak)
        print(f"백업 생성: {bak.name}")
    else:
        print(f"백업 존재: {bak.name}")

    # 텍스처 파일 존재 확인
    print("\n[텍스처 파일 확인]")
    missing = 0
    for tname, tmaps in TEXTURES.items():
        for mtype, fpath in tmaps.items():
            ok = fpath.exists()
            print(f"  {'✓' if ok else '✗'} {fpath.name}")
            if not ok:
                missing += 1
    if missing:
        print(f"\n  ※ {missing}개 파일 없음 → download_textures.bat 먼저 실행하세요")
        print("     (파일 없는 채널은 상수값으로 대체됩니다)\n")

    # USD 열기
    print(f"\n[USD 열기] {BASE_USD.name}")
    stage = Usd.Stage.Open(str(BASE_USD))

    # Looks scope 보장
    if not stage.GetPrimAtPath(LOOKS).IsValid():
        stage.DefinePrim(LOOKS, "Scope")

    # 기존 BlackMetal 제거
    old = stage.GetPrimAtPath(f"{LOOKS}/BlackMetal")
    if old.IsValid():
        stage.RemovePrim(f"{LOOKS}/BlackMetal")
        print("기존 BlackMetal 제거")

    # 재질 생성
    print("\n[재질 생성]")
    materials = {}
    for gname, ginfo in GROUPS.items():
        mat = create_material(stage, gname, ginfo)
        materials[gname] = mat
        print(f"  {gname:10s} — tex={ginfo['texture']}, metallic={ginfo['metallic']}, rough={ginfo['roughness']}")

    # visuals Xform 에 재질 바인딩
    print("\n[바인딩]")
    bound     = {g: 0 for g in GROUPS}
    unmatched = []

    for prim in stage.Traverse():
        if prim.GetName() != "visuals" or prim.GetTypeName() != "Xform":
            continue

        link_name  = prim.GetParent().GetName()
        group_name = PART_TO_GROUP.get(link_name)

        if group_name is None:
            unmatched.append(link_name)
            group_name = "frame"  # 미분류 → frame으로 폴백

        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            materials[group_name],
            UsdShade.Tokens.strongerThanDescendants,
        )
        bound[group_name] += 1

    for gname, cnt in bound.items():
        print(f"  {gname:10s}: {cnt}개 visuals 바인딩")
    if unmatched:
        print(f"\n  미분류 링크 (frame으로 처리): {unmatched}")

    stage.Save()
    print(f"\n[완료] {BASE_USD.name} 저장")
    print("Isaac Sim에서 파일을 다시 열어 확인하세요.")


if __name__ == "__main__":
    main()
