import omni.usd
from pxr import UsdShade, Sdf, Gf

# ── 설정 ──────────────────────────────────────────────────────
MATERIAL_PATH = "/Crusher_IsaacSim/Looks/BlackMetal"
COLOR         = Gf.Vec3f(0.0, 0.0, 0.0)   # 검정
ROUGHNESS     = 0.5
METALLIC      = 1.0

LINKS = [
    "fixed_base", "base_link",
    "L1_Braket1_1", "L2_Braket2_1",
    "L3_Bevel_GearBox_1", "L4_Bevel_GearBox2_1",
    "L4_Motor1_FrontBearing_1", "L5_Motor1_FrontSnapring_1",
    "L4_Motor1_RearBearing_1", "L5_Motor1_RearSnapring_1",
    "L4_Reducer1_1", "L5_Reducer2_1",
    "L6_Motor1_Bolt_1", "L6_Motor1_Braket_1", "L7_Motor1_Body_1",
    "L4_Shaft_1", "L5_Key_1",
    "L5_Link1_1", "L6_Link2_1", "L7_Link3_1",
    "L8_Link3_Shaft_1", "L9_PLATE_v3_1",
    "L1_Guide1_1", "L2_Left_Wall1_1", "L3_RackGear_1",
    "L1_Guide2_1", "L1_Slider_jig_1", "L2_Linear_bush_1",
    "L1_Wall1_1", "L2_GearCase2_1", "L3_GearCase1_1",
    "L2_GearCase3_1", "L2__Motor2_Braket_1",
    "L3_Motor2_1", "L4_Motor2_Shaft_1",
    "L5_Coupler_1", "L6_DcutShaft_1",
    "L7_Holder_Bearing1_1", "L7_Holder_Bearing2_1", "L7_SpurGear_1",
    "L1_Wall2_1", "L2_Wall3_1",
]
# ─────────────────────────────────────────────────────────────

stage = omni.usd.get_context().get_stage()

# 기존 재료 제거 후 재생성
existing = stage.GetPrimAtPath(MATERIAL_PATH)
if existing.IsValid():
    stage.RemovePrim(MATERIAL_PATH)

# MDL OmniPBR 재료 생성
material = UsdShade.Material.Define(stage, MATERIAL_PATH)
shader   = UsdShade.Shader.Define(stage, MATERIAL_PATH + "/Shader")

shader.SetSourceAsset("OmniPBR.mdl", "mdl")
shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)

shader.CreateInput("diffuse_color_constant",        Sdf.ValueTypeNames.Color3f).Set(COLOR)
shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(ROUGHNESS)
shader.CreateInput("metallic_constant",             Sdf.ValueTypeNames.Float).Set(METALLIC)
shader.CreateInput("enable_opacity",                Sdf.ValueTypeNames.Bool).Set(False)

api = shader.ConnectableAPI()
material.CreateSurfaceOutput("mdl").ConnectToSource(api, "out")
material.CreateDisplacementOutput("mdl").ConnectToSource(api, "out")
material.CreateVolumeOutput("mdl").ConnectToSource(api, "out")

print(f"재료 생성 완료: {MATERIAL_PATH}")

# 전체 링크 visuals에 바인딩
ok, fail = [], []
for link in LINKS:
    path = f"/Crusher_IsaacSim/{link}/visuals"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        ok.append(link)
    else:
        fail.append(link)

print(f"바인딩 성공: {len(ok)}개")
if fail:
    print(f"바인딩 실패 (prim 없음): {fail}")

print("완료 — 뷰포트에서 확인하세요.")
