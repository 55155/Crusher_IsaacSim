"""
Crusher_IsaacSim 전체 링크에 검정 금속 MDL 재료를 적용합니다.
Isaac Sim RTX 렌더러는 UsdPreviewSurface를 무시하므로 MDL OmniPBR을 사용합니다.

- 재료: 검정(0,0,0), 거칠기 0.5, 금속성 1.0, 불투명
- 적용 대상: 모든 */visuals Xform (자식 Mesh까지 상속됨)
- 수정 파일: Crusher_IsaacSim_base.usd
"""

import shutil
from pathlib import Path
from pxr import Usd, UsdShade, Sdf, Gf

BASE_USD = Path(__file__).parent / "configuration" / "Crusher_IsaacSim_base.usd"

MATERIAL_PATH = "/Crusher_IsaacSim/Looks/BlackMetal"

COLOR     = Gf.Vec3f(0.0, 0.0, 0.0)
ROUGHNESS = 0.5
METALLIC  = 1.0


def backup(path: Path):
    bak = path.with_suffix(".usd.bak")
    if not bak.exists():
        shutil.copy2(path, bak)
        print(f"백업 생성: {bak.name}")
    else:
        print(f"백업 이미 존재: {bak.name} (덮어쓰지 않음)")


def create_material(stage: Usd.Stage) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)

    shader = UsdShade.Shader.Define(stage, MATERIAL_PATH + "/Shader")

    # MDL OmniPBR — Isaac Sim RTX 렌더러가 인식하는 재료
    shader.SetSourceAsset("OmniPBR.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)

    shader.CreateInput("diffuse_color_constant",           Sdf.ValueTypeNames.Color3f).Set(COLOR)
    shader.CreateInput("reflection_roughness_constant",    Sdf.ValueTypeNames.Float).Set(ROUGHNESS)
    shader.CreateInput("metallic_constant",                Sdf.ValueTypeNames.Float).Set(METALLIC)
    shader.CreateInput("enable_opacity",                   Sdf.ValueTypeNames.Bool).Set(False)

    # MDL 출력 연결 (surface / displacement / volume 모두 연결해야 Isaac Sim이 인식)
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    api = shader.ConnectableAPI()
    material.CreateSurfaceOutput("mdl").ConnectToSource(api, "out")
    material.CreateDisplacementOutput("mdl").ConnectToSource(api, "out")
    material.CreateVolumeOutput("mdl").ConnectToSource(api, "out")

    print(f"재료 생성 (MDL OmniPBR): {MATERIAL_PATH}")
    print(f"  색상={tuple(COLOR)}, 거칠기={ROUGHNESS}, 금속성={METALLIC}")
    return material


def bind_all_visuals(stage: Usd.Stage, material: UsdShade.Material) -> int:
    count = 0
    for prim in stage.Traverse():
        # */visuals Xform에만 바인딩 (collisions 제외)
        if prim.GetName() == "visuals" and prim.GetTypeName() == "Xform":
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material,
                UsdShade.Tokens.weakerThanDescendants,  # 자식에 개별 재료 있으면 우선
            )
            count += 1
    print(f"바인딩 완료: {count}개 visuals")
    return count


def main():
    backup(BASE_USD)

    print(f"\nUSD 열기: {BASE_USD.name}")
    stage = Usd.Stage.Open(str(BASE_USD))

    # 기존 재료 제거 후 재생성
    existing = stage.GetPrimAtPath(MATERIAL_PATH)
    if existing.IsValid():
        stage.RemovePrim(MATERIAL_PATH)
        print("기존 BlackMetal 재료 제거")

    material = create_material(stage)
    bind_all_visuals(stage, material)

    stage.Save()
    print(f"\n저장 완료: {BASE_USD.name}")
    print("Isaac Sim에서 파일을 다시 열어 확인하세요.")


if __name__ == "__main__":
    main()
