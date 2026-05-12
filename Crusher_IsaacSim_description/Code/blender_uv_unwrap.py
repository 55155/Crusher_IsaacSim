"""
Blender 5.1 headless batch script
- Imports each STL from urdf\ root dir
- Applies UV Smart Project
- Assigns Principled BSDF material by functional group
- Exports as USD (with UV maps and UsdPreviewSurface materials)
to meshes_usd\

Usage:
    blender.exe --background --python blender_uv_unwrap.py
"""
import bpy
import os
import sys

STL_DIR = r"C:\TEMP\Crusher_IsaacSim_description\urdf"
OUT_DIR = r"C:\TEMP\Crusher_IsaacSim_description\meshes_usd"
os.makedirs(OUT_DIR, exist_ok=True)

# ---------- Material group definitions ----------
# color: linear (not sRGB) Principled BSDF base color
MATERIAL_GROUPS = {
    "frame": {
        "parts": {
            "base_link",
            "L1_Braket1_1", "L1_Wall1_1", "L1_Wall2_1",
            "L1_Guide1_1", "L1_Guide2_1", "L1_Slider_jig_1",
            "L2_Braket2_1", "L2_Left_Wall1_1", "L2_Linear_bush_1",
            "L2_Wall3_1", "L2_GearCase2_1", "L2_GearCase3_1",
        },
        "color": (0.55, 0.55, 0.58),
        "metallic": 0.9,
        "roughness": 0.35,
    },
    "gear": {
        "parts": {
            "L3_Bevel_GearBox_1", "L3_GearCase1_1", "L3_RackGear_1",
            "L4_Bevel_GearBox2_1", "L4_Reducer1_1", "L5_Reducer2_1",
            "L7_SpurGear_1",
        },
        "color": (0.40, 0.40, 0.42),
        "metallic": 0.95,
        "roughness": 0.25,
    },
    "motor": {
        "parts": {
            "L3_Motor2_1", "L2__Motor2_Braket_1",
            "L6_Motor1_Bolt_1", "L6_Motor1_Braket_1", "L7_Motor1_Body_1",
        },
        "color": (0.20, 0.20, 0.22),
        "metallic": 0.70,
        "roughness": 0.60,
    },
    "shaft": {
        "parts": {
            "L4_Shaft_1", "L4_Motor2_Shaft_1",
            "L5_Coupler_1", "L5_Key_1",
            "L6_DcutShaft_1", "L8_Link3_Shaft_1",
        },
        "color": (0.75, 0.75, 0.78),
        "metallic": 1.00,
        "roughness": 0.12,
    },
    "crusher": {
        "parts": {
            "L5_Link1_1", "L6_Link2_1", "L7_Link3_1", "L9_PLATE_v3_1",
        },
        "color": (0.35, 0.33, 0.30),
        "metallic": 0.85,
        "roughness": 0.45,
    },
    "bearing": {
        "parts": {
            "L4_Motor1_FrontBearing_1", "L4_Motor1_RearBearing_1",
            "L5_Motor1_FrontSnapring_1", "L5_Motor1_RearSnapring_1",
            "L7_Holder_Bearing1_1", "L7_Holder_Bearing2_1",
        },
        "color": (0.65, 0.63, 0.58),
        "metallic": 0.95,
        "roughness": 0.20,
    },
}

# Build part_name -> (group_name, info) lookup
PART_TO_GROUP = {}
for _gname, _ginfo in MATERIAL_GROUPS.items():
    for _part in _ginfo["parts"]:
        PART_TO_GROUP[_part] = (_gname, _ginfo)


# ---------- Helpers ----------

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for blk in (bpy.data.materials, bpy.data.meshes, bpy.data.images):
        for item in list(blk):
            blk.remove(item)


def apply_uv_smart_project(obj):
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

    # Ensure at least one UV layer exists before entering edit mode
    if not obj.data.uv_layers:
        obj.data.uv_layers.new(name="UVMap")

    try:
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.uv.smart_project(
            angle_limit=66.0,
            margin_method='SCALED',
            rotate_method='AXIS_ALIGNED_Y',
            scale_to_bounds=False,
            correct_aspect=True,
        )
        bpy.ops.object.mode_set(mode='OBJECT')
        return "smart_project"
    except RuntimeError as e:
        # Fallback: cube projection (also works without viewport in most versions)
        try:
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception:
            pass
        try:
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.uv.cube_project(scale_to_bounds=True)
            bpy.ops.object.mode_set(mode='OBJECT')
            return f"cube_project (smart_project failed: {e})"
        except RuntimeError as e2:
            try:
                bpy.ops.object.mode_set(mode='OBJECT')
            except Exception:
                pass
            return f"none (all UV ops failed: {e2})"


def create_material(part_name):
    if part_name in PART_TO_GROUP:
        group_name, info = PART_TO_GROUP[part_name]
    else:
        group_name = "unknown"
        info = {"color": (0.50, 0.50, 0.50), "metallic": 0.80, "roughness": 0.40}
        print(f"    WARNING: '{part_name}' not in any material group — using default gray")

    mat = bpy.data.materials.new(name=f"{group_name}_{part_name}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()

    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    out = nodes.new(type='ShaderNodeOutputMaterial')
    mat.node_tree.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])

    r, g, b = info["color"]
    bsdf.inputs['Base Color'].default_value = (r, g, b, 1.0)
    bsdf.inputs['Metallic'].default_value = info["metallic"]
    bsdf.inputs['Roughness'].default_value = info["roughness"]

    # Specular IOR level (Blender 4.x renamed 'Specular' input)
    for name in ('Specular IOR Level', 'Specular'):
        if name in bsdf.inputs:
            bsdf.inputs[name].default_value = 0.5
            break

    return mat


def process_stl(stl_path):
    part_name = os.path.splitext(os.path.basename(stl_path))[0]
    out_path = os.path.join(OUT_DIR, part_name + ".usd")

    print(f"\n[{part_name}]")
    clear_scene()

    # Import STL (Blender 4.0+ built-in importer)
    try:
        bpy.ops.wm.stl_import(filepath=stl_path)
    except AttributeError:
        # Older Blender fallback
        bpy.ops.import_mesh.stl(filepath=stl_path)

    objs = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    if not objs:
        print(f"  ERROR: no mesh object after import, skipping")
        return

    obj = objs[0]
    obj.name = part_name
    obj.data.name = part_name

    # UV unwrap
    uv_result = apply_uv_smart_project(obj)
    print(f"  UV: {uv_result}")

    # Material
    mat = create_material(part_name)
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)

    # USD export
    try:
        bpy.ops.wm.usd_export(
            filepath=out_path,
            selected_objects_only=False,
            export_uvmaps=True,
            export_normals=True,
            export_materials=True,
            generate_preview_surface=True,
            export_textures_mode='NONE',
            relative_paths=False,
        )
        print(f"  -> {out_path}")
    except Exception as e:
        print(f"  USD export ERROR: {e}")


# ---------- Main ----------

# Only process STL files directly in STL_DIR (not meshes/ subdir)
stl_files = sorted(
    os.path.join(STL_DIR, f)
    for f in os.listdir(STL_DIR)
    if f.lower().endswith('.stl') and os.path.isfile(os.path.join(STL_DIR, f))
)

print(f"Found {len(stl_files)} STL files in {STL_DIR}")
print(f"Output directory: {OUT_DIR}\n")

ok = 0
fail = 0
for stl_path in stl_files:
    try:
        process_stl(stl_path)
        ok += 1
    except Exception as exc:
        print(f"  FATAL ERROR for {stl_path}: {exc}")
        import traceback
        traceback.print_exc()
        fail += 1

print(f"\n{'='*50}")
print(f"Complete: {ok} OK, {fail} failed")
print(f"USD files written to: {OUT_DIR}")
