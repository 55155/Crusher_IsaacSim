import bpy
op = bpy.ops.wm.usd_export
print("=== usd_export properties ===")
try:
    rna = bpy.ops.wm.usd_export.get_rna_type()
    for prop in rna.properties:
        if prop.identifier != 'rna_type':
            print(f"  {prop.identifier}: {prop.type}")
except Exception as e:
    print(f"get_rna_type failed: {e}")
    # fallback: just try calling with minimal args to see what's valid
