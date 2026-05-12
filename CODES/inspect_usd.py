from pxr import Usd, Sdf

path = 'C:/TEMP/Crusher_IsaacSim_description/urdf/Crusher_IsaacSim/Crusher_IsaacSim.usd'
stage = Usd.Stage.Open(path)
rl = stage.GetRootLayer()

print("=== Root Layer ===")
print("  identifier :", rl.identifier)
print("  subLayers  :", rl.subLayerPaths)
print("  extRefs    :", rl.GetExternalReferences())

print()
print("=== All Layers in Stack ===")
for layer in stage.GetLayerStack(includeSessionLayers=False):
    name = layer.identifier.split("/")[-1]
    print(f"  {name}")
    print(f"    subLayers : {layer.subLayerPaths}")
    ext = layer.GetExternalReferences()
    if ext:
        print(f"    extRefs   : {ext[:5]}")

print()
print("=== Prim References (first 10 prims with refs) ===")
count = 0
for prim in stage.TraverseAll():
    stack = prim.GetPrimStack()
    for spec in stack:
        refs = spec.referenceList
        if refs.prependedItems or refs.explicitItems or refs.appendedItems:
            items = refs.prependedItems or refs.explicitItems or refs.appendedItems
            for item in items:
                print(f"  {prim.GetPath()} -> {item.assetPath}")
                count += 1
                if count >= 10:
                    break
    if count >= 10:
        break

print()
print("=== Payload ===")
for prim in stage.TraverseAll():
    if prim.HasPayload():
        print(f"  {prim.GetPath()} has payload")
        break
