import base64, os
h = os.path.dirname(os.path.abspath(__file__))
b = base64.b64encode(open(os.path.join(h, "glass_wall.glb"), "rb").read()).decode()
t = open(os.path.join(h, "template.html"), encoding="utf-8").read().replace("__GLB__", b)
open(os.path.join(h, "glass_wall_design.html"), "w", encoding="utf-8").write(t)
print(round(len(t) / 1e6, 2), "MB")
