import os, xml.etree.ElementTree as ET
os.environ["PATH"] = r"C:\Anaconda3\envs\isaaclab\Library\bin;" + os.environ.get("PATH","")
import graphviz

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()
robot_name = root.attrib.get("name", "robot")

dot = graphviz.Digraph(comment=robot_name, format="pdf")
dot.attr(rankdir="TB", dpi="150")
dot.attr("node", fontsize="10")
dot.attr("edge", fontsize="9")

joint_types = {"fixed":"gray", "continuous":"green", "revolute":"blue", "prismatic":"red"}

for link in root.findall("link"):
    dot.node(link.attrib["name"], link.attrib["name"], shape="box", style="filled", fillcolor="lightblue")
for joint in root.findall("joint"):
    jname = joint.attrib["name"]
    jtype = joint.attrib["type"]
    parent = joint.find("parent").attrib["link"]
    child  = joint.find("child").attrib["link"]
    color  = joint_types.get(jtype, "black")
    dot.edge(parent, child, label=f"{jname}\n({jtype})", color=color, fontcolor=color)

out = r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim_graph"
dot.render(out, view=True)
print("saved:", out + ".pdf")
