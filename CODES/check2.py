import xml.etree.ElementTree as ET

tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()

targets = ["8_Link3_Shaft_1", "3_RackGear_1"]
for link in root.findall("link"):
    if link.attrib["name"] in targets:
        inertia = link.find(".//inertia")
        print(link.attrib["name"], dict(inertia.attrib))
