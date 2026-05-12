import xml.etree.ElementTree as ET
from collections import Counter
tree = ET.parse(r"C:\Temp\Crusher_IsaacSim_description\urdf\Crusher_IsaacSim.urdf")
root = tree.getroot()
names = [l.attrib["name"] for l in root.findall("link")]
dupes = [n for n,c in Counter(names).items() if c > 1]
print("?? link:", dupes)
