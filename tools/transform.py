import xml.etree.ElementTree as ET
import math

def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]

def transform_link(link):
    inertial = link.find("inertial")
    if inertial is None:
        return
    origin = inertial.find("origin")
    inertia = inertial.find("inertia")
    if origin is None or inertia is None:
        return

    rpy_str = origin.get("rpy", "0 0 0")
    r, p, y = [float(x) for x in rpy_str.split()]
    if abs(r) < 1e-6 and abs(p) < 1e-6 and abs(y) < 1e-6:
        return

    # 1. 保持原有的 xyz 完全不变；仅将 rpy 置零
    origin.set("rpy", "0 0 0")

    # 2. 仅对惯性张量做旋转变换: I_new = R * I * R^T
    R = rpy_to_matrix(r, p, y)
    ixx = float(inertia.get("ixx", 0))
    ixy = float(inertia.get("ixy", 0))
    ixz = float(inertia.get("ixz", 0))
    iyy = float(inertia.get("iyy", 0))
    iyz = float(inertia.get("iyz", 0))
    izz = float(inertia.get("izz", 0))
    I = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]

    RI = [[sum(R[i][k] * I[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    I_new = [[sum(RI[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]

    inertia.set("ixx", f"{I_new[0][0]:.8f}")
    inertia.set("ixy", f"{I_new[0][1]:.8f}")
    inertia.set("ixz", f"{I_new[0][2]:.8f}")
    inertia.set("iyy", f"{I_new[1][1]:.8f}")
    inertia.set("iyz", f"{I_new[1][2]:.8f}")
    inertia.set("izz", f"{I_new[2][2]:.8f}")

tree = ET.parse("ridgeback_ur5/urdf/ridgeback_ur5.urdf")
root = tree.getroot()
for link in root.findall("link"):
    transform_link(link)

tree.write("ridgeback_ur5.urdf", encoding="utf-8", xml_declaration=True)
print("转换完成，已生成 urdf")