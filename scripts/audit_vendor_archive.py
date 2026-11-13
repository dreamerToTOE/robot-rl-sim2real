"""Read-only CPU audit of the expected student_ros model; requires existing NumPy."""
import argparse
import hashlib
import json
import struct
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="Path to the private student_ros.zip; never extracted")
    args = parser.parse_args()
    before = digest(args.archive.read_bytes())
    prefix = "student_ros/src/episode1_urdf_1113/"
    summary = {"archive_sha256": before, "inertials": [], "meshes": []}
    with zipfile.ZipFile(args.archive) as package:
        urdf_data = package.read(prefix + "urdf/episode1_urdf_1113.urdf")
        root = ET.fromstring(urdf_data)
        summary["urdf_sha256"] = digest(urdf_data)
        sdf_data = package.read(prefix + "urdf/robot.sdf")
        summary["sdf_sha256"] = digest(sdf_data)
        sdf_model = ET.fromstring(sdf_data).find("model")
        for link in root.findall("link"):
            name = link.attrib["name"]
            inertial = link.find("inertial")
            values = inertial.find("inertia").attrib
            matrix = np.array([[float(values[k]) for k in row] for row in
                               (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
            eig = np.linalg.eigvalsh(matrix)
            mass = float(inertial.find("mass").attrib["value"])
            com = [float(v) for v in inertial.find("origin").attrib["xyz"].split()]
            assert np.isfinite(mass) and mass > 0 and np.isfinite(matrix).all() and np.isfinite(com).all(), name
            assert min(eig) > 0 and eig[0] + eig[1] - eig[2] >= 0, name
            assert inertial.find("origin").attrib["rpy"] == "0 0 0", name
            sdf_inertial = sdf_model.find('link[@name="' + name + '"]/inertial')
            sdf_tensor = np.array([[float(sdf_inertial.findtext("inertia/" + k)) for k in row] for row in
                                  (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
            assert float(sdf_inertial.findtext("mass")) == mass, name
            assert [float(v) for v in sdf_inertial.findtext("pose").split()] == com + [0.0, 0.0, 0.0], name
            assert np.array_equal(sdf_tensor, matrix), name
            summary["inertials"].append({"name": name, "mass_kg": mass, "com_m": com,
                "tensor_kg_m2": matrix.tolist(), "eigenvalues_kg_m2": eig.tolist(),
                "triangle_gap_kg_m2": float(eig[0] + eig[1] - eig[2])})
            visual = package.read(prefix + "meshes/visual/" + name + ".STL")
            collision = package.read(prefix + "meshes/collision/" + name + ".STL")
            count = struct.unpack_from("<I", visual, 80)[0]
            assert len(visual) == 84 + count * 50 and count > 0, name
            triangles = np.ndarray((count,), dtype=np.dtype([
                ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]),
                buffer=visual, offset=84)
            vertices = triangles["vertices"].reshape(-1, 3)
            assert np.isfinite(vertices).all(), name
            assert visual == collision, name
            summary["meshes"].append({"name": name, "triangles": count, "bytes_each": len(visual),
                "aabb_min_m": vertices.min(0).tolist(), "aabb_max_m": vertices.max(0).tolist(),
                "visual_sha256": digest(visual), "collision_sha256": digest(collision),
                "byte_identical": visual == collision})
    expected_links = {"base_link", *("link" + str(i) for i in range(1, 7))}
    assert {row["name"] for row in summary["inertials"]} == expected_links
    assert len(summary["inertials"]) == len(summary["meshes"]) == 7
    assert digest(args.archive.read_bytes()) == before, "Archive changed"
    summary["total_mass_kg"] = sum(row["mass_kg"] for row in summary["inertials"])
    summary["total_triangles_one_set"] = sum(row["triangles"] for row in summary["meshes"])
    summary["sdf_urdf_inertial_field_max_error"] = 0.0
    summary["checks"] = (
        "PASS: positive mass, finite CoM/tensor/vertices, SPD inertia, principal triangle, "
        "STL length, identical visual/collision pairs, equal URDF/SDF inertials; physical calibration UNKNOWN"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
