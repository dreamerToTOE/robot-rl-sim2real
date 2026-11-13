"""CPU convex-hull self-collision audit; never imports a simulator or SDK.

Uses the existing NumPy/SciPy environment. The signed intersection inradius
is not a surface distance or a PhysX cooked-hull contact measurement.
"""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import time
from xml.etree import ElementTree as ET

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

from compare_model_fk import numbers, parse_urdf, pose


def hull_planes(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Expected finite N x 3 collision vertices")
    return ConvexHull(points).equations


def hull_relation(source_vertices, cooked_vertices):
    """Containment witnesses and volumes; plane residuals are not Hausdorff distances."""
    source, cooked = np.asarray(source_vertices), np.asarray(cooked_vertices)
    source_planes, cooked_planes = hull_planes(source), hull_planes(cooked)
    return {
        "source_hull_vertex_count": len(ConvexHull(source).vertices),
        "cooked_vertex_count": len(cooked),
        "maximum_cooked_outside_source_plane_m": float(np.max(cooked @ source_planes[:, :3].T + source_planes[:, 3])),
        "maximum_source_outside_cooked_plane_m": float(np.max(source @ cooked_planes[:, :3].T + cooked_planes[:, 3])),
        "source_convex_hull_volume_m3": float(ConvexHull(source).volume),
        "cooked_convex_hull_volume_m3": float(ConvexHull(cooked).volume),
    }


def intersection_inradius(planes_a, planes_b):
    """Positive = overlap, zero = contact, negative = disjoint convex hulls."""
    planes = np.vstack((planes_a, planes_b))
    if planes.shape[1] != 4 or not np.isfinite(planes).all():
        raise ValueError("Invalid hull planes")
    lengths = np.linalg.norm(planes[:, :3], axis=1)
    if not np.allclose(lengths, 1.0, atol=1e-8):
        raise ValueError("Hull planes must have unit normals")
    constraints = np.column_stack((planes[:, :3], np.ones(len(planes))))
    # Tight feasibility tolerances avoid a HiGHS status-4 result on nearly
    # touching hulls even when the returned point is otherwise valid.
    result = linprog([0, 0, 0, -1], A_ub=constraints, b_ub=-planes[:, 3],
                     bounds=[(None, None)] * 4, method="highs",
                     options={"dual_feasibility_tolerance": 1e-9,
                              "primal_feasibility_tolerance": 1e-9})
    if not result.success or not np.isfinite(result.x).all():
        raise RuntimeError(f"Collision LP failed closed: {result.message}")
    if np.max(constraints @ result.x + planes[:, 3]) > 1e-7:
        raise RuntimeError("Collision LP residual exceeds tolerance")
    return float(result.x[3])


class CollisionModel:
    def __init__(self, urdf_path):
        self.source = Path(urdf_path).resolve(strict=True)
        self.model = parse_urdf(self.source)
        self.adjacent = {frozenset((j.parent, j.child)) for j in self.model.joints}
        self.hulls = {}
        self.vertices = {}
        self.geometry = []
        root = ET.parse(self.source).getroot()
        for link in root.findall("link"):
            collisions = link.findall("collision")
            if len(collisions) != 1:
                raise ValueError("This audit requires exactly one collision mesh per link")
            collision = collisions[0]
            geometry = collision.find("geometry")
            mesh = geometry.find("mesh")
            if mesh is None or len(list(geometry)) != 1:
                raise ValueError("Only mesh collision geometry is supported")
            uri = mesh.attrib["filename"]
            if uri.startswith("package://"):
                package, relative = uri[len("package://"):].split("/", 1)
                if package != self.source.parent.parent.name:
                    raise ValueError("Collision mesh package does not match local URDF package")
                path = self.source.parent.parent / relative
            else:
                path = self.source.parent / uri
            path = path.resolve(strict=True)
            if not path.is_relative_to(self.source.parent.parent):
                raise ValueError("Collision mesh outside local model package")
            raw = path.read_bytes()
            if len(raw) < 84:
                raise ValueError("Truncated binary STL")
            count = struct.unpack_from("<I", raw, 80)[0]
            if len(raw) != 84 + 50 * count:
                raise ValueError("Unsupported or truncated binary STL")
            vertices = np.array([struct.unpack_from("<9f", raw, 96 + 50 * i)
                                 for i in range(count)]).reshape(-1, 3)
            vertices *= numbers(mesh.get("scale", "1 1 1"), 3)
            origin = collision.find("origin")
            if origin is not None:
                transform = np.array(pose(numbers(origin.get("xyz", "0 0 0"), 3),
                                          numbers(origin.get("rpy", "0 0 0"), 3)))
                vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
            self.hulls[link.attrib["name"]] = hull_planes(vertices)
            self.vertices[link.attrib["name"]] = vertices[ConvexHull(vertices).vertices]
            self.geometry.append({"link": link.attrib["name"], "triangles": count,
                                  "mesh_sha256": hashlib.sha256(raw).hexdigest(),
                                  "hull_planes": len(self.hulls[link.attrib["name"]])})

    def check(self, q, include_adjacent=False, aabb_fast=False, padding_m=0.0):
        if not math.isfinite(padding_m) or padding_m < 0:
            raise ValueError("Collision padding must be finite and nonnegative")
        for joint in self.model.moving:
            if not joint.limits["lower"] <= q[joint.name] <= joint.limits["upper"]:
                raise ValueError(f"Position limit violated: {joint.name}")
        transforms = self.model.fk(q)
        world = {}
        bounds = {}
        for name, planes in self.hulls.items():
            transform = np.array(transforms[name])
            normals = planes[:, :3] @ transform[:3, :3].T
            world[name] = np.column_stack((normals, planes[:, 3] - normals @ transform[:3, 3] - padding_m))
            if aabb_fast:
                vertices = self.vertices[name] @ transform[:3, :3].T + transform[:3, 3]
                bounds[name] = (vertices.min(axis=0) - padding_m, vertices.max(axis=0) + padding_m)
        rows = []
        for a, b in itertools.combinations(self.model.links, 2):
            adjacent = frozenset((a, b)) in self.adjacent
            if include_adjacent or not adjacent:
                if aabb_fast:
                    gap = np.maximum(bounds[a][0] - bounds[b][1], bounds[b][0] - bounds[a][1])
                    lower_bound = float(np.linalg.norm(np.maximum(gap, 0)))
                    if lower_bound > 1e-6:
                        rows.append({"a": a, "b": b, "adjacent": adjacent,
                                     "intersection_inradius_m": None,
                                     "aabb_distance_lower_bound_m": lower_bound, "separated": True})
                        continue
                margin = intersection_inradius(world[a], world[b])
                rows.append({"a": a, "b": b, "adjacent": adjacent,
                             "intersection_inradius_m": margin,
                             "separated": margin < -1e-6})
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    model = CollisionModel(args.urdf)
    names = [j.name for j in model.model.moving]
    zero = dict.fromkeys(names, 0.0)
    started = time.perf_counter()
    baseline = model.check(zero, include_adjacent=True)
    samples = []
    for name in names:
        for value in np.linspace(-0.02, 0.02, 9):
            q = zero | {name: float(value)}
            rows = model.check(q)
            samples.append({"q_rad": q, "worst_pair": max(rows, key=lambda r: r["intersection_inradius_m"]),
                            "all_nonadjacent_separated": all(r["separated"] for r in rows)})
    rng = np.random.default_rng(20261113)
    for _ in range(32):
        q = dict(zip(names, rng.uniform(-0.02, 0.02, len(names)).tolist()))
        rows = model.check(q)
        samples.append({"q_rad": q, "worst_pair": max(rows, key=lambda r: r["intersection_inradius_m"]),
                        "all_nonadjacent_separated": all(r["separated"] for r in rows)})
    elapsed = time.perf_counter() - started
    result = {"source_sha256": hashlib.sha256(model.source.read_bytes()).hexdigest(),
              "method": "exact CAD vertex convex hull, halfspace LP; not PhysX cooked hull",
              "q0_rad": zero, "baseline_pairs": baseline, "geometry": model.geometry,
              "samples": samples, "elapsed_seconds": elapsed,
              "mean_seconds_per_configuration": elapsed / (len(samples) + 1),
              "status": "PASS" if all(s["all_nonadjacent_separated"] for s in samples) else "BLOCKED"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("status", "elapsed_seconds", "mean_seconds_per_configuration")}))


if __name__ == "__main__":
    main()
