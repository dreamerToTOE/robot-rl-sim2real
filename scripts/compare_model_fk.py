#!/usr/bin/env python3
"""Read-only URDF/SDF tree FK comparison; stdlib only, no simulator or SDK.

Supported SDF: a single flat 1.7--1.11 model, local pose / axis frames,
Euler-radian poses, and fixed / revolute joints. Unsupported kinematics fail
closed. This is an offline format comparison, never hardware calibration.
"""

import argparse
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET


def eye():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse(a):
    out = eye()
    for i in range(3):
        for j in range(3):
            out[i][j] = a[j][i]
        out[i][3] = -sum(a[j][i] * a[j][3] for j in range(3))
    return out


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def numbers(text, size):
    values = [float(x) for x in text.split()]
    if len(values) != size or not all(math.isfinite(x) for x in values):
        raise ValueError(f"Expected {size} finite numbers")
    return values


def rotation(axis, angle):
    length = norm(axis)
    if not math.isfinite(angle) or length <= 0:
        raise ValueError("Invalid rotation")
    x, y, z = [v / length for v in axis]
    c, s, d = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    out = eye()
    out[:3] = [[c+x*x*d, x*y*d-z*s, x*z*d+y*s, 0],
               [y*x*d+z*s, c+y*y*d, y*z*d-x*s, 0],
               [z*x*d-y*s, z*y*d+x*s, c+z*z*d, 0]]
    return out


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    out = mul(rotation((0, 0, 1), rpy[2]),
              mul(rotation((0, 1, 0), rpy[1]), rotation((1, 0, 0), rpy[0])))
    for i in range(3):
        out[i][3] = xyz[i]
    return out


def sdf_pose(element):
    if element is None:
        return eye()
    if set(element.attrib) - {"relative_to", "rotation_format", "degrees"}:
        raise ValueError("Unsupported SDF pose attributes")
    if element.get("rotation_format", "euler_rpy") != "euler_rpy" or element.get("degrees", "false") not in ("false", "0"):
        raise ValueError("Only Euler-radian SDF poses are supported")
    values = numbers(element.text or "0 0 0 0 0 0", 6)
    return pose(values[:3], values[3:])


def error(a, b):
    dp = [a[i][3] - b[i][3] for i in range(3)]
    rel = mul(a, inverse(b))
    skew = [(rel[2][1]-rel[1][2])/2, (rel[0][2]-rel[2][0])/2, (rel[1][0]-rel[0][1])/2]
    angle = math.atan2(norm(skew), max(-1, min(1, (sum(rel[i][i] for i in range(3))-1)/2)))
    return {"position_m": norm(dp), "orientation_rad": angle, "delta_position_m": dp}


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: list
    child_offset: list
    axis: list
    limits: dict


class Model:
    def __init__(self, links, joints):
        if not links or len(set(links)) != len(links) or len({j.name for j in joints}) != len(joints):
            raise ValueError("Empty / duplicate links or joints")
        self.links = links
        children = [j.child for j in joints]
        if len(set(children)) != len(children):
            raise ValueError("Closed chains / multiple parents are unsupported")
        if any(j.parent not in links or j.child not in links for j in joints):
            raise ValueError("Unknown parent / child link")
        roots = set(links) - set(children)
        if len(roots) != 1:
            raise ValueError("Expected one connected tree root")
        self.root = roots.pop()
        ordered, visited = [], {self.root}
        while len(ordered) < len(joints):
            batch = [j for j in joints if j.parent in visited and j not in ordered]
            if not batch:
                raise ValueError("Disconnected / cyclic kinematic graph")
            ordered.extend(batch)
            visited.update(j.child for j in batch)
        if visited != set(links):
            raise ValueError("Disconnected links")
        self.joints = ordered
        self.moving = [j for j in ordered if j.kind == "revolute"]
        self.leaves = sorted(set(links) - {j.parent for j in ordered})

    def fk(self, q):
        if set(q) != {j.name for j in self.moving} or not all(math.isfinite(v) for v in q.values()):
            raise ValueError("Expected finite angles for exactly the moving joints")
        transforms = {self.root: eye()}
        for j in self.joints:
            motion = rotation(j.axis, q[j.name]) if j.kind == "revolute" else eye()
            transforms[j.child] = mul(transforms[j.parent], mul(j.origin, mul(motion, j.child_offset)))
        return transforms


def limit_values(element, sdf):
    out = {}
    for key in ("lower", "upper", "velocity", "effort"):
        value = element.findtext(key) if sdf and element is not None else element.get(key) if element is not None else None
        out[key] = None if value is None else numbers(value, 1)[0]
    if out["lower"] is not None and out["upper"] is not None and out["lower"] > out["upper"]:
        raise ValueError("Reversed joint limits")
    return out


def parse_urdf(path):
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("Expected URDF robot")
    joints = []
    for j in root.findall("joint"):
        kind = j.get("type")
        if kind not in ("fixed", "revolute") or j.find("mimic") is not None:
            raise ValueError("Only independent fixed / revolute joints are supported")
        o, axis = j.find("origin"), j.find("axis")
        origin = pose(numbers(o.get("xyz", "0 0 0"), 3), numbers(o.get("rpy", "0 0 0"), 3)) if o is not None else eye()
        vector = numbers(axis.get("xyz", "1 0 0"), 3) if axis is not None else [1, 0, 0]
        if kind == "revolute" and norm(vector) <= 0:
            raise ValueError("Zero joint axis")
        joints.append(Joint(j.attrib["name"], kind, j.find("parent").attrib["link"], j.find("child").attrib["link"],
                            origin, eye(), vector, limit_values(j.find("limit"), False)))
    return Model([link.attrib["name"] for link in root.findall("link")], joints)


def parse_sdf(path):
    root = ET.parse(path).getroot()
    if root.tag != "sdf" or root.get("version") not in {f"1.{n}" for n in range(7, 12)}:
        raise ValueError("Only SDF 1.7--1.11 supported")
    if len(root.findall("model")) != 1 or root.find("world") is not None:
        raise ValueError("Expected one top-level model")
    model = root.find("model")
    if model.find("model") is not None or model.find("include") is not None or model.get("placement_frame"):
        raise ValueError("Nested / included / placement-frame models unsupported")
    if model.find("pose") is not None and model.find("pose").get("relative_to", ""):
        raise ValueError("External model pose references unsupported")
    sdf_pose(model.find("pose"))  # Model placement cancels in base-relative FK.
    links, joints, frames = model.findall("link"), model.findall("joint"), model.findall("frame")
    if not links:
        raise ValueError("No SDF links")
    canonical = model.get("canonical_link", links[0].attrib["name"])
    if canonical not in {e.attrib["name"] for e in links}:
        raise ValueError("Unsupported canonical link")
    nodes = {}
    attachments = {"__model__": canonical}
    for element in links + joints + frames:
        name = element.attrib["name"]
        if name in nodes or name in ("__model__", "world") or "::" in name:
            raise ValueError("Invalid / duplicate SDF frame name")
        if element.tag == "joint":
            default = element.findtext("child")
            attachments[name] = default
        elif element.tag == "frame":
            default = element.get("attached_to") or "__model__"
            attachments[name] = default
        else:
            default = "__model__"
        p = element.find("pose")
        nodes[name] = ((p.get("relative_to") or default) if p is not None else default, sdf_pose(p))
    cache = {"__model__": eye()}

    def resolve(name, active=()):
        if name in cache:
            return cache[name]
        if name in active or name not in nodes:
            raise ValueError("Cyclic / unknown / scoped SDF pose reference")
        reference, local = nodes[name]
        cache[name] = mul(resolve(reference, active + (name,)), local)
        return cache[name]

    for name in nodes:
        resolve(name)
    for name in attachments:
        current, seen = name, set()
        while current not in {link.attrib["name"] for link in links}:
            if current in seen or current not in attachments:
                raise ValueError("Invalid SDF attached_to graph")
            seen.add(current)
            current = attachments[current]
    out = []
    for j in joints:
        kind, name = j.get("type"), j.attrib["name"]
        if kind not in ("fixed", "revolute") or j.find("axis2") is not None or j.find("axis/mimic") is not None:
            raise ValueError("Only independent fixed / revolute SDF joints supported")
        if j.find("axis/use_parent_model_frame") is not None:
            raise ValueError("Legacy SDF axis frame unsupported")
        parent, child = j.findtext("parent"), j.findtext("child")
        if parent not in nodes or child not in nodes:
            raise ValueError("External joint links unsupported")
        axis = j.find("axis/xyz")
        vector = numbers(axis.text or "0 0 1", 3) if axis is not None else [0, 0, 1]
        expressed = axis.get("expressed_in") or name if axis is not None else name
        if expressed != name:
            change = mul(inverse(resolve(name)), resolve(expressed))
            vector = [sum(change[i][k] * vector[k] for k in range(3)) for i in range(3)]
        if kind == "revolute" and norm(vector) <= 0:
            raise ValueError("Zero SDF joint axis")
        out.append(Joint(name, kind, parent, child, mul(inverse(resolve(parent)), resolve(name)),
                         mul(inverse(resolve(name)), resolve(child)), vector, limit_values(j.find("axis/limit"), True)))
    return Model([link.attrib["name"] for link in links], out)


def mapped(q, signs):
    return {name: signs[name] * value for name, value in q.items()}


def common_limits(a, b, signs):
    other = {j.name: j for j in b.moving}
    result = {}
    for j in a.moving:
        lo, hi = j.limits["lower"], j.limits["upper"]
        blo, bhi = other[j.name].limits["lower"], other[j.name].limits["upper"]
        if None in (lo, hi, blo, bhi):
            raise ValueError("Finite position limits required for comparison sampling")
        if signs[j.name] < 0:
            blo, bhi = -bhi, -blo
        lo, hi = max(lo, blo), min(hi, bhi)
        if not lo < 0 < hi:
            raise ValueError("Comparison requires a shared neighborhood around q=0")
        result[j.name] = (lo, hi)
    return result


def configurations(limits, seed, count):
    zero = {name: 0.0 for name in limits}
    result = [("zero", zero)]
    for name, (lo, hi) in limits.items():
        step = min(0.01, -lo / 10, hi / 10)
        for value in (-step, step):
            result.append(("single_joint", dict(zero, **{name: value})))
        margin = (hi - lo) * 1e-6
        for value in (lo + margin, hi - margin):
            result.append(("near_limit", dict(zero, **{name: value})))
    rng = random.Random(seed)
    for _ in range(count):
        result.append(("random", {name: rng.uniform(lo, hi) for name, (lo, hi) in limits.items()}))
    for edge in (0, 1):
        result.append(("near_limit_corner", {name: bounds[edge] + (1 if edge == 0 else -1) * (bounds[1]-bounds[0])*1e-6 for name, bounds in limits.items()}))
    return result


def find_signs(a, b):
    names = [j.name for j in a.moving]
    if not 1 <= len(names) <= 10:
        raise ValueError("Sign search restricted to 1--10 moving joints")
    candidates = []
    for values in itertools.product((1, -1), repeat=len(names)):
        signs = dict(zip(names, values))
        try:
            limits = common_limits(a, b, signs)
        except ValueError:
            continue
        probes = [("zero", {name: 0.0 for name in names})]
        for name, (lo, hi) in limits.items():
            probes.append(("probe", {n: min(0.1, -lo/10, hi/10) if n == name else 0.0 for n in names}))
        score = 0.0
        for _, q in probes:
            u, s = a.fk(q), b.fk(mapped(q, signs))
            for link in a.links:
                e = error(u[link], s[link])
                score += e["position_m"]**2 + e["orientation_rad"]**2
        candidates.append((score, signs))
    if not candidates:
        raise ValueError("No admissible sign mapping")
    candidates.sort(key=lambda item: item[0])
    return candidates


def compare(a, b, tip, seed=20261113, count=256, tolerance_m=1e-8, tolerance_rad=1e-8):
    if set(a.links) != set(b.links) or a.root != b.root:
        raise ValueError("Matching named link sets / root required")
    topology = lambda m: {(j.name, j.kind, j.parent, j.child) for j in m.joints}
    if topology(a) != topology(b) or tip not in a.links:
        raise ValueError("Matching named joint topology and valid tip required")
    candidates = find_signs(a, b)
    signs = candidates[0][1]
    limits = common_limits(a, b, signs)
    configs = configurations(limits, seed, count)
    zero = {name: 0.0 for name in limits}
    u0, s0 = a.fk(zero), b.fk(zero)
    registration = mul(u0[tip], inverse(s0[tip]))
    summaries = {}
    records = []
    representative_poses = {}
    links = {name: {"same_base_max_m": 0.0, "same_base_max_rad": 0.0, "registered_max_m": 0.0, "registered_max_rad": 0.0} for name in a.links}
    for family, q in configs:
        u, s, same = a.fk(q), b.fk(mapped(q, signs)), b.fk(q)
        raw, aligned, numeric = error(u[tip], s[tip]), error(u[tip], mul(registration, s[tip])), error(u[tip], same[tip])
        if family not in representative_poses:
            representative_poses[family] = {"q_urdf_rad": q, "q_sdf_rad": mapped(q, signs),
                                            "urdf_tip_matrix": u[tip], "sdf_tip_matrix": s[tip],
                                            "registered_sdf_tip_matrix": mul(registration, s[tip])}
        for label, e in (("same_numeric", numeric), ("mapped_same_base", raw), ("mapped_registered", aligned)):
            summary = summaries.setdefault(family, {}).setdefault(label, {"max_position_m": 0.0, "max_orientation_rad": 0.0, "count": 0})
            summary["count"] += 1
            summary["max_position_m"] = max(summary["max_position_m"], e["position_m"])
            summary["max_orientation_rad"] = max(summary["max_orientation_rad"], e["orientation_rad"])
        for link in a.links:
            raw_link, aligned_link = error(u[link], s[link]), error(u[link], mul(registration, s[link]))
            links[link]["same_base_max_m"] = max(links[link]["same_base_max_m"], raw_link["position_m"])
            links[link]["same_base_max_rad"] = max(links[link]["same_base_max_rad"], raw_link["orientation_rad"])
            links[link]["registered_max_m"] = max(links[link]["registered_max_m"], aligned_link["position_m"])
            links[link]["registered_max_rad"] = max(links[link]["registered_max_rad"], aligned_link["orientation_rad"])
        records.append({"family": family, "q_urdf_rad": q, "mapped_same_base": raw, "mapped_registered": aligned})
    boundaries = []
    for label, source in (("urdf", a), ("sdf", b)):
        for j in source.moving:
            lo, hi = j.limits["lower"], j.limits["upper"]
            margin = (hi - lo) * 1e-6
            for edge, value in (("lower", lo + margin), ("upper", hi - margin)):
                source_q = dict(zero, **{j.name: value})
                uq = source_q if label == "urdf" else mapped(source_q, signs)
                sq = mapped(source_q, signs) if label == "urdf" else source_q
                u, s = a.fk(uq), b.fk(sq)
                violations = lambda model, q: [joint.name for joint in model.moving if not joint.limits["lower"] <= q[joint.name] <= joint.limits["upper"]]
                boundaries.append({"source": label, "joint": j.name, "edge": edge, "q_urdf_rad": uq,
                                   "urdf_limit_violations": violations(a, uq), "sdf_limit_violations": violations(b, sq),
                                   "mapped_same_base": error(u[tip], s[tip]),
                                   "mapped_registered": error(u[tip], mul(registration, s[tip]))})
    direction = []
    h = 1e-6
    bases = [zero] + [q for family, q in configs if family == "random"][:8]
    for name in limits:
        rows = []
        for q in bases:
            plus, minus = dict(q), dict(q)
            plus[name], minus[name] = q[name]+h, q[name]-h
            up, um = a.fk(plus)[tip], a.fk(minus)[tip]
            sp, sm = b.fk(mapped(plus, signs))[tip], b.fk(mapped(minus, signs))[tip]
            uj = [(up[i][3]-um[i][3])/(2*h) for i in range(3)]
            sj = [(sp[i][3]-sm[i][3])/(2*h) for i in range(3)]
            ur, sr = mul(up, inverse(um)), mul(sp, inverse(sm))
            angular = lambda r: [(r[2][1]-r[1][2])/(4*h), (r[0][2]-r[2][0])/(4*h), (r[1][0]-r[0][1])/(4*h)]
            uw, sw = angular(ur), angular(sr)
            rows.append({"linear_column_error_m_per_rad": norm([x-y for x, y in zip(uj, sj)]),
                         "angular_column_error": norm([x-y for x, y in zip(uw, sw)]),
                         "linear_cosine": sum(x*y for x, y in zip(uj, sj))/(norm(uj)*norm(sj)) if min(norm(uj), norm(sj)) > 1e-8 else None,
                         "angular_cosine": sum(x*y for x, y in zip(uw, sw))/(norm(uw)*norm(sw)),
                         "urdf_linear_at_zero": uj, "sdf_mapped_linear_at_zero": sj})
        direction.append({"joint": name, "poses_tested": len(rows), "zero": rows[0],
                          "max_linear_column_error_m_per_rad": max(r["linear_column_error_m_per_rad"] for r in rows),
                          "max_angular_column_error": max(r["angular_column_error"] for r in rows),
                          "min_linear_cosine": min((r["linear_cosine"] for r in rows if r["linear_cosine"] is not None), default=None),
                          "min_angular_cosine": min(r["angular_cosine"] for r in rows)})
    other = {j.name: j for j in b.joints}
    joint_diffs = []
    for j in a.joints:
        s = other[j.name]
        joint_diffs.append({"joint": j.name, "origin_delta": error(j.origin, s.origin),
                            "child_offset_delta": error(j.child_offset, s.child_offset),
                            "urdf_axis": j.axis, "sdf_axis_in_joint_frame": s.axis,
                            "urdf_limits": j.limits, "sdf_limits": s.limits,
                            "parent_child_equal": (j.parent, j.child) == (s.parent, s.child)})
    maximum = lambda key, field: max(f[key][field] for f in summaries.values())
    registered_pass = maximum("mapped_registered", "max_position_m") <= tolerance_m and maximum("mapped_registered", "max_orientation_rad") <= tolerance_rad
    return {"schema": 1, "seed": seed, "configuration_count": len(configs), "random_count": count,
            "source_boundary_count": len(boundaries), "total_evaluated_configuration_count": len(configs)+len(boundaries),
            "root": a.root, "tip": tip, "joint_order": [j.name for j in a.moving],
            "sign_mapping_sdf_from_urdf": signs, "sign_candidates_tested": len(candidates),
            "best_sign_score": candidates[0][0], "runner_up_score": candidates[1][0] if len(candidates) > 1 else None,
            "common_limits_in_urdf_coordinates": limits, "joint_comparison": joint_diffs,
            "q0_urdf_tip_matrix": u0[tip], "q0_sdf_tip_matrix": s0[tip],
            "fixed_left_registration_sdf_to_urdf": registration,
            "numeric_tolerance_only": {"position_m": tolerance_m, "orientation_rad": tolerance_rad},
            "registered_tip_within_numeric_tolerance": registered_pass,
            "families": summaries, "representative_poses": representative_poses,
            "all_link_comparison": links, "direction_comparison": direction,
            "samples": records, "source_boundary_samples": boundaries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--sdf", type=Path, required=True)
    parser.add_argument("--tip", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20261113)
    parser.add_argument("--random-count", type=int, default=256)
    args = parser.parse_args()
    if args.random_count < 1 or args.output.resolve() in (args.urdf.resolve(), args.sdf.resolve()):
        parser.error("Positive random count and a separate output path required")
    result = compare(parse_urdf(args.urdf), parse_sdf(args.sdf), args.tip, args.seed, args.random_count)
    result["input_sha256"] = {key: hashlib.sha256(path.read_bytes()).hexdigest() for key, path in (("urdf", args.urdf), ("sdf", args.sdf))}
    result["script_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("configuration_count", "sign_mapping_sdf_from_urdf", "registered_tip_within_numeric_tolerance", "families")}, indent=2))


if __name__ == "__main__":
    main()
