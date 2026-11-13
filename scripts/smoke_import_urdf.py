"""Static URDF conversion audit; never plays the timeline or sets joint targets."""
import argparse
import hashlib
import json
import math
import sys
import traceback
from pathlib import Path
from xml.etree import ElementTree as ET


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--usd-dir", type=Path, required=True)
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    private_root = Path(__file__).resolve().parents[1] / "model_assets/local"
    destination = args.usd_dir.resolve()
    if not destination.is_relative_to(private_root.resolve()):
        parser.error("--usd-dir must be inside this repository's ignored model_assets/local/")
    if not args.headless or args.device != "cpu":
        parser.error("This static audit requires --headless --device cpu")
    source = args.urdf.resolve(strict=True)
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    urdf = ET.parse(source).getroot()
    destination.joinpath("static_import_summary.json").unlink(missing_ok=True)
    launcher = AppLauncher(args)
    try:
        import numpy as np
        import omni.timeline
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from compare_model_fk import error, parse_urdf

        timeline = omni.timeline.get_timeline_interface()
        assert not timeline.is_playing(), "Refuse to convert while the timeline is playing"
        cfg = UrdfConverterCfg(
            asset_path=str(source), usd_dir=str(destination), usd_file_name="robot.usd",
            fix_base=True, root_link_name="base_link", merge_fixed_joints=False,
            make_instanceable=False, force_usd_conversion=True, self_collision=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                target_type="none",
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
            ),
        )
        converted = UrdfConverter(cfg)
        stage = Usd.Stage.Open(converted.usd_path)
        assert stage and stage.GetDefaultPrim().IsValid(), "USD has no valid default prim"
        prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
        bodies = {p.GetName(): p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
        joints = {p.GetName(): p for p in prims if p.IsA(UsdPhysics.RevoluteJoint)}
        roots = [str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        assert set(bodies) == {x.attrib["name"] for x in urdf.findall("link")}, bodies.keys()
        expected_joints = {x.attrib["name"] for x in urdf.findall("joint")}
        assert set(joints) == expected_joints, joints.keys()
        assert len(roots) == 1, roots
        fixed_root = UsdPhysics.FixedJoint(stage.GetPrimAtPath(roots[0]))
        assert fixed_root and not fixed_root.GetBody0Rel().GetTargets()
        assert fixed_root.GetBody1Rel().GetTargets() == [bodies["base_link"].GetPath()]
        assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
        assert UsdGeom.GetStageUpAxis(stage) == "Z"
        cpu_model = parse_urdf(source)
        zero = cpu_model.fk({j.name: 0.0 for j in cpu_model.moving})
        cache = UsdGeom.XformCache()
        zero_errors = {}
        for name, body in bodies.items():
            transform = np.array(cache.GetLocalToWorldTransform(body)).T.tolist()
            zero_errors[name] = error(zero[name], transform)
            assert zero_errors[name]["position_m"] < 1e-6, (name, zero_errors[name])
            assert zero_errors[name]["orientation_rad"] < 1e-5, (name, zero_errors[name])
        joint_rows = []
        for element in urdf.findall("joint"):
            joint = UsdPhysics.RevoluteJoint(joints[element.attrib["name"]])
            limit = element.find("limit")
            lower, upper = joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()
            assert abs(lower - math.degrees(float(limit.attrib["lower"]))) < 1e-3
            assert abs(upper - math.degrees(float(limit.attrib["upper"]))) < 1e-3
            parent = joint.GetBody0Rel().GetTargets()
            child = joint.GetBody1Rel().GetTargets()
            assert len(parent) == len(child) == 1
            assert parent[0].name == element.find("parent").attrib["link"]
            assert child[0].name == element.find("child").attrib["link"]
            drives = [str(s) for s in joints[element.attrib["name"]].GetAppliedSchemas() if "DriveAPI" in s]
            drive = UsdPhysics.DriveAPI(joints[element.attrib["name"]], "angular")
            if drive:
                assert drive.GetStiffnessAttr().Get() == drive.GetDampingAttr().Get() == 0.0
            model_joint = next(j for j in cpu_model.joints if j.name == element.attrib["name"])
            parent_rotation = np.array(zero[model_joint.parent])[:3, :3]
            expected_axis = parent_rotation @ np.array(model_joint.origin)[:3, :3] @ np.array(model_joint.axis)
            local_rotation = np.array(Gf.Matrix3d(Gf.Quatd(joint.GetLocalRot0Attr().Get()))).T
            usd_axis = np.eye(3)[{"X": 0, "Y": 1, "Z": 2}[joint.GetAxisAttr().Get()]]
            axis_error = float(np.linalg.norm(parent_rotation @ local_rotation @ usd_axis - expected_axis))
            assert axis_error < 1e-5, (model_joint.name, axis_error)
            joint_rows.append({"name": element.attrib["name"], "body0": parent[0].name,
                               "body1": child[0].name, "lower_deg": lower,
                               "upper_deg": upper, "world_axis_error": axis_error,
                               "drive_schemas": drives, "stiffness_damping": [0.0, 0.0]})
        inertia_rows = []
        for element in urdf.findall("link"):
            mass_api = UsdPhysics.MassAPI(bodies[element.attrib["name"]])
            inertial = element.find("inertial")
            values = inertial.find("inertia").attrib
            expected = np.array([[float(values[k]) for k in row] for row in
                                 (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
            # Gf matrices act on row vectors; convert the principal-axis rotation to column form.
            rotation = np.array(Gf.Matrix3d(Gf.Quatd(mass_api.GetPrincipalAxesAttr().Get()))).T
            actual = rotation @ np.diag(mass_api.GetDiagonalInertiaAttr().Get()) @ rotation.T
            inertia_error = float(np.max(np.abs(actual - expected)))
            assert inertia_error < 1e-8, (element.attrib["name"], inertia_error)
            mass = mass_api.GetMassAttr().Get()
            assert abs(mass - float(inertial.find("mass").attrib["value"])) < 1e-6
            expected_com = np.array([float(v) for v in inertial.find("origin").attrib["xyz"].split()])
            com_error = float(np.max(np.abs(np.array(mass_api.GetCenterOfMassAttr().Get()) - expected_com)))
            assert com_error < 1e-7
            inertia_rows.append({"name": element.attrib["name"], "mass_kg": mass,
                                 "inertia_max_abs_error_kg_m2": inertia_error, "com_max_abs_error_m": com_error})
        meshes = [p for p in prims if p.IsA(UsdGeom.Mesh)]
        assert meshes, "No composed mesh geometry"
        for body in bodies.values():
            for kind in ("visuals", "collisions"):
                assert any(str(p.GetPath()).startswith(str(body.GetPath()) + "/" + kind + "/") for p in meshes), (body.GetPath(), kind)
        for prim in meshes:
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            assert points and all(math.isfinite(v) for p in points for v in p), prim.GetPath()
        from pxr import UsdUtils
        _, _, unresolved = UsdUtils.ComputeAllDependencies(converted.usd_path)
        assert not [asset for asset in unresolved if asset != "OmniPBR.mdl"], unresolved
        assert not timeline.is_playing(), "Timeline changed during static conversion"
        assert hashlib.sha256(source.read_bytes()).hexdigest() == before, "URDF changed"
        summary = {"status": "PARTIAL" if unresolved else "PASS", "geometry_checks": "PASS",
                   "source_sha256": before,
                   "rigid_bodies": sorted(bodies), "revolute_joints": joint_rows,
                   "articulation_roots": roots, "mesh_count": len(meshes),
                   "inertial_checks": inertia_rows, "zero_pose_errors": zero_errors,
                   "unresolved_assets": list(unresolved),
                   "physics_steps": 0, "timeline_played": False,
                   "limits_are_hardware_verified": False,
                   "scope": "static conversion only; no articulation view/servo or collision validation"}
        destination.joinpath("static_import_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print("B0_R1_STATIC_IMPORT=" + json.dumps(summary), flush=True)
    except BaseException:
        traceback.print_exc()
        print("B0_R1_STATIC_IMPORT_FAILED", flush=True)
        raise
    finally:
        launcher.app.close()


if __name__ == "__main__":
    main()
