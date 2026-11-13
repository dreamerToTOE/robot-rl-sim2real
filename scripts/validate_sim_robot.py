#!/usr/bin/env python3
"""B1-S0 only: one private fixed-base USD, SIM_ONLY explicit PD, no RL/SDK.

Run with the unchanged, verified Factory launcher. Never writes the source USD.
Full evidence stays in ignored runs/; publish only reviewed numerical extracts.
"""

import argparse
import faulthandler
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

import numpy as np

from compare_model_fk import error, numbers, parse_urdf, pose
from robot_rl_sim2real.sim_validation import summarize_samples, validate_sample


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def transform(pose):
    """Isaac Lab actor pose is xyz + wxyz, not COM pose."""
    result = np.eye(4)
    result[:3, 3] = pose[:3]
    w, x, y, z = np.asarray(pose[3:], dtype=float)
    length = np.linalg.norm([w, x, y, z])
    if not np.isfinite(length) or length < 1e-9:
        raise ValueError("Invalid link quaternion")
    w, x, y, z = np.asarray([w, x, y, z]) / length
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    return result


def phase_failure(summary, cfg):
    failures = [item["reason"] for item in summary["stop_reasons"]]
    for name, joint in next(iter(summary["phases"].values()))["joints"].items():
        if joint["steady_max_abs_error_rad"] > cfg["steady_position_error_tolerance_rad"]:
            failures.append(f"{name}:steady_error")
        if joint["steady_max_abs_velocity_rad_s"] > cfg["steady_velocity_tolerance_rad_s"]:
            failures.append(f"{name}:not_settled")
        if joint["effort_limit_fraction"] > 0.05:
            failures.append(f"{name}:saturation")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=REPO / "configs/b1_s0_sim_only.json")
    parser.add_argument("--output", type=Path, required=True)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu" or not args.headless:
        parser.error("This bounded diagnostic requires --headless --device cpu")
    cfg = json.loads(args.config.read_text())
    assert cfg["parameter_class"] == "SIM_ONLY"
    assert digest(args.urdf) == cfg["source_urdf_sha256"]
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "input_config.json", cfg)
    # CPU preflight runs in a separate process to avoid mixing the venv's SciPy
    # with Kit's bundled SciPy modules during extension startup.
    child_env = os.environ.copy()
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        child_env.pop(key, None)
    subprocess.run([sys.executable, str(REPO / "scripts/check_model_collisions.py"),
                    "--urdf", str(args.urdf.resolve()), "--output", str((args.output / "collision_preflight.json").resolve())],
                   env=child_env, check=True)
    preflight = json.loads((args.output / "collision_preflight.json").read_text())
    assert preflight["status"] == "PASS"
    model, names = parse_urdf(args.urdf), cfg["joint_names"]
    assert names == [joint.name for joint in model.moving]
    q0 = dict(zip(names, map(float, cfg["initial_joint_positions_rad"])))
    assert all(row["separated"] for row in preflight["baseline_pairs"] if not row["adjacent"]), "Initial non-adjacent collision"
    hashes = json.loads((REPO / "docs/reports/data/B0_R1_STATIC_IMPORT.json").read_text())["generated_usd_sha256"]
    assert all(digest(args.usd.parent / name) == value for name, value in hashes.items())
    limits = {
        "parameter_class": "SIM_ONLY", "joint_names": names,
        "lower_rad": [j.limits["lower"] for j in model.moving],
        "upper_rad": [j.limits["upper"] for j in model.moving],
        "max_velocity_rad_s": [cfg["observed_velocity_stop_rad_s"]] * 6,
        "effort_limit_nm": [cfg["effort_limit_nm"]] * 6,
        "steady_window_s": cfg["steady_window_s"],
        "fk_position_tolerance_m": cfg["fk_position_tolerance_m"],
        "fk_orientation_tolerance_rad": cfg["fk_orientation_tolerance_rad"],
    }
    write_json(args.output / "validation_limits.json", limits)
    app, rows, contacts, metadata = None, [], [], {}
    result = {"status": "BLOCKED", "failures": [], "completed_phases": []}
    started = time.perf_counter()
    try:
        app = AppLauncher(args).app
        import torch
        import isaaclab
        import isaaclab.sim as sim_utils
        from isaacsim.core.version import get_version as get_isaac_version
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab.actuators import IdealPDActuatorCfg
        from pxr import PhysxSchema, PhysicsSchemaTools, Usd, UsdPhysics
        import omni.physx
        import omni.usd
        import carb
        import scipy
        from check_model_collisions import CollisionModel
        geometry = CollisionModel(args.urdf)

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=cfg["dt_s"], device="cpu", gravity=tuple(cfg["gravity_m_s2"]),
            use_fabric=False, enable_scene_query_support=True,
            physx=sim_utils.PhysxCfg(enable_enhanced_determinism=True),
        ))
        # SimulationContext disables contact processing during construction.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        assert carb.settings.get_settings().get("/physics/disableContactProcessing") is False
        robot = Articulation(ArticulationCfg(
            prim_path="/World/Robot", articulation_root_prim_path="/root_joint",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(args.usd.resolve()),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=cfg["solver_position_iterations"],
                    solver_velocity_iteration_count=cfg["solver_velocity_iterations"],
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=q0, joint_vel={".*": 0.0}),
            actuators={"sim_only_pd": IdealPDActuatorCfg(
                joint_names_expr=names, stiffness=cfg["stiffness_nm_rad"], damping=cfg["damping_nm_s_rad"],
                effort_limit=cfg["effort_limit_nm"], effort_limit_sim=cfg["effort_limit_nm"],
                velocity_limit_sim=cfg["velocity_limit_sim_rad_s"],
                armature=cfg["armature_kg_m2"], friction=cfg["friction_coefficient"],
            )},
        ))
        stage = omni.usd.get_context().get_stage()
        sim_version = get_isaac_version()[0]
        lab_commit = subprocess.check_output(["git", "-C", str(Path(isaaclab.__file__).resolve().parents[3]),
                                              "rev-parse", "HEAD"], text=True).strip()
        assert sim_version == "4.5.0" and lab_commit == "90b79bb2d44feb8d833f260f2bf37da3487180ba"
        body_prims = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        assert len(body_prims) == 7
        collision_checks = []
        for prim in body_prims:
            report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            report_api.CreateThresholdAttr().Set(0.0)
            assert report_api.GetThresholdAttr().Get() == 0
            collider = stage.GetPrimAtPath(str(prim.GetPath()) + "/collisions")
            assert collider.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get()
            collision_api = PhysxSchema.PhysxCollisionAPI.Apply(collider)
            collision_api.CreateContactOffsetAttr().Set(cfg["contact_offset_m"])
            collision_api.CreateRestOffsetAttr().Set(cfg["rest_offset_m"])
            assert abs(collision_api.GetContactOffsetAttr().Get() - cfg["contact_offset_m"]) < 1e-7
            assert collision_api.GetRestOffsetAttr().Get() == cfg["rest_offset_m"]
            approximation = UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get()
            assert approximation == "convexHull"
            collision_checks.append({"link": prim.GetName(), "enabled": True, "approximation": approximation,
                                     "contact_offset_m": collision_api.GetContactOffsetAttr().Get(),
                                     "rest_offset_m": collision_api.GetRestOffsetAttr().Get()})
        all_prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
        pair_filters = [str(p.GetPath()) for p in all_prims if p.HasAPI(UsdPhysics.FilteredPairsAPI)
                        and UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets()]
        collision_groups = [str(p.GetPath()) for p in all_prims if p.IsA(UsdPhysics.CollisionGroup)]
        assert not pair_filters and not collision_groups, (pair_filters, collision_groups)
        roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        assert roots == ["/World/Robot/root_joint"], roots
        root_api = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(roots[0]))
        assert root_api.GetEnabledSelfCollisionsAttr().Get() is True

        def on_contact(headers, data):
            for header in headers:
                if header.num_contact_data:
                    contacts.append({
                        "actor0": str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                        "actor1": str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
                        "separations_m": [float(data[i].separation) for i in range(
                            header.contact_data_offset, header.contact_data_offset + header.num_contact_data)],
                    })

        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        sim.reset()
        assert carb.settings.get_settings().get("/physics/disableContactProcessing") is False
        assert robot.is_fixed_base
        assert set(robot.joint_names) == set(names) and len(robot.joint_names) == 6
        assert set(robot.body_names) == set(model.links)
        canonical_indices = [robot.joint_names.index(name) for name in names]
        base_id, tip_id = robot.body_names.index("base_link"), robot.body_names.index("link6")
        view = robot.root_physx_view
        actual_limits = view.get_dof_limits()[0].numpy()[canonical_indices]
        np.testing.assert_allclose(actual_limits, np.array([limits["lower_rad"], limits["upper_rad"]]).T, atol=2e-6)
        np.testing.assert_allclose(view.get_dof_max_forces().numpy(), cfg["effort_limit_nm"])
        np.testing.assert_allclose(view.get_dof_max_velocities().numpy(), cfg["velocity_limit_sim_rad_s"])
        np.testing.assert_allclose(view.get_dof_armatures().numpy(), cfg["armature_kg_m2"])
        masses = view.get_masses()[0].numpy()
        inertias = view.get_inertias()[0].numpy().reshape(7, 3, 3)
        coms = view.get_coms()[0].numpy()
        body_checks = []
        for link in ET.parse(args.urdf).getroot().findall("link"):
            name = link.attrib["name"]
            index = robot.body_names.index(name)
            inertial = link.find("inertial")
            cad_mass = float(inertial.find("mass").attrib["value"])
            tensor = inertial.find("inertia").attrib
            cad_inertia = np.array([[float(tensor["i" + a + b if a <= b else "i" + b + a])
                                    for b in "xyz"] for a in "xyz"])
            origin = inertial.find("origin")
            cad_com = np.array(pose(numbers(origin.get("xyz", "0 0 0"), 3),
                                    numbers(origin.get("rpy", "0 0 0"), 3)))
            c = coms[index]
            physx_com = transform([c[0], c[1], c[2], c[6], c[3], c[4], c[5]])
            cad_link_inertia = cad_com[:3, :3] @ cad_inertia @ cad_com[:3, :3].T
            usd_mass = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"/World/Robot/{name}"))
            usd_q = usd_mass.GetPrincipalAxesAttr().Get()
            usd_q_xyz = usd_q.GetImaginary()
            principal = transform([0, 0, 0, usd_q.GetReal(), *usd_q_xyz])[:3, :3]
            usd_inertia = principal @ np.diag(usd_mass.GetDiagonalInertiaAttr().Get()) @ principal.T
            eigen = np.linalg.eigvalsh(inertias[index])
            assert np.isfinite(inertias[index]).all() and eigen.min() > 0
            assert eigen.max() <= eigen.sum() - eigen.max() + 1e-8
            np.testing.assert_allclose(masses[index], cad_mass, rtol=1e-5, atol=1e-8)
            np.testing.assert_allclose(eigen, np.linalg.eigvalsh(cad_inertia), rtol=2e-5, atol=1e-8)
            np.testing.assert_allclose(physx_com[:3, 3], cad_com[:3, 3], atol=1e-7)
            # For this backend the returned tensor numerically matches the USD
            # principal-axis reconstruction in link axes; do not rotate it twice.
            np.testing.assert_allclose(usd_inertia, cad_link_inertia, rtol=2e-5, atol=1e-8)
            np.testing.assert_allclose(inertias[index], usd_inertia, rtol=2e-5, atol=1e-8)
            body_checks.append({"name": name, "mass_kg": float(masses[index]),
                                "inertia_com_local_kg_m2": inertias[index].tolist(),
                                "physx_com_pose_xyzw": c.tolist(),
                                "principal_inertia_kg_m2": eigen.tolist(),
                                "com_position_error_m": float(np.linalg.norm(physx_com[:3, 3] - cad_com[:3, 3])),
                                "inertia_vs_usd_reconstructed_max_abs_error_kg_m2": float(np.max(np.abs(inertias[index] - usd_inertia))),
                                "inertia_vs_urdf_link_axes_max_abs_error_kg_m2": float(np.max(np.abs(inertias[index] - cad_link_inertia)))})
        q0_physx = torch.tensor([[q0[name] for name in robot.joint_names]], device="cpu")
        zeros = torch.zeros_like(q0_physx)
        robot.write_joint_state_to_sim(q0_physx, zeros)
        robot.reset()
        robot.set_joint_position_target(q0_physx)
        robot.set_joint_velocity_target(zeros)
        robot.write_data_to_sim()
        sim.forward()
        robot.update(cfg["dt_s"])
        base_initial = transform(robot.data.body_link_pose_w[0, base_id].tolist())
        metadata = {
            "python": platform.python_version(), "torch": torch.__version__, "numpy": np.__version__, "scipy": scipy.__version__,
            "isaac_sim": sim_version, "isaac_lab_extension_version": isaaclab.__version__,
            "isaac_lab_commit": lab_commit,
            "urdf_sha256": digest(args.urdf), "usd_sha256": hashes,
            "config_sha256": digest(args.config), "script_sha256": digest(__file__),
            "physx_joint_order": robot.joint_names, "canonical_indices": canonical_indices,
            "physx_body_order": robot.body_names, "fixed_base": robot.is_fixed_base,
            "joint_limits_rad": actual_limits.tolist(), "runtime_body_properties": body_checks,
            "articulation_root": roots, "self_collisions": True, "contact_report_body_count": len(body_prims),
            "collision_checks": collision_checks, "explicit_pair_filters": pair_filters,
            "collision_groups": collision_groups,
            "connected_pair_collision_enabled": {p.GetName(): UsdPhysics.Joint(p).GetCollisionEnabledAttr().Get()
                for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)},
            "inertia_note": "Backend tensor empirically matches the USD principal-axis reconstruction in link axes, at COM; no second COM-quaternion rotation",
            "effort_note": "IdealPD applied torque is clipped command sent to PhysX, not sensor measurement; projected wrench is separate",
        }
        write_json(args.output / "runtime_metadata.json", metadata)
        phases = [("hold", list(q0.values()), cfg["hold_duration_s"])]
        for i, name in enumerate(names):
            for sign, label in ((1, "positive"), (-1, "negative")):
                target = list(q0.values())
                target[i] += sign * cfg["single_joint_delta_rad"]
                assert all(r["separated"] for r in geometry.check(dict(zip(names, target)), aabb_fast=True))
                phases.extend([(f"{name}_{label}", target, cfg["single_target_duration_s"]),
                               (f"{name}_{label}_return", list(q0.values()), cfg["single_target_duration_s"])])
        write_json(args.output / "phases.json", phases)
        step = 0
        with (args.output / "states.jsonl").open("w") as log:
            for phase, target, duration in phases:
                phase_rows = []
                target_by_name = dict(zip(names, target))
                target_tensor = torch.tensor([[target_by_name[n] for n in robot.joint_names]])
                for _ in range(round(duration / cfg["dt_s"])):
                    robot.set_joint_position_target(target_tensor)
                    robot.set_joint_velocity_target(zeros)
                    robot.write_data_to_sim()
                    applied = robot.data.applied_torque[0, canonical_indices].tolist()
                    computed = robot.data.computed_torque[0, canonical_indices].tolist()
                    sim.step(render=False)
                    step += 1
                    robot.update(cfg["dt_s"])
                    q = robot.data.joint_pos[0, canonical_indices].tolist()
                    dq = robot.data.joint_vel[0, canonical_indices].tolist()
                    if not np.isfinite(q).all() or not np.isfinite(dq).all():
                        print(f"B1_S0_NONFINITE phase={phase} step={step} q={q} dq={dq}", flush=True)
                        raise RuntimeError("nonfinite_joint_state")
                    poses = robot.data.body_link_pose_w[0].numpy()
                    base = transform(poses[base_id])
                    relative_tip = np.linalg.inv(base) @ transform(poses[tip_id])
                    cpu_tip = model.fk(dict(zip(names, q)))["link6"]
                    fk_error = error(relative_tip.tolist(), cpu_tip)
                    base_error = error(base.tolist(), base_initial.tolist())
                    # Complete-hull AABB separation certificates; overlap falls back to exact LP.
                    pairs = geometry.check(dict(zip(names, q)), aabb_fast=True)
                    sample = {
                        "time_s": step * cfg["dt_s"], "command_time_s": (step - 1) * cfg["dt_s"],
                        "phase": phase, "joint_names": names, "q_target_rad": target,
                        "q_actual_rad": q, "dq_actual_rad_s": dq,
                        "tracking_error_rad": [a-b for a, b in zip(target, q)],
                        "joint_effort_nm": applied, "computed_torque_nm": computed,
                        "projected_joint_force_nm": view.get_dof_projected_joint_forces()[0, canonical_indices].tolist(),
                        "effort_source": "IdealPDActuator.applied_effort command sent to PhysX (not a sensor)",
                        "fk_position_error_m": fk_error["position_m"],
                        "fk_orientation_error_rad": fk_error["orientation_rad"],
                        "cpu_link6_in_base": cpu_tip, "sim_link6_in_base": relative_tip.tolist(),
                        "base_drift_position_m": base_error["position_m"],
                        "base_drift_orientation_rad": base_error["orientation_rad"],
                        "collision_detected": bool(contacts) or not all(p["separated"] for p in pairs),
                        "collision_source": "PhysX pair contact reports threshold=0 plus per-step CPU CAD convex hull separation",
                        "minimum_aabb_distance_lower_bound_m": min(
                            (p["aabb_distance_lower_bound_m"] for p in pairs if "aabb_distance_lower_bound_m" in p), default=None),
                    }
                    # Preserve a failed row before stopping; JSON null cannot conceal a NaN.
                    log.write(json.dumps(sample, allow_nan=False) + "\n")
                    rows.append(sample)
                    phase_rows.append(sample)
                    reasons = validate_sample(sample, limits)
                    if base_error["position_m"] > 1e-6 or base_error["orientation_rad"] > 1e-5:
                        reasons.append("fixed_base_drift")
                    if reasons:
                        raise RuntimeError(f"{phase} step {step}: {reasons}")
                phase_summary = summarize_samples(phase_rows, limits)
                failures = phase_failure(phase_summary, cfg)
                # Direction uses measured phase start/end, never the target or imported USD pose.
                previous = rows[-len(phase_rows)-1] if len(rows) > len(phase_rows) else phase_rows[0]
                changed = [i for i in range(6) if abs(target[i] - previous["q_target_rad"][i]) > 1e-5]
                for i in changed:
                    target_step = target[i] - previous["q_target_rad"][i]
                    actual_step = phase_rows[-1]["q_actual_rad"][i] - previous["q_actual_rad"][i]
                    if actual_step * target_step <= 0:
                        failures.append(f"{names[i]}:wrong_direction")
                    if abs(actual_step) < cfg["minimum_response_fraction_of_target_step"] * abs(target_step):
                        failures.append(f"{names[i]}:insufficient_response")
                if failures:
                    raise RuntimeError(f"{phase}: {failures}")
                result["completed_phases"].append(phase)
                print("B1_S0_PHASE=" + phase, flush=True)
        assert len(result["completed_phases"]) == 25 and len(rows) == 6240
        assert not summarize_samples(rows, limits)["stop_reasons"]
        assert subscription is not None
        result["status"] = "PASS"
    except Exception as exc:
        result["failures"].append(str(exc))
        traceback.print_exc()
    finally:
        result["elapsed_seconds"] = time.perf_counter() - started
        result["contacts"] = contacts
        result["summary"] = summarize_samples(rows, limits)
        result["usd_unchanged"] = all(digest(args.usd.parent / n) == h for n, h in hashes.items())
        if not result["usd_unchanged"]:
            result["status"] = "BLOCKED"
            result["failures"].append("source_usd_changed")
        write_json(args.output / "result.json", result)
        print("B1_S0_RESULT=" + json.dumps({k: result[k] for k in ("status", "failures", "elapsed_seconds")}), flush=True)
        if app is not None:
            print("B1_S0_SHUTDOWN_START", flush=True)
            faulthandler.dump_traceback_later(8, file=sys.stderr)
            if "sim" in locals():
                # Isaac Lab uses this guard in reset(); without it its STOP callback
                # spins in render waiting for a GUI restart during headless shutdown.
                sim._disable_app_control_on_stop_handle = True
                sim.stop()
            print("B1_S0_TIMELINE_STOPPED", flush=True)
            app.close(wait_for_replicator=False)
            faulthandler.cancel_dump_traceback_later()
            print("B1_S0_SHUTDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
