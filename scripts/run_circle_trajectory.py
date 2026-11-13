#!/usr/bin/env python3
"""Run the B1-S1 classical IK reference through the B1-S0 SIM_ONLY PD chain."""

from __future__ import annotations

import argparse
import faulthandler
import json
from pathlib import Path
import os
import platform
import subprocess
import sys
import time
import traceback

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

from compare_model_fk import error, parse_urdf
from validate_sim_robot import digest, transform, write_json


def read_rows(path: Path) -> list[dict]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError("empty trajectory")
    indices = [row["sample_index"] for row in rows]
    if indices != list(range(len(rows))):
        raise ValueError("trajectory sample indices are not contiguous")
    return rows


def inspect_and_screen_cooked(stage, geometry, rows, output, padding, robot_path="/World/Robot"):
    """Read actual runtime convex meshes, then screen a CAD+cooked outer envelope."""
    from pxr import Gf, UsdGeom
    import omni.physx
    from check_model_collisions import hull_planes, hull_relation
    cooking = omni.physx.get_physx_cooking_interface()
    cache = UsdGeom.XformCache()
    raw, comparisons = {}, []
    for name in geometry.model.links:
        link = stage.GetPrimAtPath(f"{robot_path}/{name}")
        collider = stage.GetPrimAtPath(f"{robot_path}/{name}/collisions")
        path = str(collider.GetPath())
        count = cooking.get_nb_convex_mesh_data(path)
        if count != 1:
            raise RuntimeError(f"cooked hull UNKNOWN for {name}: expected one, got {count}")
        data = cooking.get_convex_mesh_data(path, 0)
        local = cache.GetLocalToWorldTransform(collider) * cache.GetLocalToWorldTransform(link).GetInverse()
        cooked = np.array([list(local.Transform(Gf.Vec3d(*vertex))) for vertex in data["vertices"]])
        if cooked.shape != (data["num_vertices"], 3) or not np.isfinite(cooked).all():
            raise RuntimeError(f"invalid cooked vertices for {name}")
        raw[name] = cooked.tolist()
        comparisons.append({"link": name, "convex_count": count,
                            "cooked_polygon_count": data["num_polygons"],
                            **hull_relation(geometry.vertices[name], cooked)})
        envelope = np.vstack((geometry.vertices[name], cooked))
        geometry.vertices[name] = envelope
        geometry.hulls[name] = hull_planes(envelope)
    # Cooked vertices are derived vendor geometry and stay only in ignored runs/.
    write_json(output / "private_cooked_vertices.json", raw)
    fallback_count, pair_count = 0, 0
    names = [joint.name for joint in geometry.model.moving]
    for row in rows:
        pairs = geometry.check(dict(zip(names, row["q_target_rad"])), aabb_fast=True, padding_m=padding)
        fallback_count += sum(pair.get("aabb_distance_lower_bound_m") is None for pair in pairs)
        pair_count += len(pairs)
        if not all(pair["separated"] for pair in pairs):
            raise RuntimeError(f"padded CAD/cooked envelope collision before motion: sample {row['sample_index']}")
    result = {"status": "PASS", "links": comparisons, "reference_samples_checked": len(rows),
              "nonadjacent_pairs_checked": pair_count, "lp_fallback_count": fallback_count,
              "padding_per_shape_m": padding,
              "method": "per-link convex hull of CAD and runtime cooked vertices, outward plane padding for contact skin",
              "scope": "sampled reference poses, standard direct-joint exclusions; not continuous swept-volume or hardware safety"}
    write_json(output / "cooked_mesh_comparison.json", result)
    return result


def summary(rows: list[dict], cfg: dict, names: list[str], contacts: list[dict], failures: list[str]) -> dict:
    if not rows:
        return {"parameter_class": "SIM_ONLY", "status": "BLOCKED", "sample_count": 0,
                "duration_s": 0.0, "failures": failures, "contacts": contacts}
    q_target = np.asarray([row["q_target_rad"] for row in rows], dtype=float)
    q_actual = np.asarray([row["q_actual_rad"] for row in rows], dtype=float)
    dq = np.asarray([row["dq_actual_rad_s"] for row in rows], dtype=float)
    ddq = np.asarray([row["ddq_actual_rad_s2"] for row in rows], dtype=float)
    effort = np.asarray([row["joint_effort_nm"] for row in rows], dtype=float)
    position = np.asarray([row["position_error_m"] for row in rows], dtype=float)
    orientation = np.asarray([row["orientation_error_rad"] for row in rows], dtype=float)
    circle_mask = np.array([row["phase"] == "circle" for row in rows])
    circle_indices = np.flatnonzero(circle_mask)
    circle_rows = [rows[index] for index in circle_indices]
    q_error = q_target - q_actual
    result = {
        "parameter_class": "SIM_ONLY",
        "status": "PASS" if not failures else "BLOCKED",
        "sample_count": len(rows),
        "duration_s": rows[-1]["time_s"] if rows else 0.0,
        "failures": failures,
        "contacts": contacts,
        "tracking": None,
        "circle_completion": None,
        "joint_dynamics": {
            "maximum_abs_velocity_rad_s": float(np.max(np.abs(dq))),
            "maximum_abs_acceleration_rad_s2": float(np.max(np.abs(ddq))),
            "maximum_abs_effort_nm": float(np.max(np.abs(effort))),
            "effort_saturation_fraction": float(np.mean(np.abs(effort) >= 0.99 * cfg["controller"]["effort_limit_nm"])),
            "maximum_target_step_rad": float(np.max(np.abs(np.diff(q_target, axis=0)))) if len(rows) > 1 else 0.0,
            "maximum_actual_step_rad": float(np.max(np.abs(np.diff(q_actual, axis=0)))) if len(rows) > 1 else 0.0,
        },
        "safety": {
            "collision_reports": len(contacts),
            "collision_detected": any(row["collision_detected"] for row in rows),
            "limit_violation": any(row["limit_violation"] for row in rows),
            "nonfinite_state": any(not np.isfinite(np.asarray(row["q_actual_rad"], dtype=float)).all() for row in rows),
            "maximum_fk_position_error_m": float(max((row["fk_position_error_m"] for row in rows), default=0.0)),
            "maximum_fk_orientation_error_rad": float(max((row["fk_orientation_error_rad"] for row in rows), default=0.0)),
            "minimum_aabb_distance_lower_bound_m": float(min((row["minimum_aabb_distance_lower_bound_m"] for row in rows), default=0.0)),
        },
        "gravity_static_bias": {
            "initial_hold_max_joint_error_rad": float(max((max(abs(v) for v in row["tracking_error_rad"])
                                                            for row in rows if row["phase"] == "initial_hold"), default=0.0)),
            "final_hold_max_joint_error_rad": float(max((max(abs(v) for v in row["tracking_error_rad"])
                                                          for row in rows if row["phase"] == "final_hold"), default=0.0)),
            "circle_mean_position_error_m": float(np.mean(position[circle_mask])) if circle_rows else None,
            "interpretation": "Position-only PD has no measured-model gravity compensation; this is SIM_ONLY evidence.",
        },
    }
    if circle_rows:
        circle_position = np.asarray([row["actual_link6_position_m"] for row in circle_rows], dtype=float)
        center = np.asarray(cfg["circle_center_m"], dtype=float)
        angles = np.unwrap(np.arctan2(circle_position[:, 1] - center[1], circle_position[:, 0] - center[0]))
        circle_q_error = q_error[circle_mask]
        q_rms = np.sqrt(np.mean(circle_q_error * circle_q_error, axis=0))
        q_max = np.max(np.abs(circle_q_error), axis=0)
        result["tracking"] = {
            "circle_sample_count": len(circle_rows),
            "rmse_position_m": float(np.sqrt(np.mean(position[circle_mask] ** 2))),
            "p95_position_m": float(np.quantile(position[circle_mask], 0.95)),
            "maximum_position_m": float(np.max(position[circle_mask])),
            "rmse_orientation_rad": float(np.sqrt(np.mean(orientation[circle_mask] ** 2))),
            "maximum_orientation_rad": float(np.max(orientation[circle_mask])),
            "joint_error_rmse_rad": dict(sorted(zip(names, q_rms.tolist()), key=lambda pair: pair[1], reverse=True)),
            "joint_error_max_rad": dict(sorted(zip(names, q_max.tolist()), key=lambda pair: pair[1], reverse=True)),
        }
        result["circle_completion"] = {
            "reference_phase_span_rad": float(circle_rows[-1]["circle_phase_rad"] - circle_rows[0]["circle_phase_rad"]),
            "actual_angle_span_rad": float(angles[-1] - angles[0]),
            "actual_radius_mean_m": float(np.mean(np.linalg.norm(circle_position[:, :2] - center[:2], axis=1))),
            "actual_radius_rmse_m": float(np.sqrt(np.mean((np.linalg.norm(circle_position[:, :2] - center[:2], axis=1) - cfg["circle_radius_m"]) ** 2))),
            "completed_one_circle": bool((angles[-1] - angles[0]) > 1.8 * np.pi),
            "execution_time_s": float(circle_rows[-1]["time_s"] - circle_rows[0]["time_s"]),
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=REPO / "configs/b1_s1_circle_sim_only.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gravity-compensation", action="store_true",
                        help="Separate SIM_ONLY comparison using PhysX model gravity forces")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu" or not args.headless:
        parser.error("B1-S1 requires --headless --device cpu")
    cfg = json.loads(args.config.read_text())
    if cfg.get("parameter_class") != "SIM_ONLY":
        raise ValueError("B1-S1 must remain SIM_ONLY")
    if digest(args.urdf) != cfg["source_urdf_sha256"]:
        raise ValueError("URDF hash does not match reviewed input")
    rows = read_rows(args.trajectory)
    plan_summary = json.loads((args.trajectory.parent / "summary.json").read_text())
    if (plan_summary.get("status") != "PASS" or plan_summary.get("sample_count") != len(rows)
            or plan_summary.get("trajectory_sha256") != digest(args.trajectory)
            or plan_summary.get("configuration_sha256") != digest(args.config)):
        raise ValueError("trajectory has not passed screening with this configuration")
    cfg = dict(cfg)
    cfg["circle_center_m"] = plan_summary["circle"]["center_m"]
    cfg["circle_radius_m"] = plan_summary["circle"]["radius_m"]
    cfg["circle_period_s"] = plan_summary["circle"]["period_s"]
    cfg["controller_mode"] = "PD_PLUS_MODEL_GRAVITY_SIM_ONLY" if args.gravity_compensation else "PD_BASELINE_SIM_ONLY"
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "input_config.json", cfg)
    write_json(args.output / "trajectory_summary.json", {
        "source": str(args.trajectory),
        "sample_count": len(rows),
        "trajectory_sha256": digest(args.trajectory),
        "circle_center_m": cfg["circle_center_m"],
        "circle_radius_m": cfg["circle_radius_m"],
        "circle_period_s": cfg["circle_period_s"],
    })
    child_env = os.environ.copy()
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        child_env.pop(key, None)
    subprocess.run([sys.executable, str(REPO / "scripts/check_model_collisions.py"),
                    "--urdf", str(args.urdf.resolve()),
                    "--output", str((args.output / "collision_preflight.json").resolve())],
                   env=child_env, check=True)
    model = parse_urdf(args.urdf)
    names = [joint.name for joint in model.moving]
    hashes = json.loads((REPO / "docs/reports/data/B0_R1_STATIC_IMPORT.json").read_text())["generated_usd_sha256"]
    if not all(digest(args.usd.parent / name) == value for name, value in hashes.items()):
        raise ValueError("private USD hash differs from the reviewed B0-R1A input")
    app, sim, subscription = None, None, None
    contacts: list[dict] = []
    states: list[dict] = []
    failures: list[str] = []
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
        from check_model_collisions import CollisionModel
        geometry = CollisionModel(args.urdf)

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=cfg["dt_s"], device="cpu", gravity=(0.0, 0.0, -9.81),
            use_fabric=False, enable_scene_query_support=True,
            physx=sim_utils.PhysxCfg(enable_enhanced_determinism=True),
        ))
        # SimulationContext disables contact processing during construction.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        if carb.settings.get_settings().get("/physics/disableContactProcessing") is not False:
            raise RuntimeError("PhysX contact processing is disabled")
        q_start = dict(zip(names, cfg["q_start_rad"]))
        controller = cfg["controller"]
        robot = Articulation(ArticulationCfg(
            prim_path="/World/Robot", articulation_root_prim_path="/root_joint",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(args.usd.resolve()),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=controller["solver_position_iterations"],
                    solver_velocity_iteration_count=controller["solver_velocity_iterations"],
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=q_start, joint_vel={".*": 0.0}),
            actuators={"sim_only_pd": IdealPDActuatorCfg(
                joint_names_expr=names,
                stiffness=controller["stiffness_nm_rad"], damping=controller["damping_nm_s_rad"],
                effort_limit=controller["effort_limit_nm"], effort_limit_sim=controller["effort_limit_nm"],
                velocity_limit_sim=controller["velocity_limit_sim_rad_s"],
                armature=controller["armature_kg_m2"], friction=controller["friction_coefficient"],
            )},
        ))
        stage = omni.usd.get_context().get_stage()
        sim_version = get_isaac_version()[0]
        lab_root = Path(isaaclab.__file__).resolve().parents[3]
        lab_commit = subprocess.check_output(["git", "-C", str(lab_root), "rev-parse", "HEAD"], text=True).strip()
        if sim_version != "4.5.0" or lab_commit != "90b79bb2d44feb8d833f260f2bf37da3487180ba":
            raise RuntimeError(f"unexpected Isaac versions: {sim_version} {lab_commit}")
        body_prims = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(body_prims) != 7:
            raise RuntimeError(f"unexpected rigid body count: {len(body_prims)}")
        collision_checks = []
        for prim in body_prims:
            report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            report.CreateThresholdAttr().Set(0.0)
            collider = stage.GetPrimAtPath(str(prim.GetPath()) + "/collisions")
            collision = UsdPhysics.CollisionAPI(collider)
            if not collider.HasAPI(UsdPhysics.CollisionAPI) or not collision.GetCollisionEnabledAttr().Get():
                raise RuntimeError(f"disabled collider: {collider.GetPath()}")
            physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(collider)
            physx_collision.CreateContactOffsetAttr().Set(controller["contact_offset_m"])
            physx_collision.CreateRestOffsetAttr().Set(controller["rest_offset_m"])
            approximation = UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get()
            if approximation != "convexHull":
                raise RuntimeError(f"unexpected collision approximation: {approximation}")
            collision_checks.append({"link": prim.GetName(), "approximation": approximation,
                                     "enabled": True, "contact_offset_m": controller["contact_offset_m"]})
        all_prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
        pair_filters = [str(prim.GetPath()) for prim in all_prims
                        if prim.HasAPI(UsdPhysics.FilteredPairsAPI)
                        and UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()]
        collision_groups = [str(prim.GetPath()) for prim in all_prims if prim.IsA(UsdPhysics.CollisionGroup)]
        if pair_filters or collision_groups:
            raise RuntimeError(f"unexpected collision filters: {pair_filters} {collision_groups}")
        roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
        root_api = PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(roots[0]))
        if roots != ["/World/Robot/root_joint"] or root_api.GetEnabledSelfCollisionsAttr().Get() is not True:
            raise RuntimeError("articulation root/self-collision check failed")

        def on_contact(headers, data):
            for header in headers:
                if header.num_contact_data:
                    contacts.append({
                        "actor0": str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                        "actor1": str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
                        "separations_m": [float(data[index].separation) for index in range(
                            header.contact_data_offset, header.contact_data_offset + header.num_contact_data)],
                    })

        subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        sim.reset()
        if carb.settings.get_settings().get("/physics/disableContactProcessing") is not False:
            raise RuntimeError("PhysX contact processing was disabled during reset")
        if not robot.is_fixed_base or set(robot.joint_names) != set(names) or len(robot.joint_names) != 6:
            raise RuntimeError("robot base or DoF mapping check failed")
        if set(robot.body_names) != set(model.links):
            raise RuntimeError("rigid body name mapping failed")
        canonical_indices = [robot.joint_names.index(name) for name in names]
        base_id, tip_id = robot.body_names.index("base_link"), robot.body_names.index("link6")
        view = robot.root_physx_view
        actual_limits = view.get_dof_limits()[0].numpy()[canonical_indices]
        expected_limits = np.array([[joint.limits["lower"], joint.limits["upper"]] for joint in model.moving])
        np.testing.assert_allclose(actual_limits, expected_limits, atol=2e-6)
        np.testing.assert_allclose(view.get_dof_max_forces().numpy(), controller["effort_limit_nm"])
        np.testing.assert_allclose(view.get_dof_max_velocities().numpy(), controller["velocity_limit_sim_rad_s"])
        zeros = torch.zeros((1, 6), dtype=torch.float32, device="cpu")
        q_initial = torch.tensor([[q_start[name] for name in robot.joint_names]], dtype=torch.float32, device="cpu")
        robot.write_joint_state_to_sim(q_initial, zeros)
        robot.reset()
        cooked_report = inspect_and_screen_cooked(stage, geometry, rows, args.output, controller["contact_offset_m"])
        base_initial = transform(robot.data.body_link_pose_w[0, base_id].tolist())
        previous_dq = np.zeros(6)
        clock_zero = sim.current_time
        initial_step_index = sim.current_time_step_index
        for row in rows:
            target = row["q_target_rad"]
            target_tensor = torch.tensor([[target[names.index(name)] for name in robot.joint_names]],
                                         dtype=torch.float32, device="cpu")
            robot.set_joint_position_target(target_tensor)
            robot.set_joint_velocity_target(zeros)
            gravity_feedforward = view.get_gravity_compensation_forces().clone() if args.gravity_compensation else zeros
            if not torch.isfinite(gravity_feedforward).all():
                raise RuntimeError("nonfinite gravity feedforward")
            robot.set_joint_effort_target(gravity_feedforward)
            robot.write_data_to_sim()
            applied = robot.data.applied_torque[0, canonical_indices].tolist()
            computed = robot.data.computed_torque[0, canonical_indices].tolist()
            command_time = sim.current_time - clock_zero
            sim.step(render=False)
            robot.update(cfg["dt_s"])
            q = np.asarray(robot.data.joint_pos[0, canonical_indices].tolist(), dtype=float)
            dq = np.asarray(robot.data.joint_vel[0, canonical_indices].tolist(), dtype=float)
            ddq = (dq - previous_dq) / cfg["dt_s"]
            previous_dq = dq
            if not np.isfinite(q).all() or not np.isfinite(dq).all():
                failures.append(f"nonfinite_state:{row['sample_index']}")
                break
            poses = robot.data.body_link_pose_w[0].numpy()
            base = transform(poses[base_id])
            actual_tip = np.linalg.inv(base) @ transform(poses[tip_id])
            cpu_tip = model.fk(dict(zip(names, q)))['link6']
            fk_error = error(actual_tip.tolist(), cpu_tip)
            ref_pose = np.eye(4)
            ref_pose[:3, :3] = np.asarray(row["reference_rotation_matrix"], dtype=float).reshape(3, 3)
            ref_pose[:3, 3] = np.asarray(row["reference_position_m"], dtype=float)
            fk_tip = np.asarray(cpu_tip, dtype=float)
            position_error = float(np.linalg.norm(fk_tip[:3, 3] - ref_pose[:3, 3]))
            orientation_error = float(error(cpu_tip, ref_pose.tolist())["orientation_rad"])
            pairs = geometry.check(dict(zip(names, q.tolist())), aabb_fast=True,
                                   padding_m=controller["contact_offset_m"])
            collision = bool(contacts) or not all(pair["separated"] for pair in pairs)
            limit_violation = bool(np.any(q < actual_limits[:, 0] - 1e-6) or np.any(q > actual_limits[:, 1] + 1e-6))
            sample = {
                "sample_index": row["sample_index"], "time_s": sim.current_time - clock_zero,
                "command_time_s": command_time,
                "reference_time_s": row["time_s"], "absolute_simulation_time_s": sim.current_time,
                "simulation_step_index": sim.current_time_step_index,
                "phase": row["phase"], "circle_phase_rad": row["circle_phase_rad"],
                "joint_names": names, "q_target_rad": target, "q_actual_rad": q.tolist(),
                "tracking_error_rad": (np.asarray(target) - q).tolist(),
                "dq_actual_rad_s": dq.tolist(), "ddq_actual_rad_s2": ddq.tolist(),
                "joint_effort_nm": applied, "computed_torque_nm": computed,
                "gravity_feedforward_nm": gravity_feedforward[0, canonical_indices].tolist(),
                "projected_joint_force_nm": view.get_dof_projected_joint_forces()[0, canonical_indices].tolist(),
                "effort_source": "IdealPDActuator.applied_effort command sent to PhysX (not a sensor)",
                "reference_position_m": row["reference_position_m"], "actual_link6_position_m": fk_tip[:3, 3].tolist(),
                "sim_link6_position_m": actual_tip[:3, 3].tolist(),
                "position_error_m": position_error, "orientation_error_rad": orientation_error,
                "fk_position_error_m": fk_error["position_m"], "fk_orientation_error_rad": fk_error["orientation_rad"],
                "base_drift_position_m": error(base.tolist(), base_initial.tolist())["position_m"],
                "base_drift_orientation_rad": error(base.tolist(), base_initial.tolist())["orientation_rad"],
                "collision_detected": collision, "limit_violation": limit_violation,
                "collision_source": "PhysX contact reports plus per-step padded CAD/cooked convex envelope CPU screening",
                "minimum_aabb_distance_lower_bound_m": min((pair.get("aabb_distance_lower_bound_m", 0.0) for pair in pairs), default=0.0),
            }
            states.append(sample)
            if (collision or limit_violation or sample["base_drift_position_m"] > 1e-6
                    or abs(sample["time_s"] - row["time_s"]) > 1e-5
                    or sim.current_time_step_index-initial_step_index != row["sample_index"]+1
                    or sample["base_drift_orientation_rad"] > 1e-5
                    or sample["fk_position_error_m"] > 1e-4
                    or sample["fk_orientation_error_rad"] > 1e-3
                    or np.any(np.abs(dq) > 0.55)
                    or np.any(np.abs(ddq) > cfg["observed_acceleration_stop_rad_s2"])
                    or np.any(np.abs(applied) > controller["effort_limit_nm"] + 1e-6)):
                failures.append(f"safety_gate:{row['sample_index']}")
                break
        if len(states) != len(rows):
            raise RuntimeError(f"trajectory stopped after {len(states)}/{len(rows)} samples")
        metadata = {
            "python": platform.python_version(), "torch": torch.__version__, "numpy": np.__version__,
            "isaac_sim": sim_version, "isaac_lab_extension_version": isaaclab.__version__,
            "isaac_lab_commit": lab_commit, "urdf_sha256": digest(args.urdf),
            "usd_sha256": hashes, "config_sha256": digest(args.config),
            "trajectory_sha256": digest(args.trajectory), "script_sha256": digest(__file__),
            "physx_joint_order": robot.joint_names, "canonical_indices": canonical_indices,
            "physx_body_order": robot.body_names, "fixed_base": robot.is_fixed_base,
            "self_collisions": True, "contact_processing_disabled": False, "collision_checks": collision_checks,
            "explicit_pair_filters": pair_filters, "collision_groups": collision_groups,
            "connected_pair_collision_enabled": {prim.GetName(): UsdPhysics.Joint(prim).GetCollisionEnabledAttr().Get()
                for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)},
            "controller_mode": cfg["controller_mode"],
            "clock_source": "SimulationContext.current_time and current_time_step_index, read after each physics step",
            "initial_clock_s": clock_zero, "final_clock_s": sim.current_time,
            "physx_cooked_hull": "runtime vertices queried in local link axes; compared and screened with CAD/cooked padded envelope",
            "cooked_geometry_report": cooked_report,
        }
        write_json(args.output / "runtime_metadata.json", metadata)
    except Exception as exc:
        failures.append(str(exc))
        traceback.print_exc()
    finally:
        if not all(digest(args.usd.parent / name) == value for name, value in hashes.items()):
            failures.append("source_usd_changed")
        with (args.output / "states.jsonl").open("w") as stream:
            for state in states:
                stream.write(json.dumps(state, allow_nan=False) + "\n")
        result_summary = summary(states, cfg, names, contacts, failures)
        if not failures and (not result_summary.get("circle_completion")
                             or not result_summary["circle_completion"]["completed_one_circle"]):
            failures.append("actual_circle_not_completed")
            result_summary["status"] = "BLOCKED"
        result_summary["controller_mode"] = cfg["controller_mode"]
        result_summary["elapsed_wall_s"] = time.perf_counter() - started
        write_json(args.output / "summary.json", result_summary)
        result = {"status": result_summary["status"], "failures": failures,
                  "sample_count": len(states), "contacts": contacts,
                  "elapsed_wall_s": result_summary["elapsed_wall_s"]}
        write_json(args.output / "result.json", result)
        print("B1_S1_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)
        if app is not None:
            print("B1_S1_SHUTDOWN_START", flush=True)
            faulthandler.dump_traceback_later(8, file=sys.stderr)
            if sim is not None:
                sim._disable_app_control_on_stop_handle = True
                sim.stop()
            print("B1_S1_TIMELINE_STOPPED", flush=True)
            app.close(wait_for_replicator=False)
            faulthandler.cancel_dump_traceback_later()
            print("B1_S1_SHUTDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
