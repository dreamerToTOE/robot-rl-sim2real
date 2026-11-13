#!/usr/bin/env python3
"""Minimal Isaac/PhysX collision-report positive and negative controls.

The scene uses ordinary enabled cuboid colliders, with no pair filters.  It is
an independent detector test and says nothing about the vendor robot's cooked
mesh vertices.
"""

from __future__ import annotations

import argparse
import faulthandler
import json
import hashlib
from pathlib import Path
import time
import traceback


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-usd", type=Path)
    parser.add_argument("--robot-urdf", type=Path)
    parser.add_argument("--robot-case", choices=("positive", "negative"), default="positive")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.device != "cpu" or not args.headless:
        parser.error("collision control requires --headless --device cpu")
    if bool(args.robot_usd) != bool(args.robot_urdf):
        parser.error("robot fixture requires both --robot-usd and --robot-urdf")
    args.output.mkdir(parents=True, exist_ok=False)
    app = None
    sim = None
    contacts: list[dict] = []
    result = {"status": "BLOCKED", "failures": [], "contacts": []}
    started = time.perf_counter()
    try:
        app = AppLauncher(args).app
        import isaaclab.sim as sim_utils
        import carb
        from pxr import PhysxSchema, PhysicsSchemaTools, Usd, UsdPhysics
        import omni.physx
        import omni.usd
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=0.01, device="cpu", gravity=(0.0, 0.0, 0.0 if args.robot_usd else -9.81), use_fabric=False,
            enable_scene_query_support=True,
            physx=sim_utils.PhysxCfg(enable_enhanced_determinism=True),
        ))
        # SimulationContext disables contact processing during construction.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        if carb.settings.get_settings().get("/physics/disableContactProcessing") is not False:
            raise RuntimeError("PhysX contact processing is disabled")
        collision = sim_utils.CollisionPropertiesCfg(collision_enabled=True, contact_offset=0.001, rest_offset=0.0)
        static = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=True)
        dynamic = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False)
        mass = sim_utils.MassPropertiesCfg(mass=1.0)
        static_cfg = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), rigid_props=static,
                                         mass_props=mass, collision_props=collision,
                                         activate_contact_sensors=True)
        dynamic_cfg = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), rigid_props=dynamic,
                                          mass_props=mass, collision_props=collision,
                                          activate_contact_sensors=True)
        robot = None
        if args.robot_usd:
            import torch
            from isaaclab.assets import Articulation, ArticulationCfg
            from isaaclab.actuators import IdealPDActuatorCfg
            root = Path(__file__).resolve().parents[1]
            fixture = json.loads((root / "configs/b1_s1_collision_sim_only.json").read_text())
            if hashlib.sha256(args.robot_urdf.read_bytes()).hexdigest() != fixture["source_urdf_sha256"]:
                raise RuntimeError("robot fixture URDF hash mismatch")
            q = dict(zip(fixture["joint_names"], map(float, fixture[args.robot_case + "_q_rad"])))
            robot = Articulation(ArticulationCfg(
                prim_path="/World/Robot", articulation_root_prim_path="/root_joint",
                spawn=sim_utils.UsdFileCfg(usd_path=str(args.robot_usd.resolve()),
                    articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True)),
                init_state=ArticulationCfg.InitialStateCfg(joint_pos=q, joint_vel={".*": 0.0}),
                actuators={"SIM_ONLY": IdealPDActuatorCfg(joint_names_expr=fixture["joint_names"],
                    stiffness=200.0, damping=4.0, effort_limit=20.0, effort_limit_sim=20.0,
                    velocity_limit_sim=0.5, armature=0.02, friction=0.0)},
            ))
        else:
            static_cfg.func("/World/Positive/Static", static_cfg, translation=(0.0, 0.0, 0.0))
            dynamic_cfg.func("/World/Positive/Dynamic", dynamic_cfg, translation=(0.75, 0.0, 2.0))
            static_cfg.func("/World/Negative/Static", static_cfg, translation=(3.0, 0.0, 0.0))
            dynamic_cfg.func("/World/Negative/Dynamic", dynamic_cfg, translation=(6.0, 0.0, 2.0))
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
                report.CreateThresholdAttr().Set(0.0)
        all_prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
        filters = [str(prim.GetPath()) for prim in all_prims
                   if prim.HasAPI(UsdPhysics.FilteredPairsAPI)
                   and UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()]
        groups = [str(prim.GetPath()) for prim in all_prims if prim.IsA(UsdPhysics.CollisionGroup)]
        if filters or groups:
            raise RuntimeError(f"unexpected pair filters/groups: {filters} {groups}")
        enabled = []
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                enabled.append(bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()))
                physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                physx_collision.CreateContactOffsetAttr().Set(0.001)
                physx_collision.CreateRestOffsetAttr().Set(0.0)
        if not enabled or not all(enabled):
            raise RuntimeError("a collider is disabled")
        if robot is not None:
            if PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath("/World/Robot/root_joint")).GetEnabledSelfCollisionsAttr().Get() is not True:
                raise RuntimeError("self-collisions are disabled")

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
        sim.play()
        if robot is not None:
            q_tensor = torch.tensor([[float(q[name]) for name in robot.joint_names]], dtype=torch.float32)
            bounds = robot.root_physx_view.get_dof_limits()[0]
            if not robot.is_fixed_base or torch.any(q_tensor[0] < bounds[:, 0]) or torch.any(q_tensor[0] > bounds[:, 1]):
                raise RuntimeError("robot fixture violates model limits or fixed-base requirement")
            zero_tensor = torch.zeros_like(q_tensor)
            robot.write_joint_state_to_sim(q_tensor, zero_tensor)
            robot.reset()
            sim.forward()
        for step in range(120 if robot is None else 60):
            if robot is not None:
                robot.set_joint_position_target(q_tensor)
                robot.set_joint_velocity_target(zero_tensor)
                robot.write_data_to_sim()
            sim.step(render=False)
            if robot is not None and contacts:
                break
        if robot is None:
            positive = [row for row in contacts if "Positive" in row["actor0"] or "Positive" in row["actor1"]]
            negative = [row for row in contacts if "Negative" in row["actor0"] or "Negative" in row["actor1"]]
        else:
            expected = {"/World/Robot/" + name for name in fixture["expected_nonadjacent_pair"]}
            positive = [row for row in contacts if {row["actor0"], row["actor1"]} == expected]
            negative = contacts if args.robot_case == "negative" else []
        if not positive and (robot is None or args.robot_case == "positive"):
            raise RuntimeError("known impacting positive pair produced no contact report")
        if negative:
            raise RuntimeError("separated negative pair produced a contact report")
        result = {"status": "PASS", "failures": [], "contacts": contacts,
                  "positive_contact_count": len(positive), "negative_contact_count": len(negative),
                  "pair_filters": filters, "collision_groups": groups,
                  "all_colliders_enabled": all(enabled), "steps": step+1,
                  "contact_processing_disabled": False,
                  "method": "enabled cuboids: falling positive impacts a kinematic cube; separated negative"}
        if robot is not None:
            result.update({"method": "independent robot self-collision spawn fixture, stop at first contact",
                           "case": args.robot_case, "q_initial_rad": q,
                           "self_collisions_enabled": True,
                           "expected_nonadjacent_pair": fixture["expected_nonadjacent_pair"]})
    except Exception as exc:
        result["failures"].append(str(exc))
        traceback.print_exc()
    finally:
        result["elapsed_wall_s"] = time.perf_counter() - started
        result["contacts"] = contacts
        write = args.output / "result.json"
        write.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
        print("B1_S1_COLLISION_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)
        if app is not None:
            print("B1_S1_COLLISION_SHUTDOWN_START", flush=True)
            faulthandler.dump_traceback_later(8, file=__import__("sys").stderr)
            if sim is not None:
                sim._disable_app_control_on_stop_handle = True
                sim.stop()
            print("B1_S1_COLLISION_TIMELINE_STOPPED", flush=True)
            app.close(wait_for_replicator=False)
            faulthandler.cancel_dump_traceback_later()


if __name__ == "__main__":
    main()
