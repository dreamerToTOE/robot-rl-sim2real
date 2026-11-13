#!/usr/bin/env python3
"""Generate and CPU-screen the B1-S1 fixed-orientation circular reference.

This is an offline planner.  It only reads the private URDF supplied by the
operator and writes derived, reviewable numbers; it never opens Isaac, ROS, an
SDK, or a hardware connection.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
sys.path.insert(0, str(REPO / "scripts"))

from check_model_collisions import CollisionModel
from compare_model_fk import parse_urdf
from robot_rl_sim2real.circle_trajectory import (
    circle_phase_count,
    circle_position,
    jacobian_metrics,
    quintic_joint_path,
    phase_ramp_integral,
    rotation_vector,
    solve_fixed_orientation,
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def limits(model):
    return (np.array([joint.limits["lower"] for joint in model.moving], dtype=float),
            np.array([joint.limits["upper"] for joint in model.moving], dtype=float))


def collision_status(geometry: CollisionModel, q: np.ndarray) -> tuple[bool, float]:
    """Return separated/minimum AABB lower bound, failing closed on ambiguity."""
    names = [joint.name for joint in geometry.model.moving]
    rows = geometry.check(dict(zip(names, q.tolist())), aabb_fast=True)
    # AABB overlap falls back to LP, which does not provide a distance bound.
    return all(row["separated"] for row in rows), min(
        (row.get("aabb_distance_lower_bound_m", 0.0) for row in rows), default=0.0)


def circle_ik(model, names, q_home, radius, period, ramp_duration, lower, upper, geometry,
              sample_count=61):
    """Solve a coarse circle for candidate search and return diagnostics."""
    home_pose = np.asarray(model.fk(dict(zip(names, q_home)))['link6'], dtype=float)
    start_phase = -math.pi * ramp_duration / period
    center = home_pose[:3, 3] - np.array([radius * math.cos(start_phase), radius * math.sin(start_phase), 0.0])
    previous = q_home.copy()
    qs, residuals, sigmas, conditions, collision_gaps = [], [], [], [], []
    for index in range(sample_count):
        phase = start_phase + 2.0 * math.pi * index / (sample_count - 1)
        target = circle_position(center, radius, phase)
        if index == 0:
            q, residual = q_home.copy(), 0.0
        else:
            q, residual = solve_fixed_orientation(
                model, names, target, home_pose[:3, :3], previous, lower, upper)
        separated, gap = collision_status(geometry, q)
        metrics = jacobian_metrics(model, names, q)
        qs.append(q)
        residuals.append(residual)
        sigmas.append(metrics["minimum_singular_value"])
        conditions.append(metrics["condition_number"])
        collision_gaps.append(gap if separated else -1.0)
        if not separated:
            raise ValueError(f"candidate circle collision at phase {phase}")
        previous = q
    qs = np.asarray(qs)
    angle_step = period / (sample_count - 1)
    q_steps = np.diff(qs, axis=0)
    return {
        "radius_m": radius,
        "period_s": period,
        "sample_count": sample_count,
        "max_ik_residual": max(residuals),
        "minimum_jacobian_sigma": min(sigmas),
        "maximum_jacobian_condition": max(conditions),
        "maximum_coarse_joint_speed_rad_s": float(np.max(np.abs(q_steps) / angle_step)),
        "minimum_aabb_distance_lower_bound_m": min(collision_gaps),
        "joint_range_rad": np.ptp(qs, axis=0).tolist(),
        "status": "PASS",
    }


def build_rows(model, geometry, cfg, selected, names, q_start, q_home, lower, upper):
    dt = float(cfg["dt_s"])
    home_pose = np.asarray(model.fk(dict(zip(names, q_home)))['link6'], dtype=float)
    omega = 2.0 * math.pi / selected["period_s"]
    ramp_duration = float(cfg["circle_ramp_duration_s"])
    start_phase = -0.5 * omega * ramp_duration
    center = home_pose[:3, 3] - np.array([selected["radius_m"] * math.cos(start_phase),
                                         selected["radius_m"] * math.sin(start_phase), 0.0])
    circle_count = circle_phase_count(selected["period_s"], dt)
    rows = []

    def append(phase, q, target_position, target_rotation, phase_value=None):
        q = np.asarray(q, dtype=float)
        if np.any(q < lower - 1e-9) or np.any(q > upper + 1e-9):
            raise ValueError(f"joint limit exceeded in {phase}")
        separated, gap = collision_status(geometry, q)
        if not separated:
            raise ValueError(f"collision in {phase}")
        actual_pose = np.asarray(model.fk(dict(zip(names, q)))['link6'], dtype=float)
        rows.append({
            "sample_index": len(rows),
            "time_s": (len(rows) + 1) * dt,
            "phase": phase,
            "circle_phase_rad": phase_value,
            "q_target_rad": q.tolist(),
            "reference_position_m": np.asarray(target_position).tolist(),
            "reference_rotation_matrix": np.asarray(target_rotation).reshape(-1).tolist(),
            "reference_fk_position_m": actual_pose[:3, 3].tolist(),
            "reference_fk_position_error_m": float(np.linalg.norm(actual_pose[:3, 3] - target_position)),
            "reference_fk_orientation_error_rad": float(np.linalg.norm(
                rotation_vector(np.asarray(target_rotation).T @ actual_pose[:3, :3]))),
            "collision_screened": True,
            "minimum_aabb_distance_lower_bound_m": gap,
        })

    hold_count = round(float(cfg["hold_duration_s"]) / dt)
    for _ in range(hold_count):
        pose = np.asarray(model.fk(dict(zip(names, q_start)))['link6'], dtype=float)
        append("initial_hold", q_start, pose[:3, 3], pose[:3, :3])

    ingress_count = round(float(cfg["ingress_duration_s"]) / dt)
    ingress = quintic_joint_path(q_start, q_home, ingress_count + 1)
    for q in ingress[1:]:
        pose = np.asarray(model.fk(dict(zip(names, q)))['link6'], dtype=float)
        append("ingress", q, pose[:3, 3], pose[:3, :3])

    previous = q_home.copy()
    ramp_count = round(ramp_duration / dt)
    if ramp_count < 2 or abs(ramp_count * dt - ramp_duration) > 1e-10:
        raise ValueError("ramp duration must be representable by the simulation timestep")
    for index in range(1, ramp_count + 1):
        phase = start_phase + omega * ramp_duration * phase_ramp_integral(index / ramp_count)
        target_position = circle_position(center, selected["radius_m"], phase)
        q, residual = solve_fixed_orientation(
            model, names, target_position, home_pose[:3, :3], previous, lower, upper)
        append("circle" if index == ramp_count else "circle_startup", q,
               target_position, home_pose[:3, :3], phase)
        rows[-1]["ik_residual"] = residual
        previous = q
    for index in range(1, circle_count + 1):
        # Constant angular speed gives x=xc+r*cos(omega*t), y=yc+r*sin(omega*t).
        phase = omega * index * dt
        target_position = circle_position(center, selected["radius_m"], phase)
        q, residual = solve_fixed_orientation(
            model, names, target_position, home_pose[:3, :3], previous, lower, upper)
        if residual > 1e-7:
            raise ValueError(f"circle IK residual {residual}")
        append("circle", q, target_position, home_pose[:3, :3], phase)
        rows[-1]["ik_residual"] = residual
        previous = q

    for index in range(1, ramp_count + 1):
        s = index / ramp_count
        phase = 2.0 * math.pi + omega * ramp_duration * (s - phase_ramp_integral(s))
        target_position = circle_position(center, selected["radius_m"], phase)
        q, residual = solve_fixed_orientation(
            model, names, target_position, home_pose[:3, :3], previous, lower, upper)
        append("circle_stop", q, target_position, home_pose[:3, :3], phase)
        rows[-1]["ik_residual"] = residual
        previous = q

    egress_count = round(float(cfg["egress_duration_s"]) / dt)
    egress = quintic_joint_path(previous, q_start, egress_count + 1)
    for q in egress[1:]:
        pose = np.asarray(model.fk(dict(zip(names, q)))['link6'], dtype=float)
        append("egress", q, pose[:3, 3], pose[:3, :3])

    for _ in range(hold_count):
        pose = np.asarray(model.fk(dict(zip(names, q_start)))['link6'], dtype=float)
        append("final_hold", q_start, pose[:3, 3], pose[:3, :3])

    q_values = np.asarray([row["q_target_rad"] for row in rows])
    q_velocity = np.gradient(q_values, dt, axis=0, edge_order=2)
    q_acceleration = np.gradient(q_velocity, dt, axis=0, edge_order=2)
    for row, velocity, acceleration in zip(rows, q_velocity, q_acceleration):
        row["q_reference_velocity_rad_s"] = velocity.tolist()
        row["q_reference_acceleration_rad_s2"] = acceleration.tolist()
    return rows, center.tolist(), home_pose


def summarize(rows, cfg, selected, candidate_search, q_home, home_pose, lower, upper, model, names):
    q = np.asarray([row["q_target_rad"] for row in rows])
    velocity = np.asarray([row["q_reference_velocity_rad_s"] for row in rows])
    acceleration = np.asarray([row["q_reference_acceleration_rad_s2"] for row in rows])
    circle = [row for row in rows if row["phase"] == "circle"]
    circle_q = np.asarray([row["q_target_rad"] for row in circle])
    circle_phase = np.asarray([row["circle_phase_rad"] for row in circle])
    phase_names = ("initial_hold", "ingress", "circle_startup", "circle", "circle_stop", "egress", "final_hold")
    jacobian_by_phase = {}
    for phase in phase_names:
        phase_rows = [row for row in rows if row["phase"] == phase]
        selected_rows = phase_rows[::8]
        if phase_rows[-1] is not selected_rows[-1]:
            selected_rows.append(phase_rows[-1])
        metrics = [jacobian_metrics(model, names, np.asarray(row["q_target_rad"])) for row in selected_rows]
        jacobian_by_phase[phase] = {"samples": len(metrics),
                                    "minimum_singular_value": min(item["minimum_singular_value"] for item in metrics),
                                    "maximum_condition_number": max(item["condition_number"] for item in metrics)}
    return {
        "status": "PASS",
        "parameter_class": "SIM_ONLY",
        "sample_count": len(rows),
        "duration_s": rows[-1]["time_s"],
        "phases": {name: sum(row["phase"] == name for row in rows) for name in phase_names},
        "circle": {
            "center_m": selected["center_m"],
            "radius_m": selected["radius_m"],
            "period_s": selected["period_s"],
            "phase_start_rad": float(circle_phase[0]),
            "phase_end_rad": float(circle_phase[-1]),
            "completed_one_reference_circle": bool(abs(circle_phase[-1] - 2.0 * math.pi) < 1e-12),
            "q_home_rad": q_home.tolist(),
            "q_range_rad": np.ptp(circle_q, axis=0).tolist(),
        },
        "ik": {
            "method": "bounded scipy least_squares on six pose residuals, previous solution warm start",
            "fixed_orientation": True,
            "maximum_residual": max(row.get("ik_residual", 0.0) for row in circle),
            "jacobian_sampling": "every 8th reference sample and last sample per phase; mixed metre/radian units",
            "jacobian_by_phase": jacobian_by_phase,
        },
        "continuity": {
            "maximum_reference_joint_step_rad": float(np.max(np.abs(np.diff(q, axis=0)))),
            "maximum_reference_joint_speed_rad_s": float(np.max(np.abs(velocity))),
            "maximum_reference_joint_acceleration_rad_s2": float(np.max(np.abs(acceleration))),
            "maximum_reference_fk_position_residual_m": max(row["reference_fk_position_error_m"] for row in rows),
            "maximum_reference_fk_orientation_residual_rad": max(row["reference_fk_orientation_error_rad"] for row in rows),
            "minimum_joint_position_margin_rad": float(np.min(np.minimum(q - lower, upper - q))),
        },
        "collision": {
            "all_samples_screened": all(row["collision_screened"] for row in rows),
            "minimum_aabb_distance_lower_bound_m": min(row["minimum_aabb_distance_lower_bound_m"] for row in rows),
            "method": "URDF CAD convex hull CPU screening; AABB separation certificate with exact LP fallback",
            "physx_cooked_hull": "UNKNOWN",
        },
        "candidate_search": candidate_search,
        "home_pose_link6": home_pose.tolist(),
        "limits_used": {
            "reference_velocity_limit_rad_s": cfg["reference_velocity_limit_rad_s"],
            "reference_acceleration_limit_rad_s2": cfg["reference_acceleration_limit_rad_s2"],
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=REPO / "configs/b1_s1_circle_sim_only.json")
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    if cfg.get("parameter_class") != "SIM_ONLY":
        raise ValueError("B1-S1 configuration must be SIM_ONLY")
    if digest(args.urdf) != cfg["source_urdf_sha256"]:
        raise ValueError("URDF hash does not match the reviewed input")
    args.output.mkdir(parents=True, exist_ok=False)
    model = parse_urdf(args.urdf)
    names = [joint.name for joint in model.moving]
    lower, upper = limits(model)
    q_start = np.asarray(cfg["q_start_rad"], dtype=float)
    q_home = np.asarray(cfg["q_home_rad"], dtype=float)
    if names != cfg["joint_names"] or np.any(q_home < lower) or np.any(q_home > upper):
        raise ValueError("joint names or home pose do not match URDF limits")
    geometry = CollisionModel(args.urdf)
    if not collision_status(geometry, q_start)[0] or not collision_status(geometry, q_home)[0]:
        raise ValueError("start/home pose did not pass collision screening")

    candidates = []
    for radius in cfg["candidate_radii_m"]:
        for period in cfg["candidate_periods_s"]:
            try:
                candidate = circle_ik(model, names, q_home, radius, period, cfg["circle_ramp_duration_s"],
                                      lower, upper, geometry)
                candidate["velocity_limit_pass"] = candidate["maximum_coarse_joint_speed_rad_s"] <= cfg["reference_velocity_limit_rad_s"]
                candidate["status"] = "PASS" if candidate["velocity_limit_pass"] else "REJECTED_SPEED"
            except Exception as exc:
                candidate = {"radius_m": radius, "period_s": period, "status": "BLOCKED", "reason": str(exc)}
            candidates.append(candidate)
    acceptable = [c for c in candidates if c["status"] == "PASS"]
    if not acceptable:
        raise RuntimeError("no circle candidate passed the CPU IK/collision screen")
    selected = next((c for c in acceptable
                     if abs(c["radius_m"] - cfg["selected_radius_m"]) < 1e-12
                     and abs(c["period_s"] - cfg["selected_period_s"]) < 1e-12), None)
    if selected is None:
        raise RuntimeError("configured circle candidate did not pass")
    home_pose = np.asarray(model.fk(dict(zip(names, q_home)))['link6'], dtype=float)
    selected = dict(selected)
    rows, center, home_pose = build_rows(model, geometry, cfg, selected, names, q_start, q_home, lower, upper)
    selected["center_m"] = center
    q_velocity = np.asarray([row["q_reference_velocity_rad_s"] for row in rows])
    q_acceleration = np.asarray([row["q_reference_acceleration_rad_s2"] for row in rows])
    if (np.max(np.abs(q_velocity)) > cfg["reference_velocity_limit_rad_s"]
            or np.max(np.abs(q_acceleration)) > cfg["reference_acceleration_limit_rad_s2"]):
        raise RuntimeError("full trajectory violates SIM_ONLY reference derivative limits")
    summary = summarize(rows, cfg, selected, candidates, q_home, home_pose, lower, upper, model, names)
    if summary["ik"]["jacobian_by_phase"]["circle"]["minimum_singular_value"] < cfg["minimum_circle_jacobian_sigma"]:
        raise RuntimeError("circle enters the SIM_ONLY singularity stop region")
    with (args.output / "trajectory.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False) + "\n")
    summary["trajectory_sha256"] = digest(args.output / "trajectory.jsonl")
    summary["configuration_sha256"] = digest(args.config)
    write_json(args.output / "trajectory_config.json", cfg)
    write_json(args.output / "summary.json", summary)
    print(json.dumps({"status": summary["status"], "sample_count": len(rows),
                      "radius_m": selected["radius_m"], "period_s": selected["period_s"],
                      "max_speed_rad_s": summary["continuity"]["maximum_reference_joint_speed_rad_s"]}))


if __name__ == "__main__":
    main()
