#!/usr/bin/env python3
"""Evaluate saved B1-S1 states and publish numerical extracts and SIM_ONLY plots."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from run_circle_trajectory import summary
from validate_sim_robot import write_json


def load_run(directory):
    rows = [json.loads(line) for line in (directory / "states.jsonl").read_text().splitlines()]
    cfg = json.loads((directory / "input_config.json").read_text())
    result = json.loads((directory / "result.json").read_text())
    if result["status"] != "PASS" or len(rows) != result["sample_count"]:
        raise ValueError(f"run is incomplete or failed: {directory.name}")
    metrics = summary(rows, cfg, cfg["joint_names"], result["contacts"], result["failures"])
    metrics["controller_mode"] = cfg["controller_mode"]
    metrics["raw_states_sha256"] = hashlib.sha256((directory / "states.jsonl").read_bytes()).hexdigest()
    metrics["runtime_metadata"] = json.loads((directory / "runtime_metadata.json").read_text())
    metrics["by_phase"] = {}
    for phase in dict.fromkeys(row["phase"] for row in rows):
        group = [row for row in rows if row["phase"] == phase]
        errors = np.array([row["position_error_m"] for row in group])
        last = [row for row in group if row["time_s"] >= group[-1]["time_s"]-0.25]
        metrics["by_phase"][phase] = {
            "sample_count": len(group), "start_time_s": group[0]["time_s"], "end_time_s": group[-1]["time_s"],
            "rmse_position_m": float(np.sqrt(np.mean(errors**2))),
            "p95_position_m": float(np.quantile(errors, 0.95)), "maximum_position_m": float(np.max(errors)),
            "maximum_abs_velocity_rad_s": float(np.max(np.abs([row["dq_actual_rad_s"] for row in group]))),
            "maximum_abs_acceleration_rad_s2": float(np.max(np.abs([row["ddq_actual_rad_s2"] for row in group]))),
            "maximum_abs_effort_nm": float(np.max(np.abs([row["joint_effort_nm"] for row in group]))),
            "steady_max_joint_error_rad": float(np.max(np.abs([row["tracking_error_rad"] for row in last]))),
        }
    circle = [row for row in rows if row["phase"] == "circle"]
    metrics["tracking"]["mean_actual_minus_reference_position_m"] = np.mean([
        np.asarray(row["actual_link6_position_m"])-row["reference_position_m"] for row in circle], axis=0).tolist()
    return rows, cfg, metrics


def publish_samples(path, rows):
    indices = set(range(0, len(rows), 24)) | {len(rows)-1}
    for i in range(1, len(rows)):
        if rows[i]["phase"] != rows[i-1]["phase"]:
            indices.update((i-1, i))
    with path.open("w") as stream:
        for index in sorted(indices):
            stream.write(json.dumps(rows[index], ensure_ascii=False, allow_nan=False)+"\n")


def plots(baseline, comparison, cfg, figures):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.25})
    center = np.asarray(cfg["circle_center_m"])
    circle = [row for row in baseline if row["phase"] == "circle"]
    reference = (np.array([row["reference_position_m"] for row in circle])-center)*1000
    actual = (np.array([row["actual_link6_position_m"] for row in circle])-center)*1000
    fig, ax = plt.subplots(figsize=(7.4, 6.2), constrained_layout=True)
    ax.plot(reference[:, 0], reference[:, 1], "k--", linewidth=1.7, label="Reference (r=10 mm)")
    ax.plot(actual[:, 0], actual[:, 1], color="#1769aa", linewidth=1.7, label="Position PD baseline")
    if comparison:
        comp = (np.array([row["actual_link6_position_m"] for row in comparison if row["phase"] == "circle"])-center)*1000
        ax.plot(comp[:, 0], comp[:, 1], color="#d55e00", linewidth=1.4, label="PD + model gravity (SIM_ONLY)")
    ax.scatter(*reference[0, :2], color="black", marker="o", s=25, zorder=5)
    ax.set(xlabel="X relative to reference center [mm]", ylabel="Y relative to reference center [mm]",
           title="SIM_ONLY: link6 XY trajectory, one constant-speed circle")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right", fontsize=9)
    fig.savefig(figures / "B1_S1_XY.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9.5, 6.7), constrained_layout=True)
    for rows, label, color in ((baseline, "Position PD baseline", "#1769aa"),
                               (comparison, "PD + model gravity (SIM_ONLY)", "#d55e00")):
        if not rows:
            continue
        axes[0].plot([row["time_s"] for row in rows], [row["position_error_m"]*1000 for row in rows],
                     label=label, color=color, linewidth=1.3)
        selected = [row for row in rows if row["phase"] == "circle"]
        start = selected[0]["time_s"]
        axes[1].plot([row["time_s"]-start for row in selected],
                     [row["position_error_m"]*1000 for row in selected], label=label, color=color, linewidth=1.4)
    t0, t1 = circle[0]["time_s"], circle[-1]["time_s"]
    axes[0].axvspan(t0, t1, color="grey", alpha=0.10, label="Constant-speed circle")
    axes[0].set(xlabel="Simulation time [s]", ylabel="3D position error [mm]",
                title="SIM_ONLY: ingress, circle, stop and return")
    axes[1].set(xlabel="Time since constant-speed circle start [s]", ylabel="3D position error [mm]",
                title="SIM_ONLY: circle tracking error from actual q and independent FK")
    axes[0].legend(loc="lower right", fontsize=9)
    axes[1].legend(loc="lower right", fontsize=9)
    fig.savefig(figures / "B1_S1_ERROR_TIME.png", dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--gravity-control", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--figures", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    args.figures.mkdir(parents=True, exist_ok=True)
    baseline, cfg, metrics = load_run(args.baseline)
    write_json(args.output / "B1_S1_BASELINE_SUMMARY.json", metrics)
    publish_samples(args.output / "B1_S1_BASELINE_10HZ.jsonl", baseline)
    comparison = None
    if args.gravity_control:
        comparison, comp_cfg, comp_metrics = load_run(args.gravity_control)
        if (comp_metrics["runtime_metadata"]["trajectory_sha256"] != metrics["runtime_metadata"]["trajectory_sha256"]
                or comp_cfg["controller"] != cfg["controller"] or len(comparison) != len(baseline)):
            raise ValueError("gravity comparison changed the reference or PD parameters")
        write_json(args.output / "B1_S1_GRAVITY_SUMMARY.json", comp_metrics)
        publish_samples(args.output / "B1_S1_GRAVITY_10HZ.jsonl", comparison)
        write_json(args.output / "B1_S1_GRAVITY_COMPARISON.json", {
            "parameter_class": "SIM_ONLY", "same_reference_and_pd_parameters": True,
            "baseline_rmse_m": metrics["tracking"]["rmse_position_m"],
            "gravity_compensated_rmse_m": comp_metrics["tracking"]["rmse_position_m"],
            "rmse_reduction_fraction": 1-comp_metrics["tracking"]["rmse_position_m"]/metrics["tracking"]["rmse_position_m"],
            "note": "PhysX model gravity feedforward, uncalibrated CAD parameters; original PD baseline retained",
        })
    plots(baseline, comparison, cfg, args.figures)
    print(json.dumps({"status": metrics["status"], "baseline_rmse_m": metrics["tracking"]["rmse_position_m"]}))


if __name__ == "__main__":
    main()
