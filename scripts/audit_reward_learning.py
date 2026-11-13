#!/usr/bin/env python3
"""Create the small, public B2-S1 reward audit summary (SIM_ONLY).

The input is an ignored B2-S0 transition log.  Only aggregate statistics and
sampled reward curves are written to the tracked report directories.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


SCALE_M = 0.005
FLOOR = 0.03
N = 630


def shapes(errors_m: np.ndarray) -> dict[str, np.ndarray]:
    x = errors_m / SCALE_M
    return {
        "clipped_quadratic": FLOOR + 1.0 - np.minimum(x * x, 1.0),
        "exp_linear": FLOOR + np.exp(-x),
        "rational_squared": FLOOR + 1.0 / (1.0 + x) ** 2,
    }


def read_rows(path: Path) -> dict[str, np.ndarray]:
    groups: dict[str, list[float]] = {}
    for line in path.read_text().splitlines():
        row = json.loads(line)
        label = row.get("test")
        if label in {"zero_action", "random_action", "diagnostic_ik"} and "position_error_m" in row:
            groups.setdefault(label, []).append(float(row["position_error_m"]))
    return {key: np.asarray(value, dtype=float) for key, value in groups.items()}


def summarize(groups: dict[str, np.ndarray]) -> dict:
    result = {
        "parameter_class": "SIM_ONLY",
        "source": "runs/b2_s0/full_pd_005/transitions.jsonl",
        "position_scale_m": SCALE_M,
        "tracking_floor": FLOOR,
        "maximum_steps": N,
        "groups": {},
        "curve_values_per_step": {},
        "action_cost_bounds_per_step": {
            "zero_action": 0.0,
            "maximum_amplitude_cost": -0.005 / N,
            "maximum_change_cost": -0.020 / N,
            "maximum_total_action_cost": -0.025 / N,
        },
    }
    for label, errors in groups.items():
        result["groups"][label] = {
            "samples": int(errors.size),
            "fraction_at_or_below_5mm": float(np.mean(errors <= SCALE_M)),
            "fraction_above_5mm": float(np.mean(errors > SCALE_M)),
            "mean_error_m": float(np.mean(errors)),
            "p50_error_m": float(np.percentile(errors, 50)),
            "p95_error_m": float(np.percentile(errors, 95)),
            "maximum_error_m": float(np.max(errors)),
        }
    grid = np.linspace(0.0, 0.04, 401)
    for name, values in shapes(grid).items():
        result["curve_values_per_step"][name] = {
            "error_m": grid.tolist(),
            "tracking_reward": (values / N).tolist(),
        }
    return result


def plot(summary: dict, output: Path) -> None:
    import matplotlib.pyplot as plt

    plt.figure(figsize=(7.0, 4.2), dpi=160)
    colors = {"clipped_quadratic": "#555555", "exp_linear": "#1769aa", "rational_squared": "#b24c00"}
    for name, values in summary["curve_values_per_step"].items():
        plt.plot(np.asarray(values["error_m"]) * 1000, np.asarray(values["tracking_reward"]) * N,
                 label=name, color=colors[name], linewidth=2)
    for label, color in (("zero_action", "#777777"), ("random_action", "#d62728"), ("diagnostic_ik", "#2ca02c")):
        if label in summary["groups"]:
            mean = summary["groups"][label]["mean_error_m"] * 1000
            plt.axvline(mean, color=color, linestyle="--", alpha=.6, linewidth=1,
                        label=f"{label} mean={mean:.1f} mm")
    plt.axvline(5, color="#222222", linestyle=":", linewidth=1)
    plt.xlabel("end-effector position error (mm)")
    plt.ylabel("tracking term x N (without action costs)")
    plt.xlim(0, 40)
    plt.ylim(0, 1.08)
    plt.grid(alpha=.2)
    plt.legend(fontsize=7)
    plt.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--transitions", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-figure", type=Path, required=True)
    args = parser.parse_args()
    summary = summarize(read_rows(args.transitions))
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    plot(summary, args.output_figure)
    print(json.dumps({"groups": summary["groups"], "output": str(args.output_figure)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
