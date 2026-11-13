"""CPU-only gates and descriptive summaries for B1-S0 simulation JSONL rows.

All thresholds are SIM_ONLY experiment settings. Torque values retain their
source label: an actuator estimate is not a hardware torque measurement.
"""

import math


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, count):
    return isinstance(value, (list, tuple)) and len(value) == count and all(_finite(x) for x in value)


def _check_limits(limits):
    names = limits["joint_names"]
    count = len(names)
    if (limits.get("parameter_class") != "SIM_ONLY" or not count
            or len(set(names)) != count or not all(isinstance(n, str) and n for n in names)):
        raise ValueError("Expected unique joint names and SIM_ONLY thresholds")
    for key in ("lower_rad", "upper_rad", "max_velocity_rad_s", "effort_limit_nm"):
        if not _vector(limits[key], count):
            raise ValueError(f"Invalid threshold vector: {key}")
    if any(lo >= hi for lo, hi in zip(limits["lower_rad"], limits["upper_rad"])):
        raise ValueError("Invalid position limits")
    if any(x <= 0 for key in ("max_velocity_rad_s", "effort_limit_nm") for x in limits[key]):
        raise ValueError("Velocity and effort thresholds must be positive")
    for key in ("fk_position_tolerance_m", "fk_orientation_tolerance_rad"):
        if not _finite(limits[key]) or limits[key] <= 0:
            raise ValueError(f"Invalid positive threshold: {key}")
    for key, default in (("position_limit_tolerance_rad", 1e-6), ("steady_window_s", 0.25)):
        if not _finite(limits.get(key, default)) or limits.get(key, default) < 0:
            raise ValueError(f"Invalid threshold: {key}")
    return count


def validate_sample(sample, limits):
    """Return stop reasons; malformed/missing required evidence fails closed."""
    count = _check_limits(limits)
    reasons = []
    if sample.get("joint_names") != limits["joint_names"]:
        reasons.append("joint_mapping_mismatch")
    for key in ("q_target_rad", "q_actual_rad", "dq_actual_rad_s", "joint_effort_nm"):
        if not _vector(sample.get(key), count):
            reasons.append(f"invalid_{key}")
    for key in ("computed_torque_nm", "projected_joint_force_nm"):
        if key in sample and not _vector(sample[key], count):
            reasons.append(f"invalid_{key}")
    for key in ("time_s", "fk_position_error_m", "fk_orientation_error_rad"):
        if not _finite(sample.get(key)) or sample[key] < 0:
            reasons.append(f"invalid_{key}")
    for key in ("phase", "effort_source", "collision_source"):
        if not isinstance(sample.get(key), str) or not sample[key].strip():
            reasons.append(f"missing_{key}")
    if not isinstance(sample.get("collision_detected"), bool):
        reasons.append("missing_collision_check")
    elif sample["collision_detected"]:
        reasons.append("collision_detected")
    # Invalid vectors/scalars cannot be used for subsequent numerical checks.
    if any(reason.startswith("invalid_") for reason in reasons):
        return reasons
    for key in ("q_target_rad", "q_actual_rad"):
        tolerance = limits.get("position_limit_tolerance_rad", 1e-6)
        if any(q < lo - tolerance or q > hi + tolerance for q, lo, hi in
               zip(sample[key], limits["lower_rad"], limits["upper_rad"])):
            reasons.append(f"{key}_outside_limits")
    if any(abs(v) > maximum for v, maximum in zip(sample["dq_actual_rad_s"], limits["max_velocity_rad_s"])):
        reasons.append("abnormal_velocity")
    if any(abs(e) > maximum + 1e-6 for e, maximum in zip(sample["joint_effort_nm"], limits["effort_limit_nm"])):
        reasons.append("applied_effort_exceeds_limit")
    if sample["fk_position_error_m"] > limits["fk_position_tolerance_m"]:
        reasons.append("fk_position_mismatch")
    if sample["fk_orientation_error_rad"] > limits["fk_orientation_tolerance_rad"]:
        reasons.append("fk_orientation_mismatch")
    return reasons


def summarize_samples(samples, limits):
    """Summarize finite, checked rows; metrics alone do not certify a pass.

    Each phase must have a constant target and start before its response. Error
    sign changes above 1e-4 rad describe crossings, not proof of instability.
    """
    count = _check_limits(limits)
    rows = list(samples)
    stop_reasons = []
    previous_time = None
    groups = {}
    for index, row in enumerate(rows):
        reasons = validate_sample(row, limits)
        if _finite(row.get("time_s")):
            if previous_time is not None and row["time_s"] <= previous_time:
                reasons.append("nonmonotonic_timestamp")
            previous_time = row["time_s"]
        stop_reasons.extend({"sample": index, "reason": reason} for reason in reasons)
        if not reasons:
            groups.setdefault(row["phase"], []).append(row)
    phases, previous_target = {}, None
    for phase, phase_rows in groups.items():
        if any(row["q_target_rad"] != phase_rows[0]["q_target_rad"] for row in phase_rows):
            stop_reasons.append({"phase": phase, "reason": "target_changed_within_phase"})
            continue
        joints = {}
        target = phase_rows[0]["q_target_rad"]
        target_steps = [0] * count if previous_target is None else [q - old for q, old in zip(target, previous_target)]
        changed_count = sum(abs(step) > 1e-4 for step in target_steps)
        if changed_count > 1:
            stop_reasons.append({"phase": phase, "reason": "multiple_joint_targets_changed"})
        steady_start = phase_rows[-1]["time_s"] - limits.get("steady_window_s", 0.25)
        steady_rows = [row for row in phase_rows if row["time_s"] >= steady_start]
        settling_rows = [row for row in phase_rows if row["time_s"] < steady_start]
        for joint, name in enumerate(limits["joint_names"]):
            errors = [row["q_target_rad"][joint] - row["q_actual_rad"][joint] for row in phase_rows]
            signs = [math.copysign(1, e) for e in errors if abs(e) > 1e-4]
            desired_delta = phase_rows[0]["q_target_rad"][joint] - phase_rows[0]["q_actual_rad"][joint]
            actual_delta = phase_rows[-1]["q_actual_rad"][joint] - phase_rows[0]["q_actual_rad"][joint]
            efforts = [abs(row["joint_effort_nm"][joint]) for row in phase_rows]
            steady_errors = [row["q_target_rad"][joint] - row["q_actual_rad"][joint] for row in steady_rows]
            joints[name] = {
                "max_abs_error_rad": max(abs(e) for e in errors),
                "rms_error_rad": math.sqrt(sum(e * e for e in errors) / len(errors)),
                "final_error_rad": errors[-1],
                "max_abs_velocity_rad_s": max(abs(row["dq_actual_rad_s"][joint]) for row in phase_rows),
                "max_abs_effort_nm": max(efforts),
                "effort_limit_fraction": sum(e >= 0.99 * limits["effort_limit_nm"][joint] for e in efforts) / len(efforts),
                "target_delta_from_start_rad": desired_delta,
                "target_step_rad": target_steps[joint],
                "actual_delta_rad": actual_delta,
                "direction_matches": (actual_delta * target_steps[joint] > 0
                                      if changed_count == 1 and abs(target_steps[joint]) > 1e-4 else None),
                "error_sign_changes": sum(a != b for a, b in zip(signs, signs[1:])),
                "steady_max_abs_error_rad": max(abs(e) for e in steady_errors),
                "steady_rms_error_rad": math.sqrt(sum(e * e for e in steady_errors) / len(steady_errors)),
                "steady_max_abs_velocity_rad_s": max(abs(row["dq_actual_rad_s"][joint]) for row in steady_rows),
                "settling_max_abs_error_rad": max((abs(row["q_target_rad"][joint] - row["q_actual_rad"][joint])
                                                    for row in settling_rows), default=None),
            }
        phases[phase] = {
            "sample_count": len(phase_rows),
            "start_time_s": phase_rows[0]["time_s"],
            "end_time_s": phase_rows[-1]["time_s"],
            "steady_start_time_s": max(steady_start, phase_rows[0]["time_s"]),
            "steady_sample_count": len(steady_rows),
            "settling_sample_count": len(settling_rows),
            "effort_sources": sorted({row["effort_source"] for row in phase_rows}),
            "max_fk_position_error_m": max(row["fk_position_error_m"] for row in phase_rows),
            "max_fk_orientation_error_rad": max(row["fk_orientation_error_rad"] for row in phase_rows),
            "joints": joints,
        }
        previous_target = target
    if not rows:
        stop_reasons.append({"reason": "no_samples"})
    return {"sample_count": len(rows), "joint_count": count, "stop_reasons": stop_reasons, "phases": phases}
