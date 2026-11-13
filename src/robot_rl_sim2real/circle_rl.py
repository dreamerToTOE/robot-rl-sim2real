"""Small, CPU-testable B2-S0 observation/action/reward contract (SIM_ONLY)."""

from __future__ import annotations

import math
import numpy as np

from .circle_trajectory import phase_ramp_integral, quintic_blend


def reference_at(time_s: float, circle: dict, ramp_s: float, hold_s: float) -> dict:
    """B1-S1's smooth startup, one constant-speed circle, stop and hold."""
    if not math.isfinite(time_s) or time_s < 0 or ramp_s <= 0 or hold_s < 0:
        raise ValueError("invalid reference time")
    period, radius = circle["period_s"], circle["radius_m"]
    omega = 2 * math.pi / period
    if time_s < ramp_s:
        s = time_s / ramp_s
        phase = -omega * ramp_s / 2 + omega * ramp_s * phase_ramp_integral(s)
        speed, segment = omega * quintic_blend(s), "startup"
    elif time_s <= ramp_s + period:
        phase = omega * (time_s - ramp_s)
        speed, segment = omega, "circle"
    elif time_s < 2 * ramp_s + period:
        s = (time_s - ramp_s - period) / ramp_s
        phase = 2 * math.pi + omega * ramp_s * (s - phase_ramp_integral(s))
        speed, segment = omega * (1 - quintic_blend(s)), "stop"
    else:
        phase = 2 * math.pi + omega * ramp_s / 2
        speed, segment = 0.0, "hold"
    position = np.asarray(circle["center_m"]) + radius * np.array([math.cos(phase), math.sin(phase), 0])
    velocity = radius * speed * np.array([-math.sin(phase), math.cos(phase), 0])
    return {"position_m": position, "velocity_m_s": velocity, "phase_rad": phase, "segment": segment}


def map_action(raw, previous, target, lower, upper, cfg):
    """Accumulate a slew-limited position increment into the last target."""
    raw, previous, target = map(lambda x: np.asarray(x, dtype=float), (raw, previous, target))
    if raw.shape != previous.shape or raw.shape != target.shape or raw.shape[-1] != 6:
        raise ValueError("actions and targets must have matching (...,6) shapes")
    if (not np.isfinite(previous).all() or not np.isfinite(target).all()
            or not math.isfinite(cfg['action_scale_rad']) or cfg['action_scale_rad'] <= 0
            or not 0 < cfg['action_slew_per_step'] <= 2):
        raise ValueError("invalid action history")
    invalid = ~np.isfinite(raw).all(axis=-1) | (np.abs(np.nan_to_num(raw)) > 1 + 1e-6).any(axis=-1)
    bounded = np.clip(np.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0), -1, 1)
    applied = previous + np.clip(bounded - previous, -cfg["action_slew_per_step"], cfg["action_slew_per_step"])
    applied = np.where(np.asarray(invalid)[..., None], 0.0, applied)
    desired = target + cfg["action_scale_rad"] * applied
    command = np.clip(desired, lower, upper)
    # The next observation/reward describes the increment actually issued after all limits.
    effective = (command - target) / cfg["action_scale_rad"]
    return {"bounded": bounded, "slewed": applied, "effective": effective,
            "target": command, "invalid": invalid, "position_clipped": np.any(desired != command, axis=-1)}


def observation(q, dq, position, reference_position, reference_velocity, previous_action, target, elapsed_s, home, cfg, circle):
    result = np.concatenate(((np.asarray(q) - home) / cfg["local_joint_radius_rad"],
                             np.asarray(dq) / cfg["velocity_observation_scale_rad_s"],
                             (np.asarray(reference_position) - position) / cfg["position_observation_scale_m"],
                             np.asarray(reference_velocity) / (circle["radius_m"] * 2 * math.pi / circle["period_s"]),
                             np.asarray(previous_action),
                             (np.asarray(target) - q) / cfg["target_observation_scale_rad"],
                             np.expand_dims(np.asarray(elapsed_s) / cfg["episode_duration_s"], axis=-1)), axis=-1).astype(np.float32)
    if result.shape[-1] != 31 or not np.isfinite(result).all():
        raise ValueError("observation must contain 31 finite values")
    return result


def reward_components(error_m, action, previous_action, maximum_steps, cfg):
    """Three terms, with strictly positive step reward to avoid profiting from early exit.

    ``clipped_quadratic`` is the B2-S0 contract.  B2-S1 may select
    ``exp_linear`` to keep a position signal above the 5 mm scale while
    retaining the same action costs and positive floor.
    """
    error_m = np.asarray(error_m, dtype=float)
    action, previous_action = np.asarray(action), np.asarray(previous_action)
    floor, amplitude_weight, change_weight = (cfg['tracking_reward_floor'], cfg['action_cost_weight'], cfg['action_change_cost_weight'])
    mode = cfg.get('tracking_reward_mode', 'clipped_quadratic')
    if (not all(math.isfinite(x) for x in (floor, amplitude_weight, change_weight, cfg['tracking_reward_scale_m']))
            or amplitude_weight < 0 or change_weight < 0 or cfg['tracking_reward_scale_m'] <= 0
            or floor <= amplitude_weight + 4*change_weight
            or mode not in ('clipped_quadratic', 'exp_linear')):
        raise ValueError('tracking floor must exceed the maximum action costs')
    if (maximum_steps <= 0 or not np.isfinite(error_m).all() or np.any(error_m < 0)
            or not np.isfinite(action).all() or not np.isfinite(previous_action).all()
            or np.any(np.abs(action) > 1+1e-6) or np.any(np.abs(previous_action) > 1+1e-6)):
        raise ValueError("invalid reward input")
    scale = 1.0 / maximum_steps
    normalized_error = error_m / cfg["tracking_reward_scale_m"]
    if mode == 'clipped_quadratic':
        tracking_shape = 1.0 - np.minimum(normalized_error ** 2, 1.0)
    else:
        tracking_shape = np.exp(-normalized_error)
    position = scale * (cfg["tracking_reward_floor"] + tracking_shape)
    amplitude = -scale * cfg["action_cost_weight"] * np.mean(action**2, axis=-1)
    change = -scale * cfg["action_change_cost_weight"] * np.mean((action - previous_action)**2, axis=-1)
    return {"tracking": position, "action": amplitude, "action_change": change,
            "total": position + amplitude + change}


def evaluate_circle_success(records, *, rmse_threshold_m=0.005,
                            p95_threshold_m=0.005, max_threshold_m=0.008,
                            phase_span_tolerance_rad=1e-6):
    """Evaluate one complete SIM_ONLY episode with frozen circle criteria.

    A timeout is an acceptable episode end, but it cannot make an episode
    successful without a full circle and all three error limits.
    """
    if not records or any(not isinstance(row, dict) for row in records):
        raise ValueError("records must be a non-empty sequence of mappings")
    limits = (rmse_threshold_m, p95_threshold_m, max_threshold_m)
    if any(not math.isfinite(x) or x <= 0 for x in limits) or phase_span_tolerance_rad < 0:
        raise ValueError("invalid success thresholds")
    circle = [row for row in records if row.get("reference_segment") == "circle"]
    errors = np.asarray([row.get("position_error_m", float("nan")) for row in circle], dtype=float)
    phases = np.asarray([row.get("reference_phase_rad", float("nan")) for row in circle], dtype=float)
    finite = errors.size > 0 and np.isfinite(errors).all() and np.isfinite(phases).all()
    phase_span = float(phases.max() - phases.min()) if finite else 0.0
    complete_circle = finite and phase_span >= 2 * math.pi - phase_span_tolerance_rad
    rmse = float(np.sqrt(np.mean(errors ** 2))) if finite else float("nan")
    p95 = float(np.percentile(errors, 95)) if finite else float("nan")
    maximum = float(np.max(errors)) if finite else float("nan")
    terminated = any(bool(row.get("terminated", False)) for row in records)
    truncated = bool(records[-1].get("truncated", False))
    ended = terminated or truncated
    success = (complete_circle and ended and not terminated and truncated
               and rmse <= rmse_threshold_m and p95 <= p95_threshold_m
               and maximum <= max_threshold_m)
    return {"success": bool(success), "complete_circle": bool(complete_circle),
            "circle_phase_span_rad": phase_span, "circle_rmse_m": rmse,
            "circle_p95_m": p95, "circle_max_m": maximum,
            "terminated": terminated, "truncated": truncated,
            "criterion": {"phase_span_rad": 2 * math.pi,
                          "rmse_threshold_m": rmse_threshold_m,
                          "p95_threshold_m": p95_threshold_m,
                          "max_threshold_m": max_threshold_m}}
