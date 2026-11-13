"""CPU-only fixed-orientation circular trajectory helpers.

The functions here operate on the existing lightweight URDF FK model.  They
do not import Isaac, ROS, an SDK, or any hardware interface.  Numerical
limits supplied by callers are experiment-only SIM_ONLY gates.
"""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from scipy.optimize import least_squares


def rotation_vector(matrix: np.ndarray) -> np.ndarray:
    """Return the principal rotation vector for a 3x3 rotation matrix."""
    matrix = np.asarray(matrix, dtype=float)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return np.array([(matrix[2, 1] - matrix[1, 2]) / 2.0,
                         (matrix[0, 2] - matrix[2, 0]) / 2.0,
                         (matrix[1, 0] - matrix[0, 1]) / 2.0])
    sine = math.sin(angle)
    if abs(sine) < 1e-8:
        # This branch is outside the small local motions used by B1-S1.
        values, vectors = np.linalg.eigh((matrix + np.eye(3)) / 2.0)
        axis = vectors[:, int(np.argmax(values))]
        return angle * axis
    return angle / (2.0 * sine) * np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ])


def quintic_blend(s: float) -> float:
    """Smooth step with zero velocity and acceleration at both ends."""
    if not math.isfinite(s):
        raise ValueError("blend parameter must be finite")
    s = min(1.0, max(0.0, s))
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def phase_ramp_integral(s: float) -> float:
    """Integral of the quintic velocity ramp, with value 1/2 at s=1."""
    if not math.isfinite(s) or not 0.0 <= s <= 1.0:
        raise ValueError("ramp parameter must be in [0, 1]")
    return s**4 * (2.5 - 3.0 * s + s * s)


def circle_position(center: Iterable[float], radius: float, phase: float) -> np.ndarray:
    """Return x/y circle position at phase in radians."""
    center = np.asarray(tuple(center), dtype=float)
    if (center.shape != (3,) or not np.isfinite(center).all()
            or not math.isfinite(radius) or radius <= 0 or not math.isfinite(phase)):
        raise ValueError("invalid circle")
    return center + np.array((radius * math.cos(phase), radius * math.sin(phase), 0.0))


def pose_residual(model, names: list[str], q: np.ndarray, target_position: np.ndarray,
                  target_rotation: np.ndarray, position_scale: float) -> np.ndarray:
    """Six-dimensional fixed-orientation residual for bounded IK."""
    transform = np.asarray(model.fk(dict(zip(names, q)))['link6'], dtype=float)
    return np.concatenate(((transform[:3, 3] - target_position) / position_scale,
                           rotation_vector(target_rotation.T @ transform[:3, :3])))


def solve_fixed_orientation(model, names: list[str], target_position: np.ndarray,
                            target_rotation: np.ndarray, q_initial: np.ndarray,
                            lower: np.ndarray, upper: np.ndarray,
                            position_scale: float = 0.01,
                            residual_tolerance: float = 1e-7) -> tuple[np.ndarray, float]:
    """Solve one bounded IK point, retaining the previous solution branch."""
    q_initial = np.asarray(q_initial, dtype=float)
    result = least_squares(
        lambda q: pose_residual(model, names, q, target_position, target_rotation, position_scale),
        q_initial,
        bounds=(lower, upper),
        x_scale='jac',
        max_nfev=1000,
        ftol=1e-13,
        xtol=1e-13,
        gtol=1e-13,
    )
    residual = float(np.linalg.norm(result.fun))
    if not result.success or residual > residual_tolerance:
        raise ValueError(f"fixed-orientation IK failed: success={result.success} residual={residual}")
    return result.x, residual


def numerical_jacobian(model, names: list[str], q: np.ndarray, epsilon: float = 1e-6) -> np.ndarray:
    """Finite-difference position/angular-velocity Jacobian for link6."""
    q = np.asarray(q, dtype=float)
    base = np.asarray(model.fk(dict(zip(names, q)))['link6'], dtype=float)
    jacobian = np.zeros((6, len(q)))
    for index in range(len(q)):
        shifted = q.copy()
        shifted[index] += epsilon
        next_pose = np.asarray(model.fk(dict(zip(names, shifted)))['link6'], dtype=float)
        jacobian[:3, index] = (next_pose[:3, 3] - base[:3, 3]) / epsilon
        jacobian[3:, index] = rotation_vector(base[:3, :3].T @ next_pose[:3, :3]) / epsilon
    return jacobian


def quintic_joint_path(start: np.ndarray, end: np.ndarray, count: int) -> np.ndarray:
    """Generate a joint-space ingress/egress path including both endpoints."""
    if count < 2:
        raise ValueError("path needs at least two samples")
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    if start.shape != end.shape:
        raise ValueError("path endpoints must have the same shape")
    return np.array([start + quintic_blend(i / (count - 1)) * (end - start)
                     for i in range(count)])


def circle_phase_count(period_s: float, dt_s: float) -> int:
    """Return an integer sample count for one closed circle."""
    count = round(period_s / dt_s)
    if count < 4 or abs(count * dt_s - period_s) > 1e-10:
        raise ValueError("period must be representable by the simulation timestep")
    return count


def jacobian_metrics(model, names: list[str], q: np.ndarray) -> dict:
    singular_values = np.linalg.svd(numerical_jacobian(model, names, q), compute_uv=False)
    return {
        "singular_values": singular_values.tolist(),
        "minimum_singular_value": float(singular_values[-1]),
        "condition_number": float(singular_values[0] / singular_values[-1]),
    }
