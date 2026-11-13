"""Reference-trajectory math only.

This module has NO robot, ROS, SDK, Isaac, GPU or network connection.
It does not compute motor commands and is not a learned policy.
"""
from dataclasses import dataclass
from math import cos, sin, pi, isfinite


@dataclass(frozen=True)
class CartesianReference:
    position_m: tuple[float, float, float]
    velocity_mps: tuple[float, float, float]


@dataclass(frozen=True)
class CircleXY:
    center_m: tuple[float, float, float]
    radius_m: float
    period_s: float

    def __post_init__(self) -> None:
        if len(self.center_m) != 3 or not all(isfinite(v) for v in self.center_m):
            raise ValueError("center_m must be three finite floats")
        if not isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError("radius_m must be finite and > 0")
        if not isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("period_s must be finite and > 0")

    def sample(self, time_s: float) -> CartesianReference:
        if not isfinite(time_s):
            raise ValueError("time_s must be finite")
        x, y, z = self.center_m
        omega = 2 * pi / self.period_s
        theta = omega * time_s
        return CartesianReference(
            position_m=(x + self.radius_m * cos(theta), y + self.radius_m * sin(theta), z),
            velocity_mps=(-self.radius_m * omega * sin(theta), self.radius_m * omega * cos(theta), 0.0),
        )
