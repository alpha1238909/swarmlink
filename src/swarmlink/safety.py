"""Transparent map bounds and keep-out checks for high-level task targets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .core import Position


@dataclass(frozen=True)
class OperationalBounds:
    """Inclusive local ENU operating bounds, in metres."""

    min_x_m: float
    max_x_m: float
    min_y_m: float
    max_y_m: float
    min_z_m: float = -float("inf")
    max_z_m: float = float("inf")

    def __post_init__(self) -> None:
        if self.min_x_m > self.max_x_m or self.min_y_m > self.max_y_m or self.min_z_m > self.max_z_m:
            raise ValueError("each operational minimum must not exceed its maximum")

    def contains(self, point: Position) -> bool:
        return (
            self.min_x_m <= point.x_m <= self.max_x_m
            and self.min_y_m <= point.y_m <= self.max_y_m
            and self.min_z_m <= point.z_m <= self.max_z_m
        )


@dataclass(frozen=True)
class CircularKeepOut:
    """A circular region no task target may enter."""

    centre: Position
    radius_m: float
    label: str = "keep-out zone"

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("radius_m must be positive")

    def contains(self, point: Position) -> bool:
        return self.centre.distance_to(point) < self.radius_m


class SafetyPolicy:
    """Pure validation policy; it never sends commands or overrides a vehicle."""

    def __init__(self, *, bounds: OperationalBounds | None = None, keep_outs: Iterable[CircularKeepOut] = ()) -> None:
        self.bounds = bounds
        self.keep_outs = tuple(keep_outs)

    def validate_target(self, target: Position) -> str | None:
        if self.bounds is not None and not self.bounds.contains(target):
            return "target outside operational bounds"
        for zone in self.keep_outs:
            if zone.contains(target):
                return f"target inside {zone.label}"
        return None
