"""Pure, hardware-independent multi-robot coordination primitives.

Coordinates use a local ENU frame: x=east, y=north, z=up, all in metres.
All telemetry timestamps are Unix seconds (UTC).  This module deliberately
does not open sockets or command motors; transports and vehicle adapters sit
outside the safety-neutral core.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import json
import math
from pathlib import Path
from typing import Iterable


class VehicleKind(str, Enum):
    DRONE = "drone"
    ROVER = "rover"
    SENSOR = "sensor"
    BOAT = "boat"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Position:
    """Position in a local east-north-up frame, in metres."""

    x_m: float
    y_m: float
    z_m: float = 0.0

    def distance_to(self, other: "Position") -> float:
        return math.dist((self.x_m, self.y_m, self.z_m), (other.x_m, other.y_m, other.z_m))


@dataclass(frozen=True)
class Velocity:
    """Velocity in local east-north-up axes, in metres per second."""

    x_mps: float = 0.0
    y_mps: float = 0.0
    z_mps: float = 0.0

    @property
    def speed_mps(self) -> float:
        return math.sqrt(self.x_mps**2 + self.y_mps**2 + self.z_mps**2)


@dataclass(frozen=True)
class NodeTelemetry:
    """A single public status update emitted by one robot or sensor station."""

    node_id: str
    timestamp_s: float
    position: Position
    velocity: Velocity = field(default_factory=Velocity)
    vehicle_kind: VehicleKind = VehicleKind.UNKNOWN
    battery_percent: float | None = None
    heading_deg: float | None = None
    mission_state: str = "idle"
    health: dict[str, float | str | bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.node_id or len(self.node_id) > 64:
            raise ValueError("node_id must contain 1–64 characters")
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if self.battery_percent is not None and not 0 <= self.battery_percent <= 100:
            raise ValueError("battery_percent must be within 0..100")

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload["vehicle_kind"] = self.vehicle_kind.value
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "NodeTelemetry":
        position = payload.get("position")
        velocity = payload.get("velocity", {})
        if not isinstance(position, dict) or not isinstance(velocity, dict):
            raise ValueError("telemetry requires position and optional velocity mappings")
        return cls(
            node_id=str(payload["node_id"]),
            timestamp_s=float(payload["timestamp_s"]),
            position=Position(**position),
            velocity=Velocity(**velocity),
            vehicle_kind=VehicleKind(payload.get("vehicle_kind", "unknown")),
            battery_percent=(None if payload.get("battery_percent") is None else float(payload["battery_percent"])),
            heading_deg=(None if payload.get("heading_deg") is None else float(payload["heading_deg"])),
            mission_state=str(payload.get("mission_state", "idle")),
            health=dict(payload.get("health", {})),
        )


@dataclass(frozen=True)
class CollisionAlert:
    node_a: str
    node_b: str
    predicted_closest_distance_m: float
    time_to_closest_s: float
    threshold_m: float

    @property
    def severity(self) -> str:
        ratio = self.predicted_closest_distance_m / self.threshold_m
        return "critical" if ratio <= 0.5 else "warning"

    def to_dict(self) -> dict[str, object]:
        return asdict(self) | {"severity": self.severity}


@dataclass(frozen=True)
class FormationCommand:
    follower_id: str
    leader_id: str
    target: Position
    spacing_m: float
    slot_index: int

    def to_dict(self) -> dict[str, object]:
        return {**asdict(self), "target": asdict(self.target)}


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _closest_approach(a: NodeTelemetry, b: NodeTelemetry, horizon_s: float) -> tuple[float, float]:
    relative_position = (
        a.position.x_m - b.position.x_m,
        a.position.y_m - b.position.y_m,
        a.position.z_m - b.position.z_m,
    )
    relative_velocity = (
        a.velocity.x_mps - b.velocity.x_mps,
        a.velocity.y_mps - b.velocity.y_mps,
        a.velocity.z_mps - b.velocity.z_mps,
    )
    velocity_squared = _dot(relative_velocity, relative_velocity)
    if velocity_squared == 0:
        time_s = 0.0
    else:
        time_s = max(0.0, min(horizon_s, -_dot(relative_position, relative_velocity) / velocity_squared))
    projected = tuple(p + time_s * v for p, v in zip(relative_position, relative_velocity))
    return math.sqrt(_dot(projected, projected)), time_s


def predict_collisions(
    nodes: Iterable[NodeTelemetry], *, threshold_m: float = 5.0, horizon_s: float = 15.0
) -> list[CollisionAlert]:
    """Predict pairwise constant-velocity closest approaches within a horizon.

    This is a planning aid, not an aircraft separation system.  Feed each
    vehicle only after its coordinate frame and clock have been aligned.
    """
    if threshold_m <= 0 or horizon_s <= 0:
        raise ValueError("threshold_m and horizon_s must be positive")
    items = list(nodes)
    alerts: list[CollisionAlert] = []
    for index, first in enumerate(items):
        for second in items[index + 1 :]:
            distance, time_s = _closest_approach(first, second, horizon_s)
            if distance <= threshold_m:
                alerts.append(CollisionAlert(first.node_id, second.node_id, distance, time_s, threshold_m))
    return sorted(alerts, key=lambda alert: (alert.time_to_closest_s, alert.predicted_closest_distance_m))


def leader_follower_commands(
    leader: NodeTelemetry, followers: Iterable[NodeTelemetry], *, spacing_m: float = 4.0
) -> list[FormationCommand]:
    """Create alternating left/right slots behind a leader's current heading."""
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    heading_deg = leader.heading_deg
    if heading_deg is None:
        # In ENU, atan2(east, north) gives compass heading clockwise from north.
        heading_deg = math.degrees(math.atan2(leader.velocity.x_mps, leader.velocity.y_mps)) if leader.velocity.speed_mps else 0.0
    heading = math.radians(heading_deg)
    forward_x, forward_y = math.sin(heading), math.cos(heading)
    right_x, right_y = math.cos(heading), -math.sin(heading)
    commands: list[FormationCommand] = []
    for index, follower in enumerate(followers, start=1):
        row = (index + 1) // 2
        side = -1 if index % 2 else 1
        target = Position(
            leader.position.x_m - row * spacing_m * forward_x + side * spacing_m * right_x,
            leader.position.y_m - row * spacing_m * forward_y + side * spacing_m * right_y,
            leader.position.z_m,
        )
        commands.append(FormationCommand(follower.node_id, leader.node_id, target, spacing_m, index))
    return commands


class NodeRegistry:
    """In-memory last-known state of a robot team, independent of transport."""

    def __init__(self) -> None:
        self._nodes: dict[str, NodeTelemetry] = {}

    def ingest(self, telemetry: NodeTelemetry) -> bool:
        """Store a newer status. Returns False for an out-of-order status."""
        previous = self._nodes.get(telemetry.node_id)
        if previous is not None and telemetry.timestamp_s < previous.timestamp_s:
            return False
        self._nodes[telemetry.node_id] = telemetry
        return True

    def get(self, node_id: str) -> NodeTelemetry | None:
        return self._nodes.get(node_id)

    def snapshot(self) -> list[NodeTelemetry]:
        return sorted(self._nodes.values(), key=lambda node: node.node_id)

    def stale(self, now_s: float, *, max_age_s: float = 3.0) -> list[NodeTelemetry]:
        if max_age_s < 0:
            raise ValueError("max_age_s cannot be negative")
        return [node for node in self.snapshot() if now_s - node.timestamp_s > max_age_s]

    def collision_alerts(self, *, threshold_m: float = 5.0, horizon_s: float = 15.0) -> list[CollisionAlert]:
        return predict_collisions(self.snapshot(), threshold_m=threshold_m, horizon_s=horizon_s)


class MissionLog:
    """Append-only JSON Lines recorder for reproducible simulated or real missions."""

    schema_version = "0.1"

    def __init__(self) -> None:
        self.records: list[NodeTelemetry] = []

    def append(self, telemetry: NodeTelemetry) -> None:
        self.records.append(telemetry)

    def write_jsonl(self, path: str | Path) -> Path:
        destination = Path(path)
        with destination.open("w", encoding="utf-8") as handle:
            for telemetry in self.records:
                handle.write(json.dumps({"schema_version": self.schema_version, "telemetry": telemetry.to_dict()}, sort_keys=True) + "\n")
        return destination

    @classmethod
    def read_jsonl(cls, path: str | Path) -> "MissionLog":
        log = cls()
        with Path(path).open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    payload = json.loads(line)
                    log.append(NodeTelemetry.from_dict(payload["telemetry"]))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                    raise ValueError(f"invalid mission record on line {line_number}: {error}") from error
        return log
