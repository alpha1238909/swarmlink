"""One owner approaches a routine inspection point; others keep their role.

Run with: python examples/exclusive_task.py
"""

from swarmlink import (
    NodeProfile, NodeTelemetry, Position, TaskCoordinator, TaskKind,
    TaskRequest, VehicleKind, Velocity,
)

nodes = [
    NodeTelemetry("drone-camera", 100.0, Position(8, 0, 10), Velocity(3, 0), VehicleKind.DRONE, battery_percent=83),
    NodeTelemetry("rover-map", 100.0, Position(10, 1, 0), Velocity(1, 0), VehicleKind.ROVER, battery_percent=95),
    NodeTelemetry("drone-observer", 100.0, Position(12, 0, 10), Velocity(0, 0), VehicleKind.DRONE, battery_percent=77),
]
profiles = [
    NodeProfile("drone-camera", capabilities=frozenset({"camera"})),
    NodeProfile("rover-map", capabilities=frozenset({"lidar"})),
    NodeProfile("drone-observer", capabilities=frozenset({"camera", "observer"})),
]
task = TaskRequest(
    "inspect-marker", TaskKind.INSPECTION, Position(15, 0, 8), 100.0,
    required_capabilities=frozenset({"camera"}), exclusion_radius_m=6,
)

plan = TaskCoordinator().plan(task, nodes, profiles=profiles, now_s=100.0)
for instruction in plan.instructions:
    print(instruction.to_dict())
