"""Run with: python examples/reliable_team.py

Demonstrates a two-task civilian robotics mission with a safety boundary,
priority-first allocation, signed telemetry, and an exportable audit trail.
"""

from swarmlink import (
    CircularKeepOut, CoordinationAuditLog, NodeProfile, NodeTelemetry,
    OperationalBounds, PacketAuthenticator, Position, SafetyPolicy,
    TaskCoordinator, TaskKind, TaskRequest, VehicleKind, Velocity,
)

now = 100.0
nodes = [
    NodeTelemetry("drone-camera", now, Position(2, 0, 8), Velocity(3, 0), VehicleKind.DRONE, battery_percent=88),
    NodeTelemetry("rover-lidar", now, Position(20, 3), Velocity(1, 0), VehicleKind.ROVER, battery_percent=92),
    NodeTelemetry("drone-observer", now, Position(-4, 2, 9), Velocity(2, 0), VehicleKind.DRONE, battery_percent=74),
]
profiles = [
    NodeProfile("drone-camera", capabilities=frozenset({"camera"})),
    NodeProfile("rover-lidar", capabilities=frozenset({"lidar"})),
    NodeProfile("drone-observer", capabilities=frozenset({"camera", "observer"})),
]

# Each real node would receive a shared secret through a secure deployment step,
# never hard-coded as done here for a short local demonstration.
auth = PacketAuthenticator("demo-only-secret-change-before-use")
assert auth.decode(auth.encode(nodes[0], issued_at_s=now, nonce="demo-1"), now_s=now).node_id == "drone-camera"

safety = SafetyPolicy(
    bounds=OperationalBounds(-30, 30, -30, 30, 0, 30),
    keep_outs=[CircularKeepOut(Position(0, 0, 0), 3, "charging pad")],
)
coordinator = TaskCoordinator(safety_policy=safety)
tasks = [
    TaskRequest("inspect-panel", TaskKind.INSPECTION, Position(8, 0, 8), now, priority=90, required_capabilities=frozenset({"camera"})),
    TaskRequest("map-corridor", TaskKind.MAPPING, Position(18, 4, 0), now, priority=40, required_capabilities=frozenset({"lidar"})),
]

batch = coordinator.plan_many(tasks, nodes, profiles=profiles, now_s=now)
audit = CoordinationAuditLog()
for plan in batch.plans:
    audit.record_plan(now, plan)
    print(plan.to_dict())

audit.write_jsonl("team-audit.jsonl")
print("Wrote team-audit.jsonl")
