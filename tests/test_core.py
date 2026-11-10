from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from swarmlink import (
    MissionLog,
    NodeRegistry,
    NodeTelemetry,
    Position,
    VehicleKind,
    Velocity,
    decode_packet,
    encode_packet,
    leader_follower_commands,
    predict_collisions,
)
from swarmlink.coordination import InstructionKind, NodeProfile, TaskCoordinator, TaskKind, TaskRequest, TaskStatus


def node(node_id: str, x: float, vx: float, *, timestamp: float = 10.0) -> NodeTelemetry:
    return NodeTelemetry(
        node_id=node_id,
        timestamp_s=timestamp,
        position=Position(x, 0.0, 0.0),
        velocity=Velocity(vx, 0.0, 0.0),
        vehicle_kind=VehicleKind.ROVER,
        battery_percent=80.0,
    )


class TelemetryTests(unittest.TestCase):
    def test_protocol_round_trip(self) -> None:
        original = node("rover-1", 4.0, 1.0)
        self.assertEqual(decode_packet(encode_packet(original)), original)

    def test_packet_rejects_unknown_type(self) -> None:
        with self.assertRaises(ValueError):
            decode_packet('{"type":"other"}')

    def test_registry_rejects_older_state(self) -> None:
        registry = NodeRegistry()
        self.assertTrue(registry.ingest(node("rover-1", 2.0, 1.0, timestamp=10)))
        self.assertFalse(registry.ingest(node("rover-1", 8.0, 1.0, timestamp=9)))
        self.assertEqual(registry.get("rover-1").position.x_m, 2.0)  # type: ignore[union-attr]

    def test_predicts_head_on_approach(self) -> None:
        alerts = predict_collisions([node("a", -10, 1), node("b", 10, -1)], threshold_m=1.0, horizon_s=15)
        self.assertEqual(len(alerts), 1)
        self.assertAlmostEqual(alerts[0].time_to_closest_s, 10.0)
        self.assertAlmostEqual(alerts[0].predicted_closest_distance_m, 0.0)

    def test_ignores_distant_or_late_approach(self) -> None:
        alerts = predict_collisions([node("a", -10, -1), node("b", 10, 1)], threshold_m=1.0, horizon_s=15)
        self.assertEqual(alerts, [])

    def test_formation_slots_are_distinct(self) -> None:
        leader = NodeTelemetry("leader", 1, Position(0, 0), Velocity(0, 2), VehicleKind.DRONE)
        commands = leader_follower_commands(leader, [node("f1", 1, 0), node("f2", 2, 0)], spacing_m=4)
        self.assertEqual([command.follower_id for command in commands], ["f1", "f2"])
        self.assertNotEqual(commands[0].target, commands[1].target)
        self.assertLess(commands[0].target.y_m, leader.position.y_m)

    def test_mission_log_round_trip(self) -> None:
        log = MissionLog()
        log.append(node("rover-1", 1, 0))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mission.jsonl"
            log.write_jsonl(path)
            restored = MissionLog.read_jsonl(path)
        self.assertEqual(restored.records, log.records)


class CoordinationTests(unittest.TestCase):
    def task(self, *, capability: str | None = None) -> TaskRequest:
        return TaskRequest(
            "inspect-panel-1", TaskKind.INSPECTION, Position(10, 0), created_s=10,
            required_capabilities=frozenset() if capability is None else frozenset({capability}),
            exclusion_radius_m=4,
        )

    def test_assigns_exactly_one_eligible_owner(self) -> None:
        coordinator = TaskCoordinator()
        near = node("drone-near", 8, 1)
        far = node("rover-far", -20, 2)
        plan = coordinator.plan(self.task(), [near, far], now_s=10)
        self.assertEqual(plan.status, TaskStatus.ACTIVE)
        self.assertEqual(plan.assigned_node_id, "drone-near")
        self.assertEqual(sum(item.kind is InstructionKind.APPROACH for item in plan.instructions), 1)
        self.assertEqual(next(item for item in plan.instructions if item.node_id == "rover-far").kind, InstructionKind.CONTINUE_ROUTE)

    def test_excludes_low_battery_and_selects_capable_backup(self) -> None:
        coordinator = TaskCoordinator(minimum_battery_percent=25)
        low_battery = NodeTelemetry("drone-low", 10, Position(9, 0), Velocity(2, 0), VehicleKind.DRONE, battery_percent=10)
        capable = NodeTelemetry("rover-camera", 10, Position(2, 0), Velocity(1, 0), VehicleKind.ROVER, battery_percent=90)
        plan = coordinator.plan(
            self.task(capability="camera"), [low_battery, capable],
            profiles=[NodeProfile("drone-low", frozenset({"camera"})), NodeProfile("rover-camera", frozenset({"camera"}))], now_s=10,
        )
        self.assertEqual(plan.assigned_node_id, "rover-camera")
        self.assertEqual(plan.excluded["drone-low"], "battery reserve too low")

    def test_excludes_missing_capability(self) -> None:
        coordinator = TaskCoordinator()
        plan = coordinator.plan(self.task(capability="camera"), [node("rover", 1, 1)], now_s=10)
        self.assertEqual(plan.status, TaskStatus.UNASSIGNABLE)
        self.assertEqual(plan.excluded["rover"], "required capability unavailable")

    def test_holds_non_owner_inside_exclusion_zone(self) -> None:
        coordinator = TaskCoordinator()
        owner = node("owner", 9, 1)
        nearby = node("nearby", 11, 0)
        plan = coordinator.plan(self.task(), [owner, nearby], now_s=10)
        hold = next(item for item in plan.instructions if item.node_id == "nearby")
        self.assertEqual(hold.kind, InstructionKind.HOLD)
        self.assertEqual(hold.target, nearby.position)

    def test_reassigns_after_owner_becomes_stale(self) -> None:
        coordinator = TaskCoordinator(default_lease_s=100, max_state_age_s=3)
        original = node("original", 9, 1, timestamp=10)
        backup = node("backup", 0, 1, timestamp=10)
        first = coordinator.plan(self.task(), [original, backup], now_s=10)
        self.assertEqual(first.assigned_node_id, "original")
        refreshed_backup = node("backup", 0, 1, timestamp=14)
        second = coordinator.plan(self.task(), [original, refreshed_backup], now_s=14)
        self.assertEqual(second.assigned_node_id, "backup")
        self.assertEqual(second.excluded["original"], "stale telemetry")

    def test_unassignable_plan_has_no_owner(self) -> None:
        coordinator = TaskCoordinator()
        stale = node("stale", 9, 1, timestamp=0)
        plan = coordinator.plan(self.task(), [stale], now_s=10)
        self.assertEqual(plan.status, TaskStatus.UNASSIGNABLE)
        self.assertIsNone(plan.assigned_node_id)


if __name__ == "__main__":
    unittest.main()
