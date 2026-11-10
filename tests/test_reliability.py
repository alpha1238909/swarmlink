from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from swarmlink import (
    CircularKeepOut,
    CoordinationAuditLog,
    FaultInjectingMesh,
    FaultProfile,
    GeoPoint,
    Heartbeat,
    HeartbeatMonitor,
    LocalFrame,
    NodeTelemetry,
    OperationalBounds,
    PacketAuthenticationError,
    PacketAuthenticator,
    Position,
    ReplayDetectedError,
    SafetyPolicy,
    TaskCoordinator,
    TaskKind,
    TaskRequest,
    TaskStatus,
    VehicleKind,
    Velocity,
)


def state(node_id: str, x_m: float, *, timestamp_s: float = 10.0, battery_percent: float = 80.0) -> NodeTelemetry:
    return NodeTelemetry(node_id, timestamp_s, Position(x_m, 0.0), Velocity(2.0, 0.0), VehicleKind.ROVER, battery_percent=battery_percent)


class ReliabilityTests(unittest.TestCase):
    def test_priority_batch_does_not_double_book_node(self) -> None:
        tasks = [
            TaskRequest("routine", TaskKind.MAPPING, Position(3, 0), 10, priority=20),
            TaskRequest("urgent", TaskKind.INSPECTION, Position(1, 0), 10, priority=90),
        ]
        batch = TaskCoordinator().plan_many(tasks, [state("near", 0), state("far", 20)], now_s=10)
        self.assertEqual(batch.assignments["urgent"], "near")
        self.assertEqual(batch.assignments["routine"], "far")
        self.assertEqual(len(set(batch.assignments.values())), 2)

    def test_safety_policy_blocks_outside_and_keep_out_targets(self) -> None:
        policy = SafetyPolicy(
            bounds=OperationalBounds(-10, 10, -10, 10),
            keep_outs=[CircularKeepOut(Position(0, 0), 2, "charging area")],
        )
        coordinator = TaskCoordinator(safety_policy=policy)
        outside = coordinator.plan(TaskRequest("outside", TaskKind.MAPPING, Position(20, 0), 10), [state("r", 1)], now_s=10)
        protected = coordinator.plan(TaskRequest("protected", TaskKind.MAPPING, Position(1, 0), 10), [state("r", 1)], now_s=10)
        self.assertEqual(outside.status, TaskStatus.BLOCKED)
        self.assertEqual(outside.excluded["task"], "target outside operational bounds")
        self.assertEqual(protected.status, TaskStatus.BLOCKED)
        self.assertIn("charging area", protected.excluded["task"])

    def test_geographic_local_frame_round_trip(self) -> None:
        frame = LocalFrame(GeoPoint(51.1694, 71.4491, 350.0))
        original = GeoPoint(51.1695, 71.4493, 355.0)
        restored = frame.to_geographic(frame.to_enu(original))
        self.assertAlmostEqual(restored.latitude_deg, original.latitude_deg, places=8)
        self.assertAlmostEqual(restored.longitude_deg, original.longitude_deg, places=8)
        self.assertAlmostEqual(restored.altitude_m, original.altitude_m)

    def test_leader_election_changes_after_timeout(self) -> None:
        monitor = HeartbeatMonitor(timeout_s=3)
        monitor.ingest(Heartbeat("base", 10, election_priority=10))
        monitor.ingest(Heartbeat("backup", 10, election_priority=5))
        self.assertEqual(monitor.leader(12).node_id, "base")  # type: ignore[union-attr]
        monitor.ingest(Heartbeat("backup", 14, election_priority=5))
        self.assertEqual(monitor.leader(14).node_id, "backup")  # type: ignore[union-attr]

    def test_signed_packet_rejects_tamper_and_replay(self) -> None:
        auth = PacketAuthenticator("this-is-a-long-test-secret")
        packet = auth.encode(state("rover", 1), issued_at_s=10, nonce="fixed-nonce")
        self.assertEqual(auth.decode(packet, now_s=11).node_id, "rover")
        with self.assertRaises(ReplayDetectedError):
            auth.decode(packet, now_s=11)
        other = PacketAuthenticator("this-is-a-long-test-secret")
        with self.assertRaises(PacketAuthenticationError):
            other.decode(packet.replace(b"rover", b"other"), now_s=11)

    def test_fault_mesh_delays_and_drops_packets_deterministically(self) -> None:
        delayed = FaultInjectingMesh(FaultProfile(latency_s=2, seed=7))
        self.assertTrue(delayed.send(state("rover", 1), now_s=10))
        self.assertEqual(delayed.advance(11.9), 0)
        self.assertEqual(delayed.advance(12), 1)
        self.assertEqual(delayed.registry.get("rover").node_id, "rover")  # type: ignore[union-attr]
        lost = FaultInjectingMesh(FaultProfile(packet_loss_rate=1, seed=7))
        self.assertFalse(lost.send(state("lost", 1), now_s=10))
        self.assertEqual(lost.packets_dropped, 1)

    def test_audit_log_exports_decision(self) -> None:
        plan = TaskCoordinator().plan(TaskRequest("check", TaskKind.INSPECTION, Position(2, 0), 10), [state("rover", 0)], now_s=10)
        audit = CoordinationAuditLog()
        audit.record_plan(10, plan)
        with tempfile.TemporaryDirectory() as directory:
            path = audit.write_jsonl(Path(directory) / "audit.jsonl")
            content = path.read_text(encoding="utf-8")
        self.assertIn('"coordination_plan"', content)
        self.assertIn('"rover"', content)


if __name__ == "__main__":
    unittest.main()
