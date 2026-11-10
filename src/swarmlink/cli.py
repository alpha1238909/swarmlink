"""Command line demo and mission-log inspector."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .core import MissionLog, NodeTelemetry, Position, VehicleKind, Velocity
from .sim import SimulatedMesh


def _simulated_node(index: int, time_s: float, node_count: int) -> NodeTelemetry:
    # Two groups deliberately cross near the centre, exercising risk detection.
    from_left = index < node_count / 2
    x = (-16 + time_s * 2.0) if from_left else (16 - time_s * 2.0)
    y = (index % 3 - 1) * 2.5
    return NodeTelemetry(
        node_id=("drone" if index % 2 == 0 else "rover") + f"-{index + 1:02d}",
        timestamp_s=time_s,
        position=Position(x, y, 8.0 if index % 2 == 0 else 0.0),
        velocity=Velocity(2.0 if from_left else -2.0, 0.0, 0.0),
        vehicle_kind=VehicleKind.DRONE if index % 2 == 0 else VehicleKind.ROVER,
        battery_percent=max(0.0, 100.0 - time_s * 0.25),
        mission_state="mapping",
    )


def simulate(args: argparse.Namespace) -> int:
    mesh, log = SimulatedMesh(), MissionLog()
    time_s = 0.0
    alerts = []
    while time_s <= args.duration + 1e-9:
        for index in range(args.nodes):
            status = _simulated_node(index, time_s, args.nodes)
            mesh.publish(status)
            log.append(status)
        alerts.extend(mesh.registry.collision_alerts(threshold_m=args.separation_m, horizon_s=args.horizon_s))
        time_s += args.step_s
    output = log.write_jsonl(args.out)
    unique_alerts = {(item.node_a, item.node_b) for item in alerts}
    print(json.dumps({"records": len(log.records), "nodes": args.nodes, "packets_sent": mesh.packets_sent, "collision_pairs": len(unique_alerts), "mission_log": str(output)}, indent=2))
    return 0


def inspect(args: argparse.Namespace) -> int:
    log = MissionLog.read_jsonl(args.path)
    if not log.records:
        print(json.dumps({"records": 0, "nodes": 0}, indent=2))
        return 0
    latest: dict[str, NodeTelemetry] = {}
    for record in log.records:
        latest[record.node_id] = record
    timestamps = [record.timestamp_s for record in log.records]
    print(json.dumps({"records": len(log.records), "nodes": sorted(latest), "start_s": min(timestamps), "end_s": max(timestamps)}, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="swarmlink", description="Multi-robot telemetry and simulation core")
    commands = parser.add_subparsers(dest="command", required=True)
    simulation = commands.add_parser("simulate", help="create a deterministic multi-robot mission log")
    simulation.add_argument("--nodes", type=int, default=6)
    simulation.add_argument("--duration", type=float, default=12.0)
    simulation.add_argument("--step-s", type=float, default=0.5)
    simulation.add_argument("--separation-m", type=float, default=5.0)
    simulation.add_argument("--horizon-s", type=float, default=5.0)
    simulation.add_argument("--out", type=Path, default=Path("swarmlink-mission.jsonl"))
    simulation.set_defaults(handler=simulate)
    inspector = commands.add_parser("inspect", help="summarise a SwarmLink JSON Lines mission log")
    inspector.add_argument("path", type=Path)
    inspector.set_defaults(handler=inspect)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "simulate" and (args.nodes < 2 or args.duration <= 0 or args.step_s <= 0):
        parser.error("simulate requires at least 2 nodes plus positive duration and --step-s")
    return args.handler(args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
