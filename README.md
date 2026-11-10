# SwarmLink

![Tests](https://github.com/alpha1238909/swarmlink/actions/workflows/test.yml/badge.svg) ![License](https://img.shields.io/badge/license-MIT-2ea44f) ![Python](https://img.shields.io/badge/python-3.10%2B-3776ab)

**Local-first telemetry, tracking, collision prediction, and mission replay for drones, rovers, boats, and sensor nodes.**

SwarmLink is the core between a robot's hardware and its autonomous behaviour. It gives every vehicle one small, versioned telemetry packet, tracks the most recent state of a team, predicts pairwise closest approaches, produces leader–follower slots, and records a reproducible mission log. It does **not** control motors, open radios, or require a cloud account.

## Why it exists

Robotics teams regularly connect an ESP32 drone, a rover, and a ground station with incompatible ad-hoc JSON and spreadsheet logs. SwarmLink makes the shared layer reusable: any transport can carry the same packet, and the same mission can be simulated, replayed, inspected, or passed to a future planner.

## Install

```bash
git clone https://github.com/alpha1238909/swarmlink.git
cd swarmlink
python -m pip install .
```

## Five-minute simulation

No drone, rover, Wi-Fi, or sensors are required.

```bash
swarmlink simulate --nodes 6 --duration 12 --out mission.jsonl
swarmlink inspect mission.jsonl
```

The built-in scenario makes virtual drones and rovers cross a local coordinate field; it writes an append-only JSON Lines record and reports predicted close-approach pairs.

## Core use

```python
from swarmlink import (
    NodeTelemetry, Position, SimulatedMesh, VehicleKind, Velocity,
)

mesh = SimulatedMesh()
mesh.publish(NodeTelemetry(
    node_id="rover-01",
    timestamp_s=1735689600.0,
    position=Position(x_m=12.4, y_m=-3.8),
    velocity=Velocity(x_mps=0.8),
    vehicle_kind=VehicleKind.ROVER,
    battery_percent=84.0,
    mission_state="mapping",
))

print(mesh.registry.snapshot())
print(mesh.registry.collision_alerts(threshold_m=4.0, horizon_s=10.0))
```

The coordinates are local **ENU**: `x=east`, `y=north`, `z=up`, measured in metres. Keeping that convention explicit makes it possible to use GPS, UWB anchors, wheel odometry, or simulation data after conversion into a common local frame.

## What it provides now

| Capability | What it does |
|---|---|
| Telemetry schema | Node ID, timestamp, position, velocity, kind, battery, heading, mission state, and custom health fields |
| JSON protocol | Compact versioned JSON packet codec usable through Wi‑Fi, LoRa, serial, MQTT, WebSocket, or a ROS bridge |
| Registry | Rejects out-of-order packets and retains every node's newest state |
| Collision prediction | Constant-velocity closest-approach warning inside a configurable distance and time horizon |
| Formation primitive | Creates alternating leader–follower target slots without commanding a vehicle |
| Mission evidence | Records and reloads portable JSON Lines mission logs |
| Hardware-free simulation | Deterministic in-memory mesh for tests and experiments |

## Exclusive task coordination

`TaskCoordinator` is the mission-decision layer: it designates **exactly one eligible owner** for a task, while every other node receives a clear, high-level instruction. It is suitable for civilian uses such as inspection, docking, mapping, delivery, return-home, and anomaly checking.

```python
from swarmlink import (
    NodeProfile, Position, TaskCoordinator, TaskKind, TaskRequest,
)

task = TaskRequest(
    task_id="inspect-water-station-7",
    kind=TaskKind.INSPECTION,
    target=Position(42.0, -8.0),
    created_s=120.0,
    required_capabilities=frozenset({"camera"}),
    exclusion_radius_m=6.0,
)

plan = TaskCoordinator().plan(
    task,
    mesh.registry.snapshot(),
    profiles=[NodeProfile("drone-01", capabilities=frozenset({"camera", "observer"}))],
    now_s=120.0,
)
print(plan.to_dict())
```

The coordinator conservatively excludes stale nodes, unavailable nodes, mismatched capability profiles, unsupported task types, and low-battery vehicles. From the remaining team it deterministically chooses the lowest estimated-arrival-time vehicle, then prefers more battery reserve for ties. Its short assignment lease makes a task available for reassignment when an owner disappears or becomes ineligible.

For an exclusive task, the selected owner gets `APPROACH`. Other nodes get one of these high-level roles:

| Instruction | Meaning |
|---|---|
| `HOLD` | Remain at the current position because the node is inside the target's exclusion zone. |
| `OBSERVE` | Keep its own mission position and observe only if its trusted local profile allows it. |
| `CONTINUE_ROUTE` | Continue its existing assignment; do not become a second task owner. |

These are intent messages, not actuator commands. Your rover, ESP32, ROS 2 node, or drone autopilot must independently validate safety, geofences, vehicle state, and local rules before acting.

## Reliable team coordination

Version 0.3 adds the parts that matter once more than one task and more than one device are involved. Run the complete local example:

```bash
python examples/reliable_team.py
```

### Priority-first multi-task allocation

Use `plan_many()` when the team has several tasks. Higher-priority tasks are allocated first, then earlier deadlines; one eligible node cannot be allocated twice in the same team snapshot.

```python
batch = coordinator.plan_many(
    [urgent_inspection, routine_mapping],
    mesh.registry.snapshot(),
    profiles=profiles,
    now_s=now_s,
)
print(batch.assignments)  # {"urgent-inspection": "drone-01", "routine-map": "rover-02"}
```

This is intentionally deterministic and explainable instead of being a hidden “AI choice”: every exclusion and assignment score can be inspected in the returned plans.

### Safety policy and local maps

Before allocation, a `SafetyPolicy` rejects targets outside declared boundaries or inside keep-out regions. A rejected task is marked `blocked` and no owner is assigned.

```python
from swarmlink import CircularKeepOut, OperationalBounds, Position, SafetyPolicy

safety = SafetyPolicy(
    bounds=OperationalBounds(-50, 50, -50, 50, 0, 30),
    keep_outs=[CircularKeepOut(Position(0, 0), radius_m=4, label="charging pad")],
)
```

For teams using GPS, convert once into the same local east-north-up frame used everywhere else:

```python
from swarmlink import GeoPoint, LocalFrame

site = LocalFrame(GeoPoint(latitude_deg=51.1694, longitude_deg=71.4491))
local_position = site.to_enu(GeoPoint(51.1695, 71.4493))
```

`LocalFrame` is for a local test site, not long-range navigation.

### Heartbeats and coordinator availability

Every participating device can issue a heartbeat. `HeartbeatMonitor` exposes currently live peers and elects the highest-priority live node, with node ID as a deterministic tie-breaker. It is a simple shared-view failover helper, not a replacement for a distributed-consensus protocol.

```python
from swarmlink import Heartbeat, HeartbeatMonitor

monitor = HeartbeatMonitor(timeout_s=3)
monitor.ingest(Heartbeat("base-station", timestamp_s=now_s, election_priority=10))
monitor.ingest(Heartbeat("rover-01", timestamp_s=now_s, election_priority=5))
leader = monitor.leader(now_s)
```

### Authenticated packets and replay rejection

The normal packet codec stays easy to use for simulation. Real deployments can use the optional `PacketAuthenticator`, which applies HMAC-SHA256 authentication, a time window, unique nonces, and bounded replay protection.

```python
from swarmlink import PacketAuthenticator

auth = PacketAuthenticator(secret_from_safe_configuration)
packet = auth.encode(telemetry, issued_at_s=now_s)
verified = auth.decode(packet, now_s=now_s)
```

It authenticates packets; it does **not** encrypt telemetry. Keep keys out of Git, rotate them for real projects, and use a network layer appropriate for the deployment.

### Fault injection and decision evidence

Test dropped or delayed packets before testing radios:

```python
from swarmlink import FaultInjectingMesh, FaultProfile

mesh = FaultInjectingMesh(FaultProfile(packet_loss_rate=0.15, latency_s=0.2, jitter_s=0.1, seed=7))
mesh.send(telemetry, now_s=now_s)
mesh.advance(now_s + 1.0)
```

`CoordinationAuditLog` stores every returned coordination plan as append-only JSON Lines, including the selected owner, reasons for exclusions, and task status. This is useful for debugging, demos, and proving why a system made a decision.

## Transport boundary

SwarmLink intentionally keeps radio/network code outside its core. A real adapter only needs to decode its incoming bytes and publish the resulting status:

```python
from swarmlink import decode_packet

def on_lora_bytes(packet: bytes) -> None:
    telemetry = decode_packet(packet)
    registry.ingest(telemetry)
```

The equivalent outgoing message is `encode_packet(telemetry)`. This lets a project choose ESP-NOW for nearby ESP32 robots, LoRa for long-range field nodes, MQTT for a local lab, or ROS 2/MAVLink at the integration layer, without embedding credentials or an irreversible transport decision in the package.

## Safety and scope

- Collision output is a planning and alerting aid, **not** an aviation separation, autonomy, or motor-control system.
- The current predictor assumes current velocity stays constant during its configurable horizon.
- Align timestamps and coordinate frames before comparing multiple nodes.
- Do not use it as the sole safety mechanism for vehicles near people, roads, property, or controlled airspace.

## Roadmap

1. ESP-NOW, serial, and MQTT reference adapters
2. UWB-anchor utilities and pose-quality estimates
3. MAVLink and ROS 2 bridges
4. Pluggable path planners that independently validate high-level intents
5. Key rotation and per-node identity provisioning for real deployments

## Development

```bash
python -m unittest discover -s tests -v
python -m swarmlink.cli simulate --nodes 4 --duration 4 --out /tmp/mission.jsonl
```

## License

[MIT](LICENSE)
