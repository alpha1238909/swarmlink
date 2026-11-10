"""A small versioned JSON packet envelope for any transport."""

from __future__ import annotations

import json

from .core import NodeTelemetry

PROTOCOL_VERSION = "swarmlink.telemetry.v1"
COMPACT_PROTOCOL_VERSION = "swarmlink.telemetry.compact.v1"


def encode_packet(telemetry: NodeTelemetry) -> bytes:
    return json.dumps(
        {"type": PROTOCOL_VERSION, "telemetry": telemetry.to_dict()}, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")


def encode_compact_packet(telemetry: NodeTelemetry) -> bytes:
    """Encode a bounded-bandwidth form intended for constrained radios.

    It decodes into precisely the same :class:`NodeTelemetry` as the regular
    envelope, but uses short field names and position/velocity triples. This
    makes ESP-NOW's 250-byte v1 frame limit practical without silently losing
    a velocity estimate.
    """
    payload: dict[str, object] = {
        "type": COMPACT_PROTOCOL_VERSION,
        "n": telemetry.node_id,
        "t": telemetry.timestamp_s,
        "p": [telemetry.position.x_m, telemetry.position.y_m, telemetry.position.z_m],
        "v": [telemetry.velocity.x_mps, telemetry.velocity.y_mps, telemetry.velocity.z_mps],
        "k": telemetry.vehicle_kind.value,
        "s": telemetry.mission_state,
    }
    if telemetry.battery_percent is not None:
        payload["b"] = telemetry.battery_percent
    if telemetry.heading_deg is not None:
        payload["h"] = telemetry.heading_deg
    if telemetry.health:
        payload["x"] = telemetry.health
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_packet(packet: bytes | str) -> NodeTelemetry:
    if isinstance(packet, bytes):
        packet = packet.decode("utf-8")
    try:
        envelope = json.loads(packet)
    except json.JSONDecodeError as error:
        raise ValueError("packet is not valid JSON") from error
    if not isinstance(envelope, dict):
        raise ValueError("packet envelope must be a JSON object")
    try:
        if envelope.get("type") == PROTOCOL_VERSION:
            telemetry = envelope.get("telemetry")
            if not isinstance(telemetry, dict):
                raise ValueError("packet has no telemetry object")
            return NodeTelemetry.from_dict(telemetry)
        if envelope.get("type") == COMPACT_PROTOCOL_VERSION:
            return _decode_compact_packet(envelope)
    except (KeyError, TypeError) as error:
        raise ValueError("packet telemetry has missing or invalid fields") from error
    raise ValueError(f"unsupported packet type; expected {PROTOCOL_VERSION} or {COMPACT_PROTOCOL_VERSION}")


def _decode_compact_packet(envelope: dict[str, object]) -> NodeTelemetry:
    position, velocity = envelope.get("p"), envelope.get("v")
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError("compact packet requires a three-value position 'p'")
    if not isinstance(velocity, list) or len(velocity) != 3:
        raise ValueError("compact packet requires a three-value velocity 'v'")
    health = envelope.get("x", {})
    if not isinstance(health, dict):
        raise ValueError("compact packet health 'x' must be an object")
    return NodeTelemetry.from_dict(
        {
            "node_id": envelope["n"],
            "timestamp_s": envelope["t"],
            "position": {"x_m": position[0], "y_m": position[1], "z_m": position[2]},
            "velocity": {"x_mps": velocity[0], "y_mps": velocity[1], "z_mps": velocity[2]},
            "vehicle_kind": envelope.get("k", "unknown"),
            "battery_percent": envelope.get("b"),
            "heading_deg": envelope.get("h"),
            "mission_state": envelope.get("s", "idle"),
            "health": health,
        }
    )
