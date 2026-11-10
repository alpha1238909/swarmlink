"""A small versioned JSON packet envelope for any transport."""

from __future__ import annotations

import json

from .core import NodeTelemetry

PROTOCOL_VERSION = "swarmlink.telemetry.v1"


def encode_packet(telemetry: NodeTelemetry) -> bytes:
    return json.dumps(
        {"type": PROTOCOL_VERSION, "telemetry": telemetry.to_dict()}, separators=(",", ":"), sort_keys=True
    ).encode("utf-8")


def decode_packet(packet: bytes | str) -> NodeTelemetry:
    if isinstance(packet, bytes):
        packet = packet.decode("utf-8")
    try:
        envelope = json.loads(packet)
    except json.JSONDecodeError as error:
        raise ValueError("packet is not valid JSON") from error
    if not isinstance(envelope, dict) or envelope.get("type") != PROTOCOL_VERSION:
        raise ValueError(f"unsupported packet type; expected {PROTOCOL_VERSION}")
    telemetry = envelope.get("telemetry")
    if not isinstance(telemetry, dict):
        raise ValueError("packet has no telemetry object")
    return NodeTelemetry.from_dict(telemetry)
