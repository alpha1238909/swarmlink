"""Deterministic in-memory mesh for tests, demonstrations, and algorithm work."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import heapq
import random

from .core import NodeRegistry, NodeTelemetry
from .protocol import decode_packet, encode_packet


class SimulatedMesh:
    """Broadcast packets locally without Wi-Fi, radios, credentials, or hardware.

    A real adapter may call ``publish`` after serial/MQTT/LoRa reception. The
    core receives the exact same validated packet format in either case.
    """

    def __init__(self) -> None:
        self.registry = NodeRegistry()
        self._listeners: list[Callable[[NodeTelemetry], None]] = []
        self.packets_sent = 0

    def subscribe(self, callback: Callable[[NodeTelemetry], None]) -> None:
        self._listeners.append(callback)

    def publish(self, telemetry: NodeTelemetry) -> bool:
        parsed = decode_packet(encode_packet(telemetry))
        accepted = self.registry.ingest(parsed)
        if accepted:
            self.packets_sent += 1
            for callback in tuple(self._listeners):
                callback(parsed)
        return accepted


@dataclass(frozen=True)
class FaultProfile:
    """Deterministic fault settings for a local network simulation."""

    packet_loss_rate: float = 0.0
    latency_s: float = 0.0
    jitter_s: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if not 0 <= self.packet_loss_rate <= 1:
            raise ValueError("packet_loss_rate must be within 0..1")
        if self.latency_s < 0 or self.jitter_s < 0:
            raise ValueError("latency_s and jitter_s cannot be negative")


class FaultInjectingMesh(SimulatedMesh):
    """A queueing mesh that injects reproducible loss, latency, and jitter.

    ``send`` schedules a packet; ``advance`` delivers due packets. This lets a
    project test how stale telemetry and leases affect its decisions before any
    radio hardware is involved.
    """

    def __init__(self, profile: FaultProfile = FaultProfile()) -> None:
        super().__init__()
        self.profile = profile
        self._random = random.Random(profile.seed)
        self._pending: list[tuple[float, int, bytes]] = []
        self._sequence = 0
        self.packets_dropped = 0
        self.packets_delivered = 0

    @property
    def pending_packets(self) -> int:
        return len(self._pending)

    def send(self, telemetry: NodeTelemetry, *, now_s: float) -> bool:
        if self._random.random() < self.profile.packet_loss_rate:
            self.packets_dropped += 1
            return False
        jitter = self._random.uniform(-self.profile.jitter_s, self.profile.jitter_s)
        deliver_at_s = max(now_s, now_s + self.profile.latency_s + jitter)
        self._sequence += 1
        heapq.heappush(self._pending, (deliver_at_s, self._sequence, encode_packet(telemetry)))
        return True

    def advance(self, now_s: float) -> int:
        delivered = 0
        while self._pending and self._pending[0][0] <= now_s:
            _, _, packet = heapq.heappop(self._pending)
            telemetry = decode_packet(packet)
            if self.registry.ingest(telemetry):
                self.packets_sent += 1
                self.packets_delivered += 1
                delivered += 1
                for callback in tuple(self._listeners):
                    callback(telemetry)
        return delivered
