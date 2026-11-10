"""Heartbeat tracking and deterministic coordinator-election helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Heartbeat:
    node_id: str
    timestamp_s: float
    election_priority: int = 0

    def __post_init__(self) -> None:
        if not self.node_id:
            raise ValueError("node_id cannot be empty")


class HeartbeatMonitor:
    """Tracks recent peers and elects one leader from a shared current view.

    This is deliberately a deterministic helper, not a replacement for a
    fault-tolerant distributed-consensus protocol. Nodes must independently
    validate a leader before accepting any high-level instruction.
    """

    def __init__(self, *, timeout_s: float = 3.0) -> None:
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = timeout_s
        self._latest: dict[str, Heartbeat] = {}

    def ingest(self, heartbeat: Heartbeat) -> bool:
        previous = self._latest.get(heartbeat.node_id)
        if previous is not None and heartbeat.timestamp_s < previous.timestamp_s:
            return False
        self._latest[heartbeat.node_id] = heartbeat
        return True

    def live(self, now_s: float) -> tuple[Heartbeat, ...]:
        return tuple(sorted((beat for beat in self._latest.values() if now_s - beat.timestamp_s <= self.timeout_s), key=lambda beat: beat.node_id))

    def leader(self, now_s: float) -> Heartbeat | None:
        peers = self.live(now_s)
        return min(peers, key=lambda beat: (-beat.election_priority, beat.node_id)) if peers else None

    def stale_node_ids(self, now_s: float) -> tuple[str, ...]:
        return tuple(sorted(node_id for node_id, beat in self._latest.items() if now_s - beat.timestamp_s > self.timeout_s))
