"""Append-only audit trail for coordination decisions and faults."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

from .coordination import CoordinationPlan


@dataclass(frozen=True)
class AuditEvent:
    sequence: int
    timestamp_s: float
    category: str
    payload: dict[str, object]

    def to_dict(self) -> dict[str, object]:
        return {"sequence": self.sequence, "timestamp_s": self.timestamp_s, "category": self.category, "payload": self.payload}


class CoordinationAuditLog:
    """A lightweight evidence log that can be exported as JSON Lines."""

    schema_version = "swarmlink.audit.v1"

    def __init__(self) -> None:
        self.events: list[AuditEvent] = []

    def record(self, timestamp_s: float, category: str, payload: dict[str, object]) -> AuditEvent:
        event = AuditEvent(len(self.events) + 1, timestamp_s, category, payload)
        self.events.append(event)
        return event

    def record_plan(self, timestamp_s: float, plan: CoordinationPlan) -> AuditEvent:
        return self.record(timestamp_s, "coordination_plan", plan.to_dict())

    def write_jsonl(self, path: str | Path) -> Path:
        destination = Path(path)
        with destination.open("w", encoding="utf-8") as handle:
            for event in self.events:
                handle.write(json.dumps({"schema_version": self.schema_version, **event.to_dict()}, sort_keys=True) + "\n")
        return destination
