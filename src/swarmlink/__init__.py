"""SwarmLink: local-first building blocks for connected robot projects."""

from .core import (
    CollisionAlert,
    FormationCommand,
    MissionLog,
    NodeRegistry,
    NodeTelemetry,
    Position,
    VehicleKind,
    Velocity,
    predict_collisions,
    leader_follower_commands,
)
from .protocol import COMPACT_PROTOCOL_VERSION, PROTOCOL_VERSION, decode_packet, encode_compact_packet, encode_packet
from .sim import FaultInjectingMesh, FaultProfile, SimulatedMesh
from .coordination import (
    CoordinationInstruction,
    CoordinationPlan,
    InstructionKind,
    NodeProfile,
    TaskAssignment,
    TaskCoordinator,
    TaskKind,
    TaskRequest,
    TaskStatus,
    TaskBatchPlan,
)
from .safety import CircularKeepOut, OperationalBounds, SafetyPolicy
from .geodesy import GeoPoint, LocalFrame
from .resilience import Heartbeat, HeartbeatMonitor
from .security import PacketAuthenticationError, PacketAuthenticator, ReplayDetectedError
from .audit import AuditEvent, CoordinationAuditLog
from .bridge import JSONLinesTelemetryDecoder, SerialBridgeError, stream_serial_telemetry

__all__ = [
    "CollisionAlert",
    "FormationCommand",
    "MissionLog",
    "NodeRegistry",
    "NodeTelemetry",
    "Position",
    "SimulatedMesh",
    "FaultInjectingMesh",
    "FaultProfile",
    "VehicleKind",
    "Velocity",
    "CoordinationInstruction",
    "CoordinationPlan",
    "InstructionKind",
    "NodeProfile",
    "TaskAssignment",
    "TaskCoordinator",
    "TaskKind",
    "TaskRequest",
    "TaskStatus",
    "TaskBatchPlan",
    "CircularKeepOut",
    "OperationalBounds",
    "SafetyPolicy",
    "GeoPoint",
    "LocalFrame",
    "Heartbeat",
    "HeartbeatMonitor",
    "PacketAuthenticationError",
    "PacketAuthenticator",
    "ReplayDetectedError",
    "AuditEvent",
    "CoordinationAuditLog",
    "JSONLinesTelemetryDecoder",
    "SerialBridgeError",
    "decode_packet",
    "encode_compact_packet",
    "encode_packet",
    "leader_follower_commands",
    "predict_collisions",
    "stream_serial_telemetry",
    "COMPACT_PROTOCOL_VERSION",
    "PROTOCOL_VERSION",
]

__version__ = "0.4.0"
