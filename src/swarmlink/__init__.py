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
from .protocol import decode_packet, encode_packet
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
    "decode_packet",
    "encode_packet",
    "leader_follower_commands",
    "predict_collisions",
]

__version__ = "0.3.0"
