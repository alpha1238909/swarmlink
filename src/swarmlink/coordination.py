"""High-level exclusive-task coordination for mixed robot teams.

This module makes *task decisions*, not motor decisions.  An application may
translate an ``APPROACH`` instruction into its own autopilot, path planner, or
human approval workflow.  The coordinator never talks to a flight controller.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Iterable

from .core import NodeTelemetry, Position
from .safety import SafetyPolicy


class TaskKind(str, Enum):
    INSPECTION = "inspection"
    ANOMALY_CHECK = "anomaly_check"
    DOCKING = "docking"
    MAPPING = "mapping"
    DELIVERY = "delivery"
    RETURN_HOME = "return_home"


class TaskStatus(str, Enum):
    OPEN = "open"
    ACTIVE = "active"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    UNASSIGNABLE = "unassignable"
    BLOCKED = "blocked"


class InstructionKind(str, Enum):
    APPROACH = "approach"
    HOLD = "hold"
    OBSERVE = "observe"
    CONTINUE_ROUTE = "continue_route"
    RETURN_HOME = "return_home"


@dataclass(frozen=True)
class TaskRequest:
    """A high-level civilian robotics task at one local ENU target point."""

    task_id: str
    kind: TaskKind
    target: Position
    created_s: float
    priority: int = 50
    required_capabilities: frozenset[str] = frozenset()
    exclusive: bool = True
    exclusion_radius_m: float = 6.0
    deadline_s: float | None = None

    def __post_init__(self) -> None:
        if not self.task_id or len(self.task_id) > 64:
            raise ValueError("task_id must contain 1–64 characters")
        if not 0 <= self.priority <= 100:
            raise ValueError("priority must be in 0..100")
        if self.exclusion_radius_m <= 0:
            raise ValueError("exclusion_radius_m must be positive")
        if self.deadline_s is not None and self.deadline_s < self.created_s:
            raise ValueError("deadline_s cannot predate created_s")


@dataclass(frozen=True)
class NodeProfile:
    """Static, locally configured eligibility data for a node.

    Capabilities are free-form labels, for example ``camera``, ``thermal``,
    ``gripper``, ``water_sampler``, or ``dock_adapter``.  They are not claimed
    from a radio packet, so another node cannot grant itself a capability.
    """

    node_id: str
    capabilities: frozenset[str] = frozenset()
    supported_tasks: frozenset[TaskKind] = frozenset(TaskKind)
    available: bool = True
    minimum_battery_percent: float | None = None

    def __post_init__(self) -> None:
        if not self.node_id:
            raise ValueError("node_id cannot be empty")
        if self.minimum_battery_percent is not None and not 0 <= self.minimum_battery_percent <= 100:
            raise ValueError("minimum_battery_percent must be in 0..100")


@dataclass(frozen=True)
class TaskAssignment:
    task_id: str
    node_id: str
    assigned_s: float
    lease_expires_s: float
    score: float


@dataclass(frozen=True)
class CoordinationInstruction:
    node_id: str
    task_id: str
    kind: InstructionKind
    reason: str
    target: Position | None = None
    minimum_distance_m: float | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "node_id": self.node_id,
            "task_id": self.task_id,
            "kind": self.kind.value,
            "reason": self.reason,
            "target": None if self.target is None else {"x_m": self.target.x_m, "y_m": self.target.y_m, "z_m": self.target.z_m},
            "minimum_distance_m": self.minimum_distance_m,
        }


@dataclass(frozen=True)
class CoordinationPlan:
    task: TaskRequest
    status: TaskStatus
    assignment: TaskAssignment | None
    instructions: tuple[CoordinationInstruction, ...]
    excluded: dict[str, str] = field(default_factory=dict)

    @property
    def assigned_node_id(self) -> str | None:
        return None if self.assignment is None else self.assignment.node_id

    def to_dict(self) -> dict[str, object]:
        return {
            "task_id": self.task.task_id,
            "task_kind": self.task.kind.value,
            "status": self.status.value,
            "assigned_node_id": self.assigned_node_id,
            "lease_expires_s": None if self.assignment is None else self.assignment.lease_expires_s,
            "instructions": [item.to_dict() for item in self.instructions],
            "excluded": self.excluded,
        }


@dataclass(frozen=True)
class TaskBatchPlan:
    """Priority-first allocation outcome for several tasks in one team snapshot."""

    plans: tuple[CoordinationPlan, ...]

    @property
    def assignments(self) -> dict[str, str]:
        return {
            plan.task.task_id: plan.assigned_node_id
            for plan in self.plans
            if plan.assigned_node_id is not None
        }

    def instructions_for(self, node_id: str) -> tuple[CoordinationInstruction, ...]:
        return tuple(
            instruction
            for plan in self.plans
            for instruction in plan.instructions
            if instruction.node_id == node_id
        )


class TaskCoordinator:
    """Deterministically allocates one eligible robot to an exclusive task.

    Eligibility is intentionally conservative: unavailable, stale, low-battery,
    unsuitable, or capability-mismatched nodes are excluded before scoring.
    Within remaining candidates the smallest estimated arrival time wins, then
    the greater battery reserve, then alphabetical node ID for reproducibility.
    """

    def __init__(
        self,
        *,
        default_lease_s: float = 15.0,
        max_state_age_s: float = 3.0,
        minimum_battery_percent: float = 20.0,
        safety_policy: SafetyPolicy | None = None,
    ) -> None:
        if default_lease_s <= 0 or max_state_age_s < 0 or not 0 <= minimum_battery_percent <= 100:
            raise ValueError("invalid coordinator configuration")
        self.default_lease_s = default_lease_s
        self.max_state_age_s = max_state_age_s
        self.minimum_battery_percent = minimum_battery_percent
        self.safety_policy = safety_policy
        self._assignments: dict[str, TaskAssignment] = {}
        self._terminal_status: dict[str, TaskStatus] = {}

    def complete(self, task_id: str) -> None:
        self._assignments.pop(task_id, None)
        self._terminal_status[task_id] = TaskStatus.COMPLETED

    def cancel(self, task_id: str) -> None:
        self._assignments.pop(task_id, None)
        self._terminal_status[task_id] = TaskStatus.CANCELLED

    def assignment_for(self, task_id: str) -> TaskAssignment | None:
        return self._assignments.get(task_id)

    def _eligible(
        self,
        task: TaskRequest,
        nodes: Iterable[NodeTelemetry],
        profiles: dict[str, NodeProfile],
        now_s: float,
        reserved_node_ids: frozenset[str] = frozenset(),
    ) -> tuple[list[tuple[NodeTelemetry, float]], dict[str, str]]:
        candidates: list[tuple[NodeTelemetry, float]] = []
        excluded: dict[str, str] = {}
        for node in nodes:
            profile = profiles.get(node.node_id, NodeProfile(node.node_id))
            if node.node_id in reserved_node_ids:
                excluded[node.node_id] = "already reserved by a higher-priority task"
            elif now_s - node.timestamp_s > self.max_state_age_s:
                excluded[node.node_id] = "stale telemetry"
            elif not profile.available:
                excluded[node.node_id] = "node unavailable"
            elif task.kind not in profile.supported_tasks:
                excluded[node.node_id] = "task type unsupported"
            elif not task.required_capabilities.issubset(profile.capabilities):
                excluded[node.node_id] = "required capability unavailable"
            elif node.battery_percent is not None and node.battery_percent < (
                self.minimum_battery_percent if profile.minimum_battery_percent is None else profile.minimum_battery_percent
            ):
                excluded[node.node_id] = "battery reserve too low"
            elif task.deadline_s is not None and now_s > task.deadline_s:
                excluded[node.node_id] = "task deadline passed"
            else:
                distance = node.position.distance_to(task.target)
                # A stationary node is still eligible, but its ETA is penalised.
                effective_speed = max(node.velocity.speed_mps, 0.25)
                eta_s = distance / effective_speed
                battery_penalty = 0.0 if node.battery_percent is None else (100.0 - node.battery_percent) * 0.01
                candidates.append((node, eta_s + battery_penalty))
        return candidates, excluded

    def plan(
        self,
        task: TaskRequest,
        nodes: Iterable[NodeTelemetry],
        *,
        profiles: Iterable[NodeProfile] = (),
        now_s: float,
        reserved_node_ids: frozenset[str] = frozenset(),
    ) -> CoordinationPlan:
        """Allocate/reconcile a task and return instructions for the whole team."""
        states = tuple(nodes)
        profile_by_id = {profile.node_id: profile for profile in profiles}
        terminal = self._terminal_status.get(task.task_id)
        if terminal is not None:
            return CoordinationPlan(task, terminal, None, (), {})

        if self.safety_policy is not None:
            violation = self.safety_policy.validate_target(task.target)
            if violation is not None:
                self._assignments.pop(task.task_id, None)
                return CoordinationPlan(task, TaskStatus.BLOCKED, None, (), {"task": violation})

        candidates, excluded = self._eligible(task, states, profile_by_id, now_s, reserved_node_ids)
        candidate_by_id = {node.node_id: (node, score) for node, score in candidates}
        assignment = self._assignments.get(task.task_id)
        if assignment is None or assignment.lease_expires_s <= now_s or assignment.node_id not in candidate_by_id:
            if not candidates:
                self._assignments.pop(task.task_id, None)
                return CoordinationPlan(task, TaskStatus.UNASSIGNABLE, None, (), excluded)
            selected, score = min(
                candidates,
                key=lambda item: (item[1], -(item[0].battery_percent if item[0].battery_percent is not None else 50.0), item[0].node_id),
            )
            assignment = TaskAssignment(task.task_id, selected.node_id, now_s, now_s + self.default_lease_s, score)
            self._assignments[task.task_id] = assignment

        instructions: list[CoordinationInstruction] = []
        for node in sorted(states, key=lambda item: item.node_id):
            if node.node_id == assignment.node_id:
                instructions.append(CoordinationInstruction(node.node_id, task.task_id, InstructionKind.APPROACH, "exclusive task owner", task.target))
            elif task.exclusive and node.position.distance_to(task.target) < task.exclusion_radius_m:
                instructions.append(CoordinationInstruction(node.node_id, task.task_id, InstructionKind.HOLD, "inside task exclusion zone", node.position, task.exclusion_radius_m))
            elif "observer" in profile_by_id.get(node.node_id, NodeProfile(node.node_id)).capabilities:
                instructions.append(CoordinationInstruction(node.node_id, task.task_id, InstructionKind.OBSERVE, "not task owner; observe from current mission position", None, task.exclusion_radius_m))
            else:
                instructions.append(CoordinationInstruction(node.node_id, task.task_id, InstructionKind.CONTINUE_ROUTE, "not task owner", None, task.exclusion_radius_m if task.exclusive else None))
        return CoordinationPlan(task, TaskStatus.ACTIVE, assignment, tuple(instructions), excluded)

    def plan_many(
        self,
        tasks: Iterable[TaskRequest],
        nodes: Iterable[NodeTelemetry],
        *,
        profiles: Iterable[NodeProfile] = (),
        now_s: float,
    ) -> TaskBatchPlan:
        """Allocate a team snapshot across multiple tasks without double booking.

        Tasks are deliberately considered in priority-first order (then earliest
        deadline and creation time) so the decision stays explainable during a
        field test. Existing assignments are reconciled under that same policy.
        """
        states = tuple(nodes)
        profile_list = tuple(profiles)
        ordered_tasks = sorted(
            tasks,
            key=lambda task: (-task.priority, float("inf") if task.deadline_s is None else task.deadline_s, task.created_s, task.task_id),
        )
        reserved: set[str] = set()
        plans: list[CoordinationPlan] = []
        for task in ordered_tasks:
            plan = self.plan(task, states, profiles=profile_list, now_s=now_s, reserved_node_ids=frozenset(reserved))
            if plan.assigned_node_id is not None:
                reserved.add(plan.assigned_node_id)
            plans.append(plan)
        return TaskBatchPlan(tuple(plans))
