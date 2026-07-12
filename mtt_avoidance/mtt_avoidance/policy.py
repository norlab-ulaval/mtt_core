"""Deterministic obstacle-hold policy; ROS-free for unit testing."""

from dataclasses import dataclass
from enum import IntEnum


class State(IntEnum):
    DISABLED = 0
    IDLE = 1
    FOLLOWING = 2
    HOLDING = 3
    PLANNING = 4
    SHADOW_READY = 5
    EXECUTING = 6
    RECOVERING = 7
    BLOCKED = 8
    SENSOR_FAULT = 9


@dataclass(frozen=True)
class Inputs:
    now_s: float
    enabled: bool
    replaying: bool
    obstacle_fresh: bool
    obstacle_stop: bool
    plan_ready: bool
    plan_failed: bool
    execution_finished: bool
    recovery_finished: bool


@dataclass(frozen=True)
class Decision:
    state: State
    request_plan: bool = False
    request_execute: bool = False
    request_recovery: bool = False
    cancel_motion: bool = False


class AvoidancePolicy:
    def __init__(
        self,
        persistent_stop_s: float = 3.0,
        execution_enabled: bool = False,
        reverse_recovery_enabled: bool = False,
    ):
        self.persistent_stop_s = persistent_stop_s
        self.execution_enabled = execution_enabled
        self.reverse_recovery_enabled = reverse_recovery_enabled
        self.state = State.DISABLED
        self.stop_since_s: float | None = None
        self._plan_requested = False

    def reset(self, state: State = State.IDLE) -> None:
        self.state = state
        self.stop_since_s = None
        self._plan_requested = False

    def update(self, data: Inputs) -> Decision:
        if not data.enabled:
            self.reset(State.DISABLED)
            return Decision(self.state, cancel_motion=True)
        if not data.replaying:
            self.reset(State.IDLE)
            return Decision(self.state, cancel_motion=True)
        if not data.obstacle_fresh:
            self.state = State.SENSOR_FAULT
            return Decision(self.state, cancel_motion=True)

        if self.state in (State.EXECUTING, State.RECOVERING):
            finished = (
                data.execution_finished
                if self.state == State.EXECUTING
                else data.recovery_finished
            )
            if finished:
                self.reset(State.FOLLOWING)
                return Decision(self.state, cancel_motion=True)
            return Decision(self.state)

        if not data.obstacle_stop:
            self.reset(State.FOLLOWING)
            return Decision(self.state, cancel_motion=True)

        if self.stop_since_s is None:
            self.stop_since_s = data.now_s
        held_for = data.now_s - self.stop_since_s

        if data.plan_ready:
            if self.execution_enabled:
                self.state = State.EXECUTING
                return Decision(self.state, request_execute=True)
            self.state = State.SHADOW_READY
            return Decision(self.state)

        if data.plan_failed:
            if self.execution_enabled and self.reverse_recovery_enabled:
                self.state = State.RECOVERING
                return Decision(self.state, request_recovery=True)
            self.state = State.BLOCKED
            return Decision(self.state)

        if held_for >= self.persistent_stop_s:
            self.state = State.PLANNING
            if not self._plan_requested:
                self._plan_requested = True
                return Decision(self.state, request_plan=True)
            return Decision(self.state)

        self.state = State.HOLDING
        return Decision(self.state)
