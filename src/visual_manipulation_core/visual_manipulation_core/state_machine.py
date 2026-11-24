"""Small, explicit task state machine (no ROS, no robot).

Each state has a handler `handler(ctx) -> next_state_name`. A handler signals a problem by
raising `StepFailed`; the machine then
  1. retries the state (or `retry_state`) while its attempt budget lasts and the failure is
     marked recoverable,
  2. otherwise jumps to the state's `on_failure` state (a recovery state), or to FAILED.
Every transition is reported to `listener(state, previous, detail, attempt)` so a ROS node can
publish it, and recorded in `history`. `cancel()` makes the machine stop at the next
transition (the recovery state of the current state is still executed if one is defined).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

DONE = 'DONE'
FAILED = 'FAILED'


class StepFailed(RuntimeError):
    def __init__(self, message: str, recoverable: bool = True):
        super().__init__(message)
        self.recoverable = recoverable


@dataclass
class StateSpec:
    name: str
    handler: Callable[[Any], str]
    max_attempts: int = 1
    retry_state: Optional[str] = None       # state to go back to for a retry (default: same)
    on_failure: Optional[str] = None        # recovery state when attempts are exhausted
    description: str = ''


@dataclass
class Transition:
    state: str
    next_state: str
    ok: bool
    detail: str
    duration: float
    attempt: int


@dataclass
class RunResult:
    success: bool
    final_state: str
    message: str
    history: List[Transition] = field(default_factory=list)


class TaskStateMachine:
    def __init__(self, name: str, listener: Optional[Callable[[str, str, str, int], None]] = None,
                 max_transitions: int = 500):
        self.name = name
        self.states: Dict[str, StateSpec] = {}
        self.listener = listener
        self.max_transitions = max_transitions
        self._cancel = threading.Event()
        self.error: Optional[str] = None       # last failure message (for recovery handlers)
        self.failed_state: Optional[str] = None

    def add_state(self, name: str, handler, max_attempts: int = 1, retry_state: Optional[str] = None,
                  on_failure: Optional[str] = None, description: str = ''):
        if name in (DONE, FAILED):
            raise ValueError(f'{name} is a terminal state')
        self.states[name] = StateSpec(name, handler, max(1, max_attempts), retry_state, on_failure, description)
        return self

    def validate(self):
        for s in self.states.values():
            for ref in (s.retry_state, s.on_failure):
                if ref is not None and ref not in self.states and ref not in (DONE, FAILED):
                    raise ValueError(f'state {s.name}: unknown state {ref!r}')

    def cancel(self):
        self._cancel.set()

    def _notify(self, state, previous, detail, attempt):
        if self.listener is not None:
            try:
                self.listener(state, previous, detail, attempt)
            except Exception:  # noqa: BLE001 - a broken listener must not break the task
                pass

    def run(self, ctx, start: str) -> RunResult:
        self.validate()
        self._cancel.clear()
        self.error = None
        self.failed_state = None
        attempts: Dict[str, int] = {}
        history: List[Transition] = []
        state, previous = start, ''
        for _ in range(self.max_transitions):
            if state in (DONE, FAILED):
                ok = state == DONE
                msg = 'completed' if ok else (self.error or 'failed')
                self._notify(state, previous, msg, 0)
                return RunResult(ok, state, msg, history)
            if state not in self.states:
                raise ValueError(f'unknown state {state!r}')
            spec = self.states[state]
            attempts[state] = attempts.get(state, 0) + 1
            n = attempts[state]
            self._notify(state, previous, spec.description, n)
            t0 = time.monotonic()
            try:
                if self._cancel.is_set():
                    raise StepFailed('cancelled by request', recoverable=False)
                nxt = spec.handler(ctx)
                ok, detail = True, ''
            except StepFailed as exc:
                ok, detail = False, str(exc)
                self.error, self.failed_state = f'{state}: {exc}', state
                if exc.recoverable and n < spec.max_attempts and not self._cancel.is_set():
                    nxt = spec.retry_state or state
                elif spec.on_failure:
                    nxt = spec.on_failure
                else:
                    nxt = FAILED
            history.append(Transition(state, nxt, ok, detail, time.monotonic() - t0, n))
            if not ok:
                self._notify(state, state, f'failed (attempt {n}/{spec.max_attempts}): {detail} -> {nxt}', n)
            previous, state = state, nxt
        return RunResult(False, state, f'more than {self.max_transitions} transitions', history)
