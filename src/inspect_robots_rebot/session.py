"""Position streaming on the maintained B601 session, without runtime patches."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from typing import Any

from inspect_robots_b601.hardware import session as native


@dataclass(frozen=True)
class Feedback:
    """One set of cached motor replies, with the oldest reply time."""

    positions: tuple[float, ...]
    efforts: tuple[float, ...]
    temperatures: tuple[float, ...]
    time: float


class StreamingSession(native.ArmSession):
    """Keep the native home/stop lifecycle and add a nonblocking position API."""

    def __init__(
        self,
        *args: Any,
        motor_temp_limit: float = 80.0,
        pair_stop: threading.Event | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        if not math.isfinite(motor_temp_limit) or not 0 < motor_temp_limit <= 80:
            raise ValueError("motor_temp_limit must be in (0, 80] C")
        self.motor_temp_limit = motor_temp_limit
        self.pair_stop = pair_stop or threading.Event()
        self._streaming = False
        self.shutdown_errors: list[str] = []
        self.shutdown_report: dict[str, Any] = {}
        self._over_cap_since: float | None = None

    def open(self) -> StreamingSession:
        """Claim the bus and require timestamped driver feedback before activation."""
        super().open()
        try:
            for motor in self._motors:
                reader = getattr(motor, "get_state_snapshot", None)
                if not callable(reader):
                    raise RuntimeError(
                        "Motorbridge must provide get_state_snapshot(); see docs/adapter.md"
                    )
                reader()  # Also test the native ABI. An empty cache is valid before enable.
        except BaseException:
            self.close()
            raise
        return self

    def feedback(self) -> Feedback:
        """Read current cached replies. Never wait for a new full polling round."""
        with self._condition:
            if self._failure is not None:
                raise RuntimeError(f"Session faulted: {self._failure}") from self._failure
            if len(self._motors) != len(self.joints):
                raise RuntimeError("Session is not open")
            positions, efforts, temperatures, ages = [], [], [], []
            over_cap = False
            for axis, motor, cap in zip(self.joints, self._motors, self.caps, strict=True):
                snapshot = motor.get_state_snapshot()
                if snapshot is None:
                    raise native._FeedbackError(f"{axis['name']}: feedback is missing")
                state, age = snapshot.state, float(snapshot.feedback_age_s)
                pos, effort = float(state.pos), float(state.torq)
                mos, rotor = float(state.t_mos), float(state.t_rotor)
                temperature = max(mos, rotor)
                if not all(math.isfinite(v) for v in (age, pos, effort, mos, rotor)):
                    raise native._FeedbackError(f"{axis['name']}: feedback is not finite")
                if not 0 <= age <= 0.1:
                    raise native._FeedbackError(f"{axis['name']}: stale feedback ({age:.3f} s)")
                if state.status_code:
                    raise native._EnvelopeError(f"{axis['name']}: motor status {state.status_code}")
                if not 0 < temperature < self.motor_temp_limit:
                    raise native._EnvelopeError(
                        f"{axis['name']}: temperature stop ({temperature:.1f} C)"
                    )
                if (
                    not axis["lower"] - native.BOUNDS_TOLERANCE
                    <= pos
                    <= axis["upper"] + native.BOUNDS_TOLERANCE
                ):
                    raise native._EnvelopeError(f"{axis['name']}: measured position outside bounds")
                over_cap |= abs(effort) > cap
                positions.append(pos)
                efforts.append(effort)
                temperatures.append(temperature)
                ages.append(age)
            now = self._clock()
            if over_cap:
                if self._over_cap_since is None:
                    self._over_cap_since = now
                if now - self._over_cap_since >= 1:
                    raise native._EnvelopeError("Motor torque exceeded its cap for one second")
            else:
                self._over_cap_since = None
            return Feedback(tuple(positions), tuple(efforts), tuple(temperatures), now - max(ages))

    def activate(self) -> None:
        """Check disabled motor feedback, then enable the native holding loop."""
        with self._condition:
            if not self._motors:
                raise RuntimeError("open() is required")
            before = [motor.get_state_snapshot() for motor in self._motors]
            for motor in self._motors:
                motor.disable()
            deadline = time.monotonic() + 0.3
            while True:
                snapshots = [motor.get_state_snapshot() for motor in self._motors]
                ready = all(
                    new is not None
                    and (old is None or new.feedback_sequence > old.feedback_sequence)
                    and ((new.state.arbitration_id >> 22) & 3) == 0
                    for old, new in zip(before, snapshots, strict=True)
                )
                if ready:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "Fresh disabled-state replies are required before activation"
                    )
                time.sleep(0.005)
            self.feedback()
        super().activate()

    def observe(self) -> tuple[float, ...]:
        """Return current measured positions while streaming."""
        if self._enabled:
            return self.feedback().positions
        return super().observe()

    def set_position(
        self,
        target: Iterable[float],
        *,
        validate: Callable[[Sequence[float], Sequence[float]], None] | None = None,
    ) -> tuple[float, ...]:
        """Set a bounded target with a margin inside the native tracking stop."""
        requested = tuple(float(v) for v in target)
        if len(requested) != len(self.joints) or not all(math.isfinite(v) for v in requested):
            raise ValueError("A position command requires one finite value per motor")
        with self._condition:
            if not self._enabled or self._failure is not None or self.pair_stop.is_set():
                raise RuntimeError("An enabled, healthy pair is required")
            if self._path is not None or self._goal is not None:
                raise RuntimeError("A native move is in progress")
            measured = self.feedback().positions
            command = []
            for i, (value, actual, low, high) in enumerate(
                zip(requested, measured, self.lower, self.upper, strict=True)
            ):
                if not low <= value <= high:
                    raise ValueError("Position command exceeds joint bounds")
                reach = self._axis_limit(i, native.HOLD_REACH, native.GRIPPER_REACH) - 0.01
                command.append(max(low, min(high, max(actual - reach, min(actual + reach, value)))))
            if validate is not None:
                validate(measured, command)
            self._streaming = True
            self._command = command
            self._condition.notify_all()
            return tuple(command)

    def hold(self) -> None:
        """Cancel queued motion and hold the last measured position."""
        with self._condition:
            self._streaming = False
            self._path = self._goal = None
            self._command = self._clamp(self._positions)
            self._condition.notify_all()

    def _check_envelope(self) -> None:
        if self._failure is not None:
            return
        try:
            if self.pair_stop.is_set():
                raise native._EnvelopeError("The other arm stopped")
            sample = self.feedback()
            super()._check_envelope()
            if self._streaming:
                for i, (position, command) in enumerate(
                    zip(sample.positions, self._command, strict=True)
                ):
                    reach = self._axis_limit(i, native.HOLD_REACH, native.GRIPPER_REACH)
                    if abs(position - command) > reach + 1e-12:
                        raise native._EnvelopeError(
                            f"{self.joints[i]['name']}: tracking stop; measured={position:.5f}, "
                            f"command={command:.5f}, limit={reach:.3f} rad"
                        )
        except native._EnvelopeError:
            self.pair_stop.set()
            raise

    def _send(self, command: Any) -> None:
        if self._enabled and self._failure is None:
            self._check_envelope()
        super()._send(command)

    def _latch(self, reason: str, details: dict[str, Any] | None = None) -> None:
        self.pair_stop.set()
        super()._latch(reason, details)

    def _disable(self) -> None:
        if not self._activation_attempted and not self._enabled:
            return
        self._enabled = False
        pending = []
        for axis, motor in zip(self.joints, self._motors, strict=True):
            try:
                before = motor.get_state_snapshot()
                sequence = -1 if before is None else before.feedback_sequence
                motor.disable()
                pending.append((axis, motor, sequence))
            except Exception as error:
                self.shutdown_errors.append(f"{axis['name']} disable: {error}")
        deadline = time.monotonic() + 0.3
        replies = []
        while pending:
            remaining = []
            for axis, motor, sequence in pending:
                try:
                    sample = motor.get_state_snapshot()
                    if (
                        sample is None
                        or sample.feedback_sequence <= sequence
                        or not 0 <= sample.feedback_age_s <= 0.1
                        or ((sample.state.arbitration_id >> 22) & 3) != 0
                        or sample.state.status_code
                    ):
                        remaining.append((axis, motor, sequence))
                    else:
                        replies.append(
                            {"motor_id": axis["motor_id"], "mode": 0, "position": sample.state.pos}
                        )
                except Exception as error:
                    self.shutdown_errors.append(f"{axis['name']} disable feedback: {error}")
            pending = remaining
            if time.monotonic() >= deadline:
                self.shutdown_errors.extend(
                    f"{axis['name']}: disable not confirmed" for axis, _, _ in pending
                )
                break
            if pending:
                time.sleep(0.005)
        self.shutdown_report = {
            "replies": replies,
            "errors": list(self.shutdown_errors),
            "home": self._home,
            "disable_confirmed": not self.shutdown_errors and len(replies) == len(self._motors),
        }

    def close(self) -> None:
        """Park, verify disable replies, and release all resources even after an error."""
        if self._closed:
            return
        self._closed = True
        with self._condition:
            self._streaming = False
        try:
            self._home_then_disable()
        except BaseException as error:
            self.shutdown_errors.append(f"Home: {error}")
        finally:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2)
                if self._thread.is_alive():
                    self.shutdown_errors.append("Control thread did not stop")
            with self._condition:
                self._disable()
                resources = [self._controller, self._bus]
                self._controller = None
                self._bus = None
                for resource in resources:
                    if resource is not None:
                        try:
                            resource.close()
                        except Exception as error:
                            self.shutdown_errors.append(f"Resource close: {error}")
                if self.motor_log is not None:
                    try:
                        self.motor_log.close(
                            max_send_gap_s=self.max_send_gap,
                            failure=None if self._failure is None else str(self._failure),
                        )
                    except Exception as error:
                        self.shutdown_errors.append(f"Log close: {error}")
        self.shutdown_report.update(errors=list(self.shutdown_errors))
        if self.shutdown_errors:
            self.shutdown_report["disable_confirmed"] = False
            raise RuntimeError("; ".join(self.shutdown_errors))
