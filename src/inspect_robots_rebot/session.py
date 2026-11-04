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
        feedback_recovery_timeout_s: float = 5.0,
        rs00_mos_temp_limit: float | None = None,
        pair_stop: threading.Event | None = None,
        pair_feedback_stop: threading.Event | None = None,
        joint2_upper_tolerance_rad: float = native.BOUNDS_TOLERANCE,
        command_mode: str = "measured",
        command_step_rad: float = 0.2,
        command_resync_rad: float = 0.35,
        **kwargs: Any,
    ) -> None:
        super().__init__(*args, **kwargs)
        if not math.isfinite(motor_temp_limit) or not 0 < motor_temp_limit <= 80:
            raise ValueError("motor_temp_limit must be in (0, 80] C")
        if command_mode not in ("measured", "previous"):
            raise ValueError("command_mode must be measured or previous")
        if not all(math.isfinite(v) and v > 0 for v in (command_step_rad, command_resync_rad)):
            raise ValueError("Command limits must be finite and positive")
        if command_step_rad + command_resync_rad >= native.MAX_EXCURSION:
            raise ValueError("Command limits must leave room inside the native fault envelope")
        tolerance = joint2_upper_tolerance_rad
        if not math.isfinite(tolerance) or not native.BOUNDS_TOLERANCE <= tolerance <= 0.012:
            raise ValueError("Joint 2 upper tolerance must be between 0.010 and 0.012 rad")
        if tolerance > native.BOUNDS_TOLERANCE and self.joints[1][
            "upper"
        ] + tolerance > math.radians(170):
            raise ValueError("Extra tolerance requires a reduced joint 2 operating range")
        self.joint2_upper_tolerance_rad = tolerance
        self.command_mode = command_mode
        self.command_step_rad = command_step_rad
        self.command_resync_rad = command_resync_rad
        self.command_resynced = False
        mos_limit = motor_temp_limit if rs00_mos_temp_limit is None else rs00_mos_temp_limit
        if not math.isfinite(mos_limit) or not 0 < mos_limit <= 100:
            raise ValueError("rs00_mos_temp_limit must be in (0, 100] C")
        self.rs00_mos_temp_limit = mos_limit
        self.motor_temp_limit = motor_temp_limit
        if (
            not math.isfinite(feedback_recovery_timeout_s)
            or not 0 < feedback_recovery_timeout_s <= 5
        ):
            raise ValueError("feedback_recovery_timeout_s must be in (0, 5] seconds")
        self.feedback_recovery_timeout_s = feedback_recovery_timeout_s
        self._disable_attempted = False
        self.pair_stop = pair_stop or threading.Event()
        self.pair_feedback_stop = pair_feedback_stop or threading.Event()
        self.feedback_loss_at: float | None = None
        self._last_good_positions: tuple[float, ...] | None = None
        self._recovery_deadline: float | None = None
        self._streaming = False
        self._disable_on_feedback_loss = True
        self._parking = False
        self.shutdown_errors: list[str] = []
        self.shutdown_report: dict[str, Any] = {}
        self._over_cap_since: float | None = None
        self.blocked = [0.0] * 6
        self._blocked_at = [0.0] * 6
        self._stall_progress: list[tuple[float, float] | None] = [None] * 6

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

    def feedback(self, *, ignore_failure: bool = False) -> Feedback:
        """Read current cached replies. Never wait for a new full polling round."""
        with self._condition:
            if self._failure is not None and not ignore_failure:
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
                mos_limit = (
                    self.rs00_mos_temp_limit if axis["model"] == "rs-00" else self.motor_temp_limit
                )
                if temperature <= 0 or mos >= mos_limit or rotor >= self.motor_temp_limit:
                    raise native._EnvelopeError(
                        f"{axis['name']}: temperature stop "
                        f"(MOS={mos:.1f}/{mos_limit:.1f} C, "
                        f"rotor={rotor:.1f}/{self.motor_temp_limit:.1f} C)"
                    )
                upper_tolerance = (
                    self.joint2_upper_tolerance_rad
                    if axis["name"] == "joint2"
                    else native.BOUNDS_TOLERANCE
                )
                if (
                    not axis["lower"] - native.BOUNDS_TOLERANCE
                    <= pos
                    <= axis["upper"] + upper_tolerance
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
            if not ignore_failure:
                self._last_good_positions = tuple(positions)
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
        self._disable_attempted = False
        self.shutdown_report = {}
        self.shutdown_errors = []
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
        """Set a bounded target using the selected command reference."""
        requested = tuple(float(v) for v in target)
        if len(requested) != len(self.joints) or not all(math.isfinite(v) for v in requested):
            raise ValueError("A position command requires one finite value per motor")
        with self._condition:
            if not self._enabled or self._failure is not None or self.pair_stop.is_set():
                raise RuntimeError("An enabled, healthy pair is required")
            if self._path is not None or self._goal is not None:
                raise RuntimeError("A native move is in progress")
            measured = self.feedback().positions
            reference = measured
            resynced = False
            if self.command_mode == "previous" and self._streaming:
                reference = tuple(self._command)
                resynced = any(
                    abs(a - b) > self.command_resync_rad
                    for a, b in zip(reference[:6], measured[:6], strict=True)
                )
                if resynced:
                    reference = measured
            command = []
            blocked = list(self.blocked)
            for i, (value, actual, low, high) in enumerate(
                zip(requested, measured, self.lower, self.upper, strict=True)
            ):
                if not low <= value <= high:
                    raise ValueError("Position command exceeds joint bounds")
                if i < 6 and blocked[i]:
                    delta = (value - self._blocked_at[i]) * blocked[i]
                    if delta < -0.01:
                        blocked[i] = 0.0
                    elif delta > 0:
                        value = self._blocked_at[i]
                reach = self._axis_limit(i, native.HOLD_REACH, native.GRIPPER_REACH) - 0.01
                anchor = actual
                if self.command_mode == "previous" and i < 6:
                    anchor, reach = reference[i], self.command_step_rad
                command.append(max(low, min(high, max(anchor - reach, min(anchor + reach, value)))))
            if validate is not None:
                validate(measured, command)
            self.blocked = blocked
            self.command_resynced = resynced
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

    def _relieve_stalls(self, sample: Feedback) -> None:
        """Hold a high-effort joint that makes no progress, including during inference."""
        if not self._streaming or self._parking:
            self._stall_progress = [None] * 6
            return
        now = self._clock()
        for i in range(6):
            position = sample.positions[i]
            gap = self._command[i] - position
            if abs(gap) <= native.TOLERANCE or abs(sample.efforts[i]) < 0.9 * self.caps[i]:
                self._stall_progress[i] = None
                continue
            previous = self._stall_progress[i]
            if previous is None or abs(position - previous[0]) > 0.005:
                self._stall_progress[i] = (position, now)
            elif now - previous[1] >= 0.5:
                self.blocked[i] = math.copysign(1.0, gap)
                self._blocked_at[i] = max(self.lower[i], min(self.upper[i], position))
                self._command[i] = self._blocked_at[i]
                self._stall_progress[i] = None
                self._log(
                    "stream-stalled",
                    joint=self.joints[i]["name"],
                    measured_rad=position,
                    command_gap_rad=gap,
                    effort_nm=sample.efforts[i],
                    blocked_direction=self.blocked[i],
                )

    def _check_envelope(self) -> None:
        if self._failure is not None:
            return
        try:
            if self.pair_feedback_stop.is_set():
                raise native._FeedbackError("The other arm lost feedback")
            if self.pair_stop.is_set() and not self._parking:
                raise native._EnvelopeError("The other arm stopped")
            sample = self.feedback()
            # HOLD_REACH limits a new command. It is not a motor fault threshold.
            # Use current snapshot positions with the native loss-of-control envelope.
            for i, (position, command) in enumerate(
                zip(sample.positions, self._command, strict=True)
            ):
                envelope = self._axis_limit(i, native.MAX_EXCURSION, native.GRIPPER_EXCURSION)
                if abs(position - command) > envelope:
                    raise native._EnvelopeError(
                        f"{self.joints[i]['name']}: command envelope exceeded; "
                        f"measured={position:.5f}, command={command:.5f}, "
                        f"limit={envelope:.3f} rad"
                    )
            self._relieve_stalls(sample)
        except native._FeedbackError:
            self.feedback_loss_at = self.feedback_loss_at or time.monotonic()
            self.pair_feedback_stop.set()
            self.pair_stop.set()
            raise
        except native._EnvelopeError:
            self.pair_stop.set()
            raise

    def _send(self, command: Any) -> None:
        if self._enabled and self._failure is None:
            self._check_envelope()
        super()._send(command)

    def _latch(self, reason: str, details: dict[str, Any] | None = None) -> None:
        self.pair_stop.set()
        if reason == "arm feedback lost":
            self.feedback_loss_at = self.feedback_loss_at or time.monotonic()
            self.pair_feedback_stop.set()
        super()._latch(reason, details)

    def _disable(self) -> None:
        """Retry unconfirmed disable commands within one recovery window; never enable."""
        if self._disable_attempted or (not self._activation_attempted and not self._enabled):
            return
        self._disable_attempted = True
        self._enabled = False
        started = time.monotonic()
        deadline = started + self.feedback_recovery_timeout_s
        if self._recovery_deadline is not None:
            deadline = min(deadline, self._recovery_deadline)
        pending = list(zip(self.joints, self._motors, strict=True))
        replies: list[dict[str, Any]] = []
        attempts: dict[str, int] = {}
        last_errors: dict[str, str] = {}
        while pending and time.monotonic() < deadline:
            remaining = []
            for axis, motor in pending:
                name = axis["name"]
                if time.monotonic() >= deadline:
                    remaining.append((axis, motor))
                    continue
                attempts[name] = attempts.get(name, 0) + 1
                try:
                    before = motor.get_state_snapshot()
                    sequence = -1 if before is None else before.feedback_sequence
                    motor.disable()
                    sample = motor.get_state_snapshot()
                    if (
                        sample is None
                        or sample.feedback_sequence <= sequence
                        or not 0 <= sample.feedback_age_s <= 0.1
                        or ((sample.state.arbitration_id >> 22) & 3) != 0
                        or sample.state.status_code
                    ):
                        raise RuntimeError("disable not confirmed")
                    replies.append(
                        {
                            "motor_id": axis["motor_id"],
                            "mode": 0,
                            "position": sample.state.pos,
                            "feedback_sequence": sample.feedback_sequence,
                            "feedback_age_s": sample.feedback_age_s,
                        }
                    )
                except Exception as error:
                    last_errors[name] = str(error)
                    remaining.append((axis, motor))
            pending = remaining
            if pending:
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        self.shutdown_errors.extend(
            f"{axis['name']} disable: {last_errors.get(axis['name'], 'recovery deadline reached')}"
            for axis, _ in pending
        )
        self.shutdown_report = {
            "replies": replies,
            "errors": list(self.shutdown_errors),
            "home": self._home,
            "disable_confirmed": not self.shutdown_errors and len(replies) == len(self._motors),
            "recovery_timeout_s": self.feedback_recovery_timeout_s,
            "disable_elapsed_s": time.monotonic() - started,
            "disable_attempts": attempts,
            "retry_errors": last_errors,
        }

    def prepare_feedback_recovery(self, deadline: float) -> tuple[Feedback, dict[str, Any]]:
        """Stop the loop and confirm fresh disabled state; do not clear or enable."""
        self._recovery_deadline = deadline
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
            if self._thread.is_alive():
                raise RuntimeError("Recovery control thread did not stop")
        with self._condition:
            self._disable()
            if not self.shutdown_report.get("disable_confirmed"):
                raise RuntimeError("Recovery requires confirmed disable")
            if self._failure is not None and not isinstance(self._failure, native._FeedbackError):
                raise RuntimeError("Recovery refused for a non-feedback fault")
            if self._last_good_positions is None or self._ledger is None:
                raise RuntimeError("Recovery has no valid pose reference")
            ledger = self._ledger.snapshot()
            if ledger["stopped"] and ledger["reason"] not in (
                "arm feedback lost",
                "paired feedback recovery",
            ):
                raise RuntimeError("Recovery refused for another stop record")
            for motor in self._motors:
                if time.monotonic() >= deadline:
                    raise RuntimeError("Feedback recovery timed out")
                before = motor.get_state_snapshot()
                motor.disable()
                after = motor.get_state_snapshot()
                if (
                    before is None
                    or after is None
                    or after.feedback_sequence <= before.feedback_sequence
                    or not 0 <= after.feedback_age_s <= 0.1
                    or ((after.state.arbitration_id >> 22) & 3) != 0
                ):
                    raise RuntimeError("Fresh disabled replies are required for recovery")
                fault, warning = motor.robstride_get_fault_report()
                if fault or warning:
                    raise RuntimeError("Motor fault or warning prevents recovery")
            sample = self.feedback(ignore_failure=True)
            for i, (before, after) in enumerate(
                zip(self._last_good_positions, sample.positions, strict=True)
            ):
                if abs(after - before) > (0.05 if i < 6 else 0.3):
                    raise RuntimeError("Pose changed too far during feedback recovery")
            if self._ledger.snapshot() != ledger:
                raise RuntimeError("Stop evidence changed during recovery")
            return sample, ledger

    def accept_feedback_recovery(self, sample: Feedback, ledger: dict[str, Any]) -> None:
        """Acknowledge only the checked stop; prepare to enable at the measured pose."""
        with self._condition:
            if self._ledger is None or self._ledger.snapshot() != ledger:
                raise RuntimeError("Stop evidence changed before recovery acknowledgement")
            self._log("feedback-recovery-ready", positions_rad=sample.positions, prior_stop=ledger)
            self._ledger.acknowledge(
                details={"feedback_recovery": True, "positions_rad": sample.positions}
            )
            self._failure = None
            self._lost_control = False
            self._path = self._goal = None
            self._path_error = None
            self._positions = sample.positions
            self._command = list(sample.positions)
            self._last_good_positions = sample.positions
            self._streaming = False
            self._stop.clear()
            self._thread = None
            self._recovery_deadline = None
            self.feedback_loss_at = None
            self.blocked = [0.0] * 6
            self._stall_progress = [None] * 6
            self._ff_scale = 0.0
            self._ff_target = 1.0

    def close(self) -> None:
        """Park, verify disable replies, and release all resources even after an error."""
        if self._closed:
            return
        self._closed = True
        with self._condition:
            self._streaming = False
            self._parking = True
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
