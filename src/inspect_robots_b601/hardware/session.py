"""Motorbridge session for a B601-RS: every configured motor, the stop latch, and recovery.

ArmSession enables the six arm motors (and the gripper, when the profile has one) with
the profile gains and per-axis torque caps, holds position, and accepts small
multi-axis targets. It is not a collision-checked or fully qualified full-arm
controller. read() and recover() open the bus without enabling anything.

Motors are only disabled after the arm has walked back to its activation pose
(home), when CAN control is lost, or when the operator interrupts a blocked
homing. A fault holds the measured pose and then walks home.
"""

from __future__ import annotations

import contextlib
import json
import math
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from pathlib import Path
from typing import Any

try:
    from motorbridge import Controller, Mode
except ImportError:  # The motorbridge extra is optional; tests inject a controller factory.
    Controller = None
    Mode = None

from . import Motor, MotorController, PathLike, Profile
from .bus import BusClaim, Closable, channel_name
from .motor_log import MotorLog
from .state import StopLedger

ControllerFactory = Callable[[str], MotorController]
BusFactory = Callable[[Profile], Closable]
# gravity(q_arm) -> six feed-forward torques in N·m, raw motor frame.
GravityModel = Callable[[Sequence[float]], tuple[float, ...]]
# (command at the start, target, duration s, start time) of a timed ramp.
Goal = tuple[list[float], tuple[float, ...], float, float]
# Where operator-facing status lines go: stdout, or an Inspect operator session.
Notify = Callable[[str], None]


def _print_line(text: str) -> None:
    print(text, flush=True)


MECH_POS = 0x7019
LIMIT_TORQUE = 0x700B
WATCHDOG = 0x7028
JOINTS = tuple(f"joint{i}" for i in range(1, 7))
MAX_STEP = 0.1
MAX_VELOCITY = 0.05
MAX_EXCURSION = 0.6
MAX_DURATION = 10.0
# A moved arm axis must cover MIN_PROGRESS of its requested change and settle within
# TOLERANCE of its target inside SETTLE_S. P-only gravity sag is about 0.05 rad.
TOLERANCE = 0.08
MIN_PROGRESS = 0.5
SETTLE_S = 1.5
# While walking a path (home or a ready pose) the command never leads the measured
# position by more than this, so a blocked joint is not pushed harder and harder.
HOLD_REACH = 0.09
PATH_STALL_S = 5.0
LOST_CONTROL_TICKS = 10
# The motors cut torque when MIT frames stop for ~0.25 s (WATCHDOG ticks). The loop
# therefore reads one motor per tick with a short timeout, never all of them at once.
READ_TIMEOUT_MS = 20
OPEN_READ_TIMEOUT_MS = 200
STALE_READS = 25
# Gravity feed-forward fades in after enable and out (at home) before disable.
FF_RAMP_S = 1.0
BOUNDS_TOLERANCE = 0.01
GRIPPER_STEP = 0.5
GRIPPER_VELOCITY = 0.5
GRIPPER_REACH = 0.45
# A gripper that stops short of its path target for this long has closed on something.
GRIPPER_SETTLE_S = 1.0
GRIPPER_EXCURSION = 5.0


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a number")
    value = float(value)
    if not value == value or value in (float("inf"), float("-inf")):
        raise ValueError("number must be finite")
    return value


class MoveError(RuntimeError):
    """A move or path did not arrive; the failing axes now hold their measured pose."""


class _EnvelopeError(RuntimeError):
    pass


class _FeedbackError(_EnvelopeError):
    pass


def _default_controller(channel: str) -> MotorController:
    if Controller is None:
        raise RuntimeError(
            "motorbridge is not installed; python -m pip install 'inspect-robots-b601[motorbridge]'"
        )
    controller: MotorController = Controller(channel)
    return controller


class ArmSession:
    """Hold and move every configured motor of one arm through a background control loop."""

    def __init__(
        self,
        profile: Profile | PathLike,
        state_dir: PathLike,
        *,
        controller_factory: ControllerFactory = _default_controller,
        bus_factory: BusFactory = BusClaim,
        clock: Callable[[], float] = time.monotonic,
        period: float = 0.02,
        tolerance: float = TOLERANCE,
        gravity: GravityModel | None = None,
        home_wait: float | None = None,
        home_retry_s: float = 5.0,
        motor_log_dir: PathLike | None = None,
        notify: Notify = _print_line,
    ) -> None:
        self.notify = notify
        self.profile: Profile = (
            profile if isinstance(profile, dict) else json.loads(Path(profile).read_text())
        )
        self.state_dir = Path(state_dir)
        self.channel = channel_name(self.profile)
        self._controller_factory, self._bus_factory = controller_factory, bus_factory
        self._clock, self._period, self._tolerance = clock, period, _number(tolerance)
        if gravity is None and self.profile.get("gravity_feedforward") is True:
            from .gravity import compensation_torques

            gravity = compensation_torques
        self._gravity = gravity
        self._ff_scale = 0.0
        self._ff_target = 1.0
        # Seconds to keep holding and retrying when home is blocked; None waits for the operator.
        self._home_wait = None if home_wait is None else _number(home_wait)
        self._home_retry_s = _number(home_retry_s)
        arm_joints = self.profile["joints"]
        if len(arm_joints) != 6 or [j["name"] for j in arm_joints] != list(JOINTS):
            raise ValueError("profile must contain ordered joint1..joint6")
        gripper = self.profile.get("gripper")
        self.joints: list[dict[str, Any]] = list(arm_joints) + ([dict(gripper)] if gripper else [])
        self.gripper_index: int | None = len(self.joints) - 1 if gripper else None
        if gripper and (
            gripper.get("name") != "gripper"
            or type(gripper.get("motor_id")) is not int
            or gripper.get("model") != "rs-00"
        ):
            raise ValueError("gripper must be an rs-00 motor with name 'gripper'")
        self.lower = tuple(_number(j["lower"]) for j in self.joints)
        self.upper = tuple(_number(j["upper"]) for j in self.joints)
        if any(low >= high for low, high in zip(self.lower, self.upper, strict=False)):
            raise ValueError("joint bounds must be increasing")
        if any("torque_cap_nm" not in j for j in self.joints):
            raise ValueError("every axis in the profile needs a torque_cap_nm")
        self.caps = tuple(_number(j["torque_cap_nm"]) for j in self.joints)
        if any(not 0 < cap <= 36.0 for cap in self.caps):
            raise ValueError("torque caps must be in (0, 36] Nm")
        self.bounds_tolerance = BOUNDS_TOLERANCE
        self._condition = threading.Condition(threading.RLock())
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._controller: MotorController | None = None
        self._motors: list[Motor] = []
        self._bus: Closable | None = None
        self._enabled = False
        self._activation_attempted = False
        self._closed = False
        self._failure: BaseException | None = None
        self._lost_control = False
        self._io_errors = 0
        self._positions: tuple[float, ...] = ()
        self._command: list[float] = []
        self._home: tuple[float, ...] | None = None
        self._goal: Goal | None = None
        self._path: tuple[float, ...] | None = None
        self._path_error: str | None = None
        self._path_best: float | None = None
        self._path_best_at: float | None = None
        self._gripper_last: float | None = None
        self._gripper_moved_at: float | None = None
        self._ledger: StopLedger | None = None
        self._last_send: float | None = None
        self.max_send_gap = 0.0
        # Every session that opens the bus writes a motor/temperature JSONL log (see motor_log.py).
        self._motor_log_dir = (
            Path(motor_log_dir)
            if motor_log_dir is not None
            else self.state_dir.resolve().parent / "logs" / "motor"
        )
        self.motor_log: MotorLog | None = None
        self._feedforward: tuple[float, ...] = (0.0,) * 6
        self._next_read = 0
        self._misses: list[int] = []
        self._cycles = 0

    def open(self) -> ArmSession:
        """Claim the bus and read every motor without enabling any; refuses while stopped."""
        if self._closed:
            raise RuntimeError("session is closed")
        if self._controller is not None:
            return self
        if self._ledger is None:
            self._ledger = StopLedger(self.state_dir, self.profile.get("legacy_stop_latch"))
        if self._ledger.stopped:
            raise RuntimeError("stop is latched; explicit recovery is required")
        try:
            self._bus = self._bus_factory(self.profile)
            self._controller = self._controller_factory(self.channel)
            self._motors = [
                self._controller.add_robstride_motor(j["motor_id"], 0xFD, j["model"])
                for j in self.joints
            ]
            self._positions = self._read()
            self._command = list(self._positions)
            try:
                self.motor_log = MotorLog.create(
                    self._motor_log_dir, self.joints, source=self.profile.get("name", "b601")
                )
                self.motor_log.event(
                    "session-open", measured_rad=list(self._positions), channel=self.channel
                )
            except Exception as error:
                self.notify(f"motor log unavailable: {error}")
            return self
        except BaseException:
            self.close()
            raise

    def _read(self) -> tuple[float, ...]:
        """Blocking read of every motor; only used while no control loop is running."""
        return tuple(
            _number(motor.robstride_get_param_f32(MECH_POS, OPEN_READ_TIMEOUT_MS))
            for motor in self._motors
        )

    def _read_next(self) -> bool:
        """Read one motor with a short timeout; returns True after a full round of motors."""
        index = self._next_read
        self._next_read = (index + 1) % len(self._motors)
        try:
            value = _number(self._motors[index].robstride_get_param_f32(MECH_POS, READ_TIMEOUT_MS))
        except Exception as error:
            self._misses[index] += 1
            if self._misses[index] >= STALE_READS:
                raise _FeedbackError(
                    f"{self.joints[index]['name']} position reads failed "
                    f"{self._misses[index]} times in a row"
                ) from error
        else:
            self._misses[index] = 0
            positions = list(self._positions)
            positions[index] = value
            self._positions = tuple(positions)
        return self._next_read == 0

    def _fresh_positions(self, timeout: float = 1.0) -> tuple[float, ...]:
        """Wait (lock released) for the loop to complete a full round of reads."""
        target = self._cycles + 1
        deadline = self._clock() + timeout
        while self._cycles < target and self._failure is None and self._enabled:
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise RuntimeError("position feedback did not refresh")
            self._condition.wait(timeout=min(0.05, remaining))
        return self._positions

    def _axis_limit(self, index: int, arm: float, gripper: float) -> float:
        return gripper if self.gripper_index == index else arm

    def _arm_axes(self) -> list[int]:
        return [index for index in range(len(self.joints)) if index != self.gripper_index]

    def _clamp(self, values: Iterable[float]) -> list[float]:
        return [
            min(high, max(low, value))
            for value, low, high in zip(values, self.lower, self.upper, strict=False)
        ]

    def _send(self, command: Sequence[float]) -> None:
        torques: tuple[float, ...] = (0.0,) * 6
        if self._gravity is not None and self._ff_scale > 0:
            torques = tuple(self._ff_scale * tau for tau in self._gravity(command[:6]))
        self._feedforward = torques
        for index, (joint, motor, position) in enumerate(
            zip(self.joints, self._motors, command, strict=False)
        ):
            motor.send_mit(
                position, 0.0, joint["kp"], joint["kd"], torques[index] if index < 6 else 0.0
            )
        now = self._clock()
        if self._last_send is not None:
            self.max_send_gap = max(self.max_send_gap, now - self._last_send)
        self._last_send = now

    def activate(self) -> None:
        """Cap torque, enable every motor holding its measured pose, and start the control loop."""
        with self._condition:
            if self._controller is None or self._ledger is None:
                raise RuntimeError("open() is required")
            if self._enabled:
                return
            if self._ledger.stopped:
                raise RuntimeError("stop is latched; explicit recovery is required")
            self._activation_attempted = True
            try:
                for joint, motor, cap in zip(self.joints, self._motors, self.caps, strict=False):
                    motor.robstride_write_param_f32(LIMIT_TORQUE, cap)
                    motor.robstride_write_param_u32(WATCHDOG, 5000)
                    if abs(motor.robstride_get_param_f32(LIMIT_TORQUE) - cap) > 1e-6:
                        raise RuntimeError(f"{joint['name']} torque cap readback mismatch")
                    if Mode is not None:
                        motor.ensure_mode(Mode.MIT, 1000)
                self._positions = self._read()
                if self._home is None:
                    self._home = tuple(self._positions)
                self._command = list(self._positions)
                self._send(self._command)
                for motor in self._motors:
                    motor.enable()
                self._misses = [0] * len(self._motors)
                self._enabled = True
                self._log(
                    "motors-enabled",
                    measured_rad=list(self._positions),
                    torque_caps_nm=list(self.caps),
                    gravity_feedforward=self._gravity is not None,
                )
            except BaseException:
                # Nothing has moved yet: the arm is still at its activation pose.
                self._disable()
                raise
            self._thread = threading.Thread(target=self._loop, name="b601-arm", daemon=True)
            self._thread.start()
            self._condition.notify_all()

    def _loop(self) -> None:
        last = self._clock()
        while not self._stop.is_set():
            with self._condition:
                now = self._clock()
                elapsed, last = min(now - last, 0.1), now
                try:
                    step = elapsed / FF_RAMP_S
                    self._ff_scale = max(
                        self._ff_scale - step, min(self._ff_scale + step, self._ff_target)
                    )
                    goal = self._goal
                    if goal is not None and self._failure is None:
                        start, target, duration, started = goal
                        fraction = min(1.0, (now - started) / duration)
                        self._command = [
                            a + (b - a) * fraction for a, b in zip(start, target, strict=False)
                        ]
                        if fraction >= 1.0:
                            self._goal = None
                            self._condition.notify_all()
                    elif self._path is not None:
                        self._advance_path(elapsed)
                    self._send(self._command)
                    self._io_errors = 0
                    if self.motor_log is not None:
                        self.motor_log.sample(
                            self._motors,
                            measured=self._positions,
                            command=self._command,
                            feedforward=self._feedforward,
                            caps=self.caps,
                        )
                    if self._read_next():
                        self._cycles += 1
                        self._condition.notify_all()
                        if self._failure is None:
                            self._check_envelope()
                        if self._path is not None:
                            self._check_path(now)
                except _EnvelopeError as error:
                    if isinstance(error, _FeedbackError) and getattr(
                        self, "_disable_on_feedback_loss", False
                    ):
                        self._failure = error
                        self._lost_control = True
                        self._path = self._goal = None
                        self._latch("arm feedback lost", {"error": str(error)})
                        try:
                            self._disable()
                        finally:
                            self._condition.notify_all()
                        break
                    # Keep control: latch the stop, hold where the arm is, let the owner walk home.
                    self._latch(
                        "arm session stopped",
                        {"error": str(error), "motors": self._motor_diagnostics()},
                    )
                    self._failure = error
                    self._goal = None
                    if self._path is not None:
                        self._path, self._path_error = None, str(error)
                    self._command = self._clamp(self._positions)
                    self._condition.notify_all()
                except Exception as error:
                    self._io_errors += 1
                    if self._io_errors >= LOST_CONTROL_TICKS:
                        # Commands no longer reach the motors, so homing is impossible.
                        self._latch("arm control lost", {"error": str(error)})
                        self._failure = error
                        self._lost_control = True
                        self._path = self._goal = None
                        self._disable()
                        self._condition.notify_all()
                        break
            self._stop.wait(self._period)

    def _log(self, status: str, **fields: Any) -> None:
        if self.motor_log is not None:
            self.motor_log.event(status, **fields)

    def _latch(self, reason: str, details: dict[str, Any] | None = None) -> None:
        self._log("stop-latched", reason=reason, details=details)
        if self._ledger is not None:
            with contextlib.suppress(Exception):
                self._ledger.stop(reason, details=details)

    def _motor_diagnostics(self) -> list[dict[str, Any]]:
        """Cached motor status/fault words for the stop record; never raises."""
        report: list[dict[str, Any]] = []
        for joint, motor in zip(self.joints, self._motors, strict=False):
            entry: dict[str, Any] = {"joint": joint["name"]}
            try:
                state = motor.get_state()
                if state is not None:
                    for field in ("status_code", "torq", "pos", "t_mos", "t_rotor"):
                        if hasattr(state, field):
                            entry[field] = getattr(state, field)
            except Exception as error:
                entry["state_error"] = type(error).__name__
            try:
                fault, warning = motor.robstride_get_fault_report()
                entry.update(fault=fault, warning=warning)
            except Exception as error:
                entry["fault_error"] = type(error).__name__
            report.append(entry)
        return report

    def _check_envelope(self) -> None:
        for index, (measured, command) in enumerate(
            zip(self._positions, self._command, strict=False)
        ):
            if abs(measured - command) > self._axis_limit(index, MAX_EXCURSION, GRIPPER_EXCURSION):
                raise _EnvelopeError(
                    f"measured pose left the command envelope: {self.joints[index]['name']} "
                    f"measured {measured:+.4f} rad, command {command:+.4f} rad, "
                    f"max send gap {self.max_send_gap:.3f} s"
                )

    def _advance_path(self, elapsed: float) -> None:
        path = self._path
        if path is None:
            return
        command = []
        for index, (current, goal, measured) in enumerate(
            zip(self._command, path, self._positions, strict=False)
        ):
            step = self._axis_limit(index, MAX_VELOCITY, GRIPPER_VELOCITY) * elapsed
            reach = self._axis_limit(index, HOLD_REACH, GRIPPER_REACH)
            following = current + max(-step, min(step, goal - current))
            command.append(max(measured - reach, min(measured + reach, following)))
        self._command = command

    def _check_path(self, now: float) -> None:
        path = self._path
        if path is None:
            return
        arm = self._arm_axes()
        distance = max(abs(self._positions[i] - path[i]) for i in arm)
        commanded = all(abs(self._command[i] - path[i]) < 1e-9 for i in arm)
        gripper, gripper_done = self.gripper_index, True
        if gripper is not None and abs(self._command[gripper] - path[gripper]) >= 1e-9:
            # The gripper is force-limited: stopping short means it is holding something.
            position = self._positions[gripper]
            if (
                self._gripper_last is None
                or self._gripper_moved_at is None
                or abs(position - self._gripper_last) > 0.005
            ):
                self._gripper_last, self._gripper_moved_at = position, now
            gripper_done = now - self._gripper_moved_at > GRIPPER_SETTLE_S
        if commanded and gripper_done and distance <= self._tolerance:
            self._path = None
            self._condition.notify_all()
            return
        if (
            distance <= self._tolerance
            or self._path_best is None
            or self._path_best_at is None
            or distance < self._path_best - 0.005
        ):
            self._path_best, self._path_best_at = distance, now
        elif now - self._path_best_at > PATH_STALL_S:
            worst = max(arm, key=lambda i: abs(self._positions[i] - path[i]))
            self._path_error = (
                f"{self.joints[worst]['name']} stalled at "
                f"{self._positions[worst]:+.4f} rad, path target "
                f"{path[worst]:+.4f} rad"
            )
            self._log(
                "path-stalled",
                error=self._path_error,
                measured_rad=list(self._positions),
                target_rad=list(path),
            )
            self._path = None
            self._command = self._clamp(self._positions)
            self._condition.notify_all()

    def observe(self) -> tuple[float, ...]:
        """Fresh measured positions of every axis; raises once the session has faulted."""
        with self._condition:
            if self._failure is not None:
                raise RuntimeError(f"session faulted: {self._failure}") from self._failure
            if self._thread is not None and self._thread.is_alive():
                return tuple(self._fresh_positions())
            self._positions = self._read()
            return self._positions

    def _shortfalls(
        self, start: Sequence[float], target: Sequence[float], measured: Sequence[float]
    ) -> list[tuple[int, str]]:
        problems = []
        for index in self._arm_axes():
            requested = target[index] - start[index]
            moved = measured[index] - start[index]
            error = measured[index] - target[index]
            stalled = abs(requested) >= 0.01 and moved * math.copysign(
                1.0, requested
            ) < MIN_PROGRESS * abs(requested)
            if stalled or abs(error) > self._tolerance:
                problems.append(
                    (
                        index,
                        f"{self.joints[index]['name']} requested {requested:+.3f} rad, "
                        f"moved {moved:+.3f} rad (target {target[index]:+.4f}, "
                        f"measured {measured[index]:+.4f})",
                    )
                )
        return problems

    def move(self, target: Iterable[float], duration: float) -> tuple[float, ...]:
        """Ramp every axis to ``target`` over ``duration`` seconds; returns the measured pose."""
        requested = tuple(_number(value) for value in target)
        duration = _number(duration)
        if len(requested) != len(self.joints) or not 0 < duration <= MAX_DURATION:
            raise ValueError("move requires one target per configured axis and a valid duration")
        with self._condition:
            if not self._enabled or self._failure is not None or self._path is not None:
                raise RuntimeError("an enabled, healthy session is required")
            start = list(self.observe())
            if any(
                value < low - self.bounds_tolerance or value > high + self.bounds_tolerance
                for value, low, high in zip(requested, self.lower, self.upper, strict=False)
            ):
                raise ValueError("target outside joint bounds")
            goal = tuple(self._clamp(requested))
            if any(
                abs(a - b) > self._axis_limit(index, MAX_STEP, GRIPPER_STEP) + 1e-9
                for index, (a, b) in enumerate(zip(goal, start, strict=False))
            ):
                raise ValueError("target exceeds the per-step limit")
            if any(
                abs(a - b) > self._axis_limit(index, MAX_EXCURSION, GRIPPER_EXCURSION) + 1e-9
                for index, (a, b) in enumerate(zip(goal, self._command, strict=False))
            ):
                raise ValueError("target exceeds the activation envelope")
            travel = max(
                abs(a - b) / self._axis_limit(index, MAX_VELOCITY, GRIPPER_VELOCITY)
                for index, (a, b) in enumerate(zip(goal, self._command, strict=False))
            )
            if travel / duration > 1.0 + 1e-9:
                raise ValueError("target exceeds the velocity limit")
            # Ramp from the held command, not the measurement, so the hold never jumps.
            self._goal = (list(self._command), goal, duration, self._clock())
            self._condition.notify_all()
            deadline = self._clock() + duration + 1.0
            while self._goal is not None and self._failure is None:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise RuntimeError("multi-joint goal did not finish")
                self._condition.wait(timeout=min(0.02, remaining))
            settle_until = self._clock() + SETTLE_S
            while self._failure is None:
                problems = self._shortfalls(start, goal, self._fresh_positions())
                if not problems or self._clock() >= settle_until:
                    break
            if self._failure is not None:
                raise RuntimeError(
                    f"session faulted during move: {self._failure}"
                ) from self._failure
            if problems:
                # Stop pushing on whatever blocked the failing axes; the rest keep their targets.
                for index, _ in problems:
                    self._command[index] = min(
                        self.upper[index], max(self.lower[index], self._positions[index])
                    )
                error = "move did not converge: " + "; ".join(text for _, text in problems)
                self._log("move-shortfall", error=error, measured_rad=list(self._positions))
                raise MoveError(error)
            return self._positions

    def go_to(
        self, pose: Iterable[float], *, timeout: float | None = None, recovery: bool = False
    ) -> tuple[float, ...]:
        """Walk every axis to an absolute pose at the velocity limit; returns the measured pose."""
        target = tuple(_number(value) for value in pose)
        if len(target) != len(self.joints):
            raise ValueError("go_to requires one value per configured axis")
        if any(
            value < low - self.bounds_tolerance or value > high + self.bounds_tolerance
            for value, low, high in zip(target, self.lower, self.upper, strict=False)
        ):
            raise ValueError("pose outside joint bounds")
        with self._condition:
            if not self._enabled or (self._failure is not None and not recovery):
                raise RuntimeError("an enabled, healthy session is required")
            self._goal = None
            path = tuple(self._clamp(target))
            self._path, self._path_error = path, None
            self._log("path-start", target_rad=list(path), recovery=recovery)
            self._path_best = self._path_best_at = None
            self._gripper_last = self._gripper_moved_at = None
            self._condition.notify_all()
            deadline = None if timeout is None else self._clock() + timeout
            while self._path is not None and self._enabled:
                if deadline is not None and self._clock() >= deadline:
                    self._path = None
                    self._command = self._clamp(self._positions)
                    raise MoveError("path timed out")
                self._condition.wait(timeout=0.05)
            if not self._enabled:
                raise RuntimeError(f"control lost while moving: {self._failure}")
            if self._path_error is not None:
                raise MoveError(self._path_error)
            return self._positions

    def go_home(self) -> tuple[float, ...]:
        """Walk back to the activation pose, even after a fault; returns the measured pose."""
        if self._home is None or not self._enabled:
            return self._positions
        distance = max(
            abs(a - b) / self._axis_limit(index, MAX_VELOCITY, GRIPPER_VELOCITY)
            for index, (a, b) in enumerate(zip(self._command, self._home, strict=False))
        )
        return self.go_to(self._home, timeout=distance + PATH_STALL_S + 10.0, recovery=True)

    def _home_then_disable(self) -> None:
        """Walk home, then disable. A blocked home keeps holding until reached or interrupted."""
        if not self._enabled:
            return
        waited_since = self._clock()
        while self._enabled:
            try:
                self.go_home()
                break
            except KeyboardInterrupt:
                self.notify("operator interrupt: disabling motors before reaching home")
                break
            except (MoveError, RuntimeError) as error:
                if not self._enabled:
                    break
                if self._home_wait is not None and self._clock() - waited_since >= self._home_wait:
                    self.notify(f"cannot reach home ({error}); hold timeout reached, disabling")
                    break
                self.notify(
                    f"cannot reach home ({error}); holding the current pose. Clear the "
                    f"obstruction; retrying in {self._home_retry_s:g} s. Ctrl-C disables "
                    "the motors where they are."
                )
                try:
                    time.sleep(self._home_retry_s)
                except KeyboardInterrupt:
                    self.notify("operator interrupt: disabling motors before reaching home")
                    break
        self._fade_feedforward()
        with self._condition:
            self._disable()

    def _fade_feedforward(self) -> None:
        """At home the support carries the arm; let the PD hold alone before torque is removed."""
        if self._gravity is None or not self._enabled:
            return
        with self._condition:
            self._ff_target = 0.0
            deadline = self._clock() + FF_RAMP_S + 1.0
            while self._ff_scale > 0 and self._enabled and self._clock() < deadline:
                self._condition.wait(timeout=0.05)

    def _disable(self) -> None:
        if not self._activation_attempted and not self._enabled:
            return
        if self._enabled and self.motor_log is not None:
            home = (
                None
                if self._home is None
                else max(abs(a - b) for a, b in zip(self._positions, self._home, strict=False))
            )
            self.motor_log.sample(
                self._motors,
                measured=self._positions,
                command=self._command,
                feedforward=self._feedforward,
                caps=self.caps,
                force=True,
            )
            self._log(
                "motors-disabled",
                measured_rad=list(self._positions),
                max_distance_from_home_rad=home,
                lost_control=self._lost_control,
            )
        self._enabled = False
        if self._controller is None:
            return
        with contextlib.suppress(Exception):
            self._controller.disable_all()
        for motor in self._motors:
            with contextlib.suppress(Exception):
                motor.disable()

    def stop(self, reason: str) -> None:
        """Latch the stop, walk home, then disable."""
        self._latch(reason)
        self._home_then_disable()
        self._stop.set()
        with self._condition:
            self._condition.notify_all()

    def close(self) -> None:
        """Walk home, disable every motor, then release the controller, log and bus; idempotent."""
        if self._closed:
            return
        self._closed = True
        self._home_then_disable()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        with self._condition:
            self._disable()
            if self._controller is not None:
                with contextlib.suppress(Exception):
                    self._controller.close()
                self._controller = None
            if self.motor_log is not None:
                self.motor_log.close(
                    max_send_gap_s=round(self.max_send_gap, 4),
                    failure=None if self._failure is None else str(self._failure),
                )
            if self._bus is not None:
                with contextlib.suppress(Exception):
                    self._bus.close()
                self._bus = None


def _axes(profile: Profile) -> list[dict[str, Any]]:
    gripper = profile.get("gripper")
    return list(profile["joints"]) + ([gripper] if gripper else [])


def _fresh_read(
    profile: Profile, controller_factory: ControllerFactory, bus_factory: BusFactory
) -> tuple[dict[str, float], dict[str, dict[str, Any]]]:
    """Claim the bus, read every motor's position and fault report, and release it;
    enables nothing.
    """
    axes = _axes(profile)
    claim = bus_factory(profile)
    try:
        controller = controller_factory(channel_name(profile))
        try:
            motors = [
                controller.add_robstride_motor(axis["motor_id"], 0xFD, axis["model"])
                for axis in axes
            ]
            positions = {
                axis["name"]: _number(motor.robstride_get_param_f32(MECH_POS, OPEN_READ_TIMEOUT_MS))
                for axis, motor in zip(axes, motors, strict=False)
            }
            faults: dict[str, dict[str, Any]] = {}
            for axis, motor in zip(axes, motors, strict=False):
                try:
                    fault, warning = motor.robstride_get_fault_report()
                except Exception as error:
                    faults[axis["name"]] = {"error": type(error).__name__}
                else:
                    faults[axis["name"]] = {"fault": fault, "warning": warning}
        finally:
            controller.close()
    finally:
        claim.close()
    return positions, faults


def read(
    profile: Profile,
    state_dir: PathLike,
    *,
    controller_factory: ControllerFactory = _default_controller,
    bus_factory: BusFactory = BusClaim,
) -> dict[str, Any]:
    """Fresh positions and fault reports of every motor, with the stop latch;
    works while latched.
    """
    ledger = StopLedger(state_dir, profile.get("legacy_stop_latch"))
    positions, faults = _fresh_read(profile, controller_factory, bus_factory)
    return {"positions_rad": positions, "faults": faults, "stop": ledger.snapshot()}


def recover(
    profile: Profile,
    state_dir: PathLike,
    *,
    controller_factory: ControllerFactory = _default_controller,
    bus_factory: BusFactory = BusClaim,
) -> dict[str, Any]:
    """Acknowledge the stop latch after fresh reads of every motor; every motor stays disabled.

    Refuses when a motor reports a fault or when new stop evidence appears during the reads.
    Returns the prior stop record and the readings that were acknowledged.
    """
    ledger = StopLedger(state_dir, profile.get("legacy_stop_latch"))
    before = ledger.snapshot()
    positions, faults = _fresh_read(profile, controller_factory, bus_factory)
    faulted = sorted(name for name, report in faults.items() if report.get("fault"))
    if faulted:
        raise RuntimeError(f"motor faults on {', '.join(faulted)}; clear them before recovery")
    if ledger.snapshot() != before:
        raise RuntimeError("new stop evidence appeared during recovery; acknowledgement refused")
    ledger.acknowledge(details={"fresh_positions_rad": positions, "faults": faults})
    return {"prior": before, "positions_rad": positions, "faults": faults}
