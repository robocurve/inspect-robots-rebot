"""Two-arm reBot embodiment with the Cartesian action layout used by YAM."""

from __future__ import annotations

import contextlib
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np
from inspect_robots import (
    Action,
    ActionSemantics,
    Box,
    CameraSpec,
    EmbodimentBase,
    EmbodimentInfo,
    Observation,
    ObservationSpace,
    Scene,
    StateField,
    StateSpec,
    StepResult,
)
from inspect_robots.errors import EmbodimentFault

from inspect_robots_b601.hardware import session as native

from .config import load_arm
from .kinematics import Kinematics, PathRejected
from .session import StreamingSession

LABELS = tuple(
    f"{side}_{axis}"
    for side in ("left", "right")
    for axis in ("x", "y", "z", "yaw", "pitch", "roll", "gripper")
)


class RebotEmbodiment(EmbodimentBase):
    """Drive two explicitly assigned CAN buses. Construction is inert."""

    def __init__(
        self,
        *,
        left_channel: str = "can1",
        right_channel: str = "can0",
        left_profile: str = "",
        right_profile: str = "",
        state_dir: str = ".hardware-state",
        left_state_dir: str = "",
        right_state_dir: str = "",
        camera_url: str = "",
        allow_motion: bool = False,
        motor_temp_limit: float = 80,
        feedback_recovery_timeout_s: float = 5.0,
        recover_feedback_in_trial: bool = False,
        rs00_mos_temp_limit: float | None = None,
        tool_floor_m: float = 0.12,
        wrist_yaw_command_margin_rad: float = 0.0,
        joint2_upper_tolerance_rad: float = 0.01,
        command_mode: str = "measured",
        command_step_rad: float = 0.2,
        command_resync_rad: float = 0.35,
        control_interface: str = "eef_pos",
        report_joint_eff: bool = True,
        docs_extra: str = "",
        session_factory: Any = None,
        camera_factory: Any = None,
    ) -> None:
        if control_interface != "eef_pos":
            raise ValueError("rebot_arms currently supports control_interface=eef_pos")
        if left_channel == right_channel:
            raise ValueError("Left and right arms require distinct CAN interfaces")
        if not np.isfinite(motor_temp_limit) or not 0 < motor_temp_limit <= 80:
            raise ValueError("motor_temp_limit must be in (0, 80] C")
        if not np.isfinite(tool_floor_m) or tool_floor_m >= 0.6:
            raise ValueError("tool_floor_m must be finite and below the 0.6 m upper bound")
        if command_mode not in ("measured", "previous"):
            raise ValueError("command_mode must be measured or previous")
        if not all(np.isfinite(v) and v > 0 for v in (command_step_rad, command_resync_rad)):
            raise ValueError("Command limits must be finite and positive")
        if command_step_rad + command_resync_rad >= 0.6:
            raise ValueError("Command limits must leave room inside the native fault envelope")
        if not np.isfinite(feedback_recovery_timeout_s) or not 0 < feedback_recovery_timeout_s <= 5:
            raise ValueError("feedback_recovery_timeout_s must be in (0, 5] seconds")
        self.feedback_recovery_timeout_s = feedback_recovery_timeout_s
        self.recover_feedback_in_trial = recover_feedback_in_trial
        self._recovering_feedback = False
        self._feedback_recoveries = 0
        self._operation_lock = threading.RLock()
        self._recovery_stop = threading.Event()
        self._recovery_thread: threading.Thread | None = None
        self._recovery_error: BaseException | None = None
        self.command_options: dict[str, Any] = {
            "command_mode": command_mode,
            "command_step_rad": command_step_rad,
            "command_resync_rad": command_resync_rad,
        }
        self.tool_floor_m = float(tool_floor_m)
        self._command_rejected = False
        self.configs = [
            load_arm(
                left_profile, left_channel, Path(left_state_dir or str(Path(state_dir) / "left"))
            ),
            load_arm(
                right_profile,
                right_channel,
                Path(right_state_dir or str(Path(state_dir) / "right")),
            ),
        ]
        if self.configs[0].state_dir == self.configs[1].state_dir:
            raise ValueError("Each arm requires its own persistent stop directory")
        tolerance = joint2_upper_tolerance_rad
        if not np.isfinite(tolerance) or not 0.01 <= tolerance <= 0.012:
            raise ValueError("Joint 2 upper tolerance must be between 0.010 and 0.012 rad")
        if tolerance > 0.01 and any(
            c.axes[1]["upper"] + tolerance > np.deg2rad(170) for c in self.configs
        ):
            raise ValueError("Extra tolerance requires a reduced joint 2 operating range")
        self.joint2_upper_tolerance_rad = tolerance
        margin = wrist_yaw_command_margin_rad
        if (
            not np.isfinite(margin)
            or margin < 0
            or any(2 * margin >= c.axes[4]["upper"] - c.axes[4]["lower"] for c in self.configs)
        ):
            raise ValueError("Wrist yaw command margin must leave a nonempty joint range")
        self.wrist_yaw_command_margin_rad = margin
        mos_limit = motor_temp_limit if rs00_mos_temp_limit is None else rs00_mos_temp_limit
        if not np.isfinite(mos_limit) or not 0 < mos_limit <= 100:
            raise ValueError("rs00_mos_temp_limit must be in (0, 100] C")
        self.rs00_mos_temp_limit = mos_limit
        self.allow_motion, self.motor_temp_limit = allow_motion, motor_temp_limit
        self.camera_url = camera_url
        self._session_factory = session_factory or StreamingSession
        self._camera_factory = camera_factory
        self._camera: Any = None
        self.sessions: list[Any] = []
        self.kinematics: list[Kinematics] = []
        self.instruction: str | None = None
        self.shutdown_reports: list[dict[str, Any]] = []
        self._driver: Any = None
        self._init_pose: Any = None
        self._report_effort = report_joint_eff
        # This is the declared operating box, not a guarantee of IK reachability.
        self.low = np.array(
            [
                0.08,
                -0.4,
                self.tool_floor_m,
                -np.pi,
                0,
                0,
                0,
                0.08,
                -0.08,
                self.tool_floor_m,
                -np.pi,
                0,
                0,
                0,
            ]
        )
        self.high = np.array([0.6, 0.05, 0.6, np.pi, 0, 0, 1, 0.6, 0.4, 0.6, np.pi, 0, 0, 1])
        fields = [
            StateField("joint_pos", (2, 7), "rad+normalized"),
            StateField("eef_state", (14,), "m+rad+normalized"),
            StateField("blocked", (2, 6), "direction"),
            StateField("command_rejected", (1,), "boolean"),
            StateField("feedback_recoveries", (1,), "count"),
        ]
        if report_joint_eff:
            fields.append(StateField("joint_eff", (2, 7), "Nm"))
        self.info = EmbodimentInfo(
            name="rebot_arms",
            action_space=Box(
                shape=(14,),
                low=self.low,
                high=self.high,
                semantics=ActionSemantics(
                    control_mode="eef_abs_pose",
                    rotation_repr="none",
                    gripper="continuous",
                    frame="base",
                    dim_labels=LABELS,
                ),
            ),
            observation_space=ObservationSpace(
                cameras=(CameraSpec("top", 480, 640),), state=StateSpec(fields=tuple(fields))
            ),
            control_hz=10,
            is_simulated=False,
            capabilities=frozenset({"self_paced"}),
            docs="Two reBot B601-RS arms, left then right. Each target uses its own base frame: "
            "+x forward, +y left, +z up. XYZ values are metres at the gripper end. "
            "Yaw, pitch, roll are radians relative to the measured reset orientation. "
            "Rotation is Rz(yaw) Ry(-pitch) Rx(roll). Pitch and roll are fixed at zero. "
            "Gripper 0 is calibrated closed; 1 is calibrated open. One overhead camera is used. "
            "Targets use bounded IK. Read eef_state to see actual movement. "
            "Commands run at 10 Hz without an added speed, acceleration, or jerk planner. "
            "The native tracking allowance can shorten a move; step info reports this. "
            "A high-effort joint with no progress holds at its measured position. "
            "State blocked reports -1 or +1 for the blocked joint direction; choose a pose "
            "that moves that joint back before pushing in that direction again. "
            "Joint bounds, torque caps, fresh feedback, temperature and watchdog stops apply. "
            "State command_rejected=1 means a proposed path crossed the tool floor. "
            "Both arms hold; choose a different pose, such as a smaller move away from the floor. "
            "The trial remains open. With in-trial feedback recovery enabled, a brief feedback "
            "loss pauses motion and disables both arms. Re-enable requires fresh disabled replies, "
            "no faults, and less than 0.05 rad arm-joint drift within the recovery window. "
            "State feedback_recoveries counts these pauses. Read the new pose before replanning. "
            "No inter-arm collision model is provided. " + docs_extra,
        )

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        """Check the camera, open both buses, then activate both arms."""
        if not self.allow_motion:
            raise EmbodimentFault("Set allow_motion only after the installation checks")
        if self.sessions:
            raise EmbodimentFault("Close the previous session before reset")
        self.shutdown_reports = []
        self.instruction = scene.instruction
        pair_stop = threading.Event()
        pair_feedback_stop = threading.Event()
        self._feedback_recoveries = 0
        try:
            if self._camera_factory is None:
                from .camera import LiveCamera

                factory = LiveCamera
            else:
                factory = self._camera_factory
            self._camera = factory(self.camera_url)
            self._camera.get()
            self.kinematics = [
                Kinematics(
                    config,
                    tool_floor_m=self.tool_floor_m,
                    wrist_yaw_command_margin_rad=self.wrist_yaw_command_margin_rad,
                )
                for config in self.configs
            ]
            for config in self.configs:
                session = self._session_factory(
                    config.profile,
                    config.state_dir,
                    home_wait=240,
                    feedback_recovery_timeout_s=self.feedback_recovery_timeout_s,
                    motor_temp_limit=self.motor_temp_limit,
                    rs00_mos_temp_limit=self.rs00_mos_temp_limit,
                    pair_stop=pair_stop,
                    pair_feedback_stop=pair_feedback_stop,
                    joint2_upper_tolerance_rad=self.joint2_upper_tolerance_rad,
                    **self.command_options,
                )
                self.sessions.append(session)
                session.open()
            for session in self.sessions:
                session.activate()
            time.sleep(1.0)  # Let the native gravity feed-forward reach full strength.
            positions = [s.observe() for s in self.sessions]
            for kin, q in zip(self.kinematics, positions, strict=True):
                kin.capture_reference(q[:6])
            self._init_pose = np.concatenate(positions)
            self._driver = self
            observation = self.observe()
            self._recovery_error = None
            self._recovery_stop.clear()
            if self.recover_feedback_in_trial:
                self._recovery_thread = threading.Thread(
                    target=self._watch_feedback, name="rebot-feedback-recovery", daemon=True
                )
                self._recovery_thread.start()
            return observation
        except BaseException as primary:
            try:
                self.close()
            except BaseException as cleanup:
                raise EmbodimentFault(
                    f"Reset failed: {primary}; shutdown failed: {cleanup}"
                ) from primary
            raise

    def _watch_feedback(self) -> None:
        while not self._recovery_stop.wait(0.02):
            if not self._operation_lock.acquire(timeout=0.05):
                continue
            try:
                if self._recovery_stop.is_set():
                    return
                if any(s.pair_feedback_stop.is_set() for s in self.sessions):
                    self._recover_feedback(native._FeedbackError("Feedback interruption"))
            except BaseException as error:
                self._recovery_error = error
                return
            finally:
                self._operation_lock.release()

    def observe(self) -> Observation:
        """Recover an eligible interruption before returning a fresh observation."""
        with self._operation_lock:
            if self._recovery_error is not None:
                raise EmbodimentFault(f"Feedback recovery failed: {self._recovery_error}")
            try:
                return self._observe_once()
            except Exception as error:
                if self._recover_feedback(error):
                    return self._observe_once()
                raise

    def _recover_feedback(self, error: BaseException) -> bool:
        if (
            not self.recover_feedback_in_trial
            or self._recovering_feedback
            or self._recovery_error is not None
            or len(self.sessions) != 2
        ):
            return False
        eligible = isinstance(error, native._FeedbackError) or any(
            isinstance(getattr(s, "_failure", None), native._FeedbackError) for s in self.sessions
        )
        if not eligible:
            return False
        if self._feedback_recoveries >= 3:
            raise RuntimeError("Feedback recovery budget exhausted") from error
        self._recovering_feedback = True
        losses = [s.feedback_loss_at for s in self.sessions if s.feedback_loss_at is not None]
        deadline = (min(losses) if losses else time.monotonic()) + self.feedback_recovery_timeout_s
        try:
            for s in self.sessions:
                s.pair_feedback_stop.set()
                s.pair_stop.set()
            with ThreadPoolExecutor(max_workers=2) as pool:
                futures = [
                    pool.submit(s.prepare_feedback_recovery, deadline) for s in self.sessions
                ]
                prepared = [future.result() for future in futures]
            if self._camera is None:
                raise RuntimeError("Camera missing during recovery")
            self._camera.get()
            for kin, (sample, _) in zip(self.kinematics, prepared, strict=True):
                kin.validate_path(sample.positions, sample.positions)
            if time.monotonic() >= deadline:
                raise RuntimeError("Feedback recovery timed out")
            for session, (sample, ledger) in zip(self.sessions, prepared, strict=True):
                session.accept_feedback_recovery(sample, ledger)
            for s in self.sessions:
                s.pair_feedback_stop.clear()
                s.pair_stop.clear()
            for s in self.sessions:
                if self._recovery_stop.is_set() or time.monotonic() >= deadline:
                    raise RuntimeError("Feedback recovery stopped before enable")
                s.activate()
            self._feedback_recoveries += 1
            for s in self.sessions:
                s._log("feedback-recovered", count=self._feedback_recoveries)
            return True
        except BaseException as recovery_error:
            # No home motion if a recovery check fails.
            for s in self.sessions:
                s._stop.set()
            for s in self.sessions:
                if s._thread is not None:
                    s._thread.join(timeout=2)
                with s._condition:
                    s._disable()
                    s._latch("feedback recovery failed", {"error": str(recovery_error)})
            raise
        finally:
            self._recovering_feedback = False

    def _observe_once(self) -> Observation:
        """Return measured arm state and the latest fresh overhead frame."""
        if len(self.sessions) != 2 or self._camera is None:
            raise EmbodimentFault("Both arms and the camera must be open")
        joints, efforts, times = [], [], []
        poses: list[float] = []
        for session, config, kin in zip(self.sessions, self.configs, self.kinematics, strict=True):
            sample = session.feedback()
            q = sample.positions
            grip = config.grip_from_motor(q[6])
            joints.append([*q[:6], grip])
            poses.extend(kin.observe(q[:6], grip))
            efforts.append(sample.efforts)
            times.append(sample.time)
        state = {
            "joint_pos": np.array(joints),
            "eef_state": np.array(poses),
            "blocked": np.array([getattr(s, "blocked", [0.0] * 6) for s in self.sessions]),
            "command_rejected": np.array([float(self._command_rejected)]),
            "feedback_recoveries": np.array([float(self._feedback_recoveries)]),
        }
        if self._report_effort:
            state["joint_eff"] = np.array(efforts)
        frame = self._camera.get()
        return Observation(
            state=state,
            images={"top": frame},
            state_time=min(times),
            image_times={"top": self._camera.at},
            instruction=self.instruction,
        )

    def step(self, action: Action) -> StepResult:
        """Serialize commands with feedback recovery."""
        with self._operation_lock:
            return self._step_once(action)

    def _step_once(self, action: Action) -> StepResult:
        """Resolve both targets before command submission and report executed targets."""
        started = time.monotonic()
        self._command_rejected = False
        try:
            if self._recovery_error is not None:
                raise EmbodimentFault(f"Feedback recovery failed: {self._recovery_error}")
            values = np.asarray(action.data, dtype=float)
            if values.shape != (14,) or not np.isfinite(values).all() or len(self.sessions) != 2:
                raise EmbodimentFault(
                    "A paired action requires 14 finite values and two active arms"
                )
            targets: list[list[float]] = []
            diagnostics: list[dict[str, Any]] = []
            for i, (session, config, kin) in enumerate(
                zip(self.sessions, self.configs, self.kinematics, strict=True)
            ):
                target = np.clip(
                    values[i * 7 : i * 7 + 7],
                    self.low[i * 7 : i * 7 + 7],
                    self.high[i * 7 : i * 7 + 7],
                )
                measured = session.observe()
                solution = kin.solve(target, measured[:6])
                targets.append([*solution.joints, config.grip_to_motor(float(target[6]))])
                diagnostics.append(
                    {
                        "side": ("left", "right")[i],
                        "position_error_m": solution.position_error_m,
                        "rotation_error_rad": solution.rotation_error_rad,
                        "ik_converged": solution.converged,
                    }
                )
            commands = []
            for session, target, row, kin in zip(
                self.sessions, targets, diagnostics, self.kinematics, strict=True
            ):
                try:
                    command = session.set_position(target, validate=kin.validate_path)
                except PathRejected as error:
                    for arm in self.sessions:
                        arm.hold()
                    self._command_rejected = True
                    time.sleep(max(0, 0.1 - (time.monotonic() - started)))
                    return StepResult(
                        observation=self.observe(),
                        info={
                            "command_rejected": True,
                            "reason": str(error),
                            "side": row["side"],
                            "arms": diagnostics,
                        },
                    )
                commands.append(list(command))
                row["command_resynced"] = bool(getattr(session, "command_resynced", False))
                row["requested_motor_rad"] = list(target)
                row["tracking_limited"] = not np.allclose(command, target, atol=1e-9, rtol=0)
            time.sleep(max(0, 0.1 - (time.monotonic() - started)))
            return StepResult(
                observation=self.observe(),
                info={
                    "arms": diagnostics,
                    "commanded_motor_rad": commands,
                    "step_duration_s": time.monotonic() - started,
                },
            )
        except BaseException as primary:
            try:
                if self._recover_feedback(primary):
                    return StepResult(
                        observation=self._observe_once(),
                        info={"feedback_recovered": True, "command_discarded": True},
                    )
            except BaseException as recovery_error:
                primary = recovery_error
            for session in self.sessions:
                with contextlib.suppress(Exception):
                    session.hold()
            try:
                self.close()
            except BaseException as cleanup:
                raise EmbodimentFault(
                    f"Step failed: {primary}; shutdown failed: {cleanup}"
                ) from primary
            raise primary

    def close(self) -> None:
        """Park and close both arms independently, then close the camera."""
        self._recovery_stop.set()
        if self._recovery_thread is not None:
            self._recovery_thread.join(timeout=self.feedback_recovery_timeout_s + 2)
            if self._recovery_thread.is_alive():
                raise EmbodimentFault("Feedback recovery thread did not stop")
            self._recovery_thread = None
        sessions, self.sessions = self.sessions, []
        errors = []
        if sessions:
            with ThreadPoolExecutor(max_workers=2) as pool:
                futures = [pool.submit(s.close) for s in sessions]
                for future in futures:
                    try:
                        future.result()
                    except BaseException as error:
                        errors.append(str(error))
            self.shutdown_reports = [s.shutdown_report for s in sessions]
        self._driver = self._init_pose = None
        if self._camera is not None:
            camera, self._camera = self._camera, None
            try:
                camera.close()
            except BaseException as error:
                errors.append(str(error))
        if errors:
            raise EmbodimentFault("Paired shutdown failed: " + "; ".join(errors))
