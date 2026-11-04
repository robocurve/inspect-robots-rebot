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

from .config import load_arm
from .kinematics import Kinematics
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
        tool_floor_m: float = 0.12,
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
        self.tool_floor_m = float(tool_floor_m)
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
            "Joint bounds, torque caps, fresh feedback, temperature and watchdog stops apply. "
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
        try:
            if self._camera_factory is None:
                from .camera import LiveCamera

                factory = LiveCamera
            else:
                factory = self._camera_factory
            self._camera = factory(self.camera_url)
            self._camera.get()
            self.kinematics = [
                Kinematics(config, tool_floor_m=self.tool_floor_m) for config in self.configs
            ]
            for config in self.configs:
                session = self._session_factory(
                    config.profile,
                    config.state_dir,
                    home_wait=240,
                    motor_temp_limit=self.motor_temp_limit,
                    pair_stop=pair_stop,
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
            return self.observe()
        except BaseException as primary:
            try:
                self.close()
            except BaseException as cleanup:
                raise EmbodimentFault(
                    f"Reset failed: {primary}; shutdown failed: {cleanup}"
                ) from primary
            raise

    def observe(self) -> Observation:
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
        state = {"joint_pos": np.array(joints), "eef_state": np.array(poses)}
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
        """Resolve both targets before command submission and report executed targets."""
        started = time.monotonic()
        try:
            values = np.asarray(action.data, dtype=float)
            if values.shape != (14,) or not np.isfinite(values).all() or len(self.sessions) != 2:
                raise EmbodimentFault(
                    "A paired action requires 14 finite values and two active arms"
                )
            targets, diagnostics = [], []
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
                command = session.set_position(target, validate=kin.validate_path)
                commands.append(list(command))
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
            for session in self.sessions:
                with contextlib.suppress(Exception):
                    session.hold()
            try:
                self.close()
            except BaseException as cleanup:
                raise EmbodimentFault(
                    f"Step failed: {primary}; shutdown failed: {cleanup}"
                ) from primary
            raise

    def close(self) -> None:
        """Park and close both arms independently, then close the camera."""
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
