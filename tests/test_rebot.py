"""Tests for two-arm control without CAN, a camera, or an API service."""

import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from inspect_robots import Action, Scene
from inspect_robots.errors import EmbodimentFault

from inspect_robots_b601.embodiment import DEFAULT_PROFILE
from inspect_robots_b601.hardware.state import StopLedger
from inspect_robots_rebot import RebotEmbodiment
from inspect_robots_rebot.config import ArmConfig
from inspect_robots_rebot.kinematics import Kinematics, fk
from inspect_robots_rebot.session import Feedback, StreamingSession
from test_session import Dummy, FakeController, FakeMotor


@pytest.fixture
def config(tmp_path):
    return ArmConfig(json.loads(DEFAULT_PROFILE.read_text()), tmp_path / "state")


class SnapshotMotor(FakeMotor):
    def __init__(self, position=0):
        super().__init__(position)
        self.age = 0
        self.temperature = 35
        self.sequence = 0
        self.disable_fails = False

    def disable(self):
        if self.disable_fails:
            raise OSError("disable failed")
        super().disable()
        self.sequence += 1

    def get_state_snapshot(self):
        state = self.get_state()
        state.t_mos = self.temperature
        state.arbitration_id = (2 if self.enabled else 0) << 22
        return SimpleNamespace(
            state=state, feedback_age_s=self.age, feedback_sequence=self.sequence
        )


@pytest.fixture
def session(config):
    controller = FakeController()
    controller.motors = [SnapshotMotor() for _ in range(7)]
    StopLedger(config.state_dir).acknowledge()
    s = StreamingSession(
        config.profile,
        config.state_dir,
        controller_factory=lambda _: controller,
        bus_factory=lambda _: Dummy(),
        home_wait=0,
    )
    s.open()
    yield s
    s.close()


def test_inert_constructor(tmp_path):
    e = RebotEmbodiment(state_dir=str(tmp_path / "new"))
    assert not (tmp_path / "new").exists()
    assert e.info.action_space.shape == (14,)
    assert e.info.action_space.semantics.dim_labels[7] == "right_x"
    with pytest.raises(EmbodimentFault, match="allow_motion"):
        e.reset(Scene(id="test", instruction="test"))
    e.close()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"left_channel": "can0"},
        {"motor_temp_limit": float("nan")},
        {"motor_temp_limit": 81},
        {"control_interface": "velocity"},
    ],
)
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        RebotEmbodiment(**kwargs)


def test_calibration(config):
    assert config.grip_to_motor(0) == 0  # Mechanical lower bound is -0.1, not closed.
    assert config.grip_to_motor(1) == 4.7
    assert config.grip_from_motor(-0.05) == 0
    assert config.grip_from_motor(2.35) == 0.5
    with pytest.raises(ValueError):
        ArmConfig(config.profile, config.state_dir, 5, 6)


@pytest.mark.parametrize(
    "field,value",
    [
        ("motor_id", 2),
        ("kp", 0),
        ("torque_cap_nm", 40),
        ("model", "other"),
        ("lower", float("nan")),
    ],
)
def test_bad_profile(config, field, value):
    config.profile["joints"][0][field] = value
    with pytest.raises(ValueError):
        ArmConfig(config.profile, config.state_dir)


def test_ik_roundtrip_and_bound(config):
    k = Kinematics(config)
    q = np.array([-0.1, 0.15, 0.2, 0.05, -0.2, 0.1])
    k.capture_reference(q)
    target = k.observe(q, 0.2)
    np.testing.assert_allclose(target[3:6], 0, atol=1e-12)
    result = k.solve(target, q)
    assert result.converged
    np.testing.assert_allclose(fk(result.joints), fk(q), atol=1e-5)
    target[:3] = [10, 10, 10]
    result = k.solve(target, q)
    assert not result.converged
    assert np.all(result.joints >= k.lower)
    assert np.all(result.joints <= k.upper)
    assert np.isfinite(result.position_error_m)


def test_ik_at_joint_bound_is_not_rejected(config):
    k = Kinematics(config)
    q = np.array([-0.08, 0.008, 0.004, -0.01, -0.34, 0.068])
    k.capture_reference(q)
    target = k.observe(q, 0)
    target[:3] = [0.33, -0.1, 0.24]
    result = k.solve(target, q)
    assert np.isfinite(result.joints).all()
    initial = np.linalg.norm(target[:3] - fk(q)[:3, 3])
    assert result.position_error_m < initial


def test_ik_bad_input(config):
    k = Kinematics(config)
    with pytest.raises(RuntimeError):
        k.observe([0] * 6, 0)
    with pytest.raises(RuntimeError):
        k.solve([0] * 7, [0] * 6)
    k.capture_reference([0] * 6)
    with pytest.raises(ValueError):
        k.solve([float("nan")] * 7, [0] * 6)
    with pytest.raises(ValueError):
        fk([0] * 5)


def test_stream_margin_and_nonblocking_feedback(session):
    session.activate()
    command = session.set_position([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1])
    assert command[0] == pytest.approx(0.08)
    assert command[6] == pytest.approx(0.44)
    # Freeze the loop so encoder jitter is tested against the exact command.
    session._stop.set()
    session._thread.join(1)
    session._motors[6].position = -0.001
    session._check_envelope()  # 0.441 < 0.45: does not latch on the command boundary.
    session._motors[6].position = -0.02
    session._check_envelope()  # Command lead is not the loss-of-control envelope.
    assert not session.pair_stop.is_set()
    session._streaming = False
    session._enabled = False


@pytest.mark.parametrize(
    "field,value",
    [("age", 0.2), ("age", float("nan")), ("temperature", 80), ("temperature", float("nan"))],
)
def test_feedback_faults(session, field, value):
    setattr(session._motors[0], field, value)
    with pytest.raises(RuntimeError):
        session.feedback()


def test_bad_command_does_not_mutate(session):
    with pytest.raises(RuntimeError):
        session.set_position([0] * 7)
    session.activate()
    before = session._command.copy()
    for target in ([0] * 6, [float("nan")] * 7, [100] * 7):
        with pytest.raises(ValueError):
            session.set_position(target)
        assert session._command == before
    session.hold()


def test_disable_confirmation(session):
    session.activate()
    session.close()
    assert session.shutdown_report["disable_confirmed"]
    assert len(session.shutdown_report["replies"]) == 7
    assert not any(m.enabled for m in session._motors)
    session.close()


def test_disable_failure_closes_resources(config):
    controller = FakeController()
    controller.motors = [SnapshotMotor() for _ in range(7)]
    controller.close = Mock()
    bus = Dummy()
    StopLedger(config.state_dir).acknowledge()
    s = StreamingSession(
        config.profile,
        config.state_dir,
        controller_factory=lambda _: controller,
        bus_factory=lambda _: bus,
        home_wait=0,
    )
    s.open()
    s.activate()
    controller.motors[0].disable_fails = True
    with pytest.raises(RuntimeError, match="disable failed"):
        s.close()
    assert bus.closed
    controller.close.assert_called_once()
    assert not s.shutdown_report["disable_confirmed"]


def test_old_driver_rejected_before_enable(config):
    controller = FakeController()
    StopLedger(config.state_dir).acknowledge()
    s = StreamingSession(
        config.profile,
        config.state_dir,
        controller_factory=lambda _: controller,
        bus_factory=lambda _: Dummy(),
    )
    with pytest.raises(RuntimeError, match="get_state_snapshot"):
        s.open()
    assert not any(m.enabled for m in controller.motors)


class Camera:
    def __init__(self, url):
        self.at = time.monotonic()
        self.closed = False

    def get(self):
        self.at = time.monotonic()
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class Arm:
    def __init__(self, profile, state_dir, **kwargs):
        self.q = np.array([-0.1, 0.15, 0.2, 0.05, -0.2, 0.1, 0])
        self.closed = False
        self.shutdown_report = {"disable_confirmed": True, "errors": []}
        self.active = False
        self.pair_stop = kwargs["pair_stop"]

    def open(self):
        pass

    def activate(self):
        self.active = True

    def observe(self):
        return self.q.copy()

    def feedback(self):
        return Feedback(tuple(self.q), (0.0,) * 7, (35.0,) * 7, time.monotonic())

    def set_position(self, q, *, validate=None):
        if validate:
            validate(self.q, q)
        self.q = np.asarray(q)
        return tuple(q)

    def hold(self):
        pass

    def close(self):
        self.active = False
        self.closed = True


def test_two_arm_lifecycle(tmp_path):
    e = RebotEmbodiment(
        allow_motion=True, state_dir=str(tmp_path), session_factory=Arm, camera_factory=Camera
    )
    observation = e.reset(Scene(id="test", instruction="move both arms"))
    arms = list(e.sessions)
    assert arms[0].pair_stop is arms[1].pair_stop
    target = observation.state["eef_state"].copy()
    target[[2, 9]] += 0.005
    result = e.step(Action(target))
    assert len(result.info["arms"]) == 2
    assert result.observation.state["joint_pos"].shape == (2, 7)
    assert result.info["step_duration_s"] >= 0.1
    e.close()
    assert all(a.closed for a in arms)
    assert len(e.shutdown_reports) == 2
    e.close()


@pytest.mark.parametrize("phase", ["open", "activate", "set_position"])
def test_pair_failure_closes_both(tmp_path, phase):
    arms = []

    def factory(*args, **kwargs):
        arm = Arm(*args, **kwargs)
        if len(arms) == 1:
            setattr(arm, phase, Mock(side_effect=RuntimeError("right failed")))
        arms.append(arm)
        return arm

    e = RebotEmbodiment(
        allow_motion=True, state_dir=str(tmp_path), session_factory=factory, camera_factory=Camera
    )
    with pytest.raises(RuntimeError, match="right failed"):
        observation = e.reset(Scene(id="test", instruction="test"))
        e.step(Action(observation.state["eef_state"]))
    assert len(arms) == 2
    assert all(a.closed for a in arms)


def test_fault_propagates_to_other_arm(session):
    session.pair_stop.set()
    with pytest.raises(RuntimeError, match="other arm"):
        session._check_envelope()


def test_camera_failure_opens_no_bus(tmp_path):
    camera = Camera("")
    camera.get = Mock(side_effect=RuntimeError("camera failed"))
    factory = Mock()
    e = RebotEmbodiment(
        allow_motion=True,
        state_dir=str(tmp_path),
        session_factory=factory,
        camera_factory=lambda _: camera,
    )
    with pytest.raises(RuntimeError, match="camera failed"):
        e.reset(Scene(id="test", instruction="test"))
    factory.assert_not_called()
    assert camera.closed


def test_camera_stream_and_shutdown(monkeypatch):
    import sys

    from inspect_robots_rebot.camera import LiveCamera

    capture = SimpleNamespace(released=False)
    capture.isOpened = lambda: True

    def frame():
        time.sleep(0.005)
        return True, np.zeros((480, 640, 3), dtype=np.uint8)

    capture.read = frame
    capture.release = lambda: setattr(capture, "released", True)
    cv = SimpleNamespace(
        VideoCapture=lambda *args: capture,
        CAP_FFMPEG=0,
        CAP_PROP_OPEN_TIMEOUT_MSEC=1,
        CAP_PROP_READ_TIMEOUT_MSEC=2,
        COLOR_BGR2RGB=3,
        cvtColor=lambda image, _: image,
    )
    monkeypatch.setitem(sys.modules, "cv2", cv)
    camera = LiveCamera("http://camera.invalid/stream")
    assert camera.get().shape == (480, 640, 3)
    assert camera.at is not None
    camera.close()
    assert capture.released
    with pytest.raises(RuntimeError, match="closed"):
        camera.get()


@pytest.mark.parametrize("problem", ["open", "read", "shape"])
def test_camera_errors(monkeypatch, problem):
    import sys

    from inspect_robots_rebot.camera import LiveCamera

    capture = SimpleNamespace(
        isOpened=lambda: problem != "open",
        read=lambda: (problem != "read", np.zeros((1, 1, 3), dtype=np.uint8)),
        release=Mock(),
    )
    cv = SimpleNamespace(
        VideoCapture=lambda *args: capture,
        CAP_FFMPEG=0,
        CAP_PROP_OPEN_TIMEOUT_MSEC=1,
        CAP_PROP_READ_TIMEOUT_MSEC=2,
    )
    monkeypatch.setitem(sys.modules, "cv2", cv)
    camera = LiveCamera("test")
    with pytest.raises(RuntimeError):
        camera.get()
    camera.close()
    capture.release.assert_called_once()


def test_camera_stale_frame():
    import threading

    from inspect_robots_rebot.camera import LiveCamera

    camera = object.__new__(LiveCamera)
    camera.condition = threading.Condition()
    camera.stop = threading.Event()
    camera.frame = np.zeros((480, 640, 3), dtype=np.uint8)
    camera.error = None
    camera.at = time.monotonic() - 2
    with pytest.raises(RuntimeError, match="stale"):
        camera.get()


def test_feedback_requires_complete_healthy_state(session):
    motor = session._motors[0]
    original = motor.get_state_snapshot
    motor.get_state_snapshot = lambda: None
    with pytest.raises(RuntimeError, match="missing"):
        session.feedback()
    motor.get_state_snapshot = original
    motor.position = -4
    with pytest.raises(RuntimeError, match="bounds"):
        session.feedback()
    motor.position = 0
    original_state = motor.get_state

    def fault():
        state = original_state()
        state.status_code = 1
        return state

    motor.get_state = fault
    with pytest.raises(RuntimeError, match="status"):
        session.feedback()


def test_torque_persistence(session):
    clock = [10.0]
    session._clock = lambda: clock[0]
    original = session._motors[0].get_state

    def state():
        result = original()
        result.torq = 40
        return result

    session._motors[0].get_state = state
    session.feedback()
    clock[0] = 11.01
    with pytest.raises(RuntimeError, match="torque"):
        session.feedback()


def test_pair_invalid_action_closes(tmp_path):
    e = RebotEmbodiment(
        allow_motion=True, state_dir=str(tmp_path), session_factory=Arm, camera_factory=Camera
    )
    e.reset(Scene(id="test", instruction="test"))
    arms = list(e.sessions)
    with pytest.raises(EmbodimentFault):
        e.step(Action(np.zeros(13)))
    assert all(a.closed for a in arms)


def test_pair_cleanup_error_preserves_primary(tmp_path):
    arms = []

    def factory(*args, **kwargs):
        arm = Arm(*args, **kwargs)
        if arms:
            arm.activate = Mock(side_effect=RuntimeError("activation failed"))
        arm.close = Mock(side_effect=RuntimeError("cleanup failed"))
        arms.append(arm)
        return arm

    e = RebotEmbodiment(
        allow_motion=True, state_dir=str(tmp_path), session_factory=factory, camera_factory=Camera
    )
    with pytest.raises(EmbodimentFault, match=r"activation failed.*cleanup failed"):
        e.reset(Scene(id="test", instruction="test"))
    assert all(a.close.call_count == 1 for a in arms)


def test_same_state_directory_rejected(tmp_path):
    with pytest.raises(ValueError, match="own persistent"):
        RebotEmbodiment(left_state_dir=str(tmp_path), right_state_dir=str(tmp_path))


def test_recorded_joint2_following_error_is_not_a_native_fault(session):
    session._command = [0.0] * 7
    session._motors[1].position = 0.0912748575
    session._check_envelope()
    assert not session.pair_stop.is_set()


def test_native_envelope_uses_fresh_snapshot_and_still_stops(session):
    # The parameter-poll cache can be older than the streamed reply.
    session._positions = (0.0,) * 7
    session._command = [0.0] * 7
    session._motors[1].position = 0.601
    with pytest.raises(RuntimeError, match="command envelope exceeded"):
        session._check_envelope()
    assert session.pair_stop.is_set()


def test_pair_stop_does_not_interrupt_native_parking(session):
    session.pair_stop.set()
    session._parking = True
    session._check_envelope()


@pytest.mark.parametrize("floor", [0.12, -0.02, 0.03])
def test_tool_floor_controls_action_bounds_and_solver(tmp_path, floor):
    e = RebotEmbodiment(
        allow_motion=True,
        state_dir=str(tmp_path),
        session_factory=Arm,
        camera_factory=Camera,
        tool_floor_m=floor,
    )
    assert e.info.action_space.low[2] == floor
    assert e.info.action_space.low[9] == floor
    e.reset(Scene(id="floor", instruction="test"))
    assert all(k.tool_floor_m == floor for k in e.kinematics)
    e.close()


@pytest.mark.parametrize("floor", [float("nan"), float("inf"), 0.6, 0.7])
def test_invalid_tool_floor(config, floor):
    with pytest.raises(ValueError, match="tool_floor_m"):
        RebotEmbodiment(tool_floor_m=floor)
    with pytest.raises(ValueError, match="tool_floor_m"):
        Kinematics(config, tool_floor_m=floor)


def test_path_uses_configured_floor(config, monkeypatch):
    import inspect_robots_rebot.kinematics as module

    height = [0.0]

    def pose(q):
        result = np.eye(4)
        result[2, 3] = height[0]
        return result

    monkeypatch.setattr(module, "fk", pose)
    relaxed = Kinematics(config, tool_floor_m=-0.02)
    relaxed.validate_path([0] * 6, [0.01] * 6)
    with pytest.raises(ValueError, match=r"0\.12 m"):
        Kinematics(config).validate_path([0] * 6, [0.01] * 6)
    height[0] = -0.021
    with pytest.raises(ValueError, match=r"-0\.02 m"):
        relaxed.validate_path([0] * 6, [0.01] * 6)


def test_previous_command_steps_and_whole_arm_resync(session):
    session.activate()
    session._stop.set()
    session._thread.join(1)
    session.command_mode = "previous"
    target = [0.8] * 6 + [1.0]
    assert session.set_position(target)[:6] == pytest.approx([0.2] * 6)
    assert session.set_position(target)[:6] == pytest.approx([0.4] * 6)
    assert not session.command_resynced
    # Only one joint needs to fall behind to reset all six references.
    for motor in session._motors[:6]:
        motor.position = 0.2
    session._motors[0].position = 0.0
    command = session.set_position(target)
    assert command[:6] == pytest.approx([0.2] + [0.4] * 5)
    assert session.command_resynced
    assert command[6] == pytest.approx(0.44)
    session._enabled = False


def test_failed_path_does_not_advance_command_reference(session):
    session.activate()
    session._stop.set()
    session._thread.join(1)
    session.command_mode = "previous"
    first = session.set_position([0.8] * 6 + [0])

    def reject(start, target):
        raise ValueError("blocked path")

    with pytest.raises(ValueError, match="blocked path"):
        session.set_position([0.8] * 6 + [0], validate=reject)
    assert tuple(session._command) == first
    session._enabled = False


@pytest.mark.parametrize(
    "kwargs",
    [
        {"command_mode": "unknown"},
        {"command_step_rad": float("nan")},
        {"command_step_rad": 0},
        {"command_resync_rad": -0.1},
        {"command_step_rad": 0.3, "command_resync_rad": 0.35},
    ],
)
def test_invalid_command_configuration(tmp_path, kwargs):
    with pytest.raises(ValueError):
        RebotEmbodiment(state_dir=str(tmp_path), **kwargs)


@pytest.mark.parametrize("margin", [-0.01, float("nan"), float("inf"), 2.0])
def test_invalid_wrist_command_margin(config, tmp_path, margin):
    with pytest.raises(ValueError, match="margin"):
        Kinematics(config, wrist_yaw_command_margin_rad=margin)
    with pytest.raises(ValueError, match="margin"):
        RebotEmbodiment(state_dir=str(tmp_path), wrist_yaw_command_margin_rad=margin)


def test_wrist_margin_bounds_ik_without_changing_motor_profile(config):
    kin = Kinematics(config, wrist_yaw_command_margin_rad=0.03)
    plain = Kinematics(config)
    q = np.array([0.0, 0.8, 0.7, 0.3, config.axes[4]["lower"], 0.0])
    kin.capture_reference(q)
    target = kin.observe(q, 0)
    result = kin.solve(target, q)
    assert result.joints[4] >= config.axes[4]["lower"] + 0.03
    assert kin.upper[4] == pytest.approx(config.axes[4]["upper"] - 0.03)
    assert plain.lower[4] == config.axes[4]["lower"]
    assert kin.lower[:4] == pytest.approx(plain.lower[:4])
    assert kin.upper[5] == plain.upper[5]


def test_reduced_joint2_upper_tolerance_only(session):
    session.joints[1]["upper"] = 1.6
    session.joint2_upper_tolerance_rad = 0.012
    session._motors[1].position = 1.6103458404541016
    session.feedback()
    session._motors[1].position = 1.6121
    with pytest.raises(Exception, match="joint2: measured position outside bounds"):
        session.feedback()
    session._motors[1].position = -0.0101
    with pytest.raises(Exception, match="joint2: measured position outside bounds"):
        session.feedback()
    session._motors[1].position = 0
    session._motors[4].position = session.joints[4]["lower"] - 0.0101
    with pytest.raises(Exception, match="joint5: measured position outside bounds"):
        session.feedback()


@pytest.mark.parametrize("tolerance", [0.009, 0.013, float("nan"), float("inf"), 0.012])
def test_tolerance_cannot_expand_stock_joint_range(config, tmp_path, tolerance):
    with pytest.raises(ValueError):
        StreamingSession(config.profile, config.state_dir, joint2_upper_tolerance_rad=tolerance)
    with pytest.raises(ValueError):
        RebotEmbodiment(state_dir=str(tmp_path), joint2_upper_tolerance_rad=tolerance)


def test_reduced_joint2_range_accepts_narrow_tolerance(config, tmp_path):
    profile = json.loads(json.dumps(config.profile))
    profile["joints"][1]["upper"] = 1.6
    path = tmp_path / "reduced.json"
    path.write_text(json.dumps(profile))
    e = RebotEmbodiment(
        left_profile=str(path),
        right_profile=str(path),
        state_dir=str(tmp_path / "pair"),
        joint2_upper_tolerance_rad=0.012,
    )
    assert e.joint2_upper_tolerance_rad == 0.012
    s = StreamingSession(profile, config.state_dir, joint2_upper_tolerance_rad=0.012)
    assert s.joint2_upper_tolerance_rad == 0.012
    assert s.joints[1]["upper"] == 1.6


@pytest.mark.parametrize("limit", [0, -1, 101, float("nan"), float("inf")])
def test_invalid_rs00_mos_limit(config, tmp_path, limit):
    with pytest.raises(ValueError, match="rs00_mos_temp_limit"):
        StreamingSession(config.profile, config.state_dir, rs00_mos_temp_limit=limit)
    with pytest.raises(ValueError, match="rs00_mos_temp_limit"):
        RebotEmbodiment(state_dir=str(tmp_path), rs00_mos_temp_limit=limit)


def test_rs00_mos_limit_does_not_raise_rs06_or_rotor_limit(session):
    session.rs00_mos_temp_limit = 100
    session._motors[3].temperature = 99
    session.feedback()
    session._motors[3].temperature = 100
    with pytest.raises(Exception, match="joint4: temperature stop"):
        session.feedback()
    session._motors[3].temperature = 99
    session._motors[0].temperature = 80
    with pytest.raises(Exception, match="joint1: temperature stop"):
        session.feedback()
    session._motors[0].temperature = 30
    motor = session._motors[3]
    original = motor.get_state_snapshot

    def hot_rotor():
        snapshot = original()
        snapshot.state.t_rotor = 80
        return snapshot

    motor.get_state_snapshot = hot_rotor
    with pytest.raises(Exception, match=r"rotor=80\.0/80\.0"):
        session.feedback()


def test_rs00_default_follows_lower_general_temperature_limit(config, tmp_path):
    s = StreamingSession(config.profile, config.state_dir, motor_temp_limit=65)
    assert s.rs00_mos_temp_limit == 65
    e = RebotEmbodiment(state_dir=str(tmp_path), motor_temp_limit=65)
    assert e.rs00_mos_temp_limit == 65


@pytest.mark.parametrize("direction", [-1, 1])
def test_saturated_stall_holds_and_requires_reverse_request(session, direction):
    session.activate()
    session._stop.set()
    session._thread.join(1)
    now = [0.0]
    session._clock = lambda: now[0]
    session._streaming = True
    session._command[3] = 0.4 * direction
    sample = Feedback((0.0,) * 7, (0, 0, 0, session.caps[3], 0, 0, 0), (35.0,) * 7, 0)
    session._relieve_stalls(sample)
    now[0] = 0.49
    session._relieve_stalls(sample)
    assert session._command[3] == 0.4 * direction
    now[0] = 0.51
    session._relieve_stalls(sample)
    assert session._command[3] == 0
    assert session.blocked[3] == direction
    command = session.set_position([0, 0, 0, 0.4 * direction, 0, 0, 0])
    assert command[3] == 0
    command = session.set_position([0, 0, 0, -0.02 * direction, 0, 0, 0])
    assert command[3] * direction < 0
    assert session.blocked[3] == 0
    session._enabled = False


def test_moving_high_effort_joint_is_not_blocked(session):
    now = [0.0]
    session._clock = lambda: now[0]
    session._streaming = True
    session._command[3] = 0.5
    for i in range(10):
        now[0] += 0.2
        q = [0.0] * 7
        q[3] = i * 0.006
        session._relieve_stalls(
            Feedback(tuple(q), (0, 0, 0, session.caps[3], 0, 0, 0), (35.0,) * 7, now[0])
        )
    assert not any(session.blocked)
    assert session._command[3] == 0.5


@pytest.mark.parametrize("side", [0, 1])
def test_floor_rejection_keeps_pair_open_and_allows_next_move(tmp_path, side):
    from inspect_robots_rebot.kinematics import PathRejected

    e = RebotEmbodiment(
        allow_motion=True, state_dir=str(tmp_path), session_factory=Arm, camera_factory=Camera
    )
    observation = e.reset(Scene(id="reject", instruction="move"))
    arms = list(e.sessions)
    for arm in arms:
        arm.hold = Mock()
    original = e.kinematics[side].validate_path
    e.kinematics[side].validate_path = Mock(side_effect=PathRejected("tool floor"))
    result = e.step(Action(observation.state["eef_state"].copy()))
    assert result.info["command_rejected"]
    assert result.observation.state["command_rejected"].tolist() == [1]
    assert all(a.active and not a.closed for a in arms)
    assert all(a.hold.call_count == 1 for a in arms)
    e.kinematics[side].validate_path = original
    result = e.step(Action(observation.state["eef_state"].copy()))
    assert result.observation.state["command_rejected"].tolist() == [0]
    e.close()


def test_stream_feedback_loss_disables_without_home_retry(session):
    session.activate()
    session.go_home = Mock(side_effect=AssertionError("must not park without feedback"))
    session._motors[0].age = 0.481
    deadline = time.monotonic() + 2
    while session._enabled and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not session._enabled
    assert session._lost_control
    assert session.pair_stop.is_set()
    assert "stale feedback" in str(session._failure)
    with pytest.raises(RuntimeError, match="disable not confirmed"):
        session.close()
    session.go_home.assert_not_called()
    assert not session.shutdown_report["disable_confirmed"]
    assert session._thread is not None and not session._thread.is_alive()


def test_short_feedback_age_does_not_stop_stream(session):
    session.activate()
    session._motors[0].age = 0.08
    time.sleep(0.12)
    assert session._enabled and session._failure is None
    session._motors[0].age = 0
