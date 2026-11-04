"""Hardware-free tests for the JSONL motor/temperature log."""

import itertools
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from inspect_robots_b601.hardware.motor_log import MotorLog

JOINTS = [
    {
        "name": "joint2",
        "motor_id": 2,
        "model": "rs-06",
        "kp": 150.0,
        "kd": 10.0,
        "torque_cap_nm": 36.0,
    },
    {
        "name": "gripper",
        "motor_id": 7,
        "model": "rs-00",
        "kp": 50.0,
        "kd": 4.0,
        "torque_cap_nm": 1.0,
    },
]


class StateMotor:
    def __init__(self, state):
        self.state = state

    def get_state(self):
        if isinstance(self.state, Exception):
            raise self.state
        return self.state


def reply(torq=1.5, t_mos=40.0, pos=0.1):
    return SimpleNamespace(status_code=0, pos=pos, vel=0.0, torq=torq, t_mos=t_mos, t_rotor=0.0)


class MotorLogTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "motor" / "run.jsonl"
        self.now = 0
        self.log = MotorLog(
            self.path,
            JOINTS,
            source="test",
            clock_ns=lambda: self.now,
            wall_ns=lambda: 1_700_000_000_000_000_000 + self.now,
        )

    def records(self):
        self.log.close()
        return [json.loads(line) for line in self.path.read_text().splitlines()]

    def test_samples_are_rate_limited_to_about_5_hz_with_both_timestamps(self):
        motors = [StateMotor(reply()), StateMotor(reply(torq=0.2, t_mos=31.0))]
        for now_ms in (0, 100, 199, 200, 350, 400):
            self.now = now_ms * 1_000_000
            self.log.sample(motors, measured=[0.1, 0.5], command=[0.11, 0.5], caps=[36.0, 1.0])
        outputs = [r for r in self.records() if r["status"] == "actuator-output"]
        self.assertEqual([r["monotonic_ns"] // 1_000_000 for r in outputs], [0, 200, 400])
        for record in outputs:
            self.assertEqual(record["kind"], "follower")
            self.assertGreater(record["time_ns"], record["monotonic_ns"])

    def test_late_control_ticks_keep_5_hz_on_average(self):
        motors = [StateMotor(reply()), StateMotor(reply())]
        start = 123_456_000_000_000  # a realistic monotonic clock
        for tick in range(150):  # 3 s of 20 ms ticks that each run 2 ms late
            self.now = start + tick * 22_000_000
            self.log.sample(motors, measured=[0.0, 0.0], command=[0.0, 0.0])
        outputs = [r for r in self.records() if r["status"] == "actuator-output"]
        gaps = [b["monotonic_ns"] - a["monotonic_ns"] for a, b in itertools.pairwise(outputs)]
        self.assertGreaterEqual(min(gaps), 150_000_000)
        span = (outputs[-1]["monotonic_ns"] - outputs[0]["monotonic_ns"]) / 1e9
        self.assertAlmostEqual((len(outputs) - 1) / span, 5.0, delta=0.3)

    def test_actuator_output_carries_the_full_thermal_object(self):
        motors = [StateMotor(reply()), StateMotor(None)]
        self.log.sample(
            motors, measured=[0.1, 0.5], command=[0.11, 0.5], feedforward=[2.5], caps=[36.0, 1.0]
        )
        record = next(r for r in self.records() if r["status"] == "actuator-output")
        joint = record["thermal"]["joint2"]
        for key in (
            "temp_c",
            "housing_c",
            "measured_c",
            "torque_nm",
            "allowed_torque_nm",
            "predicted_c",
            "latched",
            "feedback",
            "models",
            "cap_nm",
        ):
            self.assertIn(key, joint)
        self.assertEqual(joint["measured_c"], 40.0)
        self.assertEqual(joint["feedback"], "ok")
        self.assertEqual(joint["cap_nm"], 36.0)
        self.assertEqual(record["thermal"]["gripper"]["feedback"], "missing")
        self.assertEqual(record["motors"]["joint2"]["t_mos"], 40.0)
        self.assertIsNone(record["motors"]["gripper"])
        self.assertEqual(record["target_rad"], [0.11, 0.5])

    def test_unreadable_or_non_finite_replies_never_break_the_log(self):
        motors = [
            StateMotor(OSError("no reply")),
            StateMotor(reply(torq=float("nan"), t_mos=float("inf"))),
        ]
        self.log.sample(motors, measured=[0.0, 0.0], command=[0.0, 0.0])
        record = next(r for r in self.records() if r["status"] == "actuator-output")
        self.assertEqual(record["thermal"]["joint2"]["feedback"], "missing")
        self.assertIsNone(record["motors"]["gripper"]["torq"])
        self.assertIsNone(record["thermal"]["gripper"]["measured_c"])

    def test_events_bracket_the_session(self):
        self.log.event("motors-enabled", measured_rad=[0.0, 0.0])
        statuses = [r["status"] for r in self.records()]
        self.assertEqual(statuses, ["motor-log-start", "motors-enabled", "motor-log-end"])


if __name__ == "__main__":
    unittest.main()


def test_stale_snapshot_keeps_raw_evidence_but_not_live_temperature(tmp_path):
    class Snapshot:
        def get_state_snapshot(self):
            return SimpleNamespace(state=reply(), feedback_age_s=0.481, feedback_sequence=17)

    path = tmp_path / "stale.jsonl"
    log = MotorLog(path, JOINTS[:1], source="test")
    log.sample([Snapshot()], measured=[0.1], command=[0.1])
    log.close()
    output = next(
        json.loads(line)
        for line in path.read_text().splitlines()
        if json.loads(line)["status"] == "actuator-output"
    )
    assert output["feedback"]["joint2"] == {"age_s": 0.481, "sequence": 17, "fresh": False}
    assert output["motors"]["joint2"]["t_mos"] == 40
    assert output["thermal"]["joint2"]["measured_c"] is None
    assert output["thermal"]["joint2"]["feedback_fresh"] is False
