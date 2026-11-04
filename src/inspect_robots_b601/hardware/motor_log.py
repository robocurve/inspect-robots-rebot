"""JSONL motor and temperature log for any B601 motor session; independent of Inspect.

One file per session, one JSON object per line. Every record carries wall-clock ``time_ns`` and
``monotonic_ns``. While the motors are driven, ``actuator-output`` records follow the native
follower's shape at about 5 Hz: measured and commanded positions plus the full per-joint
``thermal`` object (measured_c, estimated temp_c, torque_nm, allowed_torque_nm, cap_nm, latched,
feedback). The raw cached motor replies are kept alongside under ``motors``. Lifecycle events
(open, enable, path, stall, stop, disable, close) are separate records.

Samples read only each motor's cached MIT reply (no CAN traffic), and a background thread does the
disk writes, so the control loop never waits on the log.
"""

from __future__ import annotations

import contextlib
import json
import math
import os
import queue
import sys
import threading
import time
from collections.abc import Callable, Iterable, Mapping, Sequence
from pathlib import Path
from typing import Any

from . import Motor, PathLike
from .thermal import ThermalGuard

SAMPLE_PERIOD_NS = 200_000_000
_STATE_FIELDS = ("status_code", "pos", "vel", "torq", "t_mos", "t_rotor")


def _clean(value: Any) -> Any:
    """JSON-safe copy: non-finite floats become null, tuples become lists."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    return value


def _rounded(values: Iterable[float] | None, digits: int = 6) -> list[float] | None:
    return None if values is None else [round(float(value), digits) for value in values]


class MotorLog:
    """One session's JSONL motor and temperature log, written by a background thread."""

    def __init__(
        self,
        path: PathLike,
        joints: Iterable[Mapping[str, Any]],
        *,
        source: str,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        wall_ns: Callable[[], int] = time.time_ns,
        period_ns: int = SAMPLE_PERIOD_NS,
    ) -> None:
        self.path = Path(path)
        self.joints = [dict(joint) for joint in joints]
        self.names = [joint["name"] for joint in self.joints]
        self.source = source
        self._clock_ns, self._wall_ns, self._period_ns = clock_ns, wall_ns, period_ns
        self._next_sample_ns = 0
        self.thermal = ThermalGuard({joint["name"]: joint["model"] for joint in self.joints})
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("a", encoding="utf-8")
        self._queue: queue.SimpleQueue[dict[str, Any] | None] = queue.SimpleQueue()
        self._writer = threading.Thread(target=self._write, name="b601-motor-log", daemon=True)
        self._writer.start()
        self._closed = False
        self.event(
            "motor-log-start",
            pid=os.getpid(),
            argv=sys.argv,
            joints=[
                {
                    key: joint.get(key)
                    for key in ("name", "motor_id", "model", "kp", "kd", "torque_cap_nm")
                }
                for joint in self.joints
            ],
        )

    @classmethod
    def create(
        cls, directory: PathLike, joints: Iterable[Mapping[str, Any]], *, source: str
    ) -> MotorLog:
        """Open a new log in directory, named by UTC time, source and process id."""
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        return cls(Path(directory) / f"{stamp}-{source}-{os.getpid()}.jsonl", joints, source=source)

    def _record(self, status: str, fields: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "time_ns": self._wall_ns(),
            "monotonic_ns": self._clock_ns(),
            "kind": "follower",
            "source": self.source,
            "status": status,
            **fields,
        }

    def event(self, status: str, **fields: Any) -> None:
        """Queue a lifecycle record; ignored once the log is closed."""
        if not self._closed:
            self._queue.put(self._record(status, fields))

    def sample(
        self,
        motors: Sequence[Motor],
        *,
        measured: Sequence[float] | None,
        command: Sequence[float] | None,
        feedforward: Sequence[float] | None = None,
        caps: Sequence[float] | None = None,
        force: bool = False,
    ) -> None:
        """Rate-limited actuator-output record; call it every control tick."""
        now = self._clock_ns()
        if self._closed or (not force and now < self._next_sample_ns):
            return
        # Keep the 5 Hz cadence on average:
        # schedule from the previous slot, not from this late tick.
        slot = self._next_sample_ns if self._next_sample_ns else now
        self._next_sample_ns = max(slot + self._period_ns, now - self._period_ns // 2)
        states: dict[str, tuple[object, object]] = {}
        raw: dict[str, dict[str, Any] | None] = {}
        freshness: dict[str, dict[str, Any]] = {}
        for name, motor in zip(self.names, motors, strict=False):
            try:
                reader = getattr(motor, "get_state_snapshot", None)
                if callable(reader):
                    snapshot = reader()
                    age = None if snapshot is None else float(snapshot.feedback_age_s)
                    fresh = age is not None and math.isfinite(age) and 0 <= age <= 0.1
                    freshness[name] = {
                        "age_s": age,
                        "sequence": None if snapshot is None else snapshot.feedback_sequence,
                        "fresh": fresh,
                    }
                    state = None if snapshot is None else snapshot.state
                else:
                    freshness[name] = {"age_s": None, "sequence": None, "fresh": None}
                    state = motor.get_state()
            except Exception:
                freshness[name] = {"age_s": None, "sequence": None, "fresh": False}
                state = None
            if state is None:
                states[name], raw[name] = (None, None), None
                continue
            fields = {field: getattr(state, field, None) for field in _STATE_FIELDS}
            raw[name] = fields
            # Keep MOS/controller and rotor readings distinct in the raw record.
            readings = [
                value
                for value in (fields["t_mos"], fields["t_rotor"])
                if isinstance(value, (int, float)) and math.isfinite(value) and value != 0
            ]
            states[name] = (
                (None, None)
                if freshness[name]["fresh"] is False
                else (fields["torq"], max(readings, default=None))
            )
        self.thermal.update(states, now / 1e9)
        thermal = self.thermal.snapshot()
        for name, metadata in freshness.items():
            thermal[name]["feedback_fresh"] = metadata["fresh"]
            if metadata["fresh"] is False:
                thermal[name]["measured_c"] = None
        for name, cap in zip(self.names, caps or (), strict=False):
            thermal[name]["cap_nm"] = None if cap is None else round(float(cap), 3)
        self._queue.put(
            self._record(
                "actuator-output",
                {
                    "measured_rad": _rounded(measured),
                    "target_rad": _rounded(command),
                    "feedforward_nm": _rounded(feedforward, 3),
                    "thermal": thermal,
                    "motors": raw,
                    "feedback": freshness,
                    "thermal_latched": self.thermal.latched,
                },
            )
        )

    def _write(self) -> None:
        while True:
            record = self._queue.get()
            if record is None:
                break
            try:
                self._stream.write(
                    json.dumps(_clean(record), separators=(",", ":"), allow_nan=False) + "\n"
                )
                self._stream.flush()
            except Exception:
                pass  # A logging failure must never reach the motors.

    def close(self, **fields: Any) -> None:
        """Append a motor-log-end record with fields, then stop the writer and close the file."""
        if self._closed:
            return
        self.event("motor-log-end", **fields)
        self._closed = True
        self._queue.put(None)
        self._writer.join(timeout=2.0)
        with contextlib.suppress(Exception):
            self._stream.close()
