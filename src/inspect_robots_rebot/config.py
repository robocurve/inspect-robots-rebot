"""Validated installation data for two independent reBot arms."""

from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from inspect_robots_b601.embodiment import DEFAULT_PROFILE
from inspect_robots_b601.hardware.bus import channel_name


def finite(value: Any, name: str) -> float:
    """Read a finite number; reject booleans and strings."""
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class ArmConfig:
    """One CAN bus, its calibration, and its persistent stop directory."""

    profile: dict[str, Any]
    state_dir: Path
    gripper_closed: float = 0.0
    gripper_open: float = 4.7

    def __post_init__(self) -> None:
        profile = copy.deepcopy(self.profile)
        channel_name(profile)
        axes = [*profile.get("joints", []), profile.get("gripper", {})]
        if [j.get("name") for j in axes] != [*(f"joint{i}" for i in range(1, 7)), "gripper"]:
            raise ValueError("Each arm requires ordered joint1..joint6 and gripper")
        ids = [j.get("motor_id") for j in axes]
        if any(type(i) is not int or not 1 <= i <= 127 for i in ids) or len(set(ids)) != 7:
            raise ValueError("Each arm requires seven distinct motor IDs in [1, 127]")
        for axis in axes:
            if axis.get("model") not in ("rs-00", "rs-06"):
                raise ValueError("Only rs-00 and rs-06 motors are supported")
            low, high = (finite(axis[k], k) for k in ("lower", "upper"))
            if low >= high:
                raise ValueError("Joint bounds must increase")
            for key in ("kp", "kd", "torque_cap_nm"):
                if finite(axis[key], key) <= 0:
                    raise ValueError(f"{key} must be positive")
            maximum = 14 if axis["model"] == "rs-00" else 36
            if axis["torque_cap_nm"] > maximum:
                raise ValueError("Torque cap exceeds the motor rating")
        closed = finite(self.gripper_closed, "gripper_closed")
        opened = finite(self.gripper_open, "gripper_open")
        if not axes[-1]["lower"] <= closed < opened <= axes[-1]["upper"]:
            raise ValueError("Gripper calibration must increase inside the motor bounds")
        object.__setattr__(self, "profile", profile)
        object.__setattr__(self, "state_dir", Path(self.state_dir).expanduser().resolve())

    @property
    def axes(self) -> list[dict[str, Any]]:
        """Return arm joints followed by the gripper."""
        return [*self.profile["joints"], self.profile["gripper"]]

    def grip_to_motor(self, value: float) -> float:
        """Map a normalized gripper target to calibrated motor radians."""
        return self.gripper_closed + value * (self.gripper_open - self.gripper_closed)

    def grip_from_motor(self, value: float) -> float:
        """Map motor feedback to a normalized gripper opening."""
        return max(
            0.0, min(1.0, (value - self.gripper_closed) / (self.gripper_open - self.gripper_closed))
        )


def load_arm(profile: str, channel: str, state_dir: Path) -> ArmConfig:
    """Load a profile without CAN access. Channel selection is explicit."""
    data = json.loads(Path(profile or DEFAULT_PROFILE).read_text())
    data["channel"] = channel
    calibration = data.get("gripper_calibration", {})
    return ArmConfig(
        data,
        state_dir,
        calibration.get("closed", 0.0),
        calibration.get("open", data["gripper"]["upper"]),
    )
