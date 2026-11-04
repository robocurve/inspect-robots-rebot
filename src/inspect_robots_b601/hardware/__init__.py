"""B601-RS motor sessions, CAN ownership and the durable stop latch; importing opens no hardware."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol

# A robot profile: the parsed JSON the README describes (channel, joints, gripper, ...).
Profile = dict[str, Any]
# A filesystem path given either way.
PathLike = str | Path


class Motor(Protocol):
    """The part of a Motorbridge RobStride motor handle this package uses."""

    def robstride_get_param_f32(self, parameter: int, timeout_ms: int = ...) -> float:
        """Read a float parameter from the motor, waiting up to timeout_ms for the reply."""

    def robstride_write_param_f32(self, parameter: int, value: float) -> None:
        """Write a float parameter to the motor."""

    def robstride_write_param_u32(self, parameter: int, value: int) -> None:
        """Write an unsigned 32-bit parameter to the motor."""

    def robstride_get_fault_report(self) -> tuple[int, int]:
        """Read the motor's (fault, warning) words."""

    def ensure_mode(self, mode: Any, timeout_ms: int = ...) -> None:
        """Put the motor in ``mode``, waiting up to timeout_ms."""

    def send_mit(self, position: float, velocity: float, kp: float, kd: float, tau: float) -> None:
        """Send one MIT command: position and velocity targets, PD gains and feed-forward torque."""

    def get_state(self) -> Any:
        """The motor's last cached MIT reply, or None if it has none; sends no CAN traffic."""

    def get_state_snapshot(self) -> Any:
        """Return cached feedback with its age and receive sequence, if supported."""

    def enable(self) -> None:
        """Enable the motor."""

    def disable(self) -> None:
        """Disable the motor."""


class MotorController(Protocol):
    """The part of a Motorbridge ``Controller`` this package uses."""

    def add_robstride_motor(self, motor_id: int, feedback_id: int, model: str) -> Motor:
        """Add a RobStride motor of ``model`` on this controller and return its handle."""

    def disable_all(self) -> None:
        """Disable every motor on this controller."""

    def close(self) -> None:
        """Release the controller."""
