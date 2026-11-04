"""Bounded Cartesian IK using the existing, source-traced reBot URDF data."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

from inspect_robots_b601.hardware import gravity

from .config import ArmConfig

Vec = npt.NDArray[np.float64]


def fk(joints: npt.ArrayLike) -> Vec:
    """Return the gripper-end transform in the arm base frame."""
    q = np.asarray(joints, dtype=np.float64)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("FK requires six finite motor angles")
    rotation, position = np.eye(3), np.zeros(3)
    for (xyz, origin, axis, _), angle in zip(gravity._CHAIN, q, strict=True):
        position += rotation @ np.asarray(xyz)
        rotation = (
            rotation @ np.asarray(origin) @ np.asarray(gravity._axis_angle(axis, float(angle)))
        )
    position += rotation @ np.asarray(gravity._J_GRIPPER_END[0])
    result = np.eye(4)
    result[:3, :3] = rotation @ np.asarray(gravity._rpy(*gravity._J_GRIPPER_END[1]))
    result[:3, 3] = position
    return result


@dataclass(frozen=True)
class Solution:
    """A bounded candidate and its residual, before the transport tracking clamp."""

    joints: Vec
    position_error_m: float
    rotation_error_rad: float
    converged: bool


class Kinematics:
    """Use YAM's base-frame and trial-relative yaw, pitch, and roll convention."""

    def __init__(
        self,
        config: ArmConfig,
        *,
        tool_floor_m: float = 0.12,
        wrist_yaw_command_margin_rad: float = 0.0,
    ) -> None:
        if not np.isfinite(tool_floor_m) or tool_floor_m >= 0.6:
            raise ValueError("tool_floor_m must be finite and below the 0.6 m upper bound")
        self.tool_floor_m = float(tool_floor_m)
        # Optional dependency: only Cartesian reset loads the solver.
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        self._least_squares = least_squares
        self._rotation = Rotation
        self.lower = np.array([j["lower"] for j in config.axes[:6]], dtype=np.float64)
        self.upper = np.array([j["upper"] for j in config.axes[:6]], dtype=np.float64)
        margin = wrist_yaw_command_margin_rad
        if not np.isfinite(margin) or margin < 0 or 2 * margin >= self.upper[4] - self.lower[4]:
            raise ValueError("Wrist yaw command margin must leave a nonempty joint range")
        self.lower[4] += margin
        self.upper[4] -= margin
        self.reference: Vec | None = None

    def capture_reference(self, q: npt.ArrayLike) -> None:
        """Set zero orientation from measured joints at reset."""
        self.reference = fk(q)[:3, :3]

    def observe(self, q: npt.ArrayLike, gripper: float) -> Vec:
        """Report measured Cartesian position and relative orientation."""
        if self.reference is None:
            raise RuntimeError("Reset must set the orientation reference")
        pose = fk(q)
        yaw, pitch, roll = self._rotation.from_matrix(pose[:3, :3] @ self.reference.T).as_euler(
            "ZYX"
        )
        return np.array([*pose[:3, 3], yaw, -pitch, roll, gripper], dtype=np.float64)

    def solve(self, target: npt.ArrayLike, measured: npt.ArrayLike) -> Solution:
        """Return the best bounded solution, including an inexact reachable approach."""
        target = np.asarray(target, dtype=np.float64)
        seed = np.asarray(measured, dtype=np.float64)
        if target.shape != (7,) or seed.shape != (6,):
            raise ValueError("IK requires a seven-value pose and six measured joints")
        if not np.isfinite(target).all() or not np.isfinite(seed).all():
            raise ValueError("IK inputs must be finite")
        if self.reference is None:
            raise RuntimeError("Reset must set the orientation reference")
        desired = (
            self._rotation.from_euler("ZYX", target[3:6] * [1, -1, 1]).as_matrix() @ self.reference
        )

        def residual(q: Vec) -> Vec:
            pose = fk(q)
            rotation = self._rotation.from_matrix(desired @ pose[:3, :3].T).as_rotvec()
            return np.concatenate((target[:3] - pose[:3, 3], 0.2 * rotation))

        seed = np.clip(seed, self.lower, self.upper)
        result = self._least_squares(
            residual,
            seed,
            bounds=(self.lower, self.upper),
            max_nfev=60,
            ftol=1e-6,
            xtol=1e-6,
            gtol=1e-6,
        )
        candidate = np.asarray(result.x, dtype=np.float64)
        if not np.isfinite(candidate).all():
            raise ValueError("IK returned a non-finite result")
        candidate = np.clip(candidate, self.lower, self.upper)
        # A failed search must not command a worse pose than its starting pose.
        if np.linalg.norm(residual(candidate)) > np.linalg.norm(residual(seed)):
            candidate = seed
        error = residual(candidate)
        pos, rot = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]) / 0.2)
        return Solution(candidate, pos, rot, pos <= 0.001 and rot <= 0.01)

    def validate_path(self, start: npt.ArrayLike, target: npt.ArrayLike) -> None:
        """Reject a commanded joint segment that crosses the declared tool floor."""
        start_q, target_q = np.asarray(start)[:6], np.asarray(target)[:6]
        for fraction in np.linspace(0, 1, 31):
            if fk(start_q + fraction * (target_q - start_q))[2, 3] < self.tool_floor_m:
                raise ValueError(
                    f"Commanded joint segment crosses the {self.tool_floor_m:g} m tool floor"
                )
