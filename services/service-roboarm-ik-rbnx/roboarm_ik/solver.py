from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R


DEFAULT_INIT_DEG = [0.0, 99.88, -91.26, 0.0, 63.84, 0.0]
DEFAULT_JOINT_LIMITS_DEG = [
    (-150.0, 150.0),
    (0.0, 180.0),
    (-170.0, 0.0),
    (-100.0, 100.0),
    (-70.0, 70.0),
    (-120.0, 120.0),
]


@dataclass(frozen=True)
class IkResult:
    success: bool
    joints_rad: np.ndarray
    cost: float
    message: str

    @property
    def joints_deg(self) -> list[float]:
        return np.rad2deg(self.joints_rad).astype(float).tolist()


def default_urdf_path() -> str:
    repo_root = Path(__file__).resolve().parents[4]
    candidates = [
        os.environ.get("PIPER_URDF_PATH", ""),
        repo_root / "urdf" / "piper.urdf",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate).resolve())
    raise FileNotFoundError("cannot locate Piper URDF; pass urdf_path explicitly")


def parse_float_list(value: Any, default: list[float]) -> list[float]:
    if value is None:
        return list(default)
    if isinstance(value, str):
        value = value.strip()
        if not value:
            return list(default)
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            value = [part.strip() for part in value.split(",")]
    return [float(v) for v in value]


def wrap_angle_rad(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def pose_matrix_from_xyz_quat(
    xyz: tuple[float, float, float] | list[float] | np.ndarray,
    quat_xyzw: tuple[float, float, float, float] | list[float] | np.ndarray,
) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_quat(np.asarray(quat_xyzw, dtype=float)).as_matrix()
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


def vertical_down_matrix(
    xyz: tuple[float, float, float] | list[float] | np.ndarray,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_euler(
        "zyx", [np.rad2deg(yaw_rad), 180.0, 0.0], degrees=True
    ).as_matrix()
    matrix[:3, 3] = np.asarray(xyz, dtype=float)
    return matrix


class PiperRoboarmIk:
    """Pure IK solver ported from roboarm's PiperBySDK path."""

    position_tol_m = 0.01
    tilt_tol_rad = np.deg2rad(5.0)
    yaw_tol_rad = np.deg2rad(10.0)

    def __init__(self, urdf_path: str | None = None, tip_link: str = "link6"):
        import kinpy

        self.urdf_path = urdf_path or default_urdf_path()
        self.tip_link = tip_link
        with open(self.urdf_path, "r", encoding="utf-8") as f:
            urdf_content = f.read()
        urdf_content = re.sub(r"<\?xml[^?]*\?>", "", urdf_content, count=1)
        self.chain = kinpy.build_serial_chain_from_urdf(urdf_content, tip_link)
        self.joint_bounds = self._parse_joint_bounds(urdf_content)
        if len(self.joint_bounds) != 6:
            self.joint_bounds = [
                (np.deg2rad(lo), np.deg2rad(hi))
                for lo, hi in DEFAULT_JOINT_LIMITS_DEG
            ]

    @staticmethod
    def _parse_joint_bounds(urdf_content: str) -> list[tuple[float, float]]:
        root = ElementTree.fromstring(urdf_content)
        bounds: list[tuple[float, float]] = []
        for joint_elem in root.findall("joint"):
            if joint_elem.get("type") != "revolute":
                continue
            limit = joint_elem.find("limit")
            if limit is None:
                continue
            bounds.append(
                (
                    float(limit.get("lower", "0")),
                    float(limit.get("upper", "0")),
                )
            )
            if len(bounds) == 6:
                break
        return bounds

    @classmethod
    def cost(cls, joint_angles: np.ndarray, target_pose_matrix: np.ndarray, chain) -> float:
        current_pose_matrix = chain.forward_kinematics(joint_angles).matrix()
        pos_error = np.linalg.norm(
            current_pose_matrix[:3, 3] - target_pose_matrix[:3, 3]
        )

        current_rot = R.from_matrix(current_pose_matrix[:3, :3])
        target_rot = R.from_matrix(target_pose_matrix[:3, :3])
        current_z_axis = current_rot.apply([0.0, 0.0, 1.0])
        target_z_axis = target_rot.apply([0.0, 0.0, 1.0])
        cosine = np.clip(float(np.dot(current_z_axis, target_z_axis)), -1.0, 1.0)
        tilt_error = float(np.arccos(cosine))

        current_yaw = current_rot.as_euler("zyx")[0]
        target_yaw = target_rot.as_euler("zyx")[0]
        yaw_error = abs(wrap_angle_rad(current_yaw - target_yaw))

        pos_term = (pos_error / cls.position_tol_m) ** 2
        tilt_term = (tilt_error / cls.tilt_tol_rad) ** 2
        yaw_term = (yaw_error / cls.yaw_tol_rad) ** 2
        return float(pos_term + tilt_term + yaw_term)

    def solve_pose_matrix(
        self,
        target_pose_matrix: np.ndarray,
        current_joints_rad: np.ndarray | list[float] | None = None,
        init_joints_rad: np.ndarray | list[float] | None = None,
        attempts: int = 12,
        maxiter: int = 120,
        success_cost: float = 9.0,
        seed: int = 7,
    ) -> IkResult:
        if current_joints_rad is None:
            current = np.deg2rad(DEFAULT_INIT_DEG)
        else:
            current = np.asarray(current_joints_rad, dtype=float)
        if init_joints_rad is None:
            init = np.deg2rad(DEFAULT_INIT_DEG)
        else:
            init = np.asarray(init_joints_rad, dtype=float)

        rng = np.random.default_rng(seed)
        guesses = [current, init, np.zeros(6, dtype=float)]
        span = np.array([hi - lo for lo, hi in self.joint_bounds], dtype=float)
        center = np.array([(lo + hi) * 0.5 for lo, hi in self.joint_bounds], dtype=float)
        while len(guesses) < max(1, attempts):
            if len(guesses) % 2 == 0:
                guesses.append(current + rng.normal(0.0, 0.20, 6))
            else:
                guesses.append(center + rng.uniform(-0.35, 0.35, 6) * span)

        best_joints = current
        best_cost = float("inf")
        best_msg = "no attempts"
        for guess in guesses[: max(1, attempts)]:
            x0 = np.array(
                [
                    np.clip(value, self.joint_bounds[index][0], self.joint_bounds[index][1])
                    for index, value in enumerate(guess)
                ],
                dtype=float,
            )
            result = minimize(
                self.cost,
                x0=x0,
                args=(target_pose_matrix, self.chain),
                method="SLSQP",
                bounds=self.joint_bounds,
                options={"maxiter": int(maxiter), "ftol": 1e-6, "disp": False},
            )
            cost = float(result.fun) if np.isfinite(result.fun) else float("inf")
            if cost < best_cost:
                best_cost = cost
                best_joints = np.asarray(result.x, dtype=float)
                best_msg = str(result.message)
            if result.success and best_cost <= success_cost:
                break

        return IkResult(
            success=bool(best_cost <= success_cost),
            joints_rad=best_joints,
            cost=best_cost,
            message=best_msg,
        )

    def solve_xyz_quat(
        self,
        xyz: tuple[float, float, float] | list[float] | np.ndarray,
        quat_xyzw: tuple[float, float, float, float] | list[float] | np.ndarray,
        current_joints_rad: np.ndarray | list[float] | None = None,
        **kwargs,
    ) -> IkResult:
        return self.solve_pose_matrix(
            pose_matrix_from_xyz_quat(xyz, quat_xyzw),
            current_joints_rad=current_joints_rad,
            **kwargs,
        )

    def solve_vertical_down(
        self,
        xyz: tuple[float, float, float] | list[float] | np.ndarray,
        yaw_rad: float = 0.0,
        current_joints_rad: np.ndarray | list[float] | None = None,
        **kwargs,
    ) -> IkResult:
        return self.solve_pose_matrix(
            vertical_down_matrix(xyz, yaw_rad=yaw_rad),
            current_joints_rad=current_joints_rad,
            **kwargs,
        )
