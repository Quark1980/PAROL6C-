"""
PAROL6 Kinematics Engine (6-DOF URDF Forward & Inverse Kinematics)
==================================================================
Strictly adheres to PAROL6.urdf, link dimensions, joint limits, and DH parameters.
Provides pure NumPy SE(3) matrix chaining forward kinematics, analytical Jacobian,
and Damped Least-Squares (DLS) numerical Inverse Kinematics with Levenberg-Marquardt
adaptive damping and singularity protection.
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Optional, Dict, Any

# ============================================================================
# PHYSICAL AND KINEMATIC CONSTANTS FROM PAROL6 URDF & PRINTER.CFG
# ============================================================================

# CAD Home Pose in degrees (matching GO_CAD_HOME in printer.cfg)
CAD_HOME_DEG: List[float] = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]

# Hardware Physical Joint Limits in degrees (outer bounds)
HARDWARE_LIMITS_DEG: List[Tuple[float, float]] = [
    (-175.0, 175.0),     # J1: Base Yaw
    (-150.0, 50.0),      # J2: Shoulder Pitch
    (65.0, 290.0),       # J3: Elbow Pitch
    (-240.0, 180.0),     # J4: Forearm Roll
    (-135.0, 150.0),     # J5: Wrist Pitch
    (-180.0, 360.0),     # J6: Wrist Roll
]

# Default Soft Limits in degrees (user-tightenable)
DEFAULT_SOFT_LIMITS_DEG: List[List[float]] = [
    [-175.0, 175.0],
    [-150.0, 50.0],
    [65.0, 290.0],
    [-240.0, 180.0],
    [-135.0, 150.0],
    [-180.0, 360.0],
]

# Max speeds (deg/s) and accelerations (deg/s^2) per joint from printer.cfg
MAX_JOINT_SPEEDS_DEG_S: List[float] = [100.0, 50.0, 50.0, 50.0, 50.0, 50.0]
MAX_JOINT_ACCELS_DEG_S2: List[float] = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]

# Steps per degree exact constants matching printer.cfg gear ratios and 16x microstepping
STEPS_PER_DEG: List[float] = [
    88.88888889,   # J1: 3200 * 10 / 360
    444.44444444,  # J2: 3200 * 50 / 360
    444.44444444,  # J3: 3200 * 50 / 360
    39.75308642,   # J4: 3200 * (805/180) / 360
    39.14373089,   # J5: 3200 * (480/109) / 360
    8.88888889     # J6: 3200 * 1 / 360
]

# Named Preset Poses in degrees
PRESET_POSES_DEG: Dict[str, List[float]] = {
    "CAD_HOME": [0.0, -90.0, 180.0, 0.0, 0.0, 180.0],
    "CANDLE":   [0.0, -90.0, 260.0, 0.0, 0.0, 180.0],
    "PARK":     [0.0, -138.0, 110.0, 80.0, 130.0, 180.0],
    "STANDBY":  [90.0, -90.0, 180.0, 0.0, 0.0, 180.0],
}


@dataclass
class IKResult:
    success: bool
    joints_deg: List[float]
    pose_mm_deg: List[float]
    residual: float
    iterations: int
    singularity_risk: bool = False
    message: str = ""


def rpy_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    """Convert Roll-Pitch-Yaw (extrinsic XYZ: R = Rz * Ry * Rx) in radians to 3x3 rotation matrix."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ], dtype=np.float64)


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract Roll-Pitch-Yaw (extrinsic XYZ: R = Rz * Ry * Rx) in radians from 3x3 rotation matrix."""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)

    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock at pitch = +/- 90 deg
        rx = math.atan2(-R[1, 2], R[1, 1])
        rz = 0.0

    return rx, ry, rz


class Parol6Kinematics:
    """
    Complete 6-DOF Kinematic engine for PAROL6 based on URDF joint chain specifications.
    """

    def __init__(
        self,
        tool_offset_xyz_m: Tuple[float, float, float] = (0.0, 0.0, -0.105),
        base_offset_xyz_m: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        soft_limits_deg: Optional[List[List[float]]] = None,
    ):
        self.tool_offset_xyz = np.array(tool_offset_xyz_m, dtype=np.float64)
        self.base_offset_xyz = np.array(base_offset_xyz_m, dtype=np.float64)
        self.soft_limits_deg = [
            list(lim) for lim in (soft_limits_deg or DEFAULT_SOFT_LIMITS_DEG)
        ]

        # Static link transformations extracted from PAROL6.urdf
        # Format: (xyz_m, rpy_rad, axis_xyz)
        self._link_definitions = [
            # J1: base_link -> L1
            ([0.0, 0.0, 0.1105], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            # J2: L1 -> L2
            ([0.0234207210610375, 0.0, 0.0], [-math.pi / 2.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            # J3: L2 -> L3
            ([0.180, 0.0, 0.0], [math.pi, 0.0, 0.0], [0.0, 0.0, 1.0]),
            # J4: L3 -> L4
            ([-0.0435, 0.17635, 0.0], [math.pi / 2.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            # J5: L4 -> L5
            ([0.0, 0.0, 0.0], [-math.pi / 2.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
            # J6: L5 -> L6
            ([0.0, 0.037, 0.0], [math.pi / 2.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
        ]

        # Precompute static link 4x4 SE3 matrices
        self._T_static: List[np.ndarray] = []
        for xyz, rpy, _ in self._link_definitions:
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = rpy_to_matrix(*rpy)
            T[:3, 3] = xyz
            self._T_static.append(T)

    def set_tool_offset(self, x_mm: float = 0.0, y_mm: float = 0.0, z_mm: float = -105.0):
        """Configure TCP offset in millimeters relative to L6 flange."""
        self.tool_offset_xyz = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0], dtype=np.float64)

    def set_soft_limits(self, limits_deg: List[List[float]]):
        """Update active soft limits per joint."""
        for i in range(min(6, len(limits_deg))):
            hw_lo, hw_hi = HARDWARE_LIMITS_DEG[i]
            lo = max(hw_lo, float(limits_deg[i][0]))
            hi = min(hw_hi, float(limits_deg[i][1]))
            if lo < hi:
                self.soft_limits_deg[i] = [round(lo, 2), round(hi, 2)]

    def get_effective_limits_deg(self) -> List[Tuple[float, float]]:
        """Return active joint bounds intersecting hardware and soft limits."""
        eff = []
        for i in range(6):
            hw_lo, hw_hi = HARDWARE_LIMITS_DEG[i]
            s_lo, s_hi = self.soft_limits_deg[i]
            eff.append((max(hw_lo, s_lo), min(hw_hi, s_hi)))
        return eff

    def validate_joints(self, joints_deg: List[float]) -> Tuple[bool, List[str]]:
        """Check if joint angles comply with active soft limits."""
        eff = self.get_effective_limits_deg()
        errors = []
        for i in range(6):
            val = float(joints_deg[i])
            lo, hi = eff[i]
            if val < lo - 1e-3 or val > hi + 1e-3:
                errors.append(f"J{i+1} angle {val:.2f}° outside limit [{lo:.2f}°, {hi:.2f}°]")
        return len(errors) == 0, errors

    def clamp_joints(self, joints_deg: List[float]) -> List[float]:
        """Clamp joint angles to safe limits."""
        eff = self.get_effective_limits_deg()
        clamped = []
        for i in range(6):
            lo, hi = eff[i]
            val = max(lo, min(hi, float(joints_deg[i])))
            clamped.append(round(val, 3))
        return clamped

    def fk_matrix(self, joints_deg: List[float] | np.ndarray) -> np.ndarray:
        """
        Compute 4x4 SE(3) homogeneous transformation matrix from base to TCP.
        """
        q_rad = np.deg2rad(np.asarray(joints_deg[:6], dtype=np.float64))
        T = np.eye(4, dtype=np.float64)

        for i in range(6):
            c = math.cos(q_rad[i])
            s = math.sin(q_rad[i])
            # Pure Z-axis rotation matrix for revolute joint
            Rz = np.array([
                [c, -s, 0.0, 0.0],
                [s,  c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ], dtype=np.float64)

            T = T @ self._T_static[i] @ Rz

        # Apply TCP Tool Offset
        T_tool = np.eye(4, dtype=np.float64)
        T_tool[:3, 3] = self.tool_offset_xyz
        return T @ T_tool

    def fk_pose(self, joints_deg: List[float] | np.ndarray) -> List[float]:
        """
        Compute Cartesian TCP pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg].
        """
        T = self.fk_matrix(joints_deg)
        x_mm = float(T[0, 3] * 1000.0) + self.base_offset_xyz[0] * 1000.0
        y_mm = float(T[1, 3] * 1000.0) + self.base_offset_xyz[1] * 1000.0
        z_mm = float(T[2, 3] * 1000.0) + self.base_offset_xyz[2] * 1000.0
        rx, ry, rz = matrix_to_rpy(T[:3, :3])
        return [
            round(x_mm, 3),
            round(y_mm, 3),
            round(z_mm, 3),
            round(math.degrees(rx), 3),
            round(math.degrees(ry), 3),
            round(math.degrees(rz), 3),
        ]

    def pose_to_matrix(self, pose_mm_deg: List[float]) -> np.ndarray:
        """Convert [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] to 4x4 SE(3) matrix."""
        x_m = (float(pose_mm_deg[0]) - self.base_offset_xyz[0] * 1000.0) / 1000.0
        y_m = (float(pose_mm_deg[1]) - self.base_offset_xyz[1] * 1000.0) / 1000.0
        z_m = (float(pose_mm_deg[2]) - self.base_offset_xyz[2] * 1000.0) / 1000.0
        rx = math.radians(float(pose_mm_deg[3]))
        ry = math.radians(float(pose_mm_deg[4]))
        rz = math.radians(float(pose_mm_deg[5]))

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rpy_to_matrix(rx, ry, rz)
        T[:3, 3] = [x_m, y_m, z_m]
        return T

    def fk_and_jacobian(self, q_rad: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics transform T_tcp and 6x6 geometric Jacobian J
        simultaneously in a single unified pass with minimal allocations.
        """
        T_curr = np.eye(4, dtype=np.float64)
        joint_origins = np.empty((6, 3), dtype=np.float64)
        joint_z_axes = np.empty((6, 3), dtype=np.float64)

        for i in range(6):
            T_joint_frame = T_curr @ self._T_static[i]
            joint_origins[i] = T_joint_frame[:3, 3]
            joint_z_axes[i] = T_joint_frame[:3, 2]

            c = math.cos(q_rad[i])
            s = math.sin(q_rad[i])
            col0 = T_joint_frame[:, 0].copy()
            col1 = T_joint_frame[:, 1].copy()
            T_curr = T_joint_frame.copy()
            T_curr[:, 0] = c * col0 + s * col1
            T_curr[:, 1] = -s * col0 + c * col1

        T_tcp = T_curr.copy()
        if np.any(self.tool_offset_xyz):
            T_tcp[:3, 3] += T_tcp[:3, :3] @ self.tool_offset_xyz
        p_tcp = T_tcp[:3, 3]

        J = np.empty((6, 6), dtype=np.float64)
        for i in range(6):
            z_i = joint_z_axes[i]
            p_i = joint_origins[i]
            J[0, i] = z_i[1] * (p_tcp[2] - p_i[2]) - z_i[2] * (p_tcp[1] - p_i[1])
            J[1, i] = z_i[2] * (p_tcp[0] - p_i[0]) - z_i[0] * (p_tcp[2] - p_i[2])
            J[2, i] = z_i[0] * (p_tcp[1] - p_i[1]) - z_i[1] * (p_tcp[0] - p_i[0])
            J[3, i] = z_i[0]
            J[4, i] = z_i[1]
            J[5, i] = z_i[2]

        return T_tcp, J

    def compute_jacobian(self, joints_deg: List[float] | np.ndarray) -> np.ndarray:
        """
        Compute 6x6 geometric Jacobian J = [J_v; J_omega] at current joint configuration.
        Correctly samples joint origins and Z-rotation axes after each link's static transform.
        """
        q_rad = np.deg2rad(np.asarray(joints_deg[:6], dtype=np.float64))
        _, J = self.fk_and_jacobian(q_rad)
        return J

    def check_singularities(self, joints_deg: List[float]) -> Tuple[bool, str]:
        """
        Guard against kinematic singularities:
        - Wrist singularity: J5 approx 0 degrees (axes of J4 and J6 align).
        - Elbow singularity: J3 near upper/lower extension boundary.
        - Shoulder singularity: wrist center lies on J1 axis.
        """
        j5 = abs(float(joints_deg[4]))
        if j5 < 2.0:
            return True, f"Wrist singularity: J5 angle {j5:.2f}° near 0°"

        j3 = float(joints_deg[2])
        if abs(j3 - 180.0) > 85.0:
            return True, f"Elbow boundary singularity: J3 angle {j3:.2f}°"

        T = self.fk_matrix(joints_deg)
        r_xy = math.hypot(T[0, 3], T[1, 3])
        if r_xy < 0.025:
            return True, f"Shoulder overhead singularity: distance to base axis {r_xy*1000:.1f} mm < 25 mm"

        return False, "Nominal"

    def select_closest_wrist_branch(
        self,
        candidate_joints_deg: List[float],
        seed_joints_deg: List[float],
        weights: Optional[List[float]] = None
    ) -> List[float]:
        """
        Enforce kinematic branch continuity for the 3-axis spherical wrist (J4, J5, J6).
        The wrist configuration (q4, q5, q6) has an exact kinematically equivalent branch:
            (q4 ± 180°, -q5, q6 ± 180°)
        which produces the identical end-effector rotation matrix.
        Evaluates both branches and all ±360° wraps within soft limits, selecting the
        configuration that minimizes weighted distance to seed_joints_deg.
        """
        eff = self.get_effective_limits_deg()
        w = weights or [1.0, 1.0, 1.0, 1.5, 2.0, 1.5]

        q_cand = list(candidate_joints_deg)
        q_seed = list(seed_joints_deg)

        q4, q5, q6 = q_cand[3], q_cand[4], q_cand[5]

        branch_wrist = [
            (q4, q5, q6),
            (q4 + 180.0, -q5, q6 + 180.0),
            (q4 - 180.0, -q5, q6 + 180.0),
            (q4 + 180.0, -q5, q6 - 180.0),
            (q4 - 180.0, -q5, q6 - 180.0),
        ]

        valid_candidates = []
        for b4, b5, b6 in branch_wrist:
            for wrap4 in [0.0, -360.0, 360.0]:
                for wrap6 in [0.0, -360.0, 360.0]:
                    cand = [
                        q_cand[0],
                        q_cand[1],
                        q_cand[2],
                        b4 + wrap4,
                        b5,
                        b6 + wrap6
                    ]
                    in_limits = True
                    for j in range(6):
                        lo, hi = eff[j]
                        if cand[j] < lo - 1e-2 or cand[j] > hi + 1e-2:
                            in_limits = False
                            break
                    if in_limits:
                        cost = sum(w[j] * ((cand[j] - q_seed[j]) ** 2) for j in range(6))
                        valid_candidates.append((cost, cand))

        if valid_candidates:
            valid_candidates.sort(key=lambda x: x[0])
            return [round(float(v), 3) for v in valid_candidates[0][1]]
        return [round(float(v), 3) for v in q_cand]

    def unwrap_revolute_joints(
        self,
        target_joints_deg: List[float],
        reference_joints_deg: List[float]
    ) -> List[float]:
        """
        Unwrap continuous / large-range revolute joints to shortest angular distance
        relative to reference_joints_deg:
            Δq_i = atan2(sin(target_i - ref_i), cos(target_i - ref_i))
            unwrapped_i = ref_i + Δq_i
        Preserves within soft limits.
        """
        eff = self.get_effective_limits_deg()
        res = list(target_joints_deg)
        for i in [3, 5]:  # J4 and J6
            diff_rad = math.atan2(
                math.sin(math.radians(target_joints_deg[i] - reference_joints_deg[i])),
                math.cos(math.radians(target_joints_deg[i] - reference_joints_deg[i]))
            )
            unwrapped = reference_joints_deg[i] + math.degrees(diff_rad)
            lo, hi = eff[i]
            if lo - 1e-2 <= unwrapped <= hi + 1e-2:
                res[i] = round(unwrapped, 3)
        return res

    def solve_ik(
        self,
        target_pose_mm_deg: List[float],
        seed_joints_deg: List[float],
        max_iterations: int = 15,
        tolerance_mm: float = 0.5,
        tolerance_deg: float = 0.5,
    ) -> IKResult:
        """
        Solve 6-DOF Inverse Kinematics using Damped Least-Squares (DLS)
        with adaptive Levenberg-Marquardt damping, null-space posture regularization,
        and kinematic branch selection.
        Optimized single-pass FK + Jacobian execution.
        """
        T_target = self.pose_to_matrix(target_pose_mm_deg)
        p_target = T_target[:3, 3]
        R_target = T_target[:3, :3]

        eff_limits = self.get_effective_limits_deg()
        q_min = np.deg2rad([lim[0] for lim in eff_limits])
        q_max = np.deg2rad([lim[1] for lim in eff_limits])

        q = np.deg2rad(np.asarray(seed_joints_deg[:6], dtype=np.float64))
        q = np.clip(q, q_min + 1e-3, q_max - 1e-3)
        q_seed_rad = q.copy()

        base_damping = 0.02
        weights = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5], dtype=np.float64)

        for it in range(max_iterations):
            T_curr, J = self.fk_and_jacobian(q)
            p_curr = T_curr[:3, 3]
            R_curr = T_curr[:3, :3]

            # Position error in meters
            dp = p_target - p_curr
            # Orientation error vector from rotation matrix difference
            R_err = R_target @ R_curr.T
            tr = np.trace(R_err)
            cos_theta = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
            theta = math.acos(cos_theta)
            if theta < 1e-5:
                w_err = np.zeros(3, dtype=np.float64)
            else:
                w_err = (theta / (2.0 * math.sin(theta))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ], dtype=np.float64)

            error = np.concatenate([dp, w_err]) * weights
            pos_err_mm = np.linalg.norm(dp) * 1000.0
            rot_err_deg = math.degrees(np.linalg.norm(w_err))

            # Convergence test
            if pos_err_mm < tolerance_mm and rot_err_deg < tolerance_deg:
                raw_sol_deg = [round(float(a), 3) for a in np.rad2deg(q)]
                # Ensure closest wrist branch and shortest angular distance to seed
                branch_sol_deg = self.select_closest_wrist_branch(raw_sol_deg, seed_joints_deg)
                sol_deg = self.unwrap_revolute_joints(branch_sol_deg, seed_joints_deg)
                is_sing, sing_msg = self.check_singularities(sol_deg)
                return IKResult(
                    success=True,
                    joints_deg=sol_deg,
                    pose_mm_deg=self.fk_pose(sol_deg),
                    residual=float(pos_err_mm),
                    iterations=it + 1,
                    singularity_risk=is_sing,
                    message="Converged" if not is_sing else f"Converged ({sing_msg})",
                )

            JJt = J @ J.T
            # Adaptive damping: increase damping near wrist singularity (J5 near 0)
            is_near_sing = abs(q[4]) < math.radians(4.0)
            damping = base_damping * (2.5 if is_near_sing else 1.0)

            dls_matrix = JJt + (damping ** 2) * np.eye(6, dtype=np.float64)
            try:
                J_pinv = J.T @ np.linalg.inv(dls_matrix)
            except np.linalg.LinAlgError:
                J_pinv = J.T @ np.linalg.pinv(dls_matrix)

            dq_task = J_pinv @ error

            # Null-Space posture regularization: penalize deviation from seed posture
            k_posture = 0.12
            null_proj = np.eye(6, dtype=np.float64) - J_pinv @ J
            dq_posture = null_proj @ (k_posture * (q_seed_rad - q))

            dq = dq_task + dq_posture

            # Limit max joint update step per iteration (max 15 deg per step)
            max_step = math.radians(15.0)
            dq_norm = np.linalg.norm(dq)
            if dq_norm > max_step:
                dq = dq * (max_step / dq_norm)

            # Apply step and project onto active soft limits
            q = np.clip(q + dq, q_min, q_max)

        # Iterations exhausted without full convergence
        raw_sol_deg = [round(float(a), 3) for a in np.rad2deg(q)]
        branch_sol_deg = self.select_closest_wrist_branch(raw_sol_deg, seed_joints_deg)
        sol_deg = self.unwrap_revolute_joints(branch_sol_deg, seed_joints_deg)
        is_sing, sing_msg = self.check_singularities(sol_deg)
        return IKResult(
            success=False,
            joints_deg=sol_deg,
            pose_mm_deg=self.fk_pose(sol_deg),
            residual=float(pos_err_mm),
            iterations=max_iterations,
            singularity_risk=is_sing,
            message="Max iterations reached without meeting tolerance",
        )
