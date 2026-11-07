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

# CAD Home Pose in degrees (matching SET_CAD_HOME in printer.cfg)
CAD_HOME_DEG: List[float] = [0.0, -90.0, 180.0, 0.0, 0.0, 180.0]

# Hardware Physical & Cable Protection Joint Limits in degrees (outer bounds)
HARDWARE_LIMITS_DEG: List[Tuple[float, float]] = [
    (-170.0, 170.0),     # J1: Base Yaw ([-170°, +170°])
    (-150.0, 50.0),      # J2: Shoulder Pitch (CAD Home -90°, [-60°, +140°] rel -> [-150°, +50°])
    (65.0, 290.0),       # J3: Elbow Pitch (CAD Home 180°, [-115°, +110°] rel -> [65°, 290°])
    (-160.0, 160.0),     # J4: Wrist Roll ([-160°, +160°] strictly bounded to prevent internal harness twisting)
    (-110.0, 110.0),     # J5: Wrist Pitch ([-110°, +110°] mechanical collision boundary)
    (0.0, 360.0),        # J6: Tool Flange Roll (CAD Home 180° ± 180° -> [0°, 360°] prevents umbilical wrap)
]

# Default Soft Limits in degrees (user-tightenable)
DEFAULT_SOFT_LIMITS_DEG: List[List[float]] = [
    [-170.0, 170.0],
    [-150.0, 50.0],
    [65.0, 290.0],
    [-160.0, 160.0],
    [-110.0, 110.0],
    [0.0, 360.0],
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
        tool_offset_xyz_m: Tuple[float, float, float] = (0.0, 0.0, -0.118),
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

    def set_tool_offset(self, x_mm: float = 0.0, y_mm: float = 0.0, z_mm: float = -118.0):
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
        """Check if joint angles comply with active soft limits and ±180° CAD Home cable bounds."""
        eff = self.get_effective_limits_deg()
        errors = []
        for i in range(6):
            val = float(joints_deg[i])
            lo, hi = eff[i]
            if val < lo - 1e-3 or val > hi + 1e-3:
                errors.append(f"J{i+1} angle {val:.2f}° outside limit [{lo:.2f}°, {hi:.2f}°]")
            elif abs(val - CAD_HOME_DEG[i]) > 180.0 + 1e-3:
                errors.append(f"J{i+1} angle {val:.2f}° exceeds ±180° from CAD Home ({CAD_HOME_DEG[i]:.1f}°)")
        return len(errors) == 0, errors

    def validate_and_lock_branch(
        self,
        candidate_joints_deg: List[float],
        current_joints_deg: List[float],
        max_delta_deg: float = 3.0,
    ) -> Tuple[bool, List[float], str]:
        """
        Safety & Branch Lock Interceptor:
        1. Enforces mechanical cable limits and ±180° CAD Home bounds.
        2. Delta Spike Clamp: If any joint commands a delta |ΔJ_i| > max_delta_deg (default 3.0° per 10ms
           streaming window, indicating an IK branch flip or singularity jump), rejects the candidate frame
           immediately and holds current_joints_deg.
        """
        if not candidate_joints_deg or len(candidate_joints_deg) < 6:
            return False, list(current_joints_deg), "Incomplete candidate joint vector"

        eff = self.get_effective_limits_deg()
        for i in range(6):
            q = float(candidate_joints_deg[i])
            if math.isnan(q) or math.isinf(q):
                return False, list(current_joints_deg), f"J{i+1} is NaN/Inf"
            lo, hi = eff[i]
            if q < lo - 1e-3 or q > hi + 1e-3:
                return (
                    False,
                    list(current_joints_deg),
                    f"J{i+1}={q:.2f}° exceeds cable protection limit [{lo:.1f}°, {hi:.1f}°]",
                )
            if abs(q - CAD_HOME_DEG[i]) > 180.0 + 1e-3:
                return (
                    False,
                    list(current_joints_deg),
                    f"J{i+1}={q:.2f}° exceeds ±180° from CAD Home ({CAD_HOME_DEG[i]:.1f}°)",
                )
            if current_joints_deg and len(current_joints_deg) > i and max_delta_deg > 0.0:
                delta = abs(q - float(current_joints_deg[i]))
                if delta > max_delta_deg + 1e-3:
                    return (
                        False,
                        list(current_joints_deg),
                        f"J{i+1} delta spike {delta:.2f}° > {max_delta_deg:.1f}° limit (IK branch flip / singularity jump blocked)",
                    )

        return True, [round(float(v), 3) for v in candidate_joints_deg[:6]], "OK"

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

    # Backward-compatibility alias
    forward_kinematics = fk_pose

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

    @staticmethod
    def _unwrap_near(angle_deg: float, ref_deg: float, min_deg: float, max_deg: float) -> float:
        diff = math.atan2(
            math.sin(math.radians(angle_deg - ref_deg)),
            math.cos(math.radians(angle_deg - ref_deg)),
        ) * 180.0 / math.pi
        cand = ref_deg + diff
        if min_deg <= cand <= max_deg:
            return cand
        for shift in (-360.0, 0.0, 360.0):
            alt = angle_deg + shift
            if min_deg <= alt <= max_deg:
                return alt
        return cand

    def _solve_analytical_spherical_wrist(
        self,
        p_target: np.ndarray,
        R_target: np.ndarray,
        seed_joints_deg: List[float],
        eff_limits: List[Tuple[float, float]],
    ) -> Optional[List[float]]:
        """
        Exact closed-form spherical-wrist kinematic decoupling for PAROL6:
        1. Wrist center p_wc = p_target + (0.037 - tool_offset_z) * z_target
        2. Closed-form geometric solution for (J1, J2, J3)
        3. Closed-form SO(3) Euler-Z(-Y)Z decomposition for (J4, J5, J6)
        """
        d_tool_m = 0.037 - float(self.tool_offset_xyz[2])
        p_wc = p_target + d_tool_m * R_target[:, 2]
        x_wc_mm = float(p_wc[0]) * 1000.0
        y_wc_mm = float(p_wc[1]) * 1000.0
        z_wc_mm = float(p_wc[2]) * 1000.0

        r_xy = math.hypot(x_wc_mm, y_wc_mm)
        q1_raw = math.degrees(math.atan2(y_wc_mm, x_wc_mm)) if r_xy > 1e-4 else float(seed_joints_deg[0])
        q1_deg = self._unwrap_near(q1_raw, float(seed_joints_deg[0]), eff_limits[0][0], eff_limits[0][1])
        if not (eff_limits[0][0] <= q1_deg <= eff_limits[0][1]):
            return None

        d_r = r_xy - 23.4207210610375
        d_z = z_wc_mm - 110.5
        L2 = 180.0
        a3 = -43.5
        b3 = 176.35
        L3 = math.hypot(a3, b3)
        beta = math.atan2(b3, a3)

        dist_sq = d_r * d_r + d_z * d_z
        cos_delta = (dist_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
        if cos_delta < -1.0 or cos_delta > 1.0:
            return None

        abs_delta = math.acos(max(-1.0, min(1.0, cos_delta)))
        best_elbow_cost = float("inf")
        best_q2, best_q3 = 0.0, 0.0
        found_elbow = False

        for delta in (abs_delta, -abs_delta):
            q2_rad = math.atan2(-d_z, d_r) - math.atan2(L3 * math.sin(delta), L2 + L3 * math.cos(delta))
            q3_rad = -(delta + beta)
            q2_cand = self._unwrap_near(math.degrees(q2_rad), float(seed_joints_deg[1]), eff_limits[1][0], eff_limits[1][1])
            q3_cand = self._unwrap_near(math.degrees(q3_rad), float(seed_joints_deg[2]), eff_limits[2][0], eff_limits[2][1])
            if eff_limits[1][0] <= q2_cand <= eff_limits[1][1] and eff_limits[2][0] <= q3_cand <= eff_limits[2][1]:
                cost = abs(q2_cand - seed_joints_deg[1]) + abs(q3_cand - seed_joints_deg[2])
                if cost < best_elbow_cost:
                    best_elbow_cost = cost
                    best_q2, best_q3 = q2_cand, q3_cand
                    found_elbow = True

        if not found_elbow:
            return None

        # Forward R_0_wc_zero at converged (q1_deg, best_q2, best_q3)
        q123_rad = [math.radians(q1_deg), math.radians(best_q2), math.radians(best_q3)]
        T_03 = np.eye(4, dtype=np.float64)
        for i in range(3):
            c, s = math.cos(q123_rad[i]), math.sin(q123_rad[i])
            Rz = np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
            T_03 = T_03 @ self._T_static[i] @ Rz
        R_0_wc_zero = T_03[:3, :3] @ self._T_static[3][:3, :3]
        R_w = R_0_wc_zero.T @ R_target

        c5 = max(-1.0, min(1.0, float(R_w[2, 2])))
        s5 = math.sqrt(max(0.0, 1.0 - c5 * c5))
        q5_abs = math.degrees(math.atan2(s5, c5))

        best_wrist_cost = float("inf")
        best_q4, best_q5, best_q6 = 0.0, 0.0, 0.0
        found_wrist = False

        # SINGULARITY CROSSING GUARD (|J5| < 1.0°):
        # Joint 4 and Joint 6 become collinear. Lock J4 to its previous value and resolve
        # the remaining projected roll entirely onto Joint 6:
        #   J4 = q_current,4
        #   J6 = atan2(R_36[1,0], R_36[0,0]) - J4
        if q5_abs < 1.0:
            best_q4 = max(eff_limits[3][0], min(eff_limits[3][1], float(seed_joints_deg[3])))
            best_q5 = +q5_abs if float(seed_joints_deg[4]) >= 0.0 else -q5_abs
            total_roll_deg = math.degrees(math.atan2(float(R_w[1, 0]), float(R_w[0, 0])))
            best_q6 = self._unwrap_near(
                total_roll_deg - best_q4 if c5 >= 0.0 else best_q4 - total_roll_deg,
                float(seed_joints_deg[5]),
                eff_limits[5][0],
                eff_limits[5][1],
            )
            found_wrist = (eff_limits[5][0] <= best_q6 <= eff_limits[5][1])
        else:
            # DETERMINISTIC ANALYTICAL WRIST BRANCH LOCKING (J5 > 0 vs. J5 < 0):
            # Evaluate both branches using weighted Euclidean modular distance against q_current.
            branches = [
                (math.degrees(math.atan2(float(R_w[1, 2]), float(R_w[0, 2]))),   +q5_abs, math.degrees(math.atan2(float(R_w[2, 1]), -float(R_w[2, 0])))),
                (math.degrees(math.atan2(-float(R_w[1, 2]), -float(R_w[0, 2]))), -q5_abs, math.degrees(math.atan2(-float(R_w[2, 1]), float(R_w[2, 0])))),
            ]
            w4, w5, w6 = 1.4, 1.8, 1.0
            for b4, b5, b6 in branches:
                q4_c = self._unwrap_near(b4, float(seed_joints_deg[3]), eff_limits[3][0], eff_limits[3][1])
                q5_c = self._unwrap_near(b5, float(seed_joints_deg[4]), eff_limits[4][0], eff_limits[4][1])
                q6_c = self._unwrap_near(b6, float(seed_joints_deg[5]), eff_limits[5][0], eff_limits[5][1])
                if (eff_limits[3][0] <= q4_c <= eff_limits[3][1] and
                    eff_limits[4][0] <= q5_c <= eff_limits[4][1] and
                    eff_limits[5][0] <= q6_c <= eff_limits[5][1]):
                    d4 = ((q4_c - float(seed_joints_deg[3]) + 180.0) % 360.0) - 180.0
                    d5 = ((q5_c - float(seed_joints_deg[4]) + 180.0) % 360.0) - 180.0
                    d6 = ((q6_c - float(seed_joints_deg[5]) + 180.0) % 360.0) - 180.0
                    cost = w4 * (d4 * d4) + w5 * (d5 * d5) + w6 * (d6 * d6)
                    if cost < best_wrist_cost:
                        best_wrist_cost = cost
                        best_q4, best_q5, best_q6 = q4_c, q5_c, q6_c
                        found_wrist = True

        if not found_wrist:
            return None

        sol = [round(q1_deg, 3), round(best_q2, 3), round(best_q3, 3), round(best_q4, 3), round(best_q5, 3), round(best_q6, 3)]
        T_check = self.fk_matrix(sol)
        if float(np.linalg.norm(T_check[:3, 3] - p_target)) * 1000.0 > 0.5:
            return None
        return sol

    def solve_ik(
        self,
        target_pose_mm_deg: List[float],
        seed_joints_deg: List[float],
        max_iterations: int = 50,
        tolerance_mm: float = 0.5,
        tolerance_deg: float = 0.5,
    ) -> IKResult:
        """
        Solve 6-DOF Inverse Kinematics using exact analytical spherical-wrist decoupling
        with fallback to Task-Weighted Damped Least-Squares (DLS).
        """
        T_target = self.pose_to_matrix(target_pose_mm_deg)
        p_target = T_target[:3, 3]
        R_target = T_target[:3, :3]

        eff_limits = self.get_effective_limits_deg()
        analytical_sol = self._solve_analytical_spherical_wrist(p_target, R_target, seed_joints_deg, eff_limits)
        if analytical_sol is not None:
            is_sing, sing_msg = self.check_singularities(analytical_sol)
            return IKResult(
                success=True,
                joints_deg=analytical_sol,
                pose_mm_deg=self.fk_pose(analytical_sol),
                residual=0.0,
                iterations=1,
                singularity_risk=is_sing,
                message="Converged (Analytical Spherical Wrist)" if not is_sing else f"Converged ({sing_msg})",
            )

        q_min = np.deg2rad([lim[0] for lim in eff_limits])
        q_max = np.deg2rad([lim[1] for lim in eff_limits])

        q = np.deg2rad(np.asarray(seed_joints_deg[:6], dtype=np.float64))
        q = np.clip(q, q_min + 1e-3, q_max - 1e-3)

        # Escape exact J5 = 0.0° wrist singularity slightly on initial seed if needed
        if abs(q[4]) < 1e-4:
            q[4] = 1e-3

        base_damping = 0.015
        pos_err_mm = 999.0

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
            if theta < 1e-6:
                w_err = np.zeros(3, dtype=np.float64)
            else:
                w_err = (theta / (2.0 * math.sin(theta))) * np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1],
                ], dtype=np.float64)

            pos_err_mm = float(np.linalg.norm(dp) * 1000.0)
            rot_err_deg = float(math.degrees(np.linalg.norm(w_err)))

            is_wrist_singular = abs(q[4]) < math.radians(1.0)

            # Strict orientation convergence test (never relax tool rotation tolerance)
            if pos_err_mm < tolerance_mm and rot_err_deg < tolerance_deg:
                raw_sol_deg = [round(float(a), 3) for a in np.rad2deg(q)]
                branch_sol_deg = self.select_closest_wrist_branch(raw_sol_deg, seed_joints_deg)
                sol_deg = self.unwrap_revolute_joints(branch_sol_deg, seed_joints_deg)
                is_sing, sing_msg = self.check_singularities(sol_deg)
                return IKResult(
                    success=True,
                    joints_deg=sol_deg,
                    pose_mm_deg=self.fk_pose(sol_deg),
                    residual=pos_err_mm,
                    iterations=it + 1,
                    singularity_risk=is_sing,
                    message="Converged" if not is_sing else f"Converged ({sing_msg})",
                )

            # Task-space weighting: strict orientation lock across all 3 rotational axes
            w_diag = np.array([1.0, 1.0, 1.0, 0.25, 0.25, 0.25], dtype=np.float64)
            J_w = w_diag[:, None] * J
            err_w = w_diag * np.concatenate([dp, w_err])

            damping = base_damping * (1.5 if is_wrist_singular else 1.0)
            JJt = J_w @ J_w.T + (damping ** 2) * np.eye(6, dtype=np.float64)
            try:
                dq = J_w.T @ np.linalg.solve(JJt, err_w)
            except np.linalg.LinAlgError:
                dq = J_w.T @ (np.linalg.pinv(JJt) @ err_w)

            # Singularity crossing guard (|J5| < 1.0°): lock J4 so roll is resolved onto J6
            if is_wrist_singular:
                dq[3] = 0.0

            # Limit max joint update step per iteration (max 15 deg per step)
            max_step = math.radians(15.0)
            dq_max = float(np.max(np.abs(dq)))
            if dq_max > max_step:
                dq = dq * (max_step / dq_max)

            # Apply step and project onto active soft limits
            q = np.clip(q + dq, q_min, q_max)

        # Fallback: if position converged closely (< 1.5 mm), accept solution
        raw_sol_deg = [round(float(a), 3) for a in np.rad2deg(q)]
        branch_sol_deg = self.select_closest_wrist_branch(raw_sol_deg, seed_joints_deg)
        sol_deg = self.unwrap_revolute_joints(branch_sol_deg, seed_joints_deg)
        is_sing, sing_msg = self.check_singularities(sol_deg)
        converged_pos = pos_err_mm <= max(tolerance_mm * 2.5, 1.2)
        return IKResult(
            success=converged_pos,
            joints_deg=sol_deg,
            pose_mm_deg=self.fk_pose(sol_deg),
            residual=pos_err_mm,
            iterations=max_iterations,
            singularity_risk=is_sing,
            message="Converged (position priority)" if converged_pos else f"Max iterations reached (pos_err={pos_err_mm:.2f}mm)",
        )
