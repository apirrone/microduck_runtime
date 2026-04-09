"""
Contact odometry for microduck.

Inspired by Rhoban humanoid model_service.cpp:
  - Candidate contact points: 2 per foot (front and back of sole), 4 total.
  - At each step, find the contact point with the lowest world Z.
  - If lower than the current anchor (which sits at z=0), switch to it.
  - Re-project trunk: anchor point at flat ground (z=0), trunk oriented by IMU.

This naturally handles foot rolling: whichever edge touches first becomes
the new anchor, without any explicit torque-based contact detection.
"""

import numpy as np
from microduck_fk import MicroduckFK, DEFAULT_URDF


# Half-length of the duck foot sole along X (trunk-frame forward).
# Defines front/back contact point offsets from the foot frame origin.
_FOOT_SOLE_HALF_LEN = 0.0255  # m — half of 51mm measured foot sole length

# Candidate contact points: (side_label, frame_name, local_offset_in_frame)
_CONTACT_POINTS = [
    ("left",  "left_foot",  np.array([+_FOOT_SOLE_HALF_LEN, 0.0, 0.0])),
    ("left",  "left_foot",  np.array([-_FOOT_SOLE_HALF_LEN, 0.0, 0.0])),
    ("right", "right_foot", np.array([+_FOOT_SOLE_HALF_LEN, 0.0, 0.0])),
    ("right", "right_foot", np.array([-_FOOT_SOLE_HALF_LEN, 0.0, 0.0])),
]

# A new point must be this many metres BELOW the current anchor to take over.
# Prevents micro-switching on flat ground.
_SWITCH_MARGIN = 0.003  # m


class MicroduckOdometry:
    """
    Lowest-contact-point odometry. Call update() each cycle, then read .position / .yaw.
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF):
        self._fk = MicroduckFK(urdf_path)
        self._fk.update([0.0] * 15)

        # Current anchor: which contact point is "on the ground"
        self._anchor_frame: str = "left_foot"
        self._anchor_local: np.ndarray = np.zeros(3)
        # World X,Y of the anchor (Z is always 0 — flat ground assumption)
        self._anchor_xy: np.ndarray = np.zeros(2)

        self._quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._side = "left"

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def update(self, joint_angles, imu_quat, currents_ma=None):  # type: ignore[reportUnusedParameter]
        """
        Args:
            joint_angles: 15-element list/array (runtime motor order)
            imu_quat:     [w, x, y, z] trunk orientation in world
            currents_ma:  15-element list/array (unused — kept for API compatibility)
        """
        self._quat = np.asarray(imu_quat, dtype=float)
        R = _quat_to_rot(self._quat)
        robot = self._fk._robot

        # 1. Update FK (world frame from previous step stays valid)
        self._fk.update(joint_angles)

        # 2. Re-project robot: current anchor at (anchor_xy, 0), trunk oriented by IMU
        self._apply_support_constraint(R)

        # 3. Find if any contact point went below the current anchor (z≈0)
        new_anchor = self._find_lower_point(robot, threshold=-_SWITCH_MARGIN)
        if new_anchor is not None:
            frame, local, world_xy = new_anchor
            self._anchor_frame = frame
            self._anchor_local = local
            self._anchor_xy = world_xy
            self._side = "left" if "left" in frame else "right"
            # Re-project with new anchor
            self._apply_support_constraint(R)

    @property
    def position(self) -> np.ndarray:
        """World-frame position of trunk_base [x, y, z] in metres."""
        T = self._fk._robot.get_T_world_frame("trunk_base")
        return T[:3, 3].copy()

    @property
    def yaw(self) -> float:
        """Trunk yaw in world frame (radians)."""
        w, x, y, z = self._quat
        return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    @property
    def contact(self) -> dict:
        """{'left': bool, 'right': bool} — which side is currently the anchor."""
        return {"left": self._side == "left", "right": self._side == "right"}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _apply_support_constraint(self, R: np.ndarray) -> None:
        """
        Set the floating base so that the anchor contact point lands at
        (anchor_xy, 0) in world frame, with trunk oriented by R.
        """
        robot = self._fk._robot
        T_trunk_frame = robot.get_T_a_b("trunk_base", self._anchor_frame)
        # Anchor point expressed in trunk frame
        p_trunk_anchor = T_trunk_frame[:3, :3] @ self._anchor_local + T_trunk_frame[:3, 3]
        # Rotate to world frame
        R_p = R @ p_trunk_anchor
        # Trunk world position: anchor at (xy, 0) → trunk = anchor - R_p
        p_world_trunk = np.array([
            self._anchor_xy[0] - R_p[0],
            self._anchor_xy[1] - R_p[1],
            -R_p[2],
        ])
        T_world_trunk = np.eye(4)
        T_world_trunk[:3, :3] = R
        T_world_trunk[:3, 3] = p_world_trunk
        robot.set_T_world_frame("trunk_base", T_world_trunk)
        robot.update_kinematics()

    def _find_lower_point(self, robot, threshold: float):
        """
        Return (frame, local_offset, world_xy) for the contact point lowest in
        world Z, if it is below `threshold`. Otherwise return None.
        """
        lower_z = threshold
        best = None
        for _, frame, local in _CONTACT_POINTS:
            T = robot.get_T_world_frame(frame)
            p_world = T[:3, :3] @ local + T[:3, 3]
            if p_world[2] < lower_z:
                lower_z = p_world[2]
                best = (frame, local, p_world[:2].copy())
        return best


# ------------------------------------------------------------------
# Module-level helper (no class overhead)
# ------------------------------------------------------------------

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """[w, x, y, z] → 3×3 rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - w * z),     2 * (x * z + w * y)],
        [    2 * (x * y + w * z), 1 - 2 * (x * x + z * z),     2 * (y * z - w * x)],
        [    2 * (x * z - w * y),     2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
