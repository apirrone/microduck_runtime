"""
Contact-based odometry for microduck.

Contact detection fuses two signals:
  - Torque excess: measured torque (from motor current) minus gravity-compensation
    torque from the Placo model. High excess on ankle+knee → foot in stance.
  - Foot height: which foot has lower world-frame Z (from FK + IMU orientation).

Position integration:
  - Planted foot = world anchor. FK tells us where the foot is in trunk frame,
    IMU gives trunk orientation, so:
      pos_world_trunk = anchor_world_foot - R_world_trunk @ pos_trunk_foot
  - Anchor is saved on each new contact event.
  - Double-stance: average of both foot estimates.
"""

import numpy as np
from microduck_fk import MicroduckFK, DEFAULT_URDF

# XL330 motor torque constant (Nm per A).
# Stall torque ~0.75 Nm @ 11.1V, stall current ~1.2 A → kT ≈ 0.625 Nm/A
XL330_KT = 0.625
# Static friction threshold (Nm) — below this, torque reading is noise
XL330_FRICTION_NM = 0.02

# Indices in the 15-element motor vector for the joints used in contact detection
_LEFT_KNEE_IDX  = 3
_LEFT_ANKLE_IDX = 4
_RIGHT_KNEE_IDX = 13
_RIGHT_ANKLE_IDX = 14

# Torque excess threshold (Nm) to declare contact on a foot
_TORQUE_CONTACT_THRESHOLD = 0.08

# Hysteresis: once in contact, need excess to drop below this to release
_TORQUE_RELEASE_THRESHOLD = 0.04

# Relative foot height margin (m): stance foot must be at least this much
# lower than swing foot in world frame (relative comparison, no absolute needed)
_HEIGHT_MARGIN = 0.005


class MicroduckOdometry:
    """
    Fused contact odometry. Call update() each control cycle, then read .position and .yaw.

    Args:
        urdf_path: path to robot.urdf (defaults to fk/robot.urdf)
        kt:        motor torque constant in Nm/A
        friction:  per-joint static friction floor in Nm
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF, kt: float = XL330_KT,
                 friction: float = XL330_FRICTION_NM):
        self._fk = MicroduckFK(urdf_path)
        self._kt = kt
        self._friction = friction

        # World-frame trunk position [x, y, z]
        self._pos = np.zeros(3)
        # Last IMU quaternion [w, x, y, z]
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])

        # Per-foot: world position of the foot when it last entered stance (anchor)
        self._anchor = {'left': None, 'right': None}
        # Per-foot: current contact state (bool)
        self._contact = {'left': False, 'right': False}

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def update(self, joint_angles, imu_quat, currents_ma):
        """
        Update odometry with new sensor data.

        Args:
            joint_angles: 15-element list/array (runtime motor order)
            imu_quat:     [w, x, y, z] orientation of trunk_base in world
            currents_ma:  15-element list/array of motor currents in mA
        """
        self._quat = np.asarray(imu_quat, dtype=float)
        self._fk.update(joint_angles)

        R = self._quat_to_rot(self._quat)

        # Foot positions in trunk frame
        pos_left  = self._fk.get_pose('left_foot')[:3, 3]
        pos_right = self._fk.get_pose('right_foot')[:3, 3]

        # Relative world-frame Z (same offset cancels out — valid for comparison)
        z_left  = R[2] @ pos_left
        z_right = R[2] @ pos_right

        # Gravity-compensation torques from model
        tau_grav = self._fk._robot.static_gravity_compensation_torques_dict('trunk_base')

        # Measured torques from currents
        tau_meas = np.array(currents_ma, dtype=float) / 1000.0 * self._kt

        # Torque excess per foot (ankle + knee combined)
        def excess(ankle_idx, knee_idx, ankle_name, knee_name):
            e_ankle = abs(tau_meas[ankle_idx]) - abs(tau_grav[ankle_name]) - self._friction
            e_knee  = abs(tau_meas[knee_idx])  - abs(tau_grav[knee_name])  - self._friction
            return max(0.0, e_ankle) + max(0.0, e_knee)

        excess_left  = excess(_LEFT_ANKLE_IDX,  _LEFT_KNEE_IDX,  'left_ankle',  'left_knee')
        excess_right = excess(_RIGHT_ANKLE_IDX, _RIGHT_KNEE_IDX, 'right_ankle', 'right_knee')

        # Height tiebreaker: favour the lower foot when torques are ambiguous
        height_vote_left  = z_left  < z_right - _HEIGHT_MARGIN
        height_vote_right = z_right < z_left  - _HEIGHT_MARGIN

        # Update contact state with hysteresis
        prev_left  = self._contact['left']
        prev_right = self._contact['right']

        threshold_on  = _TORQUE_CONTACT_THRESHOLD
        threshold_off = _TORQUE_RELEASE_THRESHOLD

        # Torque-based contact
        torque_left  = excess_left  > (threshold_off if prev_left  else threshold_on)
        torque_right = excess_right > (threshold_off if prev_right else threshold_on)

        # Fuse: torque OR (height AND at least some torque signal)
        new_left  = torque_left  or (height_vote_left  and excess_left  > threshold_off * 0.5)
        new_right = torque_right or (height_vote_right and excess_right > threshold_off * 0.5)

        # Save anchors on contact start
        if new_left and not prev_left:
            self._anchor['left'] = self._pos + R @ pos_left
        if new_right and not prev_right:
            self._anchor['right'] = self._pos + R @ pos_right

        self._contact['left']  = new_left
        self._contact['right'] = new_right

        # Position update from anchored feet
        estimates = []
        if new_left and self._anchor['left'] is not None:
            estimates.append(self._anchor['left'] - R @ pos_left)
        if new_right and self._anchor['right'] is not None:
            estimates.append(self._anchor['right'] - R @ pos_right)

        if estimates:
            self._pos = np.mean(estimates, axis=0)

    @property
    def position(self) -> np.ndarray:
        """World-frame position of trunk_base [x, y, z] in metres."""
        return self._pos.copy()

    @property
    def yaw(self) -> float:
        """Trunk yaw in world frame (radians), extracted from IMU quaternion."""
        w, x, y, z = self._quat
        return float(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))

    @property
    def contact(self) -> dict:
        """{'left': bool, 'right': bool} — current contact state."""
        return dict(self._contact)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_rot(q):
        """[w, x, y, z] → 3×3 rotation matrix."""
        w, x, y, z = q / np.linalg.norm(q)
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])
