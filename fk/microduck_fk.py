"""
Forward kinematics for the microduck robot using Placo.

Usage:
    fk = MicroduckFK()
    fk.update(joint_angles)          # list of 15 angles (runtime motor order)
    T = fk.get_pose("head_camera")   # 4x4 SE3 matrix, in trunk_base frame
    poses = fk.get_all_poses()       # dict of all key frames

Joint angle list order matches the runtime motor vector:
    [0]  left_hip_yaw
    [1]  left_hip_roll
    [2]  left_hip_pitch
    [3]  left_knee
    [4]  left_ankle
    [5]  neck_pitch
    [6]  head_pitch
    [7]  head_yaw
    [8]  head_roll
    [9]  mouth_motor   (no kinematic effect, ignored)
    [10] right_hip_yaw
    [11] right_hip_roll
    [12] right_hip_pitch
    [13] right_knee
    [14] right_ankle
"""

import os
import numpy as np
import placo

# Mapping from runtime motor vector index to URDF joint name.
# Index 9 (mouth) has no joint in the kinematic chain.
MOTOR_INDEX_TO_JOINT = {
    0:  "left_hip_yaw",
    1:  "left_hip_roll",
    2:  "left_hip_pitch",
    3:  "left_knee",
    4:  "left_ankle",
    5:  "neck_pitch",
    6:  "head_pitch",
    7:  "head_yaw",
    8:  "head_roll",
    # 9: mouth — no joint
    10: "right_hip_yaw",
    11: "right_hip_roll",
    12: "right_hip_pitch",
    13: "right_knee",
    14: "right_ankle",
}

# Frames of interest exposed by get_all_poses()
KEY_FRAMES = [
    "head_camera",
    "torso_camera",
    "left_foot",
    "right_foot",
    "imu",
    "mouth_tip",
]

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF = os.path.join(_HERE, "robot.urdf")


class MicroduckFK:
    """
    Lightweight forward kinematics wrapper around Placo's RobotWrapper.

    The robot root (trunk_base) is kept at the world origin with identity
    orientation, so all returned poses are expressed in the trunk_base frame.
    This is the natural reference frame for the robot's own sensors and
    perception tasks.
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF):
        self._robot = placo.RobotWrapper(urdf_path, placo.Flags.ignore_collisions)
        # Fix the floating base at the origin so FK results are in trunk_base frame
        self._robot.set_T_world_fbase(np.eye(4))
        # Neutral pose as starting state
        for joint in MOTOR_INDEX_TO_JOINT.values():
            self._robot.set_joint(joint, 0.0)
        self._robot.update_kinematics()

    def update(self, joint_angles):
        """
        Update FK with new joint angles.

        Args:
            joint_angles: sequence of 15 floats in runtime motor order,
                          OR a dict mapping joint names to angles.
        """
        if isinstance(joint_angles, dict):
            for name, angle in joint_angles.items():
                self._robot.set_joint(name, float(angle))
        else:
            for idx, joint in MOTOR_INDEX_TO_JOINT.items():
                self._robot.set_joint(joint, float(joint_angles[idx]))
        self._robot.update_kinematics()

    def get_pose(self, frame: str) -> np.ndarray:
        """
        Return the 4x4 SE3 transform of `frame` in the trunk_base frame.

        Available frames include all URDF link/site names, e.g.:
            head_camera, torso_camera, left_foot, right_foot, imu, mouth_tip,
            trunk_base, head_base_plate, foot, foot_2, ...
        """
        return self._robot.get_T_world_frame(frame)

    def get_all_poses(self) -> dict:
        """Return poses of all key frames as a dict of {name: 4x4 ndarray}."""
        return {f: self._robot.get_T_world_frame(f) for f in KEY_FRAMES}

    @property
    def joint_names(self) -> list:
        return self._robot.joint_names()

    @property
    def frame_names(self) -> list:
        return self._robot.frame_names()
