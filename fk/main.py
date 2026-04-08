"""Demo: print key frame poses at default robot pose and at a walking-like pose."""

import numpy as np
from microduck_fk import MicroduckFK

# Default pose from the runtime (motor.rs DEFAULT_POSITION)
DEFAULT_JOINTS = [
    0.0,   # left_hip_yaw
    0.0,   # left_hip_roll
    0.6,   # left_hip_pitch
    -1.2,  # left_knee
    0.6,   # left_ankle
    -0.5,  # neck_pitch
    0.5,   # head_pitch
    0.0,   # head_yaw
    0.0,   # head_roll
    0.0,   # mouth (ignored)
    0.0,   # right_hip_yaw
    0.0,   # right_hip_roll
    -0.6,  # right_hip_pitch
    1.2,   # right_knee
    -0.6,  # right_ankle
]


def print_poses(poses: dict):
    for name, T in poses.items():
        pos = T[:3, 3]
        print(f"  {name:20s}  pos=[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]")


fk = MicroduckFK()

print("=== Default pose ===")
fk.update(DEFAULT_JOINTS)
print_poses(fk.get_all_poses())

print("\n=== Head turned 45 deg left ===")
joints = DEFAULT_JOINTS.copy()
joints[7] = np.pi / 4  # head_yaw
fk.update(joints)
print_poses(fk.get_all_poses())

print("\n=== Head camera pose (full 4x4) ===")
T = fk.get_pose("head_camera")
np.set_printoptions(precision=4, suppress=True)
print(T)
