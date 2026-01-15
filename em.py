from rustypot import Xl330PyController
import time

c = Xl330PyController("/dev/ttyACM0", baudrate=1000000, timeout=0.01)

joints = {
    "left_hip_yaw": 20,
    "left_hip_roll": 21,
    "left_hip_pitch": 22,
    "left_knee": 23,
    "left_ankle": 24,
    "neck_pitch": 30,
    "head_pitch": 31,
    "head_yaw": 32,
    "head_roll": 33,
    "right_hip_yaw": 10,
    "right_hip_roll": 11,
    "right_hip_pitch": 12,
    "right_knee": 13,
    "right_ankle": 14,
}

ids = [v for v in joints.values()]

c.sync_write_torque_enable(ids, [False for _ in joints.keys()])

time.sleep(2)