from rustypot import Xl330PyController
import time
import math

def convert_speed(raw_count):
    """Convert raw velocity count to rad/s.

    Based on XL330 specs: 0.229 RPM per count.
    Conversion: RPM * (2π / 60) = rad/s
    """
    return raw_count * 0.229 * (2.0 * math.pi / 60.0)

c = Xl330PyController("/dev/ttyAMA0", baudrate=1000000, timeout=0.01)

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

init_pose = [
    0.0,
    0.0,
    0.6,
    -1.2,
    0.6,
    
    0.0,
    0.0,
    0.0,
    0.0,
    
    0.0,
    0.0,
    -0.6,
    1.2,
    -0.6
]
    

ids = [v for v in joints.values()]

c.sync_write_torque_enable(ids, [True for _ in joints.keys()])
c.sync_write_goal_position(ids, init_pose)


time.sleep(1)
exit()
while True:
    
    
    positions = c.sync_read_present_position(ids)
    speeds = c.sync_read_present_velocity(ids)
    for i, name in enumerate(joints.keys()):
        speed_rad_s = convert_speed(speeds[i])
        print(f"{name:20s} pos: {positions[i]:7.3f} rad  vel_raw: {speeds[i]:5.0f}  vel: {speed_rad_s:7.4f} rad/s")
    print("===")
        
      
    
    
    time.sleep(0.1)