from standalone_placo_walk_generator import StandalonePlacoWalkGenerator
from rustypot import Xl330PyController
import time

wg = StandalonePlacoWalkGenerator("assets/microduck/", model_filename="microduck.urdf", ignore_feet_contact=True)
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

dt = (1./50)
while True:
    s = time.time()
    
    wg.tick(dt)
    angles = wg.get_angles()
    
    c.sync_write_goal_positions(ids, angles.values())
   
    took = time.time() - s
    time.sleep(max(0, dt - took))