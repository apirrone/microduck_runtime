#!/usr/bin/env python3
"""
Get knee data from real robot with sine wave motion.
Records [timestamp, position, velocity, last_action] data.
"""

from rustypot import Xl330PyController
import time
import numpy as np
import pickle
import argparse


def convert_speed(raw_count):
    """Convert raw velocity count to rad/s.

    Based on XL330 specs: 0.229 RPM per count.
    Conversion: RPM * (2π / 60) = rad/s
    """
    return raw_count * 0.229 * (2.0 * np.pi / 60.0)


def main():
    parser = argparse.ArgumentParser(description="Record knee sine wave data from real robot")
    parser.add_argument("-f", "--frequency", type=float, default=1.0,
                       help="Sine wave frequency in Hz (default: 1.0)")
    parser.add_argument("-a", "--amplitude", type=float, default=1.0,
                       help="Sine wave amplitude in radians (default: 1.0)")
    parser.add_argument("-d", "--duration", type=float, default=3.0,
                       help="Recording duration in seconds (default: 3.0)")
    parser.add_argument("-o", "--output", type=str, default="knee_sin.pkl",
                       help="Output pickle file (default: knee_sin.pkl)")
    parser.add_argument("--kp", type=int, default=200,
                       help="Position P gain (default: 200)")
    args = parser.parse_args()

    print(f"Connecting to robot...")
    c = Xl330PyController("/dev/ttyACM0", baudrate=1000000, timeout=0.01)

    print(f"Initializing robot...")
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
        0.0, 0.0, 0.6, -1.2, 0.6,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -0.6, 1.2, -0.6
    ]

    ids = [v for v in joints.values()]

    # Set PID gains
    kp = args.kp
    print(f"Setting PID gains (kP={kp})...")
    c.sync_write_position_p_gain(ids, [kp for _ in ids])

    c.sync_write_torque_enable(ids, [True for _ in joints.keys()])
    c.sync_write_goal_position(ids, init_pose)

    left_ankle_id = joints["left_ankle"]
    left_hip_pitch_id = joints["left_hip_pitch"]
    left_knee_id = joints["left_knee"]

    print("Setting knee, hip_pitch, and ankle to 0...")
    c.sync_write_goal_position([left_knee_id, left_hip_pitch_id, left_ankle_id], [0, 0, 0])

    # Sine wave parameters from CLI
    amplitude = args.amplitude
    frequency = args.frequency
    duration = args.duration

    print(f"Recording knee data for {duration}s at 50Hz...")
    print(f"Sine wave: amplitude={amplitude} rad, frequency={frequency} Hz")

    data = []
    last_action = 0

    t0 = time.time()
    while True:
        t = time.time() - t0
        if t > duration:
            break

        # Read position and velocity
        pos = c.sync_read_present_position([left_knee_id])[0]
        vel_raw = c.sync_read_present_velocity([left_knee_id])[0]
        vel = convert_speed(vel_raw)

        # Record [timestamp, position, velocity, last_action]
        data.append([t, pos, vel, last_action])

        action = amplitude * np.sin(2 * np.pi * frequency * t)

        c.sync_write_goal_position([left_knee_id], [action])

        last_action = action

        time.sleep(1.0 / 50)  # 50Hz control loop

    print(f"Recording complete! Collected {len(data)} samples")

    # Calculate actual control frequency
    if len(data) > 1:
        actual_duration = data[-1][0] - data[0][0]
        actual_freq = (len(data) - 1) / actual_duration
        print(f"Actual duration: {actual_duration:.3f}s")
        print(f"Actual control frequency: {actual_freq:.2f} Hz (target: 50 Hz)")

    print(f"Saving data to {args.output}...")
    with open(args.output, 'wb') as f:
        pickle.dump(data, f)

    print("Done!")


if __name__ == "__main__":
    main()
