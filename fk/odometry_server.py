"""
Odometry sidecar — runs on the Pi alongside microduck_runtime.

Reads the TCP stream (joints + IMU + currents), computes contact-based odometry
using Placo FK and gravity-compensation torques, then writes [x, y, z, yaw] as
4×f32 LE (16 bytes) to a Unix datagram socket that the Rust runtime reads.

Usage:
    uv run odometry_server.py [--host HOST] [--port PORT] [--socket PATH]

Defaults match the runtime defaults:
    --host   localhost
    --port   9870
    --socket /tmp/microduck_odometry.sock
"""

import argparse
import socket
import struct
import time
import os
import numpy as np
from odometry import MicroduckOdometry

PACKET_FLOATS = 34   # 4 quat + 15 joints + 15 currents
PACKET_BYTES  = PACKET_FLOATS * 4


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (ConnectionResetError, OSError):
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def connect(host, port):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((host, port))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"Connected to stream at {host}:{port}", flush=True)
            return s
        except (ConnectionRefusedError, OSError) as e:
            print(f"  {e}, retrying in 1s ...", flush=True)
            time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description="Microduck odometry sidecar")
    parser.add_argument("--host",   default="localhost")
    parser.add_argument("--port",   type=int, default=9870)
    parser.add_argument("--socket", default="/tmp/microduck_odometry.sock")
    parser.add_argument("--kt",     type=float, default=0.625,
                        help="XL330 torque constant Nm/A (default: 0.625)")
    args = parser.parse_args()

    odo = MicroduckOdometry(kt=args.kt)

    # Unix datagram socket to send results to Rust runtime
    tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    tcp = connect(args.host, args.port)

    print("Odometry running. Sending to", args.socket, flush=True)

    while True:
        raw = recv_exact(tcp, PACKET_BYTES)
        if raw is None:
            print("Stream disconnected, reconnecting ...", flush=True)
            tcp.close()
            tcp = connect(args.host, args.port)
            continue

        floats = struct.unpack_from(f"<{PACKET_FLOATS}f", raw)
        imu_quat  = floats[0:4]
        joints    = floats[4:19]
        currents  = floats[19:34]

        odo.update(joints, imu_quat, currents)

        pos = odo.position
        yaw = odo.yaw

        packet = struct.pack("<4f", pos[0], pos[1], pos[2], yaw)
        try:
            tx.sendto(packet, args.socket)
        except (FileNotFoundError, OSError):
            pass  # Rust runtime not yet listening, or restarting — keep going


if __name__ == "__main__":
    main()
