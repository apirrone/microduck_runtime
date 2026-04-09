"""
Digital twin viewer for microduck.

Connects to the runtime's TCP stream (--stream --stream-port PORT),
receives [qw, qx, qy, qz, j0..j14] at 50 Hz, and renders the robot
live in a MuJoCo passive viewer.

Usage:
    uv run viewer.py --host 192.168.x.x --port 9870
"""

import argparse
import os
import socket
import struct
import sys
import numpy as np
import mujoco
import mujoco.viewer

_HERE = os.path.dirname(os.path.abspath(__file__))

PACKET_FLOATS = 34          # 4 quat + 15 joints + 15 currents
PACKET_BYTES  = PACKET_FLOATS * 4


def quat_to_mat(qw, qx, qy, qz):
    """Quaternion [w, x, y, z] → 3x3 rotation matrix."""
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-6:
        return np.eye(3)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


# Motor index → MuJoCo joint name (runtime motor order)
JOINT_NAMES = [
    "left_hip_yaw",    # 0
    "left_hip_roll",   # 1
    "left_hip_pitch",  # 2
    "left_knee",       # 3
    "left_ankle",      # 4
    "neck_pitch",      # 5
    "head_pitch",      # 6
    "head_yaw",        # 7
    "head_roll",       # 8
    None,              # 9  mouth — no joint
    "right_hip_yaw",   # 10
    "right_hip_roll",  # 11
    "right_hip_pitch", # 12
    "right_knee",      # 13
    "right_ankle",     # 14
]


def recv_exact(sock, n):
    """Read exactly n bytes from sock, return None on disconnect."""
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
    print(f"Connecting to {host}:{port} ...", flush=True)
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((host, port))
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected.", flush=True)
            return s
        except (ConnectionRefusedError, OSError) as e:
            print(f"  {e}, retrying in 1s ...", flush=True)
            import time; time.sleep(1)


def _load_foot_verts(model):
    """Return {geom_id: Nx3 vertex array} for the two foot collision geoms.

    Vertices are in each geom's local frame; use data.geom_xpos / data.geom_xmat
    to transform them to world frame after mj_kinematics.
    """
    result = {}
    for name in ("left_foot_collision", "right_foot_collision"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if gid < 0:
            print(f"Warning: geom '{name}' not found — Z correction disabled")
            continue
        mid = model.geom_dataid[gid]
        adr = model.mesh_vertadr[mid]
        nvert = model.mesh_vertnum[mid]
        verts = model.mesh_vert[adr : adr + nvert].copy()
        result[gid] = verts
    return result


def _min_foot_world_z(data, foot_verts):
    """Minimum world-Z across all foot collision mesh vertices."""
    min_z = np.inf
    for gid, verts in foot_verts.items():
        pos = data.geom_xpos[gid]          # (3,) geom centre in world
        mat = data.geom_xmat[gid].reshape(3, 3)  # geom→world rotation (row-major)
        # z_world = pos[2] + row2(mat) · v  for each vertex v
        z = pos[2] + mat[2, :] @ verts.T  # (nvert,)
        if z.min() < min_z:
            min_z = float(z.min())
    return min_z


def main():
    parser = argparse.ArgumentParser(description="Microduck digital twin viewer")
    parser.add_argument("--host", default="localhost", help="Runtime host (default: localhost)")
    parser.add_argument("--port", type=int, default=9870, help="Stream port (default: 9870)")
    parser.add_argument(
        "--mjcf",
        default=os.path.join(_HERE, "scene.xml"),
        help="Path to MuJoCo scene XML (default: fk/scene.xml)",
    )
    parser.add_argument(
        "--odometry", action="store_true",
        help="Compute odometry locally from the stream data (uses Placo FK)",
    )
    args = parser.parse_args()

    # Load model (scene XML for nicer visuals with floor, lighting, etc.)
    try:
        model = mujoco.MjModel.from_xml_path(args.mjcf)
    except Exception as e:
        print(f"Failed to load MJCF: {e}")
        sys.exit(1)

    data = mujoco.MjData(model)

    # Load foot mesh vertices once — used each frame to correct trunk Z via MuJoCo FK
    foot_verts = _load_foot_verts(model)

    # Build joint name → qpos index map
    joint_qpos = {}
    for name in JOINT_NAMES:
        if name is None:
            continue
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            print(f"Warning: joint '{name}' not found in model, skipping")
            continue
        joint_qpos[name] = model.jnt_qposadr[jid]

    # Find the freejoint qpos address for the floating base
    freejoint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    if freejoint_id < 0:
        print("Warning: freejoint 'trunk_base_freejoint' not found, base orientation won't be set")
        freejoint_qpos_adr = None
    else:
        freejoint_qpos_adr = model.jnt_qposadr[freejoint_id]

    # Local odometry (optional) — runs Placo FK+gravity on the received stream data
    odo = None
    if args.odometry:
        from odometry import MicroduckOdometry
        odo = MicroduckOdometry()
        print("Local odometry enabled", flush=True)

    # Seed position from the model's default qpos so the robot appears at the
    # correct height before odometry kicks in.
    if freejoint_qpos_adr is not None:
        odo_pos = list(model.qpos0[freejoint_qpos_adr:freejoint_qpos_adr + 3])
    else:
        odo_pos = [0.0, 0.0, 0.0]

    sock = connect(args.host, args.port)

    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            raw = recv_exact(sock, PACKET_BYTES)
            if raw is None:
                print("Disconnected, reconnecting ...", flush=True)
                sock.close()
                sock = connect(args.host, args.port)
                continue

            floats = struct.unpack_from(f"<{PACKET_FLOATS}f", raw)
            qw, qx, qy, qz = floats[0], floats[1], floats[2], floats[3]
            joints   = floats[4:19]   # 15 joint angles
            currents = floats[19:34]  # 15 motor currents (mA)

            # Update local odometry if enabled (X/Y tracking only — Z comes from MuJoCo)
            if odo is not None:
                odo.update(joints, (qw, qx, qy, qz), currents)
                p = odo.position
                odo_pos[0] = p[0]
                odo_pos[1] = p[1]
                # odo_pos[2] intentionally not updated — computed below via MuJoCo FK

            # Set joint angles
            for i, name in enumerate(JOINT_NAMES):
                if name is None:
                    continue
                if name in joint_qpos:
                    data.qpos[joint_qpos[name]] = joints[i]

            # Set floating base: orientation from IMU, X/Y from odometry, Z=0 first pass
            if freejoint_qpos_adr is not None:
                # freejoint qpos layout: [x, y, z, qw, qx, qy, qz]
                data.qpos[freejoint_qpos_adr + 0] = odo_pos[0]
                data.qpos[freejoint_qpos_adr + 1] = odo_pos[1]
                data.qpos[freejoint_qpos_adr + 2] = 0.0  # first pass: temp Z
                data.qpos[freejoint_qpos_adr + 3] = qw
                data.qpos[freejoint_qpos_adr + 4] = qx
                data.qpos[freejoint_qpos_adr + 5] = qy
                data.qpos[freejoint_qpos_adr + 6] = qz

                if foot_verts:
                    # Pass 1: kinematics at Z=0 to find foot sole world height
                    mujoco.mj_kinematics(model, data)
                    min_z = _min_foot_world_z(data, foot_verts)
                    # Pass 2: lift trunk so lowest foot vertex is exactly at ground (z=0)
                    data.qpos[freejoint_qpos_adr + 2] = -min_z

            # Final forward kinematics (no physics)
            mujoco.mj_kinematics(model, data)
            v.sync()

    sock.close()


if __name__ == "__main__":
    main()
