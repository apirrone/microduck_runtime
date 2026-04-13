"""
Digital twin viewer for microduck.

Connects to the runtime's TCP stream (--stream --stream-port PORT),
receives 36 × f32 LE per frame:
  [0..3]   IMU quaternion [w, x, y, z]
  [4..18]  joint positions (runtime motor order)
  [19..33] motor currents in mA
  [34..35] odometry x, y in metres (from Rust onboard odometry)

Usage:
    uv run viewer.py --host 192.168.x.x --port 9870
"""

import argparse
import math
import os
import socket
import struct
import sys
import numpy as np
import mujoco
import mujoco.viewer

_HERE = os.path.dirname(os.path.abspath(__file__))

PACKET_FLOATS = 36          # 4 quat + 15 joints + 15 currents + 2 odometry (x, y)
PACKET_BYTES  = PACKET_FLOATS * 4


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
    """Return {geom_id: Nx3 vertex array} for the two foot collision geoms."""
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
        pos = data.geom_xpos[gid]
        mat = data.geom_xmat[gid].reshape(3, 3)
        z = pos[2] + mat[2, :] @ verts.T
        if z.min() < min_z:
            min_z = float(z.min())
    return min_z


# ── Reference square overlay ──────────────────────────────────────────────────

_SQUARE_RGBA  = np.array([1.0, 0.85, 0.0, 0.85], np.float32)
_SQUARE_Z     = 0.008
_IDENTITY_MAT = np.eye(3, dtype=np.float64).flatten()


def _rot_toward(dx: float, dy: float) -> np.ndarray:
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return np.eye(3, dtype=np.float64)
    ux, uy = dx / length, dy / length
    lx = np.array([-uy, ux, 0.0])
    lz = np.array([ux, uy, 0.0])
    ly = np.cross(lz, lx)
    return np.column_stack([lx, ly, lz]).astype(np.float64)


def _draw_reference_square(scn, cx: float = 0.0, cy: float = 0.0, half: float = 0.5):
    """Draw a 1 m × 1 m yellow square centred at (cx, cy) at ground level."""
    scn.ngeom = 0
    corners = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        mx, my = (x0 + x1) * 0.5, (y0 + y1) * 0.5
        seg_half = math.hypot(x1 - x0, y1 - y0) * 0.5
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.array([0.006, seg_half, 0.0], np.float64),
            np.array([mx, my, _SQUARE_Z], np.float64),
            _rot_toward(x1 - x0, y1 - y0).flatten(),
            _SQUARE_RGBA,
        )
        scn.ngeom += 1
    for (cx_, cy_) in corners:
        if scn.ngeom >= scn.maxgeom:
            break
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(
            g, mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.015, 0.015, 0.015], np.float64),
            np.array([cx_, cy_, _SQUARE_Z], np.float64),
            _IDENTITY_MAT,
            _SQUARE_RGBA,
        )
        scn.ngeom += 1


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Microduck digital twin viewer")
    parser.add_argument("--host", default="localhost", help="Runtime host (default: localhost)")
    parser.add_argument("--port", type=int, default=9870, help="Stream port (default: 9870)")
    parser.add_argument(
        "--mjcf",
        default=os.path.join(_HERE, "scene.xml"),
        help="Path to MuJoCo scene XML (default: fk/scene.xml)",
    )
    args = parser.parse_args()

    try:
        model = mujoco.MjModel.from_xml_path(args.mjcf)
    except Exception as e:
        print(f"Failed to load MJCF: {e}")
        sys.exit(1)

    data = mujoco.MjData(model)
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

    freejoint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    if freejoint_id < 0:
        print("Warning: freejoint 'trunk_base_freejoint' not found")
        freejoint_qpos_adr = None
    else:
        freejoint_qpos_adr = model.jnt_qposadr[freejoint_id]

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
            joints = floats[4:19]
            odo_x  = floats[34]
            odo_y  = floats[35]

            # Set joint angles
            for i, name in enumerate(JOINT_NAMES):
                if name is None:
                    continue
                if name in joint_qpos:
                    data.qpos[joint_qpos[name]] = joints[i]

            # Set floating base: X/Y from onboard odometry, orientation from IMU
            if freejoint_qpos_adr is not None:
                data.qpos[freejoint_qpos_adr + 0] = odo_x
                data.qpos[freejoint_qpos_adr + 1] = odo_y
                data.qpos[freejoint_qpos_adr + 2] = 0.0   # first pass
                data.qpos[freejoint_qpos_adr + 3] = qw
                data.qpos[freejoint_qpos_adr + 4] = qx
                data.qpos[freejoint_qpos_adr + 5] = qy
                data.qpos[freejoint_qpos_adr + 6] = qz

                if foot_verts:
                    # Two-pass Z: find how high to lift trunk so lowest foot sits at z=0
                    mujoco.mj_kinematics(model, data)
                    min_z = _min_foot_world_z(data, foot_verts)
                    data.qpos[freejoint_qpos_adr + 2] = -min_z

            mujoco.mj_kinematics(model, data)
            _draw_reference_square(v.user_scn)
            v.sync()

    sock.close()


if __name__ == "__main__":
    main()
