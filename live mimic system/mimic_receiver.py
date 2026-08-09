"""
mimic_receiver.py  (v3)  —  the 12-joint humanoid animated by streamed angles
================================================================================
Run with mjpython in its own terminal:     mjpython mimic_receiver.py

v3: loads humanoid_v3.urdf (2-DOF shoulders and hips: flexion + abduction) and
drives all 12 joints from the 3D pose_sender. Targets are clamped to the URDF
joint limits; smoothing + rate limiting as before. Runs until you close the
viewer or Ctrl+C.
================================================================================
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = os.path.join(here, "humanoid_v3.urdf")

UDP_ADDR = ("127.0.0.1", 5005)
SIM_FPS = 60.0
SMOOTH_ALPHA = 0.35         # sender already One-Euro-filters; this is a light touch
MAX_RATE = 8.0              # rad/s cap on joint speed

JOINT_NAMES = ["L_shoulder_flex", "L_shoulder_abd", "L_elbow",
               "R_shoulder_flex", "R_shoulder_abd", "R_elbow",
               "L_hip_flex", "L_hip_abd", "L_knee",
               "R_hip_flex", "R_hip_abd", "R_knee"]


def main():
    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    qmap, limits = {}, {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"joint '{name}' not found — is humanoid_v3.urdf "
                               "in this folder?")
        qmap[name] = model.jnt_qposadr[jid]
        limits[name] = tuple(model.jnt_range[jid]) if model.jnt_limited[jid] \
            else (-3.2, 3.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in JOINT_NAMES}
    current = {n: 0.0 for n in JOINT_NAMES}
    dt = 1.0 / SIM_FPS
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}  (12-joint humanoid)")
    print("Run the camera side in another terminal:  python pose_sender.py\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        while viewer.is_running():
            while True:                            # keep only newest packets
                try:
                    packet, _ = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except Exception:
                    break
                try:
                    ang = json.loads(packet.decode())
                except Exception:
                    continue
                for name, val in ang.items():
                    if name in target:
                        lo, hi = limits[name]
                        target[name] = max(lo, min(hi, float(val)))
                if not connected:
                    print("Receiving 3D pose data — mirroring.")
                    connected = True

            for name in JOINT_NAMES:               # EMA + rate limiter
                step = SMOOTH_ALPHA * (target[name] - current[name])
                step = max(-max_step, min(max_step, step))
                current[name] += step
                data.qpos[qmap[name]] = current[name]

            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(dt)

    print("\nStopped.")


if __name__ == "__main__":
    main()
