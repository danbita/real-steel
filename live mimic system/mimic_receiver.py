"""
mimic_receiver.py  —  MuJoCo humanoid animated by streamed joint angles
================================================================================
Process 2 of 2. Run this with mjpython in its own terminal:

    mjpython mimic_receiver.py

It listens on UDP for the joint-angle packets produced by pose_sender.py,
applies smoothing + a rate limiter (so single bad tracking frames can't make
the figure flail), and drives the humanoid. Start pose_sender.py first or
second — order doesn't matter, the receiver just waits for packets.

Close the viewer window or Ctrl+C to stop.
================================================================================
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = os.path.join(here, "humanoid.urdf")

UDP_ADDR = ("127.0.0.1", 5005)
SIM_FPS = 60.0
SMOOTH_ALPHA = 0.25         # per-tick EMA: lower = smoother, higher = snappier
MAX_RATE = 6.0              # rad/s cap on how fast any joint may move
                            # (a real fast human joint swing is ~5-10 rad/s)

JOINT_NAMES = ["L_hip", "L_knee", "R_hip", "R_knee",
               "L_shoulder", "L_elbow", "R_shoulder", "R_elbow"]


def main():
    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    def q(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid]
    qmap = {n: q(n) for n in JOINT_NAMES}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in JOINT_NAMES}     # latest angles from the sender
    current = {n: 0.0 for n in JOINT_NAMES}    # what the sim actually shows
    last_packet = 0.0
    dt = 1.0 / SIM_FPS
    max_step = MAX_RATE * dt                   # biggest allowed move per tick

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Run the camera side in another terminal:  python pose_sender.py\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        while viewer.is_running():
            # drain everything queued, keep only the newest packet
            while True:
                try:
                    packet, _ = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                ang = json.loads(packet.decode())
                target.update(ang)             # occluded joints simply absent
                last_packet = time.time()

            if last_packet and not connected:
                print("Receiving pose data — mirroring.")
                connected = True

            # move current toward target: EMA capped by the rate limiter
            for name in JOINT_NAMES:
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
