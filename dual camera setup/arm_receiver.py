"""
arm_receiver.py  —  the physical-arm simulation driven by your arm
================================================================================
Run with mjpython:     mjpython arm_receiver.py
Pair with either sender (single or dual camera) — it consumes the same UDP
stream as the humanoid, but maps just ONE human arm onto the robot arm:

  human {SIDE}_shoulder_flex  ->  shoulder_flex   (clamped to +/-90 deg)
  human {SIDE}_shoulder_abd   ->  shoulder_abd    (clamped to +/-90 deg)
  human {SIDE}_elbow          ->  elbow           (0..150 deg)

Set ARM_SIDE below to choose which of your arms controls it. The +/-90 deg
clamps mirror the physical arm's 180-degree travel per axis, so the sim never
commands a pose the real hardware can't reach.
================================================================================
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = os.path.join(here, "arm_sim.urdf")

ARM_SIDE = "R"              # "R": your right arm drives it; "L": your left
UDP_ADDR = ("127.0.0.1", 5005)
SIM_FPS = 60.0
SMOOTH_ALPHA = 0.35
MAX_RATE = 6.0              # rad/s — keep conservative: this maps to hardware

# robot joint <- key in the incoming packet
MAPPING = {
    "shoulder_flex": f"{ARM_SIDE}_shoulder_flex",
    "shoulder_abd":  f"{ARM_SIDE}_shoulder_abd",
    "elbow":         f"{ARM_SIDE}_elbow",
}


def main():
    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    qmap, limits = {}, {}
    for name in MAPPING:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"joint '{name}' not in arm_sim.urdf")
        qmap[name] = model.jnt_qposadr[jid]
        limits[name] = tuple(model.jnt_range[jid])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in MAPPING}
    current = {n: 0.0 for n in MAPPING}
    dt = 1.0 / SIM_FPS
    max_step = MAX_RATE * dt

    print(f"Arm sim: your {'RIGHT' if ARM_SIDE == 'R' else 'LEFT'} arm is the controller.")
    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]} — start a pose sender.\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        while viewer.is_running():
            while True:
                try:
                    packet, _ = sock.recvfrom(4096)
                except (BlockingIOError, Exception):
                    break
                try:
                    ang = json.loads(packet.decode())
                except Exception:
                    continue
                for rjoint, key in MAPPING.items():
                    if key in ang:
                        lo, hi = limits[rjoint]
                        target[rjoint] = max(lo, min(hi, float(ang[key])))
                if not connected:
                    print("Receiving pose data — arm live.")
                    connected = True

            for name in MAPPING:
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
