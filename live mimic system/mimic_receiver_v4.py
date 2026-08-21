"""
mimic_receiver_v4.py  —  drop-in replacement for mimic_receiver.py, with the
real ATOM arms and actual physics.

Run exactly like the old one:
    mjpython mimic_receiver_v4.py          (macOS)
    python  mimic_receiver_v4.py           (Windows/Linux)
    python pose_sender.py                  (other terminal, unchanged)

Same UDP port, same JSON packets, same 12 joint names. pose_sender.py does not
change.

WHAT IS DIFFERENT FROM v3
-------------------------
v3 did this:
    data.qpos[adr] = angle
    mujoco.mj_forward(model, data)

That is not simulation. mj_forward only recomputes forward kinematics, so the
robot teleports into whatever pose it is told, at any speed, through anything
in the way. Contact is never evaluated. That is where the clipping came from.

v4 does this:
    data.ctrl[act] = angle
    mujoco.mj_step(model, data)

Now the pose is a *target*. Servos have to drive real inertia to reach it,
torque saturates at the joint's limit, and the forearm stops when it hits the
bicep. If the sim cannot reach a pose, that is information: the hardware
probably cannot either.

The scene is generated - do not edit it by hand:
    python sim/build_humanoid_scene.py
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENE = os.path.join(REPO_ROOT, "sim", "humanoid_v4_scene.xml")

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35     # sender already One-Euro-filters; light touch here
MAX_RATE = 8.0          # rad/s cap on commanded target motion

JOINT_NAMES = ["L_shoulder_flex", "L_shoulder_abd", "L_elbow",
               "R_shoulder_flex", "R_shoulder_abd", "R_elbow",
               "L_hip_flex", "L_hip_abd", "L_knee",
               "R_hip_flex", "R_hip_abd", "R_knee"]


def main():
    if not os.path.exists(SCENE):
        raise SystemExit(
            f"scene not found: {SCENE}\nrun:  python sim/build_humanoid_scene.py")

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    act, limits = {}, {}
    for name in JOINT_NAMES:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if aid < 0 or jid < 0:
            raise RuntimeError(f"joint/actuator '{name}' missing from the scene")
        act[name] = aid
        limits[name] = tuple(model.jnt_range[jid]) if model.jnt_limited[jid] \
            else (-3.2, 3.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in JOINT_NAMES}
    cmd = {n: 0.0 for n in JOINT_NAMES}

    dt = model.opt.timestep
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Physics ON: position actuators + mj_step. Elbow limit 5 Nm, "
          "travel 0-120 deg (contact-limited at ~115).")
    print("Run the camera side in another terminal:  python pose_sender.py\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        wall0 = time.time()
        while viewer.is_running():
            while True:                       # drain to the newest packet
                try:
                    packet, _ = sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                try:
                    ang = json.loads(packet.decode())
                except Exception:
                    continue
                for name, val in ang.items():
                    if name in target:
                        lo, hi = limits[name]
                        # Clamping to the URDF range is what keeps the elbow out
                        # of the region the real arm cannot reach.
                        target[name] = max(lo, min(hi, float(val)))
                if not connected:
                    print("Receiving pose data - mirroring.")
                    connected = True

            # Slew-limit the TARGET, not the state. The servo still has to get
            # there on its own, and may fail to.
            for name in JOINT_NAMES:
                blended = cmd[name] + SMOOTH_ALPHA * (target[name] - cmd[name])
                delta = max(-max_step, min(max_step, blended - cmd[name]))
                cmd[name] += delta
                data.ctrl[act[name]] = cmd[name]

            mujoco.mj_step(model, data)
            viewer.sync()

            # keep sim time roughly in step with wall time
            lag = (data.time) - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()
