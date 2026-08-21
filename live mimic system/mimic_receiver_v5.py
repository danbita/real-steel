"""
mimic_receiver_v5.py  —  drives the real shoulder module + real ATOM arm.

    python sim/build_humanoid_v5_scene.py      (once, after any CAD change)
    python mimic_receiver_v5.py
    python pose_sender.py                      (other terminal, unchanged)

Same UDP port and same JSON packets as v3/v4. pose_sender.py does not change.

WHAT CHANGED FROM v4
====================
v4 still had the invented 2-hinge shoulder: flexion (Y) + abduction (X).
The real hardware is PITCH (Y) + YAW (Z). There is no abduction axis.

  incoming key          driven joint            fidelity
  --------------------  ----------------------  --------------------------
  *_shoulder_flex       *_shoulder_pitch        exact - same axis and sign
  *_shoulder_abd        *_shoulder_yaw          APPROXIMATION, see below
  *_elbow               *_elbow                 exact
  legs                  unchanged               exact

ABDUCTION IS NOT PHYSICALLY AVAILABLE. Raising the arm out to the side is a
rotation about the fore-aft axis; this shoulder can only rotate about vertical.
Feeding abduction into yaw does not reproduce the motion - it turns "arm lifts
sideways" into "arm twists about vertical", which is a different pose.

It is mapped anyway, heavily scaled, so the DOF is exercised and visible rather
than silently dead. ABD_TO_YAW is the honesty dial:
    0.0  discard abduction entirely (most truthful about the hardware)
    0.1  default - a hint of yaw tracks the operator's arm raise
    1.0  full pass-through; saturates instantly against the +/-10 deg limit

The +/-10 deg yaw limit is not a servo limit - the servo does +/-90. It is where
the bicep starts hitting the torso, measured by sweeping the model. Pitch and yaw
are coupled: +yaw clashes at forward pitch, -yaw at backward pitch.
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENE = os.path.join(REPO_ROOT, "sim", "humanoid_v5_scene.xml")

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35
MAX_RATE = 8.0          # rad/s cap on how fast the target may move

ABD_TO_YAW = 0.10       # see module docstring

JOINT_MAP = {
    "L_shoulder_flex": ("L_shoulder_pitch", 1.0),
    "R_shoulder_flex": ("R_shoulder_pitch", 1.0),
    "L_shoulder_abd":  ("L_shoulder_yaw",  ABD_TO_YAW),
    "R_shoulder_abd":  ("R_shoulder_yaw",  ABD_TO_YAW),
    "L_elbow": ("L_elbow", 1.0),
    "R_elbow": ("R_elbow", 1.0),
    "L_hip_flex": ("L_hip_flex", 1.0), "L_hip_abd": ("L_hip_abd", 1.0),
    "L_knee": ("L_knee", 1.0),
    "R_hip_flex": ("R_hip_flex", 1.0), "R_hip_abd": ("R_hip_abd", 1.0),
    "R_knee": ("R_knee", 1.0),
}


def main():
    if not os.path.exists(SCENE):
        raise SystemExit(
            f"scene not found: {SCENE}\nrun:  python sim/build_humanoid_v5_scene.py")

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    joints = sorted({t for t, _ in JOINT_MAP.values()})
    act, limits = {}, {}
    for name in joints:
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

    target = {n: 0.0 for n in joints}
    cmd = {n: 0.0 for n in joints}

    dt = model.opt.timestep
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Physics ON: position actuators + mj_step.")
    print("Shoulder: pitch -70..+105 deg, yaw +/-10 deg (torso-limited, not servo).")
    print(f"Abduction -> yaw at {ABD_TO_YAW:.2f}; this shoulder cannot abduct.")
    print("Camera side, other terminal:  python pose_sender.py\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        wall0 = time.time()
        while viewer.is_running():
            while True:
                try:
                    packet, _ = sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                try:
                    ang = json.loads(packet.decode())
                except Exception:
                    continue
                for key, val in ang.items():
                    if key not in JOINT_MAP:
                        continue
                    name, scale = JOINT_MAP[key]
                    lo, hi = limits[name]
                    target[name] = max(lo, min(hi, float(val) * scale))
                if not connected:
                    print("Receiving pose data - mirroring.")
                    connected = True

            for name in joints:
                blended = cmd[name] + SMOOTH_ALPHA * (target[name] - cmd[name])
                delta = max(-max_step, min(max_step, blended - cmd[name]))
                cmd[name] += delta
                data.ctrl[act[name]] = cmd[name]

            mujoco.mj_step(model, data)
            viewer.sync()

            lag = data.time - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()
