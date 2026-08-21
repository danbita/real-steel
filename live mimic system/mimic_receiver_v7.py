"""
mimic_receiver_v7.py  —  drives the real shoulder module + real ATOM arm.

    python sim/build_humanoid_v7_scene.py      (once, after any CAD change)
    python mimic_receiver_v7.py
    python pose_sender.py                      (other terminal, unchanged)

Same UDP port and same JSON packets as v3/v4. pose_sender.py does not change.

WHAT CHANGED FROM v6
====================
shoulder_v3: real bearings on pitch and twist, and a genuinely mirrored
right-hand module, so both arms now share sign conventions. Joint names, the
abduction mapping and the rest offset are all unchanged from v6.

WHAT CHANGED FROM v5
====================
v5 drove shoulder_v1: pitch + yaw, no abduction axis. Raising the arm sideways
was impossible, so `*_shoulder_abd` was scaled down into yaw as a stand-in.

shoulder_v2 has a real abduction axis, so that fudge is gone:

  incoming key          driven joint            fidelity
  --------------------  ----------------------  --------------------------
  *_shoulder_flex       *_shoulder_pitch        exact - same axis and sign
  *_shoulder_abd        *_shoulder_roll         REAL now, but offset (below)
  (none)                *_shoulder_twist        held at 0; the sender has no
                                                internal-rotation estimate
  *_elbow               *_elbow                 exact
  legs                  unchanged               exact

ROLL IS OFFSET, NOT DIRECT. The CAD neutral is arm-straight-up, so roll 0 means
"arm up" and roll 3.1416 means "arm hanging". Abduction as the retargeting
means it (0 = hanging, positive = lifting away from the body) is therefore

    roll = REST_ROLL - abduction

Feeding raw abduction into roll would slam the arm to vertical. The mapping
below applies the offset.

ROLL AND TWIST ARE COUPLED, and no box limit can express it. Measured:
    roll 0..160 -> twist +/-90 free
    roll 170    -> twist +/-70
    roll 180    -> twist +/-25   (hanging + bent elbow sweeps across the chest)
Contact geometry is modelled, so a bad combination is refused by the sim rather
than hidden. Twist is parked at 0 here, which is always safe.
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENE = os.path.join(REPO_ROOT, "sim", "humanoid_v7_scene.xml")

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35
MAX_RATE = 8.0          # rad/s cap on how fast the target may move

REST_ROLL = 3.1416      # roll at which the arm hangs; abduction subtracts

# (joint, scale, offset): commanded = offset + scale * incoming
JOINT_MAP = {
    "L_shoulder_flex": ("L_shoulder_pitch", 1.0, 0.0),
    "R_shoulder_flex": ("R_shoulder_pitch", 1.0, 0.0),
    "L_shoulder_abd":  ("L_shoulder_roll", -1.0, REST_ROLL),
    "R_shoulder_abd":  ("R_shoulder_roll", -1.0, REST_ROLL),
    "L_elbow": ("L_elbow", 1.0, 0.0),
    "R_elbow": ("R_elbow", 1.0, 0.0),
    "L_hip_flex": ("L_hip_flex", 1.0, 0.0), "L_hip_abd": ("L_hip_abd", 1.0, 0.0),
    "L_knee": ("L_knee", 1.0, 0.0),
    "R_hip_flex": ("R_hip_flex", 1.0, 0.0), "R_hip_abd": ("R_hip_abd", 1.0, 0.0),
    "R_knee": ("R_knee", 1.0, 0.0),
}
# joints with no incoming signal still need a sane hold value
PARKED = {"L_shoulder_twist": 0.0, "R_shoulder_twist": 0.0}


def main():
    if not os.path.exists(SCENE):
        raise SystemExit(
            f"scene not found: {SCENE}\nrun:  python sim/build_humanoid_v7_scene.py")

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    joints = sorted({t for t, _, _ in JOINT_MAP.values()} | set(PARKED))
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
    for n, v in PARKED.items():
        target[n] = v
    # start where the arm actually rests, not at roll=0 (which is arm-straight-up)
    for s_ in ("L", "R"):
        target[f"{s_}_shoulder_roll"] = REST_ROLL
    cmd = dict(target)

    dt = model.opt.timestep
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Physics ON: position actuators + mj_step.")
    print("Shoulder v3: pitch +/-95, abduction 0..185, twist +/-90 (roll-coupled).")
    print(f"Rest roll = {REST_ROLL:.4f} rad; abduction is subtracted from it.")
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
                    name, scale, offset = JOINT_MAP[key]
                    lo, hi = limits[name]
                    target[name] = max(lo, min(hi, offset + float(val) * scale))
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
