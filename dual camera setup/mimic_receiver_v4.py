"""
mimic_receiver_v4.py — humanoid with physical-arm replicas + collision guard
================================================================================
Run with mjpython:     mjpython mimic_receiver_v4.py
Pair with either sender (pose_sender.py or pose_sender_dual.py).

Loads humanoid_arm_v4.urdf: a humanoid whose two arms are exact joint-level
replicas of the physical arm (arm_sim / armRosFile) — same limits (+/-90 deg
per shoulder axis, gear-elbow 0..150 deg) — with the familiar v3 legs.

NEW: COLLISION GUARD. We pose joints kinematically, so MuJoCo's physics won't
stop inter-penetration by itself. Instead, every tick each limb's candidate
move is tested with MuJoCo's contact detection BEFORE being accepted: a move
that would push the limb into the torso (or another limb) is rejected, and the
limb stops at the surface — like a real arm meeting a real chest. Blocked
limbs resume automatically the moment your motion moves them clear.
================================================================================
"""

import json
import os
import socket
import time

import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = os.path.join(here, "humanoid_arm_v4.urdf")

UDP_ADDR = ("127.0.0.1", 5005)
SIM_FPS = 60.0
SMOOTH_ALPHA = 0.35
MAX_RATE = 6.0                  # rad/s (matches the physical arm's receiver)
PENETRATION = -0.001            # contact depth (m) that counts as a collision

LIMB_GROUPS = {
    "L_arm": ["L_shoulder_flex", "L_shoulder_abd", "L_elbow"],
    "R_arm": ["R_shoulder_flex", "R_shoulder_abd", "R_elbow"],
    "L_leg": ["L_hip_flex", "L_hip_abd", "L_knee"],
    "R_leg": ["R_hip_flex", "R_hip_abd", "R_knee"],
}
JOINT_NAMES = [j for grp in LIMB_GROUPS.values() for j in grp]


def main():
    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    qmap, limits = {}, {}
    for name in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"joint '{name}' missing — is humanoid_arm_v4.urdf here?")
        qmap[name] = model.jnt_qposadr[jid]
        limits[name] = tuple(model.jnt_range[jid])

    def in_collision():
        """True if any contact shows real penetration."""
        for i in range(data.ncon):
            if data.contact[i].dist < PENETRATION:
                return True
        return False

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in JOINT_NAMES}
    current = {n: 0.0 for n in JOINT_NAMES}
    dt = 1.0 / SIM_FPS
    max_step = MAX_RATE * dt
    blocked = {g: False for g in LIMB_GROUPS}

    print("Humanoid v4: physical-arm replicas + v3 legs, collision guard ON.")
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
                for name, val in ang.items():
                    if name in target:
                        lo, hi = limits[name]
                        target[name] = max(lo, min(hi, float(val)))
                if not connected:
                    print("Receiving pose data — mirroring.")
                    connected = True

            # move each limb toward its target, one limb at a time, accepting
            # the move only if it produces no penetration
            for gname, joints in LIMB_GROUPS.items():
                saved = {n: current[n] for n in joints}
                for n in joints:
                    step = SMOOTH_ALPHA * (target[n] - current[n])
                    step = max(-max_step, min(max_step, step))
                    current[n] += step
                    data.qpos[qmap[n]] = current[n]
                mujoco.mj_forward(model, data)
                if in_collision():
                    for n, v in saved.items():        # reject: stop at surface
                        current[n] = v
                        data.qpos[qmap[n]] = v
                    mujoco.mj_forward(model, data)
                    if not blocked[gname]:
                        print(f"{gname} blocked by contact (holding at surface)")
                        blocked[gname] = True
                elif blocked[gname]:
                    blocked[gname] = False

            viewer.sync()
            time.sleep(dt)

    print("\nStopped.")


if __name__ == "__main__":
    main()
