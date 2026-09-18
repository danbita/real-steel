"""
neck_receiver.py  -  Atom's neck, driven by your head
================================================================================
Run with mjpython:   mjpython "live mimic system/neck_receiver.py"
Pair with either sender:
    python "live mimic system/pose_sender.py"
    python "dual camera setup/pose_sender_dual.py"

Consumes the same UDP stream as every other receiver and picks out the neck
keys, so it can run alongside nothing else or be used to check the neck in
isolation before it goes into the humanoid.

    your head turn  ->  neck_yaw     (+ = you turn toward your OWN left)
    your head nod   ->  neck_pitch   (+ = nose down, same sense as *_flex)
   [your head tilt] ->  neck_roll    only if gen_neck.py ENABLE_ROLL is on AND
                                     pose_sender.py SEND_NECK_ROLL is on

SELF TEST, no camera and no sender needed:
    mjpython "live mimic system/neck_receiver.py" --selftest
Sweeps each joint through its full travel one at a time and names the joint as it
goes. This is the ten-second way to settle the one thing the SolidWorks export
could not tell us - whether the axis labelled yaw really turns the head or tips
it. If it tips, swap the two axis letters in STAGES in sim36/gen_neck.py and
regenerate.
================================================================================
"""

import json
import math
import os
import socket
import sys
import time

import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(here)
SCENE = os.path.join(REPO_ROOT, "sim36", "neck_scene.xml")

UDP_ADDR = ("127.0.0.1", 5005)
SIM_FPS = 60.0
SMOOTH_ALPHA = 0.35
MAX_RATE = 4.0          # rad/s. Lower than the arm's: this is a 1.05 kg head
                        # assembly on small servos, and it maps to hardware.

SELFTEST = "--selftest" in sys.argv
SELFTEST_PERIOD = 4.0   # s per joint


def main():
    if not os.path.exists(SCENE):
        sys.exit(f"scene not found: {SCENE}\nrun:  python sim36/gen_neck.py"
                 f"  then  python sim36/build_neck_scene.py")

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    # Whatever gen_neck.py emitted is the truth: discover the joints instead of
    # hard-coding them, so turning ENABLE_ROLL on needs no edit here.
    joints = []
    for jid in range(model.njnt):
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        if aid < 0:
            sys.exit(f"actuator for '{name}' missing - "
                     f"run: python sim36/build_neck_scene.py")
        joints.append({"name": name, "adr": model.jnt_qposadr[jid], "act": aid,
                       "range": tuple(model.jnt_range[jid])})
    if not joints:
        sys.exit("the scene has no hinge joints - regenerate it")

    print(f"Neck scene: {model.nbody - 1} bodies, {len(joints)} driven joints, "
          f"{model.nu} actuators  (physics ON: position actuators + mj_step)")
    for j in joints:
        lo, hi = j["range"]
        print(f"   {j['name']:12s} [{math.degrees(lo):+6.1f}, {math.degrees(hi):+6.1f}] deg")

    target = {j["name"]: 0.0 for j in joints}
    current = {j["name"]: 0.0 for j in joints}
    dt = 1.0 / SIM_FPS
    max_step = MAX_RATE * dt

    sock = None
    if not SELFTEST:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(UDP_ADDR)
        sock.setblocking(False)
        print(f"\nListening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]} - start a pose sender.")
        print("The neck holds still until the sender's neck neutral is captured "
              "(~1.5 s of your face in frame).\n")
    else:
        print("\nSELF TEST: sweeping one joint at a time, no UDP.\n")

    t0 = time.time()
    announced = None
    connected = False

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if SELFTEST:
                el = time.time() - t0
                idx = int(el / SELFTEST_PERIOD) % len(joints)
                phase = (el % SELFTEST_PERIOD) / SELFTEST_PERIOD
                active = joints[idx]
                if announced != active["name"]:
                    announced = active["name"]
                    print(f"  sweeping {active['name']}")
                for j in joints:
                    lo, hi = j["range"]
                    target[j["name"]] = (math.sin(2 * math.pi * phase) * hi
                                         if j is active else 0.0)
            else:
                while True:
                    try:
                        packet, _ = sock.recvfrom(8192)
                    except Exception:
                        break
                    try:
                        ang = json.loads(packet.decode())
                    except Exception:
                        continue
                    if not isinstance(ang, dict):
                        continue      # a --landmarks stream; nothing for us here
                    got = False
                    for j in joints:
                        if j["name"] in ang:
                            lo, hi = j["range"]
                            target[j["name"]] = max(lo, min(hi, float(ang[j["name"]])))
                            got = True
                    if got and not connected:
                        print("Receiving neck angles - neck live.")
                        connected = True

            # Rate-limit the COMMAND, then let the solver move the model. Writing
            # qpos directly would teleport the head and skip its own joint limits.
            for j in joints:
                n = j["name"]
                step = max(-max_step, min(max_step,
                                          SMOOTH_ALPHA * (target[n] - current[n])))
                current[n] += step
                data.ctrl[j["act"]] = current[n]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(dt)

    print("\nStopped.")


if __name__ == "__main__":
    main()
