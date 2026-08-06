"""
swing_arm_mujoco.py  —  load the arm and swing its shoulder joint back and forth
================================================================================
Same goal as before, now using MuJoCo (which installs cleanly on your Mac).
It loads the SAME arm.urdf, opens a 3D viewer, and swings the shoulder joint
along a sine wave so the arm rocks back and forth.

IMPORTANT — how to run it on a Mac:
    mjpython swing_arm_mujoco.py
Use `mjpython`, NOT `python3`. On macOS the 3D viewer must run through
mjpython (it ships with MuJoCo). On Linux/Windows, `python3` is fine.

Install first (one time):
    pip3 install mujoco

Stop it by closing the viewer window, or pressing Ctrl+C in the terminal.
================================================================================
"""

import time
import math
import os
import mujoco
import mujoco.viewer

# ---------------------------------------------------------------------------
# 1. LOAD THE ROBOT
#    MuJoCo reads the very same arm.urdf we already wrote. It turns each
#    <joint> into a hinge it can move, and each <link> into a rigid body.
# ---------------------------------------------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(here, "arm.urdf"))
data = mujoco.MjData(model)   # holds the live state (joint angles, etc.)

# ---------------------------------------------------------------------------
# 2. FIND OUR JOINTS BY NAME
#    We named them "shoulder" and "elbow" in the URDF. MuJoCo stores each
#    joint's angle in the data.qpos array; jnt_qposadr tells us the slot.
# ---------------------------------------------------------------------------
shoulder_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder")
elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow")
shoulder_q = model.jnt_qposadr[shoulder_id]
elbow_q = model.jnt_qposadr[elbow_id]

print(f"Loaded arm.urdf — {model.njnt} joints found (shoulder, elbow).")
print("Swinging the shoulder. Close the window or press Ctrl+C to stop.\n")

# ---------------------------------------------------------------------------
# 3. THE ANIMATION LOOP
#    Each frame we compute a target angle from a sine wave and write it
#    straight into the shoulder joint, then mj_forward() updates the pose
#    and viewer.sync() redraws. The elbow is held straight at 0.
# ---------------------------------------------------------------------------
AMPLITUDE = 1.2      # radians (~69 degrees each way)
PERIOD = 3.0         # seconds per full back-and-forth cycle
DT = 1.0 / 60.0      # ~60 frames per second

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        data.qpos[shoulder_q] = AMPLITUDE * math.sin(2.0 * math.pi * t / PERIOD)
        data.qpos[elbow_q] = 0.0            # keep the elbow straight for now

        mujoco.mj_forward(model, data)      # recompute the arm's pose
        viewer.sync()                       # redraw the 3D view

        time.sleep(DT)
        t += DT

print("\nStopped. That's your first working simulation.")
