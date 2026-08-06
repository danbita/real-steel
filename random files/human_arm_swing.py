"""
human_arm_swing.py  —  a two-segment arm that bends and swings like a human arm
================================================================================
The arm has two straight segments (upper arm + forearm) joined by a hinge in
the middle (the elbow), plus a shoulder hinge at the base. This script moves
BOTH hinges together so the arm swings the way a person's arm does:

  - the SHOULDER rocks the whole arm forward and back
  - the ELBOW bends and straightens in coordination, and — because a real elbow
    only flexes one way — it never bends backward past straight.

Run it on a Mac with:
    mjpython human_arm_swing.py
(Use mjpython, not python3. Install once with: pip3 install mujoco)

Stop by closing the viewer window or pressing Ctrl+C.
================================================================================
"""

import time
import math
import os
import mujoco
import mujoco.viewer

# ---------------------------------------------------------------------------
# Load the SAME arm.urdf (now with an anatomically correct elbow limit).
# ---------------------------------------------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(here, "arm.urdf"))
data = mujoco.MjData(model)

shoulder_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder")
elbow_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow")
shoulder_q = model.jnt_qposadr[shoulder_id]
elbow_q = model.jnt_qposadr[elbow_id]

print("Loaded arm.urdf — shoulder + elbow. Swinging like a human arm.")
print("Close the window or press Ctrl+C to stop.\n")

# ---------------------------------------------------------------------------
# MOTION DESIGN
#   phase goes around a full circle once per PERIOD seconds.
#
#   shoulder = SHOULDER_AMP * sin(phase)
#       -> a smooth forward/back rock, symmetric around the middle.
#
#   elbow = ELBOW_AMP * (1 - cos(phase)) / 2
#       -> this quantity is ALWAYS between 0 and ELBOW_AMP, so the elbow only
#          ever bends forward (never hyperextends). It's straight (0) when the
#          arm is at the back of its swing and most bent when it's forward,
#          which is what makes it read as a natural human motion.
# ---------------------------------------------------------------------------
SHOULDER_AMP = 0.6        # radians (~34 deg) forward/back rock of the whole arm
ELBOW_AMP = 1.8           # radians (~103 deg) max bend of the elbow
PERIOD = 2.5              # seconds per full swing cycle
DT = 1.0 / 60.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        phase = 2.0 * math.pi * t / PERIOD

        data.qpos[shoulder_q] = SHOULDER_AMP * math.sin(phase)
        data.qpos[elbow_q] = ELBOW_AMP * (1.0 - math.cos(phase)) / 2.0

        mujoco.mj_forward(model, data)
        viewer.sync()

        time.sleep(DT)
        t += DT

print("\nStopped.")
