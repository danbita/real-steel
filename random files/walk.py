"""
walk.py  —  a humanoid figure animated through a walking gait
================================================================================
Loads humanoid.urdf (torso + two 2-segment arms + two 2-segment legs) and poses
all eight joints through a walking cycle so the figure walks in place, like on a
treadmill. This is a KINEMATIC animation: we set the joint angles frame by frame
to LOOK like walking. It is not (yet) physically balancing under gravity — that
harder step comes later.

The gait follows the real rules of human walking:
  - the two legs swing in opposite phase (one forward while the other is back)
  - each knee flexes as its leg swings through, and never bends the wrong way
  - the arms swing OPPOSITE to the legs on the same side (contralateral swing),
    which is what your arms naturally do when you walk

Run on a Mac with:
    mjpython walk.py
(Install once with: pip3 install mujoco. Use mjpython, not python3.)
Stop by closing the window or pressing Ctrl+C.
================================================================================
"""

import time
import math
import os
import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(here, "humanoid.urdf"))
data = mujoco.MjData(model)

# Map each joint name to the slot where its angle lives in data.qpos.
def q(name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[jid]

L_hip, L_knee = q("L_hip"), q("L_knee")
R_hip, R_knee = q("R_hip"), q("R_knee")
L_sh, L_el = q("L_shoulder"), q("L_elbow")
R_sh, R_el = q("R_shoulder"), q("R_elbow")

print(f"Loaded humanoid.urdf — {model.njnt} joints. Walking in place.")
print("Close the window or press Ctrl+C to stop.\n")

# ---------------------------------------------------------------------------
# GAIT PARAMETERS  (tweak these to change the walk)
# ---------------------------------------------------------------------------
HIP_AMP = 0.5        # how far each leg swings forward/back  (~29 deg)
KNEE_AMP = 1.0       # how much each knee bends during swing  (~57 deg)
ARM_AMP = 0.4        # how far the arms swing                 (~23 deg)
ELBOW_BASE = 0.35    # a slight constant elbow bend (arms aren't dead straight)
ELBOW_AMP = 0.35     # extra elbow flex during the swing
PERIOD = 1.4         # seconds per full stride (both legs)
DT = 1.0 / 60.0

def knee_flex(phase):
    # (1 - cos)/2 goes smoothly 0 -> 1 -> 0 once per cycle, and is never
    # negative, so the knee only ever bends forward (anatomically correct).
    return KNEE_AMP * (1.0 - math.cos(phase)) / 2.0

def elbow_flex(phase):
    return ELBOW_BASE + ELBOW_AMP * (1.0 - math.cos(phase)) / 2.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        phase = 2.0 * math.pi * t / PERIOD
        opp = phase + math.pi          # opposite phase (half a stride later)

        # ---- LEGS: left and right in opposite phase --------------------
        data.qpos[L_hip] = HIP_AMP * math.sin(phase)
        data.qpos[R_hip] = HIP_AMP * math.sin(opp)
        # each knee bends most as that leg swings forward (a bit after mid-swing)
        data.qpos[L_knee] = knee_flex(phase + 1.2)
        data.qpos[R_knee] = knee_flex(opp + 1.2)

        # ---- ARMS: swing opposite to the SAME-side leg -----------------
        data.qpos[L_sh] = ARM_AMP * math.sin(opp)      # opposite to left leg
        data.qpos[R_sh] = ARM_AMP * math.sin(phase)    # opposite to right leg
        data.qpos[L_el] = elbow_flex(opp)
        data.qpos[R_el] = elbow_flex(phase)

        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(DT)
        t += DT

print("\nStopped.")
