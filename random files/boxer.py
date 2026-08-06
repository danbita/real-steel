"""
boxer.py  —  the humanoid throwing basic boxing punches
================================================================================
Loads the same humanoid.urdf and poses it like a boxer: hands up in a guard,
then throwing alternating STRAIGHT punches (left jab, right cross), snapping the
arm out and pulling it straight back to guard, with a light bounce in the legs
and a boxer's stance.

This is a KINEMATIC animation (we pose the joints to look like boxing); it isn't
simulating the force of a punch. And because every joint is a forward/back hinge,
these are straight punches — hooks and uppercuts would need sideways joints we
haven't built yet.

Run on a Mac with:
    mjpython boxer.py
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

def q(name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[jid]

L_hip, L_knee = q("L_hip"), q("L_knee")
R_hip, R_knee = q("R_hip"), q("R_knee")
L_sh, L_el = q("L_shoulder"), q("L_elbow")
R_sh, R_el = q("R_shoulder"), q("R_elbow")

print(f"Loaded humanoid.urdf — {model.njnt} joints. Throwing punches.\n")
print("Close the window or press Ctrl+C to stop.\n")

# ---------------------------------------------------------------------------
# KEY POSES (angles in radians)
#   Guard: upper arm angled forward-and-down, elbow deeply bent so the glove
#          sits up by the face. Punch: shoulder lifts the arm toward horizontal
#          and the elbow straightens, shooting the fist forward. Then back.
# ---------------------------------------------------------------------------
SH_GUARD, SH_PUNCH = -1.0, -1.45     # shoulder: guard -> extended
EL_GUARD, EL_PUNCH = 2.0, 0.2        # elbow: folded up -> nearly straight

SLOT = 0.55          # seconds per punch (arms alternate each slot)
DT = 1.0 / 60.0

def smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)

def punch_activation(frac):
    # 0 = guard, 1 = fully extended. Snap out, brief hold, pull back.
    if frac < 0.30:
        return smoothstep(frac / 0.30)          # extend fast
    elif frac < 0.45:
        return 1.0                               # short hold at full reach
    else:
        return 1.0 - smoothstep((frac - 0.45) / 0.55)   # retract to guard

def lerp(a, b, t):
    return a + (b - a) * t

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        slot = int(t / SLOT)
        frac = (t % SLOT) / SLOT
        left_turn = (slot % 2 == 0)              # even slot = left jab, odd = right cross

        aL = punch_activation(frac) if left_turn else 0.0
        aR = 0.0 if left_turn else punch_activation(frac)

        # ---- ARMS: active arm punches, other arm holds guard -----------
        data.qpos[L_sh] = lerp(SH_GUARD, SH_PUNCH, aL)
        data.qpos[L_el] = lerp(EL_GUARD, EL_PUNCH, aL)
        data.qpos[R_sh] = lerp(SH_GUARD, SH_PUNCH, aR)
        data.qpos[R_el] = lerp(EL_GUARD, EL_PUNCH, aR)

        # ---- LEGS: boxer's stance with a light bounce ------------------
        bounce = 0.08 * (0.5 - 0.5 * math.cos(2.0 * math.pi * t / SLOT))
        data.qpos[L_hip] = -0.20                 # left leg slightly forward (lead)
        data.qpos[R_hip] = 0.15                  # right leg slightly back
        data.qpos[L_knee] = 0.30 + bounce        # knees kept soft/bent
        data.qpos[R_knee] = 0.35 + bounce

        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(DT)
        t += DT

print("\nStopped.")
