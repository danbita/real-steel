"""
boxer_full.py  —  the detailed humanoid shadow-boxing
================================================================================
Loads humanoid_full.urdf (now with a head on a neck, hands with five fingers,
and feet on ankles) and shadow-boxes: it throws alternating straight punches and
CLENCHES the fingers into a fist as each punch lands, relaxing to a looser fist
in the guard. The head bobs slightly with the rhythm; the feet stay planted.

Still a kinematic animation (joints posed to look right, not physically driven).

Run on a Mac with:
    mjpython boxer_full.py
(pip3 install mujoco first. Use mjpython, not python3.)
Stop by closing the window or pressing Ctrl+C.
================================================================================
"""

import time
import math
import os
import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(here, "humanoid_full.urdf"))
data = mujoco.MjData(model)

def qadr(name):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return -1 if jid < 0 else model.jnt_qposadr[jid]

# Collect the finger joints of each hand automatically (mcp/pip + thumb).
def finger_joints(side):
    out = []
    for i in range(model.njnt):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if n and n.startswith(side + "_") and any(k in n for k in
                ["index", "middle", "ring", "pinky", "thumb"]):
            out.append((n, model.jnt_qposadr[i], model.jnt_range[i]))
    return out

L_fingers = finger_joints("L")
R_fingers = finger_joints("R")

L_sh, L_el = qadr("L_shoulder"), qadr("L_elbow")
R_sh, R_el = qadr("R_shoulder"), qadr("R_elbow")
L_hip, L_knee, L_ank = qadr("L_hip"), qadr("L_knee"), qadr("L_ankle")
R_hip, R_knee, R_ank = qadr("R_hip"), qadr("R_knee"), qadr("R_ankle")
neck = qadr("neck")

print(f"Loaded humanoid_full.urdf — {model.njnt} joints "
      f"({len(L_fingers)+len(R_fingers)} of them are fingers). Shadow-boxing.\n")

# ---- key arm poses ----
SH_GUARD, SH_PUNCH = -1.0, -1.45
EL_GUARD, EL_PUNCH = 2.0, 0.2
SLOT = 0.55
DT = 1.0 / 60.0

def smoothstep(x):
    x = max(0.0, min(1.0, x)); return x * x * (3.0 - 2.0 * x)

def punch_activation(frac):
    if frac < 0.30:  return smoothstep(frac / 0.30)
    elif frac < 0.45: return 1.0
    else:            return 1.0 - smoothstep((frac - 0.45) / 0.55)

def lerp(a, b, t): return a + (b - a) * t

def set_fist(fingers, curl):
    # curl in [0,1]; 0 = open hand, 1 = tight fist. Respects each joint's limit.
    for _, adr, rng in fingers:
        lo, hi = rng
        # curl toward the "bent" end of whatever this joint's range allows
        target = hi * curl if hi > 0 else lo * curl
        data.qpos[adr] = max(lo, min(hi, target))

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    while viewer.is_running():
        slot = int(t / SLOT)
        frac = (t % SLOT) / SLOT
        left_turn = (slot % 2 == 0)
        aL = punch_activation(frac) if left_turn else 0.0
        aR = 0.0 if left_turn else punch_activation(frac)

        # arms
        data.qpos[L_sh] = lerp(SH_GUARD, SH_PUNCH, aL)
        data.qpos[L_el] = lerp(EL_GUARD, EL_PUNCH, aL)
        data.qpos[R_sh] = lerp(SH_GUARD, SH_PUNCH, aR)
        data.qpos[R_el] = lerp(EL_GUARD, EL_PUNCH, aR)

        # hands: tighten the fist as the punch extends; keep a loose fist otherwise
        set_fist(L_fingers, 0.55 + 0.45 * aL)
        set_fist(R_fingers, 0.55 + 0.45 * aR)

        # legs: boxer stance + light bounce; feet planted
        bounce = 0.08 * (0.5 - 0.5 * math.cos(2.0 * math.pi * t / SLOT))
        data.qpos[L_hip] = -0.20; data.qpos[R_hip] = 0.15
        data.qpos[L_knee] = 0.30 + bounce; data.qpos[R_knee] = 0.35 + bounce
        if L_ank >= 0: data.qpos[L_ank] = -0.10
        if R_ank >= 0: data.qpos[R_ank] = -0.15

        # head: a small bob in rhythm
        if neck >= 0:
            data.qpos[neck] = 0.08 * math.sin(2.0 * math.pi * t / SLOT)

        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(DT)
        t += DT

print("\nStopped.")
