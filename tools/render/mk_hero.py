"""Hero animation for the assembly guide - THE ACTUAL v11 SIM, not a
hand-posed clip: the same scene, joints, limits, servo gains and contact
physics as the robot render, with every visual except the LEFT shoulder
module hidden (no bicep/arm chain, no spine/torso, no head).
Writes hero.gif + hero.png (first frame)."""
import math
import os

import mujoco
import numpy as np
from PIL import Image

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

xml = open("sim36/humanoid_v11_scene.xml", encoding="utf8").read()
xml = xml.replace('rgb1="0.3 0.5 0.7" rgb2="0 0 0"',
                  'rgb1="0.99 0.99 1.0" rgb2="0.90 0.92 0.95"')
xml = xml.replace('<rgba haze="0.15 0.25 0.35 1" />',
                  '<rgba haze="0.97 0.97 0.99 1" />')
open("sim36/_hero_scene.xml", "w", encoding="utf8").write(xml)
m = mujoco.MjModel.from_xml_path("sim36/_hero_scene.xml")
os.remove("sim36/_hero_scene.xml")
d = mujoco.MjData(m)

# left shoulder module meshes only (right side is the *_r set)
KEEP = {"mount", "carrier", "yoke", "hub_clamp", "interface_plate",
        "pitch_retainer", "race_cap", "roll_idler", "brg_pitch", "brg_twist",
        "idler625", "servoP_body", "servoR_body", "servoT_body",
        "servoP_shaft", "servoR_shaft", "servoT_shaft"}
Cc = lambda r, g, b: (r / 255, g / 255, b / 255)
ORANGE, GRAY = Cc(235, 140, 52), Cc(120, 130, 145)
RED, TEAL, PUR = Cc(200, 45, 45), Cc(26, 128, 138), Cc(107, 71, 140)
PAL = {"mount": ORANGE, "carrier": ORANGE, "yoke": ORANGE,
       "pitch_retainer": GRAY, "race_cap": GRAY, "hub_clamp": GRAY,
       "interface_plate": Cc(70, 72, 78), "roll_idler": Cc(90, 60, 110),
       "brg_pitch": PUR, "brg_twist": PUR, "idler625": PUR,
       "servoP_body": RED, "servoR_body": RED, "servoT_body": RED,
       "servoP_shaft": TEAL, "servoR_shaft": TEAL, "servoT_shaft": TEAL}
kept = 0
for g in range(m.ngeom):
    mid = m.geom_dataid[g]
    name = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
            if mid >= 0 else None)
    if name is not None and name.split("/")[-1] in KEEP:
        m.geom_rgba[g, :3] = PAL[name.split("/")[-1]]
        m.geom_rgba[g, 3] = 1.0
        kept += 1
        continue
    if m.geom_group[g] != 3:              # visual geom, not ours: hide
        m.geom_rgba[g, 3] = 0.0
print("kept", kept, "module geoms")

AP, AR, AT = 0, 1, 2                       # L shoulder pitch/roll/twist acts
ease = lambda t: 0.5 - 0.5 * math.cos(math.pi * t)


def keys(n, ks):
    out = []
    seg = n // (len(ks) - 1)
    for i in range(len(ks) - 1):
        for f in range(seg):
            out.append(ks[i] + (ks[i + 1] - ks[i]) * ease(f / seg))
    return out


rad = math.radians
# joint-space setpoints, well inside the model's own limits - the sim's
# position servos and joint stops do the rest, exactly like the demo
P = keys(22, [0, rad(100), rad(-45), 0])   # pitch: swing up, then back
R_ = keys(16, [0, rad(80), 0])             # roll: arm out sideways
T = keys(18, [0, rad(70), rad(-70), 0])    # twist: both ways
prog = ([(p, 0, 0) for p in P] + [(0, r, 0) for r in R_]
        + [(0, 0, t) for t in T])

mujoco.mj_forward(m, d)
carrier = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "L_shoulder_carrier")
cen = d.xpos[carrier].copy()

cam = mujoco.MjvCamera()
cam.lookat[:] = cen + np.array([0.0, 0.0, -0.012])
cam.distance = 0.30
cam.azimuth, cam.elevation = -55, -14

r = mujoco.Renderer(m, 640, 760)
imgs = []
d.ctrl[:] = 0.0
for _ in range(400):                       # settle at rest first
    mujoco.mj_step(m, d)
for (p, ro, t) in prog:
    d.ctrl[AP], d.ctrl[AR], d.ctrl[AT] = p, ro, t
    for _ in range(50):                    # 0.1 s of real dynamics per frame
        mujoco.mj_step(m, d)
    r.update_scene(d, camera=cam)
    imgs.append(Image.fromarray(r.render()).resize((560, 480), Image.LANCZOS))
imgs[0].save("docs/img_v6/hero.png")
imgs[0].save("docs/img_v6/hero.gif", save_all=True, append_images=imgs[1:],
             duration=95, loop=0, optimize=True, disposal=1)
print(f"hero.gif {len(imgs)} frames, %.1f KB" %
      (os.path.getsize("docs/img_v6/hero.gif") / 1024))
