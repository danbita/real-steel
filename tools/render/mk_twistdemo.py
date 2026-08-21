"""Twist-bearing explainer: the v11 sim spinning ONLY the twist joint with the
yoke + race_cap ghosted, so the 6806's job is visible - outer race parked in
the yoke's seat, inner race gripping the plate journal that spins inside it.
Writes docs/img_v6/twist_brg.gif."""
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
open("sim36/_twist_scene.xml", "w", encoding="utf8").write(xml)
m = mujoco.MjModel.from_xml_path("sim36/_twist_scene.xml")
os.remove("sim36/_twist_scene.xml")
d = mujoco.MjData(m)

Cc = lambda r, g, b: (r / 255, g / 255, b / 255)
SOLID = {"interface_plate": Cc(70, 72, 78), "hub_clamp": Cc(120, 130, 145),
         "servoT_shaft": Cc(26, 128, 138), "servoT_body": Cc(200, 45, 45),
         "brg_twist": Cc(107, 71, 140)}
GHOST = {"yoke": Cc(235, 140, 52), "race_cap": Cc(120, 130, 145)}
for g in range(m.ngeom):
    mid = m.geom_dataid[g]
    name = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid)
            if mid >= 0 else None)
    n = name.split("/")[-1] if name else None
    if n in SOLID:
        m.geom_rgba[g, :3] = SOLID[n]
        m.geom_rgba[g, 3] = 1.0
    elif n in GHOST:
        m.geom_rgba[g, :3] = GHOST[n]
        m.geom_rgba[g, 3] = 0.22
    elif m.geom_group[g] != 3:
        m.geom_rgba[g, 3] = 0.0

ease = lambda t: 0.5 - 0.5 * math.cos(math.pi * t)
ks = [0, math.radians(90), math.radians(-90), 0]
prog = []
for i in range(len(ks) - 1):
    for f in range(14):
        prog.append(ks[i] + (ks[i + 1] - ks[i]) * ease(f / 14))

mujoco.mj_forward(m, d)
plate = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "L_shoulder_plate")
cen = d.xpos[plate].copy()

cam = mujoco.MjvCamera()
cam.lookat[:] = cen + np.array([0.0, 0.0, -0.088])
cam.distance = 0.15
cam.azimuth, cam.elevation = -55, -18

r = mujoco.Renderer(m, 640, 760)
d.ctrl[:] = 0.0
for _ in range(400):
    mujoco.mj_step(m, d)
imgs = []
for t in prog:
    d.ctrl[2] = t                          # L twist actuator only
    for _ in range(50):
        mujoco.mj_step(m, d)
    r.update_scene(d, camera=cam)
    imgs.append(Image.fromarray(r.render()).resize((560, 480), Image.LANCZOS))
imgs[0].save("docs/img_v6/twist_brg.gif", save_all=True,
             append_images=imgs[1:], duration=95, loop=0, optimize=True,
             disposal=1)
print(f"twist_brg.gif {len(imgs)} frames, %.1f KB" %
      (os.path.getsize("docs/img_v6/twist_brg.gif") / 1024))
