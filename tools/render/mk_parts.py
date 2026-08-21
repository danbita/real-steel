"""Solo thumbnails of every PRINTED part, straight from the current STLs, for
the guide's Part breakdown dropdown. Re-run after any CAD change."""
import os
import sys

import mujoco
import numpy as np
from PIL import Image

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "sim36")
from interference_check import read_stl

D = "cad/shoulder_v6"
OUT = "docs/img_v6"
MM = 0.001

C = lambda r, g, b: (r / 255, g / 255, b / 255)
PARTS = {  # name -> (color, azimuth, elevation)  angles picked per part face
    "mount":           (C(235, 140, 52), -125, -22),
    "carrier":         (C(235, 140, 52),  -35, -22),
    "yoke":            (C(235, 140, 52),  -55, -28),
    "hub_clamp":       (C(120, 130, 145), -35, -35),
    "interface_plate": (C(70, 72, 78),    -35, -35),
    "pitch_retainer":  (C(120, 130, 145), -35, -25),
    "race_cap":        (C(120, 130, 145), -35, -35),
    "roll_idler":      (C(90, 60, 110),   -35, -25),
    "bicep_ref":       (C(202, 209, 238), -35, -18),
}

DIMS = {}
for name, (col, az, el) in PARTS.items():
    xml = f"""<mujoco><visual>
  <headlight ambient="0.5 0.5 0.5" diffuse="0.5 0.5 0.5"/>
  <global offwidth="640" offheight="560"/></visual>
  <asset><mesh name="m" file="{os.path.abspath(os.path.join(D, name + '.stl'))}" scale="0.001 0.001 0.001"/>
  <texture type="skybox" builtin="gradient" rgb1="0.99 0.99 1.0" rgb2="0.90 0.92 0.95" width="64" height="64"/></asset>
  <worldbody><light pos="0.4 -0.6 0.8" dir="-0.4 0.6 -0.7" diffuse="0.55 0.55 0.55"/>
  <light pos="-0.5 0.4 0.6" dir="0.5 -0.4 -0.5" diffuse="0.35 0.35 0.35"/>
  <geom type="mesh" mesh="m" rgba="{col[0]} {col[1]} {col[2]} 1" contype="0" conaffinity="0"/>
  </worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    v = read_stl(os.path.join(D, name + ".stl")).reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (lo + hi) / 2 * MM
    cam.distance = float(np.linalg.norm(hi - lo)) * MM * 1.35
    cam.azimuth, cam.elevation = az, el
    r = mujoco.Renderer(m, 560, 640)
    r.update_scene(d, camera=cam)
    img = Image.fromarray(r.render()).resize((460, 400), Image.LANCZOS)
    img.save(os.path.join(OUT, f"part_{name}.png"))
    r.close()
    DIMS[name] = [round(float(x), 1) for x in (hi - lo)]
    print("part_%s.png" % name, DIMS[name])
import json
json.dump(DIMS, open(os.path.join(OUT, "part_dims.json"), "w"), indent=1)
print("done + part_dims.json")
