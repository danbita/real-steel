"""C7 torque-path explainer: the ROTATING chain solid, the stationary
structure ghosted. servo case (bolted to the ghosted yoke) -> spline ->
tapped disc -> hub clamp -> interface plate -> bicep."""
import os, sys
import numpy as np
import mujoco
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.abspath("sim36"))
from interference_check import read_stl

D = os.path.abspath("cad/shoulder_v6")
MM = 0.001
SOLID = {"servoT_body": (0.78, 0.18, 0.18, 1.0), "servoT_shaft": (0.10, 0.50, 0.54, 1.0),
         "hub_clamp": (0.45, 0.47, 0.52, 1.0), "interface_plate": (0.20, 0.55, 0.25, 1.0)}
GHOST = {"yoke": (0.85, 0.48, 0.16, 0.13), "brg_twist": (0.42, 0.28, 0.55, 0.18),
         "race_cap": (0.58, 0.60, 0.65, 0.15), "bicep_ref": (0.62, 0.66, 0.72, 0.10)}
parts = {**SOLID, **GHOST}
assets = "".join(f'<mesh name="{n}" file="{os.path.join(D, n + ".stl")}" scale="0.001 0.001 0.001"/>'
                 for n in parts)
geoms = "".join(f'<geom type="mesh" mesh="{n}" rgba="{c[0]} {c[1]} {c[2]} {c[3]}" contype="0" conaffinity="0"/>'
                for n, c in parts.items())
xml = f"""<mujoco><visual><headlight ambient="0.55 0.55 0.55" diffuse="0.5 0.5 0.5"/>
<global offwidth="1000" offheight="880"/></visual>
<asset>{assets}<texture type="skybox" builtin="gradient" rgb1="0.99 0.99 1.0" rgb2="0.92 0.93 0.96" width="64" height="64"/></asset>
<worldbody><light pos="0.3 -0.5 0.8" dir="-0.3 0.5 -0.8" diffuse="0.5 0.5 0.5"/>{geoms}</worldbody></mujoco>"""
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 880, 1000)
pts = np.concatenate([read_stl(os.path.join(D, n + ".stl")).reshape(-1, 3) for n in SOLID])
cen = ((pts.min(0) + pts.max(0)) / 2) * MM
out = Image.new("RGB", (1000, 500), (250, 250, 252))
for i, (az, el) in enumerate(((-55, -18), (140, -30))):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = cen
    cam.distance = 0.16
    cam.azimuth, cam.elevation = az, el
    r.update_scene(d, camera=cam)
    out.paste(Image.fromarray(r.render()).resize((500, 440), Image.LANCZOS), (i * 500, 10))
dr = ImageDraw.Draw(out)
dr.text((12, 462), "SOLID = what rotates together: spline > tapped disc > hub clamp > interface plate (> bicep).", fill=(60, 64, 72))
dr.text((12, 478), "GHOST = what it rotates AGAINST: the yoke (holding the servo case), bearing, race cap.", fill=(60, 64, 72))
out.save("docs/img_v6/drive_t.png")
print("drive_t.png written")
