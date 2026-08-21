"""
build_humanoid_v11_scene.py — humanoid_v11_atom.urdf -> physics-ready scene.

    python sim/build_humanoid_v11_scene.py

Same job as build_humanoid_scene.py but for the v7 model: shoulder_v3
(3 DOF, real bearings, genuinely mirrored L/R) plus the ATOM arm.

After MuJoCo merges fixed joints the moving bodies per side are:
    *_shoulder_yoke   turntable + yoke + servo B
    *_shoulder_arm    C-bracket + ATOM bicep + elbow bracket + small gear
    *_forearm         forearm + big gear
The housings and torso all weld into the world body.
"""

import os
import xml.etree.ElementTree as ET

import mujoco
from gen_humanoid_v11 import COLL_L

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "live mimic system", "humanoid_v11_atom.urdf")
OUT = os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml")

DRIVEN = [
    ("L_shoulder_pitch", 10.6), ("L_shoulder_roll", 10.6),
    ("L_shoulder_twist", 10.6), ("L_elbow", 4.8),
    ("R_shoulder_pitch", 10.6), ("R_shoulder_roll", 10.6),
    ("R_shoulder_twist", 10.6), ("R_elbow", 4.8),
    ("L_hip_flex", 4.8), ("L_hip_abd", 4.8), ("L_knee", 4.8),
    ("R_hip_flex", 4.8), ("R_hip_abd", 4.8), ("R_knee", 4.8),
]

# Real servos on the shoulder and elbow; the legs are still the rig's own
# fictional joints and just need to track.
# STS3215 class: 3 Nm stall. A 63 g arm needs almost nothing, so gains are
# far lower than the person-scale model.
GAINS = {"elbow": (6.0, 0.3), "shoulder": (16.0, 0.5), "default": (25.0, 1.2)}

MESH_RGBA = {
    # Atom palette (see gen_humanoid_v11.py). The module used to be an orange /
    # red / teal / purple CAD key, which read as a toy next to a photoreal head.
    # Everything is gunmetal now, but the VALUES still separate the part classes
    # so the mechanism stays legible: structure mid, servos near-black, bearings
    # and shafts bright.
    "mount": "0.32 0.32 0.32 1", "carrier": "0.32 0.32 0.32 1",
    "yoke": "0.32 0.32 0.32 1",
    "servoP_body": "0.16 0.16 0.17 1", "servoR_body": "0.16 0.16 0.17 1",
    "servoT_body": "0.16 0.16 0.17 1",
    "servoP_shaft": "0.55 0.52 0.45 1", "servoR_shaft": "0.55 0.52 0.45 1",
    "servoT_shaft": "0.55 0.52 0.45 1",
    "roll_idler": "0.62 0.63 0.66 1", "brg_twist": "0.62 0.63 0.66 1",
    "pitch_retainer": "0.42 0.42 0.44 1", "hub_collar": "0.42 0.42 0.44 1",
    "race_cap": "0.42 0.42 0.44 1", "hub_clamp": "0.42 0.42 0.44 1",
    "interface_plate": "0.24 0.24 0.26 1",
    "big_gear_Link": "0.58 0.55 0.48 1", "small_gear_Link": "0.58 0.55 0.48 1",
}


def indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


def main():
    model = mujoco.MjModel.from_xml_path(URDF)
    tmp = os.path.join(HERE, "_v11_converted.xml")
    mujoco.mj_saveLastXML(tmp, model)
    tree = ET.parse(tmp)
    root = tree.getroot()
    root.set("model", "humanoid_v11_scene")

    comp = root.find("compiler")
    if comp is None:
        comp = ET.SubElement(root, "compiler")
    comp.set("meshdir", os.path.relpath(
        os.path.join(ROOT, "live mimic system", "meshes36d"),
        os.path.join(ROOT, "sim36")).replace("\\", "/") + "/")
    comp.set("texturedir", comp.get("meshdir"))
    comp.set("texturedir", comp.get("meshdir"))

    world = root.find("worldbody")

    cg = {}
    scan = [("torso", world)] + [(b.get("name"), b) for b in root.iter("body")]
    for bname, body in scan:
        for i, geom in enumerate(body.findall("geom")):
            if geom.get("contype") == "0":
                mesh = (geom.get("mesh") or "").split("/")[-1]
                if mesh.endswith("_r"):
                    mesh = mesh[:-2]
                if mesh == "atom_head":
                    # textured OBJ: a flat rgba here would kill the texture
                    geom.set("material", "atomhead")
                    geom.attrib.pop("rgba", None)
                elif mesh in MESH_RGBA:
                    geom.set("rgba", MESH_RGBA[mesh])
                continue
            gname = f"col_{bname}_{i}"
            geom.set("name", gname)
            geom.set("condim", "4")
            geom.set("friction", "0.8 0.05 0.001")
            geom.set("solref", "0.005 1")
            geom.set("solimp", "0.9 0.95 0.001")
            geom.set("group", "3")          # hide collision prims from the render
            cg.setdefault(bname, []).append(gname)

    # MuJoCo joint limits are soft by default; with these servo gains the yaw
    # joint overshot its stop by ~4 deg, which is exactly the margin that keeps
    # the bicep out of the chest. Stiffen every limit constraint.
    for j in root.iter("joint"):
        if j.get("range"):
            j.set("solreflimit", "0.002 1")
            j.set("margin", "0")

    opt = ET.SubElement(root, "option")
    opt.set("timestep", "0.002")
    opt.set("gravity", "0 0 -9.81")
    opt.set("integrator", "implicitfast")
    opt.set("cone", "elliptic")

    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "headlight", diffuse="0.62 0.62 0.62",
                  ambient="0.38 0.38 0.38", specular="0.12 0.12 0.12")
    ET.SubElement(vis, "rgba", haze="0.15 0.25 0.35 1")
    ET.SubElement(vis, "global", azimuth="140", elevation="-20",
                  offwidth="1280", offheight="720")
    ET.SubElement(vis, "quality", shadowsize="4096")

    asset = root.find("asset")
    ET.SubElement(asset, "texture", type="2d", name="atomhead", file="atom_head.png")
    ET.SubElement(asset, "material", name="atomhead", texture="atomhead",
                  specular="0.35", shininess="0.45")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient",
                  rgb1="0.3 0.5 0.7", rgb2="0 0 0", width="512", height="3072")
    ET.SubElement(asset, "texture", type="2d", name="groundplane", builtin="checker",
                  mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
                  markrgb="0.8 0.8 0.8", width="300", height="300")
    ET.SubElement(asset, "material", name="groundplane", texture="groundplane",
                  texuniform="true", texrepeat="4 4", reflectance="0.15")

    ET.SubElement(world, "light", pos="0 0 3.0", dir="0 0 -1", directional="true")
    ET.SubElement(world, "light", pos="1.5 -1.5 2.0", dir="-0.5 0.5 -1",
                  diffuse="0.4 0.4 0.4")
    ET.SubElement(world, "geom", name="floor", type="plane", pos="0 0 0",
                  size="0 0 0.05", material="groundplane", condim="3")
    ET.SubElement(world, "camera", name="demo", pos="1.05 -0.70 0.62",
                  xyaxes="0.5547 0.8320 0 -0.1143 0.0762 0.9905")
    # front-on, matching a subject facing the camera: the robot's LEFT arm
    # lands on the RIGHT of frame, exactly like a person facing you.
    # chest-up, framed like the reference clip: the subject is cropped at the
    # chest, so showing the full robot made the two halves incomparable.
    # centred on the NECK (z = 0.748), level, and pulled back to 0.95 m so the
    # whole arm swing stays in frame - the reference clip is framed the same way
    # but we need more room because the robot's arms actually have to be visible.
    ET.SubElement(world, "camera", name="front", pos="0.95 0 0.748",
                  xyaxes="0 1 0 0 0 1")
    # close on the LEFT shoulder module (y = +0.255, axes meet at z = 0.58)
    ET.SubElement(world, "camera", name="shoulder", pos="0.30 0.36 0.85",
                  xyaxes="-0.6279 0.7784 0 -0.2488 -0.2007 0.9476")

    act = ET.SubElement(root, "actuator")
    for jname, frc in DRIVEN:
        if jname.endswith("elbow"):
            kp, kv = GAINS["elbow"]
        elif "shoulder" in jname:
            kp, kv = GAINS["shoulder"]
        else:
            kp, kv = GAINS["default"]
        ET.SubElement(act, "position", name=f"{jname}_act", joint=jname,
                      kp=str(kp), kv=str(kv),
                      forcerange=f"{-frc} {frc}", ctrlrange="-3.7 3.7")

    contact = ET.SubElement(root, "contact")
    n = 0

    def pair(g1, g2):
        nonlocal n
        ET.SubElement(contact, "pair", geom1=g1, geom2=g2, condim="4",
                      friction="0.8 0.8 0.05 0.001 0.001")
        n += 1

    # Everything below is filtered out by MuJoCo's default parent/child rules,
    # so each one has to be declared or it silently passes through.
    # The shoulder base welds into the world body, and MuJoCo ALWAYS checks
    # world contacts (so robots can stand on floors). That makes the carrier's
    # journal spigot collide with the cradle bore it is supposed to spin in.
    # Turn the module's own base prims off for default collision and re-add
    # only the pairs that are real clash candidates; explicit <pair> is
    # evaluated regardless of contype.
    TORSO_BOX = (0.1, 0.17, 0.325)
    torso_g, module_g = [], []
    for geom in world.findall("geom"):
        gname = geom.get("name") or ""
        if not gname.startswith("col_torso") or "floor" in gname:
            continue
        sz = tuple(round(float(v), 4) for v in geom.get("size", "").split())
        if sz == TORSO_BOX:
            torso_g.append(gname)
        else:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            module_g.append(gname)
    for s in ("L", "R"):
        arm = cg.get(f"{s}_shoulder_plate", [])     # plate + bicep + bracket
        fore = cg.get(f"{s}_forearm", [])
        struct = (cg.get(f"{s}_shoulder_yoke", [])
                  + cg.get(f"{s}_shoulder_carrier", []))
        for g2 in fore:                       # elbow self-collision
            for g1 in arm:
                pair(g1, g2)
        # The plate's OWN structure boxes interlock with the yoke by design -
        # the boss runs through the twist bearing and the shell wraps servo T.
        # Their real clearance is 2.00 mm, measured on the true geometry by
        # sim36/interference_check.py. MuJoCo sees axis-aligned boxes around
        # non-box parts and reads that as a 7 mm penetration, so pairing them
        # here manufactures a contact that does not exist. The CAD checker owns
        # intra-module clearance; MuJoCo owns everything distal to the plate.
        arm_distal = [g for g in arm
                      if int(g.rsplit("_", 1)[1]) >= len(COLL_L["plate"])]
        # The yoke and everything on the plate are separated by a designed 2 mm
        # across the twist bearing. MuJoCo collides convex hulls of those parts
        # and reads that gap as a 1 mm overlap. sim36/interference_check.py
        # measures the true surfaces over the whole twist range and gets 2.00 mm,
        # so the CAD owns this interface and MuJoCo is told to leave it alone.
        yoke_g = cg.get(f"{s}_shoulder_yoke", [])
        for g2 in arm_distal + fore:          # arm vs carrier only
            for g1 in cg.get(f"{s}_shoulder_carrier", []):
                pair(g1, g2)
        for g2 in fore:                       # forearm still meets the yoke
            for g1 in yoke_g:
                pair(g1, g2)
        for g2 in arm_distal + fore:          # arm vs torso, and vs the housings
            for g1 in torso_g + module_g:
                pair(g1, g2)
        # The yoke swings past the mount on every roll move. This pair was
        # missing, and it hid a 4mm interference at almost every roll angle.
        # The CARRIER is deliberately still excluded: its journal spins inside
        # the cradle bore, which is a bearing, not a clash.
        for g2 in cg.get(f"{s}_shoulder_yoke", []):
            for g1 in module_g:
                pair(g1, g2)
        for g2 in arm_distal:                 # bicep vs its own module housing
            for g1 in cg.get(f"{s}_shoulder_carrier", []):
                pair(g1, g2)

    indent(root)
    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    os.remove(tmp)

    m = mujoco.MjModel.from_xml_path(OUT)
    print(f"wrote {os.path.relpath(OUT, ROOT)}")
    print(f"  bodies={m.nbody} dof={m.nv} actuators={m.nu} contact_pairs={n}")
    print(f"  total mass {m.body_mass.sum():.3f} kg (torso is welded to world)")


if __name__ == "__main__":
    main()
