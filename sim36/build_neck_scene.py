"""
build_neck_scene.py - neck_v1.urdf -> physics-ready scene.

    python sim36/build_neck_scene.py

Same job as build_humanoid_v11_scene.py, for the neck. Run it once after every
gen_neck.py run; neck_receiver.py loads the scene, not the raw URDF.

WHY A SCENE STEP AT ALL, AND NOT JUST THE URDF
  * ACTUATORS. A URDF has no actuators, so a receiver can only write qpos, which
    teleports the model and ignores its own joint limits and inertia. v11 drives
    position actuators with mj_step and watches a real solve; the neck does the
    same, so what you see is the neck's dynamics, not a slideshow.
  * THE HEAD FRAME SURVIVES. MuJoCo WELDS fixed joints away, so the `head` link
    in the URDF is merged into the last moving body and there is no head body to
    query afterwards - mj_name2id returns -1, and code that trusts it silently
    reads the world body instead. We re-attach the head as a SITE at the same
    offset, which MuJoCo keeps, so the head frame stays addressable.
  * The base is welded to the world on purpose - the neck is bench-mounted, and
    v11 welds its torso the same way. That is why total scene mass is less than
    gen_neck.py's 1.0508 kg: the base's share belongs to the world body now.
  * COLLISION IS TURNED OFF, deliberately. See below.

WHY COLLISION IS OFF
    This is the same trap build_humanoid_v11_scene.py documents for the shoulder:
    the base welds into the world body, and MuJoCo ALWAYS checks world contacts
    so that robots can stand on floors. A neck is a NESTED assembly - the
    YesHolder group sits inside the cover, the servos sit inside their holders -
    so the moving parts start life interpenetrating the welded base by 9 to 98 mm
    and the solver spends every step trying to push them apart.

    Measured, before this fix: 16 contacts at the zero pose, and neck_yaw
    commanded to 20 deg and to 60 deg both stalled at 1.53 deg. The actuators
    were not weak; they were fighting the base.

    Travel on this mechanism is bounded by the joint limits (gen_neck.py puts
    them at 90% of servo travel), not by parts touching, so contact buys nothing
    here. Real clash checking belongs in a dedicated CAD sweep over the reachable
    pose set - the shoulder has sim36/interference_check.py for exactly that, and
    the neck wants its own once the axis assignment is confirmed on the bench.
"""

import os
import xml.etree.ElementTree as ET

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "live mimic system", "neck_v1.urdf")
OUT = os.path.join(HERE, "neck_scene.xml")

# Neck servos are the same class as the elbow's, moving ~0.57 kg of head and
# holder, so the elbow's gains are the right starting point rather than the
# shoulder's. forcerange mirrors gen_neck.py's LIMIT_EFFORT.
KP, KV = 8.0, 0.4
FORCE = 4.0


def head_site_from_urdf(path):
    """Where the head sits, and on which link, straight out of the URDF - so it
    cannot drift from gen_neck.py."""
    r = ET.parse(path).getroot()
    for j in r.findall("joint"):
        if j.get("name") == "head_fixed":
            return (j.find("parent").get("link"),
                    j.find("origin").get("xyz"))
    return None, None


def main():
    if not os.path.exists(URDF):
        raise SystemExit("neck_v1.urdf missing - run: python sim36/gen_neck.py")

    parent, head_xyz = head_site_from_urdf(URDF)

    model = mujoco.MjModel.from_xml_path(URDF)
    xml = os.path.join(HERE, "_neck_raw.xml")
    mujoco.mj_saveLastXML(xml, model)
    tree = ET.parse(xml)
    root = tree.getroot()
    os.remove(xml)

    hinges = []
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            hinges.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j))

    # the head site, on whichever body the head's parent link merged into
    if parent and head_xyz:
        wb = root.find("worldbody")
        target = None
        for body in wb.iter("body"):
            if body.get("name") == parent:
                target = body
        if target is None:                  # merged; use the deepest body
            bodies = list(wb.iter("body"))
            target = bodies[-1] if bodies else None
        if target is not None:
            ET.SubElement(target, "site", name="head", pos=head_xyz,
                          size="0.008", rgba="1 0.5 0 1")

    # kill collision on every geom (see "WHY COLLISION IS OFF" above)
    ngeom = 0
    for body in root.find("worldbody").iter():
        for geom in body.findall("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            ngeom += 1

    act = root.find("actuator")
    if act is None:
        act = ET.SubElement(root, "actuator")
    for jname in hinges:
        ET.SubElement(act, "position", name=f"{jname}_act", joint=jname,
                      kp=str(KP), kv=str(KV),
                      forcerange=f"{-FORCE} {FORCE}", ctrlrange="-1.5 1.5")

    ET.indent(tree, space="  ")
    tree.write(OUT, encoding="utf-8", xml_declaration=True)

    m = mujoco.MjModel.from_xml_path(OUT)
    print(f"wrote {OUT}")
    print(f"  bodies={m.nbody} dof={m.nv} actuators={m.nu} sites={m.nsite}")
    print(f"  driven: {hinges}")
    print(f"  total mass {m.body_mass.sum():.4f} kg (the base is welded to world)")
    print(f"  collision disabled on {ngeom} geoms (nested assembly - see docstring)")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"  contacts at the zero pose: {d.ncon}  (was 16 before this fix)")


if __name__ == "__main__":
    main()
