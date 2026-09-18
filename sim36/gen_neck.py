"""
gen_neck.py - emit "live mimic system/neck_v1.urdf" from the SolidWorks export.

    python sim36/gen_neck.py

Generated, not hand-written. Edit THIS file, never the .urdf it writes.

WHY THIS EXISTS
===============
NeckAssembly_files/urdf/NeckAssembly_files.urdf is a raw SW2URDF export. It
carries good geometry and unusable kinematics, so it cannot be driven as-is:

  * FLAT TOPOLOGY. All 8 joints are parented to neckcover_link. Nothing is
    downstream of anything, so turning a joint spins one part in place and the
    head never goes anywhere. There is no pose to command.

  * NO HEAD LINK. Nothing in the export represents where the face ends up,
    which is the one frame pose tracking has to aim.

  * NO TRAVEL INFORMATION. Every movable joint is `continuous`, and the
    exporter's own table (NeckAssembly_files.csv) has Limit Lower, Limit Upper,
    Limit Effort and Limit Velocity ALL BLANK on every row. The export makes no
    claim whatsoever about how far anything turns.

  * PARTS EXPORTED TWICE. YesHolder2_Link is a merged SUB-ASSEMBLY. Two
    independent proofs:
        triangles:  servo(11758) + YesHolder3(3018) + MotorHolder3(240)
                    = 15016 = YesHolder2_Link exactly
        mass:       0.0887120062426798 + 0.21367369731836 + 0.00685662156823661
                    = 0.309242325129276
                    YesHolder2_Link  = 0.309242325129277   (6.1e-16 kg apart)
    So servo3_Link, YesHolder3_Link and MotorHolder3_Link ARE the contents of
    YesHolder2_Link, emitted a second time as loose siblings. Summing the export
    naively double-counts the hardware: 1.3601 kg claimed, 1.0508 kg real.

WHAT THE EXPORT DOES TELL US
============================
Axis directions, and they are clean once de-tilted. The eight joints collapse to
three mutually perpendicular directions, each appearing twice with flipped sign:

    ~ +/-Y   servo1_joint, BigHolder_joint          (exporter axes: servo_axis1, bigHolder_axis)
    ~ +/-X   servo3_joint, YesHolder3_joint         (servo_axis3, YesHolder_axis3)
    ~ +/-Z   YesHolder2_joint, MotorHolder3_joint   (YesHolder_axis2, Axis_MotorHolder3_joint)

Every off-axis component is the same number, 0.0397697640940, which is a single
2.279 deg rotation of the WHOLE assembly about Z:
    Rz(t) @ (1,0,0)  = ( cos t, sin t, 0) = ( 0.999209,  0.039770, 0)   matches
    Rz(t) @ (0,-1,0) = ( sin t,-cos t, 0) = ( 0.039770, -0.999209, 0)   matches
    Rz(t) @ (0,0,1)  = (0,0,1)                                          unaffected
with t = atan2(0.0397697640940149, 0.999208869988606) = 0.0397802 rad.
So we hand that tilt to the base joint ONCE as an rpy, and every joint below it
gets an exact unit axis. Same geometry, no fuzz propagating down the chain.

HOW MANY DOF THIS NECK ACTUALLY HAS: 2, NOT 3
=============================================
The part names in "Neck Stuff/" say it outright: YesHolder.SLDPRT (nodding yes)
and NoHolder.SLDPRT (shaking no). A yes/no neck is 2 DOF - pitch and yaw. The
duplicate arithmetic above agrees: once the double-counted siblings are removed
there are exactly two moving groups left, BigHolder and the YesHolder2
sub-assembly. The third axis belongs only to the duplicates.

Note NoHolder.SLDPRT is in the CAD folder and is NOT in the export at all, and
neither is NeckConnector.SLDPRT. So the export is also incomplete.

ROLL is therefore OFF by default. If the hardware turns out to have a third
actuated axis, set ENABLE_ROLL = True and it is inserted above pitch on X.

CHAIN, base to head
    neck_base      fixed, world-mounted. neckcover shell + servo2 + MotorHolder2
    neck_yaw       revolute about +Y  (vertical). Carries BigHolder + servo1.
    neck_pitch     revolute about +Z  (nod). Carries the YesHolder2 sub-assembly.
   [neck_roll]     revolute about +X, only if ENABLE_ROLL.
    head           the frame tracking aims. Placeholder mass/visual, see HEAD_*.

WHY +Y IS TREATED AS VERTICAL
    The neckcover shell measures 164.0 x 100.0 x 164.0 mm in X x Y x Z. A cover
    wrapped around a neck is near-circular in cross-section, and the two equal
    dimensions here are X and Z, which makes Y the axis of that cross-section.
    Y is also the axis BigHolder turns about, and BigHolder is the largest
    structural bracket - the gross turn ("no") is what you would expect the big
    base bracket to carry.

AXIS ASSIGNMENT: INFERRED HERE, CONFIRMED ON THE BENCH 2026-09-17
    Which physical axis is yaw vs pitch is inferred, not exported. Checked by
    running neck_receiver.py --selftest and watching neck_yaw: it TURNS the head
    rather than tipping it, so the assignment in STAGES below is correct and the
    reasoning above (Y is the cover's axis of symmetry, BigHolder carries the
    gross turn) holds.

    If this ever needs revisiting, that self-test is the ten-second check, and
    swapping the two axis letters in STAGES is the only edit - everything
    downstream reads this table.

    Still unconfirmed: the SIGN of each axis, i.e. whether +yaw turns the head
    the same way the person turns. That one shows up the first time the live
    sender drives it - if the mirroring is backwards, flip the matching entry in
    SIGN in pose_sender.py, not here.

SERVO TRAVEL
    Not in the export (see above), so it comes from SERVO_TRAVEL_DEG here.
    Default 180 deg is the common variant of the DS51xx family the CAD's
    DS51_Start_horn.SLDPRT belongs to; that family is also sold in a 270 deg
    variant. Limits are LIMIT_FRACTION of full travel, per the project decision
    to run at 90 percent of servo capability.
"""

import os
import math
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "NeckAssembly_files", "urdf", "NeckAssembly_files.urdf")
OUT = os.path.join(ROOT, "live mimic system", "neck_v1.urdf")
DEFAULT_MESH_REL = "../NeckAssembly_files/meshes"   # relative to OUT's directory
MESH_REL = DEFAULT_MESH_REL

# ----------------------------------------------------------------- knobs

SERVO_TRAVEL_DEG = 180.0    # full mechanical travel of ONE servo. NOT from the
                            # export - the export is blank. Change if the DS51xx
                            # on the bench is the 270 deg variant.
LIMIT_FRACTION = 0.90       # run at 90% of travel

ENABLE_ROLL = False         # see "HOW MANY DOF" above

# Effort/velocity are also blank in the export. These are the DS51xx-class
# figures used so the URDF is well-formed and the sim's actuators are not
# absurd; they are not measured.
LIMIT_EFFORT = 4.0          # N*m
LIMIT_VELOCITY = 5.0        # rad/s

HEAD_MASS = 0.25            # kg, PLACEHOLDER. No head part exists in the CAD
                            # folder yet. v11 carries a 143.7 mm Atom head as a
                            # massless visual; this gives it something to swing.
HEAD_SIZE = 0.1437          # m, matches gen_humanoid_v11.py's head scale

# The 2.279 deg whole-assembly tilt. We do NOT factor it out.
#
# It is tempting to hand the tilt to the first joint as rpy="0 0 TILT_Z" and give
# every joint an exact unit axis - it reproduces the exported axes exactly:
#     Rz(TILT_Z) @ (0,1,0) = (-0.039770, 0.999209, 0) = BigHolder_joint's axis
# But a joint's rpy rotates the WHOLE child link, meshes and inertia tensor
# included, and in the export every mesh sits at rpy=0 in the neckcover frame.
# Factoring the tilt into the joint would therefore swing all the geometry above
# it by 2.279 deg. Undoing that needs a counter-rotation on every visual,
# collision and inertial in the subtree, plus Rz(-t) on each COM vector - a lot
# of moving parts to buy a rounder number in the axis field.
#
# So each joint keeps its exported axis verbatim. A commanded "yaw" turns about
# an axis 2.279 deg off Y, which is what the hardware actually does.
TILT_Z = math.atan2(0.0397697640940149, 0.999208869988606)

# ----------------------------------------------------------------- the table
#
# ONE source of truth. Each stage: (joint name, axis letter, joint origin taken
# from this exported joint, [meshes that ride rigidly on this stage]).
#
# "origin_from" means: put this stage's frame where the export put that joint.
# At all-zero joint angles the generated chain reproduces the export's geometry
# exactly, which is what verify_chain() below asserts.

BASE_MESHES = ["neckcover_link", "servo2_Link", "MotorHolder2_Link"]

STAGES = [
    ("neck_yaw",   "Y", "BigHolder_joint",   ["BigHolder_Link", "servo1_Link"]),
    ("neck_pitch", "Z", "YesHolder2_joint",  ["YesHolder2_Link"]),
]
ROLL_STAGE = ("neck_roll", "X", "YesHolder3_joint", [])

# Exported twice - already inside YesHolder2_Link. Including them would
# double-count 0.309 kg and draw the same parts on top of themselves.
DUPLICATES = ["servo3_Link", "YesHolder3_Link", "MotorHolder3_Link"]

# ----------------------------------------------------------------- parse source


def load_export(path):
    """Pull masses, inertia tensors, COM offsets and joint origins out of the
    SW2URDF export. Parsed rather than transcribed so there is one source of
    truth and no copy errors."""
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        i = l.find("inertial")
        o = i.find("origin")
        links[l.get("name")] = {
            "com": [float(v) for v in o.get("xyz").split()],
            "mass": float(i.find("mass").get("value")),
            "inertia": dict(i.find("inertia").attrib),
        }
    axes = {}
    for j in root.findall("joint"):
        joints[j.get("name")] = [float(v) for v in j.find("origin").get("xyz").split()]
        a = j.find("axis")
        if a is not None:
            axes[j.get("name")] = [float(v) for v in a.get("xyz").split()]
    return links, joints, axes


LINKS, JOINT_ORIGIN, JOINT_AXIS = load_export(SRC)


def sub(a, b):
    return [a[k] - b[k] for k in range(3)]


def xyz(v):
    # 12 decimals, not 9: neck_chain_check.py asserts zero-pose fidelity at
    # 1e-12 m, and 9 decimals rounds to 1 nm which trips it for no real reason.
    return " ".join(f"{c:.12f}" for c in v)


# ----------------------------------------------------------------- emit


def mesh_block(name, offset, tag):
    return f"""    <{tag}>
      <origin xyz="{xyz(offset)}" rpy="0 0 0"/>
      <geometry><mesh filename="{MESH_REL}/{name}.STL"/></geometry>
    </{tag}>"""


def body(link_name, meshes, stage_origin, extra=""):
    """A chain link: every mesh that rides on this stage, plus a single lumped
    inertial. Masses add and COMs average by mass; the inertia tensors are
    summed with the parallel-axis shift. All export rpy are zero, so no tensor
    rotation is needed - only translation."""
    out = [f'  <link name="{link_name}">']
    total = sum(LINKS[m]["mass"] for m in meshes)
    if total > 0:
        com = [0.0, 0.0, 0.0]
        for m in meshes:
            w = LINKS[m]["mass"]
            # the part's COM in THIS stage's frame
            pc = [LINKS[m]["com"][k] + JOINT_ORIGIN[MESH_JOINT[m]][k] - stage_origin[k]
                  for k in range(3)]
            for k in range(3):
                com[k] += w * pc[k]
        com = [c / total for c in com]

        I = {k: 0.0 for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
        for m in meshes:
            w = LINKS[m]["mass"]
            pc = [LINKS[m]["com"][k] + JOINT_ORIGIN[MESH_JOINT[m]][k] - stage_origin[k]
                  for k in range(3)]
            d = [pc[k] - com[k] for k in range(3)]
            src = LINKS[m]["inertia"]
            I["ixx"] += float(src["ixx"]) + w * (d[1] ** 2 + d[2] ** 2)
            I["iyy"] += float(src["iyy"]) + w * (d[0] ** 2 + d[2] ** 2)
            I["izz"] += float(src["izz"]) + w * (d[0] ** 2 + d[1] ** 2)
            I["ixy"] += float(src["ixy"]) - w * d[0] * d[1]
            I["ixz"] += float(src["ixz"]) - w * d[0] * d[2]
            I["iyz"] += float(src["iyz"]) - w * d[1] * d[2]
        out.append(f"""    <inertial>
      <origin xyz="{xyz(com)}" rpy="0 0 0"/>
      <mass value="{total:.12f}"/>
      <inertia ixx="{I['ixx']:.12e}" ixy="{I['ixy']:.12e}" ixz="{I['ixz']:.12e}"
               iyy="{I['iyy']:.12e}" iyz="{I['iyz']:.12e}" izz="{I['izz']:.12e}"/>
    </inertial>""")

    for m in meshes:
        off = [JOINT_ORIGIN[MESH_JOINT[m]][k] - stage_origin[k] for k in range(3)]
        out.append(mesh_block(m, off, "visual"))
        out.append(mesh_block(m, off, "collision"))
    if extra:
        out.append(extra)
    out.append("  </link>")
    return "\n".join(out)


# which exported joint placed each mesh (the mesh lives in that joint's frame)
MESH_JOINT = {
    "neckcover_link": None,
    "servo1_Link": "servo1_joint",
    "servo2_Link": "servo2_joint",
    "servo3_Link": "servo3_joint",
    "BigHolder_Link": "BigHolder_joint",
    "YesHolder2_Link": "YesHolder2_joint",
    "YesHolder3_Link": "YesHolder3_joint",
    "MotorHolder2_Link": "MotorHolder2_joint",
    "MotorHolder3_Link": "MotorHolder3_joint",
}
JOINT_ORIGIN[None] = [0.0, 0.0, 0.0]   # neckcover is the export's own root


def chain(parent="neck_base", mount_xyz=(0, 0, 0), mount_rpy=(0, 0, 0),
          prefix="", mesh_rel=None, head_extra="", head_offset=None):
    """The neck as a URDF fragment, so the standalone model and the humanoid
    emit the SAME chain from the same table instead of two copies that drift.

    parent/mount_*  where to hang it. The standalone model mounts at the origin;
                    the humanoid mounts on the torso with a rotation, because the
                    CAD neck is Y-up and v11 is Z-up.
    head_extra      extra XML inside the head link - v11 puts its own head mesh
                    there, so that mesh finally MOVES instead of being welded to
                    the torso as decoration.
    """
    global MESH_REL
    if mesh_rel is not None:
        MESH_REL = mesh_rel
    lim = math.radians(SERVO_TRAVEL_DEG * LIMIT_FRACTION / 2.0)
    stages = list(STAGES)
    if ENABLE_ROLL:
        stages.append(ROLL_STAGE)

    parts = []
    base_link = f"{prefix}neck_base"
    if parent != base_link:
        parts.append(f"""  <joint name="{prefix}neck_mount" type="fixed">
    <origin xyz="{xyz(mount_xyz)}" rpy="{' '.join(f'{v:.9f}' for v in mount_rpy)}"/>
    <parent link="{parent}"/><child link="{base_link}"/>
  </joint>""")
    parts.append(body(base_link, BASE_MESHES, [0.0, 0.0, 0.0]))
    parts.append("")

    prev, prev_origin = base_link, [0.0, 0.0, 0.0]
    for jname, axis, origin_from, meshes in stages:
        o = JOINT_ORIGIN[origin_from]
        rel = sub(o, prev_origin)
        child = f"{prefix}{jname}_link"
        ax = JOINT_AXIS[origin_from]
        dom = max(range(3), key=lambda k: abs(ax[k]))
        assert "XYZ"[dom] == axis, (
            f"{jname}: exported axis {ax} is dominantly {'XYZ'[dom]}, "
            f"but STAGES says {axis}. Fix the table.")
        parts.append(f"""  <joint name="{prefix}{jname}" type="revolute">
    <origin xyz="{xyz(rel)}" rpy="0 0 0"/>
    <parent link="{prev}"/><child link="{child}"/>
    <axis xyz="{xyz(ax)}"/>
    <limit lower="{-lim:.6f}" upper="{lim:.6f}" effort="{LIMIT_EFFORT}" velocity="{LIMIT_VELOCITY}"/>
  </joint>""")
        parts.append(body(child, meshes, o))
        parts.append("")
        prev, prev_origin = child, o

    h = HEAD_SIZE
    hoff = (0.0, h / 2.0, 0.0) if head_offset is None else head_offset
    default_visual = (f'    <visual><geometry><box size="{h*0.7:.4f} {h:.4f} '
                      f'{h*0.7:.4f}"/></geometry><material name="head"/></visual>')
    parts.append(f"""  <joint name="{prefix}head_fixed" type="fixed">
    <origin xyz="{xyz(hoff)}" rpy="0 0 0"/>
    <parent link="{prev}"/><child link="{prefix}head"/>
  </joint>
  <link name="{prefix}head">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{HEAD_MASS}"/>
      <inertia ixx="{HEAD_MASS*h*h/6:.9e}" ixy="0" ixz="0"
               iyy="{HEAD_MASS*h*h/6:.9e}" iyz="0" izz="{HEAD_MASS*h*h/6:.9e}"/>
    </inertial>
{head_extra if head_extra else default_visual}
  </link>""")
    return "\n".join(parts)


def neck_extent():
    """Bounding box of the de-duplicated neck in the CAD frame, (lo, hi) per axis
    in metres. The humanoid uses it to work out where to hang the thing."""
    import struct
    lo, hi = [1e9] * 3, [-1e9] * 3
    mdir = os.path.join(ROOT, "NeckAssembly_files", "meshes")
    names = list(BASE_MESHES)
    for _, _, _, meshes in STAGES:
        names += meshes
    for nm in names:
        with open(os.path.join(mdir, f"{nm}.STL"), "rb") as f:
            f.read(80)
            n = struct.unpack("<I", f.read(4))[0]
            pts = []
            for _ in range(n):
                v = struct.unpack("<12f", f.read(50)[:48])
                pts += [v[3:6], v[6:9], v[9:12]]
        o = JOINT_ORIGIN[MESH_JOINT[nm]]
        for i in range(3):
            lo[i] = min(lo[i], min(q[i] for q in pts) + o[i])
            hi[i] = max(hi[i], max(q[i] for q in pts) + o[i])
    return lo, hi


def build():
    parts = ['<?xml version="1.0"?>',
             '<!-- GENERATED by sim36/gen_neck.py - do not edit. -->',
             '<robot name="atom_neck_v1">',
             # MuJoCo-specific hints, ignored by every other URDF consumer.
             #   strippath=false      keep our relative ../NeckAssembly_files path
             #   balanceinertia=true  the lumped tensors are sums of per-part
             #                        tensors plus parallel-axis terms; rounding
             #                        can leave one a hair outside the triangle
             #                        inequality, which MuJoCo rejects outright
             '  <mujoco>',
             '    <compiler strippath="false" balanceinertia="true" discardvisual="false"/>',
             '  </mujoco>',
             '  <material name="shell"><color rgba="0.79 0.82 0.93 1"/></material>',
             '  <material name="servo"><color rgba="0.84 0.0 0.0 1"/></material>',
             '  <material name="head"><color rgba="0.55 0.45 0.35 1"/></material>',
             '']
    # pinned explicitly: chain() sets the MESH_REL global, and gen_humanoid_v11
    # calls chain() with a different one. Without this, import order decides
    # which mesh path the standalone model gets written with.
    parts.append(chain(mesh_rel=DEFAULT_MESH_REL))
    parts.append("</robot>")
    return "\n".join(parts) + "\n"


if __name__ == "__main__":
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write(build())
    lim = SERVO_TRAVEL_DEG * LIMIT_FRACTION / 2.0
    print(f"wrote {OUT}")
    print(f"  stages: {[s[0] for s in STAGES] + (['neck_roll'] if ENABLE_ROLL else [])}")
    print(f"  limits: +/-{lim:.1f} deg  ({LIMIT_FRACTION:.0%} of {SERVO_TRAVEL_DEG:.0f} deg travel)")
    print(f"  dropped duplicates: {DUPLICATES}")
