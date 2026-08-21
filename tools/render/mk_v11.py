import json, os, re, sys
import numpy as np
sys.path.insert(0, "sim36")
from interference_check import read_stl

MESH = "live mimic system/meshes36d/shoulder_v6"
GROUPS = {"base": ["mount", "servoP_body", "pitch_retainer"],
          "carrier": ["servoP_shaft", "carrier", "servoR_body", "roll_idler"],
          "yoke": ["servoR_shaft", "yoke", "servoT_body", "brg_twist", "race_cap"],
          "plate": ["servoT_shaft", "interface_plate", "hub_clamp"]}
# shafts, journals and races turn inside their own bores - not clashes
SKIP = {"servoP_shaft", "servoR_shaft", "servoT_shaft", "roll_idler",
        "brg_twist", "hub_clamp", "race_cap", "pitch_retainer"}
C_Z = 0.100


def slabs(t, nmax=8, step=0.018):
    """Split a part along its longest axis and AABB each slab. One box per part
    is far too coarse for a frame like the carrier; slabs follow the real shape
    closely enough that MuJoCo agrees with the CAD checker."""
    lo, hi = t.min(0), t.max(0)
    ax = int(np.argmax(hi - lo))
    n = int(np.clip(round((hi[ax] - lo[ax]) / step), 1, nmax))
    if n == 1:
        return [(lo, hi)]
    edges = np.linspace(lo[ax], hi[ax], n + 1)
    out = []
    for i in range(n):
        sel = t[(t[:, ax] >= edges[i] - 1e-9) & (t[:, ax] <= edges[i + 1] + 1e-9)]
        if len(sel) < 3:
            continue
        a, b = sel.min(0), sel.max(0)
        a[ax], b[ax] = edges[i], edges[i + 1]
        out.append((a, b))
    return out or [(lo, hi)]


coll = {}
for g, ns in GROUPS.items():
    rows = []
    for n in ns:
        if n in SKIP:
            continue
        p = os.path.join(MESH, n + ".stl")
        if not os.path.exists(p):
            continue
        t = read_stl(p).reshape(-1, 3)
        for lo, hi in slabs(t):
            c = (lo + hi) / 2 - np.array([0.0, 0.0, C_Z])
            sz = np.maximum(hi - lo, 0.002)
            rows.append(("%.4f %.4f %.4f" % (c[0], c[1], c[2]), "box",
                         "%.4f %.4f %.4f" % (sz[0], sz[1], sz[2]), n))
    coll[g] = rows

L = json.load(open("sim36/v11_links.json"))
s = open("sim36/gen_humanoid_v8.py", encoding="utf-8").read()
s = s.replace("humanoid_v8_atom.urdf", "humanoid_v11_atom.urdf")
s = re.sub(r"meshes36(?![a-z])", "meshes36d", s)
s = s.replace("shoulder_v3", "shoulder_v6")
s = s.replace("gen_humanoid_v8.py", "gen_humanoid_v11.py")
s = s.replace("sim/shoulder_v3_prep.py", "sim36/prep_v11.py")

blocks = []
for k in ("base", "carrier", "yoke", "plate"):
    d = L[k]; r = d["rel"]
    blocks.append('    "%s": ((%.6f, %.6f, %.6f), %.6f,\n'
                  '             dict(ixx=%.9f, ixy=%.9f, ixz=%.9f,\n'
                  '                  iyy=%.9f, iyz=%.9f, izz=%.9f)),'
                  % (k, r[0], r[1], r[2], d["mass"], d["ixx"], d["ixy"], d["ixz"],
                     d["iyy"], d["iyz"], d["izz"]))
new_in = ("INERTIA_L = {\n    # shoulder_v6, integrated from its meshes by sim36/prep_v11.py\n"
          + "\n".join(blocks) + "\n}")
i0 = s.index("INERTIA_L = {"); i1 = s.index("\n}", i0) + 2
s = s[:i0] + new_in + "\n" + s[i1:]

nm = ("MESHES = {\n" +
      "".join('    "%s": %s,\n' % (g, json.dumps(ns)) for g, ns in GROUPS.items()) + "}")
i0 = s.index("MESHES = {"); i1 = s.index("\n}", i0) + 2
s = s[:i0] + nm + "\n" + s[i1:]

lines = ["COLL_L = {",
         "    # slabs of the real v6 parts. Shafts, races and journals are omitted:",
         "    # a journal turning in its bore is not a clash, and modelling it as one",
         "    # gives MuJoCo a permanent fake contact.", ]
for g in ("base", "carrier", "yoke", "plate"):
    lines.append('    "%s": [' % g)
    for c, k, sz, n in coll[g]:
        lines.append('        ("%s", "%s", "%s"),   # %s' % (c, k, sz, n))
    lines.append("    ],")
lines.append("}")
i0 = s.index("COLL_L = {"); i1 = s.index("\n}", i0) + 2
s = s[:i0] + "\n".join(lines) + "\n" + s[i1:]

# ---- mount the module INVERTED -------------------------------------------
# v6 is exported in v3's neutral pose with the arm straight UP. Rotating the
# whole module 180 deg about Y makes module-roll 0 the robot's resting pose,
# arm hanging. That is not cosmetic: at module-roll 180 the twist servo swings
# inboard through the roll servo and the pitch journal, and mounted this way
# the robot never reaches that end of the travel.
s = s.replace('    <origin xyz="0 {y} {MODULE_Z}" rpy="0 0 0"/>',
              '    <origin xyz="0 {y} {MODULE_Z}" rpy="0 3.1415927 0"/>')
# The joint axes are expressed in the CHILD frame, which IS the module frame -
# the 180 deg mount rotation already reverses them in the world. Flipping the
# signs here as well double-negates: the arms folded inward over the shoulders
# instead of abducting. So v8's axes carry over unchanged, and with the module
# inverted, module-roll 0 is the arm hanging and increasing roll abducts.

import re
# Roll now goes NEGATIVE. With the lower stop at 0 the arm could never come
# inboard of vertical, so the robot could not cross its arms, tuck a guard or
# throw a cross - 0 of 1801 frames ever went below 0 because the joint forbade
# it. -10 deg is what the module actually clears (measured: -15 deg fails on
# carrier/yoke), and the torso collision geometry stops it earlier when the arm
# is hanging. The rest of a tight guard comes from elbow flexion and twist.
s = re.sub(r'(_shoulder_roll".*?)<limit lower="[-0-9.]+" upper="[-0-9.]+"',
           r'\1<limit lower="-0.4363" upper="2.6180"', s, flags=re.S)

# Hips were 120 mm across on a 914 mm robot. Anthropometric hip breadth is
# 0.191 x stature = 175 mm, and the legs move out under it to match.
s = s.replace('<box size="0.060 0.120 0.050"/>', '<box size="0.070 0.175 0.055"/>')
s = s.replace('yh = 0.055 if s == "L" else -0.055',
              'yh = 0.065 if s == "L" else -0.065')
# TWIST ZERO. The arm's natural twist solved to -155.5 deg (left) / +157.7
# (right) - both pinned against the +/-180 stop. The joint zero was simply 156
# deg away from where the arm actually rests, so the model opened every clip by
# spinning the forearm most of a turn to get to its neutral pose, and the live
# receiver commanded far outside the +/-90 the CAD has been swept to.
# The two sides need OPPOSITE offsets because the right module is a mirror while
# both arms share the same (unmirrored) bicep mesh.
s = s.replace('rpy="3.1415927 0 -1.5707963"', 'rpy="3.1415927 0 {tw0}"')
s = s.replace("{ARM.format(s=s, bx=BICEP_X_OFF, bz=BICEP_Z, mat=mat)}",
              "{ARM.format(s=s, bx=BICEP_X_OFF, bz=BICEP_Z, mat=mat, tw0=tw0)}")
_anchor = '    tag = "LEFT" if s == "L" else "RIGHT"'
assert _anchor in s
s = s.replace(_anchor,
              '    tw0 = -1.5707963 + (-2.7333 if s == "L" else 2.7333)\n' + _anchor)

# With the twist zero fixed, the solved range is -43..+35 deg, so clamp the
# joint to the +/-90 the CAD has actually been swept to. It was +/-180, which
# let the live receiver command well outside verified travel - and there is no
# slip ring on that joint.
# Clamp twist to the +/-90 the CAD has actually been swept to. It was +/-180,
# which let the live receiver command far outside verified travel - and there
# is no slip ring on that joint. Done as a literal swap of the one comment +
# limit line: a regex here ate the joint name and produced invalid XML.
_TW_OLD = """    <!-- +/-180: the XH540 does a full turn in position mode. The old +/-90 was
         an arbitrary cap and the retarget saturated against it on 100% of
         frames, costing ~40 deg of forearm error. Wiring is the real limit -
         no slip ring, so leave a service loop and re-cap if it snags. -->
    <limit lower="-3.1416" upper="3.1416" effort="3.0" velocity="4.2"/>"""
_TW_NEW = """    <!-- +/-90, which is what sim36/interference_check.py has actually
         swept. With the twist zero corrected the solved range is only
         -43..+35 deg, so this costs nothing and keeps the live receiver
         inside verified travel. No slip ring: leave a service loop. -->
    <limit lower="-1.5708" upper="1.5708" effort="3.0" velocity="4.2"/>"""
assert _TW_OLD in s, "twist limit block not found"
s = s.replace(_TW_OLD, _TW_NEW)


# PITCH RANGE. +/-90 was an inherited software cap, not a physical one: the
# module sweeps clean through +/-180 (sim36/interference_check.py) and so does
# the assembled arm - the only contact anywhere in the range is the forearm
# folding into the torso at pitch 0, which is legitimate self-collision.
# The user's own recording pinned BOTH shoulders at +90 with IK residuals of
# 0.36-0.60, because raising your arms forward needs more than 90 deg of
# flexion. Anatomical range instead: 180 flexion, 60 extension. Positive pitch
# is forward - measured, not assumed.
s = re.sub(r'(_shoulder_pitch".*?)<limit lower="[-0-9.]+" upper="[-0-9.]+"',
           lambda mm: mm.group(1) + '<limit lower="-1.0472" upper="3.1416"',
           s, flags=re.S)

# The bicep hung 38 mm BEHIND the shoulder axis - a 12 deg backward lean at
# rest - because re-zeroing twist rotated the bicep's own 20 mm mounting offset
# into -x. Solved analytically for the value that puts the elbow directly under
# the shoulder at pitch 0.
s = s.replace("BICEP_X_OFF = 0.020019", "BICEP_X_OFF = -0.018392")


# BICEP CENTRING ON THE PLATE. Re-zeroing twist rotated the bicep's own 20 mm
# mounting offset into a new direction, leaving it sitting +9.3 mm (L) and
# -6.8 mm (R) off the interface plate's centreline - visible as the bicep
# hanging off the edge of the shoulder. The offsets differ per side because
# the two arms take opposite twist zeros, so the correction is per side too.
s = s.replace('<origin xyz="{bx} 0 {bz}" rpy="3.1415927 0 {tw0}"/>',
              '<origin xyz="{bx} {by} {bz}" rpy="3.1415927 0 {tw0}"/>')
s = s.replace("{ARM.format(s=s, bx=BICEP_X_OFF, bz=BICEP_Z, mat=mat, tw0=tw0)}",
              "{ARM.format(s=s, bx=BICEP_X_OFF, by=by, bz=BICEP_Z, mat=mat, tw0=tw0)}")
_TWLINE = '    tw0 = -1.5707963 + (-2.7333 if s == "L" else 2.7333)'
_BYLINE = '    by = -0.0093 if s == "L" else 0.0068'
assert _TWLINE in s
s = s.replace(_TWLINE, _TWLINE + chr(10) + _BYLINE)

s = s.replace("BICEP_Z = 0.075488", "BICEP_Z = %.6f" % L["bicep_z"])
# ATOM STEEL HEAD + WEATHERED PALETTE - recovered from the head session's
# final patch; applied at generation time so a re-run can never shed them.
_HEAD_OLD = '''    <visual><geometry><sphere radius="0.0595"/></geometry><origin xyz="0 0 0.855"/><material name="tan"/></visual>
    <!-- front markers: a bare spine and a sphere read identically front and back -->
    <visual><origin xyz="0.050 0.022 0.870"/><geometry><sphere radius="0.0095"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.050 -0.022 0.870"/><geometry><sphere radius="0.0095"/></geometry><material name="dark"/></visual>'''
_HEAD_NEW = '''    <!-- HEAD. Atom head lifted from Atom_Real_Steel_Model.obj (Material.005).
         Source is Y-up facing -Z, so +90 about X then +90 about Z into REP-103.
         Scaled to 143.7 mm = the DONOR's own head/stature ratio 0.1572, not the
         0.130 human figure in docs/proportions.md; at 118.9 it read small.
         Centred on its own bbox; z 0.84208 holds the crown at 0.9144.
         Nudged 20 mm FORWARD because the model's neck stub hangs off the REAR
         of the skull - at x=0 the head floated above the neck cylinder.
         PITCHED -12 deg (nose up): the mesh leans forward, and -12 is where the
         crown band reads level against a true horizontal.
         atom_head.png is BaseColor + Emissive baked in - MuJoCo ignores map_Ke,
         and that bake is what makes the eyes glow. The texture carries the
         grille face, so the old dark marker spheres are gone.
         Material is assigned in build_humanoid_v11_scene.py, not here: URDF
         materials cannot carry a texture MuJoCo will pick up.
         Visual only and MASSLESS - the torso inertial above is untouched. -->
    <visual><geometry><mesh filename="atom_head.obj"/></geometry><origin xyz="0.020 0 0.84208" rpy="0 -0.209440 0"/><material name="tan"/></visual>'''
_PAL_OLD = '''  <material name="gray"><color rgba="0.45 0.47 0.52 1"/></material>
  <material name="tan"><color rgba="0.85 0.72 0.6 1"/></material>
  <material name="blue"><color rgba="0.2 0.5 0.9 1"/></material>
  <material name="orange"><color rgba="0.95 0.55 0.2 1"/></material>
  <material name="dark"><color rgba="0.12 0.13 0.16 1"/></material>
  <material name="chest"><color rgba="0.30 0.33 0.40 1"/></material>'''
_PAL_NEW = '''  <!-- ATOM PALETTE, sampled from the donor model's own BaseColor maps: torso,
       arms and upper leg all bucket to a neutral gunmetal 0.32, extremities run
       warmer/darker, and the only saturated colour on him is the emissive cyan
       of the chest light and eyes. blue/orange keep their NAMES so side() and
       legs() are untouched, but are weathered metal tints now, not primaries. -->
  <material name="gray"><color rgba="0.32 0.32 0.32 1"/></material>
  <material name="tan"><color rgba="0.30 0.29 0.27 1"/></material>
  <material name="blue"><color rgba="0.20 0.55 0.68 1"/></material>
  <material name="orange"><color rgba="0.72 0.42 0.18 1"/></material>
  <material name="dark"><color rgba="0.14 0.14 0.15 1"/></material>
  <material name="chest"><color rgba="0.22 0.23 0.25 1"/></material>'''
assert _HEAD_OLD in s and _PAL_OLD in s, 'head/palette anchors drifted'
s = s.replace(_HEAD_OLD, _HEAD_NEW).replace(_PAL_OLD, _PAL_NEW)
open("sim36/gen_humanoid_v11.py", "w", encoding="utf-8").write(s)
print("wrote sim36/gen_humanoid_v11.py  BICEP_Z=%.6f" % L["bicep_z"])
