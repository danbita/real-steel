"""
gen_humanoid_v11.py — emit humanoid_v11_atom.urdf (shoulder_v6 + ATOM arm).

    python sim/gen_humanoid_v11.py

Generated, not hand-written. Edit THIS file.

WHAT IS DIFFERENT FROM v6
=========================
v6 used shoulder_v2 and had to mount the RIGHT module rotated 180 deg about Z,
because v2 was modelled left-only and its mount plate is on the module's -Y face.
That rotation flipped the arm fore/aft, which is why the right elbow folded
backward and had to be patched with a flipped joint axis.

shoulder_v6 exports a genuinely mirrored right-hand module (cad/shoulder_v6/right),
so BOTH modules now mount with rpy = 0 and share the same sign conventions. The
axis flip hack is gone.

Also new in v3, and reflected here: real bearings on pitch (6808-2RS) and twist
(6810-2RS), which is why the module mass went 771 g -> 933.5 g per side.

GEOMETRY (module frame, mm, from cad/shoulder_v6/build_shoulder_v6_cq.py)
    C = (0, 0, 100)      pitch / roll / twist axes all meet here
    mount plate face     y = -85 (left) / y = +85 (right, mirrored)
    bicep seats at       z = 188.5   (tray floor) - unchanged from v2

JOINTS PER SIDE
    *_shoulder_pitch  Y through C   flexion, positive = FORWARD
    *_shoulder_roll   X through C   abduction. 0 = arm UP (CAD neutral),
                                    1.5708 = straight out, 3.1416 = hanging
    *_shoulder_twist  Z, humerus    internal/external rotation
    *_elbow           unchanged ATOM arm

REST POSE IS roll = 3.1416, NOT ZERO. The CAD neutral is arm-straight-up.
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "live mimic system", "humanoid_v11_atom.urdf")

SHOULDER_Y = 0.1184      # biacromial/2 at 36 in; spine surface sits 89 mm inboard
MODULE_Z = 0.748         # shoulder (acromion) height = 0.818 x stature
CZ = 0.100
BICEP_Z = 0.102305       # measured back off the built model so C->elbow = 170.1 mm
                         # (the derived 0.090788 was 15 mm long: the scaled plate's
                         #  tray floor does not move linearly with the plate scale)
BICEP_X_OFF = -0.018392   # 37.7 mm x the 0.531 arm scale

M = f"0 0 -{CZ}"

# Left-hand values, measured by sim/shoulder_v6_prep.py. The right module is a
# true mirror, so its COM y and its ixy / iyz products flip sign - which is
# exactly what the prep script measured on the mirrored meshes.
INERTIA_L = {
    # shoulder_v6, integrated from its meshes by sim36/prep_v11.py
    "base": ((0.000000, -0.068431, 0.003471), 0.234859,
             dict(ixx=0.000179494, ixy=-0.000000000, ixz=-0.000000000,
                  iyy=0.000170123, iyz=-0.000006131, izz=0.000201799)),
    "carrier": ((0.002497, -0.008889, -0.004951), 0.153571,
             dict(ixx=0.000061856, ixy=-0.000003409, ixz=0.000008073,
                  iyy=0.000102863, iyz=0.000004852, izz=0.000110884)),
    "yoke": ((0.000429, 0.002949, 0.061150), 0.248273,
             dict(ixx=0.000285351, ixy=-0.000000319, ixz=0.000004431,
                  iyy=0.000413733, iyz=0.000005057, izz=0.000264074)),
    "plate": ((0.000000, 0.000000, 0.093989), 0.050993,
             dict(ixx=0.000016039, ixy=-0.000000000, ixz=0.000000000,
                  iyy=0.000013838, iyz=0.000000000, izz=0.000022592)),
}



def inertia(key, side):
    (cx, cy, cz), mass, I = INERTIA_L[key]
    if side == "R":
        cy = -cy
        I = dict(I, ixy=-I["ixy"], iyz=-I["iyz"])
    return (cx, cy, cz), mass, I


MESHES = {
    "base": ["mount", "servoP_body", "pitch_retainer"],
    "carrier": ["servoP_shaft", "carrier", "servoR_body", "roll_idler"],
    "yoke": ["servoR_shaft", "yoke", "servoT_body", "brg_twist", "race_cap"],
    "plate": ["servoT_shaft", "interface_plate", "hub_clamp"],
}


# Collision primitives from the CadQuery constants, in each link frame
# (module coords minus C). Meshes are concave; MuJoCo would hull the yoke into a
# block that swallows the carrier it straddles.
COLL_L = {
    # slabs of the real v6 parts. Shafts, races and journals are omitted:
    # a journal turning in its bore is not a clash, and modelling it as one
    # gives MuJoCo a permanent fake contact.
    "base": [
        ("0.0000 -0.1097 0.0005", "box", "0.0620 0.0174 0.0770"),   # mount
        ("0.0000 -0.0923 0.0005", "box", "0.0680 0.0174 0.0770"),   # mount
        ("0.0000 -0.0750 0.0005", "box", "0.0680 0.0174 0.0770"),   # mount
        ("0.0000 -0.0576 0.0100", "box", "0.0232 0.0174 0.0580"),   # mount
        ("0.0000 -0.0402 0.0030", "box", "0.0680 0.0174 0.0720"),   # mount
        ("0.0000 -0.0660 -0.0082", "box", "0.0200 0.0419 0.0181"),   # servoP_body
        ("0.0000 -0.0458 0.0100", "box", "0.0123 0.0020 0.0181"),   # servoP_body
        ("0.0000 -0.0668 0.0281", "box", "0.0200 0.0404 0.0181"),   # servoP_body
    ],
    "carrier": [
        ("-0.0270 -0.0067 -0.0000", "box", "0.0180 0.0654 0.0520"),   # carrier
        ("-0.0090 -0.0297 0.0000", "box", "0.0180 0.0194 0.0520"),   # carrier
        ("0.0090 -0.0297 0.0000", "box", "0.0180 0.0194 0.0520"),   # carrier
        ("0.0270 -0.0067 -0.0063", "box", "0.0180 0.0654 0.0645"),   # carrier
        ("0.0117 0.0000 -0.0281", "box", "0.0404 0.0200 0.0181"),   # servoR_body
        ("0.0327 0.0000 -0.0100", "box", "0.0020 0.0123 0.0181"),   # servoR_body
        ("0.0125 0.0000 0.0082", "box", "0.0419 0.0200 0.0181"),   # servoR_body
    ],
    "yoke": [
        ("0.0000 0.0000 -0.0144", "box", "0.0920 0.0467 0.0192"),   # yoke
        ("0.0000 0.0000 0.0049", "box", "0.0920 0.0480 0.0192"),   # yoke
        ("0.0000 0.0000 0.0434", "box", "0.0920 0.0240 0.0192"),   # yoke
        ("0.0000 0.0100 0.0626", "box", "0.0920 0.0570 0.0192"),   # yoke
        ("0.0000 0.0032 0.0819", "box", "0.0920 0.0705 0.0192"),   # yoke
        ("0.0000 -0.0082 0.0504", "box", "0.0200 0.0181 0.0419"),   # servoT_body
        ("0.0000 0.0100 0.0706", "box", "0.0123 0.0181 0.0020"),   # servoT_body
        ("0.0000 0.0281 0.0497", "box", "0.0200 0.0181 0.0404"),   # servoT_body
    ],
    "plate": [
        ("0.0000 -0.0247 0.0997", "box", "0.0580 0.0165 0.0165"),   # interface_plate
        ("0.0000 -0.0082 0.0933", "box", "0.0344 0.0165 0.0175"),   # interface_plate
        ("0.0000 0.0082 0.0933", "box", "0.0344 0.0165 0.0175"),   # interface_plate
        ("0.0000 0.0247 0.0997", "box", "0.0580 0.0165 0.0165"),   # interface_plate
    ],
}


RPY = {"cylX": ' rpy="0 1.5707963 0"', "cylY": ' rpy="1.5707963 0 0"',
       "cylZ": "", "box": ""}


def mirror_xyz(xyz):
    x, y, z = (float(v) for v in xyz.split())
    return f"{x} {-y} {z}"


def coll(key, side):
    out = []
    for xyz, kind, size in COLL_L[key]:
        if side == "R":
            xyz = mirror_xyz(xyz)
        if kind == "box":
            geo = f'<box size="{size}"/>'
        else:
            r, l = size.split()
            geo = f'<cylinder radius="{r}" length="{l}"/>'
        out.append(f'    <collision><origin xyz="{xyz}"{RPY[kind]}/>'
                   f"<geometry>{geo}</geometry></collision>")
    return "\n".join(out)


def visuals(key, side):
    # Distinct BASENAMES per hand. MuJoCo keys mesh assets by basename, so
    # shoulder_v6/right/mount.stl silently collapsed onto shoulder_v6/mount.stl
    # and the right module wore the left module's geometry - its mount plate
    # ended up 114mm outboard of the torso, floating in air.
    sfx = "" if side == "L" else "_r"
    return "\n".join(
        f'    <visual><origin xyz="{M}"/><geometry>'
        f'<mesh filename="shoulder_v6/{n}{sfx}.stl"/></geometry></visual>'
        for n in MESHES[key])


def link_block(side, key, name):
    (cx, cy, cz), mass, I = inertia(key, side)
    return f'''  <link name="{side}_{name}">
    <inertial>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}"/>
      <mass value="{mass:.6f}"/>
      <inertia ixx="{I['ixx']:.9f}" ixy="{I['ixy']:.9f}" ixz="{I['ixz']:.9f}"
               iyy="{I['iyy']:.9f}" iyz="{I['iyz']:.9f}" izz="{I['izz']:.9f}"/>
    </inertial>
{visuals(key, side)}
{coll(key, side)}
  </link>
'''


ARM = '''  <!-- ATOM arm, unchanged below the adapter -->
  <joint name="{s}_arm_adapter" type="fixed">
    <parent link="{s}_shoulder_plate"/><child link="{s}_bicep"/>
    <origin xyz="{bx} {by} {bz}" rpy="3.1415927 0 {tw0}"/>
  </joint>

  <link name="{s}_bicep">
    <inertial>
      <origin xyz="-0.000395 0.018929 -0.038795"/>
      <mass value="0.012476492"/>
      <inertia ixx="0.00000449966" ixy="0.00000005644" ixz="0.00000014778"
               iyy="0.00000551217" iyz="0.00000028248" izz="0.00000730659"/>
    </inertial>
    <visual><geometry><mesh filename="bicep_Link.STL"/></geometry>{mat}</visual>
    <collision><geometry><mesh filename="bicep_Link.STL"/></geometry></collision>
  </link>

  <joint name="{s}_elbow_bracket_fixed" type="fixed">
    <parent link="{s}_bicep"/><child link="{s}_elbow_bracket"/>
    <origin xyz="-0.000015060 -0.000182015 -0.066715451"/>
  </joint>
  <link name="{s}_elbow_bracket">
    <inertial>
      <origin xyz="-0.000322 0.020961 -0.005702"/>
      <mass value="0.023682739"/>
      <inertia ixx="0.00001026139" ixy="-0.00000000731" ixz="0.00000010830"
               iyy="0.00001401984" iyz="0.00000017752" izz="0.00001338420"/>
    </inertial>
    <visual><geometry><mesh filename="elbow_Link.STL"/></geometry>{mat}</visual>
    <collision><geometry><mesh filename="elbow_Link.STL"/></geometry></collision>
  </link>

  <joint name="{s}_small_gear_fixed" type="fixed">
    <parent link="{s}_bicep"/><child link="{s}_small_gear"/>
    <origin xyz="0.012251040 0.020049085 -0.074446811"/>
  </joint>
  <link name="{s}_small_gear">
    <inertial><origin xyz="-0.004779000 0 0"/><mass value="0.000942778"/>
      <inertia ixx="4.33270428790079E-07" ixy="0" ixz="0" iyy="3.86651069848133E-07" iyz="0" izz="3.86651069848133E-07"/>
    </inertial>
    <visual><geometry><mesh filename="small_gear_Link.STL"/></geometry></visual>
  </link>

  <joint name="{s}_elbow" type="revolute">
    <parent link="{s}_bicep"/><child link="{s}_forearm"/>
    <origin xyz="-0.000015060 0.020049616 -0.094646582" rpy="1.5707963 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit lower="0" upper="2.0944" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.15" friction="0.05"/>
  </joint>
  <link name="{s}_forearm">
    <inertial>
      <origin xyz="-0.000000 -0.003004 -0.012485"/>
      <mass value="0.020699741"/>
      <inertia ixx="0.00001097382" ixy="-0.00000000326" ixz="-0.00000000008"
               iyy="0.00000405101" iyz="-0.00000024760" izz="0.00001202303"/>
    </inertial>
    <visual><origin xyz="0.000071897 -0.066235683 0.013072476"/><geometry><mesh filename="short_arm_link.STL"/></geometry>{mat}</visual>
    <collision><origin xyz="0.000071897 -0.066235683 0.013072476"/><geometry><mesh filename="short_arm_link.STL"/></geometry></collision>
  </link>

  <joint name="{s}_big_gear_fixed" type="fixed">
    <parent link="{s}_forearm"/><child link="{s}_big_gear"/>
    <origin xyz="0.012266100 0 0"/>
  </joint>
  <link name="{s}_big_gear">
    <inertial><origin xyz="-0.004779000 0 0"/><mass value="0.005722263"/>
      <inertia ixx="1.45911616699868E-05" ixy="0" ixz="0" iyy="8.32211862139204E-06" iyz="0" izz="8.33289258825403E-06"/>
    </inertial>
    <visual><geometry><mesh filename="big_gear_Link.STL"/></geometry></visual>
  </link>
'''


def side(s):
    y = SHOULDER_Y if s == "L" else -SHOULDER_Y
    # Mirrored geometry means BOTH modules mount unrotated. Pitch and twist keep
    # the same sign on both sides; roll flips, because "outward" flips.
    roll_axis = "-1 0 0" if s == "L" else "1 0 0"
    mat = '<material name="blue"/>' if s == "L" else '<material name="orange"/>'
    tw0 = -1.5707963 + (-2.7333 if s == "L" else 2.7333)
    by = -0.0093 if s == "L" else 0.0068
    tag = "LEFT" if s == "L" else "RIGHT"

    return f'''  <!-- ################### {tag} SHOULDER (v3 module) ################## -->
  <joint name="{s}_shoulder_mount" type="fixed">
    <parent link="torso"/><child link="{s}_shoulder_base"/>
    <origin xyz="0 {y} {MODULE_Z}" rpy="0 3.1415927 0"/>
  </joint>

{link_block(s, "base", "shoulder_base")}
  <joint name="{s}_shoulder_pitch" type="revolute">
    <parent link="{s}_shoulder_base"/><child link="{s}_shoulder_carrier"/>
    <origin xyz="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-1.0472" upper="3.1416" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

{link_block(s, "carrier", "shoulder_carrier")}
  <!-- roll = abduction. 0 arm up, 1.5708 straight out, 3.1416 hanging. -->
  <joint name="{s}_shoulder_roll" type="revolute">
    <parent link="{s}_shoulder_carrier"/><child link="{s}_shoulder_yoke"/>
    <origin xyz="0 0 0"/><axis xyz="{roll_axis}"/>
    <limit lower="-0.4363" upper="2.6180" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

{link_block(s, "yoke", "shoulder_yoke")}
  <joint name="{s}_shoulder_twist" type="revolute">
    <parent link="{s}_shoulder_yoke"/><child link="{s}_shoulder_plate"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <!-- +/-90, which is what sim36/interference_check.py has actually
         swept. With the twist zero corrected the solved range is only
         -43..+35 deg, so this costs nothing and keeps the live receiver
         inside verified travel. No slip ring: leave a service loop. -->
    <limit lower="-1.5708" upper="1.5708" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

{link_block(s, "plate", "shoulder_plate")}
{ARM.format(s=s, bx=BICEP_X_OFF, by=by, bz=BICEP_Z, mat=mat, tw0=tw0)}'''


def legs():
    out = ""
    for s, ax, mat in (("L", "1 0 0", "blue"), ("R", "-1 0 0", "orange")):
        yh = 0.065 if s == "L" else -0.065
        out += f'''  <joint name="{s}_hip_flex" type="revolute">
    <parent link="torso"/><child link="{s}_hip_link"/>
    <origin xyz="0 {yh} 0.4846"/><axis xyz="0 -1 0"/>
    <limit lower="-1.0" upper="2.1" effort="3.0" velocity="4.2"/>
  </joint>
  <link name="{s}_hip_link"><inertial><mass value="0.05"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="{s}_hip_abd" type="revolute">
    <parent link="{s}_hip_link"/><child link="{s}_thigh"/>
    <origin xyz="0 0 0"/><axis xyz="{ax}"/>
    <limit lower="-0.4" upper="1.2" effort="3.0" velocity="4.2"/>
  </joint>
  <link name="{s}_thigh">
    <visual><geometry><box size="0.032 0.032 0.224"/></geometry><origin xyz="0 0 -0.112"/><material name="{mat}"/></visual>
    <collision><geometry><box size="0.032 0.032 0.224"/></geometry><origin xyz="0 0 -0.112"/></collision>
    <inertial><mass value="0.16"/><origin xyz="0 0 -0.112"/><inertia ixx="0.0007" iyy="0.0007" izz="0.00003" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="{s}_knee" type="revolute">
    <parent link="{s}_thigh"/><child link="{s}_shin"/>
    <origin xyz="0 0 -0.224"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="2.4" effort="3.0" velocity="4.2"/>
  </joint>
  <link name="{s}_shin">
    <visual><geometry><box size="0.026 0.026 0.2249"/></geometry><origin xyz="0 0 -0.11245"/><material name="{mat}"/></visual>
    <collision><geometry><box size="0.026 0.026 0.2249"/></geometry><origin xyz="0 0 -0.11245"/></collision>
    <!-- foot: 139 mm long x 35.7 mm tall, so the robot stands at the full 36 in
         (stature is measured to the floor, and the leg chain ends at the ankle) -->
    <visual><geometry><box size="0.139 0.050 0.0357"/></geometry><origin xyz="0.030 0 -0.24275"/><material name="{mat}"/></visual>
    <collision><geometry><box size="0.139 0.050 0.0357"/></geometry><origin xyz="0.030 0 -0.24275"/></collision>
    <inertial><mass value="0.11"/><origin xyz="0 0 -0.112"/><inertia ixx="0.0005" iyy="0.0005" izz="0.00002" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>

'''
    return out


HEADER = '''<?xml version="1.0"?>
<!-- GENERATED by sim/gen_humanoid_v11.py . Do not hand-edit; edit the generator.

     humanoid_v11_atom.urdf : shoulder_v6 (3 DOF, real bearings) + the ATOM arm.

     v3 exports a genuinely mirrored right-hand module, so both shoulders mount
     unrotated and share sign conventions. v6 had to rotate the right module 180
     deg about Z, which flipped the arm fore/aft and needed a patched elbow axis.

     Module mass 933.5 g/side (was 771 g for v2) - the pitch and twist bearings
     and their retainers. ATOM arm 586 g/side.

     SERVOS ARE NOW REAL PARTS, NOT PLACEHOLDERS:
       shoulder pitch/roll/twist : Dynamixel XH540-W270, 10.6 Nm stall, 3.14 rad/s
       elbow                     : Dynamixel XH430-W350,  4.8 Nm stall, 3.40 rad/s
     Earlier versions used the MG996R's 9 Nm STALL figure as if it were a working
     limit. It is not - hobby servos should run near 1/3 of stall continuously.
     The CAD (cad/shoulder_v6) still has MG996R-sized pockets; the servo envelope
     is parameterised there and needs re-exporting for the 65x30x48 Dynamixels.

     Travel limits below are SERVO limits and MUST be re-measured by collision
     sweep; v2's advertised figures did not survive that test.
-->
<robot name="humanoid_v8_atom">

  <mujoco>
    <compiler meshdir="meshes36d/" balanceinertia="true" discardvisual="false"/>
  </mujoco>

  <!-- ATOM PALETTE, sampled from the donor model's own BaseColor maps: torso,
       arms and upper leg all bucket to a neutral gunmetal 0.32, extremities run
       warmer/darker, and the only saturated colour on him is the emissive cyan
       of the chest light and eyes. blue/orange keep their NAMES so side() and
       legs() are untouched, but are weathered metal tints now, not primaries. -->
  <material name="gray"><color rgba="0.32 0.32 0.32 1"/></material>
  <material name="tan"><color rgba="0.30 0.29 0.27 1"/></material>
  <material name="blue"><color rgba="0.20 0.55 0.68 1"/></material>
  <material name="orange"><color rgba="0.72 0.42 0.18 1"/></material>
  <material name="dark"><color rgba="0.14 0.14 0.15 1"/></material>
  <material name="chest"><color rgba="0.22 0.23 0.25 1"/></material>

  <!-- SKELETON torso. The old solid 0.20x0.34x0.65 box was a placeholder that
       also happened to be structural in the sim. Real build is a narrow spine
       with a shell over it, so the spine only has to reach the shoulder module's
       mounting plate: 118.4 mm axis - 89 mm standoff = 29.4 mm half-width. -->
  <link name="torso">
    <visual><geometry><box size="0.050 0.059 0.2634"/></geometry><origin xyz="0 0 0.6163"/><material name="gray"/></visual>
    <collision><geometry><box size="0.050 0.059 0.2634"/></geometry><origin xyz="0 0 0.6163"/></collision>
    <inertial><mass value="0.90"/><origin xyz="0 0 0.6163"/><inertia ixx="0.006" iyy="0.006" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="0.070 0.175 0.055"/></geometry><origin xyz="0 0 0.4846"/><material name="gray"/></visual>
    <collision><geometry><box size="0.070 0.175 0.055"/></geometry><origin xyz="0 0 0.4846"/></collision>
    <visual><geometry><cylinder radius="0.012" length="0.048"/></geometry><origin xyz="0 0 0.772"/><material name="gray"/></visual>
    <!-- HEAD. Atom head lifted from Atom_Real_Steel_Model.obj (Material.005).
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
    <visual><geometry><mesh filename="atom_head.obj"/></geometry><origin xyz="0.020 0 0.84208" rpy="0 -0.209440 0"/><material name="tan"/></visual>
    <visual><origin xyz="0.027 0 0.660"/><geometry><box size="0.004 0.050 0.090"/></geometry><material name="chest"/></visual>
  </link>

'''


def main():
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(HEADER + side("L") + "\n" + side("R") + "\n" + legs() + "</robot>\n")
    print("wrote", os.path.relpath(OUT, ROOT))


if __name__ == "__main__":
    main()
