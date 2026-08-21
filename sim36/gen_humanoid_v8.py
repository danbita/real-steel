"""
gen_humanoid_v8.py — emit humanoid_v8_atom.urdf (shoulder_v3 + ATOM arm).

    python sim/gen_humanoid_v8.py

Generated, not hand-written. Edit THIS file.

WHAT IS DIFFERENT FROM v6
=========================
v6 used shoulder_v2 and had to mount the RIGHT module rotated 180 deg about Z,
because v2 was modelled left-only and its mount plate is on the module's -Y face.
That rotation flipped the arm fore/aft, which is why the right elbow folded
backward and had to be patched with a flipped joint axis.

shoulder_v3 exports a genuinely mirrored right-hand module (cad/shoulder_v3/right),
so BOTH modules now mount with rpy = 0 and share the same sign conventions. The
axis flip hack is gone.

Also new in v3, and reflected here: real bearings on pitch (6808-2RS) and twist
(6810-2RS), which is why the module mass went 771 g -> 933.5 g per side.

GEOMETRY (module frame, mm, from cad/shoulder_v3/build_shoulder_v3_cq.py)
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
OUT = os.path.join(ROOT, "live mimic system", "humanoid_v8_atom.urdf")

SHOULDER_Y = 0.1184      # biacromial/2 at 36 in; spine surface sits 89 mm inboard
MODULE_Z = 0.748         # shoulder (acromion) height = 0.818 x stature
CZ = 0.100
BICEP_Z = 0.075488       # measured back off the built model so C->elbow = 170.1 mm
                         # (the derived 0.090788 was 15 mm long: the scaled plate's
                         #  tray floor does not move linearly with the plate scale)
BICEP_X_OFF = 0.020019   # 37.7 mm x the 0.531 arm scale

M = f"0 0 -{CZ}"

# Left-hand values, measured by sim/shoulder_v3_prep.py. The right module is a
# true mirror, so its COM y and its ixy / iyz products flip sign - which is
# exactly what the prep script measured on the mirrored meshes.
INERTIA_L = {
    # re-measured after the cradle relief (servo P moved to y=-42, cradle ends
    # at y=-45): the mount lost 22 g of material.
    "base": ((0.001429, -0.071071, -0.000039), 0.283397,
             dict(ixx=0.000330974, ixy=-0.000001445, ixz=-0.000000432,
                  iyy=0.000483972, iyz=-0.000000109, izz=0.000248184)),
    "carrier": ((0.003698, -0.012065, -0.000100), 0.145729,
                dict(ixx=0.000055713, ixy=-0.000002528, ixz=-0.000000505,
                     iyy=0.000079962, iyz=-0.000000036, izz=0.000097091)),
    "yoke": ((0.001193, -0.000074, 0.051668), 0.345395,
             dict(ixx=0.000375780, ixy=-0.000001152, ixz=-0.000005373,
                  iyy=0.000531537, iyz=0.000000471, izz=0.000439380)),
    "plate": ((0.0, 0.0, 0.037890), 0.039953,
              dict(ixx=0.000059063, ixy=0.0, ixz=0.0,
                   iyy=0.000058049, iyz=0.0, izz=0.000016907)),
}


def inertia(key, side):
    (cx, cy, cz), mass, I = INERTIA_L[key]
    if side == "R":
        cy = -cy
        I = dict(I, ixy=-I["ixy"], iyz=-I["iyz"])
    return (cx, cy, cz), mass, I


MESHES = {
    "base": ["mount", "servoP_body", "pitch_retainer"],
    "carrier": ["servoP_shaft", "carrier", "hub_collar", "servoR_body"],
    "yoke": ["servoR_shaft", "yoke", "roll_idler", "servoT_body",
             "brg_twist", "race_cap"],
    "plate": ["servoT_shaft", "interface_plate", "hub_clamp"],
}

# Collision primitives from the CadQuery constants, in each link frame
# (module coords minus C). Meshes are concave; MuJoCo would hull the yoke into a
# block that swallows the carrier it straddles.
COLL_L = {
    "base": [
        ("0 -0.081 0", "box", "0.110 0.008 0.140"),          # torso plate
        # servo P cradle, y -89..-45 (matches the relieved CAD). v3.0 ran it out
        # to y=-29, where the yoke arms - which sweep y -40..+40 about the pitch
        # axis - ground into it and capped pitch at 8 deg.
        ("0.001 -0.067 0", "box", "0.066 0.044 0.064"),      # servo P cradle
    ],
    "carrier": [
        ("0 -0.0325 0", "cylY", "0.020 0.007"),              # Ø40 journal
        ("0 -0.0245 0", "cylY", "0.028 0.009"),              # hub shoulder
        ("0 -0.015 0", "box", "0.068 0.014 0.028"),          # bridge
        ("0.031 0 0", "cylX", "0.030 0.006"),                # front plate
        ("-0.031 0 0", "cylX", "0.030 0.006"),               # rear plate
        ("0.026 -0.0047790005 0", "box", "0.008 0.055 0.024"),     # servo R pad
    ],
    "yoke": [
        ("0.039 0 0", "cylX", "0.024 0.006"),
        ("-0.039 0 0", "cylX", "0.024 0.006"),
        ("0.039 0 0.0285", "box", "0.006 0.080 0.073"),      # right arm
        ("-0.039 0 0.0285", "box", "0.006 0.080 0.073"),     # left arm
        ("0 0 0.070", "box", "0.104 0.084 0.014"),           # platform
        ("0.005 0 0.0375", "box", "0.058 0.026 0.051"),      # servo T bracket
    ],
    "plate": [
        ("0 0 0.0451", "box", "0.053 0.062 0.003"),          # interface plate (scaled)
        # The Ø50 journal has NO collision: it rotates inside the platform's
        # bearing bore, and the platform primitive is a solid box, so colliding
        # them reports a permanent 7mm "penetration" at every pose. Same call as
        # the pitch journal and the gears - bearings are not clash candidates.
        # The tray rim is likewise omitted; the bicep sits inside it by design.
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
    # shoulder_v3/right/mount.stl silently collapsed onto shoulder_v3/mount.stl
    # and the right module wore the left module's geometry - its mount plate
    # ended up 114mm outboard of the torso, floating in air.
    sfx = "" if side == "L" else "_r"
    return "\n".join(
        f'    <visual><origin xyz="{M}"/><geometry>'
        f'<mesh filename="shoulder_v3/{n}{sfx}.stl"/></geometry></visual>'
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
    <origin xyz="{bx} 0 {bz}" rpy="3.1415927 0 -1.5707963"/>
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
    tag = "LEFT" if s == "L" else "RIGHT"

    return f'''  <!-- ################### {tag} SHOULDER (v3 module) ################## -->
  <joint name="{s}_shoulder_mount" type="fixed">
    <parent link="torso"/><child link="{s}_shoulder_base"/>
    <origin xyz="0 {y} {MODULE_Z}" rpy="0 0 0"/>
  </joint>

{link_block(s, "base", "shoulder_base")}
  <joint name="{s}_shoulder_pitch" type="revolute">
    <parent link="{s}_shoulder_base"/><child link="{s}_shoulder_carrier"/>
    <origin xyz="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

{link_block(s, "carrier", "shoulder_carrier")}
  <!-- roll = abduction. 0 arm up, 1.5708 straight out, 3.1416 hanging. -->
  <joint name="{s}_shoulder_roll" type="revolute">
    <parent link="{s}_shoulder_carrier"/><child link="{s}_shoulder_yoke"/>
    <origin xyz="0 0 0"/><axis xyz="{roll_axis}"/>
    <limit lower="0" upper="3.2289" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.06" friction="0.02"/>
  </joint>

{link_block(s, "yoke", "shoulder_yoke")}
  <joint name="{s}_shoulder_twist" type="revolute">
    <parent link="{s}_shoulder_yoke"/><child link="{s}_shoulder_plate"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <!-- +/-180: the XH540 does a full turn in position mode. The old +/-90 was
         an arbitrary cap and the retarget saturated against it on 100% of
         frames, costing ~40 deg of forearm error. Wiring is the real limit -
         no slip ring, so leave a service loop and re-cap if it snags. -->
    <limit lower="-3.1416" upper="3.1416" effort="3.0" velocity="4.2"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

{link_block(s, "plate", "shoulder_plate")}
{ARM.format(s=s, bx=BICEP_X_OFF, bz=BICEP_Z, mat=mat)}'''


def legs():
    out = ""
    for s, ax, mat in (("L", "1 0 0", "blue"), ("R", "-1 0 0", "orange")):
        yh = 0.055 if s == "L" else -0.055
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
<!-- GENERATED by sim/gen_humanoid_v8.py . Do not hand-edit; edit the generator.

     humanoid_v8_atom.urdf : shoulder_v3 (3 DOF, real bearings) + the ATOM arm.

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
     The CAD (cad/shoulder_v3) still has MG996R-sized pockets; the servo envelope
     is parameterised there and needs re-exporting for the 65x30x48 Dynamixels.

     Travel limits below are SERVO limits and MUST be re-measured by collision
     sweep; v2's advertised figures did not survive that test.
-->
<robot name="humanoid_v8_atom">

  <mujoco>
    <compiler meshdir="meshes36/" balanceinertia="true" discardvisual="false"/>
  </mujoco>

  <material name="gray"><color rgba="0.45 0.47 0.52 1"/></material>
  <material name="tan"><color rgba="0.85 0.72 0.6 1"/></material>
  <material name="blue"><color rgba="0.2 0.5 0.9 1"/></material>
  <material name="orange"><color rgba="0.95 0.55 0.2 1"/></material>
  <material name="dark"><color rgba="0.12 0.13 0.16 1"/></material>
  <material name="chest"><color rgba="0.30 0.33 0.40 1"/></material>

  <!-- SKELETON torso. The old solid 0.20x0.34x0.65 box was a placeholder that
       also happened to be structural in the sim. Real build is a narrow spine
       with a shell over it, so the spine only has to reach the shoulder module's
       mounting plate: 118.4 mm axis - 89 mm standoff = 29.4 mm half-width. -->
  <link name="torso">
    <visual><geometry><box size="0.050 0.059 0.2634"/></geometry><origin xyz="0 0 0.6163"/><material name="gray"/></visual>
    <collision><geometry><box size="0.050 0.059 0.2634"/></geometry><origin xyz="0 0 0.6163"/></collision>
    <inertial><mass value="0.90"/><origin xyz="0 0 0.6163"/><inertia ixx="0.006" iyy="0.006" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="0.060 0.120 0.050"/></geometry><origin xyz="0 0 0.4846"/><material name="gray"/></visual>
    <collision><geometry><box size="0.060 0.120 0.050"/></geometry><origin xyz="0 0 0.4846"/></collision>
    <visual><geometry><cylinder radius="0.012" length="0.048"/></geometry><origin xyz="0 0 0.772"/><material name="gray"/></visual>
    <visual><geometry><sphere radius="0.0595"/></geometry><origin xyz="0 0 0.855"/><material name="tan"/></visual>
    <!-- front markers: a bare spine and a sphere read identically front and back -->
    <visual><origin xyz="0.050 0.022 0.870"/><geometry><sphere radius="0.0095"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.050 -0.022 0.870"/><geometry><sphere radius="0.0095"/></geometry><material name="dark"/></visual>
    <visual><origin xyz="0.027 0 0.660"/><geometry><box size="0.004 0.050 0.090"/></geometry><material name="chest"/></visual>
  </link>

'''


def main():
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(HEADER + side("L") + "\n" + side("R") + "\n" + legs() + "</robot>\n")
    print("wrote", os.path.relpath(OUT, ROOT))


if __name__ == "__main__":
    main()
