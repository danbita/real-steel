"""
prep_v11.py - mesh + mass prep for the v11 body (36 in skeleton + shoulder_v6).

    python sim36/prep_v11.py

shoulder_v6 is shoulder_v3 resized: same architecture, same concurrent axes at
C = (0, 0, 100) in the module file, but drawn around the 50 x 58 mm bicep
instead of the old 94 x 109 one, and around a MEASURED Feetech STS3215 rather
than an MG996R.

The module is mounted INVERTED in the robot (rotated 180 deg about Y), because
v6 is exported in v3's neutral pose with the arm straight UP. Inverting it makes
module-roll 0 the robot's resting pose, arm hanging - which matters for more
than convenience: at module-roll 180 the twist servo swings inboard through the
roll servo and the pitch journal. Mounted this way the robot simply never visits
that end of the travel.

    upper arm = module 111.0 + bicep chain = 205.3 mm  (target 170.1)

That 35 mm overshoot is real. Two causes stack up: the true STS3215 (45.4 mm
long, spline 12.5 mm off centre, vs the 40/10 MG996R v3 assumed), and the twist
servo raised to SRV_T_Z=180 - the only stack that clears servo R through the
FULL pitch x roll envelope (pitch 180 x roll 140+ combos). It is 3.9% of
stature: visible if you look for it, and the price of the unrestricted ROM.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from shoulder_prep import read_stl, write_stl, mass_props, PLA, STEEL, SERVO, HORN

ARM_SRC = os.path.join(ROOT, "live mimic system", "meshes")
V6_SRC = os.path.join(ROOT, "cad", "shoulder_v6")
DST = os.path.join(ROOT, "live mimic system", "meshes36d")

ARM_S = 0.531
BICEP_CUT = 0.0005           # almost nothing left above the bicep's own origin
MM = 0.001
C_Z = 0.100                  # the three axes meet here in the module file

ARM_MESHES = ["bicep_Link.STL", "elbow_Link.STL", "short_arm_link.STL",
              "big_gear_Link.STL", "small_gear_Link.STL"]
ORIG_MASS = {"bicep_Link.STL": 0.245418330630813, "elbow_Link.STL": 0.158178833168892,
             "short_arm_link.STL": 0.138255156587189,
             "big_gear_Link.STL": 0.0382194359196154,
             "small_gear_Link.STL": 0.00629688353529975}

# the real STS3215 weighs 55 g. Deriving it from a PLA-ish density gets 70 g,
# and there are six of them on the robot, so the error is worth removing.
SERVO_MASS = 0.055

V6_PARTS = ["mount", "servoP_body", "servoP_shaft", "pitch_retainer",
            "carrier", "servoR_body", "servoR_shaft", "roll_idler",
            "yoke", "servoT_body", "brg_twist", "race_cap",
            "interface_plate", "servoT_shaft", "hub_clamp"]
V6_DENS = {"mount": PLA, "carrier": PLA, "yoke": PLA, "interface_plate": PLA,
           "hub_clamp": PLA, "race_cap": PLA, "pitch_retainer": PLA,
           "roll_idler": STEEL, "brg_twist": STEEL,
           "servoP_body": SERVO, "servoR_body": SERVO, "servoT_body": SERVO,
           "servoP_shaft": HORN, "servoR_shaft": HORN, "servoT_shaft": HORN}
# groups must match cad/shoulder_v6/check_meta.json
V6_LINKS = {"base": ["mount", "servoP_body", "pitch_retainer"],
            "carrier": ["servoP_shaft", "carrier", "servoR_body", "roll_idler"],
            "yoke": ["servoR_shaft", "yoke", "servoT_body", "brg_twist", "race_cap"],
            "plate": ["servoT_shaft", "interface_plate", "hub_clamp"]}

DUMP = {}


def slice_below(tri, zc):
    """Keep only triangles lying ENTIRELY below zc. Centroid-testing is not
    enough: long triangles run most of the part's length."""
    return tri[tri[:, :, 2].max(axis=1) <= zc]


def urdf_block(mass, com, I_about_origin, ref=np.zeros(3)):
    Ic = I_about_origin - mass * ((com @ com) * np.eye(3) - np.outer(com, com))
    rel = com - ref
    ev = np.linalg.eigvalsh(Ic)
    ok = ev.min() > 0 and ev[0] + ev[1] > ev[2]
    return rel, mass, Ic, ok


def main():
    os.makedirs(DST, exist_ok=True)
    os.makedirs(os.path.join(DST, "shoulder_v6"), exist_ok=True)

    print("=== ARM (scale 0.531) ===")
    arm_out = {}
    for f in ARM_MESHES:
        tri = read_stl(os.path.join(ARM_SRC, f))
        v0, _, _ = mass_props(tri)
        rho = ORIG_MASS[f] / v0
        if f == "bicep_Link.STL":
            tri = slice_below(tri, BICEP_CUT)
        tri = tri * ARM_S
        write_stl(os.path.join(DST, f), tri)
        v, com, I = mass_props(tri)
        rel, mass, Ic, ok = urdf_block(v * rho, com, I * rho)
        arm_out[f[:-4]] = (rel, mass, Ic)
        print(f"  {f[:-4]:18} mass {mass * 1000:7.2f} g   {'valid' if ok else 'INVALID'}")

    b = read_stl(os.path.join(DST, "bicep_Link.STL")).reshape(-1, 3)
    chain = b[:, 2].max() + 0.178242 * ARM_S

    print("\n=== SHOULDER v6 (mm -> m, no scaling) ===")
    for hand in ("", "right"):
        src = os.path.join(V6_SRC, hand) if hand else V6_SRC
        sfx = "_r" if hand else ""
        for n in V6_PARTS:
            p = os.path.join(src, f"{n}.stl")
            if os.path.exists(p):
                write_stl(os.path.join(DST, "shoulder_v6", f"{n}{sfx}.stl"),
                          read_stl(p) * MM)

    for link, members in V6_LINKS.items():
        tot = 0.0
        com = np.zeros(3)
        Io = np.zeros((3, 3))
        for n in members:
            p = os.path.join(DST, "shoulder_v6", f"{n}.stl")
            if not os.path.exists(p):
                continue
            tri = read_stl(p)
            v, c, I = mass_props(tri)
            rho = V6_DENS[n]
            if n.endswith("_body") and n.startswith("servo"):
                rho = SERVO_MASS / v                 # pin servos to the real 55 g
            m = v * rho
            tot += m
            com += m * c
            Io += I * rho
        com /= tot
        # every link frame sits on the concurrent axis point C
        rel, mass, Ic, ok = urdf_block(tot, com, Io, np.array([0.0, 0.0, C_Z]))
        print(f"\n{link}: mass {mass * 1000:.1f} g  {'valid' if ok else 'INVALID'}")
        DUMP[link] = dict(rel=list(rel), mass=float(mass),
                          ixx=Ic[0, 0], ixy=Ic[0, 1], ixz=Ic[0, 2],
                          iyy=Ic[1, 1], iyz=Ic[1, 2], izz=Ic[2, 2])

    IF_Z1 = 0.202                                     # bicep mount face (short stack: servo T at 169.9)
    print("\n=== DERIVED for gen_humanoid_v11.py ===")
    print(f"  module axial (axis -> bicep face) : {IF_Z1 - C_Z:.4f} m")
    print(f"  bicep chain                       : {chain:.6f} m")
    print(f"  UPPER ARM                         : {IF_Z1 - C_Z + chain:.6f} m  target 0.1701")
    print(f"  BICEP_Z (bicep origin in plate frame) : {IF_Z1 - C_Z - b[:, 2].max():.6f}")
    DUMP["arm"] = {k: dict(rel=list(v[0]), mass=float(v[1]),
                           ixx=v[2][0, 0], ixy=v[2][0, 1], ixz=v[2][0, 2],
                           iyy=v[2][1, 1], iyz=v[2][1, 2], izz=v[2][2, 2])
                   for k, v in arm_out.items()}
    DUMP["chain"] = float(chain)
    DUMP["bicep_top"] = float(b[:, 2].max())
    DUMP["bicep_z"] = float(IF_Z1 - C_Z - b[:, 2].max())
    with open(os.path.join(HERE, "v11_links.json"), "w") as f:
        json.dump(DUMP, f, indent=1)
    print("  wrote sim36/v11_links.json")


if __name__ == "__main__":
    main()
