"""
shoulder_prep.py — turn the CadQuery shoulder into URDF-ready link data.

    python sim/shoulder_prep.py

Does two jobs:
  1. Rescales every shoulder STL from millimetres to metres and writes them
     next to the arm meshes, so the URDF needs no scale attribute.
  2. Computes REAL mass properties per link - volume, centre of mass and the
     full inertia tensor - by integrating over the mesh triangles, then groups
     parts into rigid links and applies per-material densities.

The CAD carries no mass data, so inertia has to come from geometry + an assumed
density. That is a far better estimate than hand-waving a diagonal tensor, but
the densities below are assumptions and are labelled as such.

Link grouping follows the drive paths in shoulder_views.png:
    base   : housing + bearing balls + servo A body      (bolted to torso)
    yaw    : servo A shaft + turntable + yoke + servo B body + idler ball
    pitch  : arm C-bracket + pivot gear + servo B shaft
"""

import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "cad", "shoulder_v1")
DST = os.path.join(ROOT, "real-steel", "live mimic system", "meshes", "shoulder")

MM = 0.001  # source CAD is in millimetres

# [ASSUMPTION] densities, kg/m^3. Printed parts assume solid PLA; a real print
# at ~30% infill would be lighter, which makes these a conservative upper bound.
PLA, STEEL, SERVO, HORN = 1240.0, 7850.0, 1750.0, 2700.0

DENSITY = {
    "housing": PLA, "turntable": PLA, "yoke": PLA, "arm": PLA, "pivot_gear": PLA,
    "balls": STEEL, "idler_ball": STEEL,
    "servoA_body": SERVO, "servoB_body": SERVO,
    "servoA_shaft": HORN, "servoB_shaft": HORN,
}

LINKS = {
    "base":  ["housing", "balls", "servoA_body"],
    "yaw":   ["servoA_shaft", "turntable", "yoke", "servoB_body", "idler_ball"],
    "pitch": ["arm", "pivot_gear", "servoB_shaft"],
}

# Canonical tetrahedron second-moment integral, for the covariance method.
C_CANON = np.array([[2., 1., 1.], [1., 2., 1.], [1., 1., 2.]]) / 120.0


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tri = np.zeros((n, 3, 3), dtype=np.float64)
    for k in range(3):
        tri[:, k, :] = raw[:, 12 + k * 12:24 + k * 12].copy().view(np.float32)
    return tri


def write_stl(path, tri):
    n = len(tri)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", n))
        for t in tri:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(nrm)
            nrm = nrm / ln if ln > 0 else np.zeros(3)
            f.write(struct.pack("<3f", *nrm.astype(np.float32)))
            for v in t:
                f.write(struct.pack("<3f", *v.astype(np.float32)))
            f.write(b"\0\0")


def mass_props(tri):
    """Volume, centroid and inertia tensor about the ORIGIN, for unit density.

    Decomposes the closed mesh into tetrahedra with the origin and sums signed
    contributions, so concavities and internal voids are handled correctly.
    """
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    dets = np.einsum("ij,ij->i", a, np.cross(b, c))       # 6 * signed tet volume
    vol = dets.sum() / 6.0
    centroid = ((a + b + c) * dets[:, None]).sum(0) / (24.0 * vol)

    cov = np.zeros((3, 3))
    for i in range(len(tri)):
        A = np.column_stack((a[i], b[i], c[i]))
        cov += dets[i] * (A @ C_CANON @ A.T)

    inertia = np.trace(cov) * np.eye(3) - cov            # about origin
    return abs(vol), centroid, inertia


def shift_to(inertia_o, mass, com, target):
    """Parallel-axis transfer of an inertia tensor from the origin to `target`."""
    d = com - target
    # first move origin -> com, then com -> target
    to_com = inertia_o - mass * ((com @ com) * np.eye(3) - np.outer(com, com))
    return to_com + mass * ((d @ d) * np.eye(3) - np.outer(d, d))


def main():
    os.makedirs(DST, exist_ok=True)
    parts = {}

    for name in DENSITY:
        tri_mm = read_stl(os.path.join(SRC, f"{name}.stl"))
        tri_m = tri_mm * MM
        write_stl(os.path.join(DST, f"{name}.stl"), tri_m)
        vol, com, inertia_o = mass_props(tri_m)
        rho = DENSITY[name]
        parts[name] = dict(vol=vol, com=com, mass=vol * rho,
                           inertia_o=inertia_o * rho)

    print(f"rescaled {len(parts)} meshes mm -> m into "
          f"{os.path.relpath(DST, ROOT)}\n")
    print(f"{'part':14}{'vol cm3':>9}{'mass g':>9}   centre of mass (m)")
    for n, p in parts.items():
        print(f"  {n:12}{p['vol']*1e6:8.2f}{p['mass']*1000:9.1f}   "
              f"[{p['com'][0]:+.4f} {p['com'][1]:+.4f} {p['com'][2]:+.4f}]")

    print("\n--- combined links ---")
    out = {}
    for link, members in LINKS.items():
        mass = sum(parts[m]["mass"] for m in members)
        com = sum(parts[m]["mass"] * parts[m]["com"] for m in members) / mass
        inertia_o = sum(parts[m]["inertia_o"] for m in members)
        # report about the link's own COM
        I_com = shift_to(inertia_o, mass, com, com)
        out[link] = dict(mass=mass, com=com, I=I_com, members=members)
        print(f"\n{link}:  mass {mass*1000:.1f} g   com "
              f"[{com[0]:+.5f} {com[1]:+.5f} {com[2]:+.5f}]")
        print(f'  <inertial>')
        print(f'    <origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}"/>')
        print(f'    <mass value="{mass:.6f}"/>')
        print(f'    <inertia ixx="{I_com[0,0]:.9f}" ixy="{I_com[0,1]:.9f}" '
              f'ixz="{I_com[0,2]:.9f}"')
        print(f'             iyy="{I_com[1,1]:.9f}" iyz="{I_com[1,2]:.9f}" '
              f'izz="{I_com[2,2]:.9f}"/>')
        print(f'  </inertial>')
        ev = np.linalg.eigvalsh(I_com)
        ok = ev.min() > 0 and (ev[0] + ev[1] > ev[2])
        print(f"  eigenvalues {ev} -> {'valid' if ok else 'INVALID (check mesh)'}")

    total = sum(v["mass"] for v in out.values())
    print(f"\ntotal shoulder module mass: {total*1000:.1f} g per side")
    return out


if __name__ == "__main__":
    main()
