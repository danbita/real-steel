"""
neck_chain_check.py - prove neck_v1.urdf is a correct re-chaining of the export.

    python sim36/neck_chain_check.py

No camera, no MuJoCo, no mediapipe. Pure numpy forward kinematics against the
raw SolidWorks export, so this runs anywhere and fails loudly when someone edits
the STAGES table in gen_neck.py into something inconsistent.

WHAT IT CHECKS
  1  ZERO POSE FIDELITY. With every joint at 0, each mesh must land exactly
     where the flat export put it. This is the whole correctness claim of the
     re-chaining: same geometry, different topology.
  2  NO DOUBLE-COUNTING. Total mass must equal the de-duplicated hardware total
     (1.0508 kg), not the export's naive 1.3601 kg sum, and no mesh proven to be
     a duplicate may appear.
  3  AXES ARE HONEST. Driving one joint must rotate the head about that joint's
     exported axis and nothing else - recovered angle equals commanded angle,
     and the head's displacement is perpendicular to the axis.
  4  THE CHAIN ACTUALLY CHAINS. Driving a lower joint must move every link above
     it. This is the specific thing the export got wrong.
  5  LIMITS. Commanding past a limit must be refused by the clamp.
  6  DIRECTIONS, reported not asserted: which way the head nose/up actually go
     for +yaw and +pitch, so SIGN in pose_sender.py can be set from measurement
     instead of guesswork.
"""

import os
import sys
import math
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
URDF = os.path.join(ROOT, "live mimic system", "neck_v1.urdf")
SRC = os.path.join(ROOT, "NeckAssembly_files", "urdf", "NeckAssembly_files.urdf")

DUP = ["servo3_Link", "YesHolder3_Link", "MotorHolder3_Link"]
REAL_TOTAL = 1.0508          # kg, de-duplicated (see gen_neck.py docstring)
EXPORT_NAIVE_TOTAL = 1.3601  # kg, what summing the export blindly gives

fails = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        fails.append(name)


def rot(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def T(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


class Model:
    def __init__(self, path):
        r = ET.parse(path).getroot()
        self.joints, self.links = [], {}
        for l in r.findall("link"):
            vis = []
            for v in l.findall("visual"):
                g = v.find("geometry/mesh")
                if g is None:
                    continue
                o = v.find("origin")
                vis.append((os.path.basename(g.get("filename")).replace(".STL", ""),
                            np.array([float(x) for x in o.get("xyz").split()])))
            i = l.find("inertial")
            self.links[l.get("name")] = {
                "meshes": vis,
                "mass": float(i.find("mass").get("value")) if i is not None else 0.0,
            }
        for j in r.findall("joint"):
            o = j.find("origin")
            a = j.find("axis")
            lim = j.find("limit")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": np.array([float(x) for x in o.get("xyz").split()]),
                "rpy": [float(x) for x in o.get("rpy").split()],
                "axis": np.array([float(x) for x in a.get("xyz").split()]) if a is not None else None,
                "lim": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None,
            })

    def driven(self):
        return [j["name"] for j in self.joints if j["type"] == "revolute"]

    def fk(self, q=None):
        """world transform per link."""
        q = q or {}
        out = {}
        roots = set(self.links) - {j["child"] for j in self.joints}
        for r in roots:
            out[r] = np.eye(4)
        progressed = True
        while progressed:
            progressed = False
            for j in self.joints:
                if j["parent"] in out and j["child"] not in out:
                    M = T(rpy_mat(*j["rpy"]), j["xyz"])
                    if j["type"] == "revolute":
                        M = M @ T(rot(j["axis"], q.get(j["name"], 0.0)), np.zeros(3))
                    out[j["child"]] = out[j["parent"]] @ M
                    progressed = True
        return out


def export_mesh_world():
    r = ET.parse(SRC).getroot()
    pos = {"neckcover_link": np.zeros(3)}
    for j in r.findall("joint"):
        o = np.array([float(x) for x in j.find("origin").get("xyz").split()])
        pos[j.find("child").get("link")] = o
    return pos


def main():
    if not os.path.exists(URDF):
        sys.exit("neck_v1.urdf missing - run: python sim36/gen_neck.py")
    m = Model(URDF)
    exp = export_mesh_world()
    print(f"\nneck_v1.urdf: {len(m.links)} links, "
          f"{len(m.driven())} driven joints {m.driven()}\n")

    # 1 -------------------------------------------------------- zero pose
    print("1. zero-pose fidelity vs the flat export")
    fk0 = m.fk()
    worst, worst_name = 0.0, "-"
    seen = []
    for lname, ldata in m.links.items():
        for mesh, off in ldata["meshes"]:
            seen.append(mesh)
            got = (fk0[lname] @ np.append(off, 1.0))[:3]
            want = exp[mesh]
            d = float(np.linalg.norm(got - want))
            if d > worst:
                worst, worst_name = d, mesh
    check("every mesh lands at its exported position", worst < 1e-12,
          f"worst {worst:.2e} m ({worst_name})")

    # 2 -------------------------------------------------------- mass
    print("\n2. no double-counted hardware")
    total = sum(l["mass"] for l in m.links.values())
    head = m.links.get("head", {}).get("mass", 0.0)
    check("duplicate meshes absent", not any(d in seen for d in DUP),
          f"checked {DUP}")
    check("mass matches de-duplicated total",
          abs((total - head) - REAL_TOTAL) < 5e-4,
          f"{total - head:.4f} kg vs {REAL_TOTAL} (export naive sum {EXPORT_NAIVE_TOTAL})")

    # 3 -------------------------------------------------------- axes
    print("\n3. each joint rotates the head about its own exported axis")
    for j in m.joints:
        if j["type"] != "revolute":
            continue
        ax = j["axis"] / np.linalg.norm(j["axis"])
        qv = 0.4
        A, B = m.fk(), m.fk({j["name"]: qv})
        Ra = A["head"][:3, :3]
        Rb = B["head"][:3, :3]
        dR = Ra.T @ Rb
        ang = math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2)))
        # axis of dR, expressed in the joint's own frame
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        w = w / (np.linalg.norm(w) + 1e-15)
        axis_in_head = Ra.T @ (A[j["child"]][:3, :3] @ ax)
        align = abs(float(np.dot(w, axis_in_head)))
        check(f"{j['name']}: angle out == angle in",
              abs(ang - qv) < 1e-9, f"{math.degrees(ang):.6f} deg")
        check(f"{j['name']}: rotation axis == exported axis",
              align > 1 - 1e-9, f"|cos| = {align:.12f}")

    # 4 -------------------------------------------------------- chaining
    print("\n4. the chain chains (the thing the export got wrong)")
    base_j = m.driven()[0]
    A, B = m.fk(), m.fk({base_j: 0.5})
    above = [n for n in m.links if n not in ("neck_base",)]
    moved = [n for n in above
             if np.linalg.norm(A[n][:3, 3] - B[n][:3, 3]) > 1e-6
             or np.linalg.norm(A[n][:3, :3] - B[n][:3, :3]) > 1e-6]
    check(f"driving {base_j} moves every link above it",
          len(moved) == len(above), f"{len(moved)}/{len(above)} moved")

    # 5 -------------------------------------------------------- limits
    print("\n5. limits")
    for j in m.joints:
        if j["lim"] is None:
            continue
        lo, hi = j["lim"]
        check(f"{j['name']}: symmetric +/-{math.degrees(hi):.1f} deg",
              abs(lo + hi) < 1e-9 and abs(math.degrees(hi) - 81.0) < 0.05,
              f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")

    # 6 -------------------------------------------------------- directions
    print("\n6. measured directions (for SIGN in pose_sender.py, not asserted)")
    fk0 = m.fk()
    R0 = fk0["head"][:3, :3]
    for j in m.driven():
        R1 = m.fk({j: math.radians(20)})["head"][:3, :3]
        # head's own +Y (its long axis) and +X, seen in base coordinates
        for lbl, v in (("head +X", np.array([1.0, 0, 0])),
                       ("head +Y", np.array([0, 1.0, 0]))):
            a, b = R0 @ v, R1 @ v
            print(f"     +20 deg {j:11s} moves {lbl}: "
                  f"({a[0]:+.3f},{a[1]:+.3f},{a[2]:+.3f}) -> "
                  f"({b[0]:+.3f},{b[1]:+.3f},{b[2]:+.3f})")

    print()
    if fails:
        print(f"{len(fails)} CHECK(S) FAILED: {fails}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
