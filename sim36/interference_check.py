"""
interference_check.py - does this shoulder actually assemble and move?

    python sim36/interference_check.py cad/shoulder_v5

Every shoulder version so far failed the same way: a rotating part sweeps through
space a static part occupies, and it was only ever found by accident.

  v1  bicep swept a 72 mm cylinder into the chest      -> travel died at +/-10 deg
  v2  thrust race unloaded when the arm hung           -> arm hung off horn screws
  v3  pitch cradle sat in the yoke arms' sweep         -> pitch capped at 8 deg
  v4  twist servo body occupied the carrier's volume   -> 23 interfering pairs

This checks it directly: surface point clouds per part, rigid-transformed through
the real joint travel, nearest-neighbour distance between every pair that moves
relative to each other.

NOT an OCC boolean - those eat all available RAM on meshes this size. A KD-tree
over surface samples is exact enough to catch interpenetration and runs in
seconds.

The kinematics come from check_meta.json, written by the build script, so the
checker cannot drift out of sync with the design it is checking. Joint axes do
NOT have to pass through the origin - v5 deliberately offsets them, which is the
whole reason it fits.

Bearing and drive interfaces are excluded by name: a shaft is SUPPOSED to sit
inside its bore, and flagging that is noise.
"""
import json
import os
import struct
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# used only when a folder predates check_meta.json (i.e. shoulder_v4)
LEGACY = {
    "axes":   {"pitch": [0, 1, 0], "roll": [1, 0, 0], "twist": [0, 0, 1]},
    "origin": {"pitch": [0, 0, 0], "roll": [0, 0, 0], "twist": [0, 0, 0]},
    "chain":  {"base": [], "carrier": ["pitch"], "yoke": ["pitch", "roll"],
               "plate": ["pitch", "roll", "twist"]},
    "groups": {"base":    ["mount", "servoP_body", "brg_pitch"],
               "carrier": ["carrier", "servoP_shaft", "servoR_body"],
               "yoke":    ["yoke", "servoR_shaft", "servoT_body", "brg_twist"],
               "plate":   ["interface_plate", "servoT_shaft"]},
    "exclude": {"base|carrier": ["brg_pitch", "servoP_shaft"],
                "carrier|yoke": ["servoR_shaft", "brg_twist"],
                "yoke|plate":   ["servoT_shaft", "brg_twist"]},
    "travel": {"pitch": [-90, -45, 0, 45, 90],
               "roll":  [0, 45, 90, 135, 180],
               "twist": [-90, -45, 0, 45, 90]},
}

# ASSERTION 3: parts in the SAME group were never compared, on the reasoning
# that they are one rigid body so relative motion is zero. True - but they still
# have to physically FIT. Every servo body interfered with its own bracket by
# 2000-3400 mm^3 and this checker reported PASS, because each servo shares a
# group with the bracket that is supposed to hold it. Static overlap is still an
# overlap: a part you cannot install is not assembled.
SAME_GROUP = "--no-same-group" not in sys.argv

MIN_GAP = 1.0          # mm; anything below this will not assemble or will bind


def read_stl(path):
    with open(path, "rb") as f:
        head = f.read(84)
        n = struct.unpack("<I", head[80:84])[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tri = np.zeros((n, 3, 3), dtype=np.float64)
    for k in range(3):
        tri[:, k, :] = raw[:, 12 + k * 12:24 + k * 12].copy().view(np.float32)
    return tri


def surface_points(tri, step=1.2):
    """Vertices, centroids and edge midpoints, then subdivide any triangle big
    enough to hide a clash between samples. A 1.2 mm grid means an overlap
    deeper than about 1 mm cannot slip through unseen."""
    v = tri.reshape(-1, 3)
    c = tri.mean(axis=1)
    e = np.concatenate([(tri[:, 0] + tri[:, 1]) / 2,
                        (tri[:, 1] + tri[:, 2]) / 2,
                        (tri[:, 2] + tri[:, 0]) / 2])
    pts = [v, c, e]
    # barycentric fill for large facets - CadQuery emits some very long ones
    a, b, cc = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, cc - a), axis=1)
    big = area > 4.0
    if big.any():
        ab, bb, cb = a[big], b[big], cc[big]
        for u in (0.25, 0.5, 0.75):
            for w in (0.25, 0.5, 0.75):
                if u + w < 1.0:
                    pts.append(ab + u * (bb - ab) + w * (cb - ab))
    pts = np.concatenate(pts)
    key = np.round(pts / step).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[idx]


def components(tri, tol=0.5):
    """Number of connected bodies in a mesh - welding vertices on a tol grid."""
    V = tri.reshape(-1, 3)
    key = np.round(V / tol).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    parent = np.arange(len(uniq))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for a, b, c in inv.reshape(-1, 3):
        for x, y in ((a, b), (b, c)):
            ra, rb = find(x), find(y)
            if ra != rb:
                parent[ra] = rb
    return len({find(i) for i in range(len(uniq))})


def rot(axis, ang):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def pose(pts, joints, ang, meta):
    """Apply the chain innermost-first. A part on ['pitch','roll'] is rotated by
    roll about the roll axis, and the result is rotated by pitch about the pitch
    axis - not the other way round, and not both about the origin."""
    q = pts
    for j in reversed(joints):
        O = np.asarray(meta["origin"][j], dtype=float)
        q = (q - O) @ rot(meta["axes"][j], ang[j]).T + O
    return q


_FIT_RAYS = [np.array(v) / np.linalg.norm(v) for v in
             ([0.5735, 0.2113, 0.7893], [-0.3187, 0.8011, 0.5065],
              [0.7071, -0.6325, 0.3162])]


def _inside(pts, tri):
    """median parity of 3 skew rays - the only classifier these meshes honor."""
    votes = np.zeros(len(pts), dtype=int)
    for ray in _FIT_RAYS:
        v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
        e1, e2 = v1 - v0, v2 - v0
        h = np.cross(ray, e2)
        a = np.einsum("ij,ij->i", e1, h)
        ok = np.abs(a) > 1e-9
        inv = np.where(ok, 1.0 / np.where(ok, a, 1), 0.0)
        for k, p_ in enumerate(np.asarray(pts, float)):
            sv = p_ - v0
            u = np.einsum("ij,ij->i", sv, h) * inv
            q = np.cross(sv, e1)
            vv = (q @ ray) * inv
            t = np.einsum("ij,ij->i", e2, q) * inv
            if (ok & (u >= 0) & (u <= 1) & (vv >= 0) & (u + vv <= 1)
                    & (t > 1e-7)).sum() % 2 == 1:
                votes[k] += 1
    return votes >= 2


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "cad", "shoulder_v5")
    folder = folder if os.path.isabs(folder) else os.path.join(ROOT, folder)
    mpath = os.path.join(folder, "check_meta.json")
    if os.path.exists(mpath):
        with open(mpath) as f:
            meta = json.load(f)
    else:
        meta = LEGACY
        print("(no check_meta.json - falling back to concurrent-axis defaults)")

    # ASSERTION 1: a part named in check_meta but not exported used to be
    # SILENTLY SKIPPED. brg_pitch and servoT_bracket were missing from every
    # sweep this module ever passed - the pitch bearing was never checked at
    # all, and the housing that should hold it turned out to be 8 mm of air.
    missing = [n for names in meta["groups"].values() for n in names
               if not os.path.exists(os.path.join(folder, f"{n}.stl"))]
    if missing:
        print(f"FAIL: named in check_meta.json but never exported: {missing}")
        print("      A part that does not exist cannot be checked. Export it or "
              "remove it from the groups.")
        sys.exit(1)

    parts = {}
    for grp, names in meta["groups"].items():
        for n in names:
            p = os.path.join(folder, f"{n}.stl")
            parts[n] = (grp, surface_points(read_stl(p)))

    # ASSERTION 2: every exported solid must be ONE connected body. The yoke
    # sliced as three - its two roll arms sat 4 mm clear of the platform they
    # were supposed to carry - and nothing here noticed, because a sweep only
    # ever compares parts to OTHER parts.
    bad = []
    for n in parts:
        k = components(read_stl(os.path.join(folder, f"{n}.stl")))
        if k > 1:
            bad.append(f"{n} ({k} bodies)")
    if bad:
        print(f"FAIL: part(s) are not a single connected solid: {', '.join(bad)}")
        sys.exit(1)
    if not parts:
        print(f"no parts found in {folder}")
        return

    # STATIC SAME-GROUP FIT: the ROM sweep never compares parts that move
    # together, which once let a servo sit 2300 mm^3 deep in its own deck and
    # "pass". At pose zero, each servo body must not interpenetrate the
    # printed part it bolts to.
    FIT_PAIRS = [("servoP_body", "mount"), ("servoR_body", "carrier"),
                 ("servoT_body", "yoke")]
    for sa, sb in FIT_PAIRS:
        pa = os.path.join(folder, sa + ".stl")
        pb = os.path.join(folder, sb + ".stl")
        if not (os.path.exists(pa) and os.path.exists(pb)):
            continue
        ta = read_stl(pa)
        # sample INTERIOR points: triangle centroids pushed 0.6 mm along the
        # inward normal. Vertices are the worst probes (each sits on edges of
        # several faces, and mating faces are exactly coplanar with the deck);
        # a centroid-inward point is unambiguously inside the servo, so it is
        # inside the partner mesh ONLY if the two solids genuinely overlap.
        cen3 = ta.mean(axis=1)
        nrm = np.cross(ta[:, 1] - ta[:, 0], ta[:, 2] - ta[:, 0])
        nlen = np.linalg.norm(nrm, axis=1, keepdims=True)
        keep = nlen[:, 0] > 1e-9
        cen3, nrm, nlen = cen3[keep], nrm[keep], nlen[keep]
        nrm = nrm / nlen                 # CadQuery STL winding is consistent:
        spts = cen3 - 0.6 * nrm          # the cross product IS outward, even
                                         # on concave features like slot bores
        idx = np.random.default_rng(0).choice(len(spts),
                                              min(400, len(spts)),
                                              replace=False)
        spts = spts[idx]
        tb = read_stl(pb)
        buried = int(_inside(spts, tb).sum())
        if buried > 12:
            print(f"FAIL: {sa} interpenetrates {sb} at pose zero "
                  f"({buried}/400 sampled vertices inside). Same-group fit.")
            sys.exit(1)
    travel = {j: np.radians(v) for j, v in meta["travel"].items()}
    poses = [{"pitch": p, "roll": r, "twist": t}
             for p in travel["pitch"] for r in travel["roll"] for t in travel["twist"]]
    print(f"checking {os.path.relpath(folder, ROOT)}  "
          f"({len(parts)} parts, {len(poses)} poses)\n")

    bonded = {tuple(sorted(x)) for x in meta.get("bonded", [])}
    exclude = {}
    for k, v in meta["exclude"].items():
        a, b = k.split("|")
        exclude[(a, b)] = v
        exclude[(b, a)] = v

    worst = {}
    for ang in poses:
        posed = {n: pose(pts, meta["chain"][grp], ang, meta)
                 for n, (grp, pts) in parts.items()}
        trees = {n: cKDTree(q) for n, q in posed.items()}
        names = list(posed)
        for i in range(len(names)):
            for k in range(i + 1, len(names)):
                a, b = names[i], names[k]
                ga, gb = parts[a][0], parts[b][0]
                if ga == gb and not SAME_GROUP:
                    continue                          # same rigid body
                if a in exclude.get((ga, gb), []) or b in exclude.get((ga, gb), []):
                    continue                          # bearing / drive interface
                if tuple(sorted((a, b))) in bonded:
                    continue                          # press-fit or bolted: meant to touch
                dd, ii = trees[a].query(posed[b], k=1)
                j = int(dd.argmin())
                key = (a, b)
                if key not in worst or dd[j] < worst[key][0]:
                    # report in each part's OWN design frame - that is where the
                    # offending feature is written down, so that is where it gets fixed
                    worst[key] = (float(dd[j]),
                                  np.degrees([ang["pitch"], ang["roll"], ang["twist"]]),
                                  parts[b][1][j], parts[a][1][int(ii[j])])

    # per-pair overrides for DESIGNED running clearances (e.g. a horn face
    # rotating 0.5 mm over its pocket floor): check_meta "min_override" maps
    # "partA|partB" (either order) to a reduced-but-still-enforced minimum.
    ovr = {}
    for k, v in meta.get("min_override", {}).items():
        a_, b_ = k.split("|")
        ovr[(a_, b_)] = float(v)
        ovr[(b_, a_)] = float(v)
    bad = {k: v for k, v in worst.items() if v[0] < ovr.get(k, MIN_GAP)}
    print(f"{'part A':18}{'part B':18}{'min gap':>9}   at pitch/roll/twist")
    print("-" * 74)
    for (a, b), rec in sorted(worst.items(), key=lambda x: x[1][0])[:18]:
        d, at = rec[0], rec[1]
        flag = "  <-- INTERFERENCE" if d < ovr.get((a, b), MIN_GAP) else (
            "  (override %.1f)" % ovr[(a, b)] if (a, b) in ovr else "")
        print(f"{a:18}{b:18}{d:8.2f}   {at.round(0)}{flag}")
    if bad:
        print("\nwhere they touch (each point in its own part's design frame):")
        for (a, b), rec in sorted(bad.items(), key=lambda x: x[1][0]):
            pb, pa = rec[2].round(1), rec[3].round(1)
            print(f"  {a} {pa}  <->  {b} {pb}")
    print()
    if bad:
        print(f"FAIL: {len(bad)} pair(s) closer than {MIN_GAP} mm. This will not assemble.")
        sys.exit(1)
    print(f"PASS: every moving pair keeps at least {MIN_GAP} mm through the full travel.")


if __name__ == "__main__":
    main()
