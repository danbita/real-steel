"""Assembly animation, final: follows cad/shoulder_v6's documented ASSEMBLY
ORDER exactly. The bare splines are primitives; the horns are the CAD's own
servo*_shaft meshes - O7.3 hub + O19.7 disc with REAL drilled/tapped holes -
bench-fitted (pitch, roll) or dropped onto the live spline (twist).
Movers support waypoint paths; every leg of every path is penetration-tested
with the normal-signed test. GIF + poster PNG per step.
"""
import math
import os
import sys

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, "sim36")
from interference_check import read_stl, surface_points

import mujoco

D = "cad/shoulder_v6"
OUT = "docs/img_v6"
MM = 0.001
CZ = 100.0
# CONSTANTS EXEC'D FROM THE CAD BUILD SCRIPT'S OWN ASSIGNMENT LINES - the CAD
# file is the single source of truth, expressions included, so a parallel CAD
# edit propagates here automatically on the next run.
import re as _re
_src = open(os.path.join(D, "build_shoulder_v6_cq.py"), encoding="utf-8").read()
_ns = {"math": math}
for _ln in _src.splitlines():
    if _re.match(r"^[A-Z][A-Z_0-9]*(\s*,\s*[A-Z][A-Z_0-9]*)*\s*=\s*[^=]", _ln):
        try:
            exec(_ln.split("#")[0], _ns)
        except Exception:
            pass
def _g(n, default=None):
    v = _ns.get(n, default)
    assert v is not None, n
    return float(v)
PLATE_P_Y0 = _g("PLATE_P_Y0")
CARR_X0 = _g("CARR_X0")
SRV_T_Z = _g("SRV_T_Z")
BRK_Z0 = _g("BRK_Z0")
BRG_P_Y0 = _g("BRG_P_Y0")
HORN_BC = _g("HORN_BC", 8.0)
CAP_BC = _g("CAP_BC", 25.0)
CAP_Z1 = _g("CAP_Z1")
HC_BC = _g("HC_BC")
HC_Z0, HC_Z1 = _g("HC_Z0"), _g("HC_Z1")
SPY0, SPY1 = _g("SPY0"), _g("SPY1")
YOKE_X0 = _g("YOKE_X0")
PLAT_Z1 = _g("PLAT_Z1")
IF_Z0, IF_Z1 = _g("IF_Z0"), _g("IF_Z1")
SRV_P_Y = _g("SRV_P_Y")
SRV_R_X = _g("SRV_R_X")
SRV_H = _g("SRV_H")
SRV_OFF = _g("SRV_OFF")
EAR_DX, EAR_DY = _g("EAR_DX", 49.5), _g("EAR_DY", 10.0)
EAR_P_Y = SRV_P_Y - 12.6      # pitch ear-bottom plane (heads bear here)
EAR_R_X = SRV_R_X - 12.6
EAR_T_Z = SRV_T_Z - 12.6
INSERT_D = _g("INSERT_D", 6.0)
TAB_SEAT = 8.0    # M3x8 ear screw: ear 3.1 + 4.9 into the deck insert, head flush

RGBA = {"mount": (0.92, 0.55, 0.20), "carrier": (0.91, 0.63, 0.24),
        "yoke": (0.85, 0.48, 0.16), "interface_plate": (0.30, 0.32, 0.36),
        "hub_clamp": (0.45, 0.47, 0.52), "race_cap": (0.58, 0.60, 0.65),
        "hub_collar_a": (0.45, 0.47, 0.52), "hub_collar_b": (0.45, 0.47, 0.52),
        "pitch_retainer": (0.45, 0.47, 0.52), "hub_collar": (0.45, 0.47, 0.52),
        "servoP_body": (0.78, 0.18, 0.18), "servoR_body": (0.78, 0.18, 0.18),
        "servoT_body": (0.78, 0.18, 0.18),
        "brg_pitch": (0.42, 0.28, 0.55), "brg_twist": (0.42, 0.28, 0.55),
        "bicep_ref": (0.62, 0.66, 0.72, 0.28),
        "servoT_shaft": (0.10, 0.50, 0.54), "servoP_shaft": (0.10, 0.50, 0.54),
        "servoR_shaft": (0.10, 0.50, 0.54), "idler625": (0.42, 0.28, 0.55)}
HORN = (0.10, 0.50, 0.54, 1.0)
BRG = (0.42, 0.28, 0.55, 1.0)
STEEL = (0.38, 0.40, 0.45)
SHAFT_RAD = {"M2.5": 1.25, "M3": 1.5, "M4": 2.0, "M5": 2.5,
             "INS": _g("INSERT_R", 2.1) + 0.15,
             "M3B": 1.5, "M3C": 1.5, "M3K": 1.5}
HEAD_H = {"M2.5": 2.5, "M3": 3.0, "M4": 4.0, "M5": 5.0, "INS": 0.5,
          "M3B": 1.7,   # button head - the retainer/journal gap is only 2.5
          "M3C": 0.1,   # countersunk flush - the plate passes 1.5 over these
          "M3K": 3.0}   # socket cap driven down a counterbore (twist drive)
NUT_KINDS = {"M3x80", "M5x25"}      # clamp via nut / shoulder, not head-vs-seat
SINK = {"M4x16": 4.5, "M3x20": 3.0, "M3Kx6": 3.0}  # counterbore depth: head sits this far in
NO_CHECK = {"bicep_ref"}            # solid stand-in; the real part is hollow
BRASS = (0.72, 0.58, 0.28)
SCREW_COL = {"M2.5": (0.47, 0.50, 0.60), "M3": STEEL, "M3B": STEEL,
             "M3C": (0.34, 0.36, 0.40), "M3K": STEEL, "M4": (0.30, 0.32, 0.36),
             "M5": (0.25, 0.27, 0.30), "INS": BRASS}
VIEWDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "views")
os.makedirs(VIEWDIR, exist_ok=True)
import json as _json
INSTALLED = []                      # (kind, tip, axis, seat, anchor) - compounding build
# through-case tab holes: the STS3215's 4 full-depth bolt holes, now cut in
# the servo meshes. Screws are M3x45: enter at the case's REAR face, run the
# whole case, and thread into the face-plate inserts.
TAB = [(SRV_OFF + sx * EAR_DX / 2, sy * EAR_DY / 2)
       for sx in (1, -1) for sy in (1, -1)]
# ear screws enter at the ear-bottom plane, cross the 3.1 ear via the O4.6
# slot, and thread 4.9 into the deck insert. Pitch servo is CLOCKED 90 deg:
# its ear line runs along z, width along x.
tabP = [((hy, EAR_P_Y, CZ + hx), (0, 1, 0)) for hx, hy in TAB]
tabR = [((EAR_R_X, hy, CZ - hx), (1, 0, 0)) for hx, hy in TAB]
tabT = [((-hy, hx, EAR_T_Z), (0, 0, 1)) for hx, hy in TAB]
RET_Y0, RET_Y1 = _g("RET_Y0"), _g("RET_Y1")
RET_BC = _g("RET_BC")
ret_holes = [(RET_BC * math.cos(math.radians(a)), RET_Y1,
              CZ + RET_BC * math.sin(math.radians(a))) for a in (0, 120, 240)]
cap_holes = [(CAP_BC * math.cos(math.radians(a)), CAP_BC * math.sin(math.radians(a)), CAP_Z1)
             for a in (45, 135, 225, 315)]
m4_holes = [(sx * 16, sy * 20, IF_Z0) for sx in (1, -1) for sy in (1, -1)]
HORN_DISC_Z0_ = _g("HORN_DISC_Z0", 5.0)   # disc underside above the case top
HORN_FACE_ = _g("HORN_FACE", 7.2)          # hub/disc top = the driven face
P_HEAD = SRV_P_Y + HORN_DISC_Z0_       # pitch drive heads seat on the disc underside
R_HEAD = SRV_R_X + HORN_DISC_Z0_               # roll drive heads: disc underside
TW_HEAD = _g("BRG_T_Z0", 184.5)     # twist drive entry: clamp top face (cb sinks 3)
horn_p = [((HORN_BC * math.cos(math.radians(a)), P_HEAD,
            CZ + HORN_BC * math.sin(math.radians(a))), (0, 1, 0)) for a in (0, 90, 180, 270)]
horn_r = [((R_HEAD, HORN_BC * math.cos(math.radians(a)),
            CZ + HORN_BC * math.sin(math.radians(a))), (1, 0, 0)) for a in (0, 90, 180, 270)]
horn_t = [((HORN_BC * math.cos(math.radians(a)), HORN_BC * math.sin(math.radians(a)),
            TW_HEAD), (0, 0, -1)) for a in (45, 135, 225, 315)]
collar_holes = [(15.5 * math.cos(math.radians(a)), -41.5,
                 CZ + 15.5 * math.sin(math.radians(a))) for a in (0, 120, 240)]
# M3x25 enter at the plate's TOP face (z=194) and run 17.5 mm down the
# journal bores into the clamp - tips shown at the bore entry
hub_bolts = [(HC_BC * math.cos(math.radians(a)), HC_BC * math.sin(math.radians(a)), IF_Z1)
             for a in (0, 120, 240)]
spineL = [(-20, SPY0, 76), (20, SPY0, 76), (-20, SPY0, 124), (20, SPY0, 124)]
spineR = [(-20, SPY1, 76), (20, SPY1, 76), (-20, SPY1, 124), (20, SPY1, 124)]
PARK = 500.0


def ease(t):
    return 3 * t * t - 2 * t * t * t


def cylfromto(name, r, p0, p1, rgba):
    f = lambda p: " ".join(str(v * MM) for v in p)
    rg = " ".join(str(v) for v in rgba)
    return (name, f'<geom name="p_{name}" type="cylinder" size="{r*MM}" '
                  f'fromto="{f(p0)} {f(p1)}" rgba="{rg}"/>')


# primitive stand-ins for the parts whose meshes cannot represent assembly
shaftP = cylfromto("shaftP", 2.95, (0, SRV_P_Y + 1.5, CZ), (0, SRV_P_Y + 5.1, CZ), (0.75, 0.76, 0.78, 1))
shaftR = cylfromto("shaftR", 2.95, (SRV_R_X + 1.5, 0, CZ), (SRV_R_X + 5.1, 0, CZ), (0.75, 0.76, 0.78, 1))
shaftT = cylfromto("shaftT", 2.95, (0, 0, SRV_T_Z + 1.5), (0, 0, SRV_T_Z + 5.1), (0.75, 0.76, 0.78, 1))
PRIM_POS = {"spinebox": (0.0, (_g("SPY0") + _g("SPY1")) / 2, CZ)}


class Scene:
    def __init__(self, parts, screws, prims=(), extra="", fixed=(), ghost=()):
        assets, geoms = [], []
        mesh_parts = [n for n in parts if n in RGBA]
        for n in sorted(set(mesh_parts)):
            assets.append(f'<mesh name="{n}" file="{os.path.abspath(os.path.join(D, n + ".stl"))}" scale="0.001 0.001 0.001"/>')
        for n in mesh_parts:
            col = RGBA[n]
            al = col[3] if len(col) > 3 else 1
            if n in ghost:
                al = 0.25
            geoms.append(f'<geom name="p_{n}" type="mesh" mesh="{n}" rgba="{col[0]} {col[1]} {col[2]} {al}" contype="0" conaffinity="0"/>')
        prim_names = []
        for name, xml in prims:
            geoms.append(xml)
            prim_names.append(name)
        self.screws = screws
        f = lambda p: f"{p[0]*MM} {p[1]*MM} {p[2]*MM}"
        for i, sc_ in enumerate(screws):
            kind, tip, axis, gap = sc_[0], sc_[1], sc_[2], sc_[3]
            pre = sc_[5] if len(sc_) > 5 else None
            size, ln = kind.split("x")
            ln = float(ln)
            r = SHAFT_RAD[size]
            col = SCREW_COL.get(size, STEEL)
            hr = r * (1.02 if size == "INS" else 2.0 if size == "M3C"
                      else 1.9 if size == "M3B" else 1.85)
            hh = HEAD_H.get(size, max(2.2, r * 1.4))
            if size == "M3C":
                hh = 0.3                       # countersunk: flush thin ring
            a = np.array(axis, float)
            a /= np.linalg.norm(a)
            tip = np.array(tip, float)
            h0 = tip - a * ln
            h1 = h0 - a * hh
            geoms.append(f'<geom name="s{i}a" type="cylinder" size="{r*MM}" fromto="{f(tip)} {f(h0)}" rgba="{col[0]} {col[1]} {col[2]} 1"/>')
            geoms.append(f'<geom name="s{i}b" type="cylinder" size="{hr*MM}" fromto="{f(h0)} {f(h1)}" rgba="{col[0]*0.8} {col[1]*0.8} {col[2]*0.8} 1"/>')
            gl = pre if pre is not None else gap
            geoms.append(f'<geom name="g{i}" type="cylinder" size="{0.4*MM}" fromto="{f(tip - a*gl)} {f(tip + a*6)}" rgba="0.45 0.5 0.58 0.42"/>')
            # hex-socket dot on driven heads: the visual cue for "this is a
            # machine screw you drive", absent on inserts
            if size != "INS":
                h2 = h1 - a * 0.3
                geoms.append(f'<geom name="s{i}c" type="cylinder" size="{r*0.62*MM}" fromto="{f(h1)} {f(h2)}" rgba="0.10 0.11 0.13 1"/>')
            else:
                geoms.append(f'<geom name="s{i}c" type="cylinder" size="{1.55*MM}" fromto="{f(h1)} {f(h1 - a*0.12)}" rgba="0.12 0.10 0.08 1"/>')
        self.fixmap = {}
        for i, fs in enumerate(fixed):
            kind, tip, axis, seat, anc = fs
            size, ln = kind.split("x")
            ln = float(ln)
            r = SHAFT_RAD[size]
            a = np.array(axis, float)
            a /= np.linalg.norm(a)
            t2 = np.array(tip, float) + a * seat        # seated position
            h0 = t2 - a * ln
            col = BRASS if size == "INS" else STEEL
            hr = r * (1.02 if size == "INS" else 1.85)
            h1 = h0 - a * HEAD_H.get(size, max(2.2, r * 1.4))
            geoms.append(f'<geom name="p_fx{i}a" type="cylinder" size="{r*MM}" fromto="{f(t2)} {f(h0)}" rgba="{col[0]} {col[1]} {col[2]} 1"/>')
            geoms.append(f'<geom name="p_fx{i}b" type="cylinder" size="{hr*MM}" fromto="{f(h0)} {f(h1)}" rgba="{col[0]*0.8} {col[1]*0.8} {col[2]*0.8} 1"/>')
            if size == "INS":
                geoms.append(f'<geom name="p_fx{i}c" type="cylinder" size="{1.55*MM}" fromto="{f(h1)} {f(h1 - a*0.12)}" rgba="0.12 0.10 0.08 1"/>')
            else:
                geoms.append(f'<geom name="p_fx{i}c" type="cylinder" size="{r*0.62*MM}" fromto="{f(h1)} {f(h1 - a*0.3)}" rgba="0.10 0.11 0.13 1"/>')
            self.fixmap[i] = anc
            prim_names.append(f"fx{i}a")
            prim_names.append(f"fx{i}b")
            prim_names.append(f"fx{i}c")
        xml = f"""<mujoco><visual><headlight ambient="0.5 0.5 0.5" diffuse="0.5 0.5 0.5"/>
  <global offwidth="1000" offheight="880"/></visual>
  <asset>{''.join(assets)}<texture type="skybox" builtin="gradient" rgb1="0.99 0.99 1.0" rgb2="0.90 0.92 0.95" width="64" height="64"/></asset>
  <worldbody><light pos="0.4 -0.6 0.8" dir="-0.4 0.6 -0.7" diffuse="0.55 0.55 0.55"/>
  <light pos="-0.5 0.4 0.6" dir="0.5 -0.4 -0.5" diffuse="0.35 0.35 0.35"/>
  <body>{''.join(geoms)}{extra}</body></worldbody></mujoco>"""
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        self.base = {}
        for n in mesh_parts + prim_names:
            g = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, f"p_{n}")
            self.base[n] = (g, self.m.geom_pos[g].copy())
        self.sg = []
        for i in range(len(screws)):
            ids = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, nm)
                   for nm in (f"s{i}a", f"s{i}b", f"g{i}", f"s{i}c")]
            self.sg.append([(g, self.m.geom_pos[g].copy()) for g in ids])
        self.rend = mujoco.Renderer(self.m, 880, 1000)

    def set_part(self, n, off_mm):
        g, b0 = self.base[n]
        self.m.geom_pos[g] = b0 + np.array(off_mm, float) * MM

    def screw_state(self, i, state, back_mm=0.0):
        axis = np.array(self.screws[i][2], float)
        axis /= np.linalg.norm(axis)
        ents = self.sg[i]
        if state == "parked":
            off = np.array([0, 0, PARK]) * MM
            for g, p0 in ents:
                self.m.geom_pos[g] = p0 + off
        elif state == "seated":
            seat_ = self.screws[i][4] if len(self.screws[i]) > 4 else 5.5
            off = axis * seat_ * MM
            park = np.array([0, 0, PARK]) * MM
            for k, (g, p0) in enumerate(ents):
                self.m.geom_pos[g] = (p0 + park) if k == 2 else p0 + off
        else:
            off = -axis * back_mm * MM
            for k, (g, p0) in enumerate(ents):
                self.m.geom_pos[g] = p0 if k == 2 else p0 + off

    def frame(self, view, center, dist):
        mujoco.mj_forward(self.m, self.d)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = center
        cam.distance = dist
        cam.azimuth, cam.elevation = view
        self.rend.update_scene(self.d, camera=cam)
        return Image.fromarray(self.rend.render())

    def close(self):
        self.rend.close()


_tris, _sn = {}, {}
def tris(n):
    if n not in _tris:
        _tris[n] = read_stl(os.path.join(D, n + ".stl"))
    return _tris[n]


def inside_pts(pts, part):
    if part not in _sn:
        t = tris(part)
        cen = t.mean(axis=1)
        nr = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        nr /= (np.linalg.norm(nr, axis=1, keepdims=True) + 1e-12)
        _sn[part] = (cKDTree(cen), cen, nr)
    tree, cen, nrm = _sn[part]
    d, idx = tree.query(pts, k=1)
    s = np.einsum("ij,ij->i", pts - cen[idx], nrm[idx])
    return int(((s < -0.5) & (d < 25.0)).sum())


def screw_cyl_hits(pts, kind, tip, axis, seat, off=(0, 0, 0)):
    """points inside an installed screw's body (shaft + head), analytic."""
    size, ln = kind.split("x")
    ln = float(ln)
    r = SHAFT_RAD[size]
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    t2 = np.array(tip, float) + a * seat + np.array(off, float)
    d = np.asarray(pts, float) - t2
    t = d @ a
    rad = np.linalg.norm(d - np.outer(t, a), axis=1)
    shaft = (t >= -ln) & (t <= 0) & (rad < r + 0.2)
    head = (t >= -ln - HEAD_H.get(size, 3.0)) & (t < -ln) & (rad < r * 1.85 + 0.2)
    return int((shaft | head).sum())


def iron_dir(tip, axis, placed):
    """clear approach direction for the heat-set iron: straight along the
    insert axis if possible, else tilted up to 30 deg - angled access counts."""
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    u = np.cross(a, [0, 0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(a, [0, 1.0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    cands = [-a]
    for tilt in (15, 30):
        st = math.sin(math.radians(tilt))
        ct = math.cos(math.radians(tilt))
        for az in range(0, 360, 45):
            w = math.cos(math.radians(az)) * u + math.sin(math.radians(az)) * v
            cands.append(-a * ct + w * st)
    tipv = np.array(tip, float)
    for d in cands:
        pts = []
        for dep, r_ in ((6, 2.8), (14, 2.8), (24, 9.0), (45, 9.0), (70, 9.0)):
            c = tipv + d * dep
            pts.append(c)
            for th in np.linspace(0, 2 * math.pi, 6, endpoint=False):
                w2 = math.cos(th) * u + math.sin(th) * v
                pts.append(c + w2 * r_)
        blocked = False
        for n in placed:
            if n not in RGBA or n in NO_CHECK:
                continue
            if inside_exact(np.array(pts), n) >= 2:
                blocked = True
                break
        if not blocked:
            return d
    return None


def seg_pen(movers, o0, o1, placed, press, fixed=(), riders_fixed=()):
    real = [p for p in movers if p in RGBA]
    placed = [p for p in placed if p in RGBA]
    if not real or not placed:
        return 0, ""
    perpart = []
    for nm in real:
        P_ = surface_points(tris(nm))
        if len(P_) > 120:               # per PART, not per assembly: a thin
            P_ = P_[np.random.default_rng(0).choice(len(P_), 120, replace=False)]
        perpart.append(P_)              # horn once got ~20 points and its 1.5mm
    M = np.concatenate(perpart)         # plate gouge sailed through unseen
    o0, o1 = np.array(o0, float), np.array(o1, float)
    end = 0.15 if any(p in real for p in press) else 0.0
    worst, where = 0, ""
    for n in placed:
        for t in np.linspace(0.0, 1.0 - end, 6):
            P = M + o0 + (o1 - o0) * t
            c = inside_pts(P, n)
            if c:                       # cheap screen flagged: confirm exactly.
                c = inside_exact(P, n)  # normal-signed lies near open channels
            if c > worst:
                worst, where = c, f"{'+'.join(real)[:26]} in {n}"
    fixed = [fs for fs in fixed if not fs[0].startswith("INS")]
    for fs in fixed:                    # installed screws are obstacles too
        for t in np.linspace(0.0, 1.0 - end, 6):
            P = M + o0 + (o1 - o0) * t
            c = screw_cyl_hits(P, fs[0], fs[1], fs[2], fs[3])
            if c > worst:
                worst, where = c, f"{'+'.join(real)[:26]} in screw {fs[0]}@{fs[4]}"
    # RIDING screws (their anchor moves with this mover): sweep their exposed
    # shaft+head bodies against every placed mesh - proud heads are real
    # geometry and gouge like anything else.
    for fs in riders_fixed:
        kind, tip, axis, seat = fs[0], np.array(fs[1], float), np.array(fs[2], float), fs[3]
        size, ln = kind.split("x")
        ln = float(ln)
        a = axis / np.linalg.norm(axis)
        r = SHAFT_RAD.get(size, 1.5)
        hh = HEAD_H.get(size, 3.0)
        t2 = tip + a * seat
        exposed = max(ln - seat + hh, hh)           # what stands proud
        u = np.cross(a, [0, 0, 1.0])
        if np.linalg.norm(u) < 1e-6:
            u = np.cross(a, [0, 1.0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(a, u)
        Sp = []
        for dd in np.linspace(0.2, exposed, 4):
            c0 = t2 - a * (seat - (ln - seat) - 0) - a * 0  # head end ref
            c0 = tip - a * (ln - seat) - a * dd + a * (ln - seat)  # simplify below
        Sp = []
        head_end = t2 - a * ln - a * hh
        for dd in np.linspace(0.0, exposed, 5):
            c0 = head_end + a * dd
            for th in np.linspace(0, 2 * math.pi, 6, endpoint=False):
                Sp.append(c0 + (r * 1.85 + 0.1) * (math.cos(th) * u + math.sin(th) * v))
        Sp = np.array(Sp)
        for t in np.linspace(0.0, 1.0 - end, 6):
            P = Sp + o0 + (o1 - o0) * t
            for nm in placed:
                if nm not in RGBA or nm in NO_CHECK:
                    continue
                c = inside_pts(P, nm)
                if c:
                    c = inside_exact(P, nm)
                if c >= 3:               # 30-pt cloud per screw: 3+ = real
                    worst = max(worst, 15 + c)   # force over the part gate
                    where = f"riding screw {fs[0]}@{fs[4]} in {nm}"
    return worst, where


SERVO_MESHES = set()      # servo meshes now carry their real through-holes


_RAYS = [np.array(v) / np.linalg.norm(v) for v in
         ([0.5735, 0.2113, 0.7893], [-0.3187, 0.8011, 0.5065],
          [0.7071, -0.6325, 0.3162])]


def _parity(pts, tri, ray):
    v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    h = np.cross(ray, e2)
    a = np.einsum("ij,ij->i", e1, h)
    ok = np.abs(a) > 1e-9
    inv = np.where(ok, 1.0 / np.where(ok, a, 1), 0.0)
    out = np.zeros(len(pts), bool)
    for k, p_ in enumerate(pts):
        sv = p_ - v0
        u = np.einsum("ij,ij->i", sv, h) * inv
        q = np.cross(sv, e1)
        v = (q @ ray) * inv
        t = np.einsum("ij,ij->i", e2, q) * inv
        hit = ok & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > 1e-7)
        out[k] = hit.sum() % 2 == 1
    return out


def inside_exact(pts, part):
    """median parity of 3 skew rays: immune to the bore-rim fan triangles that
    fool the normal test AND to single tessellation cracks. Used for screw
    verification, where points sit inside narrow bores next to pierced faces."""
    T = tris(part)
    votes = sum(_parity(np.asarray(pts), T, r).astype(int) for r in _RAYS)
    return int((votes >= 2).sum())


def screw_check(kind, tip, axis, placed, label, seat=5.0, pre=None, fixed=()):
    """A screw must end up IN A REAL BORE - not in solid, not in air.

    COLLISION: shaft samples along the seated depth must not sit inside any
    placed part. M2.5 screws SELF-TAP into deliberately undersized O2.1 bores,
    so their shafts are sampled at the bore's minor radius, not the thread's.
    ALIGNMENT: a ring 0.7 mm outside the shaft is sampled in depth SLABS; the
    screw passes if any slab is >=60% surrounded - so counterbores and insert
    cavities along the way cannot mask a genuinely aligned bore.
    """
    size, ln = kind.split("x")
    ln = float(ln)
    r = SHAFT_RAD[size]
    # prototype mode (CAD INSERT_R < 2): machine screws thread-form into O2.5
    # pilots, so their flanks legitimately occupy the pilot wall - sample at
    # the core, exactly like the M2.5 self-tap rule.
    _proto = _g("INSERT_R", 2.1) < 2.0
    r_samp = 0.75 if size == "M2.5" else (min(r * 0.85, 1.0) if _proto
                                          else r * 0.85)
    placed = [n for n in placed if n not in NO_CHECK]
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    tipv = np.array(tip, float)
    u = np.cross(a, [0.0, 0.0, 1.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(a, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    depths = np.linspace(0.8, seat - 0.3, 6)
    ring_by_depth = []
    shaft = []
    for d in depths:
        c = tipv + a * d
        ringrow = []
        for th in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            w = math.cos(th) * u + math.sin(th) * v
            shaft.append(c + w * r_samp)
            ringrow.append(c + w * (r + 0.7))
        ring_by_depth.append(np.array(ringrow))
    shaft = np.array(shaft)
    hits = 0
    slab = 0.0
    for n in placed:
        if n not in RGBA:
            continue
        hits += inside_exact(shaft, n)
    for row in ring_by_depth:
        got = 0
        for n in placed:
            if n not in RGBA:
                continue
            got += inside_exact(row, n)
        slab = max(slab, got / len(row))
    bad = []
    if hits > 2:
        bad.append(f"shaft in solid ({hits}pts)")
    if slab < 0.6:
        bad.append(f"no bore slab ({slab:.0%})")
    # LENGTH: head must clamp on the entry surface exactly when the tip seats.
    if kind not in NUT_KINDS and size != "INS":
        slack = ln + SINK.get(kind, 0.0) - seat
        if not -0.05 <= slack <= 0.65:
            bad.append(f"length/seat mismatch ({slack:+.1f}mm)")
    # CORRIDOR: the whole screw (or its preloaded stub) + driver head must have
    # a straight clear approach behind the hole. Blocked = cannot be inserted.
    hh = HEAD_H.get(size, 3.0)
    need = (pre + hh + 0.5) if pre is not None else (ln + hh + 1.0)
    if size == "INS":
        need = ln + 12.0                            # room for the iron's tip
    rh = r * 1.85 + 0.4
    cds = np.arange(1.0, need, 3.0)
    cpts = []
    for d in cds:
        c = tipv - a * d
        cpts.append(c)
        for th in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            w = math.cos(th) * u + math.sin(th) * v
            cpts.append(c + w * rh)
    cpts = np.array(cpts)
    chits = np.zeros(len(cpts), dtype=int)
    for n in placed:
        if n not in RGBA:
            continue
        T = tris(n)
        chits += (sum(_parity(cpts, T, r_) .astype(int) for r_ in _RAYS) >= 2)
    per = chits.reshape(len(cds), 9).sum(axis=1)
    blk = [d for d, c in zip(cds, per) if c >= 2]
    fx_blk = []
    for fs in fixed:
        if fs[0].startswith("INS"):
            continue
        if screw_cyl_hits(cpts, fs[0], fs[1], fs[2], fs[3]) >= 2:
            fx_blk.append(fs[4])
    if blk:
        bad.append(f"corridor blocked {blk[0]:.0f}mm behind hole (need {need:.0f})")
    if fx_blk:
        bad.append(f"corridor hits installed screw on {fx_blk[0]}")
    tag = " (preloaded)" if pre is not None else ""
    print(f"      screw {label:22} {kind:7} {'OK' + tag if not bad else '; '.join(bad)}")
    return not bad


def screw_visible(sc, i, view, center, dist):
    """pixel-diff: does moving this screw change the image at this view?"""
    for j in range(len(sc.screws)):
        sc.screw_state(j, "parked")
    base = np.asarray(sc.frame(view, center, dist), dtype=np.int16)
    sc.screw_state(i, "active", sc.screws[i][3] * 0.5)
    on = np.asarray(sc.frame(view, center, dist), dtype=np.int16)
    sc.screw_state(i, "parked")
    return int((np.abs(on - base).sum(axis=2) > 24).sum())


CAND_VIEWS = [(az, el) for az in range(-180, 180, 30) for el in (-35, -12, 18, 40)]


def screw_cam(center, dist, tip_mm, blend=0.55, zoom=0.55):
    # close-up for screw shots: look toward the screw, pull the camera in
    t = np.array(tip_mm, float) * MM
    return (1 - blend) * np.asarray(center) + blend * t, dist * zoom


def make_step(gifname, placed, movers, screws, views, press=(), prims=(),
              extra="", anchor=None, riders=(), screw_ride=False,
              screw_hold=False, engage=None, ghost=()):
    """movers: (parts, path) pairs, path offsets ending at (0,0,0). Screws one
    at a time. COMPOUNDING: screws installed by earlier steps whose anchor part
    is in this scene are rendered seated, ride with their anchor if it moves,
    and are collision obstacles for movers and new-screw corridors. riders:
    (kind, tip, axis, pre, mover_idx) - screws dropped into a part BEFORE it is
    inserted (no corridor exists afterwards), travelling with that mover.
    screw_ride: this step's screws ride the colinear mover at their preload
    depth, then drive home."""
    only = os.environ.get("ONLY")
    if only and only != gifname:
        for sc_ in screws:                     # bookkeeping only, no render
            INSTALLED.append((sc_[0], sc_[1], sc_[2],
                              sc_[4] if len(sc_) > 4 else 5.5,
                              anchor or placed[0]))
        return
    cfg = {}
    cfgf = os.path.join(VIEWDIR, gifname.replace(".gif", ".json"))
    if os.path.exists(cfgf):
        cfg = _json.load(open(cfgf))
        if "views" in cfg:
            views = [((v[0][0], v[0][1]), v[1]) for v in cfg["views"]]
    allparts = list(placed) + [p for ps, _ in movers for p in ps]
    FIXED = [e for e in INSTALLED if e[4] in allparts]
    prims = list(prims)
    movers = [(list(ps), path) for ps, path in movers]
    f3 = lambda pt: f"{pt[0]*MM} {pt[1]*MM} {pt[2]*MM}"
    for j, (rk, rt, ra, rpre, ridx) in enumerate(riders):
        rsz, rln = rk.split("x")
        rln = float(rln)
        rr = SHAFT_RAD[rsz]
        av = np.array(ra, float)
        av /= np.linalg.norm(av)
        t0 = np.array(rt, float) - av * rpre
        h0 = t0 - av * rln
        h1 = h0 - av * max(2.2, rr * 1.4)
        prims.append((f"rd{j}a", f'<geom name="p_rd{j}a" type="cylinder" size="{rr*MM}" fromto="{f3(t0)} {f3(h0)}" rgba="{STEEL[0]} {STEEL[1]} {STEEL[2]} 1"/>'))
        prims.append((f"rd{j}b", f'<geom name="p_rd{j}b" type="cylinder" size="{rr*1.85*MM}" fromto="{f3(h0)} {f3(h1)}" rgba="{STEEL[0]*0.8} {STEEL[1]*0.8} {STEEL[2]*0.8} 1"/>'))
        movers[ridx][0].extend([f"rd{j}a", f"rd{j}b"])
    tguides = []
    for ps, path in movers:             # travel guide line for every mover
        ref = next((p_ for p_ in ps if p_ in RGBA), None)
        if ref is not None:
            c0 = tris(ref).reshape(-1, 3).mean(axis=0)
        elif ps and ps[0] in PRIM_POS:
            c0 = np.array(PRIM_POS[ps[0]], float)
        else:
            continue
        wps = [np.array(w, float) for w in path] + [np.zeros(3)]
        for a_, b_ in zip(wps[:-1], wps[1:]):
            if np.linalg.norm(a_ - b_) < 1e-6:
                continue
            gn_ = f"tg{len(tguides)}"
            prims.append((gn_, f'<geom name="p_{gn_}" type="cylinder" size="{0.4*MM}" '
                          f'fromto="{f3(c0 + a_)} {f3(c0 + b_)}" '
                          f'rgba="0.45 0.5 0.58 0.42"/>'))
            tguides.append(gn_)
    irons = {}
    iron_fail = []
    for i, sc_ in enumerate(screws):
        if sc_[0].startswith("INS"):
            d_ = iron_dir(sc_[1], sc_[2], allparts)
            if d_ is None:
                iron_fail.append(i)
                continue
            irons[i] = d_
            t0 = np.array(sc_[1], float)
            prims.append((f"irA{i}", f'<geom name="p_irA{i}" type="cylinder" size="{2.5*MM}" fromto="{f3(t0 + d_*2)} {f3(t0 + d_*20)}" rgba="0.85 0.84 0.80 1"/>'))
            prims.append((f"irB{i}", f'<geom name="p_irB{i}" type="cylinder" size="{8.5*MM}" fromto="{f3(t0 + d_*20)} {f3(t0 + d_*75)}" rgba="0.16 0.18 0.22 1"/>'))
    sc = Scene(allparts, screws, prims, extra, fixed=FIXED, ghost=ghost)
    for i in irons:
        sc.set_part(f"irA{i}", (0, 0, 600))
        sc.set_part(f"irB{i}", (0, 0, 600))

    def set_group(ps, off):
        for p_ in ps:
            sc.set_part(p_, off)
        for i2, anc in sc.fixmap.items():
            if anc in ps:
                sc.set_part(f"fx{i2}a", off)
                sc.set_part(f"fx{i2}b", off)
    mujoco.mj_forward(sc.m, sc.d)
    lo, hj = np.full(3, 1e9), np.full(3, -1e9)
    for gi in range(sc.m.ngeom):
        c, r = sc.d.geom_xpos[gi], sc.m.geom_rbound[gi]
        if c[2] < 0.4:
            lo, hj = np.minimum(lo, c - r), np.maximum(hj, c + r)
    center = (lo + hj) / 2
    dist = float(np.linalg.norm(hj - lo)) * 0.98 * cfg.get("dist", 1.0)
    szoom = cfg.get("screw_zoom", 0.55)
    sblend = cfg.get("screw_blend", 0.55)
    vis_thresh = 100 if cfg.get("still") else 800

    def viewfor(si):
        v = views[0][0]
        for vv, start in views:
            if si >= start:
                v = vv
        return v

    frames = []
    placed_now = list(placed)
    ok = True
    for i in iron_fail:
        ok = False
        print(f"      IRON cannot reach insert {gifname[:-4]}#{i} at any angle")
    # ---- screw physics + visibility, BEFORE rendering anything ----
    all_parts = list(placed) + [p for ps, _ in movers for p in ps]
    view_over = {}
    for i, sc_ in enumerate(screws):
        kind, tip, axis, gap = sc_[0], sc_[1], sc_[2], sc_[3]
        seat = sc_[4] if len(sc_) > 4 else 5.5
        pre = sc_[5] if len(sc_) > 5 else None
        if not screw_check(kind, tip, axis, all_parts, f"{gifname[:-4]}#{i}",
                           seat=seat, pre=pre, fixed=FIXED):
            ok = False
        scen, sdst = screw_cam(center, dist, tip, sblend, szoom)
        vis = screw_visible(sc, i, viewfor(i), scen, sdst)
        if vis < vis_thresh:
            best, bv = viewfor(i), vis
            for cv in CAND_VIEWS:
                c2 = screw_visible(sc, i, cv, scen, sdst)
                if c2 > bv:
                    best, bv = cv, c2
            view_over[i] = best
            print(f"      screw {gifname[:-4]}#{i} invisible at step view "
                  f"({vis}px) -> moved to az/el {best} ({bv}px)")
    for k, (ps, path) in enumerate(movers):
        path = [np.array(p, float) for p in path] + [np.zeros(3)]
        fixed_obs = [fs for fs in FIXED if fs[4] not in ps]
        for a, b in zip(path[:-1], path[1:]):
            ride_fs = [fs for fs in FIXED if fs[4] in ps
                       and not fs[0].startswith("INS")]
            pen, where = seg_pen(ps, a, b, placed_now, press, fixed=fixed_obs,
                                 riders_fixed=ride_fs)
            if pen >= 15:
                ok = False
                print(f"    XX PENETRATION {pen} pts  {where}  seg {a}->{b}")
        for i in range(len(screws)):
            if (screw_ride or screw_hold) and len(screws[i]) > 5:
                sc.screw_state(i, "active", screws[i][5])
            else:
                sc.screw_state(i, "parked")
        for ps2, path2 in movers[k + 1:]:
            set_group(ps2, path2[0])
        segf = max(3, int(round(cfg.get("segf", 9) / len(path))))
        for a, b in zip(path[:-1], path[1:]):
            for t in np.linspace(0, 1, segf, endpoint=False):
                off = a + (b - a) * ease(t)
                set_group(ps, off)
                if screw_ride:
                    for i in range(len(screws)):
                        if len(screws[i]) > 5:
                            sc.screw_state(i, "active",
                                           screws[i][5] + np.linalg.norm(off))
                frames.append(sc.frame(viewfor(-1), center, dist))
        set_group(ps, (0, 0, 0))
        frames.append(sc.frame(viewfor(-1), center, dist))
        if engage is not None and engage[1] == k:
            ev = engage[2] if len(engage) > 2 else viewfor(-1)
            ebl = engage[3] if len(engage) > 3 else 0.72
            ezo = engage[4] if len(engage) > 4 else 0.38
            ecen, edst = screw_cam(center, dist, engage[0], ebl, ezo)
            a_, b_ = path[-2], path[-1]
            for t in np.linspace(0, 1, 6):
                off = a_ + (b_ - a_) * ease(t)
                set_group(ps, off)
                if screw_ride:
                    for i2 in range(len(screws)):
                        if len(screws[i2]) > 5:
                            sc.screw_state(i2, "active",
                                           screws[i2][5] + np.linalg.norm(off))
                frames.append(sc.frame(ev, ecen, edst))
            set_group(ps, (0, 0, 0))
            if screw_ride:
                for i2 in range(len(screws)):
                    if len(screws[i2]) > 5:
                        sc.screw_state(i2, "active", screws[i2][5])
            frames.append(sc.frame(ev, ecen, edst))
        placed_now += ps
    if screws and riders:
        for j in range(len(riders)):
            sc.set_part(f"rd{j}a", (0, 0, 600))
            sc.set_part(f"rd{j}b", (0, 0, 600))
    for i, sc_ in enumerate(screws):
        kind, tip, axis, gap = sc_[0], sc_[1], sc_[2], sc_[3]
        seat = sc_[4] if len(sc_) > 4 else 5.5
        pre = sc_[5] if len(sc_) > 5 else None
        start = pre if pre is not None else gap
        v = view_over.get(i, viewfor(i))
        scen, sdst = screw_cam(center, dist, tip, sblend, szoom)
        for j in range(i):
            sj = screws[j][4] if len(screws[j]) > 4 else 5.5
            sc.screw_state(j, "active", -sj)
        nfr = 7 if seat > 20 else 5
        for t in np.linspace(1, 0, nfr):
            sc.screw_state(i, "active", -seat + (start + seat) * ease(t))
            frames.append(sc.frame(v, scen, sdst))
        if kind.startswith("INS") and i in irons and _g("INSERT_R", 2.1) >= 2.0:
            d_ = irons[i]
            for o in (45.0, 16.0, 0.0, 0.0, 28.0):
                sc.set_part(f"irA{i}", d_ * o)
                sc.set_part(f"irB{i}", d_ * o)
                frames.append(sc.frame(v, scen, sdst))
            sc.set_part(f"irA{i}", (0, 0, 600))
            sc.set_part(f"irB{i}", (0, 0, 600))
    for gn_ in tguides:                 # the finished state carries no guides:
        sc.set_part(gn_, (0, 0, 600))   # rails and travel lines read as stray
    for i_ in range(len(screws)):       # hardware in the final frames
        sc.screw_state(i_, "seated")
    oaz, ozoom = cfg.get("orbit", [36.0, 1.18])
    vaz, vel = viewfor(len(screws))
    for t in np.linspace(0, 1, 8):
        e = ease(t)
        frames.append(sc.frame((vaz + oaz * e, vel), center,
                               dist * (1 + (ozoom - 1) * e)))
    # poster: movers at first waypoint * 0.6, screws floating - unless the
    # sidecar asks for the SEATED state (e.g. b3, where only the seated
    # near-axial view makes the bearing's bore legible)
    pf = 0.0 if cfg.get("poster_seated") else 0.6
    for ps, path in movers:
        set_group(ps, np.array(path[0], float) * pf)
    for i, s in enumerate(screws):
        sc.screw_state(i, "active", s[5] if len(s) > 5 else s[3])
    pv = tuple(cfg.get("poster_view", views[0][0]))
    pdist = dist * cfg.get("poster_zoom", 1.0)
    pcen = center
    if "poster_look" in cfg:
        pcen = np.array(cfg["poster_look"], float) * MM
    poster = sc.frame(pv, pcen, pdist).resize((500, 440), Image.LANCZOS)
    poster.save(os.path.join(OUT, gifname.replace(".gif", ".png")))
    sc.close()
    rs = [f.resize((500, 440), Image.LANCZOS) for f in frames]
    pal = rs[0].convert("P", palette=Image.ADAPTIVE, colors=128)
    small = [pal] + [f.quantize(palette=pal, dither=Image.NONE) for f in rs[1:]]
    small[0].save(os.path.join(OUT, gifname), save_all=True, append_images=small[1:],
                  duration=95, loop=0, optimize=False, disposal=1)
    kb = os.path.getsize(os.path.join(OUT, gifname)) // 1024
    for sc_ in screws:
        INSTALLED.append((sc_[0], sc_[1], sc_[2],
                          sc_[4] if len(sc_) > 4 else 5.5,
                          anchor or placed[0]))
    print(f"  {gifname}  {len(frames)}f  {kb}KB  {'OK' if ok else '** PATH FAIL **'}")


print("== P : heat-set inserts (before ANY assembly) ==")
PLAT_Z1_ = _g("PLAT_Z1")
BRK_Z0_ = _g("BRK_Z0")
BRG_P_Y1_ = _g("BRG_P_Y1")
PLATE_P_Y0_ = _g("PLATE_P_Y0")
CARR_X0_ = _g("CARR_X0")
HC_Z1_ = _g("HC_Z1")
HC_BC_ = _g("HC_BC")
CAP_BC_ = _g("CAP_BC")
rad_ = math.radians
ins_mount = ([("INSx4", (hy, SRV_P_Y - 9.5, CZ + hx), (0, 1, 0), 14, 4.0)
              for hx, hy in TAB]
             + [("INSx4", (RET_BC * math.cos(rad_(a_)), BRG_P_Y1_,
                           CZ + RET_BC * math.sin(rad_(a_))), (0, -1, 0), 14, 4.0)
                for a_ in (0, 120, 240)])
make_step("p1.gif", ["mount"], [], ins_mount, [((-125, -18), 0)])
J_FACE = _g("JRN_P_Y1", -39.4)                        # journal end face
ins_carr = ([("INSx4", (SRV_R_X - 9.5, hy, CZ - hx), (1, 0, 0), 14, 4.0)
             for hx, hy in TAB]
            + [("INSx4", (HORN_BC * math.cos(rad_(a_)), J_FACE,
                          CZ + HORN_BC * math.sin(rad_(a_))), (0, 1, 0), 14, 4.0)
               for a_ in (0, 90, 180, 270)])
make_step("p2.gif", ["carrier"], [], ins_carr, [((-35, -18), 0)])
ins_yoke = ([("INSx4", (CAP_BC_ * math.cos(rad_(a_)), CAP_BC_ * math.sin(rad_(a_)),
                        PLAT_Z1_), (0, 0, -1), 14, 4.0) for a_ in (45, 135, 225, 315)]
            + [("INSx4", (-hy, hx, SRV_T_Z - 9.5), (0, 0, 1), 14, 4.0) for hx, hy in TAB]
            + [("INSx4", (_g("ROLL_FACE_X", 39.1), HORN_BC * math.cos(rad_(a_)),
                          CZ + HORN_BC * math.sin(rad_(a_))), (1, 0, 0), 14, 4.0)
               for a_ in (0, 90, 180, 270)])
make_step("p3.gif", ["yoke"], [], ins_yoke, [((-60, -30), 0)])
ins_hub = [("INSx4", (HC_BC_ * math.cos(rad_(a_)), HC_BC_ * math.sin(rad_(a_)),
                      HC_Z1_), (0, 0, -1), 14, 4.0) for a_ in (0, 120, 240)]
make_step("p4.gif", ["hub_clamp"], [], ins_hub, [((-55, -35), 0)])

print("== A : pitch base ==")
make_step("a1.gif", ["mount"], [(["brg_pitch"], [(0, 45, 0)])], [],
          [((-55, -15), 0)], press=["brg_pitch"])
make_step("a2.gif", ["mount", "brg_pitch"], [(["pitch_retainer"], [(0, 60, 0)])],
          [("M3Bx8", h, (0, -1, 0), 30, 8.0) for h in ret_holes], [((-55, -14), 0)],
          press=["pitch_retainer"], anchor="mount")

print("== B : carrier ==")
# HTS ear mount: the servo drops in clean and its four M3x8 come up from the
# open space behind the deck - no preloading, no riders, nothing to thread
# past other geometry. The clipping-prone through-case era is over.
make_step("b1.gif", ["carrier"],
          [(["servoR_body", "shaftR"], [(-12, 0, -55), (-12, 0, 0)])],
          [], [((-150, -18), 0)], press=["servoR_body"],
          prims=[shaftR])
make_step("b2.gif", ["carrier", "servoR_body"],
          [], [("M3x8", t, a, 14, TAB_SEAT) for t, a in tabR],
          [((-150, -14), 0)], prims=[shaftR], anchor="servoR_body")
make_step("b3.gif", ["carrier", "servoR_body"],
          [(["idler625"], [(-28, 0, 0)])], [], [((-45, -12), 0)],
          prims=[shaftR], anchor="servoR_body",
          engage=((-34.0, 0.0, CZ), 0, (-8, -10), 0.80, 0.30))
# the pitch horn is shadowed by the mount's face plate once the carrier is in,
# so it is bench-fitted to the journal end NOW and rides in at F1.
make_step("b4.gif", ["carrier", "servoR_body"], [(["servoP_shaft"], [(0, -35, 0)])],
          [("M3x6", t, a, 10, 6.0) for t, a in horn_p],
          [((75, -16), 0)], press=["servoP_shaft"], prims=[shaftR],
          anchor="carrier")

print("== C : yoke + twist stack ==")
HC_Z0_ = _g("HC_Z0")
BRG_T_Z0_ = _g("BRG_T_Z0")
guide_c1 = ('<geom type="cylinder" size="%.6f" fromto="0 0 %.6f 0 0 %.6f" '
            'rgba="0.45 0.5 0.58 0.42"/>' % (0.4*MM, HC_Z0_*MM, (HC_Z0_+75)*MM))
guide_c2 = ('<geom type="cylinder" size="%.6f" fromto="0 0 %.6f 0 0 %.6f" '
            'rgba="0.45 0.5 0.58 0.42"/>' % (0.4*MM, BRG_T_Z0_*MM, (BRG_T_Z0_+60)*MM))
# ROLL horn first, on the BARE yoke: its screws are shadowed by the carrier
# front plate later, so the drilled disc is bench-fitted NOW and the drive
# completes by spline entry when the yoke lands at F3.
make_step("c1.gif", ["yoke"], [(["servoR_shaft"], [(-30, 0, 0)])],
          [("M3x6", t, a, 18, 6.0) for t, a in horn_r], [((25, -14), 0)],
          press=["servoR_shaft"], anchor="yoke")
# TWIST IS BUILT SERVO-FIRST: the servo rises until its ears land on the deck,
# and everything it drives is built ONTO it from here, in the open.
make_step("c2.gif", ["yoke", "servoR_shaft"],
          [(["servoT_body", "shaftT"], [(0, 0, -55)])],
          [("M3x8", t, a, 14, TAB_SEAT) for t, a in tabT], [((-55, 24), 0)],
          press=["servoT_body"], prims=[shaftT], anchor="servoT_body")
# the tapped disc drops down the open O39 bore onto the live spline, and the
# servo's OWN factory M3x6 centre screw is torqued NOW - the only moment the
# corridor above it is open to daylight.
make_step("c3.gif", ["yoke", "servoR_shaft", "servoT_body"],
          [(["servoT_shaft"], [(0, 0, 45)])],
          [("M3x6", (0.0, 0.0, HC_Z0), (0, 0, -1), 30, 6.0)],
          [((-55, 26), 0)], press=["servoT_shaft"], prims=[shaftT],
          anchor="servoT_shaft", extra=guide_c1, ghost=["yoke"])
# hub_clamp drops through the still-EMPTY bearing seat onto the disc, then
# 4x M3x6 go DOWN its counterbores into the disc's own tapped holes.
make_step("c4.gif", ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft"],
          [(["hub_clamp"], [(0, 0, 60)])],
          [("M3Kx6", t, a, 30, 9.0) for t, a in horn_t],
          [((-55, 28), 0)], press=["hub_clamp"], prims=[shaftT],
          anchor="hub_clamp")
make_step("c5.gif", ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft",
                     "hub_clamp"],
          [(["brg_twist"], [(0, 0, 45)])], [],
          [((-55, -32), 0)], press=["brg_twist"], extra=guide_c2)
# race_cap goes on BEFORE the plate: the plate underside sits 1.5 mm over the
# cap screws' entry, so they can only be driven while the top is open. The
# plate journal then passes through the capped bore.
make_step("c6.gif", ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft",
                     "hub_clamp", "brg_twist"],
          [(["race_cap"], [(0, 0, 45)])],
          [("M3Cx8", h, (0, 0, -1), 30, 8.0) for h in cap_holes],
          [((-55, -30), 0)], press=["race_cap"], anchor="yoke")
print("== D : bicep onto the plate (BEFORE the plate is installed) ==")
make_step("d1.gif", ["interface_plate", "bicep_ref"],
          [], [("M4x16", h, (0, 0, 1), 30, 20.5) for h in m4_holes],
          [((-50, -35), 0)], anchor="interface_plate")
make_step("c7.gif", ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft",
                     "hub_clamp", "brg_twist", "race_cap"],
          [(["interface_plate", "bicep_ref"], [(0, 0, 60)])], [],
          [((-55, -30), 0)], press=["interface_plate"])
CSET = ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft", "hub_clamp",
        "brg_twist", "race_cap", "interface_plate", "bicep_ref"]
make_step("c8.gif", CSET,
          [], [("M3x20", h, (0, 0, -1), 30, 23.0) for h in hub_bolts],
          [((-55, -14), 0)], anchor="interface_plate")

print("== F : final ==")
SA = ["mount", "brg_pitch", "pitch_retainer"]
SB = ["carrier", "servoR_body"]
make_step("f1.gif", SA, [(SB + ["shaftR", "idler625", "servoP_shaft"], [(0, 55, 0)])],
          [], [((-115, -16), 0)], press=["carrier"],
          prims=[shaftR], anchor="carrier")
make_step("f2.gif", SA + SB + ["servoP_shaft"],
          [(["servoP_body", "shaftP"], [(0, -42, 0)])],
          [("M3x8", t, a, 14, TAB_SEAT) for t, a in tabP],
          [((-115, -16), 0)], press=["servoP_body"],
          prims=[shaftR, shaftP],
          anchor="servoP_body", engage=((0.0, SRV_P_Y + 5.1, CZ), 0))
SYOKE = ["yoke", "servoR_shaft", "servoT_body", "servoT_shaft", "hub_clamp",
         "brg_twist", "race_cap", "interface_plate", "bicep_ref"]
make_step("f3.gif", SA + SB + ["servoP_body", "servoP_shaft"],
          [(SYOKE + ["shaftT"], [(3.5, 0, 65), (3.5, 0, 0)])],
          [("M5x25", (-46, 0, CZ), (1, 0, 0), 24, 24.0)],
          [((205, -16), 0)],
          press=SYOKE,
          prims=[shaftR, shaftP, shaftT],
          anchor="yoke", engage=((YOKE_X0 - 2.0, 0.0, CZ), 0, (25, -8), 0.55, 0.35))
MIDY = (SPY0 + SPY1) / 2
spinebox = ("spinebox",
            f'<geom name="p_spinebox" type="box" size="{25*MM} {29.5*MM} {70*MM}" '
            f'pos="0 {MIDY*MM} {CZ*MM}" rgba="0.55 0.58 0.62 0.35"/>')
# the mirrored RIGHT module's half-shell, ghosted: both arms share these bolts
ghost_r = (f'<geom type="box" size="{31*MM} {3.5*MM} {38*MM}" '
           f'pos="0 {(SPY0-4.5)*MM} {CZ*MM}" rgba="0.92 0.55 0.20 0.30"/>')
# bolts relocated to (+-22, CZ+-34): outside the pitch-boss shadow and the
# servo P case. Sequence: push the four M3x80 through the shell FIRST (they
# become studs), slide the spine onto them, then the far shell, then nylocs -
# so no bolt ever needs the full 83 mm straight corridor, just ~15 mm of head
# room, which every position has even on the servo R side.
spine_bolts = [(-22, SPY1 + 6, CZ - 34), (22, SPY1 + 6, CZ - 34),
               (-22, SPY1 + 6, CZ + 34), (22, SPY1 + 6, CZ + 34)]
FMOD = SA + SB + ["servoP_body", "servoP_shaft"] + SYOKE
make_step("f4.gif", FMOD,
          [(["spinebox"], [(0, -50, 0)])],
          [("M3x80", h, (0, -1, 0), 26, 71.0, 12.0) for h in spine_bolts],
          [((-60, -14), 0)],
          prims=[shaftR, shaftP, shaftT, spinebox],
          extra=ghost_r, anchor="mount", screw_hold=True)
print("all steps done")
