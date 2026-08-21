"""
compose_sidebyside_live.py - the user's webcam clip | the LIVE robot, 30 fps.

Adapted from compose_sidebyside_v11.py. Two differences:

1. Everything on the left comes from ref/live_sender_log.npz, i.e. the 2D
   landmarks the LIVE pose_sender actually had in hand on that frame - not a
   second, offline MediaPipe pass. Frames the live path had no result for are
   drawn bare.
2. The right panel is ref/simframes_live, replayed from the live receiver's
   own qpos dump, and the HUD calls out the two failure modes the numbers
   found: hips extrapolated below the bottom of the image, and shoulder pitch
   sitting on its +90 deg limit.

    python sim36/compose_sidebyside_live.py [--clean]
"""
import importlib.util
import math
import os
import sys

import cv2
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF = os.environ.get(
    "LIVE_REF", r"C:\Users\Yasir\Pictures\Camera Roll\WIN_20260818_07_32_49_Pro.mp4")
FR = os.path.join(ROOT, "ref", "simframes_live")
SLOG = os.path.join(ROOT, "ref", "live_sender_log.npz")
DUMP = os.path.join(ROOT, "ref", "live_dump.npz")
RSK = os.path.join(ROOT, "ref", "robot_skel_live.npz")
CLEAN = "--clean" in sys.argv
OUT = os.path.join(ROOT, "sim", "out36",
                   "live_diag_sidebyside_clean.mp4" if CLEAN
                   else "live_diag_sidebyside.mp4")
os.makedirs(os.path.dirname(OUT), exist_ok=True)

spec = importlib.util.spec_from_file_location(
    "pose_sender",
    os.path.join(ROOT, "live mimic system", "pose_sender.py"))
ps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps)

LW, LH, RW = 1280, 720, 720
BONES = [(11, 12), (11, 23), (12, 24), (23, 24),
         (11, 13), (13, 15), (12, 14), (14, 16), (0, 11), (0, 12)]
L_ARM, R_ARM = {(11, 13), (13, 15)}, {(12, 14), (14, 16)}
RBONES = [(0, 1), (1, 2), (1, 5), (2, 3), (3, 4), (5, 6), (6, 7),
          (2, 8), (5, 9), (8, 9)]
RL_ARM, RR_ARM = {(2, 3), (3, 4)}, {(5, 6), (6, 7)}
COL_L, COL_R, COL_T = (60, 140, 235, 240), (242, 145, 55, 240), (215, 225, 240, 190)


def camera_projector(scene_xml, cam="front"):
    import mujoco
    mm = mujoco.MjModel.from_xml_path(scene_xml)
    ci = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    pos = mm.cam_pos[ci].copy()
    R = mm.cam_mat0[ci].reshape(3, 3).copy()
    fovy = float(mm.cam_fovy[ci])

    def project(pw, w, h):
        pc = R.T @ (np.asarray(pw) - pos)
        depth = -pc[2]
        if depth < 1e-6:
            return None
        f = (h / 2.0) / np.tan(np.radians(fovy) / 2.0)
        return (w / 2.0 + f * pc[0] / depth, h / 2.0 - f * pc[1] / depth)
    return project


def font(sz):
    for n in ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(n, sz)
        except OSError:
            pass
    return ImageFont.load_default()


F_T, F_S, F_XS = font(27), font(19), font(16)

S = np.load(SLOG)
PX, VIS, WLD = S["px"], S["vis"], S["world"]
FRESH, HIPSOK, SRCI = S["fresh"].astype(bool), S["hips_ok"].astype(bool), S["i"]
NS = int(SRCI.max()) + 1
row_of = {int(v): k for k, v in enumerate(SRCI)}

# per-source-frame body-frame arm angles, from the LIVE landmarks
ELEV = np.full((NS, 2), np.nan)
FWD = np.full((NS, 2), np.nan)
for k in range(len(SRCI)):
    if not FRESH[k]:
        continue
    f, l, u = ps.body_frame(WLD[k], bool(HIPSOK[k]))
    Rb = np.stack([f, l, u])
    for c, (sh, el) in enumerate(((11, 13), (12, 14))):
        v = Rb @ (WLD[k, el] - WLD[k, sh])
        n = np.linalg.norm(v)
        if n < 1e-6:
            continue
        v = v / n
        ELEV[int(SRCI[k]), c] = math.degrees(math.acos(np.clip(-v[2], -1, 1)))
        FWD[int(SRCI[k]), c] = math.degrees(math.atan2(v[0], -v[2]))

# per-source-frame "is a shoulder pitch pinned at its limit" flag
D = np.load(DUMP)
keys = [str(x) for x in D["solve_keys"]]
ix = {k: i for i, k in enumerate(keys)}
SV = D["solves"]
PIN = np.zeros((NS, 2), bool)
RES = np.full(NS, np.nan)
for r in range(len(SV)):
    k = int(SV[r, ix["src_i"]])
    if 0 <= k < NS:
        PIN[k, 0] = SV[r, ix["L_shoulder_pitch"]] >= 1.5708 - 1e-3
        PIN[k, 1] = SV[r, ix["R_shoulder_pitch"]] >= 1.5708 - 1e-3
        RES[k] = SV[r, ix["res"]]

RPTS = np.load(RSK)["pts"]
project = camera_projector(os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml"))

cap = cv2.VideoCapture(REF)
w = imageio.get_writer(OUT, fps=30, quality=8, macro_block_size=1)
i = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    p = os.path.join(FR, f"{i:05d}.png")
    if not os.path.exists(p):
        break
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h0, w0 = rgb.shape[:2]
    sc = min(LW / w0, LH / h0)
    nw, nh = int(round(w0 * sc)), int(round(h0 * sc))
    left = np.zeros((LH, LW, 3), np.uint8)
    ox, oy = (LW - nw) // 2, (LH - nh) // 2
    left[oy:oy + nh, ox:ox + nw] = cv2.resize(rgb, (nw, nh))

    li = Image.fromarray(left)
    dl = ImageDraw.Draw(li, "RGBA")
    r_ = row_of.get(i)
    have = r_ is not None and FRESH[r_]
    if have and not CLEAN:
        pts, vis = PX[r_], VIS[r_]
        xy = [(ox + pts[k, 0] * nw, oy + pts[k, 1] * nh) for k in range(len(pts))]
        for a, b in BONES:
            if vis[a] < 0.3 and vis[b] < 0.3:
                continue
            col = (COL_L if (a, b) in L_ARM else
                   COL_R if (a, b) in R_ARM else COL_T)
            dl.line([xy[a], xy[b]], fill=col, width=5)
        for k in (0, 11, 12, 13, 14, 15, 16, 23, 24):
            below = pts[k, 1] > 1.0            # extrapolated off the bottom
            rr = 6 if vis[k] > 0.5 else 4
            c = ((255, 70, 70, 240) if below else
                 (250, 250, 250, 240) if vis[k] > 0.5 else (250, 110, 110, 230))
            dl.ellipse([xy[k][0] - rr, xy[k][1] - rr,
                        xy[k][0] + rr, xy[k][1] + rr], fill=c)
        if pts[23, 1] > 1.0 or pts[24, 1] > 1.0:
            dl.line([(ox, oy + nh - 2), (ox + nw, oy + nh - 2)],
                    fill=(255, 70, 70, 200), width=3)
            dl.text((ox + 12, oy + nh - 26),
                    "HIPS BELOW THE FRAME - extrapolated",
                    font=F_XS, fill=(255, 120, 120))
    if not CLEAN:
        dl.rectangle([12, LH - 128, 600, LH - 40], fill=(8, 12, 18, 210))
        dl.text((22, LH - 124), "WHAT THE LIVE SENDER SAW    blue = his LEFT arm,"
                "  orange = his RIGHT", font=F_XS, fill=(150, 200, 235))
        if have:
            dl.text((22, LH - 102),
                    f"upper arm off vertical   L {ELEV[i,0]:5.1f}   R {ELEV[i,1]:5.1f} deg",
                    font=F_XS, fill=(235, 240, 248))
            dl.text((22, LH - 82),
                    f"upper arm forward lean   L {FWD[i,0]:5.1f}   R {FWD[i,1]:5.1f} deg",
                    font=F_XS, fill=(235, 240, 248))
            dl.text((22, LH - 62),
                    f"hip vis {VIS[r_,23]:.2f}/{VIS[r_,24]:.2f}"
                    f"   wrist {VIS[r_,15]:.2f}/{VIS[r_,16]:.2f}"
                    f"   body frame: {'HIPS' if HIPSOK[r_] else 'GRAVITY'}",
                    font=F_XS, fill=(250, 170, 120))
        else:
            dl.text((22, LH - 92), "no MediaPipe result on this frame",
                    font=F_XS, fill=(250, 120, 120))
    left = np.asarray(li)

    ri = Image.open(p).convert("RGB").resize((RW, LH))
    dr = ImageDraw.Draw(ri, "RGBA")
    if i < len(RPTS) and not CLEAN:
        q = [project(pt, RW, LH) for pt in RPTS[i]]
        for a_, b_ in RBONES:
            if q[a_] is None or q[b_] is None:
                continue
            col = (COL_L if (a_, b_) in RL_ARM else
                   COL_R if (a_, b_) in RR_ARM else COL_T)
            dr.line([q[a_], q[b_]], fill=col, width=5)
        for k in range(len(q)):
            if q[k] is None:
                continue
            dr.ellipse([q[k][0] - 6, q[k][1] - 6, q[k][0] + 6, q[k][1] + 6],
                       fill=(250, 250, 250, 240))
        if PIN[i].any():
            side = "+".join(s for s, fl in zip(("L", "R"), PIN[i]) if fl)
            dr.rectangle([10, LH - 62, RW - 10, LH - 28], fill=(90, 10, 10, 220))
            dr.text((20, LH - 58),
                    f"{side} shoulder pitch PINNED at +90 deg limit"
                    f"   IK residual {RES[i]:.2f}", font=F_XS, fill=(255, 190, 190))
    right = np.asarray(ri)

    sheet = Image.fromarray(np.hstack([left, right]))
    if not CLEAN:
        d = ImageDraw.Draw(sheet, "RGBA")
        d.rectangle([0, 0, LW + RW, 52], fill=(8, 12, 18, 200))
        d.text((22, 12), "YOUR RECORDING  (full frame, letterboxed)  + live "
               "MediaPipe skeleton", font=F_T, fill=(240, 244, 250))
        d.text((LW + 16, 6), "LIVE PATH   pose_sender --landmarks -> UDP -> "
               "mimic_receiver_v11 ArmIK", font=F_S, fill=(240, 244, 250))
        d.text((LW + 16, 28), "replayed from the receiver's own qpos dump",
               font=F_S, fill=(150, 200, 235))
        d.rectangle([0, LH - 34, LW + RW, LH], fill=(8, 12, 18, 190))
        d.text((22, LH - 28), f"t={i/30:5.1f}s   frame {i}", font=F_S,
               fill=(205, 215, 228))
    w.append_data(np.asarray(sheet))
    i += 1
cap.release()
w.close()
print(f"wrote {OUT}  ({i} frames, {i/30:.1f}s)")
