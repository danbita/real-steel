"""
compose_sidebyside_v11.py - reference clip | MediaPipe skeleton | robot, 30 fps.

Two things changed from the earlier compositors:

1. THE SOURCE IS NO LONGER CROPPED. Previous versions cropped toward his upper
   body, which meant you could not see the whole frame MediaPipe was actually
   measuring. The full frame is letterboxed in instead, so what you see is what
   the tracker saw.

2. THE SKELETON IS DRAWN ON IT. The robot's arms sit higher and further forward
   than his do, and the reason is not the robot - it is that MediaPipe reports
   his upper arms about 55 deg off vertical and 52 deg forward. His hips are
   visible in 16% of frames and his wrists in under half, so the body frame's
   "up" comes from nose-minus-shoulders and the elbow depth is barely
   constrained. Drawing the skeleton makes that visible rather than arguable.
"""
import json
import os
import sys

import cv2
import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# SECOND CLIP. `ATOM_CLIP=<path>` runs this whole pipeline against another video
# and suffixes every derived artefact with that file's stem, so the ref_clip.mp4
# results are never overwritten. Unset, the paths below are exactly as before.
_CLIP = os.environ.get("ATOM_CLIP")
_SLUG = ("_" + os.path.splitext(os.path.basename(_CLIP))[0]) if _CLIP else ""
REF = _CLIP or os.path.join(ROOT, "ref", "ref_clip.mp4")
FR = os.path.join(ROOT, "ref", f"simframes_v11{_SLUG}")
LMK = os.path.join(ROOT, "ref", f"landmarks_v11{_SLUG}.npz")
RSK = os.path.join(ROOT, "ref", f"robot_skel_v11{_SLUG}.npz")
# --clean: plain side-by-side, no skeleton, no captions, no readouts.
CLEAN = "--clean" in sys.argv
OUT = os.path.join(ROOT, "sim", "out36",
                   f"mimic_sidebyside_v11{_SLUG}_clean.mp4" if CLEAN
                   else f"mimic_sidebyside_v11{_SLUG}.mp4")
SEQ = json.load(open(os.path.join(ROOT, "ref", f"pose_seq{_SLUG}.json")))["raw"]

LW, LH, RW = 1280, 720, 720

# MediaPipe Pose topology, torso + arms only - the legs are never in frame
BONES = [(11, 12), (11, 23), (12, 24), (23, 24),
         (11, 13), (13, 15), (12, 14), (14, 16),
         (0, 11), (0, 12)]
L_ARM, R_ARM = {(11, 13), (13, 15)}, {(12, 14), (14, 16)}

# The robot's own skeleton, recorded by the driver as world-space joint centres:
#   0 head  1 neck  2 L_sh  3 L_elbow  4 L_wrist  5 R_sh  6 R_elbow  7 R_wrist
#   8 L_hip 9 R_hip
RBONES = [(0, 1), (1, 2), (1, 5), (2, 3), (3, 4), (5, 6), (6, 7),
          (2, 8), (5, 9), (8, 9)]
RL_ARM, RR_ARM = {(2, 3), (3, 4)}, {(5, 6), (6, 7)}
COL_L, COL_R, COL_T = (60, 140, 235, 240), (242, 145, 55, 240), (215, 225, 240, 190)


def camera_projector(scene_xml, cam="front"):
    """World -> pixel for the camera the robot frames were rendered with.
    MuJoCo cameras are OpenGL-style: +X right, +Y up, looking down -Z."""
    import mujoco
    mm = mujoco.MjModel.from_xml_path(scene_xml)
    ci = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    pos = mm.cam_pos[ci].copy()
    R = mm.cam_mat0[ci].reshape(3, 3).copy()      # columns = camera axes in world
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

Z = np.load(LMK) if os.path.exists(LMK) else None
PX = Z["px"] if Z is not None else None
VIS = Z["vis"] if Z is not None else None
ELEV = Z["elev"] if Z is not None else None
FWD = Z["fwd"] if Z is not None else None
RZ = np.load(RSK) if os.path.exists(RSK) else None
RPTS = RZ["pts"] if RZ is not None else None
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

    # letterbox the WHOLE frame - no crop, no stretch
    sc = min(LW / w0, LH / h0)
    nw, nh = int(round(w0 * sc)), int(round(h0 * sc))
    left = np.zeros((LH, LW, 3), np.uint8)
    ox, oy = (LW - nw) // 2, (LH - nh) // 2
    left[oy:oy + nh, ox:ox + nw] = cv2.resize(rgb, (nw, nh))

    li = Image.fromarray(left)
    dl = ImageDraw.Draw(li, "RGBA")
    if PX is not None and i < len(PX) and not CLEAN:
        pts = PX[i]
        vis = VIS[i]
        xy = [(ox + pts[k, 0] * nw, oy + pts[k, 1] * nh) for k in range(len(pts))]
        for a, b in BONES:
            if vis[a] < 0.3 and vis[b] < 0.3:
                continue
            # match the ROBOT's materials: blue = left, orange = right. Drawing
            # them the other way round is what made the two panels look mirrored
            # - they were already on the same anatomical side.
            col = ((60, 140, 235, 240) if (a, b) in L_ARM else
                   (242, 145, 55, 240) if (a, b) in R_ARM else (215, 225, 240, 190))
            dl.line([xy[a], xy[b]], fill=col, width=5)
        for k in (0, 11, 12, 13, 14, 15, 16, 23, 24):
            r = 6 if vis[k] > 0.5 else 4
            c = (250, 250, 250, 240) if vis[k] > 0.5 else (250, 110, 110, 230)
            dl.ellipse([xy[k][0] - r, xy[k][1] - r, xy[k][0] + r, xy[k][1] + r], fill=c)
        if ELEV is not None and not np.isnan(ELEV[i, 0]):
            dl.rectangle([12, LH - 122, 470, LH - 44], fill=(8, 12, 18, 205))
            dl.text((22, LH - 118), "WHAT MEDIAPIPE REPORTS    blue = his LEFT arm,"
                    "  orange = his RIGHT   (same as the robot)", font=F_XS,
                    fill=(150, 200, 235))
            dl.text((22, LH - 96),
                    f"upper arm off vertical   L {ELEV[i,0]:5.1f}   R {ELEV[i,1]:5.1f} deg",
                    font=F_XS, fill=(235, 240, 248))
            dl.text((22, LH - 76),
                    f"upper arm forward lean   L {FWD[i,0]:5.1f}   R {FWD[i,1]:5.1f} deg",
                    font=F_XS, fill=(235, 240, 248))
            dl.text((22, LH - 56),
                    f"hips visible {vis[23]:.2f}/{vis[24]:.2f}   wrists {vis[15]:.2f}/{vis[16]:.2f}",
                    font=F_XS, fill=(250, 170, 120))
    left = np.asarray(li)

    ri = Image.open(p).convert("RGB").resize((RW, LH))
    if RPTS is not None and i < len(RPTS) and not CLEAN:
        dr = ImageDraw.Draw(ri, "RGBA")
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
    right = np.asarray(ri)
    sheet = Image.fromarray(np.hstack([left, right]))
    if not CLEAN:
        d = ImageDraw.Draw(sheet, "RGBA")
        d.rectangle([0, 0, LW + RW, 52], fill=(8, 12, 18, 200))
        d.text((22, 12), "REFERENCE  43s - 1:43   + MediaPipe skeleton", font=F_T,
               fill=(240, 244, 250))
        d.text((LW + 16, 8), "humanoid_v11  -  36 in, shoulder v6", font=F_T,
               fill=(240, 244, 250))
        d.text((LW + 16, 32), "solved joint centres - same colour code", font=F_S,
               fill=(150, 200, 235))
        a = SEQ[i] if i < len(SEQ) else {}
        hud = ("36 in skeleton - arms solved by 4-DOF IK, legs held idle"
               if a else "no pose this frame - joints hold last command")
        d.rectangle([0, LH - 34, LW + RW, LH], fill=(8, 12, 18, 190))
        d.text((22, LH - 28), f"t={i/30:5.1f}s   {hud}", font=F_S, fill=(205, 215, 228))
    w.append_data(np.asarray(sheet))
    i += 1
cap.release()
w.close()
print(f"wrote {OUT}  ({i} frames, {i/30:.1f}s)")
