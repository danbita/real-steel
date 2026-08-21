"""
extract_landmarks.py - dump MediaPipe's 2D landmarks and the arm angles it is
actually reporting, so the side-by-side can show its skeleton and so the
"why are the robot's arms higher than his?" question can be answered with a
number instead of an opinion.

    python sim36/extract_landmarks.py   ->  ref/landmarks_v11.npz

Writes, per frame:
    px      33 x 2   normalised image coords, for drawing
    vis     33       landmark visibility
    elev    2        upper-arm angle from straight-DOWN in the body frame,
                     left and right. This is the number the retarget turns
                     into shoulder roll, so it is the number to compare
                     against the robot.
    fwd     2        upper-arm forward lean in the body frame, same idea
    ok      1        whether a pose was detected at all
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# SECOND CLIP. `ATOM_CLIP=<path>` runs this whole pipeline against another video
# and suffixes every derived artefact with that file's stem, so the ref_clip.mp4
# results are never overwritten. Unset, the paths below are exactly as before.
_CLIP = os.environ.get("ATOM_CLIP")
_SLUG = ("_" + os.path.splitext(os.path.basename(_CLIP))[0]) if _CLIP else ""
REF = _CLIP or os.path.join(ROOT, "ref", "ref_clip.mp4")
OUT = os.path.join(ROOT, "ref", f"landmarks_v11{_SLUG}.npz")

import importlib.util
PS = os.path.join(ROOT, "live mimic system", "pose_sender.py")
spec = importlib.util.spec_from_file_location("pose_sender", PS)
ps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ps)

import mediapipe as mp
from mediapipe.tasks import python as mpp
from mediapipe.tasks.python import vision


def main():
    ps.ensure_model()
    opts = vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=ps.MODEL_PATH),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.5, min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5)
    lm = vision.PoseLandmarker.create_from_options(opts)
    cap = cv2.VideoCapture(REF)

    PX, VIS, ELEV, FWD, OK = [], [], [], [], []
    i = 0
    while True:
        got, frame = cap.read()
        if not got:
            break
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                  int(i * 1000 / 30))
        if res.pose_landmarks:
            px = np.array([[p.x, p.y] for p in res.pose_landmarks[0]], dtype=np.float32)
            vis = np.array([p.visibility for p in res.pose_landmarks[0]], dtype=np.float32)
            W = np.array([[p.x, p.y, p.z] for p in res.pose_world_landmarks[0]])
            hips_ok = vis[ps.L_HIP] > 0.5 and vis[ps.R_HIP] > 0.5
            f, l, u = ps.body_frame(W, hips_ok)
            Rb = np.stack([f, l, u])
            e, w = [], []
            for sh, elb in ((ps.L_SHOULDER, ps.L_ELBOW), (ps.R_SHOULDER, ps.R_ELBOW)):
                v = Rb @ (W[elb] - W[sh])
                n = np.linalg.norm(v)
                if n < 1e-6:
                    e.append(np.nan); w.append(np.nan); continue
                v = v / n
                # angle away from straight-down, and forward lean
                e.append(np.degrees(np.arccos(np.clip(-v[2], -1, 1))))
                w.append(np.degrees(np.arctan2(v[0], -v[2])))
            PX.append(px); VIS.append(vis); ELEV.append(e); FWD.append(w); OK.append(1)
        else:
            PX.append(np.zeros((33, 2), np.float32)); VIS.append(np.zeros(33, np.float32))
            ELEV.append([np.nan, np.nan]); FWD.append([np.nan, np.nan]); OK.append(0)
        i += 1
    cap.release()

    PX = np.array(PX); VIS = np.array(VIS)
    ELEV = np.array(ELEV, dtype=np.float64); FWD = np.array(FWD, dtype=np.float64)
    OK = np.array(OK)
    np.savez_compressed(OUT, px=PX, vis=VIS, elev=ELEV, fwd=FWD, ok=OK)

    good = ~np.isnan(ELEV[:, 0])
    print(f"{len(PX)} frames, pose found on {OK.sum()}")
    print("\nWHAT MEDIAPIPE SAYS THE UPPER ARMS ARE DOING")
    print("  (angle away from straight-down, in the body frame - this is exactly")
    print("   what the retarget converts into shoulder roll)")
    for k, side in enumerate(("left", "right")):
        e = ELEV[good, k]
        f = FWD[good, k]
        print(f"  {side:5}  elevation mean {e.mean():5.1f}  median {np.median(e):5.1f}"
              f"  p10 {np.percentile(e,10):5.1f}  p90 {np.percentile(e,90):5.1f} deg")
        print(f"         forward   mean {f.mean():5.1f}  median {np.median(f):5.1f} deg")
    print(f"\n  elbow visibility  mean {VIS[good][:, ps.L_ELBOW].mean():.2f} (L) "
          f"{VIS[good][:, ps.R_ELBOW].mean():.2f} (R)")
    print(f"  wrist visibility  mean {VIS[good][:, ps.L_WRIST].mean():.2f} (L) "
          f"{VIS[good][:, ps.R_WRIST].mean():.2f} (R)")
    print(f"  hip   visibility  mean {VIS[good][:, ps.L_HIP].mean():.2f} (L) "
          f"{VIS[good][:, ps.R_HIP].mean():.2f} (R)")
    print(f"\nwrote {OUT}")


if __name__ == "__main__":
    main()
