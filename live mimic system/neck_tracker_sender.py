"""
neck_tracker_sender.py  -  Atom's neck follows a person in the room
================================================================================
    python "live mimic system/neck_tracker_sender.py"
    mjpython "live mimic system/neck_receiver.py"        # in another terminal

Watches the camera, picks a person, and sends neck_yaw / neck_pitch on the same
UDP socket every other receiver listens to. Empty room -> the neck rests looking
straight ahead.

This is a SEPARATE sender from pose_sender.py, deliberately:
  * pose_sender runs num_poses=1. Choosing who to look at needs to SEE more than
    one person, so this one runs several.
  * They are different jobs. pose_sender mirrors a person's own head pose onto
    the neck; this aims the neck AT a person. Mirroring still works exactly as it
    did - nothing here touched it - and you pick a behaviour by choosing which
    sender to run.
  * Both write to udp://127.0.0.1:5005, so run ONE of them at a time. Two
    senders means two sets of commands racing on the same joints.

The decisions all live in neck_tracking.py, which has no camera dependencies and
is tested frame-by-frame in sim36/neck_tracking_check.py. This file only turns
frames into candidates.

FLAGS
    --fov <deg>     camera horizontal FOV. DEFAULT 60 - measure yours, it sets
                    how far the neck turns for a given spot in the frame.
    --source <path> run off a video file instead of the webcam
    --cam <index>   camera index (Continuity Camera reorders these)
    --noshow        no preview window
    --maxpeople <n> how many people to detect at once (default 5)
================================================================================
"""

import json
import math
import os
import socket
import sys
import time

os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import neck_tracking as nt                       # noqa: E402
from pose_sender import (ensure_model, MODEL_PATH, UDP_ADDR,   # noqa: E402
                         NOSE, L_EYE, R_EYE, L_EAR, R_EAR,
                         L_SHOULDER, R_SHOULDER, VIS_OFF)


def _arg(flag, default=None, cast=str):
    if flag in sys.argv:
        i = sys.argv.index(flag)
        if i + 1 < len(sys.argv):
            return cast(sys.argv[i + 1])
    return default


HFOV = _arg("--fov", 60.0, float)
SOURCE = _arg("--source")
CAM = _arg("--cam", 0, int)
NOSHOW = "--noshow" in sys.argv
MAX_PEOPLE = _arg("--maxpeople", 5, int)

latest = {"poses": [], "stamp": 0.0}


def on_result(result, output_image, timestamp_ms):
    """Keep EVERY detected pose, not just the first - the whole point is to
    choose between people."""
    try:
        poses = []
        for lm in (result.pose_landmarks or []):
            poses.append((np.array([[p.x, p.y] for p in lm]),
                          np.array([p.visibility for p in lm])))
        latest["poses"] = poses
        latest["stamp"] = time.time()
    except Exception:
        pass


def aim_point(img, vis):
    """Where on this person to look.

    The FACE, in order of preference: ear midpoint, then eyes, then the nose,
    and only as a last resort a point lifted above the shoulders. Aiming at the
    body centroid would point the neck at someone's chest, which reads as
    ignoring them rather than looking at them.
    """
    for a, b in ((L_EAR, R_EAR), (L_EYE, R_EYE)):
        if min(vis[a], vis[b]) >= VIS_OFF:
            return (img[a] + img[b]) * 0.5
    if vis[NOSE] >= VIS_OFF:
        return img[NOSE]
    if min(vis[L_SHOULDER], vis[R_SHOULDER]) >= VIS_OFF:
        mid = (img[L_SHOULDER] + img[R_SHOULDER]) * 0.5
        span = float(np.linalg.norm(img[L_SHOULDER] - img[R_SHOULDER]))
        return np.array([mid[0], mid[1] - 0.6 * span])   # roughly head height
    return None


def apparent_size(img, vis, aspect):
    """Bigger = closer. Shoulder span, with a head-width fallback.

    Apparent size is the only distance cue one camera gives without depth. It is
    imperfect - turning side-on narrows the shoulders - which is why the tracker
    uses it ONLY to choose who to acquire, never to abandon a lock it holds.
    x is normalized by width and y by height, so x is scaled by the aspect ratio
    before any length is taken from it.
    """
    def seg(a, b):
        d = img[a] - img[b]
        return float(math.hypot(d[0] * aspect, d[1]))
    if min(vis[L_SHOULDER], vis[R_SHOULDER]) >= VIS_OFF:
        return seg(L_SHOULDER, R_SHOULDER)
    if min(vis[L_EAR], vis[R_EAR]) >= VIS_OFF:
        return seg(L_EAR, R_EAR) * 2.2        # ear span is ~45% of shoulder span
    good = img[vis >= VIS_OFF]
    if len(good) >= 2:
        return float(good[:, 1].max() - good[:, 1].min()) * 0.35
    return 0.0


def make_landmarker():
    return vision.PoseLandmarker.create_from_options(
        vision.PoseLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=vision.RunningMode.LIVE_STREAM,
            result_callback=on_result,
            num_poses=MAX_PEOPLE,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        ))


def main():
    ensure_model()
    src = SOURCE if SOURCE else CAM
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        sys.exit(f"could not open camera/source {src!r}")
    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 1280.0
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 720.0
    aspect = float(w) / float(h)

    nt.HFOV_DEG = HFOV
    nt.ASPECT = aspect
    tracker = nt.NeckTracker(hfov_deg=HFOV, aspect=aspect)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    landmarker = make_landmarker()
    t0 = time.time()

    print(f"Neck tracker: {int(w)}x{int(h)}, aspect {aspect:.3f}, "
          f"HFOV {HFOV:.1f} deg -> VFOV {math.degrees(nt.vfov_rad(HFOV, aspect)):.1f} deg")
    print(f"Detecting up to {MAX_PEOPLE} people. Sending to "
          f"udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}.")
    print("Empty frame -> neutral. First person in -> lock and follow.\n")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            landmarker.detect_async(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                    int((time.time() - t0) * 1000))

            cands = []
            for img, vis in latest["poses"]:
                a = aim_point(img, vis)
                if a is None:
                    continue
                s = apparent_size(img, vis, aspect)
                if s <= 0.0:
                    continue
                cands.append(nt.Candidate(a, s, float(np.mean(vis))))

            angles = tracker.update(cands, now=time.time())
            sock.sendto(json.dumps(angles).encode(), UDP_ADDR)

            if not NOSHOW:
                for c in cands:
                    px = (int(c.aim[0] * frame.shape[1]), int(c.aim[1] * frame.shape[0]))
                    locked = (tracker.target_aim is not None
                              and abs(c.aim[0] - tracker.target_aim[0]) < 1e-9
                              and abs(c.aim[1] - tracker.target_aim[1]) < 1e-9)
                    col = (0, 255, 0) if locked else (128, 128, 128)
                    cv2.circle(frame, px, 14 if locked else 8, col, 3 if locked else 2)
                    cv2.putText(frame, f"{c.size:.2f}", (px[0] + 18, px[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
                cv2.putText(frame, tracker.describe(), (12, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"{len(cands)} in frame", (12, 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imshow("neck tracker  -  q to quit", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        if not NOSHOW:
            cv2.destroyAllWindows()
        print("\nStopped.")


if __name__ == "__main__":
    main()
