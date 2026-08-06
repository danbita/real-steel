"""
pose_sender.py  —  camera + pose tracking + LIVE SKELETON OVERLAY window
================================================================================
Process 1 of 2. Run this with plain python (NOT mjpython):

    python pose_sender.py

It opens the webcam, runs MediaPipe pose tracking, shows the camera feed with
the skeleton drawn on it (like the offline --preview mode), converts landmarks
to the 8 joint angles, and streams them over UDP to the sim process.

Run mimic_receiver.py with mjpython in a SECOND terminal to see the humanoid.
Press q in the camera window to quit.
================================================================================
"""

import os
os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"   # before cv2 import

import json
import socket
import time
import urllib.request

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

here = os.path.dirname(os.path.abspath(__file__))

# Full model: more accurate than lite, still real-time on Apple silicon.
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_full/float16/latest/pose_landmarker_full.task")
MODEL_PATH = os.path.join(here, "pose_landmarker_full.task")

CAMERA_INDEX = 0
UDP_ADDR = ("127.0.0.1", 5005)
MIN_VISIBILITY = 0.6     # a joint only updates if its landmarks are this visible
SIGN = {"hip": 1.0, "knee": 1.0, "shoulder": 1.0, "elbow": 1.0}

# Landmark indices
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24
L_KNEE, R_KNEE = 25, 26
L_ANKLE, R_ANKLE = 27, 28
L_HEEL, R_HEEL = 29, 30
L_FOOT, R_FOOT = 31, 32

POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (27, 29), (29, 31), (27, 31),
    (24, 26), (26, 28), (28, 30), (30, 32), (28, 32),
]


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("Downloading full pose model...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def seg_angle(pts, a, b, facing):
    dx = (pts[b, 0] - pts[a, 0]) * facing
    dy = pts[b, 1] - pts[a, 1]
    return np.arctan2(dx, dy)


def frame_angles(pts, vis):
    """Returns dict of joint -> angle, only for joints whose landmarks are
    confidently visible this frame. Occluded joints are simply omitted."""
    facing = np.sign((pts[L_FOOT, 0] - pts[L_HEEL, 0]) +
                     (pts[R_FOOT, 0] - pts[R_HEEL, 0])) or 1.0

    torso = 0.5 * (seg_angle(pts, L_SHOULDER, L_HIP, facing) +
                   seg_angle(pts, R_SHOULDER, R_HIP, facing))

    out = {}
    for side, hip, knee, ankle, sh, el, wr in [
        ("L", L_HIP, L_KNEE, L_ANKLE, L_SHOULDER, L_ELBOW, L_WRIST),
        ("R", R_HIP, R_KNEE, R_ANKLE, R_SHOULDER, R_ELBOW, R_WRIST),
    ]:
        if min(vis[hip], vis[knee]) > MIN_VISIBILITY:
            thigh = seg_angle(pts, hip, knee, facing)
            out[f"{side}_hip"] = SIGN["hip"] * (thigh - torso)
            if vis[ankle] > MIN_VISIBILITY:
                shank = seg_angle(pts, knee, ankle, facing)
                out[f"{side}_knee"] = SIGN["knee"] * max(thigh - shank, 0.0)
        if min(vis[sh], vis[el]) > MIN_VISIBILITY:
            uarm = seg_angle(pts, sh, el, facing)
            out[f"{side}_shoulder"] = SIGN["shoulder"] * (uarm - torso)
            if vis[wr] > MIN_VISIBILITY:
                farm = seg_angle(pts, el, wr, facing)
                out[f"{side}_elbow"] = SIGN["elbow"] * max(farm - uarm, 0.0)
    return out


latest = {"pts": None, "vis": None, "stamp": 0.0}


def on_result(result, output_image, timestamp_ms):
    if result.pose_landmarks:
        lms = result.pose_landmarks[0]
        latest["pts"] = np.array([[lm.x, lm.y] for lm in lms])
        latest["vis"] = np.array([lm.visibility for lm in lms])
        latest["stamp"] = time.time()


def main():
    ensure_model()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    options = vision.PoseLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=vision.RunningMode.LIVE_STREAM,
        result_callback=on_result,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    print(f"Streaming joint angles to udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Start the sim in another terminal:  mjpython mimic_receiver.py")
    print("Press q in the camera window to quit.\n")

    t0 = time.time()
    n, fps, fps_clock = 0, 0.0, time.time()

    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            h, w = frame.shape[:2]

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            landmarker.detect_async(mp_image, int((time.time() - t0) * 1000))

            fresh = (latest["pts"] is not None and
                     time.time() - latest["stamp"] < 0.5)
            if fresh:
                pts = latest["pts"] * np.array([w, h])
                ang = frame_angles(pts, latest["vis"])
                if ang:
                    sock.sendto(json.dumps(ang).encode(), UDP_ADDR)

                # skeleton overlay
                px = pts.astype(int)
                for a, b in POSE_CONNECTIONS:
                    cv2.line(frame, tuple(px[a]), tuple(px[b]), (0, 255, 0), 2)
                for p in px[11:]:
                    cv2.circle(frame, tuple(p), 3, (0, 0, 255), -1)
            else:
                cv2.putText(frame, "no person detected", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            n += 1
            if n % 10 == 0:
                now = time.time()
                fps = 10.0 / (now - fps_clock)
                fps_clock = now
            cv2.putText(frame, f"{fps:.0f} fps", (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            cv2.imshow("live pose (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("Stopped.")


if __name__ == "__main__":
    main()
