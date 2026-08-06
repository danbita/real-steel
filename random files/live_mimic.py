"""
live_mimic.py  —  the humanoid mirrors you from the webcam, in real time
================================================================================
webcam frame -> PoseLandmarker (LIVE_STREAM, async) -> 8 sagittal joint angles
-> exponential smoothing -> data.qpos -> MuJoCo viewer

NOTE (macOS): there is NO camera preview window. OpenCV GUI windows and the
MuJoCo viewer both require the Mac's main thread, so they can't coexist under
mjpython. The MuJoCo humanoid itself is your mirror; tracking status and fps
are printed to the terminal.

Run:
    mjpython live_mimic.py
Stand SIDE-ON to the camera, full body in frame. Close the viewer window or
Ctrl+C to stop. Expects humanoid.urdf in the same folder.
================================================================================
"""

import os
os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"   # must be set before cv2 import

import time
import urllib.request

import cv2
import numpy as np
import mujoco
import mujoco.viewer
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

here = os.path.dirname(os.path.abspath(__file__))
URDF_FILE = os.path.join(here, "humanoid.urdf")

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task")
MODEL_PATH = os.path.join(here, "pose_landmarker_lite.task")

CAMERA_INDEX = 0        # try 1, 2... if the wrong camera opens
SMOOTH_ALPHA = 0.4      # 0..1  higher = more responsive, lower = smoother
SIGN = {"hip": 1.0, "knee": 1.0, "shoulder": 1.0, "elbow": 1.0}  # match replay_pose.py

# Landmark indices
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24
L_KNEE, R_KNEE = 25, 26
L_ANKLE, R_ANKLE = 27, 28
L_HEEL, R_HEEL = 29, 30
L_FOOT, R_FOOT = 31, 32

JOINT_NAMES = ["L_hip", "L_knee", "R_hip", "R_knee",
               "L_shoulder", "L_elbow", "R_shoulder", "R_elbow"]


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("Downloading lite pose model...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


# ---------------------------------------------------------------------------
# Retargeting (single frame): landmark pixels -> 8 joint angles
# ---------------------------------------------------------------------------
def seg_angle(pts, a, b, facing):
    dx = (pts[b, 0] - pts[a, 0]) * facing
    dy = pts[b, 1] - pts[a, 1]          # image y grows downward
    return np.arctan2(dx, dy)


def frame_angles(pts):
    facing = np.sign((pts[L_FOOT, 0] - pts[L_HEEL, 0]) +
                     (pts[R_FOOT, 0] - pts[R_HEEL, 0])) or 1.0

    torso = 0.5 * (seg_angle(pts, L_SHOULDER, L_HIP, facing) +
                   seg_angle(pts, R_SHOULDER, R_HIP, facing))

    out = {}
    for side, hip, knee, ankle, sh, el, wr in [
        ("L", L_HIP, L_KNEE, L_ANKLE, L_SHOULDER, L_ELBOW, L_WRIST),
        ("R", R_HIP, R_KNEE, R_ANKLE, R_SHOULDER, R_ELBOW, R_WRIST),
    ]:
        thigh = seg_angle(pts, hip, knee, facing)
        shank = seg_angle(pts, knee, ankle, facing)
        uarm = seg_angle(pts, sh, el, facing)
        farm = seg_angle(pts, el, wr, facing)
        out[f"{side}_hip"] = SIGN["hip"] * (thigh - torso)
        out[f"{side}_knee"] = SIGN["knee"] * max(thigh - shank, 0.0)
        out[f"{side}_shoulder"] = SIGN["shoulder"] * (uarm - torso)
        out[f"{side}_elbow"] = SIGN["elbow"] * max(farm - uarm, 0.0)
    return out


# ---------------------------------------------------------------------------
# Live stream: MediaPipe delivers results via callback; keep only the latest
# ---------------------------------------------------------------------------
latest = {"pts": None, "stamp": 0.0}


def on_result(result, output_image, timestamp_ms):
    if result.pose_landmarks:
        lms = result.pose_landmarks[0]
        latest["pts"] = np.array([[lm.x, lm.y] for lm in lms])  # normalized
        latest["stamp"] = time.time()


def main():
    ensure_model()

    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    def q(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid]
    qmap = {n: q(n) for n in JOINT_NAMES}

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
        raise RuntimeError("Could not open webcam — check CAMERA_INDEX and "
                           "macOS camera permissions for your terminal app.")

    smoothed = {n: 0.0 for n in JOINT_NAMES}
    t0 = time.time()
    n_frames = 0
    was_tracking = False
    last_status = time.time()

    print("Live mimicry running. Stand side-on to the camera, full body in frame.")
    print("The MuJoCo figure is your mirror. Close the viewer or Ctrl+C to stop.\n")

    try:
        with vision.PoseLandmarker.create_from_options(options) as landmarker, \
             mujoco.viewer.launch_passive(model, data) as viewer:

            while viewer.is_running():
                ok, frame = cap.read()
                if not ok:
                    print("Camera stream ended.")
                    break
                h, w = frame.shape[:2]

                # hand frame to MediaPipe (non-blocking; result arrives in callback)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                landmarker.detect_async(mp_image, int((time.time() - t0) * 1000))

                pts_norm = latest["pts"]
                tracking = pts_norm is not None and (time.time() - latest["stamp"]) < 0.5

                if tracking:
                    pts = pts_norm * np.array([w, h])           # pixel coords
                    target = frame_angles(pts)
                    for name in JOINT_NAMES:                    # causal smoothing
                        smoothed[name] += SMOOTH_ALPHA * (target[name] - smoothed[name])
                    for name, adr in qmap.items():
                        data.qpos[adr] = smoothed[name]
                    mujoco.mj_forward(model, data)
                    viewer.sync()

                # terminal status: tracking changes + fps once per 2 s
                if tracking != was_tracking:
                    print("tracking: PERSON FOUND" if tracking else
                          "tracking: lost — step back so your full body is visible")
                    was_tracking = tracking
                n_frames += 1
                now = time.time()
                if now - last_status >= 2.0:
                    fps = n_frames / (now - last_status)
                    print(f"  {fps:5.1f} fps | tracking: {'yes' if tracking else 'no'}")
                    n_frames = 0
                    last_status = now
    except KeyboardInterrupt:
        pass

    cap.release()
    print("\nStopped.")


if __name__ == "__main__":
    main()
