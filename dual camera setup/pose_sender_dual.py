"""
pose_sender_dual.py  —  TWO-CAMERA 3D motion capture with per-joint fusion
================================================================================
Run with plain python (NOT mjpython):     python pose_sender_dual.py
Pair with any receiver (humanoid or arm) — same UDP packet format as v3.

How it works: each camera runs the full v3 pipeline independently (MediaPipe
world landmarks -> swap guard -> bone-length depth correction -> body-frame
joint angles). Because v3 angles are computed in YOUR BODY's coordinate frame,
they are camera-invariant — both cameras' outputs are directly comparable with
no stereo calibration. Fusion is per joint: each camera reports a confidence
(how well it sees that joint's landmarks) and the fused angle is the
confidence-weighted average; if the two views disagree strongly, the more
confident one wins outright. Place the cameras ~45-90 degrees apart so every
limb is well seen by at least one of them.

Set CAM_A / CAM_B below. On a Mac, the built-in camera and a Continuity
Camera iPhone usually enumerate as 0 and 1 (order varies — if the windows
show the wrong feeds, swap the numbers).
Quit with q in the video window or Ctrl+C.
================================================================================
"""

import os
os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "live mimic system"))

import json
import math
import socket
import time

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

# reuse the v3 building blocks — keep pose_sender.py in the same folder
from pose_sender import (OneEuro, SwapGuard, BoneCalibrator, Gates,
                         frame_angles, ensure_model, MODEL_PATH,
                         JOINT_NAMES, POSE_CONNECTIONS, DEADBAND, VIS_OFF,
                         L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW,
                         L_WRIST, R_WRIST, L_HIP, R_HIP, L_KNEE, R_KNEE,
                         L_ANKLE, R_ANKLE,
                         ARM_CHAIN_L, ARM_CHAIN_R, LEG_CHAIN_L, LEG_CHAIN_R)

CAM_A = 0                   # built-in camera (usually)
CAM_B = 1                   # iPhone via Continuity Camera (usually)
UDP_ADDR = ("127.0.0.1", 5005)
DISAGREE = 0.6              # rad: above this, trust only the better view

# which landmarks each joint's confidence depends on
REQ = {
    "L_shoulder_flex": [L_SHOULDER, L_ELBOW], "L_shoulder_abd": [L_SHOULDER, L_ELBOW],
    "L_elbow": [L_SHOULDER, L_ELBOW, L_WRIST],
    "L_shoulder_twist": [L_SHOULDER, L_ELBOW, L_WRIST],
    "R_shoulder_flex": [R_SHOULDER, R_ELBOW], "R_shoulder_abd": [R_SHOULDER, R_ELBOW],
    "R_elbow": [R_SHOULDER, R_ELBOW, R_WRIST],
    "R_shoulder_twist": [R_SHOULDER, R_ELBOW, R_WRIST],
    "L_hip_flex": [L_HIP, L_KNEE], "L_hip_abd": [L_HIP, L_KNEE],
    "L_knee": [L_HIP, L_KNEE, L_ANKLE],
    "R_hip_flex": [R_HIP, R_KNEE], "R_hip_abd": [R_HIP, R_KNEE],
    "R_knee": [R_HIP, R_KNEE, R_ANKLE],
}


class PoseCam:
    """One camera running the complete v3 pipeline, producing
    (angles, per-joint confidences) each frame."""

    def __init__(self, index, label):
        self.label = label
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"camera {index} ({label}) failed to open — check the index "
                "and that the iPhone is connected for Continuity Camera")
        self.latest = {"img": None, "world": None, "vis": None, "stamp": 0.0}
        self.gates = Gates()
        self.arm_guard = SwapGuard(L_WRIST, R_WRIST, ARM_CHAIN_L, ARM_CHAIN_R)
        self.leg_guard = SwapGuard(L_ANKLE, R_ANKLE, LEG_CHAIN_L, LEG_CHAIN_R)
        self.bones = BoneCalibrator()
        self.t0 = time.time()
        self.landmarker = self._make()

    def _make(self):
        options = vision.PoseLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=vision.RunningMode.LIVE_STREAM,
            result_callback=self._cb,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        return vision.PoseLandmarker.create_from_options(options)

    def _cb(self, result, output_image, timestamp_ms):
        try:
            if result.pose_landmarks and result.pose_world_landmarks:
                img = result.pose_landmarks[0]
                wld = result.pose_world_landmarks[0]
                self.latest["img"] = np.array([[lm.x, lm.y] for lm in img])
                self.latest["world"] = np.array([[lm.x, lm.y, lm.z] for lm in wld])
                self.latest["vis"] = np.array([lm.visibility for lm in img])
                self.latest["stamp"] = time.time()
        except Exception:
            pass

    def step(self):
        """Grab a frame, run detection, return (display_frame, angles, confs)."""
        ok, frame = self.cap.read()
        if not ok:
            return None, {}, {}
        try:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            self.landmarker.detect_async(
                mp_image, int((time.time() - self.t0) * 1000))
        except Exception:
            try:
                self.landmarker.close()
            except Exception:
                pass
            self.landmarker = self._make()
            self.t0 = time.time()
            return frame, {}, {}

        now = time.time()
        angles, confs = {}, {}
        h, w = frame.shape[:2]
        if self.latest["world"] is not None and now - self.latest["stamp"] < 0.5:
            world = self.latest["world"].copy()
            vis = self.latest["vis"]
            self.arm_guard.fix(world, now)
            self.leg_guard.fix(world, now)
            world = self.bones.process(world, vis)
            angles = frame_angles(world, vis, self.gates)
            confs = {k: float(min(vis[i] for i in REQ[k])) for k in angles}

            px = (self.latest["img"] * np.array([w, h])).astype(int)
            for a, b in POSE_CONNECTIONS:
                seen = min(vis[a], vis[b]) > VIS_OFF
                cv2.line(frame, tuple(px[a]), tuple(px[b]),
                         (0, 255, 0) if seen else (120, 120, 120), 2)
            if not self.bones.ready:
                cv2.putText(frame, f"calibrating {self.bones.progress()*100:.0f}%",
                            (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        else:
            cv2.putText(frame, "no person", (12, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"{self.label}: {len(angles)}/12", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        return frame, angles, confs


def fuse(a1, c1, a2, c2):
    """Per-joint confidence-weighted fusion of the two views."""
    out = {}
    for name in set(a1) | set(a2):
        in1, in2 = name in a1, name in a2
        if in1 and in2:
            if abs(a1[name] - a2[name]) > DISAGREE:
                out[name] = a1[name] if c1[name] >= c2[name] else a2[name]
            else:
                w1, w2 = c1[name] ** 2, c2[name] ** 2
                out[name] = (w1 * a1[name] + w2 * a2[name]) / (w1 + w2 + 1e-9)
        else:
            out[name] = a1[name] if in1 else a2[name]
    return out


def main():
    ensure_model()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cam_a = PoseCam(CAM_A, "cam A")
    cam_b = PoseCam(CAM_B, "cam B")
    filters = {n: OneEuro() for n in JOINT_NAMES}
    last_sent = {}

    print(f"Dual-camera fusion -> udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Place cameras 45-90 degrees apart. Both calibrate bones on startup.")
    print("Quit with q in the video window or Ctrl+C.\n")

    try:
        while True:
            try:
                fa, ang_a, conf_a = cam_a.step()
                fb, ang_b, conf_b = cam_b.step()

                fused = fuse(ang_a, conf_a, ang_b, conf_b)
                now = time.time()
                sent = {}
                for name, val in fused.items():
                    filt = filters[name](val, now)
                    if abs(filt - last_sent.get(name, 99.0)) > DEADBAND:
                        last_sent[name] = filt
                    sent[name] = last_sent[name]
                if sent:
                    sock.sendto(json.dumps(sent).encode(), UDP_ADDR)

                # side-by-side display
                tiles = []
                for f in (fa, fb):
                    if f is not None:
                        s = 360.0 / f.shape[0]
                        tiles.append(cv2.resize(f, None, fx=s, fy=s))
                if tiles:
                    view = np.hstack(tiles) if len(tiles) == 2 else tiles[0]
                    cv2.putText(view, f"fused: {len(sent)}/12 joints",
                                (12, view.shape[0] - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.imshow("dual pose (q to quit)", view)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except Exception as e:
                print(f"recovered from error: {e}")
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    cam_a.cap.release()
    cam_b.cap.release()
    cv2.destroyAllWindows()
    print("Stopped (by you).")


if __name__ == "__main__":
    main()
