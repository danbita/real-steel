"""
Pose extraction pipeline (MediaPipe Tasks API, works with mediapipe >= 0.10.31):
  video -> PoseLandmarker landmarks per frame -> (T, 33, 4) tensor -> .npz file

Install:
  pip install mediapipe opencv-python numpy scipy

Usage:
  python pose_extractor.py runningvideo.mp4 running_pose.npz
  python pose_extractor.py runningvideo.mp4 running_pose.npz --preview

On first run this auto-downloads the pose model (~9 MB) into the script's folder.
"""

import argparse
import os
import urllib.request

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_full/float16/latest/pose_landmarker_full.task"
)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "pose_landmarker_full.task")

# The 33 landmarks, in index order (same as the old solutions API)
LANDMARK_NAMES = [
    "NOSE", "LEFT_EYE_INNER", "LEFT_EYE", "LEFT_EYE_OUTER",
    "RIGHT_EYE_INNER", "RIGHT_EYE", "RIGHT_EYE_OUTER",
    "LEFT_EAR", "RIGHT_EAR", "MOUTH_LEFT", "MOUTH_RIGHT",
    "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_PINKY", "RIGHT_PINKY",
    "LEFT_INDEX", "RIGHT_INDEX", "LEFT_THUMB", "RIGHT_THUMB",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE", "LEFT_HEEL", "RIGHT_HEEL",
    "LEFT_FOOT_INDEX", "RIGHT_FOOT_INDEX",
]

# Skeleton connections for the preview overlay
POSE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10),
    (11, 12), (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (27, 29), (29, 31), (27, 31),
    (24, 26), (26, 28), (28, 30), (30, 32), (28, 32),
]


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Downloading pose model to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Model downloaded.")


def draw_skeleton(frame, landmarks, w, h):
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in POSE_CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (0, 255, 0), 2)
    for p in pts:
        cv2.circle(frame, p, 3, (0, 0, 255), -1)


def extract_pose(video_path: str, preview: bool = False):
    """Returns (poses, fps, (width, height)); poses is (T, 33, 4) float32,
    channels = (x, y, z, visibility), NaN where no person detected."""
    ensure_model()

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    options = vision.PoseLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    frames = []
    with vision.PoseLandmarker.create_from_options(options) as landmarker:
        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

            # timestamps must be monotonically increasing, in milliseconds
            timestamp_ms = int(frame_idx * 1000.0 / fps)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            if result.pose_landmarks:  # list of detected people
                lms = result.pose_landmarks[0]
                frames.append(
                    [[lm.x, lm.y, lm.z, lm.visibility] for lm in lms]
                )
                if preview:
                    draw_skeleton(frame, lms, width, height)
            else:
                frames.append(np.full((33, 4), np.nan))

            if preview:
                cv2.imshow("pose preview (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_idx += 1
            if frame_idx % 100 == 0:
                print(f"  processed {frame_idx} frames...")

    cap.release()
    if preview:
        cv2.destroyAllWindows()

    poses = np.asarray(frames, dtype=np.float32)  # (T, 33, 4)
    return poses, fps, (width, height)


def compute_motion_features(poses: np.ndarray, fps: float) -> dict:
    """Derive basic motion data from the pose tensor."""
    dt = 1.0 / fps
    velocity = np.diff(poses[:, :, :3], axis=0) / dt      # (T-1, 33, 3)
    speed = np.linalg.norm(velocity, axis=2)              # (T-1, 33)

    def angle(a, b, c):
        v1 = poses[:, a, :3] - poses[:, b, :3]
        v2 = poses[:, c, :3] - poses[:, b, :3]
        cos = np.sum(v1 * v2, axis=1) / (
            np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1) + 1e-8
        )
        return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    return {
        "velocity": velocity.astype(np.float32),
        "speed": speed.astype(np.float32),
        "right_elbow_angle_deg": angle(12, 14, 16).astype(np.float32),
        "left_elbow_angle_deg": angle(11, 13, 15).astype(np.float32),
        "right_knee_angle_deg": angle(24, 26, 28).astype(np.float32),
        "left_knee_angle_deg": angle(23, 25, 27).astype(np.float32),
    }


def main():
    parser = argparse.ArgumentParser(description="Video -> pose tensor extractor")
    parser.add_argument("video", help="input video path")
    parser.add_argument("output", help="output .npz path")
    parser.add_argument("--preview", action="store_true", help="show live skeleton overlay")
    args = parser.parse_args()

    print(f"Extracting pose from {args.video} ...")
    poses, fps, (w, h) = extract_pose(args.video, preview=args.preview)
    print(f"Done: {poses.shape[0]} frames, tensor shape {poses.shape}")

    detected = np.mean(~np.isnan(poses[:, 0, 0])) * 100
    print(f"Person detected in {detected:.1f}% of frames")

    motion = compute_motion_features(poses, fps)

    np.savez_compressed(
        args.output,
        poses=poses,                       # (T, 33, 4): x, y, z, visibility
        fps=fps,
        video_size=np.array([w, h]),
        landmark_names=np.array(LANDMARK_NAMES),
        **motion,
    )
    print(f"Saved to {args.output}")
    print("\nLoad it back with:")
    print("  data = np.load('output.npz')")
    print("  poses = data['poses']   # (T, 33, 4)")


if __name__ == "__main__":
    main()
