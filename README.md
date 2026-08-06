# Real Steel — Human Motion Capture → Simulated Humanoid

Move in front of a webcam and a simulated humanoid robot copies you in real
time. Or feed it a video and it replays the motion. Built with
[MediaPipe](https://developers.google.com/mediapipe) pose estimation and the
[MuJoCo](https://mujoco.org) physics simulator.

The pipeline: **video / webcam → 33 body landmarks per frame → 8 joint angles
(retargeting) → animated humanoid**.

## How it works

MediaPipe's PoseLandmarker detects 33 body landmarks (head, shoulders, elbows,
wrists, hips, knees, ankles, feet...) in each frame, each with x/y/z
coordinates and a visibility score. A retargeting step converts those raw
positions into the 8 sagittal-plane joint angles the robot understands —
left/right hip, knee, shoulder, and elbow — by measuring segment angles with
`atan2` (e.g. knee flexion = thigh angle − shank angle, referenced against the
torso line). Smoothing, visibility gating, and rate limiting clean up the
jitter inherent in per-frame pose estimation, and the result drives the
humanoid's joints in the MuJoCo viewer.

The current system is kinematic (joint angles are written directly, no physics
balancing yet) and 2D — it captures forward/back motion, so it works best when
the person is **side-on to the camera**.

## Repo layout

```
live system (real time, two processes)
  pose_sender.py        webcam + pose tracking + skeleton overlay window,
                        streams joint angles over UDP        [run with python]
  mimic_receiver.py     receives angles, animates the humanoid [run with mjpython]

offline pipeline (from a video file)
  pose_extractor.py     video -> (T, 33, 4) pose tensor -> .npz
  replay_pose.py        .npz -> retargeted joint angles -> humanoid playback
  inspect_pose.py       prints a summary of a .npz + saves motion plots

model
  humanoid.urdf         torso + 2-segment arms and legs, 8 hinge joints
```

The `pose_landmarker_*.task` files (MediaPipe models) are downloaded
automatically on first run.

## Setup

Requires Python ≤ 3.12 recommended (3.13 works with current mediapipe), macOS /
Linux / Windows. Developed on an Apple Silicon Mac.

```bash
pip install mediapipe opencv-python mujoco numpy scipy matplotlib
```

## Usage

### Live mimicry (webcam)

Open **two terminals** in the repo folder:

```bash
# terminal 1 — camera, tracking, skeleton overlay (plain python)
python pose_sender.py

# terminal 2 — the humanoid (mjpython on macOS, python elsewhere)
mjpython mimic_receiver.py
```

Stand back so your full body is in frame, side-on to the camera, and move.
Press `q` in the camera window to quit.

### Offline (video file)

```bash
python pose_extractor.py runningvideo.mp4 running_pose.npz --preview
python inspect_pose.py running_pose.npz          # optional: summary + plots
mjpython replay_pose.py                          # humanoid replays the motion
```

The `.npz` contains the `(T, 33, 4)` pose tensor (x, y, z, visibility per
landmark), fps, video size, landmark names, per-landmark velocities, and
example joint-angle series.

## Tuning

| Knob | File | Effect |
|---|---|---|
| `SMOOTH_ALPHA` | mimic_receiver.py | lower = smoother, higher = snappier |
| `MAX_RATE` | mimic_receiver.py | rad/s cap per joint; kills tracking spikes |
| `MIN_VISIBILITY` | pose_sender.py | occluded joints hold pose instead of flailing |
| `SIGN` | pose_sender.py / replay_pose.py | flip if a joint mirrors backwards |
| `CAMERA_INDEX` | pose_sender.py | pick a different camera |
| model URL (`lite` ↔ `full`) | pose_sender.py | speed vs tracking accuracy |

## macOS notes (hard-won)

- The MuJoCo viewer must run under `mjpython`; OpenCV windows and the MuJoCo
  viewer **cannot share one process** on macOS (both need the main thread).
  That's why the live system is two processes talking over UDP (port 5005).
- Camera permission under `mjpython` can't trigger the macOS consent popup.
  The scripts set `OPENCV_AVFOUNDATION_SKIP_AUTH=1`; grant camera access to
  your terminal once via plain `python` first (System Settings → Privacy &
  Security → Camera).
- mediapipe ≥ 0.10.31 removed the legacy `mp.solutions` API — this project
  uses the Tasks API (`PoseLandmarker`) throughout.

## Roadmap

- Retarget more joints (neck, ankles) onto the richer `humanoid_full.urdf`
- 3D retargeting from `pose_world_landmarks` (twisting, lateral motion)
- Physics: gravity on, PD controllers tracking the captured angles as
  reference trajectories (DeepMimic-style), so the robot balances instead of
  being puppeteered
