"""
replay_pose.py  —  make the humanoid replicate motion captured from video
================================================================================
Loads running_pose.npz (produced by pose_extractor.py), converts the 33 MediaPipe
landmarks per frame into the 8 joint angles your humanoid.urdf understands
(L/R hip, knee, shoulder, elbow), then plays them back in the MuJoCo viewer,
looping forever. The figure runs in place, driven by the real person's motion.

Pipeline inside this file:
  1. load (T, 33, 4) pose tensor, convert to pixel coordinates
  2. fill gaps (NaN frames where detection failed) by interpolation
  3. auto-detect which way the person faces in the video
  4. landmarks -> 8 sagittal joint angles per frame (retargeting)
  5. smooth the angles (pose estimation is jittery frame-to-frame)
  6. resample from video fps to the sim's 60 Hz and play back

Run on Mac (same as walk.py):
    mjpython replay_pose.py
Requires: pip install mujoco scipy numpy
Expects humanoid.urdf and running_pose.npz in the same folder.
================================================================================
"""

import os
import time

import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import mujoco
import mujoco.viewer

here = os.path.dirname(os.path.abspath(__file__))
POSE_FILE = os.path.join(here, "running_pose.npz")
URDF_FILE = os.path.join(here, "humanoid.urdf")

# If any joint on the robot moves the WRONG WAY (e.g. knees bend backwards),
# flip its sign here — this depends on the axis conventions in your URDF.
SIGN = {"hip": 1.0, "knee": 1.0, "shoulder": 1.0, "elbow": 1.0}

SIM_FPS = 60.0

# MediaPipe landmark indices we need
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24
L_KNEE, R_KNEE = 25, 26
L_ANKLE, R_ANKLE = 27, 28
L_HEEL, R_HEEL = 29, 30
L_FOOT, R_FOOT = 31, 32


# ---------------------------------------------------------------------------
# 1-2. Load pose data, convert to pixels, fill detection gaps
# ---------------------------------------------------------------------------
def load_poses():
    data = np.load(POSE_FILE)
    poses = data["poses"].copy()          # (T, 33, 4) normalized
    fps = float(data["fps"])
    w, h = data["video_size"]

    xy = poses[:, :, :2]
    xy[:, :, 0] *= w                      # pixel coords so angles aren't
    xy[:, :, 1] *= h                      # distorted by the aspect ratio

    # interpolate over NaN frames (where no person was detected)
    T = xy.shape[0]
    t_idx = np.arange(T)
    for lm in range(33):
        for c in range(2):
            col = xy[:, lm, c]
            bad = np.isnan(col)
            if bad.any() and not bad.all():
                col[bad] = np.interp(t_idx[bad], t_idx[~bad], col[~bad])
    return xy, fps


# ---------------------------------------------------------------------------
# 3-4. Retarget: landmarks -> 8 sagittal joint angles
# ---------------------------------------------------------------------------
def segment_angle(xy, a, b, facing):
    """Signed angle of segment a->b relative to straight-down vertical,
    positive when b is FORWARD of a (in the person's facing direction).
    Note image y grows downward, so 'down' is +y."""
    dx = (xy[:, b, 0] - xy[:, a, 0]) * facing
    dy = xy[:, b, 1] - xy[:, a, 1]
    return np.arctan2(dx, dy)


def retarget(xy):
    # Which way is the person facing? Toes point forward of heels.
    facing = np.sign(np.nanmean(
        (xy[:, L_FOOT, 0] - xy[:, L_HEEL, 0]) +
        (xy[:, R_FOOT, 0] - xy[:, R_HEEL, 0])
    )) or 1.0
    print(f"Detected facing direction: {'right' if facing > 0 else 'left'} side of frame")

    # torso lean (shoulder->hip line), used as the reference for hip/shoulder
    torso = 0.5 * (segment_angle(xy, L_SHOULDER, L_HIP, facing) +
                   segment_angle(xy, R_SHOULDER, R_HIP, facing))

    ang = {}
    for side, hip, knee, ankle, sh, el, wr in [
        ("L", L_HIP, L_KNEE, L_ANKLE, L_SHOULDER, L_ELBOW, L_WRIST),
        ("R", R_HIP, R_KNEE, R_ANKLE, R_SHOULDER, R_ELBOW, R_WRIST),
    ]:
        thigh = segment_angle(xy, hip, knee, facing)
        shank = segment_angle(xy, knee, ankle, facing)
        uarm = segment_angle(xy, sh, el, facing)
        farm = segment_angle(xy, el, wr, facing)

        ang[f"{side}_hip"] = SIGN["hip"] * (thigh - torso)
        ang[f"{side}_knee"] = SIGN["knee"] * np.clip(thigh - shank, 0, None)
        ang[f"{side}_shoulder"] = SIGN["shoulder"] * (uarm - torso)
        ang[f"{side}_elbow"] = SIGN["elbow"] * np.clip(farm - uarm, 0, None)
    return ang


# ---------------------------------------------------------------------------
# 5-6. Smooth and resample to the sim clock
# ---------------------------------------------------------------------------
def smooth_and_resample(ang, fps):
    T = len(next(iter(ang.values())))
    window = min(15, T - (1 - T % 2))      # odd window <= 15
    duration = (T - 1) / fps
    t_video = np.arange(T) / fps
    t_sim = np.arange(0, duration, 1.0 / SIM_FPS)

    out = {}
    for name, series in ang.items():
        if window >= 5:
            series = savgol_filter(series, window, polyorder=3)
        out[name] = interp1d(t_video, series, kind="cubic")(t_sim)
    n = len(t_sim)
    print(f"Retargeted {T} video frames -> {n} sim frames "
          f"({duration:.2f}s loop at {SIM_FPS:.0f} Hz)")
    return out, n


# ---------------------------------------------------------------------------
# Playback in MuJoCo
# ---------------------------------------------------------------------------
def main():
    xy, fps = load_poses()
    ang, n_frames = smooth_and_resample(retarget(xy), fps)

    model = mujoco.MjModel.from_xml_path(URDF_FILE)
    data = mujoco.MjData(model)

    def q(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return model.jnt_qposadr[jid]

    qmap = {
        "L_hip": q("L_hip"), "L_knee": q("L_knee"),
        "R_hip": q("R_hip"), "R_knee": q("R_knee"),
        "L_shoulder": q("L_shoulder"), "L_elbow": q("L_elbow"),
        "R_shoulder": q("R_shoulder"), "R_elbow": q("R_elbow"),
    }

    print("Playing captured motion on the humanoid. Close window / Ctrl+C to stop.\n")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        i = 0
        while viewer.is_running():
            for name, adr in qmap.items():
                data.qpos[adr] = ang[name][i % n_frames]
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / SIM_FPS)
            i += 1

    print("\nStopped.")


if __name__ == "__main__":
    main()
