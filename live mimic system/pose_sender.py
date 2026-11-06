"""
pose_sender.py  (v3)  —  full 3D motion capture -> 12 joint angles over UDP
================================================================================
Run with plain python (NOT mjpython):     python pose_sender.py
Pair with:                                mjpython mimic_receiver.py
Quit ONLY with q in the camera window or Ctrl+C.

What's new in v3:
  - 3D RETARGETING. Uses MediaPipe's world landmarks (metric 3D positions) and
    measures every angle in a coordinate frame built from YOUR OWN hips and
    shoulders. Camera angle no longer matters: no facing detection, no
    projection, no inversion bugs by construction. Sideways motion is captured.
  - 12 JOINTS. Shoulders and hips now send flexion (forward/back) AND
    abduction (out to the side). Elbows/knees are true 3D bend angles.
  - ONE EURO FILTER per joint: locks rock-solid when you're still, stays
    responsive when you move fast. Replaces the old EMA+median stack.
  - DEADBAND: sub-degree wiggle is not motion and is not sent.
  - LEFT/RIGHT SWAP GUARD: if MediaPipe's left/right labels jump across the
    body between frames (classic when legs cross), the swap is detected by
    continuity and corrected before angles are computed.
  - Visibility gates now have hysteresis (on > 0.7, off < 0.5) so a joint at
    the threshold can't flicker.

Convention sent over the wire (matches humanoid_v3.urdf exactly):
  *_flex     0 = limb hanging down, positive = forward
  *_abd      0 = limb hanging down, positive = out to the side (both sides)
  elbow/knee 0 = straight, positive = bending (elbow forward, knee backward)
================================================================================
"""

import os
import sys
os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"

import json
import math
import socket
import threading
import time
import urllib.request

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

here = os.path.dirname(os.path.abspath(__file__))

# --model lite|full|heavy  (default full). lite is roughly 2x faster and a
# little less accurate; heavy is the reverse. Each is downloaded on first use.
MODEL_NAME = (sys.argv[sys.argv.index("--model") + 1]
              if "--model" in sys.argv else "full")
if MODEL_NAME not in ("lite", "full", "heavy"):
    raise SystemExit("--model must be lite, full or heavy")
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             f"pose_landmarker_{MODEL_NAME}/float16/latest/"
             f"pose_landmarker_{MODEL_NAME}.task")
MODEL_PATH = os.path.join(here, f"pose_landmarker_{MODEL_NAME}.task")
# --gpu : run MediaPipe on the GPU (Metal on a Mac). Falls back to CPU if the
#   GPU delegate will not start.
USE_GPU = "--gpu" in sys.argv
# Camera capture size, and the width frames are shrunk to before inference.
# MediaPipe resizes to 256x256 internally anyway, so feeding it a 1080p frame
# only costs colour-conversion and copy time - that was a big share of the
# 5-15 fps. Landmarks come back NORMALISED (and world landmarks in metres), so
# shrinking the inference image changes nothing downstream.
CAP_W, CAP_H = (int(v) for v in (sys.argv[sys.argv.index("--res") + 1].split("x")
                                 if "--res" in sys.argv else ("1280", "720")))
INFER_W = 640

CAMERA_INDEX = 0
# --source <path|index>: run the exact live pipeline off a video file instead of
# the webcam. Same code path, so it is a real test of the loop, not a mock.
UDP_ADDR = ("127.0.0.1", 5005)

VIS_ON, VIS_OFF = 0.7, 0.5      # gate hysteresis thresholds
DEADBAND = 0.03                 # rad (~1.7 deg): smaller changes aren't motion
SWAP_MARGIN = 0.05              # meters, for the L/R swap continuity test

# One Euro filter tuning: raise OE_MIN_CUTOFF if idle drift remains,
# raise OE_BETA if fast motion feels laggy.
OE_MIN_CUTOFF = 1.0
OE_BETA = 0.4
OE_D_CUTOFF = 1.0

# flip any joint family if it mirrors on your setup
SIGN = {"shoulder_flex": 1.0, "shoulder_abd": 1.0, "shoulder_twist": 1.0,
        "elbow": 1.0, "hip_flex": 1.0, "hip_abd": 1.0, "knee": 1.0,
        # Set these from sim36/neck_chain_check.py section 6, which prints which
        # way the modelled head actually moves for +20 deg on each joint - not
        # by guessing.
        "neck_yaw": 1.0, "neck_pitch": 1.0, "neck_roll": 1.0}

LIMITS = {
    "shoulder_flex": (-1.2, 3.1), "shoulder_abd": (-0.5, 3.0),
    "shoulder_twist": (-1.5708, 1.5708),
    "elbow": (0.0, 2.7),
    "hip_flex": (-1.0, 2.1), "hip_abd": (-0.4, 1.2),
    "knee": (0.0, 2.4),
    # Mirror neck_v1.urdf exactly (gen_neck.py: 90% of the servo's travel), so
    # the sender never commands a pose the model will refuse. Regenerate both if
    # SERVO_TRAVEL_DEG there changes.
    "neck_yaw": (-1.413717, 1.413717),
    "neck_pitch": (-1.413717, 1.413717),
    "neck_roll": (-1.413717, 1.413717),
}

# The neck model is 2 DOF - yaw and pitch. See gen_neck.py: the CAD's own part
# names are YesHolder (nod) and NoHolder (turn), and once the export's
# double-counted parts are removed only two moving groups remain. Roll is
# measured anyway, because it is free once the head frame exists and it is the
# evidence that would justify a third axis, but it is NOT sent by default -
# a receiver driving neck_v1.urdf has no joint to put it in.
SEND_NECK_ROLL = False

# Below this, the ear-to-ear baseline is too short to trust for roll: |sin| of
# the head's tilt out of the body's frontal plane. Ear world-depth is the weakest
# signal MediaPipe gives, so roll degrades first.
NECK_GIMBAL_MIN = 0.25

NOSE = 0
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24
L_KNEE, R_KNEE = 25, 26
L_ANKLE, R_ANKLE = 27, 28
L_HEEL, R_HEEL = 29, 30
L_FOOT, R_FOOT = 31, 32

# Face landmarks, for the neck. MediaPipe Pose numbering: 0 nose, 2/5 the eye
# centres, 7/8 the ear tragions. The EARS are the primary baseline - they sit on
# the skull's own left-right axis and barely move with expression, where the eyes
# are 60 mm apart and their world-depth is the noisiest part of the whole
# landmark set. Eyes are only a fallback when an ear is occluded.
L_EYE, R_EYE = 2, 5
L_EAR, R_EAR = 7, 8

ARM_CHAIN_L, ARM_CHAIN_R = [13, 15, 17, 19, 21], [14, 16, 18, 20, 22]
LEG_CHAIN_L, LEG_CHAIN_R = [25, 27, 29, 31], [26, 28, 30, 32]

POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (27, 29), (29, 31), (27, 31),
    (24, 26), (26, 28), (28, 30), (30, 32), (28, 32),
]

JOINT_NAMES = ["L_shoulder_flex", "L_shoulder_abd", "L_shoulder_twist", "L_elbow",
               "R_shoulder_flex", "R_shoulder_abd", "R_shoulder_twist", "R_elbow",
               "L_hip_flex", "L_hip_abd", "L_knee",
               "R_hip_flex", "R_hip_abd", "R_knee",
               "neck_yaw", "neck_pitch", "neck_roll"]


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print(f"Downloading {MODEL_NAME} pose model...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


class CameraStream:
    """Reads a camera on its OWN thread and keeps only the newest frame.

    cv2.VideoCapture.read() blocks until the camera delivers, and OpenCV keeps
    a small buffer - so a loop that reads, then runs pose, then draws, both
    waits on the camera AND processes frames that are already old. Here the
    grabber thread drains the camera continuously; the main loop always gets
    the most recent frame and never waits for the hardware.
    """

    FIRST_FRAME_S = 15.0     # an iPhone can take seconds to send its first frame
    DEAD_S = 8.0             # no frame for this long after frames = dropped

    def __init__(self, src, width=None, height=None, name="cam"):
        self.name = name
        self.src = src
        self.wh = (width, height)
        self.cap = self._open()
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {src!r}.")
        self.frame, self.seq, self.stamp = None, 0, 0.0
        self.ok = True
        self.nread = 0
        self.reconnects = 0
        self.t_open = time.time()
        self.t_last = None           # wall time of the newest frame
        self._lock = threading.Lock()
        self._stop = False
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _open(self):
        cap = cv2.VideoCapture(self.src)
        w, h = self.wh
        if cap.isOpened() and w and h:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    def status(self):
        """'live', or a short plain-English reason there is no video."""
        now = time.time()
        if self.t_last is not None and now - self.t_last < 1.0:
            return "live"
        if self.t_last is None:
            return f"waiting for first frame ({now - self.t_open:.0f} s)"
        return (f"NO VIDEO for {now - self.t_last:.0f} s"
                + (f", reconnecting (try {self.reconnects})" if self.reconnects else ""))

    def _run(self):
        # A Continuity Camera iPhone drops out when it locks, sleeps, moves
        # away or loses Wi-Fi/Bluetooth, and macOS can hand the stream back a
        # moment later. So a dead camera is REOPENED every few seconds instead
        # of being left as a frozen or black tile for the rest of the run.
        # A REOPENED camera gets the same FIRST_FRAME_S to start as a new one:
        # reopening an iPhone faster than it can wake up (the first version did,
        # every 3 s) keeps killing it before its first frame, so it never comes
        # back.
        t_opened = time.time()
        while not self._stop:
            ok, f = self.cap.read() if self.cap is not None else (False, None)
            now = time.time()
            if not ok or f is None:
                fresh_since = max(t_opened, self.t_last or 0.0)
                limit = self.DEAD_S if (self.t_last or 0.0) > t_opened else self.FIRST_FRAME_S
                if now - fresh_since > limit:
                    self.ok = False
                    self.reconnects += 1
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = self._open()
                    t_opened = time.time()
                time.sleep(0.02)
                continue
            self.t_last = now
            self.ok = True
            with self._lock:
                self.frame, self.seq, self.stamp = f, self.seq + 1, now
                self.nread += 1

    def latest(self):
        with self._lock:
            return self.frame, self.seq, self.stamp

    def size(self):
        return (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def release(self):
        self._stop = True
        self._t.join(timeout=1.0)
        self.cap.release()


def infer_image(frame):
    """BGR camera frame -> mp.Image, shrunk to INFER_W wide."""
    h, w = frame.shape[:2]
    if w > INFER_W:
        frame = cv2.resize(frame, (INFER_W, int(round(h * INFER_W / w))),
                           interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)


# ---------------------------------------------------------------------------
# One Euro filter — adaptive smoothing (Casiez et al.):
# heavy smoothing at low speed (kills idle jitter), light at high speed.
# ---------------------------------------------------------------------------
class OneEuro:
    def __init__(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = max(t - self.t_prev, 1e-3)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(OE_D_CUTOFF, dt)
        self.dx_prev = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = OE_MIN_CUTOFF + OE_BETA * abs(self.dx_prev)
        a = self._alpha(cutoff, dt)
        self.x_prev = a * x + (1 - a) * self.x_prev
        self.t_prev = t
        return self.x_prev


# ---------------------------------------------------------------------------
# Left/right swap guard: if the new "left" wrist/ankle is closer to where the
# RIGHT one just was (and vice versa), MediaPipe swapped labels — swap back.
# ---------------------------------------------------------------------------
class SwapGuard:
    def __init__(self, probe_l, probe_r, chain_l, chain_r):
        self.pl, self.pr = probe_l, probe_r
        self.cl, self.cr = chain_l, chain_r
        self.prev_l = None
        self.prev_r = None
        self.last_t = 0.0
        self.last_swap = False     # diagnostics: did THIS frame get swapped?
        self.nswap = 0

    def fix(self, world, t):
        self.last_swap = False
        L, R = world[self.pl].copy(), world[self.pr].copy()
        if self.prev_l is not None and t - self.last_t < 0.3:
            keep = np.linalg.norm(L - self.prev_l) + np.linalg.norm(R - self.prev_r)
            swap = np.linalg.norm(L - self.prev_r) + np.linalg.norm(R - self.prev_l)
            self.last_swap = bool(swap + SWAP_MARGIN < keep)
            if self.last_swap:
                self.nswap += 1
                for a, b in zip(self.cl, self.cr):
                    world[[a, b]] = world[[b, a]]
                L, R = world[self.pl].copy(), world[self.pr].copy()
        self.prev_l, self.prev_r, self.last_t = L, R, t


# ---------------------------------------------------------------------------
# Bone-length depth correction — the single-camera fix for backward glitches.
# Bones don't change length, so apparent shortening = foreshortening. After a
# short calibration learns your true segment lengths, each frame's depth
# offsets are RECOMPUTED from geometry (|dz| = sqrt(L^2 - dxy^2)) instead of
# trusting MediaPipe's noisy z. The depth SIGN keeps its previous value unless
# the raw z gives strong contrary evidence, so limbs can't flip backward on a
# single bad frame.
# ---------------------------------------------------------------------------
class BoneCalibrator:
    CHAINS = [[L_SHOULDER, L_ELBOW, L_WRIST], [R_SHOULDER, R_ELBOW, R_WRIST],
              [L_HIP, L_KNEE, L_ANKLE], [R_HIP, R_KNEE, R_ANKLE]]
    SAMPLES_NEEDED = 120          # ~4-5 s of well-tracked frames

    def __init__(self):
        self.samples = {}         # (parent, child) -> [lengths]
        self.ref = {}             # (parent, child) -> calibrated length
        self.prev_dz = {}
        self.ema_dz = {}
        self.ready = False

    def progress(self):
        if self.ready:
            return 1.0
        counts = [len(self.samples.get((c[i], c[i + 1]), []))
                  for c in self.CHAINS for i in range(2)]
        return min(counts) / self.SAMPLES_NEEDED if counts else 0.0

    def process(self, world, vis):
        if not self.ready:
            for chain in self.CHAINS:
                for p, c in zip(chain, chain[1:]):
                    if min(vis[p], vis[c]) > 0.7:
                        self.samples.setdefault((p, c), []).append(
                            float(np.linalg.norm(world[c] - world[p])))
            if self.progress() >= 1.0:
                # p90 of observed lengths ~ true length (foreshortened frames
                # only ever make a bone look SHORTER, never longer)
                self.ref = {k: float(np.percentile(v, 90))
                            for k, v in self.samples.items()}
                self.ready = True
                print("Bone calibration complete:",
                      {f"{k}": round(v, 3) for k, v in self.ref.items()})
            return world

        for chain in self.CHAINS:
            for p, c in zip(chain, chain[1:]):
                L0 = self.ref[(p, c)]
                d = world[c] - world[p]
                dxy = math.hypot(d[0], d[1])
                dz2 = L0 * L0 - dxy * dxy
                if dz2 <= 0:
                    dz = 0.0          # segment lies (nearly) in the image plane
                else:
                    mag = math.sqrt(dz2)
                    raw = d[2]
                    prev = self.prev_dz.get((p, c), raw)
                    if abs(raw) > 0.35 * L0:      # strong evidence: trust it
                        s = math.copysign(1.0, raw)
                    else:                          # weak evidence: continuity
                        s = math.copysign(1.0, prev if prev != 0 else 1.0)
                    dz = s * mag
                self.prev_dz[(p, c)] = dz
                world[c] = world[p] + np.array([d[0], d[1], dz])
        return world



class Gates:
    def __init__(self):
        self.state = {}

    def check(self, key, worst_vis, on=VIS_ON, off=VIS_OFF):
        s = self.state.get(key, False)
        if s and worst_vis < off:
            s = False
        elif not s and worst_vis > on:
            s = True
        self.state[key] = s
        return s


# ---------------------------------------------------------------------------
# 3D retargeting in the body frame.
# fwd = out of the chest, left = toward the person's left, up = along the spine.
# Built purely from the person's own hips/shoulders -> camera angle irrelevant.
# ---------------------------------------------------------------------------
def unit(v):
    return v / (np.linalg.norm(v) + 1e-9)


TWIST_MIN_FLEX = 0.42          # sin(25 deg): below this the bend plane is noise


def twist_from_bend(upper, fore, fwd):
    """Humeral internal/external rotation, from the plane the elbow bends in.

    The shoulder->elbow vector is a direction: 2 DOF, so it cannot see twist at
    all. But rotating the humerus carries the elbow's bend plane around with it,
    so the direction the forearm folds resolves the third DOF.

    Zero twist is defined as "the elbow bends straight forward" in the body
    frame, which is the anatomically neutral arm.

    Returns (twist, ok). ok is False near full extension, where the across-axis
    component of the forearm shrinks like sin(flexion) and its direction becomes
    pure noise - the same singularity the flexion estimate has when the limb goes
    fully lateral. Callers should hold the previous value there."""
    u = unit(upper)
    b = fore - np.dot(fore, u) * u              # forearm, across the arm axis
    nb = np.linalg.norm(b)
    ua, fl = np.linalg.norm(upper), np.linalg.norm(fore)
    sin_flex = nb / (fl + 1e-9)                 # = sin(elbow flexion)
    r0 = fwd - np.dot(fwd, u) * u               # reference: bends straight forward
    if nb < 1e-6 or np.linalg.norm(r0) < 0.25 or ua < 1e-6:
        return 0.0, False
    b, r0 = unit(b), unit(r0)
    tw = math.atan2(float(np.dot(u, np.cross(r0, b))), float(np.dot(r0, b)))
    return tw, sin_flex > TWIST_MIN_FLEX


def clamp(fam, val):
    lo, hi = LIMITS[fam]
    return max(lo, min(hi, SIGN[fam] * val))



# ---------------------------------------------------------------------------
# BONE-LENGTH CONSTRAINED DEPTH
# ---------------------------------------------------------------------------
# MediaPipe's x,y are good and its z is not. Measured on a turn-in-place clip:
# the upper arm, a rigid bone, varied 240.0 +/- 20.9 mm with a 130.6 mm spread
# (8.7% cv); the right forearm 12.6% cv; the shoulder line 6.4%. Torso yaw from
# its z averaged -7 deg while the 2D shoulder foreshortening showed up to 76.
#
# A bone cannot change length. Given the true length L and the in-plane offset
# (dx,dy), depth follows:      dz = +/- sqrt(L^2 - dx^2 - dy^2)
# so z stops being a guess and becomes a consequence. Only the SIGN is
# ambiguous, and that is resolved by continuity with the previous frame.
#
# L is calibrated per bone as a high percentile of the observed in-plane length:
# when a limb is square to the camera dz ~ 0 and the projection IS the bone.
class BoneDepth:
    """Rebuild z from fixed bone lengths. Call update() per frame."""

    LIMBS = None          # filled in below, after the landmark ids exist

    def __init__(self, warmup=45):
        self.hist = {}
        self.L = {}
        self.prev_sign = {}
        self.prev_dz = {}
        self.ema_dz = {}
        self.n = 0
        self.warmup = warmup

    def _calib(self, key, dxy):
        h = self.hist.setdefault(key, [])
        h.append(dxy)
        if len(h) > 900:
            del h[0]
        if len(h) >= self.warmup:
            # p90 not p97: the tail is tracker noise, and an inflated L
            # forces a spurious dz that shows up as length error.
            self.L[key] = float(np.percentile(h, 90))

    def update(self, world):
        self.n += 1
        w = world.copy()
        for key, (a, b) in self.LIMBS.items():
            d = w[b] - w[a]
            dxy = float(np.hypot(d[0], d[1]))
            self._calib(key, dxy)
            L = self.L.get(key)
            if L is None or L <= 1e-6:
                continue
            inner = L * L - dxy * dxy
            mag = float(np.sqrt(inner)) if inner > 0 else 0.0
            # SIGN. sqrt gives magnitude only; the sign is genuinely ambiguous
            # from one camera - the limb could point toward or away. Choosing it
            # from MediaPipe's own z sign flips whenever that noisy value crosses
            # zero, and a flip MIRRORS the limb: measured 136 deg/frame jumps,
            # worse than doing nothing. Depth is continuous in reality, so pick
            # the sign that stays closest to the previous frame's depth, and only
            # flip when the evidence clearly favours it (hysteresis).
            # Sign anchor: a SMOOTHED MediaPipe dz. Its magnitude is unusable
            # (bones flex 130-190 mm) but its SIGN is mostly right - its yaw
            # still correlated +0.79 with 2D foreshortening. Pure continuity
            # was not enough on its own: it locked onto the wrong branch and
            # the rate limiter held it there, mirroring the torso for whole
            # sections of the clip. That was invisible to a |yaw| metric and
            # only showed up in a top-down plot of the reconstruction.
            ema = self.ema_dz.get(key)
            ema = d[2] if ema is None else 0.75 * ema + 0.25 * d[2]
            self.ema_dz[key] = ema
            prev = self.prev_dz.get(key)
            if abs(ema) > 0.012:
                sgn = np.sign(ema)
            elif prev is not None:
                sgn = np.sign(prev)
            else:
                sgn = 1.0
            if sgn == 0:
                sgn = 1.0
            dz = sgn * mag
            # rate-limit depth: a real limb cannot teleport in z between frames
            if prev is not None:
                dz = float(np.clip(dz, prev - 0.06, prev + 0.06))
            self.prev_dz[key] = dz
            w[b] = w[a] + np.array([d[0], d[1], dz])
        return w

    def ready(self):
        return len(self.L) >= 4

def body_frame(world, hips_ok=True):
    """Body frame from the person's own torso: fwd out of the chest, left toward
    their left, up along the spine.

    WHEN THE HIPS ARE NOT VISIBLE, UP COMES FROM GRAVITY - NOT FROM THE HEAD.

    The previous fallback used up = nose - mid_shoulder. It restored pose
    coverage from 75% to 100%, and it silently wrecked every angle downstream:
    people lean their head forward when they talk, so that vector sat a MEASURED
    47.7 deg off vertical on a talking-head clip, on 93.8% of its frames. Arms
    hanging at 20 deg from vertical were reported at 54 deg, and the robot
    faithfully reproduced the wrong number.

    MediaPipe's world landmarks are already gravity-referenced (+y is down), so
    when the torso is unobservable, gravity is a far better estimate of the
    spine than the head is. The trade is that camera roll is no longer cancelled
    out - if the camera is tilted, this frame tilts with it. Hips remain the
    preferred path precisely because they are torso-relative; the real fix for a
    clip like that is to frame the shot so the hips are in it.
    """
    mid_sh = 0.5 * (world[L_SHOULDER] + world[R_SHOULDER])
    left = unit(world[L_SHOULDER] - world[R_SHOULDER])
    if hips_ok:
        left = unit(world[L_HIP] - world[R_HIP])
        mid_hip = 0.5 * (world[L_HIP] + world[R_HIP])
        up = unit(mid_sh - mid_hip)
    else:
        up = np.array([0.0, -1.0, 0.0])      # MediaPipe world: +y is down
    fwd = unit(np.cross(left, up))
    left = np.cross(up, fwd)          # re-orthogonalize
    up = np.cross(fwd, left)          # and again, so the triad is exact
    return fwd, left, up


class NeckNeutral:
    """Learns the head's RESTING orientation relative to the torso, then reports
    every later frame relative to that.

    THIS IS NOT A REFINEMENT, IT IS REQUIRED FOR CORRECTNESS.
        head_frame() builds forward from (nose - ear_midpoint) with the ear-line
        component removed. On a real head the nose sits BELOW the ear line - on
        the order of 15-20 deg below - so the recovered frame is pitched down by
        that angle even when the person is looking dead ahead. Sent raw, Atom
        stares at the floor at rest and every nod carries a constant offset. The
        offset is a property of face geometry, so it differs per person and
        cannot be hard-coded.

        Taking the neutral out also cancels resting head posture (most people
        carry the head slightly forward and a few degrees off-axis), which is
        what you want when the job is to MIRROR movement rather than to measure
        absolute anatomy.

    Rotations are averaged as matrices and projected back onto SO(3) with an SVD.
    Averaging the three Euler angles instead would be wrong - they do not
    commute, and the mean of a set of angle triples is not the triple of the mean
    rotation.
    """

    verbose = True
    N = 45      # ~1.5 s at 30 fps. Long enough to average out landmark jitter,
                # short enough that nobody holds a pose waiting for it.

    def __init__(self):
        self.acc = []
        self.R0 = None

    @property
    def ready(self):
        return self.R0 is not None

    def progress(self):
        return min(1.0, len(self.acc) / float(self.N))

    def observe(self, R):
        if self.ready:
            return
        self.acc.append(np.asarray(R, dtype=float))
        if len(self.acc) >= self.N:
            M = sum(self.acc) / float(len(self.acc))
            U, _, Vt = np.linalg.svd(M)      # nearest rotation to the mean
            self.R0 = U @ Vt
            if float(np.linalg.det(self.R0)) < 0:
                U[:, -1] *= -1.0             # guard against a reflection
                self.R0 = U @ Vt
            if self.verbose:
                print("Neck neutral captured (head-vs-torso rest pose).")

    def rel(self, R):
        return np.asarray(R, dtype=float) if self.R0 is None else self.R0.T @ R


NECK_NEUTRAL = NeckNeutral()


def head_frame(world, vis):
    """The skull's own orthonormal triad, or None if the face is not observable.

    Returns (h_fwd, h_left, h_up, used_eyes). Same handedness convention as
    body_frame: fwd = left x up, so the two frames can be compared directly.

    WHY THE EARS AND NOT THE EYES
        The ear tragions are ~150 mm apart and sit on the skull's true
        left-right axis, so they give the longest, most rigid baseline the pose
        model offers above the shoulders. The eyes are ~60 mm apart; halving the
        baseline doubles the angular error for the same landmark noise, and
        MediaPipe's world-landmark DEPTH on face points is its weakest output.
        Eyes are therefore a degraded fallback for when a head turn occludes an
        ear, which happens well before 90 deg of yaw - and the caller is told
        via used_eyes so it can hold roll rather than trust it.
    """
    if vis[NOSE] < VIS_OFF:
        return None
    ears_ok = min(vis[L_EAR], vis[R_EAR]) >= VIS_OFF
    eyes_ok = min(vis[L_EYE], vis[R_EYE]) >= VIS_OFF
    if ears_ok:
        span, mid = world[L_EAR] - world[R_EAR], 0.5 * (world[L_EAR] + world[R_EAR])
        used_eyes = False
    elif eyes_ok:
        span, mid = world[L_EYE] - world[R_EYE], 0.5 * (world[L_EYE] + world[R_EYE])
        used_eyes = True
    else:
        return None

    if np.linalg.norm(span) < 1e-6:
        return None
    h_left = unit(span)
    nose_out = world[NOSE] - mid              # out of the face, roughly
    h_up = np.cross(nose_out, h_left)         # up = fwd x left
    if np.linalg.norm(h_up) < 1e-6:
        return None                           # nose collinear with the ear line
    h_up = unit(h_up)
    h_fwd = unit(np.cross(h_left, h_up))      # fwd = left x up
    h_left = np.cross(h_up, h_fwd)            # re-orthogonalize, exactly
    return h_fwd, h_left, h_up, used_eyes


def neck_angles(world, vis, gates, fwd, left, up, neutral=None):
    """Head orientation RELATIVE TO THE TORSO -> neck yaw / pitch / roll.

    The whole point of measuring in the body frame, exactly as the limbs already
    do: the answer is what the neck is doing, not where the camera is. Lean the
    torso, tilt the camera, walk around the person - these angles do not move.
    A head-vs-camera measurement would fold all of that in.

    Decomposition is intrinsic yaw -> pitch -> roll (Rz Ry Rx in the body's
    fwd/left/up basis), matching the order the joints stack in neck_v1.urdf.
    Composition order matters: applying the same three numbers in a different
    order gives a different head pose.
    """
    if not gates.check("neck", min(vis[NOSE],
                                   max(min(vis[L_EAR], vis[R_EAR]),
                                       min(vis[L_EYE], vis[R_EYE])))):
        return {}
    hf = head_frame(world, vis)
    if hf is None:
        return {}
    h_fwd, h_left, h_up, used_eyes = hf

    # R[i][j] = (body axis i) . (head axis j)
    B = (fwd, left, up)
    R = np.array([[float(np.dot(B[i], H)) for H in (h_fwd, h_left, h_up)]
                  for i in range(3)])

    neutral = NECK_NEUTRAL if neutral is None else neutral
    neutral.observe(R)
    if not neutral.ready:
        return {}                 # biased until the rest pose is known - see NeckNeutral
    R = neutral.rel(R)

    out = {}
    # pitch first: it is the one with a singularity, and it gates the others.
    sp = max(-1.0, min(1.0, -R[2][0]))
    pitch = math.asin(sp)                     # + = nose down, matching *_flex
    cp = math.hypot(R[0][0], R[1][0])         # cos(pitch)
    out["neck_pitch"] = clamp("neck_pitch", SIGN["neck_pitch"] * pitch)

    if cp > NECK_GIMBAL_MIN:
        # yaw: + = face turns toward the person's OWN left
        out["neck_yaw"] = clamp("neck_yaw", SIGN["neck_yaw"] * math.atan2(R[1][0], R[0][0]))
        if SEND_NECK_ROLL and not used_eyes:
            out["neck_roll"] = clamp("neck_roll",
                                     SIGN["neck_roll"] * math.atan2(R[2][1], R[2][2]))
    # else: near-vertical nose, yaw and roll trade off against each other and
    # noise picks the split. Holding the previous value beats sending a guess.
    return out


def frame_angles(world, vis, gates):
    # Shoulders are mandatory - without them there is no torso to reference.
    if min(vis[L_SHOULDER], vis[R_SHOULDER]) < VIS_OFF:
        return {}
    hips_ok = min(vis[L_HIP], vis[R_HIP]) >= VIS_OFF
    if not hips_ok and vis[NOSE] < VIS_OFF:
        return {}                      # no hips AND no head: nothing to build on
    fwd, left, up = body_frame(world, hips_ok)

    def decompose(v, side):
        """Limb direction -> (flexion, abduction, flex_ok) in the body frame.
        flex_ok is False near pure-lateral poses, where flexion is a gimbal
        singularity (the sagittal components are ~0 and noise decides the
        angle) — callers should hold the previous flexion there."""
        u = unit(v)
        f, l, uu = np.dot(u, fwd), np.dot(u, left), np.dot(u, up)
        flex = math.atan2(f, -uu)
        lat = l if side == "L" else -l        # positive = away from body
        abd = math.asin(max(-1.0, min(1.0, lat)))
        flex_ok = math.hypot(f, uu) > 0.25    # cos(abd): limb not ~fully lateral
        return flex, abd, flex_ok

    def bend(v1, v2):
        """3D angle between consecutive segments: 0 = straight."""
        return math.acos(max(-1.0, min(1.0, float(np.dot(unit(v1), unit(v2))))))

    out = {}
    for side, sh, el, wr, hip, kn, an in [
        ("L", L_SHOULDER, L_ELBOW, L_WRIST, L_HIP, L_KNEE, L_ANKLE),
        ("R", R_SHOULDER, R_ELBOW, R_WRIST, R_HIP, R_KNEE, R_ANKLE),
    ]:
        if gates.check(f"{side}_arm", min(vis[sh], vis[el])):
            flex, abd, flex_ok = decompose(world[el] - world[sh], side)
            if flex_ok:
                out[f"{side}_shoulder_flex"] = clamp("shoulder_flex", flex)
            out[f"{side}_shoulder_abd"] = clamp("shoulder_abd", abd)
            if gates.check(f"{side}_forearm", vis[wr]):
                ua, fa = world[el] - world[sh], world[wr] - world[el]
                out[f"{side}_elbow"] = clamp("elbow", bend(ua, fa))
                tw, tw_ok = twist_from_bend(ua, fa, fwd)
                if tw_ok:
                    # mirror the sign on the right: the arms are mirror images,
                    # so the same anatomical rotation is the opposite sense about
                    # a shoulder->elbow axis that points down both arms
                    out[f"{side}_shoulder_twist"] = clamp(
                        "shoulder_twist", tw if side == "L" else -tw)
        if hips_ok and gates.check(f"{side}_leg", min(vis[hip], vis[kn])):
            flex, abd, flex_ok = decompose(world[kn] - world[hip], side)
            if flex_ok:
                out[f"{side}_hip_flex"] = clamp("hip_flex", flex)
            out[f"{side}_hip_abd"] = clamp("hip_abd", abd)
            # ankle evidence: best of ankle/heel/foot — ankles are chronically
            # low-visibility (bottom of frame, occluded by the other leg), and
            # gating the knee behind a strict ankle test froze knees straight
            heel, foot = (L_HEEL, L_FOOT) if side == "L" else (R_HEEL, R_FOOT)
            ankle_ev = max(vis[an], vis[heel], vis[foot])
            if gates.check(f"{side}_shin", ankle_ev, on=0.5, off=0.35):
                out[f"{side}_knee"] = clamp(
                    "knee", bend(world[kn] - world[hip], world[an] - world[kn]))

    out.update(neck_angles(world, vis, gates, fwd, left, up))
    return out


# ---------------------------------------------------------------------------
# MediaPipe live stream plumbing
# ---------------------------------------------------------------------------
class PoseResults:
    """Holds the newest MediaPipe result for one camera, plus the numbers
    that say whether the camera or the model is the bottleneck."""

    def __init__(self):
        self.img = self.world = self.vis = None
        self.stamp = 0.0
        self.seq = 0               # +1 per result, person or not
        self.npose = 0             # results that contained a person
        self.busy = False          # a frame is in MediaPipe right now
        self.submitted = {}        # timestamp_ms -> wall time submitted
        self.lat_ms = 0.0          # submit -> result, smoothed

    def __getitem__(self, k):      # old code reads latest["world"] etc.
        return getattr(self, k)

    def callback(self, result, output_image, timestamp_ms):
        try:
            t_sub = self.submitted.pop(timestamp_ms, None)
            if t_sub is not None:
                ms = (time.time() - t_sub) * 1e3
                self.lat_ms = ms if not self.lat_ms else 0.8 * self.lat_ms + 0.2 * ms
            if result.pose_landmarks and result.pose_world_landmarks:
                img = result.pose_landmarks[0]
                wld = result.pose_world_landmarks[0]
                self.img = np.array([[lm.x, lm.y] for lm in img])
                self.world = np.array([[lm.x, lm.y, lm.z] for lm in wld])
                self.vis = np.array([lm.visibility for lm in img])
                self.stamp = time.time()
                self.npose += 1
            self.seq += 1
        except Exception:
            pass
        finally:
            self.busy = False

    def submit(self, landmarker, frame, ts_ms):
        """Hand a frame to MediaPipe ONLY if it is idle. Submitting while it
        is busy just makes it drop frames internally after we have paid to
        convert them; skipping here keeps latency to one frame."""
        if self.busy:
            return False
        self.busy = True
        self.submitted[ts_ms] = time.time()
        if len(self.submitted) > 50:
            self.submitted.clear()
        try:
            landmarker.detect_async(infer_image(frame), ts_ms)
        except Exception:
            self.busy = False
            raise
        return True


latest = PoseResults()
on_result = latest.callback


def make_landmarker(callback=None):
    """LIVE_STREAM PoseLandmarker. GPU if --gpu was given and it starts,
    otherwise CPU."""
    def build(delegate):
        bo = (mp_tasks.BaseOptions(model_asset_path=MODEL_PATH, delegate=delegate)
              if delegate is not None else
              mp_tasks.BaseOptions(model_asset_path=MODEL_PATH))
        options = vision.PoseLandmarkerOptions(
            base_options=bo,
            running_mode=vision.RunningMode.LIVE_STREAM,
            result_callback=callback or on_result,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        return vision.PoseLandmarker.create_from_options(options)
    if USE_GPU:
        try:
            lm = build(mp_tasks.BaseOptions.Delegate.GPU)
            print("MediaPipe: GPU delegate")
            return lm
        except Exception as e:
            print(f"MediaPipe: GPU delegate failed ({e}); using CPU")
    return build(None)


SEND_LANDMARKS = "--landmarks" in sys.argv   # v9+ receivers want raw landmarks
# Bone-length depth rebuild. ANGLE mode: on by default (--nodepthfix turns it
# off), as before. LANDMARK mode: OFF by default (--depthfix turns it on). The
# landmark path feeds the receiver's IK, and that IK was validated (3.56 deg on
# the reference clip) on RAW MediaPipe world landmarks - the offline driver
# never ran BoneDepth or BoneCalibrator. The live path was stacking BOTH depth
# rebuilders on top of each other before the IK, so it was not the pipeline
# that had been tested.
if SEND_LANDMARKS:
    DEPTH_FIX = "--depthfix" in sys.argv
else:
    DEPTH_FIX = "--nodepthfix" not in sys.argv


def _argval(flag, default=None):
    """--flag <value> off the command line, or default."""
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


# --realtime : when --source is a FILE, pace playback at the file's own fps.
#   Without this the loop decodes as fast as the disk allows and MediaPipe's
#   LIVE_STREAM graph silently drops every frame it is too busy for, so a file
#   test does NOT reproduce webcam timing. With it, the timing is the webcam's.
REALTIME = "--realtime" in sys.argv
# --noshow : no cv2 window (unattended runs).
NOSHOW = "--noshow" in sys.argv
# --log <path.npz> : per-iteration diagnostic record of what the LIVE path saw
#   and sent. Written on exit. Does not touch the wire format.
LOGPATH = _argval("--log")


# ORDER MATTERS - this is a kinematic chain. The shoulder line is solved first
# because both arms hang off it; solving it last moved the shoulder after the
# arm had already been built from its old position, and the upper arm's length
# error went UP (8.7% -> 15.2% cv) even though every other bone improved.
BoneDepth.LIMBS = {
    "shoulders": (R_SHOULDER, L_SHOULDER),
    "ua_L": (L_SHOULDER, L_ELBOW), "fa_L": (L_ELBOW, L_WRIST),
    "ua_R": (R_SHOULDER, R_ELBOW), "fa_R": (R_ELBOW, R_WRIST),
}


_wait_next = [0.0]


def now_wait_msg(t0):
    """True about every 2 s while waiting, so the wait is not silent."""
    now = time.time()
    if now - t0 > 1.0 and now >= _wait_next[0]:
        _wait_next[0] = now + 2.0
        return True
    return False


class RateMeter:
    """Counts events and reports events/s over a sliding ~2 s window."""

    def __init__(self):
        self.n, self.t, self.rate = 0, time.time(), 0.0

    def tick(self, k=1):
        self.n += k

    def update(self, now):
        el = now - self.t
        if el >= 1.0:
            self.rate = self.n / el
            self.n, self.t = 0, now
        return self.rate


def prepare_world(world, vis, now, bonedepth, arm_guard, leg_guard, bones):
    """The ONE preprocessing path for a frame's world landmarks, used for both
    the landmark packet and the angles. Depth rebuild only if DEPTH_FIX."""
    w = world.copy()
    if DEPTH_FIX:
        w = bonedepth.update(w)        # z from bone length
    arm_guard.fix(w, now)              # undo any L/R label swaps
    leg_guard.fix(w, now)
    if DEPTH_FIX:
        w = bones.process(w, vis)
    return w


def hips_in_frame(img, vis):
    """Visibility alone is not enough: on a chest-up shot MediaPipe places the
    hips BELOW the image and still reports ~0.68 confidence. A landmark
    outside the frame is not an observation."""
    inframe = all(0.0 <= img[k][1] <= 1.0 and 0.0 <= img[k][0] <= 1.0
                  for k in (L_HIP, R_HIP))
    return bool(min(vis[L_HIP], vis[R_HIP]) >= VIS_OFF and inframe)


def landmark_packet(world, hips_ok, i):
    """Raw world landmarks for the receiver's own IK. 4 dp keeps it inside one
    UDP datagram."""
    return json.dumps({"world": [[round(float(c), 4) for c in p3] for p3 in world],
                       "hips_ok": bool(hips_ok), "i": int(i)}).encode()


def draw_skeleton(frame, img, vis):
    h, w = frame.shape[:2]
    px = (img * np.array([w, h])).astype(int)
    for a, b in POSE_CONNECTIONS:
        seen = min(vis[a], vis[b]) > VIS_OFF
        cv2.line(frame, tuple(px[a]), tuple(px[b]),
                 (0, 255, 0) if seen else (120, 120, 120), 2)
    for i in range(11, 33):
        cv2.circle(frame, tuple(px[i]), 3,
                   (0, 0, 255) if vis[i] > VIS_OFF else (140, 140, 140), -1)


def main():
    ensure_model()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    src = CAMERA_INDEX
    is_file = False
    if "--source" in sys.argv:
        v = sys.argv[sys.argv.index("--source") + 1]
        if v.isdigit():
            src = int(v)
        else:
            src, is_file = v, True
    if "--camera" in sys.argv:
        src = int(sys.argv[sys.argv.index("--camera") + 1])

    # A FILE is read in order (optionally paced with --realtime) so tests are
    # repeatable; a CAMERA is read on its own thread, newest frame only.
    cam, cap = None, None
    if is_file:
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open {src}.")
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    else:
        cam = CameraStream(src, CAP_W, CAP_H, "cam")
        src_fps = 30.0
    log = [] if LOGPATH else None

    landmarker = make_landmarker()
    gates = Gates()
    filters = {n: OneEuro() for n in JOINT_NAMES}
    last_sent = {}
    arm_guard = SwapGuard(L_WRIST, R_WRIST, ARM_CHAIN_L, ARM_CHAIN_R)
    leg_guard = SwapGuard(L_ANKLE, R_ANKLE, LEG_CHAIN_L, LEG_CHAIN_R)
    bones = BoneCalibrator()
    bonedepth = BoneDepth()

    mode = "LANDMARKS (for mimic_receiver_v11)" if SEND_LANDMARKS else "ANGLES"
    print(f"3D mocap -> udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}   sending {mode}")
    print(f"model {MODEL_NAME}, {'GPU requested' if USE_GPU else 'CPU'}, "
          f"depth rebuild {'ON' if DEPTH_FIX else 'OFF'}"
          + ("" if is_file else f", camera {src} at {cam.size()[0]}x{cam.size()[1]}"))
    if not SEND_LANDMARKS:
        print("NOTE: the v11 robot needs --landmarks; angles only drive the "
              "older receivers.")
    print("Quit with q in the camera window or Ctrl+C.\n")

    t0 = time.time()
    last_ts = -1
    last_fseq = 0
    last_pose_stamp = 0.0
    src_i = 0
    m_cam, m_pose, m_sent, m_loop = RateMeter(), RateMeter(), RateMeter(), RateMeter()
    last_seq, last_cam_n = 0, 0
    next_print = time.time() + 2.0
    err_last, err_count = 0.0, 0
    sent = {}

    try:
        while True:
            # ---------------- 1. newest frame --------------------------------
            if is_file:
                ok, frame = cap.read()
                if not ok:
                    break
                src_i = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1
                if REALTIME:
                    slack = t0 + src_i / src_fps - time.time()
                    if slack > 0:
                        time.sleep(slack)
                m_cam.tick()
            else:
                frame, fseq, _ = cam.latest()
                st = cam.status()
                if st != "live" and now_wait_msg(t0):
                    print(f"camera {src}: {st}. Wrong index? Try --camera N "
                          "(see: python \"live mimic system/pose_sender_stereo.py\" "
                          "--list-cams)", flush=True)
                if frame is None or fseq == last_fseq:
                    time.sleep(0.002)
                    if not NOSHOW and cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue
                last_fseq = fseq
                src_i = fseq
                frame = frame.copy()
            m_loop.tick()
            now = time.time()

            # ---------------- 2. hand it to MediaPipe if it is idle ----------
            ts = int((now - t0) * 1000)
            if ts <= last_ts:
                ts = last_ts + 1
            try:
                if latest.submit(landmarker, frame, ts):
                    last_ts = ts
            except Exception as e:
                print(f"tracker error ({e}); rebuilding...")
                try:
                    landmarker.close()
                except Exception:
                    pass
                landmarker = make_landmarker()
                latest.busy = False
                continue
            if latest.seq != last_seq:
                m_pose.tick(latest.seq - last_seq)
                last_seq = latest.seq

            # ---------------- 3. a NEW person result -> process + send -------
            fresh = latest.world is not None and now - latest.stamp < 0.5
            world = None
            if fresh and latest.stamp != last_pose_stamp:
                last_pose_stamp = latest.stamp
                vis = latest.vis.copy()
                img = latest.img.copy()
                world = prepare_world(latest.world, vis, now, bonedepth,
                                      arm_guard, leg_guard, bones)
                if SEND_LANDMARKS and np.all(np.isfinite(world)):
                    # sent FIRST and on its own: an error in the angle code
                    # below used to skip the send for that frame entirely
                    sock.sendto(landmark_packet(world, hips_in_frame(img, vis),
                                                src_i), UDP_ADDR)
                    m_sent.tick()
                try:
                    raw = frame_angles(world.copy(), vis, gates)
                    sent = {}
                    for name, val in raw.items():
                        if name not in filters:
                            continue
                        filt = filters[name](val, now)
                        if abs(filt - last_sent.get(name, 99.0)) > DEADBAND:
                            last_sent[name] = filt
                        sent[name] = last_sent[name]
                    if not SEND_LANDMARKS and sent:
                        sock.sendto(json.dumps(sent).encode(), UDP_ADDR)
                        m_sent.tick()
                except Exception as e:
                    err_count += 1
                    if now - err_last > 2.0:
                        print(f"angle code error ({e}) x{err_count} - "
                              f"{'landmarks still sent' if SEND_LANDMARKS else 'frame skipped'}")
                        err_last = now

            # ---------------- 4. overlay -------------------------------------
            h, w = frame.shape[:2]
            if fresh:
                draw_skeleton(frame, latest.img, latest.vis)
                cv2.putText(frame, f"tracking {len(sent)} joints"
                            + (" + landmarks" if SEND_LANDMARKS else ""),
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if sent else (0, 165, 255), 2)
                if DEPTH_FIX and not bones.ready:
                    cv2.putText(frame,
                                f"CALIBRATING bones {bones.progress()*100:.0f}% "
                                "- face camera, move arms & legs around",
                                (20, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 200, 255), 2)
            else:
                cv2.putText(frame, "no person detected (still running)",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)
            if cam is not None:
                m_cam.tick(cam.nread - last_cam_n)
                last_cam_n = cam.nread
            rc, rp, rs = m_cam.update(now), m_pose.update(now), m_sent.update(now)
            m_loop.update(now)
            cv2.putText(frame, f"camera {rc:4.1f} fps | pose {rp:4.1f}/s | "
                        f"model {latest.lat_ms:4.0f} ms | sent {rs:4.1f}/s",
                        (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 0), 2)

            if log is not None:
                log.append({
                    "i": src_i, "t": now - t0,
                    "fresh": bool(fresh),
                    "stamp": (latest.stamp - t0) if fresh else float("nan"),
                    "px": latest.img.copy() if fresh else np.full((33, 2), np.nan),
                    "vis": latest.vis.copy() if fresh else np.full(33, np.nan),
                    "world_raw": (latest.world.copy() if fresh
                                  else np.full((33, 3), np.nan)),
                    "world": (world.copy() if world is not None
                              else np.full((33, 3), np.nan)),
                    "hips_ok": (hips_in_frame(latest.img, latest.vis)
                                if fresh else False),
                    "bones_ready": bool(bones.ready),
                    "swap_arm": int(arm_guard.last_swap),
                    "swap_leg": int(leg_guard.last_swap),
                    "nsent": len(sent),
                })

            # ---------------- 5. terminal status ------------------------------
            if now >= next_print:
                next_print = now + 2.0
                print(f"STATUS camera {rc:4.1f} fps | pose results {rp:4.1f}/s | "
                      f"model {latest.lat_ms:4.0f} ms | sent {rs:4.1f}/s | "
                      f"person {'YES' if fresh else 'no'}", flush=True)
                if rc and rc < 20:
                    print("   -> the CAMERA itself is slow. Webcams drop their "
                          "frame rate in dim light - add light, or try "
                          "--res 640x480", flush=True)
                if rc and rp < 0.7 * rc:
                    print("   -> the MODEL is the bottleneck. Try --model lite "
                          "and/or --gpu", flush=True)
                if fresh and rs == 0:
                    print("   -> person seen but nothing sent: check for "
                          "'angle code error' lines above", flush=True)

            if not NOSHOW:
                cv2.imshow("live pose 3D (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass

    if cam is not None:
        cam.release()
    if cap is not None:
        cap.release()
    try:
        landmarker.close()
    except Exception:
        pass
    if not NOSHOW:
        cv2.destroyAllWindows()
    if log:
        keys = ("i", "t", "stamp", "fresh", "hips_ok", "bones_ready",
                "swap_arm", "swap_leg", "nsent")
        blob = {k: np.array([r[k] for r in log]) for k in keys}
        for k in ("px", "vis", "world", "world_raw"):
            blob[k] = np.array([r[k] for r in log])
        os.makedirs(os.path.dirname(os.path.abspath(LOGPATH)), exist_ok=True)
        np.savez_compressed(LOGPATH, **blob)
        print(f"sender log -> {LOGPATH}  ({len(log)} iterations)")
    print("Stopped (by you).")


if __name__ == "__main__":
    main()
