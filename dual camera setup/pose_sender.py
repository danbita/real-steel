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
os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"

import json
import math
import socket
import time
import urllib.request

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

here = os.path.dirname(os.path.abspath(__file__))

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
             "pose_landmarker_full/float16/latest/pose_landmarker_full.task")
MODEL_PATH = os.path.join(here, "pose_landmarker_full.task")

CAMERA_INDEX = 0
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
SIGN = {"shoulder_flex": 1.0, "shoulder_abd": 1.0, "elbow": 1.0,
        "hip_flex": 1.0, "hip_abd": 1.0, "knee": 1.0}

LIMITS = {
    "shoulder_flex": (-1.2, 3.1), "shoulder_abd": (-0.5, 3.0),
    "elbow": (0.0, 2.7),
    "hip_flex": (-1.0, 2.1), "hip_abd": (-0.4, 1.2),
    "knee": (0.0, 2.4),
}

NOSE = 0
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24
L_KNEE, R_KNEE = 25, 26
L_ANKLE, R_ANKLE = 27, 28
L_HEEL, R_HEEL = 29, 30
L_FOOT, R_FOOT = 31, 32

ARM_CHAIN_L, ARM_CHAIN_R = [13, 15, 17, 19, 21], [14, 16, 18, 20, 22]
LEG_CHAIN_L, LEG_CHAIN_R = [25, 27, 29, 31], [26, 28, 30, 32]

POSE_CONNECTIONS = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (27, 29), (29, 31), (27, 31),
    (24, 26), (26, 28), (28, 30), (30, 32), (28, 32),
]

JOINT_NAMES = ["L_shoulder_flex", "L_shoulder_abd", "L_elbow",
               "R_shoulder_flex", "R_shoulder_abd", "R_elbow",
               "L_hip_flex", "L_hip_abd", "L_knee",
               "R_hip_flex", "R_hip_abd", "R_knee"]


def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("Downloading full pose model...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


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

    def fix(self, world, t):
        L, R = world[self.pl].copy(), world[self.pr].copy()
        if self.prev_l is not None and t - self.last_t < 0.3:
            keep = np.linalg.norm(L - self.prev_l) + np.linalg.norm(R - self.prev_r)
            swap = np.linalg.norm(L - self.prev_r) + np.linalg.norm(R - self.prev_l)
            if swap + SWAP_MARGIN < keep:
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


def clamp(fam, val):
    lo, hi = LIMITS[fam]
    return max(lo, min(hi, SIGN[fam] * val))


def body_frame(world):
    left = unit(world[L_HIP] - world[R_HIP])
    mid_hip = 0.5 * (world[L_HIP] + world[R_HIP])
    mid_sh = 0.5 * (world[L_SHOULDER] + world[R_SHOULDER])
    up = unit(mid_sh - mid_hip)
    fwd = unit(np.cross(left, up))
    left = np.cross(up, fwd)          # re-orthogonalize
    return fwd, left, up


def frame_angles(world, vis, gates):
    if min(vis[L_SHOULDER], vis[R_SHOULDER], vis[L_HIP], vis[R_HIP]) < VIS_OFF:
        return {}
    fwd, left, up = body_frame(world)

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
                out[f"{side}_elbow"] = clamp(
                    "elbow", bend(world[el] - world[sh], world[wr] - world[el]))
        if gates.check(f"{side}_leg", min(vis[hip], vis[kn])):
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
    return out


# ---------------------------------------------------------------------------
# MediaPipe live stream plumbing
# ---------------------------------------------------------------------------
latest = {"img": None, "world": None, "vis": None, "stamp": 0.0}


def on_result(result, output_image, timestamp_ms):
    try:
        if result.pose_landmarks and result.pose_world_landmarks:
            img = result.pose_landmarks[0]
            wld = result.pose_world_landmarks[0]
            latest["img"] = np.array([[lm.x, lm.y] for lm in img])
            latest["world"] = np.array([[lm.x, lm.y, lm.z] for lm in wld])
            latest["vis"] = np.array([lm.visibility for lm in img])
            latest["stamp"] = time.time()
    except Exception:
        pass


def make_landmarker():
    options = vision.PoseLandmarkerOptions(
        base_options=mp_tasks.BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=vision.RunningMode.LIVE_STREAM,
        result_callback=on_result,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    return vision.PoseLandmarker.create_from_options(options)


def main():
    ensure_model()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam.")

    landmarker = make_landmarker()
    gates = Gates()
    filters = {n: OneEuro() for n in JOINT_NAMES}
    last_sent = {}
    arm_guard = SwapGuard(L_WRIST, R_WRIST, ARM_CHAIN_L, ARM_CHAIN_R)
    leg_guard = SwapGuard(L_ANKLE, R_ANKLE, LEG_CHAIN_L, LEG_CHAIN_R)
    bones = BoneCalibrator()

    print(f"3D mocap streaming to udp://{UDP_ADDR[0]}:{UDP_ADDR[1]} (12 joints)")
    print("Any camera angle works now. Partial body is fine.")
    print("Quit with q in the camera window or Ctrl+C.\n")

    t0 = time.time()
    n, fps, fps_clock = 0, 0.0, time.time()

    try:
        while True:
            try:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.1)
                    continue
                h, w = frame.shape[:2]

                try:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    landmarker.detect_async(mp_image, int((time.time() - t0) * 1000))
                except Exception as e:
                    print(f"tracker error ({e}); rebuilding...")
                    try:
                        landmarker.close()
                    except Exception:
                        pass
                    landmarker = make_landmarker()
                    t0 = time.time()
                    continue

                now = time.time()
                fresh = latest["world"] is not None and now - latest["stamp"] < 0.5

                sent = {}
                if fresh:
                    world = latest["world"].copy()
                    vis = latest["vis"]

                    arm_guard.fix(world, now)      # undo any L/R label swaps
                    leg_guard.fix(world, now)
                    world = bones.process(world, vis)   # bone-length depth fix

                    raw = frame_angles(world, vis, gates)
                    for name, val in raw.items():
                        filt = filters[name](val, now)
                        if abs(filt - last_sent.get(name, 99.0)) > DEADBAND:
                            last_sent[name] = filt
                        sent[name] = last_sent[name]
                    if sent:
                        sock.sendto(json.dumps(sent).encode(), UDP_ADDR)

                    # overlay (2D landmarks): green tracked, gray gated
                    px = (latest["img"] * np.array([w, h])).astype(int)
                    for a, b in POSE_CONNECTIONS:
                        seen = min(vis[a], vis[b]) > VIS_OFF
                        color = (0, 255, 0) if seen else (120, 120, 120)
                        cv2.line(frame, tuple(px[a]), tuple(px[b]), color, 2)
                    for i in range(11, 33):
                        c = (0, 0, 255) if vis[i] > VIS_OFF else (140, 140, 140)
                        cv2.circle(frame, tuple(px[i]), 3, c, -1)
                    cv2.putText(frame, f"tracking {len(sent)}/12 joints (3D)",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0) if sent else (0, 165, 255), 2)
                    if not bones.ready:
                        cv2.putText(frame,
                                    f"CALIBRATING bones {bones.progress()*100:.0f}% "
                                    "- face camera, move arms & legs around",
                                    (20, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 200, 255), 2)
                    # live knee readout: if these numbers move when you bend
                    # but the sim doesn't, the problem is downstream; if they
                    # stay near 0, the measurement itself is failing
                    lk = math.degrees(sent.get("L_knee", 0.0))
                    rk = math.degrees(sent.get("R_knee", 0.0))
                    cv2.putText(frame,
                                f"knee L {lk:5.1f}  R {rk:5.1f}   "
                                f"[{'L sent' if 'L_knee' in sent else 'L gated'} / "
                                f"{'R sent' if 'R_knee' in sent else 'R gated'}]",
                                (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 0), 2)
                else:
                    cv2.putText(frame, "no person detected (still running)",
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 0, 255), 2)

                n += 1
                if n % 10 == 0:
                    fps = 10.0 / (time.time() - fps_clock)
                    fps_clock = time.time()
                cv2.putText(frame, f"{fps:.0f} fps", (20, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                cv2.imshow("live pose 3D (q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            except Exception as e:
                print(f"recovered from error: {e}")
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    cap.release()
    cv2.destroyAllWindows()
    print("Stopped (by you).")


if __name__ == "__main__":
    main()
