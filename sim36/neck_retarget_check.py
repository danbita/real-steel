"""
neck_retarget_check.py - prove the neck retargeting math, with no camera,
no MuJoCo and no mediapipe installed.

    python sim36/neck_retarget_check.py

Builds synthetic skeletons whose head pose relative to the torso is known
exactly, pushes them through pose_sender's own head_frame() / neck_angles(), and
demands the numbers come back out.

mediapipe and cv2 are stubbed into sys.modules before the import. pose_sender
touches neither at module scope (only inside make_landmarker / main), so the
retargeting code under test is the real code, not a copy - which is the point.
A second copy of this math would drift from the sender within a week.

WHAT IT CHECKS
  1  ROUND TRIP. A commanded yaw/pitch/roll must come back out of neck_angles.
  2  CAMERA INVARIANCE. Rotate the entire scene - person and all - by an
     arbitrary rotation and the angles must not budge. This is the property the
     whole body-frame approach exists to buy, and the one a head-vs-camera
     measurement silently loses.
  3  THE NOSE-DROP BIAS IS REAL, AND NeckNeutral REMOVES IT. With a realistic
     nose sitting below the ear line, the raw measurement is pitched down by
     that angle. Shown failing without the neutral and passing with it.
  4  EYE FALLBACK. Occlude one ear and a frame must still come out, with roll
     withheld rather than guessed.
  5  CLAMPS. Command past the servo limit and the output must sit on the limit.
  6  GATING. No face, no output - and no exception.
"""

import os
import sys
import math
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def stub_mediapipe():
    """Minimal stand-ins so `import pose_sender` works without the real wheels."""
    for name in ("cv2", "mediapipe", "mediapipe.tasks", "mediapipe.tasks.python",
                 "mediapipe.tasks.python.vision"):
        if name in sys.modules:
            continue
        m = types.ModuleType(name)
        m.__path__ = []
        sys.modules[name] = m
    sys.modules["mediapipe"].tasks = sys.modules["mediapipe.tasks"]
    sys.modules["mediapipe.tasks"].python = sys.modules["mediapipe.tasks.python"]
    sys.modules["mediapipe.tasks.python"].vision = sys.modules["mediapipe.tasks.python.vision"]


stub_mediapipe()
sys.path.insert(0, os.path.join(ROOT, "live mimic system"))
import pose_sender as ps          # noqa: E402

fails = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        fails.append(name)


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def skeleton(yaw, pitch, roll, world_rot=None, nose_drop_deg=0.0,
             hide=(), nose_reach=0.10):
    """A 33-landmark synthetic person whose head sits at exactly (yaw, pitch,
    roll) relative to their own torso.

    nose_drop_deg tilts the nose below the ear line, which is what a real face
    does and what NeckNeutral exists to cancel.
    """
    W = np.eye(3) if world_rot is None else world_rot
    # body triad. MediaPipe world has +y DOWN, so the spine is -y.
    up = W @ np.array([0.0, -1.0, 0.0])
    left = W @ np.array([1.0, 0.0, 0.0])
    fwd = np.cross(left, up)

    B = np.column_stack([fwd, left, up])          # body->world
    H = B @ (Rz(yaw) @ Ry(pitch) @ Rx(roll))      # head->world
    h_fwd, h_left, h_up = H[:, 0], H[:, 1], H[:, 2]

    w = np.zeros((33, 3))
    vis = np.ones(33)
    mid_hip = np.zeros(3)
    w[ps.L_HIP] = mid_hip + 0.090 * left
    w[ps.R_HIP] = mid_hip - 0.090 * left
    mid_sh = mid_hip + 0.300 * up
    w[ps.L_SHOULDER] = mid_sh + 0.120 * left
    w[ps.R_SHOULDER] = mid_sh - 0.120 * left
    # limbs, so a full frame_angles() call has something to chew on
    for side, sgn in (("L", 1.0), ("R", -1.0)):
        sh = getattr(ps, f"{side}_SHOULDER")
        w[getattr(ps, f"{side}_ELBOW")] = w[sh] - 0.14 * up + sgn * 0.03 * left
        w[getattr(ps, f"{side}_WRIST")] = w[sh] - 0.28 * up + sgn * 0.05 * left
        hp = getattr(ps, f"{side}_HIP")
        w[getattr(ps, f"{side}_KNEE")] = w[hp] - 0.20 * up
        w[getattr(ps, f"{side}_ANKLE")] = w[hp] - 0.40 * up
        w[getattr(ps, f"{side}_HEEL")] = w[hp] - 0.41 * up
        w[getattr(ps, f"{side}_FOOT")] = w[hp] - 0.41 * up + 0.04 * fwd

    ear_mid = mid_sh + 0.120 * up
    w[ps.L_EAR] = ear_mid + 0.075 * h_left
    w[ps.R_EAR] = ear_mid - 0.075 * h_left
    drop = nose_reach * math.tan(math.radians(nose_drop_deg))
    w[ps.NOSE] = ear_mid + nose_reach * h_fwd - drop * h_up
    w[ps.L_EYE] = ear_mid + 0.030 * h_left + 0.070 * h_fwd - 0.2 * drop * h_up
    w[ps.R_EYE] = ear_mid - 0.030 * h_left + 0.070 * h_fwd - 0.2 * drop * h_up

    for i in hide:
        vis[i] = 0.0
    return w, vis


def measure(yaw, pitch, roll, **kw):
    """Run one pose through the sender, with a neutral trained on the SAME
    face geometry at zero head rotation (what the real startup does)."""
    neutral = ps.NeckNeutral()
    neutral.N = 3
    zw, zv = skeleton(0.0, 0.0, 0.0, **{k: v for k, v in kw.items() if k != "hide"})
    gates = ps.Gates()
    for _ in range(6):
        f, l, u = ps.body_frame(zw, True)
        ps.neck_angles(zw, zv, gates, f, l, u, neutral=neutral)
    w, v = skeleton(yaw, pitch, roll, **kw)
    f, l, u = ps.body_frame(w, True)
    for _ in range(3):
        out = ps.neck_angles(w, v, gates, f, l, u, neutral=neutral)
    return out


def main():
    print("\nusing pose_sender from:", ps.__file__)
    print("SEND_NECK_ROLL =", ps.SEND_NECK_ROLL, " (roll is measured, not sent)\n")
    # roll has to be emitted for these checks to see it
    ps.SEND_NECK_ROLL = True
    ps.NeckNeutral.verbose = False       # the real sender announces it once

    # 1 ------------------------------------------------------------- round trip
    print("1. round trip: commanded angle comes back out")
    worst = 0.0
    for y in (-60, -25, 0, 25, 60):
        for p in (-40, -15, 0, 15, 40):
            for r in (-20, 0, 20):
                want = dict(neck_yaw=y, neck_pitch=p, neck_roll=r)
                got = measure(math.radians(y), math.radians(p), math.radians(r))
                for k, v in want.items():
                    worst = max(worst, abs(math.degrees(got[k]) - v))
    check("75 poses recovered", worst < 1e-6, f"worst error {worst:.3e} deg")

    # 2 ------------------------------------------------- camera invariance
    print("\n2. camera invariance: rotate the whole scene, angles must not move")
    ref = measure(math.radians(30), math.radians(-20), math.radians(12))
    spread = 0.0
    rng = np.random.default_rng(7)
    for _ in range(12):
        A = rng.normal(size=(3, 3))
        Q, _ = np.linalg.qr(A)
        if np.linalg.det(Q) < 0:
            Q[:, 0] *= -1
        got = measure(math.radians(30), math.radians(-20), math.radians(12), world_rot=Q)
        for k in ref:
            spread = max(spread, abs(math.degrees(got[k] - ref[k])))
    check("12 arbitrary scene rotations agree", spread < 1e-6,
          f"worst disagreement {spread:.3e} deg")

    # 3 ------------------------------------------- nose drop / NeckNeutral
    print("\n3. the nose-drop bias, and that NeckNeutral removes it")
    DROP = 18.0
    gates = ps.Gates()
    raw = ps.NeckNeutral()
    raw.N = 1
    w0, v0 = skeleton(0, 0, 0, nose_drop_deg=DROP)
    f, l, u = ps.body_frame(w0, True)
    raw.R0 = np.eye(3)                       # pretend no neutral was learned
    bare = ps.neck_angles(w0, v0, gates, f, l, u, neutral=raw)
    bias = math.degrees(bare["neck_pitch"])
    check("raw measurement IS biased by the nose drop", abs(bias - DROP) < 0.5,
          f"pitch reads {bias:+.2f} deg at rest, nose drop is {DROP:.0f} deg")
    fixed = measure(0.0, 0.0, 0.0, nose_drop_deg=DROP)
    check("neutral removes it", abs(math.degrees(fixed["neck_pitch"])) < 1e-6,
          f"pitch at rest {math.degrees(fixed['neck_pitch']):+.2e} deg")
    nod = measure(0.0, math.radians(25), 0.0, nose_drop_deg=DROP)
    check("and a real nod still reads true through the offset",
          abs(math.degrees(nod["neck_pitch"]) - 25) < 1e-6,
          f"{math.degrees(nod['neck_pitch']):.6f} deg")

    # 4 ------------------------------------------------------- eye fallback
    print("\n4. one ear occluded -> eyes carry it, roll withheld")
    got = measure(math.radians(35), math.radians(-10), 0.0, hide=(ps.R_EAR,))
    check("still produces yaw and pitch", "neck_yaw" in got and "neck_pitch" in got,
          f"keys {sorted(got)}")
    check("roll withheld on the eye baseline", "neck_roll" not in got)

    # 5 ------------------------------------------------------------- clamps
    print("\n5. clamps at the servo limit")
    lim = math.degrees(ps.LIMITS["neck_yaw"][1])
    got = measure(math.radians(100), 0.0, 0.0)
    check(f"yaw commanded 100 deg clamps to {lim:.1f}",
          abs(math.degrees(got["neck_yaw"]) - lim) < 1e-6,
          f"{math.degrees(got['neck_yaw']):.3f} deg")

    # 6 ------------------------------------------------------------- gating
    print("\n6. gating when the face is gone")
    neutral = ps.NeckNeutral()
    neutral.N = 1
    gates = ps.Gates()
    w, v = skeleton(0, 0, 0, hide=(ps.NOSE, ps.L_EAR, ps.R_EAR, ps.L_EYE, ps.R_EYE))
    f, l, u = ps.body_frame(w, True)
    check("no face -> empty, no exception",
          ps.neck_angles(w, v, gates, f, l, u, neutral=neutral) == {})

    # integration: the neck rides along in a full frame_angles() call
    print("\n7. integration through frame_angles()")
    ps.NECK_NEUTRAL = ps.NeckNeutral()
    ps.NECK_NEUTRAL.N = 2
    gates = ps.Gates()
    w, v = skeleton(math.radians(20), math.radians(10), 0.0)
    out = {}
    for _ in range(8):
        out = ps.frame_angles(w, v, gates)
    check("frame_angles emits neck joints alongside the limbs",
          "neck_yaw" in out and "neck_pitch" in out and "L_elbow" in out,
          f"{len(out)} joints")
    check("every emitted name is in JOINT_NAMES (main's filter dict)",
          all(k in ps.JOINT_NAMES for k in out),
          f"unknown: {[k for k in out if k not in ps.JOINT_NAMES]}")

    print()
    if fails:
        print(f"{len(fails)} CHECK(S) FAILED: {fails}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
