"""
crop_tracker_test.py - scripted scenes for PersonTracker (pose_sender.py).

    python tests/crop_tracker_test.py          # all scenes
    python tests/crop_tracker_test.py -v       # also print the timeline around failures

No camera, no MediaPipe: cv2 and mediapipe are replaced by stubs before
pose_sender is loaded, so this runs on any Python with numpy. What is tested is
the tracker's own logic - which crop it hands MediaPipe, which results it
accepts, when it re-searches - driven through the REAL submit() and
_on_result() on a simulated 30 fps clock.

A fake MediaPipe stands in for the landmarker. Each frame it sees only the crop
the tracker submitted and returns ONE body from it, modelled on how
num_poses=1 behaves:
  * people are 2D bodies in a 1280x720 frame with a depth order (z: lower is
    nearer the camera); a nearer body hides the landmarks of anyone behind it
    (visibility 0.1 inside its silhouette box)
  * it keeps returning the body it returned last frame (its own tracking)
    while that body is in the crop with both shoulders visible; otherwise it
    re-detects: the nearest body in the crop with visible shoulders, else the
    last one again (low visibility), else nothing
  * a scene can override that per frame ("mp": id or None)
Partial covers by an OBJECT (not a body MediaPipe could detect) are given as
per-landmark visibility overrides.

Scenes marked KNOWN GAP describe behaviour the current design does not have.
They are reported, but do not fail the run; if one starts passing, it says so.
"""
import importlib.util as il
import os
import sys
import types
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W, H = 1280, 720
FPS = 30.0
DT = 1.0 / FPS
VERBOSE = "-v" in sys.argv
ANY = "*"


# ---------------------------------------------------------------------------
# stubs: cv2 / mediapipe, and a simulated clock
# ---------------------------------------------------------------------------
class FakeLandmarker:
    def __init__(self, options):
        self.cb = options.result_callback

    def detect_async(self, image, ts):
        SIM.detect(self.cb, ts)

    def close(self):
        pass


def install_stubs():
    cv2 = types.ModuleType("cv2")
    cv2.COLOR_BGR2RGB = 4
    cv2.cvtColor = lambda img, code: img
    mp = types.ModuleType("mediapipe")
    mp.Image = lambda image_format, data: SimpleNamespace(data=data)
    mp.ImageFormat = SimpleNamespace(SRGB=1)
    tasks = types.ModuleType("mediapipe.tasks")
    tpy = types.ModuleType("mediapipe.tasks.python")
    vision = types.ModuleType("mediapipe.tasks.python.vision")
    tpy.BaseOptions = lambda **k: SimpleNamespace(**k)
    vision.PoseLandmarkerOptions = lambda **k: SimpleNamespace(**k)
    vision.RunningMode = SimpleNamespace(LIVE_STREAM=2)
    vision.PoseLandmarker = SimpleNamespace(create_from_options=FakeLandmarker)
    tpy.vision, tasks.python, mp.tasks = vision, tpy, tasks
    sys.modules.update({"cv2": cv2, "mediapipe": mp, "mediapipe.tasks": tasks,
                        "mediapipe.tasks.python": tpy,
                        "mediapipe.tasks.python.vision": vision})


install_stubs()
spec = il.spec_from_file_location(
    "pose_sender", os.path.join(ROOT, "live mimic system", "pose_sender.py"))
ps = il.module_from_spec(spec)
spec.loader.exec_module(ps)

CLOCK = [1000.0]
ps.time = SimpleNamespace(time=lambda: CLOCK[0], sleep=lambda s: None)
FRAME = np.zeros((H, W, 3), np.uint8)


# ---------------------------------------------------------------------------
# synthetic people (full-frame pixels)
# ---------------------------------------------------------------------------
CODE = {"A": 1, "B": 2, "C": 3}
NAME = {v: k for k, v in CODE.items()}


def body(cx, cy=300.0, s=100.0, z=1.0, hide=()):
    """Torso centre (cx, cy), shoulder width s px, depth z. hide: landmark ids
    covered by an object (visibility 0.1)."""
    img = np.tile([cx, cy], (33, 1)).astype(float)
    img[ps.NOSE] = [cx, cy - 1.0 * s]
    img[ps.L_SHOULDER] = [cx + s / 2, cy - 0.6 * s]
    img[ps.R_SHOULDER] = [cx - s / 2, cy - 0.6 * s]
    img[ps.L_ELBOW] = [cx + 0.7 * s, cy]
    img[ps.R_ELBOW] = [cx - 0.7 * s, cy]
    img[ps.L_WRIST] = [cx + 0.75 * s, cy + 0.5 * s]
    img[ps.R_WRIST] = [cx - 0.75 * s, cy + 0.5 * s]
    img[ps.L_HIP] = [cx + 0.35 * s, cy + 0.6 * s]
    img[ps.R_HIP] = [cx - 0.35 * s, cy + 0.6 * s]
    img[ps.L_KNEE] = [cx + 0.35 * s, cy + 1.6 * s]
    img[ps.R_KNEE] = [cx - 0.35 * s, cy + 1.6 * s]
    img[ps.L_ANKLE] = [cx + 0.35 * s, cy + 2.6 * s]
    img[ps.R_ANKLE] = [cx - 0.35 * s, cy + 2.6 * s]
    vis = np.ones(33)
    vis[list(hide)] = 0.1
    return {"img": img, "vis": vis, "c": np.array([cx, cy]), "s": s, "z": z}


def silhouette(b):
    (cx, cy), s = b["c"], b["s"]
    return cx - 0.9 * s, cy - 1.2 * s, cx + 0.9 * s, cy + 2.8 * s


SHOULDERS = [ps.L_SHOULDER, ps.R_SHOULDER]
ARM_L = [ps.L_ELBOW, ps.L_WRIST]
AUTO = "auto"


class Sim:
    def __init__(self):
        self.people, self.force, self.prev = {}, AUTO, None
        self.calls = 0

    def visible(self):
        """visibility after occlusion by nearer bodies"""
        out = {}
        for pid, b in self.people.items():
            v = b["vis"].copy()
            for qid, q in self.people.items():
                if q["z"] < b["z"]:
                    x0, y0, x1, y1 = silhouette(q)
                    inside = ((b["img"][:, 0] >= x0) & (b["img"][:, 0] <= x1)
                              & (b["img"][:, 1] >= y0) & (b["img"][:, 1] <= y1))
                    v[inside] = np.minimum(v[inside], 0.1)
            out[pid] = v
        return out

    def detect(self, cb, ts):
        self.calls += 1
        trk = cb.__self__
        (x0, y0, cw, ch), _, _ = trk._pending[ts]
        vis = self.visible()

        def in_crop(pid):
            cx, cy = self.people[pid]["c"]
            return x0 <= cx <= x0 + cw and y0 <= cy <= y0 + ch

        def shoulders_ok(pid):
            return vis[pid][SHOULDERS].min() >= 0.5

        cands = [p for p in self.people if in_crop(p)]
        if self.force is None:
            pick = None
        elif self.force != AUTO:
            pick = self.force if self.force in self.people else None
        else:
            ok = [p for p in cands if shoulders_ok(p)]
            if self.prev in ok:
                pick = self.prev
            elif ok:
                pick = min(ok, key=lambda p: self.people[p]["z"])
            elif self.prev in cands:
                pick = self.prev
            elif cands:
                pick = min(cands, key=lambda p: self.people[p]["z"])
            else:
                pick = None
        self.prev = pick
        if pick is None:
            res = SimpleNamespace(pose_landmarks=[], pose_world_landmarks=[])
        else:
            b = self.people[pick]
            cimg = (b["img"] - [x0, y0]) / [cw, ch]
            lms = [SimpleNamespace(x=float(x), y=float(y), z=0.0, visibility=float(v))
                   for (x, y), v in zip(cimg, vis[pick])]
            wl = [SimpleNamespace(x=0.0, y=0.0, z=0.0) for _ in range(33)]
            wl[0] = SimpleNamespace(x=float(CODE[pick]), y=0.0, z=0.0)   # identity tag
            res = SimpleNamespace(pose_landmarks=[lms], pose_world_landmarks=[wl])
        cb(res, None, ts)


SIM = Sim()


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------
def frame(people, mp=AUTO, r=False):
    return {"people": people, "mp": mp, "r": r}


def sec(s):
    return int(round(s * FPS))


def lerp(a, b, k, n):
    return a + (b - a) * (k / max(n - 1, 1))


def run(frames):
    """Returns (followed id per frame or None, crop per frame, tracker)."""
    global SIM
    SIM = Sim()
    CLOCK[0] = 1000.0
    trk = ps.PersonTracker()
    out, crops = [], []
    for k, f in enumerate(frames):
        CLOCK[0] = 1000.0 + k * DT
        SIM.people, SIM.force = f["people"], f["mp"]
        if f["r"]:
            trk.reset()
        trk.submit(FRAME)
        lat = trk.latest()
        if lat is not None and lat["stamp"] == CLOCK[0]:
            out.append(NAME[int(round(lat["world"][0, 0]))])
        else:
            out.append(None)
        crops.append(trk.crop if trk.mode == "track" else None)
    trk.close()
    return out, crops, trk


def contains(crop, pt):
    if crop is None:
        return True                       # full frame
    x0, y0, cw, ch = crop
    return x0 <= pt[0] <= x0 + cw and y0 <= pt[1] <= y0 + ch


def n_moves(crops):
    return sum(1 for a, b in zip(crops, crops[1:])
               if a is not None and b is not None and a != b)


# ---------------------------------------------------------------------------
# scenes: return (frames, expected, check). expected[k]: id, None, ANY or a
# tuple of allowed values. check(out, crops, trk, frames) -> error str or None.
# ---------------------------------------------------------------------------
def scene_single_acquires():
    n = sec(2)
    return ([frame({"A": body(640)})] * n, ["A"] * n,
            lambda o, c, t, f: None if t.generation == 1 else f"generation {t.generation}")


def scene_bystander_outside_crop_never_seen():
    """A alone, then B stands far to the side for 3 s: the crop excludes B,
    MediaPipe never gets the chance to switch."""
    fr, ex = [], []
    for k in range(sec(3.5)):
        p = {"A": body(640)}
        if k >= sec(0.5):
            p["B"] = body(150, s=110, z=0.5)          # nearer and bigger, even
        fr.append(frame(p))
        ex.append("A")

    def check(o, crops, t, f):
        bad = [k for k, (c, fm) in enumerate(zip(crops, f))
               if "B" in fm["people"] and contains(c, fm["people"]["B"]["c"])]
        return f"crop contained B on {len(bad)} frames" if bad else None
    return fr, ex, check


def scene_arm_covered_by_object():
    """Something hides A's left arm for 0.5 s. MediaPipe still tracks A, so
    every frame is still accepted (the arm joints are gated downstream)."""
    fr = []
    for k in range(sec(2.5)):
        hide = ARM_L if sec(1.0) <= k < sec(1.5) else ()
        fr.append(frame({"A": body(640, hide=hide)}))
    return fr, ["A"] * len(fr), lambda o, c, t, f: (
        None if t.generation == 1 else f"re-acquired ({t.generation})")


def scene_shoulders_covered_briefly():
    """An object hides both shoulders for 1 s: those frames are dropped (no
    torso to identify), then A resumes with NO re-acquisition, same crop."""
    fr, ex = [], []
    for k in range(sec(3)):
        cov = sec(1.0) <= k < sec(2.0)
        fr.append(frame({"A": body(640, hide=SHOULDERS if cov else ())}))
        ex.append(None if cov else "A")

    def check(o, crops, t, f):
        if t.generation != 1:
            return f"re-acquired ({t.generation}) - recalibration triggered"
        if crops[sec(0.9)] != crops[-1]:
            return "crop moved"
        return None
    return fr, ex, check


def scene_shoulders_covered_long_object():
    """Shoulders hidden 3 s (> LOST_S): falls back to full-frame search, then
    re-acquires A when they reappear - as a NEW performer (recalibrates)."""
    fr, ex = [], []
    for k in range(sec(5)):
        cov = sec(1.0) <= k < sec(4.0)
        fr.append(frame({"A": body(640, hide=SHOULDERS if cov else ())}))
        ex.append(None if cov else "A")
    return fr, ex, lambda o, c, t, f: (
        None if t.generation == 2 else f"generation {t.generation}, expected 2")


def scene_shoulder_covered_by_near_walker():
    """A much nearer walker (2x A's size) crosses in front of A. MediaPipe
    re-detects onto the walker; the size check rejects it until A is back."""
    fr, ex = [], []
    n = sec(3.0)
    for k in range(n):
        p = {"A": body(640), "B": body(lerp(1400, -200, k, n), cy=330, s=200, z=0.3)}
        fr.append(frame(p))
        ex.append(("A", None))
    fr = [frame({"A": body(640)})] * sec(0.5) + fr + [frame({"A": body(640)})] * sec(0.5)
    ex = ["A"] * sec(0.5) + ex + ["A"] * sec(0.5)
    return fr, ex, None


def scene_walker_same_depth_passes_in_front():
    """KNOWN GAP. A walker at about A's depth (1.2x) crosses directly in front.
    MediaPipe re-detects onto them; once their torso is within JUMP_GATE of
    A's last position and their size within JUMP_SCALE, they pass the jump
    check, and the tracker follows them away."""
    fr, ex = [], []
    n = sec(3.0)
    for k in range(n):
        p = {"A": body(640), "B": body(lerp(1400, -200, k, n), s=120, z=0.5)}
        fr.append(frame(p))
        ex.append(("A", None))
    fr = [frame({"A": body(640)})] * sec(0.5) + fr + [frame({"A": body(640)})] * sec(1.0)
    ex = ["A"] * sec(0.5) + ex + ["A"] * sec(1.0)
    return fr, ex, None


def scene_walker_stops_in_front():
    """KNOWN GAP. A walker steps in front of A and stays 3 s. When the
    walker hides A's shoulders long enough, the tracker either takes the
    walker (similar size) or, after LOST_S, re-searches the full frame where
    the walker is the most visible person."""
    fr, ex = [], []
    for k in range(sec(1.0)):
        fr.append(frame({"A": body(640)}))
        ex.append("A")
    for k in range(sec(3.0)):
        fr.append(frame({"A": body(640), "B": body(660, s=125, z=0.5)}))
        ex.append(("A", None))
    for k in range(sec(1.0)):
        fr.append(frame({"A": body(640), "B": body(lerp(660, 1300, k, sec(1.0)), s=125, z=0.5)}))
        ex.append(("A", None))
    for k in range(sec(1.0)):
        fr.append(frame({"A": body(640)}))
        ex.append(ANY if k < 3 else "A")
    return fr, ex, None


def scene_redetect_onto_walker_far_rejected():
    """MediaPipe jumps onto someone 2+ torso widths away inside the crop:
    rejected, crop held, A resumes when MediaPipe returns to them."""
    fr, ex = [], []
    for k in range(sec(3)):
        p = {"A": body(640), "B": body(860, z=0.5)}
        jump = sec(1.0) <= k < sec(1.5)
        fr.append(frame(p, mp="B" if jump else "A"))
        ex.append(None if jump else "A")

    def check(o, crops, t, f):
        if t.n_reject < sec(0.5) - 1:
            return f"only {t.n_reject} rejections"
        return None if crops[sec(0.9)] == crops[sec(1.6)] else "crop moved during rejection"
    return fr, ex, check


def scene_performer_walks():
    """A walks 400 -> 900 px over 4 s: followed throughout, crop always holds
    them, and the crop moves in a few steps, not every frame."""
    n = sec(4)
    fr = [frame({"A": body(lerp(400, 900, k, n))}) for k in range(n)]

    def check(o, crops, t, f):
        out = [k for k, (c, fm) in enumerate(zip(crops, f))
               if not contains(c, fm["people"]["A"]["c"])]
        if out:
            return f"A outside crop on {len(out)} frames"
        m = n_moves(crops)
        return None if 1 <= m <= 8 else f"crop moved {m} times"
    return fr, ["A"] * n, check


def scene_performer_approaches():
    """A walks toward the camera (size 100 -> 170 px): followed, crop grows."""
    n = sec(3)
    fr = [frame({"A": body(640, s=lerp(100, 170, k, n))}) for k in range(n)]

    def check(o, crops, t, f):
        return None if crops[-1][2] > crops[0][2] else "crop did not grow"
    return fr, ["A"] * n, check


def scene_fast_punch_reaches_edge():
    """A's arm shoots out to the side: the crop re-fits around it."""
    fr = []
    for k in range(sec(2)):
        b = body(640)
        if k >= sec(1.0):
            b["img"][ps.L_WRIST] = [640 + 320, 300 - 60]     # arm fully out
            b["img"][ps.L_ELBOW] = [640 + 180, 300 - 60]
        fr.append(frame({"A": b}))

    def check(o, crops, t, f):
        x0, y0, cw, ch = crops[-1]
        return None if x0 + cw >= 640 + 320 else "wrist still outside crop"
    return fr, ["A"] * len(fr), check


def scene_performer_leaves_newcomer():
    """A leaves; nobody for 3 s; C walks in alone -> C becomes the performer."""
    fr, ex = [], []
    for k in range(sec(1)):
        fr.append(frame({"A": body(640)}))
        ex.append("A")
    for k in range(sec(3)):
        fr.append(frame({}))
        ex.append(None)
    for k in range(sec(1)):
        fr.append(frame({"C": body(950)}))
        ex.append(ANY if k < 2 else "C")
    return fr, ex, lambda o, c, t, f: (
        None if t.generation == 2 else f"generation {t.generation}")


def scene_lost_timing():
    """MediaPipe finds nothing for 1.9 s: still tracking, crop held. At
    LOST_S it goes back to searching the full frame."""
    fr = [frame({"A": body(640)})] * sec(1)
    fr += [frame({"A": body(640)}, mp=None)] * sec(1.9)
    out1, crops1, t1 = run(fr)
    fr2 = fr + [frame({"A": body(640)}, mp=None)] * sec(0.3)
    out2, crops2, t2 = run(fr2)
    errs = []
    if t1.mode != "track" or crops1[-1] != crops1[sec(1) - 1]:
        errs.append(f"after 1.9 s: mode {t1.mode}")
    if t2.mode != "search":
        errs.append(f"after 2.2 s: mode {t2.mode}")
    return errs


def scene_reset_key():
    """r with A alone: forget, search full frame, re-acquire A (generation 2)."""
    fr = [frame({"A": body(640)}, r=(k == sec(1))) for k in range(sec(2))]
    ex = ["A"] * len(fr)
    ex[sec(1)] = ANY
    return fr, ex, lambda o, c, t, f: (
        None if t.generation == 2 else f"generation {t.generation}")


def unit_mapping_and_inview():
    """Landmarks come back in full-frame normalised coords; a landmark outside
    the crop is flagged not in view."""
    errs = []
    b = body(640)
    b["img"][ps.L_WRIST] = [1250, 300]       # far outside the crop
    b["vis"][ps.L_WRIST] = 0.1               # low vis: doesn't re-fit the crop
    fr = [frame({"A": body(640)})] * 5 + [frame({"A": b})]
    global SIM
    out, crops, t = run(fr)
    lat = t.latest()
    err = np.abs(lat["img"] * [W, H] - b["img"]).max()
    if err > 1e-6 * W:
        errs.append(f"img mapping off by {err:.3f} px")
    if lat["inview"][ps.L_WRIST]:
        errs.append("wrist outside crop reported in view")
    if not lat["inview"][ps.L_SHOULDER]:
        errs.append("shoulder reported out of view")
    if crops[-1] is None or crops[-1][2] >= W:
        errs.append(f"crop not narrower than frame: {crops[-1]}")
    return errs


def unit_fit_stays_in_frame():
    errs = []
    for cx, cy, s in [(20, 300, 100), (1260, 300, 100), (640, 20, 100),
                      (640, 700, 100), (640, 360, 400), (640, 360, 5)]:
        b = body(cx, cy, s)
        sig = ps.pose_signature(b["img"], hips_in=True, min_scale=4.0)
        x0, y0, cw, ch = ps.PersonTracker._fit(b["img"], b["vis"], sig, W, H)
        if not (0 <= x0 and 0 <= y0 and x0 + cw <= W and y0 + ch <= H):
            errs.append(f"({cx},{cy},{s}) -> crop {x0, y0, cw, ch} leaves frame")
        if min(cw, ch) < min(ps.CROP_MIN_PX, W, H):
            errs.append(f"({cx},{cy},{s}) -> crop {cw}x{ch} below CROP_MIN_PX")
    return errs


SCENES = [
    scene_single_acquires,
    scene_bystander_outside_crop_never_seen,
    scene_arm_covered_by_object,
    scene_shoulders_covered_briefly,
    scene_shoulders_covered_long_object,
    scene_shoulder_covered_by_near_walker,
    scene_redetect_onto_walker_far_rejected,
    scene_performer_walks,
    scene_performer_approaches,
    scene_fast_punch_reaches_edge,
    scene_performer_leaves_newcomer,
    scene_reset_key,
    scene_walker_same_depth_passes_in_front,
    scene_walker_stops_in_front,
]
UNITS = [scene_lost_timing, unit_mapping_and_inview, unit_fit_stays_in_frame]
KNOWN_GAPS = {"scene_walker_same_depth_passes_in_front", "scene_walker_stops_in_front"}


def ok_frame(got, want):
    if want == ANY:
        return True
    if isinstance(want, tuple):
        return got in want
    return got == want


def timeline(out, ex, k0):
    lo, hi = max(0, k0 - 5), min(len(out), k0 + 15)
    return "\n".join(f"      {k:4d} t={k * DT:5.2f}s  want {ex[k]!s:12} got {out[k]}"
                     for k in range(lo, hi))


def main():
    failed, gaps_fixed = [], []
    for fn in SCENES + UNITS:
        name = fn.__name__
        gap = name in KNOWN_GAPS
        if fn in UNITS:
            errs = fn()
            detail = ""
        else:
            frames, ex, check = fn()
            out, crops, trk = run(frames)
            bad = [k for k in range(len(out)) if not ok_frame(out[k], ex[k])]
            errs = []
            if bad:
                wrong = {out[k] for k in bad}
                errs.append(f"{len(bad)}/{len(out)} frames wrong (got {sorted(map(str, wrong))}),"
                            f" first at t={bad[0] * DT:.2f}s")
            if check is not None:
                e = check(out, crops, trk, frames)
                if e:
                    errs.append(e)
            detail = timeline(out, ex, bad[0]) if (bad and VERBOSE) else ""
        tag = "PASS" if not errs else ("KNOWN GAP" if gap else "FAIL")
        if gap and not errs:
            tag = "PASS (known gap now passes - update KNOWN_GAPS)"
            gaps_fixed.append(name)
        print(f"{tag:10} {name}")
        for e in errs:
            print(f"           {e}")
        if detail:
            print(detail)
        if errs and not gap:
            failed.append(name)
    print(f"\n{len(SCENES) + len(UNITS) - len(failed)} ok, {len(failed)} failed"
          f" ({len(KNOWN_GAPS)} known gaps reported, not counted)")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
