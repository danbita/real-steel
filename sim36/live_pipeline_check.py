"""
live_pipeline_check.py - test the WHOLE live pipeline without a camera.

    python sim36/live_pipeline_check.py            # all three checks (~45 s)
    python sim36/live_pipeline_check.py fusion     # just one of them
    python sim36/live_pipeline_check.py single
    python sim36/live_pipeline_check.py stereo

A synthetic person (live mimic system/synthetic_person.py) stands in for
you. Everything else is the REAL code:

  fusion  two simulated cameras 60 deg apart with MediaPipe-like depth noise
          -> stereo_fusion -> limb-direction error, one camera vs fused.
  single  fake camera + fake MediaPipe -> the real pose_sender.py main loop
          -> UDP -> the real mimic_receiver_v11.py (headless) -> did the
          robot's joints follow the person, how far behind, how accurately.
  stereo  the same, through pose_sender_stereo.py with two fake cameras.

Only the camera and the MediaPipe network are faked, so a PASS here means a
failure on your Mac is in the camera / MediaPipe / viewer, and the STATUS
lines in the two terminals say which.
"""
import _thread
import os
import subprocess
import sys
import tempfile
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LIVE = os.path.join(ROOT, "live mimic system")
sys.path.insert(0, LIVE)
import synthetic_person as sp  # noqa: E402
import stereo_fusion as sf  # noqa: E402

RUN_S = 13.0
CAM_YAW = 60.0          # simulated angle between the two cameras


def Ry(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Noise:
    """MediaPipe-like error: ~1.2 cm across the image, much worse and
    slowly-wandering along the camera's line of sight."""

    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)
        self.bias = np.zeros(33)

    def __call__(self, X):
        X = X.copy()
        self.bias = 0.95 * self.bias + 0.05 * self.rng.normal(0, 0.06, 33)
        X[:, :2] += self.rng.normal(0, 0.012, (33, 2))
        X[:, 2] += self.rng.normal(0, 0.04, 33) + self.bias
        return X


def limb_err(X, W):
    e = []
    for sh, el, wr in ((11, 13, 15), (12, 14, 16)):
        for p, q in ((sh, el), (el, wr)):
            u, v = X[q] - X[p], W[q] - W[p]
            c = u @ v / np.linalg.norm(u) / np.linalg.norm(v)
            e.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    return e


# --------------------------------------------------------------------------
def check_fusion():
    print("\n== fusion: one camera vs two, MediaPipe-like depth noise")
    al = sf.ViewAligner()
    nA, nB = Noise(1), Noise(2)
    Rb = Ry(CAM_YAW)
    eA, eF = [], []
    for t in np.arange(0, 24, 1 / 20):
        W, vis = sp.pose(t)
        A = nA(W)
        B = nB((Rb @ W.T).T)
        al.update(A, vis, B, vis)
        if t < 6 or not al.ready:
            continue
        F, _ = sf.fuse(A, vis, B, vis, al.R, al.t)
        eA += limb_err(A, W)
        eF += limb_err(F, W)
    ang = al.angle_deg()
    print(f"   aligner found the cameras {ang:.1f} deg apart (true {CAM_YAW:.0f})")
    print(f"   limb direction error: one camera {np.mean(eA):.1f} deg "
          f"(p95 {np.percentile(eA, 95):.1f})  ->  fused {np.mean(eF):.1f} deg "
          f"(p95 {np.percentile(eF, 95):.1f})")
    ok = abs(ang - CAM_YAW) < 3 and np.mean(eF) < 0.7 * np.mean(eA)
    print("   PASS" if ok else "   FAIL")
    return ok


# --------------------------------------------------------------------------
# fakes for the camera and MediaPipe
class _P:
    __slots__ = ("x", "y", "z", "visibility")

    def __init__(self, x, y, z=0.0, v=0.99):
        self.x, self.y, self.z, self.visibility = x, y, z, v


class FakeCam:
    """30 fps camera whose frames carry their camera index in the pixels."""

    def __init__(self, src, width=None, height=None, name="cam"):
        self.idx = int(src) if str(src).isdigit() else 0
        self.frame = np.full((360, 640, 3), 40 + 60 * self.idx, np.uint8)
        self.seq, self.nread, self.ok = 0, 0, True
        self.t0 = time.time()

    def latest(self):
        n = int((time.time() - self.t0) * 30)
        if n != self.seq:
            self.nread += n - self.seq
            self.seq = n
        return self.frame, self.seq, time.time()

    def size(self):
        return 640, 360

    def status(self):
        return "live"

    def release(self):
        pass


class FakeLandmarker:
    """detect_async -> callback ~25 ms later with the synthetic person, as
    seen by the camera whose index is painted in the frame."""

    t0 = None
    noise = {}

    def __init__(self, cb):
        self.cb = cb
        if FakeLandmarker.t0 is None:
            FakeLandmarker.t0 = time.time()

    def detect_async(self, image, ts):
        idx = int(round((int(image.numpy_view()[0, 0, 0]) - 40) / 60))
        t = time.time() - FakeLandmarker.t0
        W, vis = sp.pose(t)
        if idx == 1:
            W = (Ry(CAM_YAW) @ W.T).T
        W = FakeLandmarker.noise.setdefault(idx, Noise(10 + idx))(W)
        img = [_P(0.5 + 0.8 * p[0], 0.55 + 0.8 * p[1]) for p in W]
        wld = [_P(*p) for p in W]
        res = types.SimpleNamespace(pose_landmarks=[img],
                                    pose_world_landmarks=[wld])
        threading.Timer(0.025, self.cb, (res, None, ts)).start()

    def close(self):
        pass


def _run_pipeline(label, sender_module, argv):
    """Real receiver (headless, subprocess) <- UDP <- real sender (here,
    with fakes). Returns per-joint (lag ms, rms deg, peak, truth peak)."""
    dump = os.path.join(tempfile.mkdtemp(), "dump.npz")
    rcv = subprocess.Popen(
        [sys.executable, os.path.join(LIVE, "mimic_receiver_v11.py"),
         "--headless", "--dump", dump],
        env=dict(os.environ, RUN_SECS=str(RUN_S + 3)),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(2.0)

    sys.argv = argv
    for m in [m for m in sys.modules if m in ("pose_sender", "pose_sender_stereo")]:
        del sys.modules[m]
    import pose_sender as ps
    ps.ensure_model = lambda: None
    ps.CameraStream = FakeCam
    ps.make_landmarker = lambda callback=None: FakeLandmarker(callback or ps.on_result)
    FakeLandmarker.t0 = None
    FakeLandmarker.noise = {}
    mod = __import__(sender_module) if sender_module != "pose_sender" else ps
    threading.Timer(RUN_S, _thread.interrupt_main).start()
    try:
        mod.main()
    except KeyboardInterrupt:
        pass
    out, _ = rcv.communicate(timeout=60)
    status = [l for l in out.splitlines() if l.startswith("STATUS")]
    if status:
        print("   receiver: " + status[len(status) // 2])
    d = np.load(dump)
    J = list(d["joints"])
    # the fake person's clock starts with the sender; align on the first packet
    first = d["t"][np.argmax(d["npkt"] > 0)]
    ts = d["t"] - first
    res = {}
    lags = []
    for jn, ix in (("L_shoulder_pitch", 0), ("L_shoulder_roll", 1),
                   ("L_elbow", 2), ("R_shoulder_roll", 1)):
        truth = np.array([sp.truth(x)[ix] if x > 0 else 0.0 for x in ts])
        y = np.degrees(d["qpos"][:, J.index(jn)])
        sel = (ts > 0.5) & (ts < RUN_S - 1.5)
        n = int(sel.sum())
        errs = {L: np.sqrt(np.mean((y[sel][L:] - truth[sel][:n - L]) ** 2))
                for L in range(0, 25)}
        L = min(errs, key=errs.get)
        lags.append(L)
        res[jn] = (L * 33, errs[L], y[sel].max(), truth[sel].max())
        print(f"   {jn:17} lag {L*33:4d} ms   joint rms {errs[L]:4.1f} deg   "
              f"peak {y[sel].max():5.1f} / {truth[sel].max():5.1f}")
    # What you SEE is where the arm points, not the joint numbers: near arm-
    # straight-out-sideways, pitch barely moves the arm, so a pitch "error"
    # there is not a visible error. Score the robot's upper-arm and forearm
    # DIRECTIONS against the person's, after removing the measured lag.
    import mujoco
    spec = __import__("importlib.util").util.spec_from_file_location(
        "rcv_mod", os.path.join(LIVE, "mimic_receiver_v11.py"))
    rm = __import__("importlib.util").util.module_from_spec(spec)
    sys_argv, sys.argv = sys.argv, ["x", "--headless"]
    spec.loader.exec_module(rm)
    sys.argv = sys_argv
    ik = rm.ArmIK(mujoco.MjModel.from_xml_path(rm.SCENE))
    lag = int(np.median(lags))
    derr = []
    sel_i = np.where((ts > 0.5) & (ts < RUN_S - 1.5))[0]
    for k in sel_i[lag:]:
        W, _ = sp.pose(ts[k - lag])
        for side, (sh, el, wr) in rm.LM.items():
            q = np.array([d["qpos"][k, J.index(f"{side}_{j}")] for j in rm.J4])
            u0, f0 = ik.fk(side, q)
            u, f = W[el] - W[sh], W[wr] - W[el]
            B = np.stack([sp.FWD, sp.LEFT, sp.UP])
            u, f = B @ u, B @ f
            if side == "R":
                u, f = u * rm.MIR, f * rm.MIR
            for a, b in ((u, u0), (f, f0)):
                c = a @ b / np.linalg.norm(a) / np.linalg.norm(b)
                derr.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    derr = np.array(derr)
    print(f"   arm DIRECTION error (what you see): mean {derr.mean():4.1f} deg, "
          f"p95 {np.percentile(derr, 95):4.1f}")
    ok = (derr.mean() < 8.0 and max(lags) * 33 < 400 and
          all(v[2] > 0.8 * v[3] for v in res.values()))
    print("   PASS" if ok else "   FAIL")
    return ok


def check_single():
    print("\n== single camera: fake camera + fake MediaPipe -> REAL pose_sender "
          "-> UDP -> REAL receiver")
    return _run_pipeline("single", "pose_sender",
                         ["pose_sender.py", "--landmarks", "--noshow"])


def check_stereo():
    print("\n== two cameras: fake cameras + fake MediaPipe -> REAL "
          "pose_sender_stereo -> UDP -> REAL receiver")
    return _run_pipeline("stereo", "pose_sender_stereo",
                         ["pose_sender_stereo.py", "--cams", "0", "1", "--noshow"])


if __name__ == "__main__":
    which = sys.argv[1:] or ["fusion", "single", "stereo"]
    fns = {"fusion": check_fusion, "single": check_single, "stereo": check_stereo}
    results = {w: fns[w]() for w in which}
    print("\nRESULT:", "PASS" if all(results.values()) else
          "FAIL " + str([k for k, v in results.items() if not v]))
    sys.exit(0 if all(results.values()) else 1)
