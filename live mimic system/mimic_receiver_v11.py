"""
mimic_receiver_v11.py - live receiver for the 36 inch robot (humanoid_v9).

    python sim36/build_humanoid_v9_scene.py     (once)
    python mimic_receiver_v11.py
    python pose_sender.py --landmarks           (other terminal)

WHAT CHANGED FROM v7
====================
v7 consumed JOINT ANGLES from pose_sender and mapped flex/abd onto pitch/roll,
with twist parked at zero. That mapping is what made the early videos poor.

v9 consumes RAW WORLD LANDMARKS and runs the same 4-DOF Gauss-Newton IK the
offline driver uses, against this robot's real kinematics:

    unknowns : pitch, roll, twist, elbow
    residual : [u_desired - u_fk, f_desired - f_fk]   (6 eq, 4 unknowns)

so twist is solved rather than guessed, and what you see in
sim/out36/mimic_sidebyside_v9.mp4 is what a live camera produces.

Falls back to the old angle mapping if the sender is an older one that only
transmits angles, so it stays compatible.
"""
import json
import os
import socket
import time

import sys

import mujoco
import mujoco.viewer

HEADLESS = "--headless" in sys.argv   # no window: robot rigs and CI


def _argval(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


# --dump <path.npz> : record, every 1/30 s of SIM time, exactly what this loop
#   commanded and where the robot actually ended up. The rendered video is then
#   replayed from these qpos, so what you watch is the live solve, not a rerun.
DUMP = _argval("--dump")
# --------------------------------------------------------------------------
# VIEWER CHROME.
# MuJoCo's passive viewer opens with two docked UI panels (left: simulation /
# rendering settings, right: watch + control sliders). They are laid out in
# FIXED PIXEL WIDTHS, so shrinking the window does not shrink them - below
# about 900 px wide they cover the robot completely.
#
# mujoco.viewer.launch_passive(model, data, show_left_ui=..., show_right_ui=...)
# is the supported way to start without them; internally it just sets
# Simulate.ui0_enable / ui1_enable. Verified against the installed mujoco
# (3.11.0) with inspect.signature, not assumed.
#
# Default here is OFF - a clean window. Pass --ui to get the panels back at
# startup, or press u in the window to toggle them at any time.
# --------------------------------------------------------------------------
SHOW_UI = "--ui" in sys.argv


def make_key_callback(handle_cell):
    """u toggles both docked UI panels; the viewer's own Tab / Shift-Tab
    bindings keep working alongside it."""
    def cb(keycode):
        if keycode not in (ord("u"), ord("U")):
            return
        h = handle_cell.get("h")
        sim = h._sim() if h is not None else None      # weakref to _Simulate
        if sim is None:
            return
        on = not (sim.ui0_enable or sim.ui1_enable)
        sim.ui0_enable = on
        sim.ui1_enable = on
    return cb

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENE = os.path.join(REPO_ROOT, "sim36", "humanoid_v11_scene.xml")

import importlib.util as _il
_spec = _il.spec_from_file_location(
    "pose_sender", os.path.join(HERE, "pose_sender.py"))
ps = _il.module_from_spec(_spec); _spec.loader.exec_module(ps)

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35
RUN_SECS = float(__import__("os").environ.get("RUN_SECS",65.0))         # headless run length
# Slew-limit the command to what the servo can ACTUALLY do. The STS3215 is
# 0.22 s/60 deg at 12 V = 273 deg/s = 4.8 rad/s no-load, and less under load.
# This was 8.0 rad/s (458 deg/s) - 1.7x beyond the hardware - so the live path
# promised motion the real servo could never deliver, and it looked visibly
# jumpier than the offline driver, which has been limited to 3.0 all along.
# Matching the offline value: 3.0 rad/s = 172 deg/s, comfortably inside the
# servo with margin for load. Measured demand on real clips is p90 45-118 deg/s,
# so this clips only the ~2% of frames that were tracking glitches anyway.
MAX_RATE = 3.0          # rad/s cap on how fast the command may move

# --------------------------------------------------------------------------
# GLITCH REJECTION GATE  (--gate, OFF by default)
# --------------------------------------------------------------------------
# MAX_RATE alone cannot tell a tracking glitch from a fast real move: it only
# limits SPEED, and it applies that same limit to both. MEASURED on ref_clip
# (sim36/noise_experiments.py, 3 onsets, deviation from the clean run):
#
#   glitch injected           joint travels   wrist tip   visibly wrong for
#   ONE frame,  40 cm             10-17 deg    3.6-5.2 cm    7-8 frames  240 ms
#   two frames, 40 cm             16-23 deg    6.3-8.3 cm    9-10 frames 300 ms
#   ten frames, 40 cm             40-61 deg     18-19 cm     17-20 frames
#
# i.e. the 5.73 deg/frame slew budget does NOT contain even a single bad
# frame - the One Euro filter keeps the bad target alive for several frames
# after it is gone, so one bad frame buys ~7 frames of visible motion.
#
# The gate adds the missing piece - PLAUSIBILITY. A jump larger than any human
# joint can do in 1/30 s is not believed on the strength of one frame; it has
# to still be there GATE_HOLD frames later. Real motion is continuous, so its
# per-frame steps are small and the gate rarely fires on it; a teleport that
# persists (the tracker re-locked onto a genuinely new pose) is accepted after
# at most GATE_HOLD frames, so nothing can be blocked forever.
#
# TUNING (sweep in noise_experiments.py section e/f, 26 deg / 3 frames):
#   1-2 frame impulses:  joint travel 10-23 deg  ->  2-7 deg
#   a REAL move:         followed to within 0.15 deg, delayed 33-67 ms
#   cost on the clean clip: fires on 2.3% of frames, 3.7 deg rms / 0.9 cm rms
#   5+ frame glitches:   barely helped - by then it IS a pose change, and no
#                        causal filter can tell the difference
#   dense gaussian landmark noise: the gate makes it slightly WORSE (it is an
#                        outlier rejector, not a filter) - One Euro owns that.
# Default is still OFF; pass --gate to switch it on.
GATE_ON = "--gate" in sys.argv
GATE_STEP = float(os.environ.get("GATE_STEP", 0.45))   # rad/frame, per joint
GATE_HOLD = int(os.environ.get("GATE_HOLD", 3))        # max frames to hold
GATE_MODE = os.environ.get("GATE_MODE", "joint")       # "joint" or "all"


class StepGate:
    """Hold the last good command for any joint whose demanded step is bigger
    than a human joint can move in one frame.

        filter(raw_q) -> (q_to_apply, max_step_rad, n_rejected)

    mode "joint" (default): only the joints that jumped are held; the rest of
        the body keeps tracking. A glitched wrist landmark then cannot freeze
        the other arm.
    mode "all": the whole packet is dropped (simpler, more collateral).

    Neither mode can hold a joint for more than max_hold consecutive frames,
    so a real movement is delayed by at most max_hold/30 s and is never
    blocked: if the new pose is still there after the hold, it is accepted.
    """

    def __init__(self, step_thresh=GATE_STEP, max_hold=GATE_HOLD, mode=GATE_MODE):
        self.thr = float(step_thresh)
        self.max_hold = int(max_hold)
        self.mode = mode
        self.last = {}        # last ACCEPTED raw IK solution, per joint
        self.held = {}        # consecutive rejects, per joint
        self.nrej = 0         # joint-frames rejected
        self.nfrm = 0         # frames with at least one rejection
        self.nforce = 0       # holds that ran out of budget and were accepted

    def filter(self, q):
        if not self.last:
            self.last = dict(q)
            self.held = {k: 0 for k in q}
            return dict(q), 0.0, 0
        mx = max((abs(q[k] - self.last[k]) for k in q if k in self.last),
                 default=0.0)
        if self.mode == "all":
            hold_all = mx > self.thr and self.held.get("*", 0) < self.max_hold
            if hold_all:
                self.held["*"] = self.held.get("*", 0) + 1
                self.nrej += len(q); self.nfrm += 1
                return {}, mx, len(q)
            if mx > self.thr:
                self.nforce += 1
            self.held["*"] = 0
            self.last = dict(q)
            return dict(q), mx, 0
        out, nrej = {}, 0
        for k, v in q.items():
            prev = self.last.get(k)
            h = self.held.get(k, 0)
            if prev is not None and abs(v - prev) > self.thr and h < self.max_hold:
                self.held[k] = h + 1
                nrej += 1
                continue                      # hold: this joint keeps its target
            if prev is not None and abs(v - prev) > self.thr:
                self.nforce += 1
            self.held[k] = 0
            self.last[k] = v
            out[k] = v
        self.nrej += nrej
        self.nfrm += 1 if nrej else 0
        return out, mx, nrej

    # snapshot/restore, used by the offline harness
    def state(self):
        return (dict(self.last), dict(self.held), self.nrej, self.nfrm, self.nforce)

    def load(self, s):
        if s is None:
            self.last, self.held = {}, {}
            self.nrej = self.nfrm = self.nforce = 0
            return
        last, held, self.nrej, self.nfrm, self.nforce = s
        self.last, self.held = dict(last), dict(held)


REST_ROLL = 0.0      # v5 zero IS the arm hanging, so abduction ADDS to it

# (joint, scale, offset): commanded = offset + scale * incoming
JOINT_MAP = {
    "L_shoulder_flex": ("L_shoulder_pitch", 1.0, 0.0),
    "R_shoulder_flex": ("R_shoulder_pitch", 1.0, 0.0),
    "L_shoulder_abd":  ("L_shoulder_roll", 1.0, REST_ROLL),
    "R_shoulder_abd":  ("R_shoulder_roll", 1.0, REST_ROLL),
    "L_elbow": ("L_elbow", 1.0, 0.0),
    "R_elbow": ("R_elbow", 1.0, 0.0),
    "L_hip_flex": ("L_hip_flex", 1.0, 0.0), "L_hip_abd": ("L_hip_abd", 1.0, 0.0),
    "L_knee": ("L_knee", 1.0, 0.0),
    "R_hip_flex": ("R_hip_flex", 1.0, 0.0), "R_hip_abd": ("R_hip_abd", 1.0, 0.0),
    "R_knee": ("R_knee", 1.0, 0.0),
}
# joints with no incoming signal still need a sane hold value
PARKED = {"L_shoulder_twist": 0.0, "R_shoulder_twist": 0.0}


def main():
    if not os.path.exists(SCENE):
        raise SystemExit(
            f"scene not found: {SCENE}\nrun:  python sim/sim36/build_humanoid_v9_scene.py")

    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)

    joints = sorted({t for t, _, _ in JOINT_MAP.values()} | set(PARKED))
    act, limits = {}, {}
    for name in joints:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if aid < 0 or jid < 0:
            raise RuntimeError(f"joint/actuator '{name}' missing from the scene")
        act[name] = aid
        limits[name] = tuple(model.jnt_range[jid]) if model.jnt_limited[jid] \
            else (-3.2, 3.2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(UDP_ADDR)
    sock.setblocking(False)

    target = {n: 0.0 for n in joints}
    for n, v in PARKED.items():
        target[n] = v
    # start where the arm actually rests, not at roll=0 (which is arm-straight-up)
    for s_ in ("L", "R"):
        target[f"{s_}_shoulder_roll"] = REST_ROLL
    cmd = dict(target)

    ik = ArmIK(model)
    gate = StepGate() if GATE_ON else None
    if gate is not None:
        print(f"glitch gate ON ({gate.mode}): hold a joint whose demanded step "
              f"exceeds {_np.degrees(gate.thr):.1f} deg/frame, for at most "
              f"{gate.max_hold} frames ({gate.max_hold/30.0*1e3:.0f} ms).")
    qadr ={n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in joints}
    jfilt = {}          # One Euro per joint, applied per PACKET not per step
    dt = model.opt.timestep
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Physics ON: position actuators + mj_step.")
    print("Shoulder v6: pitch +/-90, roll -10..150, twist +/-90, elbow 0..120.")
    print(f"Rest roll = {REST_ROLL:.4f} rad (arm hanging); abduction adds to it.")
    print("Camera side, other terminal:  python pose_sender.py\n")

    class _NoViewer:
        def __enter__(self): return self
        def __exit__(self, *a): return False
        def is_running(self): return time.time() - self._t0 < RUN_SECS
        def sync(self): pass
        _t0 = time.time()

    cell = {}
    ctx = (_NoViewer() if HEADLESS else
           mujoco.viewer.launch_passive(model, data,
                                        key_callback=make_key_callback(cell),
                                        show_left_ui=SHOW_UI,
                                        show_right_ui=SHOW_UI))
    if not HEADLESS:
        cell["h"] = ctx
        print("Viewer: side panels are OFF (clean window). Press u to toggle "
              "them, or start with --ui.")
    with ctx as viewer:
        connected = False
        wall0 = time.time()
        npkt = 0            # heartbeat: bring-up needs to see the wire is live
        nhb = time.time()
        rec = [] if DUMP else None
        solves = [] if DUMP else None
        next_rec = 0.0
        while viewer.is_running():
            while True:
                try:
                    packet, _ = sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    break
                npkt += 1
                try:
                    ang = json.loads(packet.decode())
                except Exception:
                    continue
                if "world" in ang:
                    # RAW LANDMARK PACKET (pose_sender.py --landmarks).
                    # This runs the same 4-DOF IK as the offline driver, so the
                    # live robot and the rendered video agree. The angle path
                    # below cannot express twist at all - the ArmIK class was
                    # already here but nothing ever called it, so every
                    # --landmarks packet was being silently dropped.
                    _t0 = time.perf_counter()
                    raw_q = ik.targets(ang["world"],
                                       bool(ang.get("hips_ok", False)), ps)
                    _pkt_ms = (time.perf_counter() - _t0) * 1e3
                    use_q, _gstep, _nrej = ((raw_q, 0.0, 0) if gate is None
                                            else gate.filter(raw_q))
                    if solves is not None:
                        solves.append({"t": time.time() - wall0,
                                       "npkt": npkt,
                                       "src_i": int(ang.get("i", -1)),
                                       "hips_ok": bool(ang.get("hips_ok", False)),
                                       "iters": int(ik.last_iters),
                                       "res": float(ik.last_res),
                                       "ms": float(_pkt_ms),
                                       "gated": float(_nrej),
                                       "gstep": float(_gstep),
                                       **{k: float(v) for k, v in raw_q.items()}})
                    if not use_q:
                        continue      # every joint held: nothing to apply
                    # One Euro ONCE PER PACKET. This used to be a
                    # SMOOTH_ALPHA blend inside the mj_step loop - 500 Hz
                    # against a 30 Hz input, a 5.7 ms time constant, i.e. a
                    # pass-through. Raw solve jitter (p90 5-7 deg/frame, max 29)
                    # reached the servos untouched and the live path had 2.5-3x
                    # the command reversals of the offline one.
                    tnow = time.time()
                    for name, val in use_q.items():
                        if name in limits:
                            lo, hi = limits[name]
                            f = jfilt.get(name)
                            if f is None:
                                f = jfilt[name] = ps.OneEuro()
                            target[name] = max(lo, min(hi, float(f(val, tnow))))
                    continue
                for key, val in ang.items():
                    if key not in JOINT_MAP:
                        continue
                    name, scale, offset = JOINT_MAP[key]
                    lo, hi = limits[name]
                    target[name] = max(lo, min(hi, offset + float(val) * scale))
                if not connected:
                    print("Receiving pose data - mirroring.")
                    connected = True

            # Smoothing now happens once per packet (One Euro, above). What is
            # left here is the RATE LIMITER, which is a safety feature, not a
            # filter: it caps how fast a command may move regardless of what the
            # tracker does. SMOOTH_ALPHA at 500 Hz was never filtering anything.
            for name in joints:
                blended = target[name]
                delta = max(-max_step, min(max_step, blended - cmd[name]))
                cmd[name] += delta
                data.ctrl[act[name]] = cmd[name]

            mujoco.mj_step(model, data)
            viewer.sync()
            if rec is not None and data.time >= next_rec:
                rec.append({"t": data.time, "npkt": npkt,
                            # in the SAME order as cmd/target below. Saving
                            # raw data.qpos here stored model order against
                            # sorted-name labels, which made the right arm look
                            # frozen when it was moving perfectly.
                            "qpos": _np.array([data.qpos[qadr[n]] for n in joints]),
                            "cmd": _np.array([cmd[n] for n in joints]),
                            "target": _np.array([target[n] for n in joints])})
                next_rec += 1.0 / 30.0
            if time.time() - nhb >= 2.0:
                print(f"  {npkt:6d} packets   L roll {_np.degrees(cmd['L_shoulder_roll']):6.1f}"
                      f"  pitch {_np.degrees(cmd['L_shoulder_pitch']):6.1f}"
                      f"  elbow {_np.degrees(cmd['L_elbow']):6.1f} deg", flush=True)
                nhb = time.time()

            lag = data.time - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)

    if rec:
        blob = {"t": _np.array([r["t"] for r in rec]),
                "npkt": _np.array([r["npkt"] for r in rec]),
                "qpos": _np.array([r["qpos"] for r in rec]),
                "cmd": _np.array([r["cmd"] for r in rec]),
                "target": _np.array([r["target"] for r in rec]),
                "joints": _np.array(joints),
                "solve_keys": _np.array(sorted(solves[0]) if solves else []),
                "solves": _np.array([[s_[k] for k in sorted(solves[0])]
                                     for s_ in solves]) if solves else _np.zeros((0, 0))}
        os.makedirs(os.path.dirname(os.path.abspath(DUMP)), exist_ok=True)
        _np.savez_compressed(DUMP, **blob)
        print(f"receiver dump -> {DUMP}  ({len(rec)} ticks, "
              f"{len(solves or [])} IK solves, {npkt} packets)", flush=True)


# ---------------------------------------------------------------------------
# 4-DOF closed-form-ish IK against the real chain (same maths as the offline
# driver in sim36/mimic_from_video_v9.py)
# ---------------------------------------------------------------------------
import numpy as _np

MIR = _np.array([1.0, -1.0, 1.0])
SHP = {"L": _np.array([0.0, 0.1184, 0.748]), "R": _np.array([0.0, -0.1184, 0.748])}
J4 = ("shoulder_pitch", "shoulder_roll", "shoulder_twist", "elbow")
LM = {"L": (11, 13, 15), "R": (12, 14, 16)}


class ArmIK:
    def __init__(self, model):
        import mujoco as mj
        self.mj = mj
        self.m = model
        self.d = mj.MjData(model)
        self.Q = lambda n: model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]
        self.qh = _np.zeros(model.nq)
        for s in "LR":
            # v9 measured roll down from an overhead zero, so pi meant hanging.
            # v6 mounted inverted hangs at 0 - probing from pi put every FK
            # probe in an arms-overhead pose and the solver answered with ~160
            # deg of humeral twist on every frame.
            self.qh[self.Q(f"{s}_shoulder_roll")] = 0.0
        self.lo = _np.array([model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"L_{j}")][0] for j in J4])
        self.hi = _np.array([model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"L_{j}")][1] for j in J4])
        self.seed = {"L": _np.array([0.0, 0.35, 0.0, 0.6]),
                     "R": _np.array([0.0, 0.35, 0.0, 0.6])}
        self.last_iters = 0     # diagnostics only
        self.last_res = 0.0
        self.last_ms = 0.0

    def fk(self, side, q):
        mj = self.mj
        self.d.qpos[:] = self.qh
        for k, j in enumerate(J4):
            self.d.qpos[self.Q(f"{side}_{j}")] = q[k]
        mj.mj_forward(self.m, self.d)
        b = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_BODY, f"{side}_forearm")
        ua = self.d.xpos[b] - SHP[side]
        fa = self.d.xmat[b].reshape(3, 3) @ _np.array([0.0, -1.0, 0.0])
        if side == "R":
            ua, fa = ua * MIR, fa * MIR
        return ua / (_np.linalg.norm(ua) + 1e-9), fa / (_np.linalg.norm(fa) + 1e-9)

    def solve(self, side, u, f):
        _t = time.perf_counter()
        q = self.seed[side].copy()
        nit = 0
        for _ in range(12):
            nit += 1
            u0, f0 = self.fk(side, q)
            res = _np.concatenate([u - u0, f - f0])
            if _np.linalg.norm(res) < 1e-4:
                break
            J = _np.zeros((6, 4))
            for k in range(4):
                qp = q.copy(); qp[k] += 1e-3
                u1, f1 = self.fk(side, qp)
                J[:, k] = _np.concatenate([(u1 - u0) / 1e-3, (f1 - f0) / 1e-3])
            # DAMPED least squares (Levenberg-Marquardt). Plain lstsq is
            # undamped, so when a joint saturates - and shoulder roll saturates
            # whenever the user crosses their arms, because the module only has
            # -10 deg of adduction against the -43 a cross demands - the
            # unreachable residual drives huge dq, the clip fights it, and the
            # solution jumps between branches frame to frame. That jumping is
            # the "it gets confused". Damping makes an unreachable target
            # degrade to the CLOSEST reachable pose instead, smoothly.
            lam = 0.05 + 2.0 * float(_np.linalg.norm(res)) ** 2
            JtJ = J.T @ J + lam * _np.eye(4)
            dq = _np.linalg.solve(JtJ, J.T @ res)
            q = _np.clip(q + _np.clip(dq, -0.35, 0.35), self.lo, self.hi)
        self.seed[side] = q.copy()
        u0, f0 = self.fk(side, q)
        self.last_iters = nit
        self.last_res = float(_np.linalg.norm(
            _np.concatenate([u - u0, f - f0])))
        self.last_ms = (time.perf_counter() - _t) * 1e3
        return q

    def targets(self, world, hips_ok, ps):
        """world: (33,3) RAW MediaPipe world landmarks, NOT axis-flipped."""
        W = _np.asarray(world)
        fwd, left, up = ps.body_frame(W, hips_ok)
        Rb = _np.stack([fwd, left, up])
        out = {}
        for side, (sh, el, wr) in LM.items():
            u = Rb @ (W[el] - W[sh]); f = Rb @ (W[wr] - W[el])
            if side == "R":
                u, f = u * MIR, f * MIR
            nu, nf = _np.linalg.norm(u), _np.linalg.norm(f)
            if nu < 1e-6 or nf < 1e-6:
                continue
            q = self.solve(side, u / nu, f / nf)
            for k, j in enumerate(J4):
                out[f"{side}_{j}"] = float(q[k])
        return out


if __name__ == "__main__":
    # NOTE: this must stay at the BOTTOM. It used to sit above ArmIK, so
    # running the file as a script hit ArmIK(model) before the class existed
    # and died with NameError - importing it hid the bug because main() never
    # ran on import.
    main()
