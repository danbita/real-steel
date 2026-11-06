"""
mimic_receiver_v11.py - live receiver for the 36 inch robot (humanoid_v9).

    mjpython "live mimic system/mimic_receiver_v11.py"              (terminal 1)
    python   "live mimic system/pose_sender.py" --landmarks         (terminal 2)
 or python   "live mimic system/pose_sender_stereo.py"              (2 cameras)

    mjpython "live mimic system/mimic_receiver_v11.py" --selftest   no camera:
        a built-in fake person drives the arms, to prove the viewer side works.

Every 2 s it prints one STATUS line - packets/s, IK solves/s, IK time, how
fast the sim runs vs real time, age of the newest packet - plus a plain-English
warning when one of those is wrong. That line says which stage is failing.

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
import select
import socket
import time

import sys

import mujoco
import mujoco.viewer

HEADLESS = "--headless" in sys.argv   # no window: robot rigs and CI
SELFTEST = "--selftest" in sys.argv   # drive from synthetic_person.py, no sender
# --pretty : keep shadows and reflections in the viewer. Off by default: the
#   viewer redraws every display refresh (up to 120x/s on a ProMotion Mac) on
#   the same GPU that MediaPipe and the iPhone's Continuity Camera stream use,
#   and shadows+reflections are the bulk of that cost.
PRETTY = "--pretty" in sys.argv


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
REPO_ROOT = os.path.dirname(HERE)          # "live mimic system" sits directly in the repo
SCENE = os.path.join(REPO_ROOT, "sim36", "humanoid_v11_scene.xml")

import importlib.util as _il
_spec = _il.spec_from_file_location(
    "pose_sender", os.path.join(HERE, "pose_sender.py"))
ps = _il.module_from_spec(_spec); _spec.loader.exec_module(ps)

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35
RUN_SECS = float(__import__("os").environ.get("RUN_SECS",65.0))         # headless run length
# Slew-limit the command to what the servo can ACTUALLY do. The shoulder is now
# the Hiwonder HTS-35H (elbow HTS-20H): 0.18 s/60 deg at 11.1 V = 333 deg/s =
# 5.82 rad/s no-load, and less under load (sim36/servo_spec.py). This was once
# 8.0 rad/s - beyond the hardware - so the live path promised motion the real
# servo could never deliver. Matching the offline value: 3.0 rad/s = 172 deg/s,
# about half the servo's no-load speed, which is the margin a loaded servo
# needs. Measured demand on real clips is p90 45-118 deg/s, so this clips only
# the ~2% of frames that were tracking glitches anyway.
# Keep equal to servo_spec.MAX_RATE (that file asserts it is under servo speed).
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
    if not PRETTY:
        model.light_castshadow[:] = 0          # no shadow-map passes
        model.mat_reflectance[:] = 0.0         # no mirrored floor pass
    # The camera/pose sender is the time-critical process; this one can wait
    # a few ms. Lower OUR priority so the OS never starves the cameras for us.
    try:
        os.nice(10)
    except Exception:
        pass
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
    synth = None
    if SELFTEST:
        sys.path.insert(0, HERE)
        import synthetic_person as synth
        print("SELFTEST: a synthetic person drives the arms (12 s cycle: "
              "arms forward, arms out, elbows bend). No sender needed.")

    # Physics runs on the WALL CLOCK, the viewer redraws at ~60 Hz, and only the
    # NEWEST packet is solved. The old loop called viewer.sync() after every
    # 2 ms physics step (500 redraw hand-offs a second - on macOS that alone
    # can drag the sim far below real time) and solved EVERY queued packet, so
    # once it fell behind it spent its time on stale poses and the robot lagged
    # further and further behind you, or looked frozen.
    MAX_BEHIND = 0.25          # s: further behind than this, skip ahead
    SYNC_DT = 1.0 / 30.0       # viewer hand-off rate (30 Hz is plenty to watch)
    TICK = 0.01                # main loop runs ~100x/s, sleeping in between
    SYNTH_DT = 1.0 / 30.0
    with ctx as viewer:
        connected = False
        wall0 = time.time()
        sim_off = data.time            # sim time that corresponds to wall0
        npkt = 0
        rec = [] if DUMP else None
        solves = [] if DUMP else None
        next_rec = 0.0
        next_sync = 0.0
        next_synth = 0.0
        last_pkt_wall = None
        win = dict(t=time.time(), pkt=0, lm=0, ang=0, stale=0, bad=0, solve=0,
                   ik_ms=0.0, res=0.0, sim0=data.time, skips=0,
                   cpu=time.process_time(), loops=0)
        next_tick = time.time()
        while viewer.is_running():
            now = time.time()
            win["loops"] += 1
            # ---------------- 1. network: keep only the NEWEST packet -------
            newest_lm, newest_ang = None, None
            while True:
                try:
                    packet, _ = sock.recvfrom(65535)
                except (BlockingIOError, OSError):
                    break
                npkt += 1
                win["pkt"] += 1
                last_pkt_wall = now
                try:
                    msg = json.loads(packet.decode())
                except Exception:
                    win["bad"] += 1
                    continue
                if not isinstance(msg, dict):
                    win["bad"] += 1
                    continue
                if "world" in msg:
                    if newest_lm is not None:
                        win["stale"] += 1
                    newest_lm = msg
                    win["lm"] += 1
                else:
                    newest_ang = msg
                    win["ang"] += 1
            if synth is not None and now >= next_synth:
                W, _vis = synth.pose(now - wall0)
                newest_lm = {"world": W.tolist(), "hips_ok": True, "i": -1}
                next_synth = now + SYNTH_DT

            if newest_lm is not None:
                # RAW LANDMARK PACKET (pose_sender.py --landmarks, or the
                # stereo sender). Same 4-DOF IK as the offline driver.
                ang = newest_lm
                if not connected:
                    print("Receiving landmark packets - arms are live.", flush=True)
                    connected = True
                _t0 = time.perf_counter()
                raw_q = ik.targets(ang["world"],
                                   bool(ang.get("hips_ok", False)), ps)
                _pkt_ms = (time.perf_counter() - _t0) * 1e3
                win["solve"] += 1
                win["ik_ms"] += _pkt_ms
                win["res"] = max(win["res"], float(ik.last_res))
                use_q, _gstep, _nrej = ((raw_q, 0.0, 0) if gate is None
                                        else gate.filter(raw_q))
                if solves is not None:
                    solves.append({"t": now - wall0,
                                   "npkt": npkt,
                                   "src_i": int(ang.get("i", -1)),
                                   "hips_ok": bool(ang.get("hips_ok", False)),
                                   "iters": int(ik.last_iters),
                                   "res": float(ik.last_res),
                                   "ms": float(_pkt_ms),
                                   "gated": float(_nrej),
                                   "gstep": float(_gstep),
                                   **{k: float(raw_q.get(k, _np.nan))
                                      for k in sorted(target) if k[2:] in J4}})
                # One Euro ONCE PER PACKET (see the git history for why the old
                # per-step SMOOTH_ALPHA blend was a pass-through).
                for name, val in (use_q or {}).items():
                    if name in limits and _np.isfinite(val):
                        lo, hi = limits[name]
                        f = jfilt.get(name)
                        if f is None:
                            f = jfilt[name] = ps.OneEuro()
                        fv = float(f(val, now))
                        if not _np.isfinite(fv):          # never let NaN in
                            jfilt[name] = ps.OneEuro()
                            continue
                        target[name] = max(lo, min(hi, fv))
            elif newest_ang is not None:
                for key, val in newest_ang.items():
                    if key not in JOINT_MAP:
                        continue
                    try:
                        val = float(val)
                    except (TypeError, ValueError):
                        continue
                    if not _np.isfinite(val):
                        continue
                    name, scale, offset = JOINT_MAP[key]
                    lo, hi = limits[name]
                    target[name] = max(lo, min(hi, offset + val * scale))
                if not connected:
                    print("Receiving ANGLE packets. The v11 arms need landmarks: "
                          "restart the sender with --landmarks.", flush=True)
                    connected = True

            # ---------------- 2. physics, paced to the wall clock -----------
            wall_t = time.time() - wall0
            behind = wall_t - (data.time - sim_off)
            if behind > MAX_BEHIND:
                # Too far behind to catch up without a burst: skip ahead, so the
                # robot follows you NOW instead of replaying the past in slow-mo
                sim_off += behind - SYNC_DT
                win["skips"] += 1
            while data.time - sim_off < wall_t:
                for name in joints:
                    delta = max(-max_step, min(max_step, target[name] - cmd[name]))
                    cmd[name] += delta
                    data.ctrl[act[name]] = cmd[name]
                mujoco.mj_step(model, data)
                if rec is not None and data.time >= next_rec:
                    rec.append({"t": data.time, "npkt": npkt,
                                # in the SAME order as cmd/target below
                                "qpos": _np.array([data.qpos[qadr[n]] for n in joints]),
                                "cmd": _np.array([cmd[n] for n in joints]),
                                "target": _np.array([target[n] for n in joints])})
                    next_rec += 1.0 / 30.0
            if not _np.all(_np.isfinite(data.qpos)):
                print("!! physics went unstable (NaN) - resetting the robot", flush=True)
                mujoco.mj_resetData(model, data)
                sim_off = data.time - (time.time() - wall0)

            # ---------------- 3. viewer at ~60 Hz ----------------------------
            if now >= next_sync:
                viewer.sync()
                next_sync = now + SYNC_DT

            # ---------------- 4. status line ---------------------------------
            el = now - win["t"]
            if el >= 2.0:
                pps = win["pkt"] / el
                sps = win["solve"] / el
                ms = win["ik_ms"] / max(win["solve"], 1)
                rtf = (data.time - win["sim0"]) / el
                cpu = 100.0 * (time.process_time() - win["cpu"]) / el
                age = (now - last_pkt_wall) if last_pkt_wall else float("inf")
                print(f"STATUS pkts {pps:5.1f}/s (landmark {win['lm']/el:4.1f}, "
                      f"angle {win['ang']/el:4.1f}, stale-skipped {win['stale']}) | "
                      f"IK {sps:5.1f}/s {ms:4.1f} ms res {win['res']:.2f} | "
                      f"sim x{rtf:4.2f} | receiver CPU {cpu:3.0f}% "
                      f"({win['loops']/el:3.0f} loops/s) | newest pkt {age:5.2f}s old | "
                      f"L pitch {_np.degrees(cmd['L_shoulder_pitch']):5.0f} "
                      f"roll {_np.degrees(cmd['L_shoulder_roll']):5.0f} "
                      f"elbow {_np.degrees(cmd['L_elbow']):5.0f}", flush=True)
                if synth is None and win["pkt"] == 0:
                    print("   -> NO PACKETS. Is the sender running (python "
                          "\"live mimic system/pose_sender.py\" --landmarks)? "
                          "Does its window show a skeleton on you?", flush=True)
                elif win["ang"] and not win["lm"]:
                    print("   -> angle packets only: restart the sender WITH "
                          "--landmarks", flush=True)
                if win["bad"]:
                    print(f"   -> {win['bad']} unreadable packets", flush=True)
                if rtf < 0.8:
                    print("   -> the sim is running slower than real time "
                          "(close other heavy apps, or try --headless to test)",
                          flush=True)
                if win["res"] > 0.6:
                    print("   -> IK could not match some poses (high residual): "
                          "arms out of the robot's reach, or bad landmarks",
                          flush=True)
                if 0 < pps < 12:
                    print("   -> fewer than 12 packets/s: the CAMERA side is the "
                          "bottleneck - check the fps numbers in its window",
                          flush=True)
                win = dict(t=now, pkt=0, lm=0, ang=0, stale=0, bad=0, solve=0,
                           ik_ms=0.0, res=0.0, sim0=data.time, skips=0,
                           cpu=time.process_time(), loops=0)

            # ---------------- 5. sleep until the next tick OR a packet --------
            # The old loop slept at most 4 ms and often not at all, so it spun
            # ~1000x/s and held a whole CPU core next to the camera process.
            # Now it blocks in select(): wakes the moment a packet arrives (so
            # no added latency) or after TICK for the next physics chunk.
            next_tick = max(next_tick + TICK, time.time())
            wait = next_tick - time.time()
            if wait > 0:
                try:
                    select.select([sock], [], [], wait)
                except (OSError, ValueError):
                    time.sleep(wait)

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


class _OE:
    """One Euro filter on a vector, with its own (metric) tuning."""

    def __init__(self, min_cutoff, beta, d_cutoff=1.0):
        self.mc, self.beta, self.dc = min_cutoff, beta, d_cutoff
        self.x = self.dx = self.t = None

    @staticmethod
    def _a(cut, dt):
        return 1.0 / (1.0 + 1.0 / (2.0 * _np.pi * cut * dt))

    def __call__(self, x, t):
        if self.x is None:
            self.x, self.dx, self.t = x.copy(), _np.zeros_like(x), t
            return x
        dt = max(t - self.t, 1e-3)
        dx = (x - self.x) / dt
        self.dx = self.dx + self._a(self.dc, dt) * (dx - self.dx)
        cut = self.mc + self.beta * _np.abs(self.dx)
        self.x = self.x + self._a(cut, dt) * (x - self.x)
        self.t = t
        return self.x


class LandmarkFilter:
    """Smooths the arm/torso landmarks BEFORE the IK, depth harder than x/y.

    Smoothing joint angles AFTER the IK (which the receiver also does) cannot
    fix this: near the shoulder's gimbal point a few cm of depth noise flips
    the IK between two very different joint solutions, and filtering the
    angles only blends two wrong answers. Depth (MediaPipe world z, along the
    camera's line of sight) is the noisy axis from a single camera, so it gets
    the heavier filter; x/y stay responsive. One Euro: heavy smoothing when
    still, light when moving fast, so quick punches are not delayed much."""

    IDS = [0, 11, 12, 13, 14, 15, 16, 23, 24]

    def __init__(self):
        self.f = {i: (_OE(2.0, 3.0), _OE(1.0, 1.5)) for i in self.IDS}

    def __call__(self, W, t):
        W = W.copy()
        for i, (fxy, fz) in self.f.items():
            W[i, :2] = fxy(W[i, :2], t)
            W[i, 2:] = fz(W[i, 2:], t)
        return W


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
        self.lmf = LandmarkFilter()
        self.fore = {s: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{s}_forearm")
                     for s in "LR"}
        # GLOBAL SEED TABLE. Gradient IK only finds the nearest minimum, and
        # this shoulder has bad ones (at roll ~90 deg pitch and twist become
        # the same motion). So every solve also starts from the best match in
        # a precomputed grid of upper-arm directions over the WHOLE pitch x
        # roll travel - the solver can no longer get trapped far from the
        # answer, whatever happened on earlier frames.
        P = _np.radians(_np.arange(-60, 181, 10))
        Rl = _np.radians(_np.arange(-25, 151, 10))
        self.grid_q, self.grid_u = [], {"L": [], "R": []}
        for p_ in P:
            for r_ in Rl:
                self.grid_q.append([p_, r_, 0.0, 0.0])
        self.grid_q = _np.clip(_np.array(self.grid_q), self.lo, self.hi)
        for s_ in "LR":
            self.grid_u[s_] = _np.array([self.fk(s_, q)[0] for q in self.grid_q])
        self.last_iters = 0     # diagnostics only
        self.last_res = 0.0
        self.last_ms = 0.0

    def fk(self, side, q):
        mj = self.mj
        self.d.qpos[:] = self.qh
        for k, j in enumerate(J4):
            self.d.qpos[self.Q(f"{side}_{j}")] = q[k]
        # kinematics only: the IK needs body poses, not contacts or dynamics.
        # mj_forward ran collision detection on every probe - ~10x the cost,
        # and the solver makes ~100 probes per packet.
        mj.mj_kinematics(self.m, self.d)
        b = self.fore[side]
        ua = self.d.xpos[b] - SHP[side]
        fa = self.d.xmat[b].reshape(3, 3) @ _np.array([0.0, -1.0, 0.0])
        if side == "R":
            ua, fa = ua * MIR, fa * MIR
        return ua / (_np.linalg.norm(ua) + 1e-9), fa / (_np.linalg.norm(fa) + 1e-9)

    # Regularisation weights (see _lm). Units: residual is a unit-vector
    # difference, so 0.08 on continuity means "1 rad of jump costs as much as
    # a 0.08 pointing error".
    W_CONT = 0.08
    _wc = W_CONT
    W_TWIST_STRAIGHT = 0.6
    W_TWIST_MIN = 0.03
    NEUTRAL = _np.array([0.0, 0.35, 0.0, 0.6])

    W_ELBOW = 1.0

    def _lm(self, side, u, f, q0, prev, wt, wf, bend):
        """Damped least squares from seed q0. Residual = pointing error of the
        upper arm and forearm, PLUS two soft priors:
          * continuity: stay near the previous solution (prev);
          * twist -> 0, weighted by wt, which is large when the elbow is
            nearly straight.
        WHY: with a straight elbow the forearm points the same way as the upper
        arm, so humeral twist is invisible to the camera and the 3-DOF shoulder
        has a whole family of exact answers (e.g. roll 125 / pitch 117 /
        twist -38 points the arm the same way as roll 80 / pitch 0). Plain
        least squares wandered through that family on landmark noise and
        could park at a joint limit - on noisy single-camera data the right
        arm sat at roll 150 deg while the person's arm was at 80. The priors
        pick the natural member of the family and the one closest to where
        the arm already is.

        THE ELBOW. It is a 1-DOF hinge, so its angle IS the angle between the
        upper arm and forearm - measured directly (bend) and held there by a
        stiff prior. The forearm's DIRECTION only adds the orientation of the
        bend plane, and that is only observable in proportion to sin(bend):
        a 10 deg bend's plane is mostly landmark noise. So the forearm term is
        weighted by wf. Before this, a few degrees of noisy bend on a straight
        arm swung the shoulder through 100+ deg to "orient" a bend that did
        not exist."""
        q = q0.copy()
        nit = 0
        I4 = _np.eye(4)
        et = _np.array([[0.0, 0.0, wt, 0.0],
                        [0.0, 0.0, 0.0, self.W_ELBOW]])
        for _ in range(12):
            nit += 1
            u0, f0 = self.fk(side, q)
            res = _np.concatenate([u - u0, wf * (f - f0),
                                   self._wc * (prev - q),
                                   [wt * (0.0 - q[2]), self.W_ELBOW * (bend - q[3])]])
            if _np.linalg.norm(res) < 1e-4:
                break
            J = _np.zeros((6, 4))
            for k in range(4):
                qp = q.copy(); qp[k] += 1e-3
                u1, f1 = self.fk(side, qp)
                J[:, k] = _np.concatenate([(u1 - u0) / 1e-3,
                                           wf * (f1 - f0) / 1e-3])
            Ja = _np.vstack([J, self._wc * I4, et])
            # DAMPED least squares (Levenberg-Marquardt): an unreachable
            # target degrades to the closest reachable pose, smoothly.
            lam = 0.05 + 2.0 * float(_np.linalg.norm(res[:6])) ** 2
            dq = _np.linalg.solve(Ja.T @ Ja + lam * I4, Ja.T @ res)
            q = _np.clip(q + _np.clip(dq, -0.35, 0.35), self.lo, self.hi)
        u0, f0 = self.fk(side, q)
        geo = float(_np.linalg.norm(_np.concatenate([u - u0, wf * (f - f0)])))
        cost = (geo ** 2 + (self._wc ** 2) * float(_np.sum((q - prev) ** 2))
                + (wt * q[2]) ** 2 + (self.W_ELBOW * (bend - q[3])) ** 2)
        return q, geo, cost, nit

    def solve(self, side, u, f):
        _t = time.perf_counter()
        bend = float(_np.arccos(_np.clip(_np.dot(u, f), -1.0, 1.0)))
        straight = float(_np.clip((0.7 - bend) / 0.5, 0.0, 1.0))
        wt = self.W_TWIST_MIN + self.W_TWIST_STRAIGHT * straight
        wf = float(_np.clip(_np.sin(bend) / _np.sin(0.6), 0.1, 1.0))
        bend_c = float(_np.clip(bend, self.lo[3], self.hi[3]))
        prev = self.seed[side]
        # Near arm-straight-out-sideways the pitch axis lines up with the arm,
        # so pitch stops moving the arm and very different joint sets point
        # it almost the same way. There, stay where you are unless the data
        # clearly says otherwise: continuity weight rises with |u . left|.
        self._wc = self.W_CONT + 0.35 * abs(float(u[1])) ** 4
        q, geo, cost, nit = self._lm(side, u, f, prev, prev, wt, wf, bend_c)
        # Second start from the global table whenever the warm start is not
        # clearly right. Without it one bad frame could lock the solver onto a
        # wrong branch for good, because every later frame starts from it.
        if geo > 0.05:
            score = (_np.sum((self.grid_u[side] - u) ** 2, axis=1)
                     + (self._wc ** 2) * _np.sum((self.grid_q[:, :2] - prev[:2]) ** 2,
                                                    axis=1))
            g = self.grid_q[int(_np.argmin(score))].copy()
            g[2], g[3] = 0.0, bend_c
            q2, geo2, cost2, nit2 = self._lm(side, u, f, g, prev, wt, wf, bend_c)
            nit += nit2
            if cost2 < cost:
                q, geo, cost = q2, geo2, cost2
        self.seed[side] = q.copy()
        self.last_iters = nit
        self.last_res = geo
        self.last_ms = (time.perf_counter() - _t) * 1e3
        return q

    def targets(self, world, hips_ok, ps, t=None):
        """world: (33,3) RAW MediaPipe world landmarks, NOT axis-flipped."""
        W = _np.asarray(world, dtype=float)
        if W.shape != (33, 3) or not _np.all(_np.isfinite(W)):
            return {}           # a NaN here would pin a joint at its limit forever
        W = self.lmf(W, time.time() if t is None else t)
        fwd, left, up = ps.body_frame(W, hips_ok)
        if not all(_np.all(_np.isfinite(v)) for v in (fwd, left, up)):
            return {}
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
            if not _np.all(_np.isfinite(q)):
                self.seed[side] = _np.array([0.0, 0.35, 0.0, 0.6])
                continue
            for k, j in enumerate(J4):
                out[f"{side}_{j}"] = float(q[k])
        return out


if __name__ == "__main__":
    # NOTE: this must stay at the BOTTOM. It used to sit above ArmIK, so
    # running the file as a script hit ArmIK(model) before the class existed
    # and died with NameError - importing it hid the bug because main() never
    # ran on import.
    main()
