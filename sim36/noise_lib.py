"""
noise_lib.py - replay CACHED world landmarks through the REAL live pipeline
(mimic_receiver_v11.ArmIK + OneEuro + MAX_RATE slew + position actuators +
mj_step) with controlled glitches injected into the landmarks.

No UDP, no MediaPipe. Everything that touches the joint command is imported
from mimic_receiver_v11 / pose_sender, so what this measures is the shipping
code path, not a re-implementation.

Landmark source: ref/pose_seq.json["raw"] - the cached MediaPipe world
landmarks for ref/ref_clip.mp4 (1801 frames @ 30 fps, the calm talking clip).
"""
import importlib.util
import json
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE = os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml")
POSE = os.path.join(ROOT, "ref", "pose_seq.json")
FRAME_DT = 1.0 / 30.0

# import the receiver WITHOUT letting our own argv leak into its flag parsing
_argv = sys.argv[:]
sys.argv = [sys.argv[0]]
_spec = importlib.util.spec_from_file_location(
    "mimic_receiver_v11",
    os.path.join(ROOT, "live mimic system", "mimic_receiver_v11.py"))
mr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mr)
sys.argv = _argv
ps = mr.ps

# MediaPipe landmark ids
L_SH, L_EL, L_WR = 11, 13, 15
R_SH, R_EL, R_WR = 12, 14, 16
ARM_LM = [11, 12, 13, 14, 15, 16]
ARM_JOINTS = ["L_shoulder_pitch", "L_shoulder_roll", "L_shoulder_twist", "L_elbow",
              "R_shoulder_pitch", "R_shoulder_roll", "R_shoulder_twist", "R_elbow"]
WRIST_OFF = np.array([0.0, -0.1335, 0.0131])   # forearm-frame wrist tip


def load_clip():
    d = json.load(open(POSE))
    raw = d["raw"]
    W = np.array([r["world"] for r in raw], float)          # (N,33,3)
    hips = np.array([bool(r["hips_ok"]) for r in raw])
    ok = np.array([bool(r["ok"]) for r in raw])
    return W, hips, ok


class Sim:
    """One instance of the live receiver's control loop, driven frame by frame."""

    def __init__(self, gate=None):
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.joints = sorted({t for t, _, _ in mr.JOINT_MAP.values()} | set(mr.PARKED))
        self.act, self.limits, self.qadr = {}, {}, {}
        for n in self.joints:
            aid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_act")
            jid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)
            self.act[n] = aid
            self.qadr[n] = int(self.m.jnt_qposadr[jid])
            self.limits[n] = (tuple(self.m.jnt_range[jid]) if self.m.jnt_limited[jid]
                              else (-3.2, 3.2))
        self.ik = mr.ArmIK(self.m)
        self.target = {n: 0.0 for n in self.joints}
        self.target.update(mr.PARKED)
        for s in "LR":
            self.target[f"{s}_shoulder_roll"] = mr.REST_ROLL
        self.cmd = dict(self.target)
        self.jfilt = {}
        self.dt = float(self.m.opt.timestep)
        self.max_step = mr.MAX_RATE * self.dt
        self.gate = gate
        self.k = 0                      # frame index (drives the OneEuro clock)
        self.jidx = {n: i for i, n in enumerate(self.joints)}
        self._sub = 0.0

    # ---- state snapshot so experiments can warm-start from a common point ----
    def snapshot(self):
        return {
            "qpos": self.d.qpos.copy(), "qvel": self.d.qvel.copy(),
            "act": self.d.act.copy() if self.d.act.size else None,
            "ctrl": self.d.ctrl.copy(), "time": float(self.d.time),
            "target": dict(self.target), "cmd": dict(self.cmd), "k": self.k,
            "seed": {s: v.copy() for s, v in self.ik.seed.items()},
            "filt": {n: (f.x_prev, f.dx_prev, f.t_prev) for n, f in self.jfilt.items()},
            "sub": self._sub,
            "gate": (None if self.gate is None else self.gate.state()),
        }

    def restore(self, s):
        self.d.qpos[:] = s["qpos"]; self.d.qvel[:] = s["qvel"]
        if s["act"] is not None and self.d.act.size:
            self.d.act[:] = s["act"]
        self.d.ctrl[:] = s["ctrl"]; self.d.time = s["time"]
        self.target = dict(s["target"]); self.cmd = dict(s["cmd"]); self.k = s["k"]
        self.ik.seed = {a: b.copy() for a, b in s["seed"].items()}
        self.jfilt = {}
        for n, (x, dx, t) in s["filt"].items():
            f = ps.OneEuro(); f.x_prev, f.dx_prev, f.t_prev = x, dx, t
            self.jfilt[n] = f
        self._sub = s["sub"]
        if self.gate is not None:
            self.gate.load(s["gate"])
        mujoco.mj_forward(self.m, self.d)

    # ---------------------------- one 1/30 s frame ---------------------------
    def step(self, world, hips_ok, drop=False):
        """world: (33,3) landmarks for this frame. drop=True -> no packet."""
        rec = {"gated": 0, "step": 0.0, "iters": 0, "res": 0.0}
        if not drop:
            raw_q = self.ik.targets(world, hips_ok, ps)
            rec["iters"] = self.ik.last_iters
            rec["res"] = self.ik.last_res
            use_q = raw_q
            if self.gate is not None:
                use_q, rec["step"], nrej = self.gate.filter(raw_q)
                rec["gated"] = nrej
            if use_q:
                tnow = self.k * FRAME_DT
                for name, val in use_q.items():
                    if name in self.limits:
                        lo, hi = self.limits[name]
                        f = self.jfilt.get(name)
                        if f is None:
                            f = self.jfilt[name] = ps.OneEuro()
                        self.target[name] = max(lo, min(hi, float(f(val, tnow))))
            rec["raw"] = np.array([raw_q.get(n, np.nan) for n in self.joints])
        else:
            rec["raw"] = np.full(len(self.joints), np.nan)
        # physics: same substep count the live loop gets between 30 Hz packets
        self._sub += FRAME_DT
        nstep = int(round(self._sub / self.dt))
        self._sub -= nstep * self.dt
        peak, nsat = 0.0, 0
        for _ in range(nstep):
            sat = False
            for name in self.joints:
                delta = max(-self.max_step,
                            min(self.max_step, self.target[name] - self.cmd[name]))
                self.cmd[name] += delta
                self.d.ctrl[self.act[name]] = self.cmd[name]
                if abs(delta) >= self.max_step - 1e-12:
                    sat = True
                peak = max(peak, abs(delta))
            nsat += sat
            mujoco.mj_step(self.m, self.d)
        rec["peak_step"] = peak / self.dt                  # rad/s, commanded
        rec["sat_frac"] = nsat / max(nstep, 1)  # share of substeps at the limit
        rec["nstep"] = nstep
        rec["target"] = np.array([self.target[n] for n in self.joints])
        rec["cmd"] = np.array([self.cmd[n] for n in self.joints])
        rec["qpos"] = np.array([self.d.qpos[self.qadr[n]] for n in self.joints])
        self.k += 1
        return rec


def run(world, hips, gate=None, drops=None, warm=None, i0=0, i1=None,
        mutate=None, sim=None):
    """Replay frames [i0,i1). mutate(k, W)->W is the glitch injector."""
    s = sim if sim is not None else Sim(gate=gate)
    if warm is not None:
        s.restore(warm)
    i1 = len(world) if i1 is None else i1
    out = []
    for k in range(i0, i1):
        W = world[k].copy()
        if mutate is not None:
            W = mutate(k, W)
        drop = bool(drops[k]) if drops is not None else False
        out.append(s.step(W, bool(hips[k]), drop=drop))
    return s, out


def stack(recs, key):
    return np.array([r[key] for r in recs])


_FKC = {}


def fk_wrist(qpos_rows, joints):
    """(N,14) joint rows -> (N,2,3) L/R wrist-tip positions in robot world."""
    if "m" not in _FKC:
        _FKC["m"] = mujoco.MjModel.from_xml_path(SCENE)
        _FKC["d"] = mujoco.MjData(_FKC["m"])
    m, d = _FKC["m"], _FKC["d"]
    adr = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
           for n in joints]
    bod = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_forearm")
           for s in "LR"}
    out = np.zeros((len(qpos_rows), 2, 3))
    for i, row in enumerate(qpos_rows):
        d.qpos[:] = 0.0
        for a, v in zip(adr, row):
            d.qpos[a] = v
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        for j, s in enumerate("LR"):
            b = bod[s]
            out[i, j] = d.xpos[b] + d.xmat[b].reshape(3, 3) @ WRIST_OFF
    return out


# --------------------------------------------------------------------------
# glitch injectors
# --------------------------------------------------------------------------
DIR = np.array([0.6, -0.5, 0.6])
DIR = DIR / np.linalg.norm(DIR)


def impulse(lm, meters, k0, nframes, direction=DIR):
    """Displace landmark `lm` (and nothing else) for frames [k0, k0+n)."""
    def f(k, W):
        if k0 <= k < k0 + nframes:
            W[lm] = W[lm] + meters * direction
        return W
    return f


def gaussian(sigma, seed=0, lms=ARM_LM):
    rng = np.random.default_rng(seed)

    def f(k, W):
        W[lms] = W[lms] + rng.normal(0.0, sigma, size=(len(lms), 3))
        return W
    return f
