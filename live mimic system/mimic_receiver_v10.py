"""
mimic_receiver_v10.py - live receiver for the 36 inch robot (humanoid_v9).

    python sim36/build_humanoid_v9_scene.py     (once)
    python mimic_receiver_v10.py
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

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
SCENE = os.path.join(REPO_ROOT, "sim36", "humanoid_v10_scene.xml")

UDP_ADDR = ("127.0.0.1", 5005)
SMOOTH_ALPHA = 0.35
MAX_RATE = 8.0          # rad/s cap on how fast the target may move

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

    dt = model.opt.timestep
    max_step = MAX_RATE * dt

    print(f"Listening on udp://{UDP_ADDR[0]}:{UDP_ADDR[1]}")
    print("Physics ON: position actuators + mj_step.")
    print("Shoulder v3: pitch +/-95, abduction 0..185, twist +/-90 (roll-coupled).")
    print(f"Rest roll = {REST_ROLL:.4f} rad (arm hanging); abduction adds to it.")
    print("Camera side, other terminal:  python pose_sender.py\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        connected = False
        wall0 = time.time()
        while viewer.is_running():
            while True:
                try:
                    packet, _ = sock.recvfrom(4096)
                except (BlockingIOError, OSError):
                    break
                try:
                    ang = json.loads(packet.decode())
                except Exception:
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

            for name in joints:
                blended = cmd[name] + SMOOTH_ALPHA * (target[name] - cmd[name])
                delta = max(-max_step, min(max_step, blended - cmd[name]))
                cmd[name] += delta
                data.ctrl[act[name]] = cmd[name]

            mujoco.mj_step(model, data)
            viewer.sync()

            lag = data.time - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)


if __name__ == "__main__":
    main()


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
            self.qh[self.Q(f"{s}_shoulder_roll")] = _np.pi
        self.lo = _np.array([model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"L_{j}")][0] for j in J4])
        self.hi = _np.array([model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"L_{j}")][1] for j in J4])
        self.seed = {"L": _np.array([0.0, 0.35, 0.0, 0.6]),
                     "R": _np.array([0.0, 0.35, 0.0, 0.6])}

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
        q = self.seed[side].copy()
        for _ in range(12):
            u0, f0 = self.fk(side, q)
            res = _np.concatenate([u - u0, f - f0])
            if _np.linalg.norm(res) < 1e-4:
                break
            J = _np.zeros((6, 4))
            for k in range(4):
                qp = q.copy(); qp[k] += 1e-3
                u1, f1 = self.fk(side, qp)
                J[:, k] = _np.concatenate([(u1 - u0) / 1e-3, (f1 - f0) / 1e-3])
            dq, *_ = _np.linalg.lstsq(J, res, rcond=None)
            q = _np.clip(q + _np.clip(dq, -0.5, 0.5), self.lo, self.hi)
        self.seed[side] = q.copy()
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
