"""
live_diag_render.py - render the robot exactly as the LIVE receiver drove it.

    python sim36/live_diag_render.py

Reads ref/live_dump.npz (written by
`mimic_receiver_v11.py --headless --dump ref/live_dump.npz`) and
ref/live_sender_log.npz (written by `pose_sender.py --log ...`).

The joint angles are NOT recomputed. The dump stores data.qpos sampled every
1/30 s of sim time during the live run, so this script only sets qpos and
renders: what you see is the live ArmIK solve plus the live SMOOTH_ALPHA /
MAX_RATE loop plus the live physics, not an offline re-derivation.

Alignment: every UDP packet carries the source frame index it came from, so a
source frame is matched to the sim state one video frame after its packet
landed. Frames with no packet hold the previous robot pose, which is what the
receiver itself does.

The dump's qpos/cmd/target columns are labelled by its "joints" array (sorted
NAME order), so they are written back into MuJoCo through
model.jnt_qposadr[mj_name2id(...)] per name - never positionally.

Writes ref/simframes_live/%05d.png and ref/robot_skel_live.npz.
"""
import os
import sys

import numpy as np
import mujoco
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE = os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml")
DUMP = os.path.join(ROOT, "ref", "live_dump.npz")
SLOG = os.path.join(ROOT, "ref", "live_sender_log.npz")
FRAMEDIR = os.path.join(ROOT, "ref", "simframes_live")
SKEL_OUT = os.path.join(ROOT, "ref", "robot_skel_live.npz")
W = H = 720
LAG = 1.0 / 30.0          # render the state one video frame after the packet

# forearm-tip offset and shoulder/hip centres, same numbers the v11 driver uses
WRIST_OFF = np.array([0.0, -0.1335, 0.0131])
HEAD = np.array([0.0, 0.0, 0.855])
NECK = np.array([0.0, 0.0, 0.748])
SH = {"L": np.array([0.0, 0.1184, 0.748]), "R": np.array([0.0, -0.1184, 0.748])}
HIP = {"L": np.array([0.0, 0.055, 0.4846]), "R": np.array([0.0, -0.055, 0.4846])}


def main():
    D = np.load(DUMP)
    S = np.load(SLOG)
    keys = [str(k) for k in D["solve_keys"]]
    idx = {k: i for i, k in enumerate(keys)}
    SV = D["solves"]
    rec_t, qpos = D["t"], D["qpos"]
    # The dump's qpos columns are labelled by D["joints"], which the receiver
    # builds with sorted(), i.e. NAME order - NOT model order. Replaying it
    # with d.qpos[:] = row silently reindexes every joint (the model happens to
    # have nq == 14 too, so nothing raises): R_shoulder_pitch landed on
    # R_hip_flex and the right LEG swung across the body while the arm sat
    # still. Always resolve columns BY NAME.
    dump_joints = ([str(x) for x in D["joints"]] if "joints" in D.files
                   else None)

    nsrc = int(S["i"].max()) + 1
    # source frame -> sim tick
    tick = np.full(nsrc, -1, dtype=int)
    for r in range(len(SV)):
        k = int(SV[r, idx["src_i"]])
        if 0 <= k < nsrc:
            tick[k] = int(np.argmin(np.abs(rec_t - (SV[r, idx["t"]] + LAG))))
    # frames before the first packet sit at the rest pose; later gaps hold
    last = 0
    for k in range(nsrc):
        if tick[k] < 0:
            tick[k] = last
        last = tick[k]
    print(f"{nsrc} source frames -> sim ticks "
          f"({np.sum(tick == 0)} holding the rest pose before the stream started)")

    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)

    if dump_joints is None:
        # legacy dump: no labels, columns were raw data.qpos in model order
        if qpos.shape[1] != m.nq:
            raise SystemExit(f"unlabelled dump has {qpos.shape[1]} columns but "
                             f"the model has nq={m.nq}")
        cols = np.arange(m.nq)
        adr = np.arange(m.nq)
        print("dump has no 'joints' array - assuming model qpos order")
    else:
        adr, cols = [], []
        for c, name in enumerate(dump_joints):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise SystemExit(f"dump joint '{name}' is not in {SCENE}")
            adr.append(int(m.jnt_qposadr[jid]))
            cols.append(c)
        adr, cols = np.array(adr), np.array(cols)
        perm = [dump_joints[c] for c in cols]
        model_order = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
                       for i in range(m.njnt)]
        print(f"mapping {len(cols)} dump columns onto qpos BY NAME")
        if perm != model_order:
            print(f"  dump  order: {perm}")
            print(f"  model order: {model_order}")
            print("  (they differ - positional assignment would scramble joints)")
    os.makedirs(FRAMEDIR, exist_ok=True)
    r = mujoco.Renderer(m, height=H, width=W)
    skel = []
    for k in range(nsrc):
        d.qpos[:] = 0.0
        d.qpos[adr] = qpos[tick[k]][cols]
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        row = [HEAD, NECK]
        for s in "LR":
            eb = B(f"{s}_forearm")
            elbow = d.xpos[eb].copy()
            wrist = elbow + d.xmat[eb].reshape(3, 3) @ WRIST_OFF
            row += [SH[s], elbow, wrist]
        row += [HIP["L"], HIP["R"]]
        skel.append(np.array(row))
        r.update_scene(d, camera="front")
        Image.fromarray(r.render()).save(os.path.join(FRAMEDIR, f"{k:05d}.png"))
    r.close()
    np.savez_compressed(SKEL_OUT, pts=np.array(skel))
    print(f"wrote {nsrc} frames to {FRAMEDIR} and {SKEL_OUT}")


if __name__ == "__main__":
    main()
