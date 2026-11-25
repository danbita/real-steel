"""
replay_dump.py - play back a receiver recording as robot motion.

    python sim36/replay_dump.py runs/20261120-123833.npz             # MuJoCo window, loops
    python sim36/replay_dump.py runs/20261120-123833.npz --mp4 out.mp4  # write a video instead

Works on the operator console's Record files and on mimic_receiver_v11.py
--dump files (same format). Nothing is recomputed: the recording holds the
robot's actual joint angles (qpos) every 1/30 s of sim time, and this just
poses the model with them. Columns are mapped BY NAME through the dump's
"joints" array (it is sorted, not model order - see live_diag_render.py).

Timing follows sim time. If the sim ran slower than real time while
recording, the replay looks correspondingly shorter than what you lived
through.
"""
import os
import sys
import time

import numpy as np
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml")
FPS = 30.0
SIZE = 720


def load(path, model):
    d = np.load(path)
    if "joints" not in d.files:
        raise SystemExit(f"{path}: no 'joints' array - not a v11 receiver recording")
    names = [str(j) for j in d["joints"]]
    adr = []
    for n in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            raise SystemExit(f"{path}: joint '{n}' is not in the scene")
        adr.append(int(model.jnt_qposadr[jid]))
    qpos = d["qpos"]
    if len(qpos) == 0:
        raise SystemExit(f"{path}: recording is empty")
    return np.array(adr), qpos, d["t"]


def pose(model, data, adr, row):
    data.qpos[:] = 0.0
    data.qpos[adr] = row
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def write_mp4(model, data, adr, qpos, out):
    import cv2
    r = mujoco.Renderer(model, height=SIZE, width=SIZE)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (SIZE, SIZE))
    for row in qpos:
        pose(model, data, adr, row)
        r.update_scene(data, camera="front")
        w.write(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
    w.release()
    r.close()
    print(f"wrote {len(qpos)} frames ({len(qpos) / FPS:.1f} s) -> {out}")


def play(model, data, adr, qpos):
    import mujoco.viewer
    print(f"playing {len(qpos)} frames ({len(qpos) / FPS:.1f} s), looping. "
          "Close the window to quit.")
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                      show_right_ui=False) as v:
        k, t0 = 0, time.time()
        while v.is_running():
            pose(model, data, adr, qpos[k])
            v.sync()
            k = (k + 1) % len(qpos)
            if k == 0:
                time.sleep(0.5)                 # short pause between loops
                t0 = time.time()
            else:
                time.sleep(max(0.0, t0 + k / FPS - time.time()))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        raise SystemExit(__doc__)
    model = mujoco.MjModel.from_xml_path(SCENE)
    data = mujoco.MjData(model)
    adr, qpos, t = load(args[0], model)
    print(f"{args[0]}: {len(qpos)} frames, sim time {t[0]:.2f}-{t[-1]:.2f} s")
    if "--mp4" in sys.argv:
        write_mp4(model, data, adr, qpos, sys.argv[sys.argv.index("--mp4") + 1])
    else:
        play(model, data, adr, qpos)


if __name__ == "__main__":
    main()
