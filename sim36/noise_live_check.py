"""
noise_live_check.py - the same glitch test, but through the REAL live path:
a separate mimic_receiver_v11 process, real UDP packets, real wall clock.

The offline harness (noise_lib.py) imports ArmIK and StepGate directly, so it
proves the maths. This proves the WIRING - that the gate is actually consulted
in main()'s packet loop and that holding a joint does not stall the loop.

    python sim36/noise_live_check.py            # runs it twice, gate off/on

Prints, for each run, what the arm did around the injected glitches.
"""
import json
import os
import socket
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RX = os.path.join(ROOT, "live mimic system", "mimic_receiver_v11.py")
POSE = os.path.join(ROOT, "ref", "pose_seq.json")
ADDR = ("127.0.0.1", 5005)
K0, NFR = 400, 600                      # source frames to replay
DIR = np.array([0.6, -0.5, 0.6]); DIR = DIR / np.linalg.norm(DIR)
L_WR = 15
GLITCH = {60: 2, 150: 5, 240: 10}       # local frame -> length, 40 cm impulse
REAL = (400, 60)                        # local frame, length: a genuine ramp


def feed():
    raw = json.load(open(POSE))["raw"]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    for k in range(NFR):
        f = raw[K0 + k]
        W = np.array(f["world"], float)
        for g0, n in GLITCH.items():
            if g0 <= k < g0 + n:
                W[L_WR] = W[L_WR] + 0.40 * DIR
        if REAL[0] <= k < REAL[0] + REAL[1]:
            a = min(1.0, (k - REAL[0] + 1) / 6.0)
            W[L_WR] = W[L_WR] + a * 0.40 * DIR
        pkt = {"i": k, "world": [[round(float(c), 4) for c in p] for p in W],
               "hips_ok": bool(f["hips_ok"])}
        sock.sendto(json.dumps(pkt).encode(), ADDR)
        nxt = t0 + (k + 1) / 30.0
        time.sleep(max(0.0, nxt - time.time()))
    return time.time() - t0


def run(gate):
    dump = os.path.join(ROOT, "ref", f"live_glitch_{'gate' if gate else 'raw'}.npz")
    env = dict(os.environ, RUN_SECS="27")
    cmd = [sys.executable, RX, "--headless", "--dump", dump]
    if gate:
        cmd.append("--gate")
    p = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    time.sleep(3.0)
    el = feed()
    out = p.communicate(timeout=90)[0]
    print(f"  fed {NFR} frames in {el:.1f}s "
          f"({NFR/el:.1f} fps){'  [gate on]' if gate else ''}")
    for line in out.splitlines():
        if "gate ON" in line or "receiver dump" in line or "Receiving" in line:
            print("   " + line)
    return dump


def report(dump, label):
    D = np.load(dump)
    joints = [str(x) for x in D["joints"]]
    keys = [str(x) for x in D["solve_keys"]]
    ix = {k: i for i, k in enumerate(keys)}
    S, cmd, t = D["solves"], D["cmd"], D["t"]
    arm = [joints.index(j) for j in
           ("L_shoulder_pitch", "L_shoulder_roll", "L_shoulder_twist", "L_elbow")]
    ng = int(S[:, ix["gated"]].sum()) if "gated" in ix else 0
    print(f"\n{label}: {len(S)} packets solved, {ng} joint-frames held by the gate")
    # sim time of each source frame
    tof = {}
    for r in range(len(S)):
        tof.setdefault(int(S[r, ix["src_i"]]), float(S[r, ix["t"]]))
    for g0, n in sorted(GLITCH.items()):
        ta = tof.get(g0)
        if ta is None:
            continue
        w = (t >= ta - 0.1) & (t <= ta + 0.6)
        ref = cmd[np.argmin(np.abs(t - (ta - 0.05)))][arm]
        exc = np.degrees(np.abs(cmd[w][:, arm] - ref)).max() if w.any() else np.nan
        print(f"   {n:2d}-frame 40 cm impulse at t={ta:5.2f}s -> arm swung "
              f"{exc:5.1f} deg")
    ta = tof.get(REAL[0])
    if ta is not None:
        w = (t >= ta - 0.1) & (t <= ta + 2.0)
        ref = cmd[np.argmin(np.abs(t - (ta - 0.05)))][arm]
        exc = np.degrees(np.abs(cmd[w][:, arm] - ref)).max()
        print(f"   GENUINE 40 cm move at t={ta:5.2f}s -> arm followed "
              f"{exc:5.1f} deg  (must NOT be blocked)")
    return ng


if __name__ == "__main__":
    for gate in (False, True):
        print(f"\n=== live receiver, gate {'ON' if gate else 'OFF'} ===")
        d = run(gate)
        report(d, "gate ON " if gate else "gate OFF")
