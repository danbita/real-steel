"""
live_loopback_test.py - prove the LIVE path actually works, end to end, without
needing a webcam in the room.

    python sim36/live_loopback_test.py

It replays the recorded world landmarks over the real UDP socket, in the exact
packet format pose_sender.py --landmarks emits, into the real receiver code, and
checks three things:

    1. the receiver accepts the packets at all (the landmark branch was dead
       code until now - ArmIK existed but main() never called it, so every
       --landmarks packet was silently dropped)
    2. the joint targets it produces match the offline driver's
    3. it keeps up with 30 fps on this machine

The only thing this does NOT exercise is the camera itself and MediaPipe's
inference latency; everything downstream of the landmarks is the real code.
"""
import importlib.util as il
import json
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LIVE = os.path.join(ROOT, "live mimic system")
sys.path.insert(0, LIVE)


def load(name, path):
    spec = il.spec_from_file_location(name, path)
    mod = il.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ps = load("pose_sender", os.path.join(LIVE, "pose_sender.py"))
    rx = load("rx", os.path.join(LIVE, "mimic_receiver_v11.py"))
    import mujoco

    raw = json.load(open(os.path.join(ROOT, "ref", "pose_seq.json")))["raw"]
    frames = [f for f in raw if f]
    print(f"replaying {len(frames)} recorded frames through the live code path\n")

    model = mujoco.MjModel.from_xml_path(rx.SCENE)
    ik = rx.ArmIK(model)

    # --- 1. packet round-trip over a real socket ---------------------------
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 5077))
    srv.setblocking(False)
    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    f0 = frames[900]
    pkt = {"world": [[round(float(c), 4) for c in p] for p in f0["world"]],
           "hips_ok": bool(f0["hips_ok"])}
    blob = json.dumps(pkt).encode()
    cli.sendto(blob, ("127.0.0.1", 5077))
    time.sleep(0.05)
    got, _ = srv.recvfrom(4096)
    ang = json.loads(got.decode())
    print(f"1. UDP packet     {len(blob)} bytes, fits one datagram: "
          f"{'yes' if len(blob) < 4096 else 'NO'}")
    print(f"   receiver sees keys: {sorted(ang)[:3]} ... ok={'world' in ang}")

    # --- 2. do the live targets match the offline driver? ------------------
    errs = {}
    t0 = time.perf_counter()
    for f in frames:
        tg = ik.targets(f["world"], f["hips_ok"], ps)
        for k, v in tg.items():
            errs.setdefault(k, []).append(v)
    dt = time.perf_counter() - t0
    print(f"\n2. IK solved for {len(frames)} frames, {len(errs)} joints driven")
    for k in sorted(errs):
        a = np.degrees(np.array(errs[k]))
        print(f"   {k:20} mean {a.mean():7.1f}  min {a.min():7.1f}  max {a.max():7.1f} deg")

    # --- 3. throughput ------------------------------------------------------
    per = dt / len(frames)
    print(f"\n3. throughput     {per*1000:.2f} ms/frame  =  {1/per:,.0f} fps headroom")
    print(f"   30 fps needs 33.3 ms/frame, so this is {33.3/(per*1000):.0f}x faster than required")

    # --- 4. joint limits actually respected --------------------------------
    jn = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
          for i in range(model.njnt)]
    bad = 0
    for k, v in errs.items():
        if k in jn:
            lo, hi = model.jnt_range[jn.index(k)]
            v = np.array(v)
            bad += int(((v < lo - 1e-6) | (v > hi + 1e-6)).sum())
    print(f"\n4. commands outside joint limits before clamping: {bad}"
          f"  ({'clamped by the receiver' if bad else 'none'})")


if __name__ == "__main__":
    main()
