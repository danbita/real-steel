"""
noise_video.py - sim/out36/noise_test.mp4

Three robots driven from the SAME cached landmarks of ref/ref_clip.mp4:

    CLEAN            what the live path does today on the untouched landmarks
    GLITCHED         the same, with scripted tracking glitches injected
    GLITCHED + GATE  the same glitches, with the StepGate turned on

Everything below the panels is a scrolling trace of one joint command so the
lunges (and their absence) are visible as well as watchable.

    python sim36/noise_video.py [--frames 1050] [--thr 26] [--hold 3]
"""
import os
import sys

import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import noise_lib as nl

ROOT = nl.ROOT
OUT = os.path.join(ROOT, "sim", "out36", "noise_test.mp4")
FRAMEDIR = os.path.join(ROOT, "ref", "simframes_noise")


def argf(flag, d):
    return type(d)(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else d


NF = argf("--frames", 1050)
THR = np.radians(argf("--thr", 26.0))
HOLD = argf("--hold", 3)
PW = 600                      # robot panel size
PLOT_H = 230
HEAD_H = 64
CW = PW * 3
CH = HEAD_H + PW + PLOT_H

# ------------------------------------------------------------------ script
# (start, nframes, kind, params, label)
EVENTS = [
    (90,  1,  "impulse", dict(lm=nl.L_WR, m=0.20), "1 frame  wrist +20 cm"),
    (180, 2,  "impulse", dict(lm=nl.L_WR, m=0.20), "2 frames wrist +20 cm"),
    (270, 2,  "impulse", dict(lm=nl.L_WR, m=0.40), "2 frames wrist +40 cm"),
    (360, 5,  "impulse", dict(lm=nl.L_WR, m=0.40), "5 frames wrist +40 cm"),
    (450, 10, "impulse", dict(lm=nl.L_WR, m=0.40), "10 frames wrist +40 cm"),
    (540, 15, "dropout", dict(), "15 frames NO PACKETS"),
    (630, 60, "gauss",   dict(sigma=0.02), "60 frames gaussian 2 cm on all arm lm"),
    (750, 45, "sustain", dict(lm=nl.L_WR, m=0.40), "45 frames +40 cm  (a REAL move)"),
    (870, 45, "ramp",    dict(lm=nl.L_WR, m=0.40, ramp=6),
     "REAL move: 40 cm ramped over 6 frames, held"),
]


def build_script():
    rng = np.random.default_rng(11)
    mut = [None] * NF
    drops = np.zeros(NF, bool)
    label = [""] * NF
    active = [False] * NF
    kind_of = [""] * NF
    for k0, n, kind, p, txt in EVENTS:
        for k in range(k0, min(NF, k0 + n)):
            active[k] = True
            label[k] = txt
            kind_of[k] = kind
            if kind == "dropout":
                drops[k] = True
            elif kind == "gauss":
                mut[k] = ("gauss", rng.normal(0, p["sigma"], (len(nl.ARM_LM), 3)))
            elif kind == "ramp":
                a = min(1.0, (k - k0 + 1) / p["ramp"])
                mut[k] = ("shift", p["lm"], a * p["m"] * nl.DIR)
            else:
                mut[k] = ("shift", p["lm"], p["m"] * nl.DIR)
        if kind == "ramp":                      # hold the new pose afterwards
            for k in range(k0 + n, min(NF, k0 + n + 40)):
                mut[k] = ("shift", p["lm"], p["m"] * nl.DIR)
                label[k] = txt
                active[k] = True
                kind_of[k] = kind
        # let the label linger so it is readable
        for k in range(k0 + n, min(NF, k0 + n + 25)):
            if not label[k]:
                label[k] = txt
    return mut, drops, label, active, kind_of


def mutator(mut):
    def f(k, Wk):
        m = mut[k] if k < len(mut) else None
        if m is None:
            return Wk
        if m[0] == "gauss":
            Wk[nl.ARM_LM] = Wk[nl.ARM_LM] + m[1]
        else:
            Wk[m[1]] = Wk[m[1]] + m[2]
        return Wk
    return f


def font(sz, bold=False):
    for n in (("segoeuib.ttf", "arialbd.ttf") if bold else ("segoeui.ttf", "arial.ttf")):
        try:
            return ImageFont.truetype(n, sz)
        except OSError:
            pass
    return ImageFont.load_default()


F_T, F_S, F_XS = font(30, True), font(21), font(17)


def main():
    W, HIPS, _ = nl.load_clip()
    mut, drops, label, active, kind_of = build_script()
    mf = mutator(mut)

    runs = {}
    print("simulating three robots ...", flush=True)
    for name, g, m, dr in (("CLEAN", None, None, None),
                           ("GLITCHED", None, mf, drops),
                           ("GLITCHED + GATE", nl.mr.StepGate(THR, HOLD, "joint"),
                            mf, drops)):
        s = nl.Sim(gate=g)
        recs = []
        for k in range(NF):
            Wk = W[k].copy()
            if m is not None:
                Wk = m(k, Wk)
            recs.append(s.step(Wk, bool(HIPS[k]),
                               drop=bool(dr[k]) if dr is not None else False))
        runs[name] = recs
        print(f"  {name:16s} done"
              + (f"   joint-frames held: {g.nrej}" if g else ""), flush=True)

    order = ["CLEAN", "GLITCHED", "GLITCHED + GATE"]
    joints = nl.Sim().joints
    qp = {n: nl.stack(runs[n], "qpos") for n in order}
    cmd = {n: nl.stack(runs[n], "cmd") for n in order}
    arm = [joints.index(j) for j in nl.ARM_JOINTS]
    # trace the joint that the glitches move the most
    dev = np.abs(cmd["GLITCHED"] - cmd["CLEAN"])[:, arm].max(axis=0)
    jsel = arm[int(np.argmax(dev))]
    jname = joints[jsel]
    print(f"  tracing {jname} (largest glitch response, "
          f"{np.degrees(dev.max()):.1f} deg)")
    tr = {n: np.degrees(cmd[n][:, jsel]) for n in order}
    lo = min(t.min() for t in tr.values()) - 3
    hi = max(t.max() for t in tr.values()) + 3
    dev_c = {n: np.degrees(np.abs(cmd[n] - cmd["CLEAN"])[:, arm]).max(axis=1)
             for n in order}

    m = mujoco.MjModel.from_xml_path(nl.SCENE)
    d = mujoco.MjData(m)
    adr = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
           for n in joints]
    r = mujoco.Renderer(m, height=PW, width=PW)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    os.makedirs(FRAMEDIR, exist_ok=True)
    wr = imageio.get_writer(OUT, fps=30, quality=8, macro_block_size=1)

    COL = {"CLEAN": (110, 220, 140), "GLITCHED": (250, 110, 100),
           "GLITCHED + GATE": (105, 175, 255)}
    px_per_f = CW / float(NF)

    def yof(v):
        return HEAD_H + PW + PLOT_H - 30 - (v - lo) / (hi - lo) * (PLOT_H - 52)

    print("rendering ...", flush=True)
    for k in range(NF):
        sheet = Image.new("RGB", (CW, CH), (11, 14, 20))
        dr_ = ImageDraw.Draw(sheet, "RGBA")
        for i, name in enumerate(order):
            d.qpos[:] = 0.0
            d.qpos[adr] = qp[name][k]
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            r.update_scene(d, camera="front")
            im = Image.fromarray(r.render())
            sheet.paste(im, (i * PW, HEAD_H))
            x0 = i * PW
            dr_.rectangle([x0, HEAD_H, x0 + PW, HEAD_H + 40], fill=(8, 10, 16, 205))
            dr_.text((x0 + 14, HEAD_H + 7), name, font=F_S, fill=COL[name])
            if i:
                e = dev_c[name][k]
                dr_.text((x0 + PW - 210, HEAD_H + 7),
                         f"off by {e:5.1f} deg", font=F_S,
                         fill=(255, 210, 120) if e > 5 else (170, 180, 195))
            if name == "GLITCHED + GATE" and runs[name][k]["gated"]:
                dr_.rectangle([x0 + 6, HEAD_H + 46, x0 + 200, HEAD_H + 78],
                              fill=(20, 60, 120, 230))
                dr_.text((x0 + 14, HEAD_H + 50),
                         f"HOLDING {runs[name][k]['gated']} joint(s)",
                         font=F_XS, fill=(160, 210, 255))
            if name == "GLITCHED" and active[k]:
                real = kind_of[k] in ("sustain", "ramp")
                dr_.rectangle([x0 + 6, HEAD_H + 46, x0 + 250, HEAD_H + 78],
                              fill=(20, 80, 40, 230) if real else (110, 25, 25, 230))
                dr_.text((x0 + 14, HEAD_H + 50),
                         "REAL MOVE INJECTED" if real else "GLITCH INJECTED",
                         font=F_XS,
                         fill=(180, 255, 200) if real else (255, 190, 185))
            dr_.line([(x0, HEAD_H), (x0, HEAD_H + PW)], fill=(40, 46, 58), width=2)

        # header
        dr_.rectangle([0, 0, CW, HEAD_H], fill=(8, 10, 16))
        dr_.text((18, 8), "TRACKING-GLITCH TEST   same clip, same IK, same "
                 "3.0 rad/s slew - only the landmarks differ", font=F_T,
                 fill=(238, 242, 248))
        dr_.text((18, 40), f"t = {k/30:5.2f} s   frame {k:4d}"
                 f"     gate: hold a joint whose demanded step exceeds "
                 f"{np.degrees(THR):.0f} deg/frame, for at most {HOLD} frames",
                 font=F_XS, fill=(150, 165, 185))
        if label[k]:
            dr_.text((CW - 700, 34), "injected: " + label[k], font=F_S,
                     fill=(255, 165, 120) if active[k] else (110, 120, 135))

        # plot
        py0 = HEAD_H + PW
        dr_.rectangle([0, py0, CW, CH], fill=(15, 18, 26))
        for k0, n, kind, p, txt in EVENTS:
            n2 = n + (40 if kind == "ramp" else 0)
            xa, xb = k0 * px_per_f, (k0 + n2) * px_per_f
            dr_.rectangle([xa, py0 + 22, max(xb, xa + 2), CH - 22],
                          fill=((25, 80, 40, 120) if kind in ("sustain", "ramp")
                                else (90, 30, 30, 120)))
        for name, wdt in (("GLITCHED + GATE", 5), ("CLEAN", 3), ("GLITCHED", 2)):
            pts = [(i * px_per_f, yof(tr[name][i])) for i in range(0, min(k + 1, NF))]
            if len(pts) > 1:
                dr_.line(pts, fill=COL[name] + (245,), width=wdt)
        dr_.line([(k * px_per_f, py0 + 20), (k * px_per_f, CH - 20)],
                 fill=(255, 255, 255, 150), width=2)
        dr_.text((12, py0 + 4), f"{jname} command, deg    red band = glitch "
                 f"injected     green band = a REAL move injected",
                 font=F_XS, fill=(165, 178, 196))
        for i, name in enumerate(order):
            dr_.text((1230 + i * 190, py0 + 4), name, font=F_XS, fill=COL[name])
        arr = np.asarray(sheet)
        wr.append_data(arr)
        if k % 30 == 0:
            sheet.save(os.path.join(FRAMEDIR, f"{k:05d}.png"))
            print(f"   frame {k}/{NF}", flush=True)
    wr.close()
    r.close()
    np.savez_compressed(os.path.join(ROOT, "ref", "noise_video_traces.npz"),
                        joints=np.array(joints),
                        **{f"cmd_{i}": cmd[n] for i, n in enumerate(order)},
                        events=np.array([[e[0], e[1]] for e in EVENTS]))
    print(f"wrote {OUT}  ({NF} frames, {NF/30:.1f}s)")


if __name__ == "__main__":
    main()
