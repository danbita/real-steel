"""
noise_experiments.py - how does the live arm respond to tracking glitches, and
is the MAX_RATE slew limit enough on its own?

Replays the cached ref_clip world landmarks through the real ArmIK + OneEuro +
slew + physics (sim36/noise_lib.py), injecting controlled glitches, with the
rejection gate OFF and ON.

    python sim36/noise_experiments.py            # everything
    python sim36/noise_experiments.py --quick    # one glitch onset instead of 3

Comparisons are always like-for-like: a gated glitch run is compared against a
gated CLEAN run from the same warm state, so what is measured is the response
to the glitch and not the gate's own effect on ordinary tracking. That effect
is measured separately, over the whole clip, in section (f).

Writes ref/noise_results.json.
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import noise_lib as nl

ROOT = nl.ROOT
QUICK = "--quick" in sys.argv
OUT = os.path.join(ROOT, "ref", "noise_results.json")
DEG = np.degrees
FDT = nl.FRAME_DT
PRE = 30                      # frames of warm-up in front of every window

W, HIPS, OKF = nl.load_clip()
N = len(W)
JOINTS = nl.Sim().joints
ARM = [JOINTS.index(j) for j in nl.ARM_JOINTS]
LARM = [JOINTS.index(j) for j in nl.ARM_JOINTS[:4]]

GSTEP, GHOLD, GMODE = np.radians(26.0), 3, "joint"     # recommended default


def gate(thr=None, hold=None, mode=None):
    return nl.mr.StepGate(GSTEP if thr is None else thr,
                          GHOLD if hold is None else hold,
                          GMODE if mode is None else mode)


# --------------------------------------------------------------------------
def full_run(g=None, mutate=None, drops=None, want_snaps=()):
    s = nl.Sim(gate=g)
    recs, snaps = [], {}
    for k in range(N):
        if k in want_snaps:
            snaps[k] = s.snapshot()
        Wk = W[k].copy()
        if mutate is not None:
            Wk = mutate(k, Wk)
        recs.append(s.step(Wk, bool(HIPS[k]),
                           drop=bool(drops[k]) if drops is not None else False))
    return recs, snaps


_WC = {}


def window(snaps, k0, k1, g=None, mutate=None, drops=None, key=None):
    """Warm-start at k0 and replay to k1. `key` memoises repeated windows."""
    if key is not None and key in _WC:
        return _WC[key]
    s = nl.Sim(gate=g)
    s.restore(snaps[k0])
    out = []
    for k in range(k0, k1):
        Wk = W[k].copy()
        if mutate is not None:
            Wk = mutate(k, Wk)
        out.append(s.step(Wk, bool(HIPS[k]),
                          drop=bool(drops[k]) if drops is not None else False))
    if key is not None:
        _WC[key] = out
    return out


def metrics(recs, base, k0, kg0, kg1):
    cmd, bcmd = nl.stack(recs, "cmd"), nl.stack(base, "cmd")
    tgt = nl.stack(recs, "target")
    wr = nl.fk_wrist(nl.stack(recs, "qpos"), JOINTS)
    bwr = nl.fk_wrist(nl.stack(base, "qpos"), JOINTS)
    perj = DEG(np.abs(cmd - bcmd))[:, ARM].max(axis=1)
    wrerr = np.linalg.norm(wr - bwr, axis=2).max(axis=1) * 100.0
    dem = DEG(np.abs(np.diff(tgt[:, ARM], axis=0))).max(axis=1) / FDT
    rate = DEG([r["peak_step"] for r in recs])
    gi1 = kg1 - k0
    rec_f = None
    for i in range(gi1, len(perj)):
        if perj[i:].max() < 2.0:
            rec_f = i - gi1
            break
    return {"peak_demand_degs": float(dem.max()),
            "peak_cmd_degs": float(rate.max()),
            "max_joint_dev_deg": float(perj.max()),
            "max_wrist_dev_cm": float(wrerr.max()),
            "frames_over_2deg": int((perj > 2.0).sum()),
            "frames_over_5deg": int((perj > 5.0).sum()),
            "recover_ms": (None if rec_f is None else float(rec_f * FDT * 1e3)),
            "ngated": int(sum(r["gated"] for r in recs))}


def rise_latency(cg, cn, j=None, fracs=(0.5, 0.9)):
    """When does the gated command reach x% of what the ungated one reached?"""
    st, fin = cn[PRE - 1], cn[-1]
    if j is None:
        j = int(np.argmax(np.abs(fin - st)))
    span = fin[j] - st[j]
    out = {}
    for f in fracs:
        tv, sg = st[j] + f * span, np.sign(span)
        ia = np.where(sg * (cn[:, j] - tv) >= 0)[0]
        ib = np.where(sg * (cg[:, j] - tv) >= 0)[0]
        ia, ib = ia[ia >= PRE], ib[ib >= PRE]
        out[f] = (None if not (len(ia) and len(ib))
                  else float((ib[0] - ia[0]) * FDT * 1e3))
    return out, j, float(DEG(abs(span)))


def ramp_mutate(k0, cm=40, ramp=4, lm=None):
    lm = nl.L_WR if lm is None else lm

    def f(k, Wk):
        if k >= k0:
            a = min(1.0, (k - k0 + 1) / ramp)
            Wk[lm] = Wk[lm] + a * (cm / 100.0) * nl.DIR
        return Wk
    return f


def pick_onsets(base):
    cmd = nl.stack(base, "cmd")[:, ARM]
    d = np.abs(np.diff(cmd, axis=0)).max(axis=1)
    act = np.array([d[max(0, k - 20):min(len(d), k + 20)].max() for k in range(N)])
    picked = []
    for k in sorted(range(200, N - 260), key=lambda k: act[k]):
        if all(abs(k - p) > 180 for p in picked):
            picked.append(k)
        if len(picked) == (1 if QUICK else 3):
            break
    return sorted(picked), act


def main():
    t0 = time.time()
    R = {}
    print("baseline, no gate ...", flush=True)
    base_all, _ = full_run()
    onsets, act = pick_onsets(base_all)
    print(f"  glitch onsets (calmest frames): {onsets}")

    # ---------------- (0) what the CLEAN clip already does -------------------
    raw = nl.stack(base_all, "raw")[:, ARM]
    st = np.abs(np.diff(raw, axis=0))
    st = st[np.isfinite(st).all(axis=1)]
    worst = DEG(st.max(axis=1))
    tgt = DEG(np.abs(np.diff(nl.stack(base_all, "target")[:, ARM], axis=0))).max(axis=1)
    cst = DEG(np.abs(np.diff(nl.stack(base_all, "cmd")[:, ARM], axis=0))).max(axis=1)
    budget = DEG(nl.mr.MAX_RATE * FDT)
    clean = {"nframes": N, "budget_deg_per_frame": float(budget),
             "raw_step_deg": {p: float(np.percentile(worst, p))
                              for p in (50, 90, 99, 99.9)},
             "raw_step_deg_max": float(worst.max()),
             "raw_over_20deg": int((worst > 20).sum()),
             "raw_over_26deg": int((worst > 26).sum()),
             "raw_over_40deg": int((worst > 40).sum()),
             "post_onEuro_step_deg_p99": float(np.percentile(tgt, 99)),
             "post_onEuro_step_deg_max": float(tgt.max()),
             "cmd_step_deg_p99": float(np.percentile(cst, 99)),
             "cmd_step_deg_max": float(cst.max()),
             "frames_slew_clipped": int((cst > 0.99 * budget).sum()),
             "mean_sat_frac": float(np.mean([r["sat_frac"] for r in base_all]))}
    R["clean"] = clean
    print(f"  slew budget = {budget:.2f} deg per 1/30 s frame")
    print("  RAW IK step, worst arm joint, per frame:  "
          + "  ".join(f"p{p} {clean['raw_step_deg'][p]:.1f}"
                      for p in (50, 90, 99, 99.9))
          + f"  max {clean['raw_step_deg_max']:.1f} deg")
    print(f"  frames whose RAW step exceeds 20/26/40 deg: "
          f"{clean['raw_over_20deg']}/{clean['raw_over_26deg']}/"
          f"{clean['raw_over_40deg']} of {N}   <- real tracking glitches, no "
          f"noise injected")
    print(f"  after OneEuro: p99 {clean['post_onEuro_step_deg_p99']:.1f} "
          f"max {clean['post_onEuro_step_deg_max']:.1f} deg/frame")
    print(f"  after slew:    p99 {clean['cmd_step_deg_p99']:.2f} "
          f"max {clean['cmd_step_deg_max']:.2f} deg/frame, clipped on "
          f"{clean['frames_slew_clipped']} frames ({100*clean['frames_slew_clipped']/N:.1f}%)")

    print("snapshots ...", flush=True)
    _, snaps = full_run(want_snaps={k - PRE for k in onsets})

    R["onsets"] = onsets
    R["cases"] = []

    def pair(k, k0, k1, mut, kg0, kg1, g=None, tag=""):
        gg = None if g is None else g
        b = window(snaps, k0, k1, g=gg,
                   key=("base", k0, k1, None if g is None
                        else (g.thr, g.max_hold, g.mode)))
        r = window(snaps, k0, k1, g=(None if g is None else
                                     gate(g.thr, g.max_hold, g.mode)), mutate=mut)
        m = metrics(r, b, k0, kg0, kg1)
        m["tag"] = tag
        return m

    def show(rows, label, extra=""):
        agg = {k: float(np.mean([r[k] for r in rows]))
               for k in ("peak_demand_degs", "peak_cmd_degs", "max_joint_dev_deg",
                         "max_wrist_dev_cm", "frames_over_2deg", "frames_over_5deg")}
        rv = [r["recover_ms"] for r in rows if r["recover_ms"] is not None]
        print(f"{label} | {agg['peak_demand_degs']:8.0f} {agg['peak_cmd_degs']:7.1f} | "
              f"{agg['max_joint_dev_deg']:7.1f} {agg['max_wrist_dev_cm']:7.1f} "
              f"{agg['frames_over_2deg']:6.1f} {agg['frames_over_5deg']:6.1f} "
              f"{(np.mean(rv) if rv else float('nan')):7.0f} {extra}", flush=True)
        return agg

    HDR = (f"{'landmark':>8} {'cm':>4} {'frm':>4} {'gate':>5} | {'demand':>8} "
           f"{'cmdrate':>7} | {'jointdev':>7} {'wristdev':>7} {'>2deg':>6} "
           f"{'>5deg':>6} {'recov':>7}")
    UNITS = ("                                      deg/s    deg/s  |    deg      "
             "cm   frames frames     ms")

    # ---------------- (a) IMPULSE -------------------------------------------
    print("\n(a) IMPULSE - one landmark displaced for n frames, then snaps back")
    print(HDR); print(UNITS)
    for lmname, lm in (("wrist", nl.L_WR), ("elbow", nl.L_EL)):
        for cm in (10, 20, 40):
            for dur in (1, 2, 5, 10):
                for g in (None, gate()):
                    rows = [pair(k, k - PRE, k + 150,
                                 nl.impulse(lm, cm / 100.0, k, dur),
                                 k, k + dur, g=g, tag="impulse")
                            for k in onsets]
                    for r, k in zip(rows, onsets):
                        r.update({"tag": "impulse", "lm": lmname, "cm": cm,
                                  "dur": dur, "onset": k,
                                  "gate": None if g is None else
                                  [float(DEG(g.thr)), g.max_hold, g.mode]})
                    R["cases"] += rows
                    show(rows, f"{lmname:>8} {cm:>4} {dur:>4} "
                               f"{('ON' if g else 'off'):>5}")

    # ---------------- (b) SUSTAINED -----------------------------------------
    print("\n(b) SUSTAINED - displaced for 30/60 frames (the arm really did move)"
          "\n    dev/recovery are vs the CLEAN run, so they SHOULD be large: the "
          "robot is supposed to follow.\n    what matters is that ON and off agree"
          " - the gate must not block it.")
    print(HDR); print(UNITS)
    for cm in (20, 40):
        for dur in (30, 60):
            for g in (None, gate()):
                rows = [pair(k, k - PRE, k + dur + 120,
                             nl.impulse(nl.L_WR, cm / 100.0, k, dur),
                             k, k + dur, g=g, tag="sustained")
                        for k in onsets]
                for r, k in zip(rows, onsets):
                    r.update({"tag": "sustained", "cm": cm, "dur": dur, "onset": k,
                              "gate": None if g is None else
                              [float(DEG(g.thr)), g.max_hold, g.mode]})
                R["cases"] += rows
                show(rows, f"{'wrist':>8} {cm:>4} {dur:>4} "
                           f"{('ON' if g else 'off'):>5}")

    # ---- latency the gate adds to a sustained (teleport) and a ramped move --
    print("\n(b2) LATENCY the gate adds to a REAL move  (gated vs ungated, same "
          "injected move)")
    print(f"{'move':>16} {'span_deg':>9} {'lat50_ms':>9} {'lat90_ms':>9} "
          f"{'final_err_deg':>13} {'gated_jf':>9}")
    lat = []
    for name, mk, kg in (("teleport 40cm", lambda k: nl.impulse(nl.L_WR, .40, k, 60), 60),
                         ("ramp 2f  40cm", lambda k: ramp_mutate(k, 40, 2), 60),
                         ("ramp 4f  40cm", lambda k: ramp_mutate(k, 40, 4), 60),
                         ("ramp 8f  40cm", lambda k: ramp_mutate(k, 40, 8), 60),
                         ("ramp 15f 40cm", lambda k: ramp_mutate(k, 40, 15), 60)):
        rows = []
        for k in onsets:
            k0, k1 = k - PRE, k + 150
            mut = mk(k)
            rn = window(snaps, k0, k1, mutate=mut, key=("mv", name, k))
            rg = window(snaps, k0, k1, g=gate(), mutate=mut)
            cn, cg = nl.stack(rn, "cmd")[:, LARM], nl.stack(rg, "cmd")[:, LARM]
            l, j, span = rise_latency(cg, cn)
            rows.append({"tag": "latency", "move": name, "onset": k,
                         "span_deg": span, "lat50_ms": l[0.5], "lat90_ms": l[0.9],
                         "final_err_deg": float(DEG(np.abs(cg[-1] - cn[-1])).max()),
                         "gated_jointframes": int(sum(r["gated"] for r in rg))})
        lat += rows
        f = lambda key: np.mean([r[key] for r in rows if r[key] is not None])
        print(f"{name:>16} {f('span_deg'):9.1f} {f('lat50_ms'):9.1f} "
              f"{f('lat90_ms'):9.1f} {f('final_err_deg'):13.2f} "
              f"{f('gated_jointframes'):9.1f}", flush=True)
    R["latency"] = lat

    # ---------------- (c) GAUSSIAN ------------------------------------------
    print("\n(c) GAUSSIAN noise on all six arm landmarks, whole 60 s clip")
    print(f"{'sigma_cm':>8} {'gate':>5} | {'demand_p99':>10} {'jdev_rms':>9} "
          f"{'jdev_max':>9} {'wdev_rms_cm':>11} {'wdev_max_cm':>11} "
          f"{'reversals':>9} {'clip%':>6} {'gated_jf':>8}")
    bcmd = nl.stack(base_all, "cmd")
    bwr = nl.fk_wrist(bcmd, JOINTS)
    brev = int(np.sum(np.diff(np.sign(np.diff(bcmd[:, ARM], axis=0)), axis=0) != 0))
    print(f"{0.0:8.1f} {'-':>5} | {'-':>10} {0.0:9.2f} {0.0:9.2f} {0.0:11.2f} "
          f"{0.0:11.2f} {brev:9d} "
          f"{100*clean['frames_slew_clipped']/N:6.1f} {0:8d}")
    for sig in (0.01, 0.02, 0.05):
        for g in (None, gate()):
            # gate-matched reference: the same gate on CLEAN landmarks
            if g is None:
                ref = base_all
            else:
                if "gclean" not in _WC:
                    _WC["gclean"] = full_run(g=gate())[0]
                ref = _WC["gclean"]
            r, _ = full_run(g=(None if g is None else gate()),
                            mutate=nl.gaussian(sig, seed=7))
            cmd = nl.stack(r, "cmd")
            rcmd = nl.stack(ref, "cmd")
            wr, rwr = nl.fk_wrist(cmd, JOINTS), nl.fk_wrist(rcmd, JOINTS)
            dev = DEG(np.abs(cmd - rcmd))[:, ARM].max(axis=1)
            wdev = np.linalg.norm(wr - rwr, axis=2).max(axis=1) * 100
            dem = DEG(np.abs(np.diff(nl.stack(r, "target")[:, ARM], axis=0))).max(axis=1) / FDT
            cs = DEG(np.abs(np.diff(cmd[:, ARM], axis=0))).max(axis=1)
            row = {"tag": "gaussian", "sigma_cm": sig * 100, "gate_on": bool(g),
                   "demand_p99": float(np.percentile(dem, 99)),
                   "jdev_rms": float(np.sqrt((dev ** 2).mean())),
                   "jdev_max": float(dev.max()),
                   "wdev_rms_cm": float(np.sqrt((wdev ** 2).mean())),
                   "wdev_max_cm": float(wdev.max()),
                   "reversals": int(np.sum(np.diff(np.sign(np.diff(cmd[:, ARM], axis=0)), axis=0) != 0)),
                   "clip_pct": float(100 * (cs > 0.99 * budget).sum() / N),
                   "gated_jf": int(sum(x["gated"] for x in r))}
            R["cases"].append(row)
            print(f"{sig*100:8.1f} {('ON' if g else 'off'):>5} | {row['demand_p99']:10.0f} "
                  f"{row['jdev_rms']:9.2f} {row['jdev_max']:9.2f} "
                  f"{row['wdev_rms_cm']:11.2f} {row['wdev_max_cm']:11.2f} "
                  f"{row['reversals']:9d} {row['clip_pct']:6.1f} "
                  f"{row['gated_jf']:8d}", flush=True)

    # ---------------- (d) DROPOUT -------------------------------------------
    print("\n(d) DROPOUT - no packets at all for n frames")
    print(f"{'nframes':>7} {'ms':>6} | {'jointdev':>8} {'wristdev_cm':>11} "
          f"{'catchup_ms':>10}")
    for nd in (5, 15, 30, 60):
        rows = []
        for k in onsets:
            k0, k1 = k - PRE, k + nd + 120
            dr = np.zeros(N, bool); dr[k:k + nd] = True
            b = window(snaps, k0, k1, key=("base", k0, k1, None))
            r = window(snaps, k0, k1, drops=dr)
            m = metrics(r, b, k0, k, k + nd)
            m.update({"tag": "dropout", "dur": nd, "onset": k})
            rows.append(m)
        R["cases"] += rows
        f = lambda key: np.mean([x[key] for x in rows if x[key] is not None])
        print(f"{nd:7d} {nd*FDT*1e3:6.0f} | {f('max_joint_dev_deg'):8.2f} "
              f"{f('max_wrist_dev_cm'):11.2f} {f('recover_ms'):10.0f}", flush=True)

    # ---------------- (e) GATE SWEEP ----------------------------------------
    print("\n(e) GATE SWEEP - suppression of a 2-frame 40 cm impulse vs the "
          "latency added to a genuine 4-frame ramp")
    print(f"{'thr_deg':>7} {'hold':>5} {'mode':>6} | {'imp_jdev':>8} {'imp_wdev':>8} "
          f"{'imp>2deg':>8} | {'lat50':>6} {'lat90':>6} {'ramp_err':>8} | "
          f"{'clean_frm_gated':>15}")
    sweep = []
    for thr_deg in (12, 17, 22, 26, 32, 40):
        for hold in (1, 2, 3, 5):
            for mode in ("joint", "all"):
                thr = np.radians(thr_deg)
                ij, iw, i2, l5, l9, fe = [], [], [], [], [], []
                for k in onsets:
                    k0, k1 = k - PRE, k + 150
                    b = window(snaps, k0, k1, g=gate(thr, hold, mode),
                               key=("base", k0, k1, (thr, hold, mode)))
                    r = window(snaps, k0, k1, g=gate(thr, hold, mode),
                               mutate=nl.impulse(nl.L_WR, 0.40, k, 2))
                    m = metrics(r, b, k0, k, k + 2)
                    ij.append(m["max_joint_dev_deg"]); iw.append(m["max_wrist_dev_cm"])
                    i2.append(m["frames_over_2deg"])
                    mut = ramp_mutate(k, 40, 4)
                    rn = window(snaps, k0, k1, mutate=mut,
                                key=("mv", "ramp 4f  40cm", k))
                    rg = window(snaps, k0, k1, g=gate(thr, hold, mode), mutate=mut)
                    cn, cg = nl.stack(rn, "cmd")[:, LARM], nl.stack(rg, "cmd")[:, LARM]
                    l, j, span = rise_latency(cg, cn)
                    if l[0.5] is not None:
                        l5.append(l[0.5])
                    if l[0.9] is not None:
                        l9.append(l[0.9])
                    fe.append(float(DEG(np.abs(cg[-1] - cn[-1])).max()))
                # how often does this gate fire on the CLEAN clip?
                g2 = gate(thr, hold, mode)
                nfrm = 0
                for row in nl.stack(base_all, "raw"):
                    q = {n: row[i] for i, n in enumerate(JOINTS) if np.isfinite(row[i])}
                    if not q:
                        continue
                    _, _, nr = g2.filter(q)
                    nfrm += 1 if nr else 0
                e = {"tag": "sweep", "thr_deg": thr_deg, "hold": hold, "mode": mode,
                     "imp_jdev": float(np.mean(ij)), "imp_wdev": float(np.mean(iw)),
                     "imp_over2": float(np.mean(i2)),
                     "lat50_ms": float(np.mean(l5)) if l5 else None,
                     "lat90_ms": float(np.mean(l9)) if l9 else None,
                     "ramp_final_err_deg": float(np.mean(fe)),
                     "clean_frames_gated": nfrm,
                     "clean_pct": 100.0 * nfrm / N}
                sweep.append(e)
                print(f"{thr_deg:7d} {hold:5d} {mode:>6} | {e['imp_jdev']:8.2f} "
                      f"{e['imp_wdev']:8.2f} {e['imp_over2']:8.1f} | "
                      f"{(e['lat50_ms'] if e['lat50_ms'] is not None else np.nan):6.0f} "
                      f"{(e['lat90_ms'] if e['lat90_ms'] is not None else np.nan):6.0f} "
                      f"{e['ramp_final_err_deg']:8.2f} | {nfrm:6d} "
                      f"({e['clean_pct']:4.1f}%)", flush=True)
    R["sweep"] = sweep

    # ---------------- (f) cost of the gate on the REAL clip ------------------
    print("\n(f) COST OF THE GATE ON THE UNMODIFIED CLIP (no injected noise)")
    print(f"{'thr_deg':>7} {'hold':>5} {'mode':>6} | {'jdev_rms':>8} {'jdev_max':>8} "
          f"{'wdev_rms_cm':>11} {'wdev_max_cm':>11} {'frames_gated':>12} "
          f"{'forced':>7}")
    costs = []
    for thr_deg, hold, mode in ((17, 3, "joint"), (22, 3, "joint"), (26, 3, "joint"),
                                (26, 2, "joint"), (26, 5, "joint"), (32, 3, "joint"),
                                (26, 3, "all")):
        g2 = gate(np.radians(thr_deg), hold, mode)
        r, _ = full_run(g=g2)
        cmd = nl.stack(r, "cmd")
        wr = nl.fk_wrist(cmd, JOINTS)
        dev = DEG(np.abs(cmd - bcmd))[:, ARM].max(axis=1)
        wdev = np.linalg.norm(wr - bwr, axis=2).max(axis=1) * 100
        c = {"tag": "cost", "thr_deg": thr_deg, "hold": hold, "mode": mode,
             "jdev_rms": float(np.sqrt((dev ** 2).mean())),
             "jdev_max": float(dev.max()),
             "wdev_rms_cm": float(np.sqrt((wdev ** 2).mean())),
             "wdev_max_cm": float(wdev.max()),
             "frames_gated": int(g2.nfrm), "joint_frames_gated": int(g2.nrej),
             "forced_accepts": int(g2.nforce)}
        costs.append(c)
        print(f"{thr_deg:7d} {hold:5d} {mode:>6} | {c['jdev_rms']:8.2f} "
              f"{c['jdev_max']:8.2f} {c['wdev_rms_cm']:11.2f} {c['wdev_max_cm']:11.2f} "
              f"{c['frames_gated']:12d} {c['forced_accepts']:7d}", flush=True)
    R["cost"] = costs

    json.dump(R, open(OUT, "w"), indent=1, default=float)
    print(f"\nwrote {OUT}   ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
