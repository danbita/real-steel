"""noise_summary.py - compact answers pulled out of ref/noise_results.json."""
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R = json.load(open(os.path.join(ROOT, "ref", "noise_results.json")))
C = R["cases"]


def sel(**kw):
    out = []
    for c in C:
        if all(c.get(k) == v for k, v in kw.items()):
            out.append(c)
    return out


def m(rows, k):
    v = [r[k] for r in rows if r.get(k) is not None]
    return float(np.mean(v)) if v else float("nan")


print("=== Q1  does the 3.0 rad/s slew alone contain a short impulse? ===")
print("     (no gate; deviation from the clean run, mean over 3 onsets)")
print(f"{'glitch':>22} {'joint travel':>13} {'wrist':>8} {'visible':>9} {'back within':>12}")
print(f"{'':>22} {'deg':>13} {'cm':>8} {'frames':>9} {'2 deg, ms':>12}")
for lm in ("wrist", "elbow"):
    for cm in (10, 20, 40):
        for dur in (1, 2, 5, 10):
            rows = [r for r in sel(tag="impulse", lm=lm, cm=cm, dur=dur)
                    if r.get("gate") is None]
            if not rows:
                continue
            print(f"{lm+' '+str(cm)+'cm '+str(dur)+'f':>22} "
                  f"{m(rows,'max_joint_dev_deg'):13.1f} {m(rows,'max_wrist_dev_cm'):8.1f} "
                  f"{m(rows,'frames_over_2deg'):9.1f} {m(rows,'recover_ms'):12.0f}")

print("\n=== Q2  gate ON vs OFF, same glitches ===")
print(f"{'glitch':>22} {'joint deg off/ON':>18} {'wrist cm off/ON':>18} "
      f"{'visible frames':>16}")
for lm in ("wrist", "elbow"):
    for cm in (10, 20, 40):
        for dur in (1, 2, 5, 10):
            a = [r for r in sel(tag="impulse", lm=lm, cm=cm, dur=dur) if r.get("gate") is None]
            b = [r for r in sel(tag="impulse", lm=lm, cm=cm, dur=dur) if r.get("gate")]
            if not a or not b:
                continue
            print(f"{lm+' '+str(cm)+'cm '+str(dur)+'f':>22} "
                  f"{m(a,'max_joint_dev_deg'):8.1f} {m(b,'max_joint_dev_deg'):8.1f}  "
                  f"{m(a,'max_wrist_dev_cm'):8.1f} {m(b,'max_wrist_dev_cm'):8.1f}  "
                  f"{m(a,'frames_over_2deg'):7.1f} {m(b,'frames_over_2deg'):7.1f}")

print("\n=== Q3  latency the gate adds to a REAL move ===")
for name in sorted({r["move"] for r in R["latency"]}):
    rows = [r for r in R["latency"] if r["move"] == name]
    print(f"{name:>16}  span {m(rows,'span_deg'):5.1f} deg   "
          f"lat50 {m(rows,'lat50_ms'):5.1f} ms   lat90 {m(rows,'lat90_ms'):5.1f} ms   "
          f"final error {m(rows,'final_err_deg'):.2f} deg")

print("\n=== Q4  gate sweep, best trade-offs ===")
S = R["sweep"]
S2 = [s for s in S if s["lat90_ms"] is not None]
S2.sort(key=lambda s: (s["imp_jdev"], s["lat90_ms"]))
print(f"{'thr':>4} {'hold':>5} {'mode':>6} {'imp_jdev':>9} {'imp_wdev':>9} "
      f"{'lat50':>6} {'lat90':>6} {'clean%':>7}")
for s in S2[:14]:
    print(f"{s['thr_deg']:4d} {s['hold']:5d} {s['mode']:>6} {s['imp_jdev']:9.2f} "
          f"{s['imp_wdev']:9.2f} {s['lat50_ms']:6.0f} {s['lat90_ms']:6.0f} "
          f"{s['clean_pct']:7.1f}")

print("\n=== Q5  cost on the untouched clip ===")
for c in R["cost"]:
    print(f"thr {c['thr_deg']:3d} hold {c['hold']} {c['mode']:>6}: "
          f"jdev rms {c['jdev_rms']:5.2f} max {c['jdev_max']:6.2f} deg   "
          f"wrist rms {c['wdev_rms_cm']:4.2f} max {c['wdev_max_cm']:5.2f} cm   "
          f"frames gated {c['frames_gated']:4d}  forced {c['forced_accepts']:3d}")

print("\n=== Q6  gaussian noise ===")
for c in sel(tag="gaussian"):
    print(f"sigma {c['sigma_cm']:4.1f} cm gate {'ON ' if c['gate_on'] else 'off'}: "
          f"jdev rms {c['jdev_rms']:5.2f} max {c['jdev_max']:5.2f} deg  "
          f"wrist rms {c['wdev_rms_cm']:4.2f} max {c['wdev_max_cm']:5.2f} cm  "
          f"reversals {c['reversals']:5d}  slew-clipped {c['clip_pct']:4.1f}%")

print("\n=== Q7  dropout ===")
for d in sorted({c["dur"] for c in sel(tag="dropout")}):
    rows = sel(tag="dropout", dur=d)
    print(f"{d:3d} frames ({d/30*1000:4.0f} ms) no packets: joint {m(rows,'max_joint_dev_deg'):5.2f} deg, "
          f"wrist {m(rows,'max_wrist_dev_cm'):5.2f} cm, catch-up {m(rows,'recover_ms'):4.0f} ms")

print("\n=== Q8  sustained (the arm really did move): gate must not block ===")
for cm in (20, 40):
    for dur in (30, 60):
        a = [r for r in sel(tag="sustained", cm=cm, dur=dur) if r.get("gate") is None]
        b = [r for r in sel(tag="sustained", cm=cm, dur=dur) if r.get("gate")]
        print(f"{cm} cm for {dur} frames: followed {m(a,'max_joint_dev_deg'):5.1f} deg "
              f"(no gate) vs {m(b,'max_joint_dev_deg'):5.1f} deg (gate)")
