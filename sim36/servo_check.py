"""
servo_check.py - does the simulated robot still move the way it should on the
servos in sim36/servo_spec.py?

    python sim36/servo_check.py                  # checks the current scene
    python sim36/servo_check.py --vs OTHER.xml   # A/B against another scene

Three checks, no camera or MediaPipe needed:

  1. TRAVEL   every servo-driven joint range fits in the servo's 240 deg.
  2. HOLDING  worst-case static gravity torque on every arm joint, swept over
              its whole range (other joints at their worst too), against the
              servo's stall torque and the ~1/3-of-stall continuous budget.
              Also re-run with the elbow servo's mass hung at the elbow, since
              the ATOM arm meshes do not include it yet.
  3. TRACKING drives both arms through randomised "boxing" waypoints, slew-
              limited at MAX_RATE exactly like the receivers, through mj_step,
              and reports how closely the joints follow and how often the
              actuators hit their torque ceiling.
"""
import itertools
import math
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from servo_spec import SHOULDER, ELBOW, MAX_RATE  # noqa: E402

SCENE = os.path.join(HERE, "humanoid_v11_scene.xml")
ARM_J = ["shoulder_pitch", "shoulder_roll", "shoulder_twist", "elbow"]
G = 9.81


def servo_for(j):
    return ELBOW if j.endswith("elbow") else SHOULDER


def jid(m, n):
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)


def travel(m):
    ok = True
    print("1. TRAVEL (joint range vs 240 deg servo)")
    for s in "LR":
        for j in ARM_J:
            lo, hi = m.jnt_range[jid(m, f"{s}_{j}")]
            span = math.degrees(hi - lo)
            cap = math.degrees(servo_for(j).travel)
            fits = span <= cap + 0.01                 # 3.1416 rad rounds to 240.0003
            ok &= fits
            tag = ("EXCEEDS SERVO" if not fits else
                   "fits - but uses ALL of it, zero margin" if span > cap - 1.0 else "fits")
            if s == "L":
                print(f"   {j:15} {math.degrees(lo):7.1f} .. {math.degrees(hi):6.1f}"
                      f"  = {span:5.1f} deg   {tag}")
    return ok


def holding(m, extra_elbow_mass=0.0):
    """max |gravity torque| per joint over a grid of the arm's whole range."""
    d = mujoco.MjData(m)
    grids = []
    for j in ARM_J:
        lo, hi = m.jnt_range[jid(m, f"L_{j}")]
        grids.append(np.linspace(lo, hi, 9))
    worst = {j: 0.0 for j in ARM_J}
    for q in itertools.product(*grids):
        d.qpos[:] = 0
        for j, v in zip(ARM_J, q):
            d.qpos[m.jnt_qposadr[jid(m, f"L_{j}")]] = v
        mujoco.mj_forward(m, d)
        el_anchor = d.xanchor[jid(m, "L_elbow")].copy()
        for j in ARM_J:
            k = jid(m, f"L_{j}")
            tau = d.qfrc_bias[m.jnt_dofadr[k]]
            if extra_elbow_mass and not j.endswith("elbow"):
                r = el_anchor - d.xanchor[k]
                tau += np.dot(d.xaxis[k], np.cross(r, [0, 0, -extra_elbow_mass * G])) * -1
            worst[j] = max(worst[j], abs(tau))
    return worst


# Waypoint envelope: what the live retarget actually asks for (boxing guard,
# punches, arms down), not the raw joint limits - random targets across the
# full limits mostly drive the bicep into the chest, and then the test only
# measures contact, not the servo. Degrees, applied as an intersection with
# the joint's own range.
ENVELOPE = {"shoulder_pitch": (-30, 120), "shoulder_roll": (60, 150),
            "shoulder_twist": (-40, 35), "elbow": (0, 120)}


def tracking(m, seconds=60.0, seed=7):
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    names = [f"{s}_{j}" for s in "LR" for j in ARM_J]
    act = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_act") for n in names}
    rng_lim = {n: m.jnt_range[jid(m, n)] for n in names}
    rest = {n: (math.pi if n.endswith("shoulder_roll") else 0.0) for n in names}
    for n in names:
        lo, hi = rng_lim[n]
        rest[n] = min(max(rest[n], lo), hi)
        d.qpos[m.jnt_qposadr[jid(m, n)]] = rest[n]
        d.ctrl[act[n]] = rest[n]
    mujoco.mj_forward(m, d)
    cur = dict(rest)
    tgt = dict(rest)
    sub = max(1, int(round((1 / 30.0) / m.opt.timestep)))
    step = MAX_RATE / 30.0
    errs, sat, tot, peak = [], 0, 0, {n: 0.0 for n in names}
    contact_frames = 0
    # bodies moved by an arm joint (feet resting on the floor are not ours)
    arm_body = np.zeros(m.nbody, bool)
    for n in names:
        arm_body[m.jnt_bodyid[jid(m, n)]] = True
    for b in range(m.nbody):
        p = b
        while p and not arm_body[p]:
            p = m.body_parentid[p]
        arm_body[b] = arm_body[p] if p else False
    probe = mujoco.MjData(m)

    def arm_contact(dd):
        return any((arm_body[m.geom_bodyid[dd.contact[i].geom1]] or
                    arm_body[m.geom_bodyid[dd.contact[i].geom2]])
                   and dd.contact[i].dist < 0 for i in range(dd.ncon))
    frames = int(seconds * 30)
    for f in range(frames):
        if f % 12 == 0:                       # new waypoint every 0.4 s
            for _ in range(200):              # only poses the arms can reach
                for n in names:               # without hitting the body
                    lo, hi = rng_lim[n]
                    elo, ehi = (math.radians(v) for v in ENVELOPE[n[2:]])
                    tgt[n] = rng.uniform(max(lo, elo), min(hi, ehi))
                probe.qpos[:] = d.qpos
                for n in names:
                    probe.qpos[m.jnt_qposadr[jid(m, n)]] = tgt[n]
                mujoco.mj_forward(m, probe)
                if not arm_contact(probe):
                    break
        for n in names:
            cur[n] += max(-step, min(step, tgt[n] - cur[n]))
            d.ctrl[act[n]] = cur[n]
        for _ in range(sub):
            mujoco.mj_step(m, d)
        if f > 30:
            if arm_contact(d):                # arm touching body: not a servo test
                contact_frames += 1
                continue
            for n in names:
                errs.append(abs(d.qpos[m.jnt_qposadr[jid(m, n)]] - cur[n]))
                fr = m.actuator_forcerange[act[n]][1]
                a = abs(d.actuator_force[act[n]])
                peak[n] = max(peak[n], a)
                sat += a >= 0.999 * fr
                tot += 1
    e = np.degrees(np.array(errs))
    if e.size == 0:
        # every frame had an arm part inside something: that is a COLLISION
        # problem in the scene, not a servo one - say so instead of crashing
        print("   !! the arms were in contact on EVERY frame - check the scene's")
        print("      collision geometry (e.g. parts welded to the torso overlapping")
        print("      the shoulder modules) before trusting any tracking number")
        return dict(mean=float("nan"), p95=float("nan"), max=float("nan"),
                    sat=0.0, peak=peak, contact=1.0)
    return dict(mean=e.mean(), p95=np.percentile(e, 95), max=e.max(),
                sat=sat / tot, peak=peak, contact=contact_frames / (frames - 31))


def report(path, label):
    m = mujoco.MjModel.from_xml_path(path)
    print(f"\n===== {label}: {os.path.relpath(path)} =====")
    print(f"servos: shoulder {SHOULDER}\n        elbow    {ELBOW}\n")
    ok = travel(m)
    print("\n2. HOLDING (worst static gravity torque, whole range)")
    for extra, tag in ((0.0, "as modelled"), (ELBOW.mass, f"+{ELBOW.mass*1000:.0f} g elbow servo")):
        w = holding(m, extra)
        print(f"   {tag}:")
        for j in ARM_J:
            s = servo_for(j)
            pct = 100 * w[j] / s.stall_nm
            flag = "ok" if w[j] < s.stall_nm / 3 else ("OVER 1/3 STALL" if w[j] < s.stall_nm else "OVER STALL")
            print(f"     {j:15} {w[j]:6.3f} N*m = {pct:5.1f}% of {s.stall_nm:.2f} stall   {flag}")
            ok &= w[j] < s.stall_nm
    t = tracking(m)
    print(f"\n3. TRACKING (random waypoints, {MAX_RATE} rad/s slew, mj_step)")
    print(f"   error mean {t['mean']:.2f} deg   p95 {t['p95']:.2f}   max {t['max']:.2f}")
    print(f"   actuator at torque ceiling {100 * t['sat']:.2f}% of samples"
          f"   (frames with body contact skipped: {100 * t['contact']:.1f}%)")
    print("   peak actuator torque (L side): " + ", ".join(
        f"{j} {t['peak']['L_' + j]:.2f}" for j in ARM_J))
    return ok, t


if __name__ == "__main__":
    ok, t = report(SCENE, "CURRENT")
    if "--vs" in sys.argv:
        other = sys.argv[sys.argv.index("--vs") + 1]
        _, t2 = report(other, "COMPARE")
        print(f"\nTRACKING DELTA (current - compare): mean {t['mean'] - t2['mean']:+.3f} deg,"
              f" p95 {t['p95'] - t2['p95']:+.3f}, max {t['max'] - t2['max']:+.3f}")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
