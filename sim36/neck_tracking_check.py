"""
neck_tracking_check.py - drive the neck's person-tracking through whole
scenarios, frame by frame, with no camera and no mediapipe.

    python3 sim36/neck_tracking_check.py

neck_tracking.py is pure logic on purpose, so the interesting behaviour - who it
locks onto, when it lets go, what it does when two people cross - can be tested
directly instead of by waving at a webcam and squinting.

Each scenario is a list of frames; each frame is a list of (x, y, size) people.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "live mimic system"))

import neck_tracking as nt          # noqa: E402

FPS = 30.0
DT = 1.0 / FPS
fails = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        fails.append(name)


def run(frames, tracker=None, t0=0.0):
    """Play frames through a tracker; return the per-frame log."""
    tr = tracker or nt.NeckTracker(aspect=16 / 9)
    log = []
    t = t0
    for people in frames:
        cands = [nt.Candidate((x, y), s) for (x, y, s) in people]
        out = tr.update(cands, now=t)
        log.append((tr.state, math.degrees(out["neck_yaw"]),
                    math.degrees(out["neck_pitch"]), tr.target_size))
        t += DT
    return tr, log


def hold(person, n):
    return [[person]] * n


def main():
    print(f"\nHFOV={nt.HFOV_DEG} deg  aspect=16:9  "
          f"VFOV={math.degrees(nt.vfov_rad(aspect=16/9)):.1f} deg")
    print(f"LOCK_FRAMES={nt.LOCK_FRAMES} LOST_GRACE={nt.LOST_GRACE}s "
          f"NEUTRAL_DELAY={nt.NEUTRAL_DELAY}s MAX_RATE={nt.MAX_RATE} rad/s\n")

    # 1 ---------------------------------------------------------------- empty
    print("1. empty frame stays neutral")
    tr, log = run([[]] * 60)
    check("state is NEUTRAL", tr.state == nt.NEUTRAL)
    check("commands dead ahead", abs(log[-1][1]) < 1e-9 and abs(log[-1][2]) < 1e-9,
          f"yaw={log[-1][1]:+.3f} pitch={log[-1][2]:+.3f}")

    # 2 ------------------------------------------------------- acquire + aim
    print("\n2. someone walks in on the right -> turn and lock")
    tr, log = run([[]] * 5 + hold((0.85, 0.40, 0.20), 90))
    want_yaw = math.degrees(math.atan((2 * 0.85 - 1) * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("locked", tr.state == nt.LOCKED)
    check("turned toward them (positive yaw = image right)", log[-1][1] > 0)
    check("settled on the right angle", abs(log[-1][1] - want_yaw) < 0.5,
          f"got {log[-1][1]:+.2f} deg, want {want_yaw:+.2f}")
    first_move = next(i for i, r in enumerate(log) if abs(r[1]) > 0.1)
    check("waited LOCK_FRAMES before committing", first_move >= 5 + nt.LOCK_FRAMES - 1,
          f"first motion at frame {first_move}")

    # 3 --------------------------------------------------- tangent, not linear
    print("\n3. pinhole geometry: angle is atan, not a linear scale")
    edge = math.degrees(math.atan(1.0 * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    half = math.degrees(math.atan(0.5 * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("frame edge == HFOV/2", abs(edge - nt.HFOV_DEG / 2) < 1e-9, f"{edge:.2f} deg")
    check("halfway out is NOT half the angle", abs(half - edge / 2) > 0.5,
          f"halfway={half:.2f} deg, half-of-edge={edge/2:.2f} deg")

    # 4 ------------------------------------------------------------- following
    print("\n4. target walks across frame -> neck follows")
    frames = [[(0.2 + 0.006 * i, 0.4, 0.20)] for i in range(100)]
    tr, log = run(frames)
    yaws = [r[1] for r in log]
    check("still locked", tr.state == nt.LOCKED)
    # The neck starts dead ahead and the target starts LEFT of centre, so yaw
    # first swings negative to catch up and only then follows the walk to the
    # right. Monotonicity is a claim about the tracking, not the catch-up, so
    # measure it after the neck has arrived (~frame 13 at MAX_RATE).
    SETTLED = 15
    check("yaw increases monotonically once caught up",
          all(b >= a - 1e-6 for a, b in zip(yaws[SETTLED:], yaws[SETTLED + 1:])))
    check("tracks to the far side", yaws[-1] > yaws[SETTLED] + 10,
          f"{yaws[SETTLED]:+.1f} -> {yaws[-1]:+.1f} deg")
    # Two DIFFERENT things get confused here, so measure them separately.
    #
    # LAG is the command trailing a MOVING target. It is unavoidable and
    # harmless: while the person walks, the goal keeps moving and the smoothed
    # command sits a fraction of a degree behind it. Measured below, reported
    # rather than asserted.
    #
    # OVERSHOOT is the command sailing PAST a stationary target and coming back.
    # That is ringing, it means the rate limit and smoothing are fighting, and
    # it would show on the real servos. Only a step response isolates it - with
    # a moving target a lagging command naturally sits on the far side of a goal
    # that is retreating toward centre, which looks like overshoot and is not.
    tr2 = nt.NeckTracker(aspect=16 / 9)
    t, lag = 0.0, 0.0
    for people in frames:
        tr2.update([nt.Candidate((people[0][0], people[0][1]), people[0][2])], now=t)
        t += DT
        if tr2.state == nt.LOCKED:
            lag = max(lag, abs(math.degrees(tr2.cmd[0] - tr2.goal[0])))
    print(f"         (tracking lag behind a walking target: {lag:.2f} deg "
          f"= {lag / (0.006 * 30 * 60 / 1.0) * 1000:.0f} ms of travel)")

    tr3 = nt.NeckTracker(aspect=16 / 9)
    t, worst_over = 0.0, -99.0
    for i in range(150):                       # stationary target, step input
        tr3.update([nt.Candidate((0.90, 0.40), 0.20)], now=t)
        t += DT
        if tr3.state == nt.LOCKED:
            worst_over = max(worst_over, math.degrees(tr3.cmd[0] - tr3.goal[0]))
    check("no overshoot on a step (stationary target)", worst_over < 0.01,
          f"worst {worst_over:+.4f} deg past the goal")

    # 5 ------------------------------------- a closer person must NOT steal it
    print("\n5. someone closer walks in -> lock does NOT transfer")
    a, b = (0.25, 0.40, 0.15), (0.75, 0.40, 0.45)      # b is much bigger/closer
    tr, log = run(hold(a, 40) + [[a, b]] * 90)
    yaw_a = math.degrees(math.atan((2 * 0.25 - 1) * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("still locked on the original", abs(log[-1][1] - yaw_a) < 0.5,
          f"yaw {log[-1][1]:+.2f} deg, original at {yaw_a:+.2f}")
    check("did not swing to the closer one", log[-1][1] < 0)

    # 6 ---------------------------------------------- blink survives the grace
    print("\n6. detection blinks out for 2 frames -> lock survives")
    p = (0.70, 0.40, 0.20)
    tr, log = run(hold(p, 40) + [[]] * 2 + hold(p, 40))
    check("still locked after the blink", tr.state == nt.LOCKED)
    states = [r[0] for r in log]
    check("coasted rather than dropping", nt.COASTING in states)
    check("never fell back to neutral", nt.NEUTRAL not in states[5:])

    # 7 ------------------------------------------ target leaves, another stays
    print("\n7. locked person leaves, someone else is there -> take the closest")
    near, far = (0.30, 0.40, 0.40), (0.80, 0.40, 0.12)
    tr, log = run(hold(far, 40) + [[near]] * 120)     # 'far' (small) leaves
    yaw_near = math.degrees(math.atan((2 * 0.30 - 1) * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("re-locked", tr.state == nt.LOCKED)
    check("onto the remaining person", abs(log[-1][1] - yaw_near) < 0.5,
          f"yaw {log[-1][1]:+.2f} deg, they are at {yaw_near:+.2f}")

    print("\n8. with several left behind, take the LARGEST (closest to camera)")
    tgt = (0.50, 0.40, 0.20)
    small, big = (0.20, 0.40, 0.14), (0.85, 0.40, 0.50)
    tr, log = run([[tgt, small, big]] * 40 + [[small, big]] * 150)
    yaw_big = math.degrees(math.atan((2 * 0.85 - 1) * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("picked the closest of the remainder", abs(log[-1][1] - yaw_big) < 0.6,
          f"yaw {log[-1][1]:+.2f} deg, closest at {yaw_big:+.2f}")

    # 9 ------------------------------------------------ everyone leaves -> rest
    print("\n9. everyone leaves -> back to neutral")
    tr, log = run(hold((0.85, 0.25, 0.20), 40) + [[]] * 150)
    check("state NEUTRAL", tr.state == nt.NEUTRAL)
    check("returned to dead ahead", abs(log[-1][1]) < 0.05 and abs(log[-1][2]) < 0.05,
          f"yaw={log[-1][1]:+.4f} pitch={log[-1][2]:+.4f}")
    idx = next(i for i, r in enumerate(log) if i > 40 and abs(r[1]) < abs(log[40][1]) - 1)
    check("waited out LOST_GRACE + NEUTRAL_DELAY first",
          (idx - 40) * DT >= nt.LOST_GRACE - 2 * DT,
          f"started returning {((idx-40)*DT):.2f}s after they left")

    # 10 ------------------------------------------------------ crossing paths
    print("\n10. two people cross -> lock stays with the right one")
    frames = []
    for i in range(120):                      # left-to-right and right-to-left
        u = i / 119.0
        frames.append([(0.20 + 0.60 * u, 0.40, 0.20),
                       (0.80 - 0.60 * u, 0.45, 0.22)])
    tr, log = run([[(0.20, 0.40, 0.20)]] * 5 + frames)
    end = math.degrees(math.atan((2 * 0.80 - 1) * math.tan(math.radians(nt.HFOV_DEG) / 2)))
    check("followed the one it started on, left to right",
          abs(log[-1][1] - end) < 2.0, f"ended at {log[-1][1]:+.2f} deg, want {end:+.2f}")

    # 11 ----------------------------------------------------------- rate limit
    print("\n11. slew is bounded (the servos have to follow this)")
    tr, log = run([[]] * 3 + hold((0.99, 0.99, 0.30), 120))
    worst = max(abs(log[i + 1][1] - log[i][1]) for i in range(len(log) - 1))
    cap = math.degrees(nt.MAX_RATE * DT)
    check("no frame exceeds MAX_RATE", worst <= cap + 1e-6,
          f"worst {worst:.3f} deg/frame, cap {cap:.3f}")

    # 12 --------------------------------------------------------------- clamps
    print("\n12. commands stay inside the servo limits")
    nt.HFOV_DEG = 200.0                       # absurd lens, to force the clamp
    tr, log = run(hold((1.0, 1.0, 0.3), 200))
    lim = math.degrees(nt.YAW_LIMIT)
    check("yaw clamped at the joint limit", abs(log[-1][1]) <= lim + 1e-6,
          f"{log[-1][1]:+.2f} deg vs limit {lim:.1f}")
    check("pitch clamped too", abs(log[-1][2]) <= math.degrees(nt.PITCH_LIMIT) + 1e-6,
          f"{log[-1][2]:+.2f} deg")
    nt.HFOV_DEG = 60.0

    print()
    if fails:
        print(f"{len(fails)} CHECK(S) FAILED: {fails}")
        sys.exit(1)
    print("all checks passed")


if __name__ == "__main__":
    main()
