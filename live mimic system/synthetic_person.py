"""
synthetic_person.py - a fake person, in MediaPipe WORLD-landmark format.

Used to test the live pipeline without a camera:
    mimic_receiver_v11.py --selftest    drives the robot from this, no sender
    sim36/live_pipeline_check.py        UDP end-to-end + two-camera fusion test

MediaPipe world landmarks: metres, origin between the hips, +x = image right,
+y = DOWN, +z = away from the camera. The person faces the camera, so their
forward is -z, their left is +x (it appears on the right of the image), and
up is -y.

One 12 s cycle:
    0-4 s   both arms swing FORWARD to 90 deg and back      (shoulder flexion)
    4-8 s   both arms swing OUT to the side to 80 deg       (abduction)
    8-12 s  both elbows bend to 100 deg                     (elbow)
"""
import numpy as np

FWD = np.array([0.0, 0.0, -1.0])
LEFT = np.array([1.0, 0.0, 0.0])
UP = np.array([0.0, -1.0, 0.0])
PERIOD = 12.0

# MediaPipe landmark ids used here
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 2, 5, 7, 8
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR = 11, 12, 13, 14, 15, 16
L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANK, R_ANK = 23, 24, 25, 26, 27, 28


def _b(f, l, u):
    return f * FWD + l * LEFT + u * UP


def truth(t):
    """(flex, abd, elbow) in degrees at time t - same for both arms."""
    ph = t % PERIOD

    def bump(a, b):
        s = min(max((ph - a) / (b - a), 0.0), 1.0)
        return np.sin(np.pi * s)

    return 90.0 * bump(0, 4), 80.0 * bump(4, 8), 100.0 * bump(8, 12)


def pose(t):
    """(33, 3) world landmarks and (33,) visibility for time t."""
    W = np.zeros((33, 3))
    W[L_HIP], W[R_HIP] = _b(0, 0.10, 0), _b(0, -0.10, 0)
    W[L_KNEE], W[R_KNEE] = _b(0, 0.10, -0.42), _b(0, -0.10, -0.42)
    W[L_ANK], W[R_ANK] = _b(0, 0.10, -0.82), _b(0, -0.10, -0.82)
    W[L_SH], W[R_SH] = _b(0, 0.18, 0.50), _b(0, -0.18, 0.50)
    W[NOSE] = _b(0.09, 0, 0.68)
    W[L_EYE], W[R_EYE] = _b(0.08, 0.03, 0.70), _b(0.08, -0.03, 0.70)
    W[L_EAR], W[R_EAR] = _b(0.0, 0.07, 0.68), _b(0.0, -0.07, 0.68)
    flex, abd, elb = (np.radians(v) for v in truth(t))
    for sh, el, wr, sgn in ((L_SH, L_EL, L_WR, 1.0), (R_SH, R_EL, R_WR, -1.0)):
        ua = _b(np.sin(flex) * np.cos(abd), sgn * np.sin(abd),
                -np.cos(flex) * np.cos(abd))
        ua /= np.linalg.norm(ua)
        fa = np.cos(elb) * ua + np.sin(elb) * FWD     # forearm folds forward
        fa /= np.linalg.norm(fa)
        W[el] = W[sh] + 0.28 * ua
        W[wr] = W[el] + 0.25 * fa
    vis = np.full(33, 0.99)
    return W, vis
