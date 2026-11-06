"""
stereo_fusion.py - combine two cameras' MediaPipe WORLD landmarks into one
skeleton with real depth. Pure numpy, no camera needed, so it is unit-tested
by sim36/live_pipeline_check.py.

WHY TWO CAMERAS HELP
    MediaPipe gives each landmark in metres, but its DEPTH (z, along the
    camera's line of sight) is a learned guess - several times noisier than
    x / y, which it can actually see. A second camera placed 45-90 deg around
    you sees that same depth as left/right in ITS image, which it measures
    well. Fusing the two, each trusted along the directions it sees, gives a
    skeleton that is good in all three axes.

NO CALIBRATION BOARD
    MediaPipe world landmarks are already metric and centred on the hips in
    BOTH cameras, so the only unknown between the two views is a ROTATION
    (how far apart the cameras are around you). ViewAligner learns it from
    your own body while you move - the same 33 points seen from two sides is
    a textbook rotation fit (Kabsch). It averages over many frames, so the
    depth noise in each single frame washes out, and keeps adapting slowly in
    case a camera gets bumped.

FUSION
    Each camera's landmark is treated as a measurement with an uncertainty
    ellipsoid: tight across its image, DEPTH_RATIO times looser along its line
    of sight, and looser still when MediaPipe's visibility is low. Two such
    measurements combine by inverse-covariance weighting - which is exactly
    "trust each camera along the directions it sees". If only one camera
    sees a landmark, that camera's estimate is used.
"""
import numpy as np

DEPTH_RATIO = 3.0        # MediaPipe z noise vs x/y noise (conservative)
BASE_SIGMA = 0.012       # m, x/y noise of a fully visible landmark
VIS_MIN = 0.3            # below this a camera's landmark is ignored
ALIGN_IDS = [0, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24]   # stable body points


def kabsch(H):
    """Rotation R minimising sum |R b - a|^2, from H = sum b a^T."""
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    return Vt.T @ D @ U.T


class ViewAligner:
    """Learns R such that  world_A ~= R @ world_B  (both hip-centred)."""

    def __init__(self, tau_frames=300, min_frames=30):
        self.H = np.zeros((3, 3))
        self.w = 0.0
        self.n = 0
        self.decay = 1.0 - 1.0 / tau_frames
        self.min_frames = min_frames
        self.R = np.eye(3)
        self.t = np.zeros(3)       # small residual offset between the origins

    @property
    def ready(self):
        return self.n >= self.min_frames

    def update(self, A, visA, B, visB):
        ids = [i for i in ALIGN_IDS if visA[i] > 0.6 and visB[i] > 0.6]
        if len(ids) < 5:
            return False
        w = np.minimum(visA[ids], visB[ids])
        ca = np.average(A[ids], axis=0, weights=w)
        cb = np.average(B[ids], axis=0, weights=w)
        a = A[ids] - ca
        b = B[ids] - cb
        # reject frames whose shape disagrees wildly (bad detection in one view)
        if self.ready:
            res = np.linalg.norm((self.R @ b.T).T - a, axis=1)
            if np.median(res) > 0.15:
                return False
        self.H = self.decay * self.H + (b * w[:, None]).T @ a
        self.n += 1
        self.R = kabsch(self.H)
        t = ca - self.R @ cb
        self.t = t if self.n == 1 else 0.95 * self.t + 0.05 * t
        return True

    def angle_deg(self):
        """How far apart the two cameras are, about any axis."""
        c = (np.trace(self.R) - 1.0) / 2.0
        return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _cov(vis):
    """Per-landmark covariance in the camera's own frame, (N, 3, 3)."""
    s = BASE_SIGMA / np.clip(vis, 0.05, 1.0)
    C = np.zeros((len(vis), 3, 3))
    C[:, 0, 0] = s ** 2
    C[:, 1, 1] = s ** 2
    C[:, 2, 2] = (DEPTH_RATIO * s) ** 2
    return C


def fuse(A, visA, B, visB, R, t=None):
    """Fused (33,3) landmarks in camera A's frame, and fused visibility."""
    Bin = (R @ B.T).T + (0.0 if t is None else t)   # B's points in A's frame
    CA = _cov(visA)
    CB = R[None] @ _cov(visB) @ R.T[None]  # rotate B's ellipsoids too
    out = A.copy()
    vis = np.maximum(visA, visB)
    for i in range(len(A)):
        okA, okB = visA[i] >= VIS_MIN, visB[i] >= VIS_MIN
        if okA and okB:
            IA, IB = np.linalg.inv(CA[i]), np.linalg.inv(CB[i])
            out[i] = np.linalg.solve(IA + IB, IA @ A[i] + IB @ Bin[i])
        elif okB:
            out[i] = Bin[i]
        # okA only, or neither: keep A's own estimate
    return out, vis
