"""
neck_tracking.py - decide where the neck should look.

Pure logic: math and numpy only, no mediapipe, no OpenCV, no sockets. The
camera runner (neck_tracker_sender.py) turns frames into Candidates and hands
them here; this file owns every decision. That split is what makes the behaviour
testable without a camera - see sim36/neck_tracking_check.py, which drives whole
scenarios (someone walks in, two people cross, everybody leaves) frame by frame.

BEHAVIOUR
    empty frame            -> neutral, looking straight ahead
    someone appears        -> turn to them and LOCK
    locked                 -> follow that person only, ignoring everyone else
    locked person leaves   -> the next CLOSEST person, or neutral if nobody is left

CAMERA MODEL: FIXED, OFF-BOARD
    The camera does not move with the neck, so this is open loop: a person's
    bearing in the frame maps straight to an absolute neck angle. Nothing feeds
    back, so it cannot oscillate, and the neck can point at anyone the camera
    can see.

    IF THE CAMERA EVER MOVES TO THE HEAD, this file is where it changes, and the
    change is not cosmetic. On-board, a centred target reads as bearing zero no
    matter where the neck is pointing, so the command becomes
    current_angle + gain * bearing_error rather than an absolute angle, and a
    target that walks out of frame is genuinely gone - there is no longer any
    evidence of which way they went. HEAD_MOUNTED below is the switch, and
    aim_angles() is the only function that needs to care.

WHY NOT REUSE pose_sender's One Euro FILTER
    Different signal. One Euro is tuned to pass fast limb motion while killing
    idle jitter, and its derivative term actively fights step changes. Here a
    step change - re-targeting to a new person - is a thing we WANT, and we want
    it to come out as a controlled sweep rather than a snap or a fought-down
    crawl. A slew-rate limit plus light smoothing does that; One Euro would
    lurch, then drag.
"""

import math
import time

try:
    import numpy as np
except Exception:                      # numpy is optional here
    np = None

# --------------------------------------------------------------------- config

HFOV_DEG = 60.0      # horizontal field of view of the camera.
                     # MEASURE THIS - webcams vary from about 55 to 78 deg, and
                     # it is the single number that sets how far the neck turns
                     # for a given position in frame. Too small and the neck
                     # under-turns; too large and it overshoots. Easiest
                     # calibration: stand so you are exactly at the left edge of
                     # frame, note the angle the neck commands, and compare it
                     # with the real angle from the camera to you.
ASPECT = 16.0 / 9.0  # frame width / height. The runner overrides this from the
                     # actual capture size.

HEAD_MOUNTED = False  # see the camera-model note above

YAW_LIMIT = 1.413717    # rad, +/-81 deg. Mirrors neck_v1.urdf, which mirrors
PITCH_LIMIT = 1.413717  # gen_neck.py's 90%-of-servo-travel rule.

SIGN_YAW = 1.0       # flip if the neck turns away from the person instead of
SIGN_PITCH = 1.0     # toward them. Settle these on the bench, same as the
                     # mirroring signs in pose_sender.py.

LOCK_FRAMES = 3      # consecutive frames a candidate must be seen before the
                     # neck commits. Stops a one-frame false positive from
                     # whipping the head across the room.
LOST_GRACE = 0.5     # s to keep holding the last aim when the target vanishes.
                     # Detection drops out for a frame or two constantly; without
                     # this the neck would abandon a lock every time someone
                     # turned sideways.
NEUTRAL_DELAY = 0.8  # s of a genuinely empty frame before returning to rest.

MAX_ASSOC_JUMP = 0.25   # normalized image units. How far the target may move
                        # between frames and still be recognised as the same
                        # person. Too big and the lock hops to whoever is
                        # nearest when two people cross; too small and it drops
                        # the lock on anyone walking quickly.

MAX_RATE = 2.5       # rad/s. The sweep speed when re-targeting. Also what keeps
                     # the commanded angle inside what the servos can follow.
SMOOTH_TAU = 0.08    # s, light first-order smoothing on top of the rate limit.

NEUTRAL = "NEUTRAL"
ACQUIRING = "ACQUIRING"
LOCKED = "LOCKED"
COASTING = "COASTING"


class Candidate:
    """One detected person, as the camera runner sees them.

    aim     (x, y) in normalized image coords, 0..1, of the point to look AT -
            the face, not the body centroid. Looking at someone means looking at
            their face; aiming at a centroid points the neck at their chest.
    size    apparent size, larger = closer to the camera. Used only to choose
            between people, never for an absolute distance.
    quality 0..1, how much of the person the detector actually saw.
    """

    __slots__ = ("aim", "size", "quality", "key")

    def __init__(self, aim, size, quality=1.0, key=None):
        self.aim = (float(aim[0]), float(aim[1]))
        self.size = float(size)
        self.quality = float(quality)
        self.key = key

    def __repr__(self):
        return (f"Candidate(aim=({self.aim[0]:.3f},{self.aim[1]:.3f}), "
                f"size={self.size:.3f}, q={self.quality:.2f})")


def vfov_rad(hfov_deg=None, aspect=None):
    """Vertical FOV implied by the horizontal one and the frame's aspect ratio.

    Derived rather than configured: they are not independent, and letting
    someone set both invites a pair that no real lens could produce.
    """
    h = math.radians(HFOV_DEG if hfov_deg is None else hfov_deg)
    a = ASPECT if aspect is None else aspect
    return 2.0 * math.atan(math.tan(h / 2.0) / a)


def aim_angles(aim, hfov_deg=None, aspect=None, current=(0.0, 0.0)):
    """Normalized image position -> (yaw, pitch) the neck should hold, radians.

    A pinhole camera maps angle to image position through a TANGENT, not
    linearly: a person at the very edge of a 60 deg frame is at 30 deg, but one
    halfway to the edge is at 16.1 deg, not 15. The error is small near the
    centre and grows toward the edges, which is exactly where you notice the
    neck under-pointing. Hence atan/tan rather than a straight scale factor.

    Image y runs DOWNWARD, and the neck's +pitch is nose-down (matching *_flex
    in pose_sender.py), so pitch needs no sign flip here. The frame is NOT
    mirrored - pose_sender feeds MediaPipe the raw capture with no cv2.flip - so
    image +x is the camera's right.
    """
    h = math.radians(HFOV_DEG if hfov_deg is None else hfov_deg)
    v = vfov_rad(hfov_deg, aspect)
    yaw = math.atan((2.0 * aim[0] - 1.0) * math.tan(h / 2.0))
    pitch = math.atan((2.0 * aim[1] - 1.0) * math.tan(v / 2.0))
    if HEAD_MOUNTED:
        # the bearing is relative to where the neck already points, so it is an
        # ERROR to be driven out, not an absolute target
        yaw += current[0]
        pitch += current[1]
    yaw = max(-YAW_LIMIT, min(YAW_LIMIT, SIGN_YAW * yaw))
    pitch = max(-PITCH_LIMIT, min(PITCH_LIMIT, SIGN_PITCH * pitch))
    return yaw, pitch


def _dist(a, b, aspect=None):
    """Distance in image space, with x rescaled by the aspect ratio.

    x is normalized by WIDTH and y by HEIGHT, so on a 16:9 frame one unit of x
    is 1.78 units of y in real angle. Comparing them raw makes horizontal motion
    look smaller than it is and biases every association test."""
    a_ = ASPECT if aspect is None else aspect
    return math.hypot((a[0] - b[0]) * a_, a[1] - b[1])


class NeckTracker:
    """The state machine. One instance per camera; call update() once a frame."""

    def __init__(self, hfov_deg=None, aspect=None):
        self.hfov_deg = HFOV_DEG if hfov_deg is None else hfov_deg
        self.aspect = ASPECT if aspect is None else aspect
        self.state = NEUTRAL
        self.target_aim = None       # last known position of the locked person
        self.target_size = 0.0
        self.cmd = [0.0, 0.0]        # what we are actually commanding
        self.goal = [0.0, 0.0]       # where we want to be
        self._seen = 0               # consecutive frames a candidate has existed
        self._lost_since = None
        self._empty_since = None
        self._t = None
        self.locked_frames = 0

    # ---------------------------------------------------------------- helpers

    def _match(self, candidates):
        """Which candidate is the person we locked onto?

        Nearest to where they were last frame, and only if that is close enough.
        MediaPipe does not promise stable ordering between frames, so the index
        is worthless as an identity - position continuity is what we have."""
        if self.target_aim is None or not candidates:
            return None
        best, bd = None, 1e9
        for c in candidates:
            d = _dist(c.aim, self.target_aim, self.aspect)
            if d < bd:
                best, bd = c, d
        return best if bd <= MAX_ASSOC_JUMP else None

    @staticmethod
    def _closest(candidates):
        """The person nearest the CAMERA, taken as the largest in frame.

        Apparent size is the only distance cue a single camera gives without
        depth. It is imperfect - someone turning side-on narrows their shoulders
        and someone half out of frame measures small - which is why it decides
        only who to acquire, never whether to abandon a lock already held."""
        return max(candidates, key=lambda c: c.size) if candidates else None

    def _lock(self, cand):
        self.target_aim = cand.aim
        self.target_size = cand.size
        self.state = LOCKED
        self._lost_since = None
        self.locked_frames = 0

    def _release(self):
        self.target_aim = None
        self.target_size = 0.0
        self._lost_since = None
        self._seen = 0

    # ------------------------------------------------------------------ update

    def update(self, candidates, now=None):
        """One frame. Returns {"neck_yaw":…, "neck_pitch":…} in radians."""
        now = time.time() if now is None else now
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        candidates = [c for c in (candidates or []) if c is not None]

        if candidates:
            self._empty_since = None
        elif self._empty_since is None:
            self._empty_since = now

        if self.state in (LOCKED, COASTING):
            hit = self._match(candidates)
            if hit is not None:
                self._lock(hit)
                self.locked_frames += 1
                self.goal = list(aim_angles(hit.aim, self.hfov_deg, self.aspect,
                                            current=self.cmd))
            else:
                # target missing. Hold the last aim briefly - detection blinks
                # out constantly and dropping the lock on every blink would make
                # the neck twitch between people.
                if self._lost_since is None:
                    self._lost_since = now
                self.state = COASTING
                if now - self._lost_since > LOST_GRACE:
                    self._release()
                    nxt = self._closest(candidates)
                    if nxt is not None:
                        self._lock(nxt)
                        self.goal = list(aim_angles(nxt.aim, self.hfov_deg,
                                                    self.aspect, current=self.cmd))
                    else:
                        self.state = NEUTRAL
                        self.goal = [0.0, 0.0]

        if self.state in (NEUTRAL, ACQUIRING):
            if candidates:
                self._seen += 1
                self.state = ACQUIRING
                if self._seen >= LOCK_FRAMES:
                    pick = self._closest(candidates)
                    self._lock(pick)
                    self.goal = list(aim_angles(pick.aim, self.hfov_deg,
                                                self.aspect, current=self.cmd))
            else:
                self._seen = 0
                self.state = NEUTRAL
                if self._empty_since is not None and now - self._empty_since >= NEUTRAL_DELAY:
                    self.goal = [0.0, 0.0]

        self._advance(dt)
        return {"neck_yaw": self.cmd[0], "neck_pitch": self.cmd[1]}

    def _advance(self, dt):
        """Slew toward the goal: rate limit first, then light smoothing.

        Rate limit before smoothing, not after - it is what bounds the sweep when
        the neck re-targets across the room, and smoothing afterwards takes the
        corner off without letting the peak speed back up."""
        if dt <= 0.0:
            return
        step = MAX_RATE * dt
        alpha = 1.0 - math.exp(-dt / SMOOTH_TAU) if SMOOTH_TAU > 0 else 1.0
        for i in (0, 1):
            err = self.goal[i] - self.cmd[i]
            err = max(-step, min(step, err))
            self.cmd[i] += alpha * err
        self.cmd[0] = max(-YAW_LIMIT, min(YAW_LIMIT, self.cmd[0]))
        self.cmd[1] = max(-PITCH_LIMIT, min(PITCH_LIMIT, self.cmd[1]))

    def describe(self):
        return (f"{self.state:9s} "
                f"yaw={math.degrees(self.cmd[0]):+6.1f} "
                f"pitch={math.degrees(self.cmd[1]):+6.1f}"
                + (f"  target size={self.target_size:.3f}"
                   if self.target_aim else ""))
