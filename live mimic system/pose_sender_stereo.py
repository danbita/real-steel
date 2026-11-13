"""
pose_sender_stereo.py - TWO OR THREE cameras (e.g. the Mac's own + an iPhone via
Continuity Camera + an iPad via Camo) -> one skeleton with real depth -> the
v11 robot.

    python "live mimic system/pose_sender_stereo.py" --list-cams   # find indices
    python "live mimic system/pose_sender_stereo.py" --cams 0 1
    python "live mimic system/pose_sender_stereo.py" --cams 0 1 2   # 3 cameras
    mjpython "live mimic system/mimic_receiver_v11.py"            # other terminal

Same UDP packet as `pose_sender.py --landmarks`, so the receiver needs no
change and you can switch between one and two cameras freely.

SETTING UP
  * Put the cameras 45-90 deg apart around you, all seeing your upper body
    (hips too, if you can). E.g. Mac in front, iPhone off to one side, and a
    third camera off to the OTHER side.
  * Start it and MOVE YOUR ARMS AROUND for ~5 s. It learns how the cameras
    are angled from your own body (no checkerboard); the window says
    "aligning" until then, then shows the angle it found between them.
    If that angle looks wrong (it should roughly match how you placed them),
    press r to re-learn.
  * Camera A (first index) is the reference: every other camera learns its
    angle relative to A, so A should see you well. Once learned, any camera
    that drops out (A included) is simply left out of the fusion and the
    rest carry on, so the robot never just stops.
  * A third camera mostly helps when an arm is hidden from one view (crossing
    your body, turned away); depth is already good with two.
  * Each camera runs its own MediaPipe, so three cost ~50% more CPU than two.
    If fps drops: --res 640x480, --model lite, or --gpu.

FLAGS  --cams A B [C]  --model lite|full|heavy  --gpu  --res 1280x720  --noshow
       --list-cams
       each camera can be a camera number OR a stream URL from an IP-camera app on
       the phone (e.g. --cams 0 http://192.168.1.23:8081/video): that goes
       over Wi-Fi with no Continuity Camera distance limit.
How it fuses: see stereo_fusion.py.
"""
import os
import sys

os.environ["OPENCV_AVFOUNDATION_SKIP_AUTH"] = "1"
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import socket  # noqa: E402
import time  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import pose_sender as ps  # noqa: E402
from stereo_fusion import ViewAligner, fuse_many  # noqa: E402

FRESH_S = 0.3          # a view older than this is treated as lost
PAIR_S = 0.15          # results this close in time are treated as simultaneous


def list_cams(n=6):
    print("Probing camera indices (on a Mac the iPhone shows up when it is "
          "near, unlocked and Continuity Camera is on):")
    found = 0
    for i in range(n):
        cap = cv2.VideoCapture(i)
        if not cap.isOpened():
            cap.release()
            break                    # indices are contiguous on macOS
        found += 1
        # cameras (the iPhone especially) can take a few seconds to wake up
        ok, f, t0 = False, None, time.time()
        while time.time() - t0 < 6.0:
            ok, f = cap.read()
            if ok and f is not None:
                break
            time.sleep(0.1)
        if ok:
            fn = os.path.join(HERE, f"camera_{i}.jpg")
            cv2.imwrite(fn, f)
            print(f"  camera {i}: {f.shape[1]}x{f.shape[0]}  "
                  f"(snapshot saved: live mimic system/camera_{i}.jpg - open it "
                  f"to see which camera this is)")
        else:
            print(f"  camera {i}: opens but sent no frame in 6 s "
                  "(asleep, locked iPhone, or no camera permission)")
        cap.release()
    print(f"{found} camera(s). Use them as: --cams <Mac index> <iPhone index> "
          "[<third index>]")


class View:
    """One camera + its own MediaPipe instance + its own swap guards."""

    def __init__(self, idx, name):
        self.name = name
        self.idx = idx
        self.cam = ps.CameraStream(idx, ps.CAP_W, ps.CAP_H, name)
        self.res = ps.PoseResults()
        self.lm = ps.make_landmarker(self.res.callback)
        self.arm_guard = ps.SwapGuard(ps.L_WRIST, ps.R_WRIST,
                                      ps.ARM_CHAIN_L, ps.ARM_CHAIN_R)
        self.leg_guard = ps.SwapGuard(ps.L_ANKLE, ps.R_ANKLE,
                                      ps.LEG_CHAIN_L, ps.LEG_CHAIN_R)
        self.t0 = time.time()
        self.last_ts = -1
        self.last_fseq = 0
        self.used_stamp = 0.0
        self.frame = None
        self.world = self.vis = self.img = None
        self.stamp = 0.0
        self.m_pose, self.m_cam = ps.RateMeter(), ps.RateMeter()
        self.last_seq, self.last_n = 0, 0

    def poll(self, now):
        """Feed the newest frame to MediaPipe; return True on a new person."""
        frame, fseq, _ = self.cam.latest()
        if frame is not None and fseq != self.last_fseq:
            self.last_fseq = fseq
            self.frame = frame.copy()
            ts = max(int((now - self.t0) * 1000), self.last_ts + 1)
            try:
                if self.res.submit(self.lm, self.frame, ts):
                    self.last_ts = ts
            except Exception as e:
                print(f"{self.name}: tracker error ({e}); rebuilding")
                try:
                    self.lm.close()
                except Exception:
                    pass
                self.lm = ps.make_landmarker(self.res.callback)
                self.res.busy = False
        self.m_cam.tick(self.cam.nread - self.last_n)
        self.last_n = self.cam.nread
        if self.res.seq != self.last_seq:
            self.m_pose.tick(self.res.seq - self.last_seq)
            self.last_seq = self.res.seq
        r = self.res
        if r.world is not None and r.stamp != self.used_stamp:
            self.used_stamp = r.stamp
            w = r.world.copy()
            self.arm_guard.fix(w, now)
            self.leg_guard.fix(w, now)
            self.world, self.vis, self.img, self.stamp = w, r.vis.copy(), r.img.copy(), r.stamp
            return True
        return False

    def fresh(self, now):
        return (self.world is not None and now - self.stamp < FRESH_S
                and self.cam.status() == "live")

    def tile(self, now, height=400):
        st = self.cam.status()
        if self.frame is None:
            f = np.zeros((height, int(height * 16 / 9), 3), np.uint8)
        else:
            f = self.frame.copy()
            if self.fresh(now) and st == "live":
                ps.draw_skeleton(f, self.img, self.vis)
            s = height / f.shape[0]
            f = cv2.resize(f, None, fx=s, fy=s)
        if st != "live":
            # never show a frozen last frame as if it were live
            f = (f * 0.25).astype(np.uint8)
            cv2.putText(f, f"CAMERA {self.name} (index {self.idx}): NO VIDEO",
                        (10, f.shape[0] // 2 - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
            cv2.putText(f, st, (10, f.shape[0] // 2 + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(f, "it reconnects by itself when the camera comes back",
                        (10, f.shape[0] // 2 + 42), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200, 200, 200), 1)
        rc, rp = self.m_cam.update(now), self.m_pose.update(now)
        cv2.putText(f, f"{self.name} (camera {self.idx})  {rc:4.1f} fps  "
                    f"pose {rp:4.1f}/s  {self.res.lat_ms:3.0f} ms",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        if not self.fresh(now):
            cv2.putText(f, "no person", (10, 54), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)
        return f

    def close(self):
        self.cam.release()
        try:
            self.lm.close()
        except Exception:
            pass


NAMES = "ABCDEF"


def parse_cams(argv):
    """--cams 0 1 [2 ...]: numbers are local cameras (Mac webcam, Continuity
    Camera iPhone, Camo iPad); anything else is a stream URL from an IP-camera
    app (http://192.168.1.23:8081/video), which goes over Wi-Fi."""
    if "--cams" not in argv:
        return [0, 1]
    i = argv.index("--cams") + 1
    out = []
    while i < len(argv) and not argv[i].startswith("--"):
        out.append(int(argv[i]) if argv[i].isdigit() else argv[i])
        i += 1
    if not 2 <= len(out) <= len(NAMES):
        sys.exit(f"--cams needs 2 to {len(NAMES)} cameras, got {len(out)}")
    return out


def main():
    if "--list-cams" in sys.argv:
        list_cams()
        return
    srcs = parse_cams(sys.argv)
    noshow = "--noshow" in sys.argv

    ps.ensure_model()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    views = [View(src, NAMES[k]) for k, src in enumerate(srcs)]
    A, others = views[0], views[1:]
    # one aligner per non-reference camera: maps that camera into A's frame
    align = {v.name: ViewAligner() for v in others}
    m_sent = ps.RateMeter()
    mode = "starting"
    nxt = time.time() + 2.0
    next_draw = 0.0
    cpu0, loops = time.process_time(), 0
    seen = tuple(0 for _ in views)
    src_i = 0
    tile_h = 400 if len(views) == 2 else 300

    def aligned(v):
        return v is A or align[v.name].ready

    def to_ref(v):
        if v is A:
            return np.eye(3), None
        return align[v.name].R, align[v.name].t

    def align_text():
        parts = []
        for v in others:
            a = align[v.name]
            parts.append(f"{v.name} {a.angle_deg():.0f} deg" if a.ready else
                         f"{v.name} aligning {min(a.n, a.min_frames)}/{a.min_frames}")
        return "vs A: " + ", ".join(parts)

    print(f"Multi-camera mocap -> udp://{ps.UDP_ADDR[0]}:{ps.UDP_ADDR[1]} (landmarks)")
    print("cameras: " + ", ".join(f"{v.name} = {v.idx}" for v in views)
          + "  (A is the reference). Move your arms for ~5 s so the views can "
          "align. q quits, r re-aligns.\n")
    try:
        while True:
            now = time.time()
            new = {v.name: v.poll(now) for v in views}
            ok = {v.name: v.fresh(now) for v in views}

            # learn each camera's angle relative to A from simultaneous views
            for v in others:
                if ((new["A"] or new[v.name]) and ok["A"] and ok[v.name]
                        and abs(A.stamp - v.stamp) < PAIR_S):
                    align[v.name].update(A.world, A.vis, v.world, v.vis)

            if any(new.values()):
                F = None
                live = [v for v in views if ok[v.name] and aligned(v)]
                if live:
                    # fuse the newest result with every camera that saw the
                    # same moment (within PAIR_S); all in A's frame
                    t_new = max(v.stamp for v in live)
                    group = [v for v in live if t_new - v.stamp < PAIR_S]
                    F, _ = fuse_many([(v.world, v.vis) + to_ref(v) for v in group])
                    names = "+".join(v.name for v in group)
                    if len(group) >= 2:
                        mode = f"FUSED ({len(group)} cameras: {names})"
                    else:
                        mode = f"camera {names} only"
                    waiting = [v.name for v in others if not align[v.name].ready]
                    if waiting:
                        mode += f" ({'/'.join(waiting)} aligning)"
                else:
                    # nobody aligned is fresh: IK works in YOUR body frame, so
                    # any single camera is fine on its own before alignment
                    for v in views:
                        if ok[v.name]:
                            F = v.world
                            mode = f"camera {v.name} only (aligning)"
                            break
                if F is not None and np.all(np.isfinite(F)):
                    hips_ok = any(ok[v.name] and ps.hips_in_frame(v.img, v.vis)
                                  for v in views)
                    src_i += 1
                    sock.sendto(ps.landmark_packet(F, hips_ok, src_i), ps.UDP_ADDR)
                    m_sent.tick()
            elif not any(ok.values()):
                mode = "no person in any camera"
            dead = [v.name for v in views if v.cam.status() != "live"]
            if dead:
                mode = (f"camera {'/'.join(dead)} has NO VIDEO - running on the rest"
                        if len(dead) < len(views) else "NO VIDEO from any camera")

            rs = m_sent.update(now)
            if now >= nxt:
                nxt = now + 2.0
                cpu = 100.0 * (time.process_time() - cpu0) / 2.0
                cpu0 = time.process_time()
                print(f"STATUS {mode} | sender CPU {cpu:3.0f}% | "
                      + " | ".join(f"{v.name} {v.m_cam.rate:4.1f} fps pose "
                                   f"{v.m_pose.rate:4.1f}/s" for v in views)
                      + f" | sent {rs:4.1f}/s | {align_text()}", flush=True)
                for v in views:
                    st = v.cam.status()
                    if st != "live":
                        print(f"   -> camera {v.name} (index {v.idx}): {st}. "
                              "Running on the other cameras until it is back. "
                              "If it never starts, the index is probably wrong: "
                              "--list-cams", flush=True)
                    elif v.m_cam.rate and v.m_cam.rate < 15:
                        print(f"   -> camera {v.name} is slow ({v.m_cam.rate:.0f} "
                              "fps): more light, or --res 640x480", flush=True)
                for v in others:
                    a = align[v.name]
                    if a.ready and a.angle_deg() < 15:
                        print(f"   -> cameras A and {v.name} look almost the same "
                              "direction - move one 45-90 deg around you",
                              flush=True)

            loops += 1
            frames = tuple(v.last_fseq for v in views)
            new_frame = frames != seen
            seen = frames
            # Redraw only when a camera actually delivered something new, and
            # at most 30x/s. The old loop rebuilt and redrew the tiles as fast
            # as it could spin - hundreds of times a second of pure CPU, which
            # was fine alone but starved the cameras once the sim also ran.
            # (and twice a second regardless, so a dead camera's NO VIDEO
            # banner still updates)
            if not noshow and now >= next_draw and (new_frame or now >= next_draw + 0.5):
                next_draw = now + 1.0 / 30.0
                tiles = [v.tile(now, tile_h) for v in views]
                h = min(t.shape[0] for t in tiles)
                view = np.hstack([t[:h] for t in tiles])
                ready = all(a.ready for a in align.values())
                cv2.putText(view, f"{mode} | sent {rs:4.1f}/s | " +
                            (align_text() if ready else
                             align_text() + " - move your arms around"),
                            (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if "FUSED" in mode else (0, 200, 255), 2)
                cv2.imshow("multi-camera pose (q quit, r re-align)", view)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("r"):
                    align = {v.name: ViewAligner() for v in others}
                    print("re-aligning: move your arms around")
            if not (any(new.values()) or new_frame):
                time.sleep(0.003)            # nothing new: don't spin a core
    except KeyboardInterrupt:
        pass
    for v in views:
        v.close()
    if not noshow:
        cv2.destroyAllWindows()
    print("Stopped.")


if __name__ == "__main__":
    main()
