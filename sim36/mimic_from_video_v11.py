"""
mimic_from_video_v11.py — drive humanoid_v7 from a reference video, side by side.

    python sim/mimic_from_video_v11.py

Reads ref/ref_clip.mp4, runs MediaPipe pose on it, retargets with the REPO'S OWN
frame_angles() from pose_sender.py (3D body-frame decomposition, so camera angle
does not matter), then drives the v7 model through mj_step and renders a
side-by-side against the source.

Retarget mapping v3-humanoid -> v7 hardware:
    *_shoulder_flex -> *_shoulder_pitch    direct, same sign
    *_shoulder_abd  -> *_shoulder_roll     roll = REST_ROLL - abd
    *_shoulder_twist                       parked at 0 (no estimate from pose)
    *_elbow, legs   -> direct
"""
import importlib.util, os, sys
import cv2, mujoco, numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# SECOND CLIP. `ATOM_CLIP=<path>` runs this whole pipeline against another video
# and suffixes every derived artefact with that file's stem, so the ref_clip.mp4
# results are never overwritten. Unset, the paths below are exactly as before.
_CLIP = os.environ.get("ATOM_CLIP")
_SLUG = ("_" + os.path.splitext(os.path.basename(_CLIP))[0]) if _CLIP else ""
REF = _CLIP or os.path.join(ROOT, "ref", "ref_clip.mp4")
SCENE = os.path.join(ROOT, "sim36", "humanoid_v11_scene.xml")
OUTDIR = os.path.join(ROOT, "sim", "out36"); os.makedirs(OUTDIR, exist_ok=True)
FRAMEDIR = os.path.join(ROOT, "ref", f"simframes_v11{_SLUG}"); os.makedirs(FRAMEDIR, exist_ok=True)

# import the repo's retargeting so the maths is theirs, not a reimplementation
PS = os.path.join(ROOT, "live mimic system", "pose_sender.py")
spec = importlib.util.spec_from_file_location("pose_sender", PS)
ps = importlib.util.module_from_spec(spec); spec.loader.exec_module(ps)

# v5 puts roll 0 at the arm HANGING and 140 deg near overhead, so there is no
# separate "safe" clamp any more - the model's own joint limits are the measured
# envelope. v9 clamped roll to 68..140 deg, which meant the robot could not put
# its arms down at all; that single line is why it stood there like a scarecrow
# through the whole reference clip.
SKIP_RENDER = os.environ.get("SKIP_RENDER") == "1"
REST_ROLL = 0.0
EASE = 45            # frames to blend from arms-down into tracking
W = H = 720

import mediapipe as mp
from mediapipe.tasks import python as mpp
from mediapipe.tasks.python import vision


def extract():
    ps.ensure_model()
    opts = vision.PoseLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=ps.MODEL_PATH),
        running_mode=vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.5, min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5)
    lm = vision.PoseLandmarker.create_from_options(opts)
    cap = cv2.VideoCapture(REF)
    gates, filters = ps.Gates(), {n: ps.OneEuro() for n in ps.JOINT_NAMES}
    seq, held, i = [], {}, 0
    raw = []                       # world landmarks + frame validity, for scoring
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                  int(i * 1000 / 30))
        if res.pose_world_landmarks:
            wl = res.pose_world_landmarks[0]
            # DO NOT flip a single axis here. MediaPipe world landmarks are
            # already right-handed (x right, y down, z away from camera), and
            # body_frame() relies on that handedness: up = mid_sh - mid_hip
            # comes out correct because y is down, and fwd = left x up then
            # points out of the chest. Negating y alone makes the frame
            # left-handed, which silently reverses every cross product - the
            # robot leaned and bent backwards while the error metric, measured
            # in the same broken frame, still read low.
            world = np.array([[p.x, p.y, p.z] for p in wl])
            vis = np.array([p.visibility for p in res.pose_landmarks[0]])
            ang = ps.frame_angles(world, vis, gates)
            hips_ok = min(vis[ps.L_HIP], vis[ps.R_HIP]) >= ps.VIS_OFF
            ok_frame = (min(vis[ps.L_SHOULDER], vis[ps.R_SHOULDER]) >= ps.VIS_OFF
                        and (hips_ok or vis[ps.NOSE] >= ps.VIS_OFF))
            raw.append({"world": world.tolist(), "hips_ok": bool(hips_ok),
                        "ok": bool(ok_frame)} if ok_frame else None)
            t = i / 30.0
            for k, v in ang.items():
                held[k] = filters[k](v, t) if k in filters else v
        else:
            raw.append(None)
        while len(raw) < i + 1:
            raw.append(None)
        seq.append(dict(held))
        i += 1
    cap.release(); lm.close()
    return seq, raw


def drive(seq, raw=None):
    m = mujoco.MjModel.from_xml_path(SCENE)
    d = mujoco.MjData(m)
    A = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_act")
    Q = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    JN = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    lim = {n: tuple(m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
           for n in JN}

    # ---------------------------------------------------------------
    # Closed-form retarget against OUR chain, not the v3 flex/abd convention.
    #
    # We observe the elbow AND the wrist, so the arm is fully determined -
    # 4 unknowns, 6 observations, no redundancy and no optimiser needed.
    #   u = shoulder->elbow direction  -> pitch, roll   (2 DOF, exact)
    #   angle(u, f)                    -> elbow
    #   azimuth of f about u           -> twist
    #
    # Twist is resolved against the robot's OWN zero-twist forearm direction,
    # obtained by evaluating forward kinematics at twist=0. Deriving it on
    # paper means guessing how the CAD's twist zero rotates under pitch/roll;
    # asking the model removes the guess.
    # u(p, r) = (-cos r sin p,  sin r,  cos r cos p)   in the shoulder frame
    # ---------------------------------------------------------------
    RLIM = lim["L_shoulder_roll"]
    PLIM = lim["L_shoulder_pitch"]

    def pitch_roll(u):
        ux, uy, uz = u
        best = None
        for p in (np.arctan2(-ux, uz), np.arctan2(-ux, uz) + np.pi,
                  np.arctan2(-ux, uz) - np.pi):
            cr = uz * np.cos(p) - ux * np.sin(p)
            # v5 convention: roll 0 = hanging, pi = overhead. v9 had it
            # the other way up, so the seed is reflected.
            r = np.arctan2(uy, -cr)
            for rr in (r, r + 2 * np.pi):
                if PLIM[0] <= p <= PLIM[1] and RLIM[0] <= rr <= RLIM[1]:
                    cost = abs(p)
                    if best is None or cost < best[0]:
                        best = (cost, p, rr)
        if best is None:
            return None
        return best[1], best[2]

    probe = mujoco.MjData(m)
    MIR = np.array([1.0, -1.0, 1.0])
    SHP = {"L": np.array([0.0, 0.1184, 0.748]), "R": np.array([0.0, -0.1184, 0.748])}

    def fk_limbs(side, p, r, tw, el):
        """(upper-arm dir, forearm dir) for these joint values, mirrored into the
        left-equivalent frame. Measured off the real chain, so the bicep's
        37.7mm lateral offset and the elbow's own offset are included - the
        idealised u(p,r) formula ignores both and is several degrees out."""
        probe.qpos[:] = qhold
        probe.qpos[Q(f"{side}_shoulder_pitch")] = p
        probe.qpos[Q(f"{side}_shoulder_roll")] = r
        probe.qpos[Q(f"{side}_shoulder_twist")] = tw
        probe.qpos[Q(f"{side}_elbow")] = el
        mujoco.mj_forward(m, probe)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{side}_forearm")
        elbow = probe.xpos[b].copy()
        fa = probe.xmat[b].reshape(3, 3) @ np.array([0.0, -1.0, 0.0])
        ua = elbow - SHP[side]
        if side == "R":
            ua, fa = ua * MIR, fa * MIR
        return ua / (np.linalg.norm(ua) + 1e-9), fa / (np.linalg.norm(fa) + 1e-9)

    ELIM = lim["L_elbow"]; TLIM = lim["L_shoulder_twist"]

    def solve_arm(side, u_d, f_d, seed):
        """Full 4-DOF Gauss-Newton: (pitch, roll, twist, elbow) so that BOTH the
        shoulder->elbow and the forearm directions match.

        Solving elbow as angle(u, f) is wrong: at elbow = 0 the robot's forearm
        is NOT parallel to the shoulder->elbow line, because the bicep hangs
        37.7mm off that line and the elbow pivot is offset again. Measured on the
        real chain that assumption costs ~63 deg of forearm error."""
        q = np.array(seed, dtype=float)
        eps = 1e-3
        lo = np.array([PLIM[0], RLIM[0], TLIM[0], ELIM[0]])
        hi = np.array([PLIM[1], RLIM[1], TLIM[1], ELIM[1]])
        for _ in range(14):
            u0, f0 = fk_limbs(side, *q)
            res = np.concatenate([u_d - u0, f_d - f0])
            if np.linalg.norm(res) < 1e-4:
                break
            J = np.zeros((6, 4))
            for k in range(4):
                qp = q.copy(); qp[k] += eps
                u1, f1 = fk_limbs(side, *qp)
                J[:, k] = np.concatenate([(u1 - u0) / eps, (f1 - f0) / eps])
            dq, *_ = np.linalg.lstsq(J, res, rcond=None)
            q = np.clip(q + np.clip(dq, -0.5, 0.5), lo, hi)
        return q

    def fk_forearm(side, p, r, tw, el):
        """Forearm direction at the given joint values, on a THROWAWAY MjData.
        Probing on the live `d` would rewrite the simulation state every frame."""
        d = probe
        d.qpos[:] = qhold
        d.qpos[Q(f"{side}_shoulder_pitch")] = p
        d.qpos[Q(f"{side}_shoulder_roll")] = r
        d.qpos[Q(f"{side}_shoulder_twist")] = tw
        d.qpos[Q(f"{side}_elbow")] = el
        mujoco.mj_forward(m, d)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{side}_forearm")
        v = d.xmat[b].reshape(3, 3) @ np.array([0.0, -1.0, 0.0])
        return v / (np.linalg.norm(v) + 1e-9)

    seeds = {}
    # The IK runs on raw landmarks, so it inherits MediaPipe's frame-to-frame
    # jitter. Filter the SOLVED joint angles - smoothing here rather than on the
    # landmarks keeps the solve exact and stops the servo chasing noise.
    jfilt = {}
    HEAD = np.array([0.0, 0.0, 0.855])
    NECK = np.array([0.0, 0.0, 0.748])
    SKEL = []
    qhold = np.zeros(m.nq)
    for s_ in "LR":
        # v9 measured roll DOWN from an overhead zero, so pi was "hanging".
        # v6 mounted inverted makes 0 the arm hanging, and pi is straight
        # overhead - which is why the clip used to open with the arms up.
        qhold[Q(f"{s_}_shoulder_roll")] = 0.0

    def targets(rawf):
        """rawf: cached world landmarks for this frame, or None."""
        if not rawf:
            return {}
        W3 = np.array(rawf["world"])
        fwd, left, up = ps.body_frame(W3, rawf["hips_ok"])
        Rb = np.stack([fwd, left, up])
        t = {}
        for side, (sh, el_, wr) in (("L", (ps.L_SHOULDER, ps.L_ELBOW, ps.L_WRIST)),
                                    ("R", (ps.R_SHOULDER, ps.R_ELBOW, ps.R_WRIST))):
            u = Rb @ (W3[el_] - W3[sh])
            f = Rb @ (W3[wr] - W3[el_])
            if side == "R":
                u = u * np.array([1.0, -1.0, 1.0])
                f = f * np.array([1.0, -1.0, 1.0])
            nu, nf = np.linalg.norm(u), np.linalg.norm(f)
            if nu < 1e-6:
                continue
            u = u / nu
            # body frame -> robot world frame. fwd=+X, left=+Y, up=+Z already.
            if nf < 1e-6:
                continue
            f = f / nf
            seed = seeds.get(side, (0.0, 0.35, 0.0, 0.6))
            q = solve_arm(side, u, f, seed)
            seeds[side] = tuple(q)
            t[f"{side}_shoulder_pitch"] = q[0]
            t[f"{side}_shoulder_roll"] = q[1]
            t[f"{side}_shoulder_twist"] = q[2]
            t[f"{side}_elbow"] = q[3]
        out = {}
        for k, v in t.items():
            if k not in lim:
                continue
            fl = jfilt.setdefault(k, ps.OneEuro())
            out[k] = float(np.clip(fl(v, targets.t), *lim[k]))
        return out
    targets.t = 0.0

    # settle at the first pose so frame 1 is not a lurch
    first = targets(raw[0] if raw else None) or {}
    for n, v in first.items():
        d.qpos[Q(n)] = v
    for s in "LR":
        if f"{s}_shoulder_roll" not in first:
            d.qpos[Q(f"{s}_shoulder_roll")] = np.pi
    mujoco.mj_forward(m, d)

    r = mujoco.Renderer(m, height=H, width=W)
    sub = max(1, int(round((1 / 30.0) / m.opt.timestep)))
    # Slew-limit the COMMAND to what the servo can actually do. The XH540 is
    # 3.14 rad/s no-load; without this the retargeted signal asks for jumps of
    # >100 deg in one frame and the joint can never catch up.
    MAX_RATE = 3.0
    step = MAX_RATE / 30.0
    # Seed EVERY driven joint with the rest pose. MuJoCo ctrl defaults to 0, and
    # roll = 0 means "arm straight up" - so any frame without a pose detection
    # silently commanded both arms vertical. 25% of this clip has no detection.
    cur = {}
    for jn in JN:
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + "_act") >= 0:
            cur[jn] = np.pi if jn.endswith("shoulder_roll") else 0.0
    stats = {"err": 0.0, "n": 0, "tracked": 0}
    per = {}
    # Accuracy is measured on limb DIRECTIONS, not joint values: the robot and
    # the person have different link lengths, so only the pointing direction is
    # comparable. Each is expressed in its OWN body frame, so camera pose and
    # body size drop out.
    B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    acc = {"ua": [], "fa": []}

    def robot_limbs():
        out = {}
        for s_ in "LR":
            sh = np.array([0.0, 0.1184 if s_ == "L" else -0.1184, 0.748])
            eb = B(f"{s_}_forearm")
            elbow = d.xpos[eb].copy()
            wrist = elbow + d.xmat[eb].reshape(3, 3) @ np.array([0.0, -0.1335, 0.0131])
            out[s_] = (elbow - sh, wrist - elbow)      # robot body frame == world
        return out
    for i, a in enumerate(seq):
        targets.t = i / 30.0
        tg = targets(raw[i] if raw and i < len(raw) else None)
        for k, v in tg.items():
            if k in cur:
                cur[k] += float(np.clip(v - cur[k], -step, step))
            else:
                cur[k] = v
        # open from the rest pose - arms straight down at the sides - and ease
        # into the tracked pose over the first 1.5 s instead of snapping to
        # whatever frame 0 happened to be
        if i < EASE:
            a_ = i / EASE
            for k in cur:
                cur[k] *= a_
        for _ in range(sub):
            for k, v in cur.items():
                d.ctrl[A(k)] = v
            mujoco.mj_step(m, d)
        # record the robot's own joint centres so the side-by-side can draw a
        # skeleton on it too, instead of leaving the comparison to the eye
        row = [HEAD, NECK]
        for s_ in "LR":
            eb = B(f"{s_}_forearm")
            elbow = d.xpos[eb].copy()
            wrist = elbow + d.xmat[eb].reshape(3, 3) @ np.array([0.0, -0.1335, 0.0131])
            row += [np.array([0.0, 0.1184 if s_ == "L" else -0.1184, 0.748]), elbow, wrist]
        row += [np.array([0.0, 0.055, 0.4846]), np.array([0.0, -0.055, 0.4846])]
        SKEL.append(np.array(row))
        if tg:
            stats["tracked"] += 1
            stats["err"] += max(abs(d.qpos[Q(k)] - cur[k]) for k in tg)
            for k in tg:
                e = abs(d.qpos[Q(k)] - cur[k])
                pk = per.setdefault(k, [0.0, 0.0, 0])
                pk[0] = max(pk[0], e); pk[1] += e; pk[2] += 1
            stats["n"] += 1
        if raw and i < len(raw) and raw[i]:
            W3 = np.array(raw[i]["world"])
            fwd, left, up = ps.body_frame(W3, raw[i]["hips_ok"])
            Rb = np.stack([fwd, left, up])             # world -> body
            rl = robot_limbs()
            for s_, (sh, el, wr) in (("L", (ps.L_SHOULDER, ps.L_ELBOW, ps.L_WRIST)),
                                     ("R", (ps.R_SHOULDER, ps.R_ELBOW, ps.R_WRIST))):
                hu = Rb @ (W3[el] - W3[sh])
                hf = Rb @ (W3[wr] - W3[el])
                ru, rf = rl[s_]
                if s_ == "R":                          # mirror to compare like for like
                    hu = hu * np.array([1, -1, 1]); hf = hf * np.array([1, -1, 1])
                    ru = ru * np.array([1, -1, 1]); rf = rf * np.array([1, -1, 1])
                for key, a_, b_ in (("ua", hu, ru), ("fa", hf, rf)):
                    ca = float(np.dot(a_ / (np.linalg.norm(a_) + 1e-9),
                                      b_ / (np.linalg.norm(b_) + 1e-9)))
                    acc[key].append(np.degrees(np.arccos(np.clip(ca, -1, 1))))
        if not SKIP_RENDER:
            r.update_scene(d, camera="front")
            Image.fromarray(r.render()).save(os.path.join(FRAMEDIR, f"{i:05d}.png"))
    if not SKIP_RENDER:
        r.close()
    np.savez_compressed(os.path.join(ROOT, "ref", f"robot_skel_v11{_SLUG}.npz"),
                        pts=np.array(SKEL))
    print(f"  wrote ref/robot_skel_v11{_SLUG}.npz  {np.array(SKEL).shape}")
    if acc["ua"]:
        for k, lbl in (("ua", "upper arm"), ("fa", "forearm  ")):
            v = np.array(acc[k])
            print(f"  {lbl} direction error: mean {v.mean():5.1f}  median "
                  f"{np.median(v):5.1f}  p90 {np.percentile(v,90):5.1f} deg")
    print("  per-joint tracking error (worst / mean, deg):")
    for k in sorted(per, key=lambda x: -per[x][1] / max(per[x][2], 1)):
        w, tot, n = per[k]
        print(f"    {k:20} {np.degrees(w):7.2f} / {np.degrees(tot/max(n,1)):6.2f}")
    return stats


CACHE = os.path.join(ROOT, "ref", f"pose_seq{_SLUG}.json")

if __name__ == "__main__":
    import json
    if os.path.exists(CACHE) and "--refresh" not in sys.argv:
        blob = json.load(open(CACHE))
        seq, raw = blob["seq"], blob["raw"]
        print(f"loaded cached pose ({len(seq)} frames)")
    else:
        print("extracting pose ...")
        seq, raw = extract()
        json.dump({"seq": seq, "raw": raw}, open(CACHE, "w"))
    got = sum(1 for s in seq if s)
    print(f"  {len(seq)} frames, pose on {got} ({100*got/len(seq):.1f}%)")
    print("driving sim ...")
    st = drive(seq, raw)
    if st["n"]:
        print(f"  mean tracking error {np.degrees(st['err']/st['n']):.2f} deg "
              f"over {st['tracked']} driven frames")
