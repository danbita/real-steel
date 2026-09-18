# Atom's Neck

A 2-DOF neck, driven two ways: it can **follow a person around the room**, or it
can **mirror your own head movement**. Tracking is the primary behaviour.

Everything here is built from `NeckAssembly_files/`, the SolidWorks export.
**That export is never modified.** It is read at generation time and re-emitted
as a drivable model; its geometry is carried across untouched and a test asserts
exactly that.

---

## Quick start

```bash
# 1. build the model and the physics scene  (needs mujoco)
python sim36/gen_neck.py
python sim36/build_neck_scene.py

# 2. confirm the axes, no camera needed
mjpython "live mimic system/neck_receiver.py" --selftest

# 3a. TRACKING - the neck follows a person
python  "live mimic system/neck_tracker_sender.py" --fov 60
mjpython "live mimic system/neck_receiver.py"

# 3b. MIRRORING - the neck copies your head pose
python  "live mimic system/pose_sender.py"
mjpython "live mimic system/neck_receiver.py"
```

Run **one sender at a time**. Both write to `udp://127.0.0.1:5005`, so two of
them means two sets of commands racing on the same joints.

---

## The two behaviours

### Tracking (primary)

| situation | what the neck does |
|---|---|
| empty frame | rests looking straight ahead |
| someone appears | turns to them, locks after 3 consecutive frames |
| other people walk in | **ignored** — a lock is never stolen |
| locked person leaves | after 0.5 s grace, takes the largest remaining person |
| nobody left | returns to neutral after 0.8 s |

The preview window rings the locked person in green, everyone else in grey, and
prints the state and commanded angles.

### Mirroring (kept, secondary)

Measures your head's orientation **relative to your own torso** and sends it to
the neck, so leaning, walking around, or tilting the camera changes nothing —
the same property the arms already had. Holds still for ~1.5 s at startup while
it captures your resting head pose; that is deliberate, not a hang.

---

## Numbers you have to set

Four values are not derivable from anything in the repo. Three of them are
currently placeholders.

| what | where | now | how to settle it |
|---|---|---|---|
| **Camera horizontal FOV** | `--fov`, default 60 | guess | Stand at the exact left edge of frame; compare the commanded angle with the real camera→you angle. Webcams run 55–78°. |
| **Servo travel** | `SERVO_TRAVEL_DEG` in `gen_neck.py` | 180° | The export states nothing (see below). If your DS51 is the 270° variant, change this one number — limits are 90% of it. |
| **Axis signs** | `SIGN_YAW` / `SIGN_PITCH` in `neck_tracking.py`, `SIGN` in `pose_sender.py` | +1 | First live run. If the neck turns *away* from the person, flip. |
| **Head mass** | `HEAD_MASS` in `gen_neck.py` | 0.25 kg | No head part exists in the CAD folder yet. |

FOV is the one that matters most day to day: it alone decides how far the neck
turns for a given position in frame.

---

## Why `gen_neck.py` exists

The raw export carries good geometry and unusable kinematics. Four problems,
all verified rather than assumed:

**Flat topology.** All 8 joints are parented to `neckcover_link`, so nothing is
downstream of anything. Turning a joint spins one part in place and the head
never moves. This is not a description of the hardware — it is a description of
nothing, and it is why a re-chaining step is unavoidable.

**No head link.** Nothing represents where the face ends up, which is the one
frame tracking has to aim.

**No travel information.** Every movable joint is `continuous`, and the
exporter's own table (`NeckAssembly_files.csv`) has Limit Lower, Limit Upper,
Limit Effort and Limit Velocity blank on every row.

**Parts exported twice.** `YesHolder2_Link` is a merged sub-assembly. Two
independent proofs:

```
triangles   servo(11758) + YesHolder3(3018) + MotorHolder3(240) = 15016
            YesHolder2_Link                                     = 15016   exact

mass        0.0887120062426798 + 0.21367369731836 + 0.00685662156823661
                                        = 0.309242325129276
            YesHolder2_Link             = 0.309242325129277   (6.1e-16 kg apart)
```

So `servo3_Link`, `YesHolder3_Link` and `MotorHolder3_Link` are the *contents* of
`YesHolder2_Link`, emitted a second time as loose siblings. Summing the export
naively double-counts the hardware: **1.3601 kg claimed, 1.0508 kg real.**

**Worth checking against the physical build** — this is the only change that
alters content rather than structure. If those are three separate parts on your
bench, the model is missing hardware.

### What survives unchanged

- every mesh at **exactly** its exported position (asserted to 1e-12 m)
- joint axes **verbatim**, including the 2.28° whole-assembly tilt
- masses and inertia tensors, parsed from the export at generation time

Factoring the tilt out into a joint `rpy` was tried and reverted: a joint's rpy
rotates the whole child link, meshes included, so it would have swung all the
geometry by 2.28°.

### Why 2 DOF and not 3

The CAD part names say it: `YesHolder.SLDPRT` (nodding yes) and
`NoHolder.SLDPRT` (shaking no). Once the double-counted siblings are removed,
exactly two moving groups remain. The third axis belongs only to the duplicates.

Roll is measured by the mirroring path but not sent. Set `ENABLE_ROLL` in
`gen_neck.py` and `SEND_NECK_ROLL` in `pose_sender.py` if the bench says
otherwise.

`NoHolder.SLDPRT` and `NeckConnector.SLDPRT` are in the CAD folder and **are not
in the export at all**, so the export is also incomplete.

---

## Files

**New**

```
sim36/gen_neck.py               the export -> a drivable 2-DOF chain. Edit THIS,
                                never the .urdf it writes.
sim36/build_neck_scene.py       chain -> physics scene (actuators, head site)
sim36/neck_chain_check.py       proves the re-chaining. numpy only.
sim36/neck_retarget_check.py    proves the head-angle math. numpy only.
sim36/neck_tracking_check.py    plays whole tracking scenarios. numpy only.

live mimic system/neck_tracking.py         tracking decisions. No camera deps.
live mimic system/neck_tracker_sender.py   camera -> candidates -> UDP
live mimic system/neck_receiver.py         the neck in MuJoCo (+ --selftest)
live mimic system/neck_v1.urdf             GENERATED
sim36/neck_scene.xml                       GENERATED
```

**Modified**

```
live mimic system/pose_sender.py    + head_frame(), neck_angles(), NeckNeutral,
                                      and neck entries in SIGN/LIMITS/JOINT_NAMES.
                                      Nothing existing was changed, so the other
                                      receivers are unaffected.
sim36/gen_humanoid_v11.py           neck mounted on the torso; the head moved onto
                                      it; placeholder neck stub removed
sim36/build_humanoid_v11_scene.py   neck actuators, neck collision off
```

`neck_tracking.py` has no camera dependencies on purpose — that is what lets the
scenario tests exist. Waving at a webcam cannot test "two people cross paths"
repeatably.

---

## Tests

```bash
python3 sim36/neck_chain_check.py       # the model
python3 sim36/neck_retarget_check.py    # the mirroring math
python3 sim36/neck_tracking_check.py    # the tracking behaviour
```

All three need **numpy only** — no camera, no MuJoCo, no mediapipe (it is
stubbed, so the code under test is the real sender, not a copy). Highlights:

- every mesh lands at its exported position, worst error **6e-13 m**
- 12 arbitrary whole-scene rotations agree to **5e-14°** — the camera-invariance
  claim, measured
- 75 head poses round-trip through the sender and come back out
- tracking: acquire, follow, ignore a closer person, survive a detection blink,
  re-target on loss, return to neutral, two people crossing, rate limit, clamps

---

## Two things that bit, in case they bite again

**MuJoCo welds fixed joints away.** The `head` link vanishes as a body,
`mj_name2id` returns −1, and code that trusts it silently reads the *world* body
instead. The scene builder re-attaches the head as a **site**.

**A nested assembly starts out interpenetrating.** Holders inside the cover,
servos inside the holders — 16 contacts at the zero pose, and `neck_yaw` stalled
at **1.53°** no matter what it was commanded. The actuators were not weak; they
were fighting the base. Collision is off, the same call
`build_humanoid_v11_scene.py` already makes for the shoulder. Real clash checking
belongs in a CAD sweep like `sim36/interference_check.py`.

---

## Atom is now 40 inches

The neck is 124.8 mm tall and 164.0 mm wide, against 94.1 mm of
shoulder-to-head-centre and a 143.7 mm head. Mounted unscaled it pushes the crown
from **914.4 mm (36.0 in) to 1016.5 mm (40.0 in)**. `gen_humanoid_v11.py` prints
this every run.

Nothing was scaled. Shrinking the mesh would make the picture tidy and the sim
meaningless — you cannot shrink a DS51. Three servos at 61 × 46 × 23 mm each is
the root cause; if Atom must stay at 36 in, the servos are what has to change.

One trap worth knowing: v11 sets `meshdir="meshes36d/"`, and URDF `strippath`
defaults to **true**, discarding everything but a mesh's basename. Harmless for
v11's own meshes, fatal for the neck's relative path. `strippath="false"` is now
set; without it the model does not load at all.

---

## Open

- **FOV, servo travel, axis signs** — see the table above
- **Duplicate removal** — confirm against the physical build
- **Camera mount** — currently assumed fixed and off-board. If it moves to the
  head, `HEAD_MOUNTED` in `neck_tracking.py` is the switch and `aim_angles()` the
  only function that changes. It is not cosmetic: on-board, a centred target
  reads as zero bearing wherever the neck points, so the command becomes
  error-driven rather than absolute, and a target leaving frame is genuinely
  gone.
- **Apparent size as a distance cue** is imperfect — turning side-on narrows the
  shoulders. It only picks who to *acquire*; it never breaks a held lock.
- **Crossing paths** is handled by position continuity, since MediaPipe does not
  promise stable ordering between frames. Two people passing very close could
  swap the lock (`MAX_ASSOC_JUMP`).
