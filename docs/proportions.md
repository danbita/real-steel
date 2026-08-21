# ATOM — full body proportions

**Target stature: 914.4 mm / 36.0 in.**
That is exactly **0.400× Real Steel's Atom** (7 ft 6 in / 2286 mm), so a clean 2/5 scale.

Every length below is a standard anthropometric fraction of stature (Drillis & Contini).
Atom is essentially human-proportioned on screen, so "human proportions" and
"Atom proportions" are the same target — there is no conflict between them.

---

## Full body

| segment | mm | inches | × stature |
|---|---|---|---|
| **overall height** | **914.4** | **36.00** | 1.000 |
| head height | 118.9 | 4.68 | 0.130 |
| shoulder (acromion) height | 748.0 | 29.45 | 0.818 |
| hip height | 484.6 | 19.08 | 0.530 |
| shoulder-to-shoulder (biacromial) | 236.8 | 9.32 | 0.259 |
| chest breadth | 159.1 | 6.26 | 0.174 |
| hip breadth | 174.7 | 6.88 | 0.191 |
| torso (shoulder → hip) | 263.3 | 10.37 | 0.288 |
| **upper arm** (shoulder → elbow) | **170.1** | **6.70** | 0.186 |
| **forearm** (elbow → wrist) | **133.5** | **5.26** | 0.146 |
| hand | 98.8 | 3.89 | 0.108 |
| **thigh** | **224.0** | **8.82** | 0.245 |
| **shin** | **224.9** | **8.86** | 0.246 |
| foot height (ankle) | 35.7 | 1.40 | 0.039 |
| foot length | 139.0 | 5.47 | 0.152 |

Segment stack sums to 867 mm against 914 mm stature; the 47 mm difference is
neck and joint overlap, which is normal for fraction tables.

## The arm, broken down

The upper arm is not one part — it is the shoulder module plus the bicep:

| | mm | inches |
|---|---|---|
| shoulder module (axis centre → bicep mounting face) | 83.1 | 3.27 |
| bicep + elbow bracket | 87.0 | 3.43 |
| **upper arm total** | **170.1** | **6.70** |
| forearm | 133.5 | 5.26 |
| shoulder → wrist | 303.6 | 11.95 |

## As built (humanoid_v8_atom.urdf)

| | target | built | note |
|---|---|---|---|
| overall height | 914.4 | 914.5 | ✓ |
| upper arm | 170.1 | 170.1 | ✓ |
| forearm | 133.5 | 134.1 | ✓ |
| shoulder height | 748.0 | 748.0 | ✓ |
| hip height | 484.6 | 484.6 | ✓ |
| total mass | — | 1.829 kg | skeleton, no shell |

Achieved by two different scale factors, deliberately:

- **arm × 0.531** — sized so the forearm lands on target.
- **bicep sliced, not scaled** — the top is cut off at z = −14.4 mm in the bicep
  frame. Scaling it to length instead would have shrunk the elbow housing to
  ~34 mm, and no real servo fits that.
- **shoulder module × 1.0** — it does *not* scale, because its size is set by the
  servo, which is a fixed physical part. Only the interface plate is scaled
  (× 0.531) so its tray matches the smaller bicep.

## Open problem: the shoulder module is too big

This is the one place the design does not meet proportion, and it is structural,
not cosmetic.

| | as built | human equivalent | ratio |
|---|---|---|---|
| shoulder-to-shoulder over modules | 320.8 mm (0.351 H) | 265.2 mm (0.290 H) | **1.21× too wide** |
| module height | 156.0 mm | — | dominates the upper torso |
| module depth | 110.0 mm | 155.4 mm chest depth | 0.71× — fine |
| module share of upper arm | **49%** | ~45% (deltoid) | acceptable |

The *axial* share is fine — 49% against a human deltoid's ~45%. The problem is
**lateral bulk**: the modules stick out 160 mm each side and visually swamp a
914 mm skeleton.

Root cause: the module was designed around MG996R at person scale and never
re-packaged. It cannot shrink by scaling either, because the servo inside it is
a fixed size.

**The fix is a re-packaged shoulder around the STS3215 (45 × 25 × 35 mm)**, with
the servos tucked alongside or below the axis rather than outboard of it. Target
is roughly 118 mm of lateral half-span instead of the current 160 mm.

## Servo

**Feetech STS3215** — 3.0 N·m stall, serial bus with position feedback, ~$16.

Required torque at this scale, worked backward (torque scales as L⁴):

| load | shoulder torque |
|---|---|
| bare skeleton arm (63 g) | 0.008 N·m |
| with 2.5× shell | 0.021 N·m |
| with 4× shell | 0.033 N·m |

Working torque (⅓ of stall) is 1.0 N·m, so margin is ~30×. The servo is chosen
for **position feedback and packaging**, not torque — open-loop PWM servos
cannot close the mimicry loop.
