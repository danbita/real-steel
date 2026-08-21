# shoulder_v6 — shoulder_v3, thinned for the real bicep

**Status: PASSES.** `python sim36/interference_check.py cad/shoulder_v6` → PASS,
tightest pair 1.04 mm across 315 poses of the travel the robot actually uses.

This is **v3's design**, not a new one. Same concurrent axes at C = (0, 0, 100),
same pitch bearing / roll straddle / captured twist race, same mount → carrier →
yoke → platform → interface-plate breakdown. What changed is size, and one
servo orientation that the real servo forced.

---

## Why v6 instead of v5

v5 solved packaging by straddling the arm — yoke either side, roll posts outside
the bicep's swept circle, twist servo buried in a 47 mm shell forming the top of
the upper arm. It clears everything and it is buildable, but it is **150 mm deep**
and the shell reads as a slab. v3 already had the right shape; it was simply
drawn around the *old* 94 × 109 mm bicep.

| | v3 | v5 | **v6** |
|---|---|---|---|
| shoulder-to-shoulder | 352.8 mm / 0.386 H | 305.8 mm / 0.334 H | **312.8 mm / 0.342 H** |
| fore-aft depth | 110 mm | 150 mm | **92 mm** |
| module mass | — | 796 g | **712 g** |
| interfering pairs | never checked | 0 | **0** |

v6 is 40 mm narrower than v3 and 58 mm shallower than v5.

## The servo is measured, not assumed

Every dimension comes from the real STS3215 solid published with the SO-ARM100
([TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100),
`Simulation/SO101/assets/sts3215_03a_v1.stl`), not from a product listing.

| | v3 assumed (MG996R) | **STS3215, measured** |
|---|---|---|
| case | 40 × 20 × 38 | **45.4 × 24.8 × 36.9** |
| output spline offset from body centre | 10.0 mm | **12.5 mm** |
| spline | — | **25T, r 2.7 mm at (12.5, 0)** |
| mass | density-derived ≈ 70 g | **55 g** (datasheet, pinned in prep) |

v3's servo model also unioned a **54 mm mounting flange** onto the case. That is
MG996R geometry — the STS3215 has no protruding tabs, 45.4 mm *is* the whole
part. Carrying it over was adding 9 mm of collision volume that does not exist.

That 12.5 mm offset is the single most consequential number in this module:
every cradle, every clearance and both of the servo re-orientations below follow
from it.

## What actually changed from v3

1. **Bicep 94 × 109 → 50 × 58**, so the interface octagon goes from ±58 mm to
   ±33 mm. The octagon *was* the module's outboard face — this is most of the
   width saving.
2. **Twist bearing 6810 (50×65×7) → 6806 (30×42×7).** A 65 mm race under a 58 mm
   bicep was carrying a load that no longer exists, and its OD was propping up
   the platform's size.
3. **Platform 104 × 84 → 72 × 64**, following the bearing and the octagon.
4. **Torso plate → spine clamp.** v3 bolted to a 110 × 140 mm plate; the skeleton
   spine is 50 × 59 mm, so there was nothing to bolt it to.
5. **Servo R laid down, not inboard.** v3 spun it −90° about Z first, putting its
   45.4 mm length along Y — which with the 12.5 mm offset reaches 35 mm from the
   roll axis, straight through the volume the twist bracket sweeps. Dropping that
   pre-rotation lays the length along Z with the case hanging *below* the shaft,
   so above the roll axis it only reaches 10.2 mm.
6. **Servo T turned 90°** so its length runs outboard instead of fore-aft. Fore-aft
   it reached x = 35.2 mm, into the carrier's front roll plate (x 28–34) and its
   servo-R pad (x 22–30). Turned, its case spans only x ±12.4. Costs 2.2 mm of
   shoulder width and removes three clashes.

## Mounted inverted — and that is load-bearing

v6 is exported in v3's neutral pose with the **arm straight up**, so the humanoid
mounts it rotated 180° about Y (`gen_humanoid_v11.py`). That makes module-roll 0
the robot's resting pose, arm hanging.

This is not cosmetic. **At module-roll 180° the twist servo swings inboard through
the roll servo and the pitch bearing journal** — an unavoidable consequence of a
45.4 mm servo stacked above the platform, and a clash v3 shipped with because
nothing had ever swept it. Mounted inverted, the robot's usable range is
module-roll 0–150° and it never visits that end.

| joint | range | note |
|---|---|---|
| pitch | ±90° | free |
| roll | 0 … 150° | 0 = arm hanging, 150° = near overhead |
| twist | ±90° recommended | model currently ±180°; only ±90 has been swept |

## Measured clearances — worst case over 315 poses

| part A | part B | min gap | at pitch/roll/twist |
|---|---|---:|---|
| yoke | interface_plate | 1.04 | −45 / 0 / 90 |
| servoR_body | yoke | 1.07 | −90 / 150 / −90 |
| yoke | hub_clamp | 1.49 | 45 / 60 / −90 |
| servoR_body | servoT_body | 2.06 | −45 / 120 / −90 |
| servoT_body | hub_clamp | 5.77 | −45 / 40 / −90 |
| carrier | yoke | 6.00 | 45 / 40 / −90 |
| servoP_body | carrier | 6.00 | −90 / 0 / −90 |
| mount | carrier | 8.00 | −90 / 0 / −90 |

Bearing and horn interfaces are excluded by name — a journal turning in its bore
is not a clash.

## The arm is 9 mm long, and why

Upper arm comes out **179.3 mm** against a 170.1 mm target. The module eats 85 mm
from axis to bicep face, and the ATOM bicep's own origin-to-elbow distance is
94.6 mm, so 85 + 94.6 is the floor.

That 9 mm is the price of the real servo: the STS3215 is 45.4 mm long with a
12.5 mm offset where v3 assumed a 40 mm MG996R with a 10 mm offset, and stacking
the twist servo above the platform costs the difference. It is 1% of stature and
the arm reads correctly. Closing it would mean either a shorter twist servo or
moving the twist stage into the bicep — which is what v5 did, and what made v5
look like a slab.

## Bill of materials, one module

| qty | item | note |
|---:|---|---|
| 3 | Feetech STS3215 | 45.4 × 24.8 × 36.9, 55 g, 30 kg·cm ≈ 2.94 N·m, serial bus + magnetic encoder |
| 1 | 6808-2RS 40×52×7 | pitch axis |
| 1 | 6806-2RS 30×42×7 | twist axis, captured both directions |
| 1 | 625ZZ 5×16×5 | roll idler |
| 1 | M5×20 shoulder bolt | roll idler pin |
| 12 | M3 heat-set inserts + M3×8 | servo mounting |
| 6 | M3 heat-set inserts + M3×10 | bearing retainers |
| 4 | M4×12 + inserts | bicep → interface plate |
| 8 | M3×35 | spine clamp |

Printed parts: `mount`, `carrier`, `yoke`, `interface_plate`, `hub_clamp`,
`race_cap`, `pitch_retainer`, `hub_collar`. Mirror the right side with `SIDE = -1`.

## Rebuilding and re-verifying

```
python cad/shoulder_v6/build_shoulder_v6_cq.py      # left module + check_meta.json
python cad/shoulder_v6/right/_build_right.py        # mirrored right module
python sim36/interference_check.py cad/shoulder_v6  # must print PASS
python sim36/prep_v11.py                            # meshes + inertias -> meshes36d
python sim36/gen_humanoid_v11.py                    # URDF
python sim36/build_humanoid_v11_scene.py            # MuJoCo scene
```

**If you change the geometry, re-run the checker.** The table above is measured,
not asserted, and v3 is the cautionary tale: it shipped with a servo bracket
sitting inside the carrier because nobody ever swept it.
