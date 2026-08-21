# Atom Design Principles

Every rule here exists because a specific correction during development exposed a
specific failure. The failure is named so the rule doesn't decay into slogan.
These apply to all CAD, animation, verification, and sourcing work on this project.

## 1. Parts are real or they are not in the model

1. **Only measured, purchasable parts.** Every servo, bearing, horn, and screw in
   the CAD carries the dimensions of a specific product you can put in a cart.
   *Origin: the whole project reset around the measured STS3215 (45.4×24.8×36.9,
   12.5mm spline offset) after early models used idealized servos.*
2. **No fictional geometry — ever.** If a rendered solid doesn't correspond to a
   physical part, it is lying to you and will hide a real defect. A "shaft"
   cylinder drawn between a servo and its horn concealed a 4mm air gap on **all
   three drive axes**; the drivetrain physically did not exist and every check
   passed around it. Render the honest 4mm spline nub or render nothing.
3. **A part used as designed beats a part used cleverly.** The servo's own spline,
   its own horn, its own mounting features, its factory center-screw hole. When a
   design routes around the intended interface (horn bolted far from the spline),
   the intended function quietly disappears.
4. **Use the manufacturer's accessories where they exist** (horns, brackets) and
   match printed geometry to *their* published patterns — never invent a bolt
   circle a real part doesn't have. When a spec is unpublished (spline tooth
   count, shaft offset), that's a blocking data gap: get the drawing, don't guess.

## 2. Screws are physics, not decoration

5. **Every screw must land in a real, aligned bore** — verified geometrically
   (shaft-in-void samples, surrounding-ring alignment slabs), not visually.
6. **Length is exact, both ways.** Too long bottoms out before the head clamps
   (M3×45 in a 42.5mm stack; M3×10 in an 8.5mm stack; M3×25 poking into the horn
   space). Too short under-engages. Head-flush-at-seat is an audited equality:
   `length + counterbore = seat`, with nut-clamped screws as the only exemption.
7. **Threads land in metal.** Brass insert, nyloc nut, or a part's own tapped
   metal. Printed threads strip; self-tapping into plastic was first a narrow
   documented exception and later eliminated entirely.
8. **Head type is part of the spec.** Button heads where a cap head fouls a moving
   face (2.5mm retainer gap); countersunk where a plate passes 1.5mm overhead;
   counterbores where a head would sweep another part. Renders distinguish head
   types visually (shape, shade, hex-dot) and every step caption states
   count × thread × length × head type × what it lands in.
9. **One SKU where it's free.** All 34 M3 inserts became a single 4.0mm size
   because no screw engages more than 4mm anyway — the bores didn't change, the
   shopping list collapsed. Consolidate whenever the engineering cost is zero;
   never consolidate by weakening a loaded joint (the bicep M4s stayed).

## 3. Assembly is a physical process, not a diagram

10. **Approach corridors are part of the design.** Everything inserted — screw,
    insert, soldering iron, part — needs a verified clear path *including the
    tool*: the driver behind the screw, the iron behind the insert (angled
    approach counts; verify the angle exists). A screw with no corridor forced
    preloading (M3×40s riding in the servo case past a 21mm gap); a horn with no
    driver window forced bench-fitting; an 83mm bolt with a 70mm corridor forced
    the studs-first spine sequence.
11. **Assembly order is geometry, discovered not asserted.** Race cap before the
    plate (the plate shadows its screws); bicep before the plate is installed
    (counterbores unreachable after); horns bench-fit before their parts install;
    all inserts before any assembly (the iron can't reach half the bores later).
    Each ordering rule exists because the reverse order was *proven* impossible.
12. **The build compounds.** Once installed, every screw and insert persists in
    all later scenes, rides with its parent part when it moves, and collides like
    any other body. Checking steps in isolation hid real interferences (a plate
    descending over installed screw heads) and confused viewers ("where did the
    screws go?").
13. **Designed running clearances are explicit, not waved through.** A 0.5mm
    coaxial gap between a horn face and its pocket floor is legitimate — so it's
    named in the CAD, and the interference checker polices it with a per-pair
    minimum instead of either failing it blindly or excluding it silently.
14. **Verify at full range of motion, both hands.** Mirrored parts mirror their
    sweep axes; a check that passes with the wrong-handed axes is a check that
    didn't run.

## 4. Look at it. Actually look at it.

15. **Math passing is not the same as correct.** Extract frames and look with
    eyes: the wrong-leg-animating bug, the invisible idler press, and the
    occluded pitch horn were all caught visually after every numeric check
    passed. Every render pass ends with a human-standard visual audit.
16. **Depth ambiguity reads as clipping.** A screw drawn in front of an unrelated
    bright part, with no occlusion cue, is indistinguishable from a screw through
    solid — and the viewer is right to reject it. Choose cameras where moving
    hardware reads against the thing it enters. Physically-clear-but-looks-wrong
    is still wrong.
17. **Camera discipline:** one still camera that keeps all fasteners legible;
    jump-cut only when something is genuinely invisible; end every step with a
    small zoom-out orbit so the viewer recovers orientation. Close-ups for the
    engagement moments (spline meeting horn) — the connection must be *shown*,
    not implied.
18. **Show the invisible functions.** Heat-set inserts get their bore mouth
    rendered (a solid brass pin tells you nothing about which side takes the
    screw); the iron appears and presses each one; drive connections get explicit
    "this attaches HERE" beats. If a mechanism never visibly connects, a builder
    will correctly refuse to believe it.

## 5. Loads and sourcing

19. **Torque math honestly:** worst pose (arm straight forward), shell mass
    budgeted, distal servo masses included, 2× dynamic factor for punching,
    empty hand (no payload — the robot carries nothing). kg·cm means kg × cm of
    lever: a 1kg arm at 15cm *is* a 30 kg·cm problem.
20. **Spec per joint, not per robot.** The shoulder's 32 kg·cm applied to all six
    "arm servos" bought strength nobody needed — and the count itself was wrong
    (the arm actuates 4: three shoulder + one elbow; forearm gearing is passive).
    Count the actual joints; spec each one.
21. **Requirement-first sourcing, never incumbent-first.** Naming the current
    part in a scout brief biases the whole search: the anchored search returned a
    servo 6% *under* spec for *more* money than the clean search's spec-meeting
    winner. State the requirement; let every candidate compete; the incumbent's
    only privilege is a "no rework" tradeoff line on its card.
22. **Buy exactly what the build needs.** The metric is landed cost to cover one
    arm; packs win only when they beat that or the overshoot feeds arm two.
    Distrust listing metadata: packaged weight isn't servo weight, copy-pasted
    voltage bullets lie, pack size hides in variant selectors.
23. **Cost lives in line-item count.** Amazon's ~$6–7 per-listing floor means the
    cheap path is fewer distinct specs (head-type consolidation), not cheaper
    individual bags.

## 6. Process discipline

24. **Every automated edit asserts its match.** Unverified `replace()` calls
    no-op silently — one un-asserted patch left horn holes drilled into empty
    air while everything reported success. If a patch can't find its pattern,
    it must fail loudly. And remember: one replacement can invalidate the next
    pattern in the same file.
25. **Agents die and drift; supervise them.** Heartbeat their transcripts,
    resume stalled fleets from cache, spot-check their claims against the
    artifacts. Give review agents explicit permission to say "already good" —
    forced findings are fabricated findings.
26. **When a check fails, find the mechanism before touching anything.** Every
    deep dive in this project ended somewhere unexpected: a "clipping screw" was
    an occluded camera; a "penetration" was a classifier lying near open
    channels; a "reverted GIF" was palette compositing. The fix applied before
    the diagnosis is usually wrong.
