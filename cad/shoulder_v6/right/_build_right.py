"""
shoulder_v6 - shoulder_v3 geometry, RE-SYNCED to the Feetech STS3215.

WHY THIS REWRITE
----------------
v6 inherited every mounting feature from v3, which was drawn around an MG996R.
The servo then changed to an STS3215 (45.4 x 24.8 x 36.9, output spline 12.5 mm
off the body centre) and nothing downstream was re-cut. The result passed the
sweep checker only because the checker never compared a servo with the bracket
that holds it, never noticed a bolt circle that does not exist, and never looked
at a part that was named but not exported.

FIXED IN THIS PASS  (labels match the build review)
  A  servo R / servo T no longer collide with their own brackets. Every servo
     sits in a pocket cut from the real 45.4 x 24.8 x 36.9 case with
     POCKET_CLR = 1.6 mm all round, and bolts to a face plate at its flange.
  B  THE DRIVE JOINTS: A REAL HORN, AND A TWIST DRIVE INVERTED TO SERVO-FIRST.
     Two corrections live here and they compound.
     (1) THE HORN IS A HUB AND A DISC, NOT A FLAT PUCK. It was modelled Ø20 x
     3.0 for two revisions and every driven plane in the module hung off that
     fiction. The real generic 25T aluminium disc horn is a Ø7.3 SPLINED HUB,
     HORN_HUB_H tall, standing on the servo's Ø12.5 output boss, with a
     Ø19.7 x 2.2 DISC hung under the hub's TOP face. What that changes:
       - THE DRIVEN FACE IS THE HUB TOP, at case_top + BOSS_H + HORN_HUB_H, not
         3.0 mm above the case top. Every driven plane moved outboard - carrier
         journal end -42.1 -> -39.4, yoke roll boss face 36.4 -> 39.1,
         hub_clamp underside 175.5 -> 177.1 - and each is now DERIVED from its
         servo's case top plus HORN_FACE, so the three cannot drift apart
         again. They already had: HC_Z0 was a literal and was 1.1 mm stale.
       - THE SPLINE DISAPPEARS INSIDE THE HUB. It stands SPLINE_TIP = 5.1 proud
         of the case top; the hub is 7.2, so the tip finishes 2.1 mm down the
         bore. Engagement is the FULL 3.6 mm of live serration. HORN_LIFT -
         which traded engagement against running clearance and had already been
         driven negative - is gone, and so are all three spline-tip reliefs.
       - THE DRIVE SCREW HEADS WERE SITTING INSIDE THE SERVO. On pitch and roll
         the four M3 pass DOWN through the disc, so their heads hang beneath it
         over the ring r 4.1..9.9, while the Ø12.5 boss fills r < 6.25 up to
         case_top + 1.5. The two rings overlap radially and the boss is round,
         so there is no clocking out of it. At the first HORN_HUB_H estimate of
         4.5 the heads reach case_top + 0.8: 0.7 mm INSIDE the boss - the horn
         cannot seat and the drive does not exist. HORN_HUB_H = 5.7 is the
         smallest hub that clears it, and it clears it by exactly 0.5 mm.
         THAT NUMBER IS UNPUBLISHED AND ESTIMATED. Read the box at the constant
         and put calipers on the real horn before printing anything.
       - THE Ø23 POCKETS ARE HEAD-CLEARANCE POCKETS NOW, floor at
         case_top + HORN_FLOOR, which is exactly the boss top plane. The DISC
         itself no longer touches them: it clears the pitch plate by 2.4 mm,
         the carrier plate by 1.6 and the twist bracket by 3.0. All three
         min_override entries retire, and the twist bracket loses its pocket.
     (2) THE TWIST DRIVE IS ASSEMBLED SERVO-FIRST. It used to be bench-fit like
     the other two - horn bolted to hub_clamp's underside into four heat-set
     inserts, then the pair lowered onto the spline blind, with no way to see
     or reach the engagement. The order is now:
        servo T into its bracket  ->  THE DISC ONTO THE VISIBLE SPLINE, held by
        the servo's OWN FACTORY M3x6 CENTRE SCREW while nothing stands above it
        ->  hub_clamp lowered onto the disc  ->  4x M3x6 driven DOWNWARD
        through counterbored Ø3.4 holes in the clamp into THE DISC'S OWN TAPPED
        M3 HOLES  ->  6806, race cap, plate as before.
     Aluminium threads in a bought part, used the way its maker cut them: no
     drill-out on that disc, no insert on that joint, no bench sub-assembly,
     and the factory centre screw - dead weight on all three axes before - does
     real retention work on one of them. hub_clamp loses four insert positions
     and the module drops from 34 M3 inserts to 30.
     PITCH AND ROLL STAY BENCH-FIT, and the reason is structural, not habit:
     THEIR DRIVEN PARTS ARE MONOLITHIC. The carrier journal end face and the
     yoke roll boss are solid faces on one-piece parts, so there is no second
     piece to lower on afterwards. The horn has to go on before the spline
     does, its four tapped holes still have to be drilled out to Ø3.2, and its
     centre screw is blind the instant the spline engages. INVERT A DRIVE JOINT
     ONLY WHERE THE DRIVEN PART ARRIVES IN TWO PIECES.
     THE TWIST HORN IS CLOCKED 45 deg (HORN_A_T). The disc is 4-fold symmetric
     so it costs the builder nothing, and it is what lets the four drive screws
     clear hub_clamp's three r=11 plate-bolt bores - see block E.
  C  THE EAR DECK - the servo mount architecture, replacing the flange/face
     plate on all three joints. The HTS-35H is not a flangeless brick like the
     STS3215: it carries the standard 54.38 mm ear plate with four O4.6 open
     keyhole slots on a 49.5 x 10 pattern, its ear BOTTOM 12.6 mm below the case
     top and its ear TOP 9.5 mm below it, with a 1.7 x 2.2 stiffening rib
     standing proud of that top face for almost the whole span. So the servo is
     mounted the way its maker intends: it inserts SPLINE-FIRST and its EAR TOPS
     stop against a DECK, a DECK_T = 6.0 plate whose bearing face sits at
     case_top - 9.5. The deck carries three features and nothing else:
       - a CASE CUTOUT, SRV_L+0.7 x SRV_W+0.6 about the BODY centre (40.64 x
         20.64), which the case slips through and which continues past the deck
         as the case-top relief, stopping CASE_CLR = 0.6 above the case top;
       - a GUSSET RELIEF, 3.0 wide x 2.5 deep along the ear centreline, without
         which the servo stands on its rib and the ears never touch anything;
       - 4x M3 inserts, TAB_INS_D deep, on tab_points()' ear pattern.
     4x M3x8 SOCKET CAP clamp from the EAR BOTTOM up through the servo's own
     slots. That is the whole fixing, and three defects die with the flange.
     No screw enters the servo case (the twelve M3x40 are gone, and with them
     the preloaded-rider dodge that a 21 mm approach gap forced). No servo
     interpenetrates its own bracket - the old plates had their inner faces at
     case_top - 4 and each case sat 2.9..4.0 mm INSIDE them, hidden because the
     checker skips bonded pairs; measured overlap is 0.000 mm^3 on all three
     now. And no insert is anywhere near a horn: the deck bores point AWAY from
     the drive, ending 4.5 mm from the nearest head-pocket floor and 9.0 mm
     from the nearest disc underside, so the 0.5 mm depth-separation dance that
     TAB_INS_D = 4.5 used to perform is gone and all 30 bores are 5.5 again.
     The deck also has to be REACHED, which is what drives each joint's
     geometry: the pitch servo is clocked 90 deg so its 54.38 mm ear span rides
     Z instead of trying to cross a 50 mm spine pocket, the carrier's servo R
     face plate grows to z 61.5..118.5, and the twist bracket widens to
     y -18.5..38.5. See ASSEMBLY ORDER 3, 11 and 12 for the corridors.
  D  the interface plate's rotating shoulder is Ø34 (it bears on the INNER race
     only) and the race_cap ID is Ø39, so the cap grips only the outer race
     (Ø39..Ø42) and clears the rotating flange by 2.5 mm. The cap's own bolt
     circle used to be at r=39, outside its own Ø56 rim, so it had no bolt holes
     at all; it is r=25 now, with matching inserts in the platform.
  E  hub_clamp is bolted to a SOLID Ø30 journal on a r=11 circle (Ø22, was
     Ø18). The clamp's own bores span r 9.75..12.25 in the Ø36 clamp; the
     interface plate's Ø3.4 through-holes span r 9.3..12.7 in the Ø30 journal,
     a 2.3 mm wall, and that 2.3 (from 4.3) is still the price of the move.
     Those three M3 carry the hanging arm in tension - plate -> M3 ->
     hub_clamp -> inner race -> shoulder - and at r=13 they broke straight out
     through the journal wall.
     WHAT THEY NOW SHARE THE PART WITH is four Ø3.4 DRIVE THROUGH-HOLES on the
     Ø14 circle, counterbored Ø5.8 x 3.0 from the TOP face, not four heat-set
     inserts coming up from the underside. Same clash, different shape. The
     45 deg clocking is still forced - a 3-hole circle reduces mod 90 deg to
     phi, phi+30, phi+60, so 15 deg is the best separation any phi can buy
     against a 4-hole 90 deg circle, and 0/120/240 against the 45 family
     already is 15. Closest pair is 4.6095 mm on centres, and the wall is
     0.4595 mm against the counterbore over the top 3.0 mm, 1.6595 below it.
     That 0.46 is the tightest feature in the part. It is also the hard limit
     on this circle: at Ø4.2 for heat-set inserts the counterbore breaks in,
     and the escape is HC_BC = 12.0 at the cost of a 1.3 mm journal wall.
     See the worked note at the hub_clamp code.
  F  the 4 M4 bicep bolts are COUNTERBORED into the underside of the interface
     plate, so no head projects into the race_cap's space. See ASSEMBLY ORDER.
  G  spine sandwich: this mount is a half shell. Four M3x80 run through it, the
     spine (drilled 3.4 mm) and the OTHER arm's half shell into nyloc nuts, and
     no M3x35 could ever reach. The four bores MOVED, from (+/-20, C_Z+/-24) to
     (+/-22, C_Z+/-34). At the old positions the pitch housing boss (r=33, face
     at y=-43.0) closed the bolt's approach 39.99 mm outboard of the entry face
     and an M3x80 plus head needs 83 mm of straight run - not one of the four
     could be started. The new ring clears the boss by 4.3 mm.

ASSEMBLY ORDER (this is not optional - every step below is a corridor that was
probed, not a preference)
  1  heat-set all 30 inserts. It was 34: hub_clamp's four twist-horn positions
     are deleted, because that joint now lands in the horn's own aluminium.
  2  BENCH-FIT TWO HORNS - PITCH AND ROLL, NOT THREE. Both of those driven
     parts are MONOLITHIC, so their horn cannot be fitted after assembly: the
     pitch disc's r=7 screws end up shadowed by the mount's servo P plate and
     the roll disc's by the carrier front plate. PREP: drill THOSE TWO discs'
     four M3 TAPPED holes out to Ø3.2 - THE TWIST DISC STAYS TAPPED, it is the
     one joint that uses its own threads. Then 4x M3x6 SOCKET CAP straight
     through each disc into the driven part's four inserts: pitch disc onto the
     carrier's journal end face (y = -39.4), roll disc onto the yoke boss face
     (x = 39.1). Do NOT fit the factory M3x6 centre screw on either - it is
     blind the instant the spline engages, and both axes are retained
     structurally anyway (6808 + pitch_retainer; yoke + idler pin).
     THE DRIVE ITSELF IS NOT MADE HERE. It completes later, by SPLINE
     ENGAGEMENT, when servo P is fed into the mount (pitch) or servo R is
     pushed +X (roll) and the 25T spline enters the horn's HUB. Expect the hub
     to bottom on the servo's Ø12.5 output BOSS, not on the spline tip: the tip
     finishes 2.1 mm down inside the bore and it is meant to.
  3  servo T rises into its bracket until the ears land on the 169.4 deck;
     4x M3x8 up through the ear slots into the deck inserts. TWIST IS BUILT
     SERVO-FIRST FROM HERE, and steps 4 and 5 are the whole reason.
  4  TWIST DISC ONTO THE SPLINE, ALONE, AND SCREW ITS CENTRE. Press the third
     25T disc - undrilled, still tapped - down the Ø39 platform/bracket bore
     onto servo T's spline until its hub bottoms on the output boss at 171.4.
     Then run the servo's OWN FACTORY M3x6 CENTRE SCREW into the shaft. THIS IS
     THE ONE MOMENT IT IS REACHABLE: nothing whatever stands above the disc,
     the corridor is the open Ø39 bore straight to daylight, and one step later
     hub_clamp closes it for good. Torque it now. This screw is the only one in
     the module that does its work inside a bought part's own thread, and it is
     the difference between a drive you can see engage and one you hope did.
  5  hub_clamp DOWN ONTO THE DISC, then 4x M3x6 SOCKET CAP driven DOWNWARD.
     The Ø36 clamp drops from above through the still EMPTY Ø42 bearing seat
     and the Ø39 bores - it cannot pass a fitted 6806, so it goes in before the
     bearing, not after - and lands flat on the disc top at 177.1. Its
     underside carries a Ø8 x 3.5 relief so the step-4 centre screw head cannot
     hold it off, whatever the horn's own recess turns out to be. Then the four
     drive screws, down the Ø5.8 x 3.0 counterbores in its top face, through
     4.4 mm of clamp and into THE DISC'S OWN TAPPED M3 HOLES - 1.6 mm of
     aluminium thread each, tips stopping 0.6 mm short of the disc's far face.
     Heads finish flush with 184.5, which they must: that face is the seat for
     the interface plate's journal. M3x6 AND NOT M3x8: pitch and roll are
     FORCED to 6 - 2.2 of disc plus a 5.5 insert bore is 7.7 mm of stack and an
     8 bottoms before its head clamps - and at 6 the twist screw is wholly
     contained inside the clamp/disc sandwich. An 8 would physically fit here
     (full 2.2 mm of thread, tip still 1.6 mm off the bracket plate) but it
     leaves 1.4 mm of ROTATING screw hanging in the drive gap, a solid no sweep
     in this project models, to buy 0.6 mm of thread the joint does not need at
     122 N per screw. One length for all twelve instead.
     There is no bench sub-assembly here and no blind lowering onto a spline
     nobody can see.
  6  bicep -> interface_plate: 4x M4x16 UP from the plate underside into the
     bicep's inserts, heads counterbored flush. Do this BEFORE the plate goes
     near the bearing - the counterbores are reachable only from below.
  7  6806 into the platform seat, resting on the platform lip.
  8  RACE CAP NOW, AND ITS FOUR SCREWS, BEFORE THE PLATE GOES ON. 4x M3x8
     COUNTERSUNK down the r=25 circle into the platform's inserts. Not a
     preference: the screws enter at z = CAP_Z1 heading -Z and an M3x8 plus
     head needs ~11 mm of straight approach, and once the interface plate is
     on there is solid plate from 1.5 mm above that entry plane (the plate's
     bicep counterbore leaves a 200.5..202.0 sliver right over the bolt
     circle) and bicep_ref above that. Probed at all four angles: plate off,
     the corridor is void to +40 mm; plate on, blocked. Rotating the bolt
     circle does not help - the whole r=25 circle lies inside the octagon.
     The cap never has to come off again: the plate's Ø30 journal and its Ø34
     rotating shoulder both drop straight THROUGH the cap's Ø39 ID.
     Countersunk, not socket cap - the plate underside runs 1.5 mm over the
     cap's top face and sweeps the entire bolt circle at every twist angle.
  9  interface_plate journal down through the capped 6806, then 3x M3x20 down
     from the plate's top face into hub_clamp's bores. That top face carries a
     Ø5.8 x 3.0 counterbore at each of the three positions, so the heads finish
     flush with IF_Z1 and never intrude on the bicep cavity: head seat 199.0,
     tip 179.0, bore floor 177.5 - 1.5 mm of margin. An M3x25 into a 6.0 bore
     bottomed 1.5 mm early. hub_clamp is 7.4 mm thick now, and the four DRIVE
     holes from step 5 share its TOP 3.0 mm with these three: closest pair
     4.6095 mm on centres, wall 0.4595 mm against the Ø5.8 counterbore. That
     wall is the tightest feature in the part - print it solid.
 10  pitch: carrier (disc already on) journal through the 6808, then
     pitch_retainer 3x M3x8 BUTTON HEAD. Button, not socket cap: the carrier
     bridge passes 2.50 mm off the retainer's outer face and a 3.0 mm socket
     head fouls it by 0.5 mm.
 11  roll: servo R IN TWO MOVES, and it is two because the idler plate closes
     the straight run. Lower the servo into the 51.40 mm window between the
     idler plate's outer face (x = -29.0) and the deck face (x = 22.4) from
     +Y - the only open side, the carrier bridge occupies -Y from y = -15.5 -
     with the case lying x -19.5..22.4. Then push it +X exactly 11.0 mm: the
     ears land on the 22.4 deck and, in the last 3.6 mm of that same push, the
     spline runs up inside the yoke's already-horned roll HUB. THE YOKE MUST
     ALREADY BE ON: the disc is pinned to the yoke boss, so this push IS the
     drive engagement, and it ends with the hub bottomed on the output boss. Probed - straight -X retract stays clear to 20.5 mm (0.5 mm
     off the idler plate) and the +Y lift-out is clear to 55 mm at the 11 mm
     station. Then 4x M3x8 from -X through the ear slots, then M5x25 + M5
     nyloc + a O5x4 spacer on the idler side - read the roll-idler note in the
     buy list, the CAD gives that pin no thread at all.
 12  servo P into the mount, CLOCKED 90 deg about its own spline so the body
     and the 54.38 mm ear span run along Z (body z 90.0..129.94, ears
     82.78..137.16) and only the 20.04 width runs along X. Straight up the
     channel from under the spine pocket, spline-first, until the ears land on
     the -56.1 deck; 4x M3x8 up through the ear slots at (+-5, 85.22) and
     (+-5, 134.72). Straight-line corridor, probed clear to 55 mm. NEVER once
     the module is on the spine - the spine fills the approach.
 13  SPINE SANDWICH, LAST, AND ONLY ONCE BOTH ARM MODULES ARE COMPLETE,
     step 12 included. Offer both half shells to the spine and run
     4x M3x80 + M3 nyloc straight through shell + spine + shell at
     (+/-22, C_Z+/-34). Park the pitch joint at 0 deg for three of them; the
     (+22, C_Z-34) bolt is run with pitch swung to 180 deg, where servo R's
     case has left its 83 mm approach. No single pitch angle clears all four -
     swept every 5 deg from -60 to 180 - but 0 and 180 between them do.

Units mm, Z up, REP-103 (X forward, Y left). Exported in the NEUTRAL pose with
the arm straight UP. SIDE = +1 builds the LEFT module; SIDE = -1 mirrors it.

BUY LIST (one module)  - every length below is a measured stack, not a guess
---------------------
  3x Hiwonder HTS-35H  39.94 x 20.04 x 40.40 case, 54.38 ear span, spline
                       10.0 mm off the near end face, 35 kg.cm @ 11.1V
  1x 6808-2RS  40x52x7   pitch axis
  1x 6806-2RS  30x42x7   twist axis
  1x 625ZZ     5x16x5    roll idler

  12x M3x8 SOCKET CAP      SERVO MOUNTING, 4 per servo, driven from the EAR
                           BOTTOM through the servo's own O4.6 open slots into
                           the ear-deck inserts. Stack: 3.1 mm of ear + 4.9 mm
                           into a 5.5 bore, so the whole 4.0 mm insert is
                           engaged and the tip stops 0.6 mm short of the floor.
                           M3x10 bottoms. NOTHING ENTERS THE SERVO ANY MORE -
                           the 12x M3x40 that used to cross the whole case are
                           gone with the flange architecture, and with them the
                           preloading dodge that a 21 mm approach gap forced.
                           The rubber GROMMETS and brass eyelets in the servo
                           box are NOT USED: they are vibration isolators and
                           this is a drive joint on a punching arm. Bin those.
                           KEEP the little M3x6 centre screws, though - one of
                           the three now retains the twist disc.
  3x  M3x8 BUTTON HEAD     pitch_retainer. Was M3x10 - 3 mm retainer + 5.5 mm
                           insert is 8.5 mm of stack and 10 bottoms. The head
                           must be low: the carrier bridge sweeps 2.50 mm off
                           the retainer's outer face, a 3.0 mm cap head fouls.
  4x  M3x8 COUNTERSUNK     race cap. Was M3x10 - 3 mm cap + 5.5 mm insert, 10
                           bottoms. Countersunk because the interface plate's
                           underside runs 1.5 mm over the cap's top face.
  3x  M3x20                interface_plate -> hub_clamp, COUNTERBORED O5.8 x 3.0
                           into the plate's top face. Head seat 199.0, tip
                           179.0, bore floor 177.5. Was M3x25 into a 6.0 bore,
                           which bottomed 1.5 mm early. hub_clamp is 7.4 mm
                           thick now - it lost 1.6 when the taller hub horn
                           pushed its underside up - and the 177.5 floor is
                           fixed by the M3x20 tip at 179.0, not by thickness.
  3x  25T HORN DISC        generic aluminium disc horn, and IT IS A HUB AND A
                           DISC, not a flat puck: O7.3 splined hub HORN_HUB_H
                           tall over an O19.7 x 2.2 disc, 25T / O5.9 bore, 4x M3
                           TAPPED holes at 90 deg on an O14 circle through the
                           2.2 mm disc. The hub bottoms on the servo's O12.5
                           OUTPUT BOSS and swallows the whole spline - the tip
                           finishes 2.1 mm down inside the bore, which is
                           correct and is not a part pushed on wrong.
                           HORN_HUB_H IS UNPUBLISHED. The CAD needs 5.7 and no
                           vendor prints it. CALIPER THE HUB BEFORE PRINTING
                           ANYTHING - a short hub drives the pitch and roll
                           screw heads into the servo's own boss. Read the box
                           at the constant.
                           PREP, AND IT IS NOW ONLY TWO OF THE THREE: drill the
                           PITCH and ROLL discs' four tapped holes out to O3.2
                           so their drive screws pass clean through into the
                           driven part's inserts. THE TWIST DISC STAYS TAPPED -
                           its four holes ARE the thread that joint lands in.
                           The twist disc's hole pattern is clocked 45 deg in
                           the CAD; the disc is 4-fold symmetric, so the part is
                           identical and there is nothing to orient.
  1x  M3x6 CENTRE SCREW    THE TWIST DISC'S RETAINER, and it is the servo's own
                           factory screw - it ships in the horn bag, so this
                           line costs nothing. It goes in at ASSEMBLY ORDER 4,
                           the one moment it is reachable: the disc is on the
                           spline and NOTHING stands above it yet. hub_clamp
                           closes that corridor one step later and the screw is
                           blind for the life of the arm, which is exactly why
                           it is torqued before the clamp comes down.
                           THE OTHER TWO SHIP UNUSED. Pitch and roll bench-fit
                           their disc to a MONOLITHIC driven part, so their
                           centre screw is blind from the first second and their
                           retention is structural anyway (6808 +
                           pitch_retainer; yoke + idler pin). Keep the two as
                           spares - they are the same M3x6 as the drive screws
                           below, so two of those twelve come free.
  12x M3x6 SOCKET CAP      HORN DRIVE SCREWS, 4 per axis, and one length for all
                           three even though two of the joints are built
                           inside-out from the third:
                           PITCH / ROLL - down THROUGH the disc (O3.2 after
                           prep) into the driven part's insert. 2.2 mm of disc
                           then 3.8 mm of thread, so the full 4.0 mm insert is
                           95% engaged and nothing bottoms. M3x8 needs 5.8 mm
                           of bore and the insert bore is 5.5 - it bottoms
                           before the head clamps. That is why this line went
                           from 8 to 6 when the horn stopped being 3.0 thick.
                           TWIST - down through hub_clamp's O5.8 x 3.0
                           counterbore, 4.4 mm of clamp, then into THE DISC'S
                           OWN TAPPED M3 for 1.6 of its 2.2 mm. Tip stops
                           0.6 mm short of the disc's far face. M3x8 here would
                           still fit - full 2.2 mm of thread, tip still 1.6 mm
                           off the bracket plate - but it leaves 1.4 mm of
                           ROTATING screw hanging in the drive gap where no
                           sweep models it, for 0.6 mm of thread the joint does
                           not need. 6 is the length everywhere: ONE SKU for all
                           twelve drive screws AND for the centre screw above.
  4x  M4x16                bicep -> interface plate, counterbored from below.
  1x  M5x25 + 1x M5 nyloc nut + 1x O5x4 spacer     roll idler pin.
                           NOTE, and it is a real one: the CAD gives this pin
                           no thread anywhere. It passes a O5.1 slip hole in
                           the yoke disc (x -46..-40), 4.0 mm of AIR, the
                           625ZZ bore (x -36..-31) and a O6.4 clearance hole
                           in the carrier idler plate (x -32..-28). No insert,
                           no tapped boss, nothing. 25 mm is the right length
                           ONLY with a nyloc on the inboard side - head face
                           x=-46, tip x=-21, nut sits at -29..-24 - and the
                           4.0 mm gap needs a spacer or the yoke disc gets
                           pulled inwards the moment the nut is torqued.
  4x  M3x80 + 4x M3 nyloc  spine sandwich, SHARED with the other arm. Grip is
                           6 + 59 + 6 = 71 mm. Bores at (+/-22, C_Z+/-34); the
                           spine is drilled O3.4 through at the same four
                           spots. See ASSEMBLY ORDER 13 for the pitch angles.
  30x M3 insert, 4.0 mm   ONE SKU for every M3 position (e.g. a 100pc "M3x4x5"
                           pack), and ONE BORE DEPTH: all 30 are 5.5 mm. IT WAS
                           34. hub_clamp's four twist-horn positions are gone -
                           that joint lands in the horn's own aluminium now, so
                           the only fastener metal it needs was already bought
                           with the horn. TAB_INS_D used to be 4.5 to hold the
                           twelve servo bores 0.5 mm clear of a horn pocket
                           floor they were drilled straight at; THE EAR DECK
                           ENDED THAT. Those twelve are now in a face 9.5 mm
                           BELOW the case top pointing AWAY from the drive, the
                           nearest finishing 4.5 mm from a pocket floor. No
                           screw in this build engages more than 4.0 mm of
                           insert thread (retainer 4.0, cap 4.0, hub 4.0, horn
                           3.8, ears 4.0), so a 4.0 insert seats flush
                           everywhere. Longer 5.5-5.7 inserts fit ALL 30.
      (the 30, by part)    mount 3 retainer + 4 ear = 7;
                           carrier 4 pitch-horn + 4 ear = 8; yoke 4 race-cap +
                           4 roll-horn + 4 ear = 12; hub_clamp 3 hub = 3.
                           hub_clamp's other four holes are Ø3.4 CLEARANCE
                           through-holes and hold no thread at all.
"""

import os
import math
import cadquery as cq

OUT = os.path.dirname(os.path.abspath(__file__))
SIDE = -1                                  # +1 left, -1 right (mirror in Y)

# ---------------- servo envelope (swap these to change servo class) ----------
# HTS-35H (Hiwonder, 35 kg*cm @ 11.1V): a dimensionally STANDARD servo. Two
# factory drawings cross-validate every labelled figure below; unlabelled ones
# are pixel-measured on both drawings (see docs/img_shop/hts35h_ref_1/2.jpg).
SRV_L, SRV_W, SRV_H = 39.94, 20.04, 40.40  # body; z=0 in servo() is the CASE TOP
SRV_OFF = 9.97                             # spline axis 10.0 from the near end face
EAR_SPAN = 54.38                           # ear tip to ear tip (ears flush with sides)
EAR_T = 3.1                                # ear plate thickness
EAR_Z = -12.6                              # ear BOTTOM face below case top - THE DECK DATUM
EAR_DX, EAR_DY = 49.5, 10.0                # standard 4-slot pattern, O4.6 open keyholes
BOSS_H = 1.5                               # O12.5 output boss above the case top
SPLINE_TIP = 5.1                           # serration tip above case top (= 1.5 boss + 3.5)
                                           # v3's 32 x 10 put two of the four bolts at
                                           # r=6.1 from the spline - inside the output
                                           # collar. Nothing could ever be drilled there.
PROTOTYPE_THREADS = True                   # ME's call for early prototypes: NO heat-set
                                           # inserts - print O2.5 pilots and let the M3
                                           # machine screws thread-form (or run a $3 M3
                                           # tap first). Same screws, same seats, same
                                           # engagement depths. Any bore that strips in
                                           # testing gets drilled to O4.2 and an insert
                                           # pressed in as the field repair. Set False to
                                           # restore insert bores for the final build.
INSERT_R, INSERT_D = (1.25, 6.5) if PROTOTYPE_THREADS else (2.1, 5.5)
TAB_INS_D = 5.5                            # the twelve EAR-DECK inserts - four per joint,
                                           # drilled into the deck FACE (the ear-top plane
                                           # at case_top - 9.5) on the tab_points() ear
                                           # pattern. Plain full depth, identical to
                                           # INSERT_D, and the old 4.5 "depth separation"
                                           # is GONE with the flange architecture that
                                           # needed it. It existed because the flange
                                           # plane sat at case_top - 4 and the bores ran
                                           # TOWARD the horn: 0.5 mm was all that kept
                                           # them out of the horn pocket floor. The ear
                                           # deck is 9.5 mm below the case top, so a 5.5
                                           # bore ends at case_top - 4.0, which is 5.5 mm
                                           # short of the head-pocket floor at
                                           # case_top + HORN_FLOOR and 9.0 mm short of the
                                           # disc underside. Nothing on this face is within
                                           # half a centimetre of anything the horn
                                           # touches. Holds the same 4.0 mm SHORT M3
                                           # insert as every other bore in the module,
                                           # with 1.5 mm of bore left under it: the M3x8
                                           # crosses only EAR_T = 3.1 of servo ear, so its
                                           # tip lands 0.6 mm short of the floor.
DECK_T = 6.0                               # ear-deck plate thickness. The deck is the
                                           # ONLY thing that locates the servo axially:
                                           # the case is a slip fit through a
                                           # SRV_L+0.7 x SRV_W+0.6 cutout and the ear TOPS
                                           # bear on the deck face, with the four M3x8
                                           # pulling up from the ear BOTTOMS through the
                                           # servo's own O4.6 open slots. Nothing bolts to
                                           # the case any more and no screw enters the
                                           # servo.
DECK_CLR_L, DECK_CLR_W = 0.7, 0.6          # case cutout, on the body length and width
DECK_GUS_W, DECK_GUS_D = 3.0, 2.5          # gusset relief channel across the deck face,
                                           # on the ear centreline. The HTS-35H stands a
                                           # 1.7 x 2.2 stiffening rib PROUD of its ear
                                           # tops for almost the whole 54.38 span; without
                                           # this channel the ears never touch the deck at
                                           # all and the servo sits 2.2 mm high on a rib.
                                           # 3.0 wide leaves 0.65 either side, 2.5 deep
                                           # leaves 0.3 over the rib.
CASE_CLR = 0.6                             # air over the case TOP, deck face to the
                                           # underside of the horn plate. The ears set the
                                           # depth, so the case top is a free end now.
# BRK_T (the 7.0 mm servo FACE-PLATE thickness) is deleted, not repointed. There
# is no face plate any more: the ear deck is DECK_T and the three horn plates
# are 3.0 / 2.8 / 2.4, each set by a named plane pair, not by a shared constant.
POCKET_CLR = 1.6                           # insertion-channel clearance round the
                                           # servo case. NOT a bolted fit any more -
                                           # the deck locates the servo, this is
                                           # just the corridor it slides down.
MOV_CLR = 1.5                              # moving-clearance target everywhere else

# ---------------- horn : the REAL purchasable 25T disc ----------------------
# Generic 25T aluminium disc horn, and IT IS NOT A FLAT PUCK. It is a HUB and a
# DISC. Modelling it as a Ø20 x 3.0 washer is what hid four drive-screw heads
# inside the servo's own output boss:
#   HUB   Ø7.3 splined barrel. Base ON THE BOSS TOP at case_top + BOSS_H, top at
#         case_top + BOSS_H + HORN_HUB_H. It swallows the WHOLE serration - the
#         spline tip finishes 2.1 mm down inside it - so nothing stands proud of
#         the driven face and no driven part needs a centre relief.
#   DISC  Ø19.7 x HORN_DISC_T, hanging UNDER the hub top: it spans
#         hub_top - 2.2 .. hub_top. The 4x M3 TAPPED holes on the Ø14 circle go
#         through that 2.2 mm of aluminium and through nothing else.
# The DRIVEN FACE on every axis is the HUB/DISC TOP, at case_top + HORN_FACE.
#
# ############################################################################
# ##  HORN_HUB_H IS UNPUBLISHED. IT IS AN ESTIMATE. PUT CALIPERS ON THE REAL
# ##  HORN BEFORE YOU PRINT ANYTHING. Every driven plane in this module -
# ##  JRN_P_Y1, ROLL_FACE_X, HC_Z0 and both head-clearance pocket floors - is
# ##  that one number plus a fixed offset. No vendor publishes hub height for a
# ##  generic 25T disc horn, so 5.7 is the value THE DESIGN NEEDS (see the
# ##  heads-vs-boss audit at HORN_HUB_H), not a value anyone measured. A SHORT
# ##  hub is a hard fault, not a tolerance: at 4.5 the drive heads sit 0.7 mm
# ##  INSIDE the Ø12.5 output boss and the horn cannot seat. If the caliper
# ##  reads under 5.7, do not print - either shim the horn up off the boss or
# ##  countersink the disc's four holes (an M3 csk sinks 1.7 of the 2.2) and
# ##  re-derive. If it reads over, every driven plane moves out with it and the
# ##  module still builds; only the arithmetic in these comments goes stale.
# ############################################################################
HORN_HUB_R = 3.65                          # Ø7.3 splined hub. Its bore is SPLINE_TIP -
                                           # BOSS_H = 3.6 mm of live serration, engaged in
                                           # full: the hub bottoms on the boss top, not on
                                           # the spline tip, so there is no HORN_LIFT to
                                           # trade engagement against any more and the old
                                           # 2.5-of-3.0 compromise is gone with it.
HORN_HUB_H = 5.7                           # hub height above the boss top. READ THE BOX.
                                           # THE HEADS-VS-BOSS AUDIT, which is what set it:
                                           # on PITCH and ROLL the four M3 drive screws
                                           # pass DOWN through the disc, so their heads
                                           # hang beneath it on the Ø14 circle, filling the
                                           # ring r 4.1..9.9 over
                                           #     disc_bottom - 3.0 .. disc_bottom
                                           #   = HORN_HUB_H - 3.7 .. HORN_HUB_H - 0.7
                                           # above the case top. The servo's own Ø12.5
                                           # output boss (r 6.25) stands BOSS_H = 1.5 proud
                                           # of that same case top, and the two rings
                                           # OVERLAP RADIALLY over 4.1 < r < 6.25. There is
                                           # no clocking out of that - the boss is round -
                                           # so the only escape is axial:
                                           #     clearance = (HORN_HUB_H - 3.7) - BOSS_H
                                           #               =  HORN_HUB_H - 5.2
                                           # At the first 4.5 estimate that is -0.7 mm:
                                           # four heads driven 0.7 mm INTO the boss, the
                                           # horn never seats, the drive is fiction. 5.7 is
                                           # the SMALLEST hub that clears it, and it clears
                                           # it by exactly 0.5 mm.
HORN_DISC_R = 9.85                         # Ø19.7 disc
HORN_DISC_T = 2.2                          # disc thickness - and the WHOLE thread depth
                                           # available on the twist axis, where the drive
                                           # screws land in the disc's own tapped holes and
                                           # there is no insert and no drill-out.
HORN_BC = 7.0                              # Ø14 bolt circle, real 25T disc
HORN_A = (0.0, 90.0, 180.0, 270.0)         # 4x M3 at 90 deg, not the old 45 family
HORN_A_T = (45.0, 135.0, 225.0, 315.0)     # TWIST ONLY, and it is pure CLOCKING: the disc
                                           # is 4-fold symmetric so turning it 45 deg costs
                                           # the builder nothing, and it is the only thing
                                           # that lets the four twist DRIVE SCREWS clear
                                           # hub_clamp's three r=11 plate-bolt bores.
                                           # Worked clearance is at the hub_clamp code.
HORN_FACE = BOSS_H + HORN_HUB_H            # 7.2 - THE DRIVEN FACE, above the case top.
                                           # JRN_P_Y1, ROLL_FACE_X and HC_Z0 are each just
                                           # their own servo's case top plus this one
                                           # number. There is no per-axis shaft length any
                                           # more: SHAFT_P / SHAFT_R / SHAFT_T were three
                                           # names for 4.5 and they are gone.
HORN_DISC_Z0 = HORN_FACE - HORN_DISC_T     # 5.0 - disc underside above the case top
HORN_HEAD_Z0 = HORN_DISC_Z0 - 3.0          # 2.0 - underside of an M3 socket-cap head, ON
                                           # PITCH AND ROLL ONLY. On TWIST the screw is the
                                           # other way up and its head is buried in
                                           # hub_clamp's top face, 7 mm further out.
HORN_GAP = 0.5                             # designed running air UNDER THOSE HEADS, and it
                                           # is the tightest designed clearance left in the
                                           # module. THE SWEEP CANNOT SEE IT - screws are
                                           # not modelled solids - so it is a bench-verify
                                           # item, and it is the only reason the two
                                           # pockets below exist at all. The DISC itself
                                           # now clears every plate by 1.6..3.0 mm.
HORN_FLOOR = HORN_HEAD_Z0 - HORN_GAP       # 1.5 - head-pocket floor above the case top,
                                           # which lands EXACTLY on the boss top plane.
HORN_CLR_R = 11.5                          # Ø23 head-clearance pocket, 1.6 mm outside the
                                           # r 9.9 head ring. Cut SHALLOW from the pitch
                                           # and roll plates' OUTER faces only, leaving
                                           # each a 0.9 mm web at 6.5 < r < 11.5. THE TWIST
                                           # BRACKET HAS NO POCKET AT ALL any more - its
                                           # disc underside is bare, because the twist
                                           # drive screws come DOWN from hub_clamp above.
                                           # The axial spline path is a separate Ø13 bore.

# ---------------- frame ----------------
C_Z = 100.0                                # the three axes meet at (0,0,C_Z)
SPINE_X = 50.0                             # skeleton spine, fore-aft
SPY0, SPY1 = -147.9, -88.9                 # spine faces; outer one 88.9 in
MNT_T = 6.0

# ---------------- pitch stage (6808-2RS 40x52x7) ----------------
P_BORE, P_OD, P_W = 40.0, 52.0, 7.0
BRG_P_Y0, BRG_P_Y1 = -38.5, -31.5          # bearing seat, moved 2.5 outboard so the
                                           # housing boss can end at -31.5 and still
                                           # leave the carrier's r=26 discs 1.5 mm of air
BOSS_R = 33.0                              # pitch housing OD; 6.95 mm wall on the Ø52 seat
SRV_P_X, SRV_P_Y = 0.0, -46.6              # servo P ON the pitch axis; case top (was
                                           # -45.5 for the STS3215: tip 5.1 not 4.0)
# SERVO P IS CLOCKED 90 deg ABOUT ITS OWN SPLINE AXIS. Body LENGTH and the
# 54.38 mm ear span run along WORLD Z; only the 20.04 width runs along X. Forced,
# not chosen: the spine pocket is 50 mm across in x and the shell walls stand at
# |x| = 25..31, so a 54.38 ear span laid in x would have to break both side walls
# clean off. In z the shell is 77 mm of free height and nothing but the spine
# bolts lives out there. SRV_OFF runs +Z (body 90.0..129.94, ears 82.78..137.16)
# rather than -Z because the module mounts INVERTED on the robot (rpy 0 pi 0,
# gen_humanoid_v11.py): +Z here is DOWN the torso, so the long side of the servo
# and its ear deck extend into the ribcage. -Z would have pushed the same 37 mm
# up past the acromion into the neck/head volume.
SRV_P_ZC = C_Z + SRV_OFF                   # 109.97, servo P body centre in z
CRADLE_Y0, CRADLE_Y1 = SPY1, -43.0
CH_P_HX = SRV_W / 2 + POCKET_CLR           # 11.62 - the insertion channel is only the
                                           # servo WIDTH wide now (23.24 mm of x, was
                                           # 43.14). It no longer reaches |x| = 25, so the
                                           # 28 mm slot the old channel tore through the
                                           # +x spine-clamp side wall IS GONE and both
                                           # side walls run unbroken from z 62 to 139.
CH_P_Z0, CH_P_Z1 = 81.0, 139.0             # channel in z: the EARS set this, not the case
                                           # (82.78..137.16 plus ~1.6). The cradle
                                           # extension walls stand on exactly this span,
                                           # so the channel has no end walls in z - which
                                           # is also what gives the servo's two z-end
                                           # connector faces 9.0 mm of open air each.
DECK_P_HX = 16.5                           # deck / channel-wall half width. 16.5 and not
                                           # the cradle's 34: the (+/-22, C_Z+34) spine
                                           # bolts now pass alongside this extension and
                                           # an M3 head ring reaches x = 19.25.
DECK_P_Y0 = SRV_P_Y - 9.5                  # -56.1  EAR-TOP PLANE = the deck FACE
DECK_P_Y1 = DECK_P_Y0 + DECK_T             # -50.1
PLATE_P_Y0, PLATE_P_Y1 = SRV_P_Y + CASE_CLR, CRADLE_Y1   # -46.0 .. -43.0. The plate is
                                           # 3.0 mm now, not 7.6: it starts 0.6 mm ABOVE
                                           # the case top instead of 4.0 below it, because
                                           # nothing bolts to it any more. It is a horn
                                           # plate and a spline path, nothing else.
JRN_P_Y1 = SRV_P_Y + HORN_FACE             # -39.4  (was -42.1 on the flat-puck horn).
                                           # SRV_P_Y IS servo P's case top; the hub/disc
                                           # top sits HORN_FACE past it and THAT is the
                                           # driven face. Chain: case top -46.6, boss top
                                           # -45.1, disc bottom -41.6, driven face -39.4.
                                           # The carrier Ø40 journal is extended OUTBOARD
                                           # from BRG_P_Y0 (-38.5) by 0.9 mm to meet it,
                                           # inside the mount's r=24 outer-race lip pocket.
RET_Y0, RET_Y1 = BRG_P_Y1, BRG_P_Y1 + 3.0  # pitch_retainer traps the outer race
RET_BC = 29.5

# ---------------- roll stage ----------------
CARR_X0, CARR_X1 = 29.0, 35.3              # outer face thinned 36.0 -> 35.3 and it STAYS
                                           # thinned on the hub+disc horn: the disc
                                           # underside is at SRV_R_X + HORN_DISC_Z0 = 36.9,
                                           # so 35.3 leaves 1.6 mm of running air and 36.0
                                           # would leave 0.9 - under the checker's 1.0 mm
                                           # floor. Plate thickness is spare here; nothing
                                           # bolts to it, its inserts live in the deck.
CARR_R = 26.0
YOKE_X0, YOKE_X1 = 40.0, 46.0
SRV_R_X = 31.9
SRV_R_ZC = C_Z - SRV_OFF                   # 90.03 body centre in z. Servo R keeps v6's
                                           # orientation - length along z with the case
                                           # hanging BELOW the roll axis, reaching only
                                           # 10.0 mm above it - so the ears simply follow
                                           # the body: tips at 62.84 and 117.22.
DECK_R_X0 = SRV_R_X - 9.5                  # 22.4  EAR-TOP PLANE = the deck FACE
DECK_R_X1 = CARR_X0                        # 29.0. Nominal deck is DECK_T = 6.0 (to 28.4);
                                           # the last 0.6 mm is the butt joint into the
                                           # existing front plate, so deck and horn plate
                                           # are one continuous 13.6 mm slab everywhere
                                           # the case relief does not pass through them.
DECK_R_HY = 14.0                           # matches the front plate's own y half width
DECK_R_Z0, DECK_R_Z1 = 61.5, 118.5         # ear span 62.84..117.22 plus 1.6/1.3, and it
                                           # leaves 1.68 mm of deck outside the outermost
                                           # insert bore. The servo R face plate grows to
                                           # the same z (was 65..113) so the deck lands on
                                           # real plate over its whole length.
PLATE_R_X0 = SRV_R_X + CASE_CLR            # 32.5 - case relief ceiling inside the front
                                           # plate. Leaves 2.8 mm of plate, 0.9 mm of it
                                           # under the head-pocket floor at 33.4.
ROLL_FACE_X = SRV_R_X + HORN_FACE          # 39.1  (was 36.4). SRV_R_X IS servo R's case
                                           # top. Chain: case top 31.9, boss top 33.4,
                                           # disc bottom 36.9, driven face 39.1. The yoke
                                           # roll disc's inner face is ALREADY at
                                           # YOKE_X0 = 40.0, so its boss only has to reach
                                           # 0.9 mm inboard now - it was 3.6 - and the
                                           # carrier-plate-to-boss running gap opens from
                                           # 1.1 to 3.8 mm.
ROLL_BOSS_R = 11.5                         # Ø23 boss backing the Ø19.7 disc: 3.25 mm of
                                           # wall outside the r=8.25 horn insert bores.
                                           # Boss 0.9 + disc 6.0 = 6.9 mm of material for
                                           # an INSERT_D = 6.5 blind bore: 0.4 mm of floor,
                                           # the thinnest bore floor in the module. With
                                           # real Ø4.2 inserts (INSERT_D 5.5) it is 1.4.

# ---------------- twist stage (6806-2RS 30x42x7) ----------------
T_BORE, T_OD, T_W = 30.0, 42.0, 7.0
PLAT_Z0, PLAT_Z1 = 180.0, 191.5    # STACK DROP -9.0: ear-mount hangs shallower            # platform. THE WHOLE TWIST STACK IS SET BY
                                           # SRV_T_Z - see the note there. Every constant
                                           # from here down to TRAY_Z1 is that one height
                                           # plus a fixed spacing; move one, move all.
BRG_T_Z0, BRG_T_Z1 = 184.5, 191.5
CAP_Z0, CAP_Z1 = 191.5, 194.5
CAP_ID, CAP_OD, CAP_BC = 19.5, 28.0, 25.0
HC_Z0, HC_Z1 = 177.1, BRG_T_Z0             # hub_clamp. HC_Z0 = SRV_T_Z + HORN_FACE =
                                           # 169.9 + 7.2, and its UNDERSIDE IS THE DRIVEN
                                           # FACE - it lands flat on the disc top. Written
                                           # as a literal for the same reason
                                           # PLAT_Z0/BRG_T_Z0/CAP_Z0 are: SRV_T_Z carries
                                           # the roll-sweep argument and is defined below.
                                           # THE OLD 175.5 WAS STALE by 1.1 mm - it was
                                           # SRV_T_Z + 5.6 while pitch and roll were on
                                           # +4.5, a drift the derived form now cannot
                                           # repeat. HC_Z1 unchanged, so the part is
                                           # 7.4 mm deep, not 9.0.
HC_R, HC_BC = 18.0, 11.0                   # bolt circle Ø22, was Ø18. Moved OUT so the
                                           # three hub bolts clear the four TWIST DRIVE
                                           # holes on the Ø14 circle: at Ø18 the 0 deg pair
                                           # sat 2.0 mm apart on centres and merged. Walls
                                           # it leaves - hub_clamp's own bores reach
                                           # r 12.25 in the Ø36 clamp, 5.75 mm;
                                           # the interface plate Ø3.4 through-holes reach
                                           # r 12.7 in the Ø30 journal, 2.3 mm (was 4.3,
                                           # and it is the tightest thing this move costs).
IF_Z0, IF_Z1 = 196.0, 202.0                # interface plate
TRAY_Z1 = 208.0
SHOULDER_R = 17.0                          # rotating flange - bears on the inner race only
# TWIST STACK HEIGHT. This is not a packaging preference, it is the roll sweep.
# Servo T hangs UNDER the platform, so its case bottom sits
#       d0 = SRV_T_Z - 40.9 - C_Z
# above the roll axis, and that face sweeps a circle of radius d0 as the arm
# rolls. Servo R's case hangs BELOW the roll axis on the other side, reaching
# 35.2 mm down and 37.3 mm to its lower corners (SRV_L/2 + SRV_OFF, and the
# corner with SRV_W/2). Past roll ~95 deg the T case sweeps straight through
# that corner - it is a solid overlap, not a near miss: at SRV_T_Z = 158 the
# two cases share 6231 mm^3 at roll 140. The ONLY escape is radial, because
# both are bought parts and neither may move off its axis:
#       d0 >= 37.3 + 1.0  ->  SRV_T_Z >= 179.2
# 180.0 clears servo R at EVERY roll angle by >= 1.78 mm, so the joint cannot
# reach this clash anywhere in 360 deg and the travel needs no software cap.
# Lowering does the opposite of what the sampled checker suggests: 158 clears
# only to roll 96, and below 157 the two cases overlap at roll 0.
SRV_T_Z = 169.9
assert abs(HC_Z0 - (SRV_T_Z + HORN_FACE)) < 1e-9, (
    "HC_Z0 must be servo T's case top plus HORN_FACE - it is a literal only "
    "because SRV_T_Z is defined after the twist stack it sets. "
    f"HC_Z0={HC_Z0}, SRV_T_Z+HORN_FACE={SRV_T_Z + HORN_FACE}")
SRV_T_YC = SRV_OFF                         # 9.97, servo T body centre in y
BRK_Z0 = SRV_T_Z - 9.5                     # 169.4  EAR-TOP PLANE = the bracket deck FACE.
                                           # The old face plate at SRV_T_Z - 4.0 IS this
                                           # deck now - it moved down 5.5 mm and thickened
                                           # from 7.0 to DECK_T, and the horn plate split
                                           # off above it at PLATE_T_Z0.
DECK_T_Z1 = BRK_Z0 + DECK_T                # 175.4 deck top
PLATE_T_Z0 = SRV_T_Z + CASE_CLR            # 179.5 horn-plate underside, 0.6 over the case
BRK_Z1 = SRV_T_Z + 3.0                     # 172.9 horn-plate top. The CAP_ID bore takes
                                           # r < 19.5 from 171.9 up, so at 6.5 < r < 19.5
                                           # this plate is 1.4 mm and the disc above it
                                           # (174.9) clears it by 3.0 mm with NO POCKET.
BRK_HX = 17.0
BRK_Y0, BRK_Y1 = -18.5, 38.5               # WAS -12..37, which the ear tips (-17.22 and
                                           # 37.16) and their screws overhung by 5.2 and
                                           # 0.2 mm - the two inboard ear screws would
                                           # have been driven into air. Widened both ways
                                           # so every ear tip and every bore lands on
                                           # material with >= 1.28 mm of edge left. The
                                           # wall stubs follow; past the platform edge at
                                           # y = 32 the +y stub is a free-standing rib
                                           # stiffening the deck's cantilevered end.

BOLT_DX, BOLT_DY = 16.0, 20.0              # bicep M4 grid 32 x 40
BICEP_X, BICEP_Y, BICEP_L = 50.0, 58.0, 116.0


# ---------------- helpers ----------------
def B(x0, x1, y0, y1, z0, z1):
    """Axis-aligned box from two corners, in any order."""
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    z0, z1 = min(z0, z1), max(z0, z1)
    return (cq.Workplane("XY", origin=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
            .box(x1 - x0, y1 - y0, z1 - z0))


def cylX(x0, x1, y, z, r):
    return cq.Workplane("YZ", origin=(x0, y, z)).circle(r).extrude(x1 - x0)


def cylY(x, y0, y1, z, r):
    return cq.Workplane("XZ", origin=(x, y1, z)).circle(r).extrude(y1 - y0)


def cylZ(x, y, z0, z1, r):
    return cq.Workplane("XY", origin=(x, y, z0)).circle(r).extrude(z1 - z0)


def ring(r_in, r_out, z0, z1):
    return cylZ(0, 0, z0, z1, r_out).cut(cylZ(0, 0, z0 - 1, z1 + 1, r_in))


def servo(drilled=True):
    """Body+collar as one solid, HORN as another. Identical on all three axes -
    there is no per-axis shaft length, because the horn is a bought part and it
    seats the same way everywhere.

    Shaft axis = +Z at the origin. The planes, all above the CASE TOP at z = 0:
      BOSS_H       = 1.5   top of the O12.5 output boss. THE HORN HUB BOTTOMS
                           HERE, on the boss, not on the spline.
      SPLINE_TIP   = 5.1   end of the serration - 3.6 mm of live spline above
                           the boss, ALL of it inside the hub, none of it
                           visible once the horn is on.
      HORN_DISC_Z0 = 5.0   disc underside.
      HORN_FACE    = 7.2   HUB/DISC TOP = THE DRIVEN FACE, the flat every driven
                           part sits on. Nothing stands proud of it: the spline
                           tip stops 2.1 mm below it, inside the hub bore, which
                           is where the factory M3x6 centre screw also lives.
    So the returned 'shaft' solid is the HORN, not a shaft: a O7.3 hub from the
    boss top to the driven face, with the O19.7 x 2.2 disc hung under its top.
    The serration itself is never drawn - it is enclosed by the hub, and drawing
    a cylinder where a real part already is would only double-count it."""
    body = (cq.Workplane("XY", origin=(SRV_OFF, 0, -SRV_H / 2))
            .box(SRV_L, SRV_W, SRV_H)
            .union(cq.Workplane("XY", origin=(SRV_OFF, 0, EAR_Z + EAR_T / 2))
                   .box(EAR_SPAN, SRV_W, EAR_T))
            .union(cq.Workplane("XY", origin=(SRV_OFF, 0, EAR_Z + EAR_T + 1.1))
                   .box(EAR_SPAN - 1.0, 1.7, 2.2))
            .union(cq.Workplane("XY", origin=(0, 0, BOSS_H / 2))
                   .cylinder(BOSS_H, 6.25)))
    # THE HORN: hub + disc, one solid. The hub runs boss top -> driven face and
    # covers the whole serration; the disc hangs under the hub's top face.
    shaft = (cylZ(0, 0, BOSS_H, HORN_FACE, HORN_HUB_R)
             .union(cylZ(0, 0, HORN_DISC_Z0, HORN_FACE, HORN_DISC_R)))
    # THE HORN'S OWN HOLES - real, because screws pass through them on camera
    # and the screw physics samples them. Pitch/roll discs are the DRILLED
    # (O3.2) ones on HORN_A; the twist disc keeps its factory M3 taps, drawn
    # at the O2.6 tap-minor bore, and carries the 45 deg clocking (HORN_A_T).
    # The hub gets the O5.9 spline bore to the serration's full 3.6 mm depth
    # and the O3.4 centre-screw hole the factory M3x6 drops down.
    for hx_, hy_ in circle_points(HORN_BC, HORN_A if drilled else HORN_A_T):
        shaft = shaft.cut(cylZ(hx_, hy_, HORN_DISC_Z0 - 0.5, HORN_FACE + 0.5,
                               1.6 if drilled else 1.3))
    shaft = (shaft.cut(cylZ(0, 0, BOSS_H - 0.5, SPLINE_TIP, 2.95))
                  .cut(cylZ(0, 0, SPLINE_TIP, HORN_FACE + 0.5, 1.7)))
    # the HTS-35H's 4 ear slots (O4.6 open keyholes on the 49.5 x 10 pattern).
    # Modelled as round holes - the open neck matters to the builder, not the
    # screw physics. Without them every ear screw would clip through the ear.
    for sx_ in (1, -1):
        for sy_ in (1, -1):
            body = body.cut(cq.Workplane(
                "XY", origin=(SRV_OFF + sx_ * EAR_DX / 2, sy_ * EAR_DY / 2,
                              EAR_Z - 1)).circle(2.3).extrude(EAR_T + 2))
    return body, shaft


def tab_points():
    """The 4 ear-slot screws in the servo's own frame (z = case top): the
    standard 49.5 x 10 pattern, centred on the body (not the spline)."""
    return [(SRV_OFF + sx * EAR_DX / 2, sy * EAR_DY / 2)
            for sx in (1, -1) for sy in (1, -1)]


def horn_points(r=HORN_BC):
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            for a in HORN_A]


def circle_points(r, angles):
    return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            for a in angles]


# =============================================================================
# MOUNT : spine clamp + servo P cradle/face plate + pitch bearing housing
# =============================================================================
# The skeleton spine is 50 x 59 mm, so the v3 torso plate becomes a sleeve that
# closes around it and bolts through each wall into inserts in the spine.
# The shell top went from C_Z + 38 to C_Z + 39 so that it finishes flush with the
# cradle extension the clocked servo P needs (CH_P_Z1 = 139). One millimetre of
# clamp length, and it is the only envelope change in this pass.
mount = B(-31, 31, SPY0 - MNT_T, SPY1 + MNT_T, C_Z - 38, C_Z + 39)
mount = mount.cut(B(-SPINE_X / 2, SPINE_X / 2, SPY0, SPY1, C_Z - 50, C_Z + 50))
mount = mount.cut(B(-33, 33, SPY0 - 8, (SPY0 + SPY1) / 2, C_Z - 52, C_Z + 52))

# G. SPINE SANDWICH BOLTS - 4x M3x80. Each mount is a HALF shell (the cut
# above): the left arm's shell covers one spine face, the right arm's shell the
# other, and four M3x80 run through shell + spine + far shell into nyloc nuts.
# Grip = 6 + 59 + 6 = 71 mm, so M3x70 cannot take a nut - 80 is the length.
# The spine itself gets four O3.4 through-holes drilled at (+/-22, C_Z+/-34).
#
# THE POSITIONS MOVED, and that is the only reason the module can be built at
# all. The bolt is offered up at the shell's outer face y = SPY1 + MNT_T =
# -82.9 and driven -Y, so its 80 mm body plus head has to be held in the +Y
# space above that face first: 83 mm of straight empty corridor, 3.2 mm around
# the axis for the head. At the old (+/-20, C_Z+/-24) the pitch housing boss
# (r=33, starting at y=-43.0) closed that corridor after 39.99 mm and every one
# of the four was un-insertable. At (+/-22, C_Z+/-34) the axis lies at r=40.5
# from the pitch axis - 4.3 mm outside the boss with the head ring counted - and
# the corridor is void over the full 85 mm. THE CLOCKED SERVO P DID NOT CLOSE
# IT. The cradle now runs a z 81..139 extension that reaches past the
# (+/-22, C_Z+34) pair, but that extension is only DECK_P_HX = 16.5 wide and an
# M3 head ring reaches x = 18.8: 2.3 mm of daylight either side, probed void
# over the full corridor at all four positions.
# The one exception is unchanged and it is on the moving side: the
# (+22, C_Z-34) corridor is crossed by the carrier's servo R deck/front plate
# (x 22.4..36.0, z 61.5..118.5) and by servo R's own case (x -8.5..31.9,
# z 62.84..117.22) while the pitch joint sits near 0. Swept: at pitch 0 that
# one bolt is blocked and the other three are clear; at pitch 180 the blocked
# one is (-22, C_Z+34) instead. So three at 0 and (+22, C_Z-34) at 180, exactly
# as before the reorientation. See ASSEMBLY ORDER 13.
for bx in (-22.0, 22.0):
    for bz in (C_Z - 34.0, C_Z + 34.0):
        mount = mount.cut(cylY(bx, SPY1 - 1, SPY1 + MNT_T + 1, bz, 1.7))

# CRADLE. A box round the servo P case - x -34..34 now, not ..40: the old +40
# followed the UNCLOCKED case out to x = 31.54 and nothing needs it any more,
# the widest thing on this face being the Ø66 pitch boss at x +-33. Plus a
# narrower EXTENSION that carries the channel walls and the ear deck over the
# clocked servo's full z span.
mount = mount.union(B(-34, 34, CRADLE_Y0, CRADLE_Y1, 82, 118))
mount = mount.union(B(-DECK_P_HX, DECK_P_HX, CRADLE_Y0, CRADLE_Y1,
                      CH_P_Z0, CH_P_Z1))

# A. SERVO P INSERTION CHANNEL. The clocked servo (see SRV_P_ZC) puts its body
#    LENGTH and its 54.38 ear span along z and only its 20.04 width along x, so
#    the channel is sized by the EARS in z and by the case WIDTH in x. Two
#    things fall out of that. It is 23.24 wide instead of 43.14, so it never
#    reaches the |x| = 25 spine-clamp side walls and THE 28 mm SIDE-WALL SLOT IS
#    GONE - both walls run unbroken from z 62 to 139. And it spans the whole
#    extension in z, so it has no end walls at all: the case's z = 90.0 end
#    looks out on 9.0 mm of open air and its z = 129.94 end on 9.06 mm. That is
#    the third of the servo's three connector faces, served without a cut.
#    What it costs is the back plate's top tie: the channel leaves the plate as
#    a U open at z = 139, braced by the 45 mm deep cradle box behind it. All
#    four spine bolts sit in the side lobes at x = +-22 and are unaffected.
#    Servo P still goes in spline-first (+Y) BEFORE the mount meets the spine.
mount = mount.cut(B(SRV_P_X - CH_P_HX, SRV_P_X + CH_P_HX,
                    SPY0 - 8, DECK_P_Y0, CH_P_Z0, CH_P_Z1))

# EAR DECK. Everything between DECK_P_Y0 and PLATE_P_Y0 that the case does not
# need is LEFT SOLID, so the 6.0 mm deck and the 3.0 mm horn plate are one stiff
# 13.1 mm slab wherever the case cutout does not pass through them. The cutout
# is SRV_L+0.7 x SRV_W+0.6 about the body centre and doubles as the case-top
# relief: it runs from the deck face to the horn plate underside, clearing the
# case top by CASE_CLR. Deck face -56.1, deck top -50.1, 4.1 mm of air, plate
# underside -46.0.
mount = mount.cut(B(SRV_P_X - (SRV_W + DECK_CLR_W) / 2,
                    SRV_P_X + (SRV_W + DECK_CLR_W) / 2,
                    DECK_P_Y0, PLATE_P_Y0,
                    SRV_P_ZC - (SRV_L + DECK_CLR_L) / 2,
                    SRV_P_ZC + (SRV_L + DECK_CLR_L) / 2))
# GUSSET RELIEF - 3.0 wide x 2.5 deep along the ear centreline, full deck length
mount = mount.cut(B(SRV_P_X - DECK_GUS_W / 2, SRV_P_X + DECK_GUS_W / 2,
                    DECK_P_Y0, DECK_P_Y0 + DECK_GUS_D, CH_P_Z0, CH_P_Z1))
# CONNECTOR WINDOWS. The lead band is 6.0..10.8 above the case bottom, i.e.
# y -81.0..-76.2. The two z END faces are already open on the channel. The two
# x SIDE faces look into 22.4 mm of cradle wall each, so each gets a 12 (z) x
# 8 (y) window cut CLEAR THROUGH to outside air, centred on the band and on the
# body centre in z. Nothing structural: the walls are 22.4 mm of solid and the
# windows sit 2.03 mm below the cradle top and 0.3 mm above the shell plate.
for _sx in (1, -1):
    mount = mount.cut(B(_sx * CH_P_HX, _sx * 41.0, -82.6, -74.6,
                        SRV_P_ZC - 6.0, SRV_P_ZC + 6.0))
# SPLINE PATH - Ø13 (r 6.5), FULL DEPTH of the 3.0 mm horn plate. The HTS-35H's
# Ø12.5 output boss stands BOSS_H = 1.5 proud of the case top, so it occupies
# y -46.6..-45.1 and crosses this plate's underside at -46.0; 0.25 mm radial.
# The Ø7.3 horn hub runs the rest of the way out, -45.1..-39.4; 2.85 mm radial.
mount = mount.cut(cylY(SRV_P_X, PLATE_P_Y0 - 1, PLATE_P_Y1 + 1, C_Z, 6.5))
# HEAD-CLEARANCE POCKET - Ø23 and SHALLOW, from the plate OUTER face only, and
# IT IS NOT FOR THE DISC. The Ø19.7 disc now lies at y -41.6..-39.4, entirely
# OUTBOARD of this plate (which ends at -44.0 for r<24, the Ø48 outer-race lip
# floor) with 2.4 mm of air under it. What still reaches in is the FOUR DRIVE
# SCREW HEADS hanging under the disc on the Ø14 circle: r 4.1..9.9, spanning
# y -44.6..-41.6, i.e. 0.6 mm INSIDE this plate. So the pocket floor is
# SRV_P_Y + HORN_FLOOR = -45.1 - which is exactly the boss top plane - and the
# heads spin over it with HORN_GAP = 0.5. The plate is 3.0 (-46.0..-43.0), the
# pocket takes it back to -45.1 at r < 11.5, and 6.5 < r < 11.5 keeps a 0.9 mm
# web. No insert is anywhere near it: the four ear inserts are in the deck face
# 9.5 mm BELOW the case top and end at -50.6, 4.5 mm clear of this floor.
mount = mount.cut(cylY(SRV_P_X, SRV_P_Y + HORN_FLOOR, PLATE_P_Y1 + 0.5,
                       C_Z, HORN_CLR_R))

# C. SERVO P EAR INSERTS - 4x M3, TAB_INS_D deep, into the DECK FACE at
#    y = -56.1 on tab_points()' 49.5 x 10 ear pattern. Clocked mapping: the
#    pattern's long axis (hx) runs along z about C_Z, its short axis (hy) along
#    x. Bores land at (x +-5.0, z 85.22) and (x +-5.0, z 134.72), each 2.1 mm in
#    radius, with 1.68/1.62 mm of deck outside the outer pair and 1.4 mm to the
#    gusset relief inboard. Both pairs sit OUTSIDE the case cutout in z
#    (89.65..130.29), so all four land on full-thickness deck.
#    The 4x M3x8 come UP from below through the servo's own Ø4.6 ear slots:
#    3.1 mm of ear, then 4.9 mm into a 5.5 bore - the whole 4.0 mm insert
#    engaged and the tip 0.6 mm short of the floor. Corridor is the open
#    insertion channel; both z positions are clear of the case's own z ends
#    (4.78 and 4.78 mm), so a driver runs straight down beside it.
for hx, hy in tab_points():
    mount = mount.cut(cylY(SRV_P_X + hy, DECK_P_Y0, DECK_P_Y0 + TAB_INS_D,
                           C_Z + hx, INSERT_R))

# PITCH HOUSING BOSS. Without this the 6808 seat is cut into thin air.
mount = mount.union(cylY(0, CRADLE_Y1, BRG_P_Y1, C_Z, BOSS_R))
mount = mount.cut(cylY(0, BRG_P_Y0, BRG_P_Y1, C_Z, P_OD / 2 + 0.05))   # Ø52 seat
mount = mount.cut(cylY(0, CRADLE_Y1 - 1, BRG_P_Y0, C_Z, 24.0))         # outer-race lip
# pitch_retainer inserts, into the inboard end face of the boss
for rx, rz in circle_points(RET_BC, (0, 120, 240)):
    mount = mount.cut(cylY(rx, BRG_P_Y1 - INSERT_D, BRG_P_Y1, C_Z + rz, INSERT_R))

# retainer ring: traps the 6808 outer race against the lip
pitch_retainer = ring(24.0, BOSS_R, 0, RET_Y1 - RET_Y0)
pitch_retainer = (pitch_retainer.rotate((0, 0, 0), (1, 0, 0), -90)
                  .translate((0, RET_Y0, C_Z)))
for rx, rz in circle_points(RET_BC, (0, 120, 240)):
    pitch_retainer = pitch_retainer.cut(cylY(rx, RET_Y0 - 1, RET_Y1 + 1,
                                             C_Z + rz, 1.7))

# =============================================================================
# SERVO P
# =============================================================================
servoP_body, servoP_shaft = servo()
# CLOCKED -90 deg about its own spline first, then tilted +Z -> +Y as before.
# Net mapping, servo frame -> world:  local +x -> +Z, local +y -> +X, local +z
# -> +Y. So the body runs z 90.0..129.94, the ears z 82.78..137.16, the width
# x +-10.02, and the case y -87.0..-46.6. The -90 (not +90) is what sends
# SRV_OFF's long side to +Z - down the torso on the inverted robot - instead of
# -Z into the neck. See the note at SRV_P_ZC.
# DO NOT FLIP THIS SIGN WITHOUT MOVING THE DECK WITH IT. The mount's deck is
# built at +Z: case cutout z 89.65..130.29, ear inserts at z 85.22 and 134.72,
# channel and cradle extension z 81..139, shell top raised to 139 to match. At
# +90 the servo lands at z 62.84..117.22 instead and its case drives
# 2328.39 mm^3 straight through the deck - measured, not argued (intersect
# volume, mount & servoP_body; it is 0.000 at -90). The bodies are BONDED in
# check_meta, and the checker SKIPS bonded pairs, so the sweep still says PASS
# with the servo inside the plate. That is exactly how the old flange
# architecture hid the same defect on all three joints.
rotP1 = ((0, 0, 0), (0, 0, 1), -90)
rotP2 = ((0, 0, 0), (1, 0, 0), -90)
servoP_body = (servoP_body.rotate(*rotP1).rotate(*rotP2)
               .translate((SRV_P_X, SRV_P_Y, C_Z)))
servoP_shaft = (servoP_shaft.rotate(*rotP1).rotate(*rotP2)
                .translate((SRV_P_X, SRV_P_Y, C_Z)))

# =============================================================================
# CARRIER : rides the pitch bearing, carries servo R
# =============================================================================
# The journal is SOLID. v6.0 bored it out Ø22 for a horn passage, which is what
# left the hub_clamp/hub_collar bolt circles nowhere to land.
# The journal is EXTENDED OUTBOARD, from BRG_P_Y0 (-38.5) to JRN_P_Y1 (-39.4).
# Same Ø40, same cylinder, 0.9 mm longer - the horn cannot come to the journal
# (it is pinned to the spline, which reaches only SPLINE_TIP past the case top),
# so the journal goes to the horn. That 0.9 mm runs inside the mount's Ø48
# outer-race lip pocket, bored r=24 from y=-44.0, so the r=20 journal has 4.0 mm
# radially and its end face clears that pocket floor by 4.6 mm. The tall hub
# horn shortened this overhang from 3.6 mm - the driven face came IN to meet a
# taller part, and NO SPLINE-TIP RELIEF IS CUT IN IT ANY MORE: the tip finishes
# 2.1 mm down inside the horn hub and never reaches this face.
carrier = cylY(0, JRN_P_Y1, -27.0, C_Z, P_BORE / 2)                   # Ø40 journal
carrier = carrier.union(cylY(0, -27.0, -20.0, C_Z, CARR_R))           # hub disc
carrier = carrier.union(B(-36, 36, -26, -15.5, C_Z - 20, C_Z + 6))    # bridge
carrier = carrier.union(cylX(CARR_X0, CARR_X1, 0, C_Z, CARR_R))       # front plate
# SERVO R FACE PLATE, grown from z 65..113 to the deck's 61.5..118.5 so the ear
# deck lands on real plate over its whole 54.38 mm span; the Ø52 front disc only
# covers z 74..126 and stops 11 mm short of the lower ear tip on its own.
carrier = carrier.union(B(CARR_X0, CARR_X1, -DECK_R_HY, DECK_R_HY,
                          DECK_R_Z0, DECK_R_Z1))
# EAR DECK, butted straight onto that plate's inner face: one continuous slab
# x 22.4..36.0, relieved back to PLATE_R_X0 only where the case passes through.
carrier = carrier.union(B(DECK_R_X0, DECK_R_X1, -DECK_R_HY, DECK_R_HY,
                          DECK_R_Z0, DECK_R_Z1))
carrier = carrier.union(cylX(-CARR_X1, -CARR_X0, 0, C_Z, CARR_R))     # idler plate

# B. PITCH HORN. 4x M3 inserts on Ø14 into the journal END FACE at
#    y = JRN_P_Y1 = -39.4, bores running +Y (inboard) to -32.9. PITCH IS A
#    BENCH-FIT JOINT and it stays one: the horn bolts to the DRIVEN part first,
#    with 4x M3x6 through its own Ø3.2-drilled holes, and the spline enters the
#    hub later when servo P is fed into the mount. That is only possible because
#    the driven part is MONOLITHIC here - a solid journal end - so there is
#    nothing to lower on afterwards and no second face to reach past. (Twist is
#    the opposite case and is assembled the opposite way: see hub_clamp.)
#    Bores reach r 8.25 in a r=20 journal - 11.75 mm of wall - and the journal
#    runs 12.4 mm (-39.4..-27.0), so a full INSERT_D bore is blind by 5.9 mm.
for hx, hz in horn_points():
    carrier = carrier.cut(cylY(hx, JRN_P_Y1, JRN_P_Y1 + INSERT_D, C_Z + hz, INSERT_R))
# NOTE: the three r=15.5 insert bores that used to sit on this face belonged to
# hub_collar, and hub_collar is not part of this design - it was never in PARTS
# and never in check_meta, so those three inserts fastened nothing at all. Gone;
# the module now takes exactly 30 M3 inserts - 22 structural plus the EIGHT
# remaining horn positions. The twist four went with the drive inversion: that
# joint lands in the disc's own tapped aluminium, so it needs no metal of ours.

# A. SERVO R INSERTION CHANNEL. Orientation is UNCHANGED from v6 - body length
#    along z with the case hanging BELOW the roll axis (z 70.06..110.0, only
#    10.0 mm above it) - so the ears simply follow the body along z, tips at
#    62.84 and 117.22. The servo goes in spline-first (+X) from open space
#    between the front plate and the idler plate; this cut is the guaranteed
#    corridor, and it stops dead on the deck face.
carrier = carrier.cut(B(SRV_R_X - SRV_H - POCKET_CLR, DECK_R_X0,
                        -SRV_W / 2 - POCKET_CLR, SRV_W / 2 + POCKET_CLR,
                        DECK_R_Z0, DECK_R_Z1))
# CASE CUTOUT / CASE-TOP RELIEF, one cut: SRV_L+0.7 x SRV_W+0.6 about the body
# centre, from the deck face at 22.4 up to PLATE_R_X0 = 32.5. It takes the whole
# deck thickness and then the inboard 3.5 mm of the old front plate, whose inner
# face at 29.0 the case top (31.9) used to sit 2.9 mm inside. What is left of
# the plate is 32.5..35.3, with the Ø23 head-pocket floor at 33.4 - a 0.9 mm
# annular web at 6.5 < r < 11.5, the same as the pitch plate keeps.
carrier = carrier.cut(B(DECK_R_X0, PLATE_R_X0,
                        -(SRV_W + DECK_CLR_W) / 2, (SRV_W + DECK_CLR_W) / 2,
                        SRV_R_ZC - (SRV_L + DECK_CLR_L) / 2,
                        SRV_R_ZC + (SRV_L + DECK_CLR_L) / 2))
# GUSSET RELIEF - 3.0 wide x 2.5 deep along the ear centreline (y = 0)
carrier = carrier.cut(B(DECK_R_X0, DECK_R_X0 + DECK_GUS_D,
                        -DECK_GUS_W / 2, DECK_GUS_W / 2,
                        DECK_R_Z0, DECK_R_Z1))
# SPLINE PATH - Ø13 (r 6.5), FULL DEPTH of what is left of the front plate. The
# Ø12.5 output boss stands BOSS_H proud of the case top, so it occupies
# x 31.9..33.4 and crosses this plate's underside at 32.5; 0.25 mm radial. The
# Ø7.3 horn hub runs 33.4..39.1 through the rest; 2.85 mm radial.
carrier = carrier.cut(cylX(PLATE_R_X0 - 1, CARR_X1 + 1, 0, C_Z, 6.5))
# HEAD-CLEARANCE POCKET - Ø23 and SHALLOW, from the plate OUTER face at
# CARR_X1 = 35.3, and like the pitch one IT IS NOT FOR THE DISC. The Ø19.7 roll
# disc lies at x 36.9..39.1, wholly outboard of this plate with 1.6 mm of air.
# The FOUR DRIVE SCREW HEADS under it (ring r 4.1..9.9, x 33.9..36.9) reach
# 1.4 mm inside it, so the floor is SRV_R_X + HORN_FLOOR = 33.4 - the boss top
# plane - and the heads spin over it with HORN_GAP = 0.5. Plate 32.5..35.3, so
# 6.5 < r < 11.5 keeps a 0.9 mm web; rim to the r=26 disc edge, 14.5 mm. No
# insert is near it - the four ear bores end at x = 27.9, 5.5 mm clear.
carrier = carrier.cut(cylX(SRV_R_X + HORN_FLOOR, CARR_X1 + 0.5,
                           0, C_Z, HORN_CLR_R))

# C. SERVO R EAR INSERTS - 4x M3, TAB_INS_D deep, into the DECK FACE at
#    x = 22.4 on the 49.5 x 10 ear pattern. Mapping for this orientation: the
#    pattern's long axis (hx) runs along -z about C_Z, its short axis (hy) along
#    y. Bores land at (y +-5.0, z 65.28) and (y +-5.0, z 114.78), running +X to
#    27.9 - blind by 1.1 mm inside the 22.4..29.0 deck and 1.68/1.62 mm inside
#    its z ends. Both pairs are OUTSIDE the case cutout in z (69.71..110.35).
#    The 4x M3x8 come in from -X through the servo's own Ø4.6 ear slots, and
#    that corridor is open space - nothing of the carrier lives between the
#    idler plate at x = -29 and the deck at 22.4 except the bridge, which is at
#    y -26..-15.5 and never crosses the y = +-5 screw lines.
for hx, hy in tab_points():
    carrier = carrier.cut(cylX(DECK_R_X0, DECK_R_X0 + TAB_INS_D, hy,
                               C_Z - hx, INSERT_R))

# idler side: 625ZZ seat + shoulder-bolt pass
carrier = carrier.cut(cylX(-CARR_X1 - 0.5, -31.0, 0, C_Z, 8.05))
carrier = carrier.cut(cylX(-32.0, -CARR_X0 + 1, 0, C_Z, 3.2))

# =============================================================================
# SERVO R  (shaft along +X)
# =============================================================================
servoR_body, servoR_shaft = servo()
_rr = lambda w: w.rotate((0, 0, 0), (0, 1, 0), 90).translate((SRV_R_X, 0, C_Z))
servoR_body, servoR_shaft = _rr(servoR_body), _rr(servoR_shaft)

# =============================================================================
# YOKE : rides the roll axis, carries the twist bearing and servo T
# =============================================================================
yoke = None
for s in (1, -1):
    x0 = YOKE_X0 if s > 0 else -YOKE_X1
    disc = cylX(x0, x0 + 6, 0, C_Z, 24)
    if s > 0:
        # ROLL DRIVE BOSS. The roll disc inner face is at YOKE_X0 = 40.0 (the
        # disc is 40..46, there is no 3 mm step here), 8.1 mm off servo R's case
        # top - 0.9 mm past the horn's driven face. A Ø23 boss carries that face
        # inboard those 0.9 mm to ROLL_FACE_X = 39.1. It used to have to reach
        # 3.6 mm: the tall hub horn brought the driven face out to meet it.
        # NO SPLINE-TIP RELIEF - the tip stops 2.1 mm down inside the hub.
        disc = disc.union(cylX(ROLL_FACE_X, YOKE_X0, 0, C_Z, ROLL_BOSS_R))
    arm = (cq.Workplane("YZ", origin=(x0, 0, 0))
           .polyline([(-22, C_Z - 8), (22, C_Z - 8), (30, PLAT_Z1), (-30, PLAT_Z1)])
           .close().extrude(6)
           .cut(cylX(x0 - 1, x0 + 7, 0, PLAT_Z0 - 30, 12)))
    yoke = disc.union(arm) if yoke is None else yoke.union(disc.union(arm))

# B. ROLL HORN. 4x M3 inserts on Ø14 into the BOSS face at x = ROLL_FACE_X =
#    39.1, bores running +X (outboard) to 45.6. ROLL IS A BENCH-FIT JOINT and it
#    stays one, for the same reason pitch is: the driven part is MONOLITHIC - a
#    solid boss on a solid disc - so the horn goes on it first with 4x M3x6
#    through its Ø3.2-drilled holes, and the spline enters the hub later, on the
#    +X push that seats servo R (ASSEMBLY ORDER 11).
#    Bores reach r 8.25 inside the r=11.5 boss: 3.25 mm of wall. Depth is the
#    tight one: boss 0.9 + disc 6.0 = 6.9 mm of material, so an INSERT_D = 6.5
#    prototype pilot is blind by 0.4 mm and a 5.5 insert bore by 1.4 mm. That
#    0.4 is the thinnest bore floor in the module - print it solid.
for hy, hz in horn_points():
    yoke = yoke.cut(cylX(ROLL_FACE_X, ROLL_FACE_X + INSERT_D, hy, C_Z + hz, INSERT_R))
yoke = yoke.cut(cylX(-YOKE_X1 - 1, -YOKE_X0 + 1, 0, C_Z, 2.55))       # idler pin

# PLATFORM. Full width to x = +/-40 so it meets the roll arms directly; v6.0
# needed two separate webs to bridge a 4 mm gap and still sliced as 3 bodies.
plat = B(-40, 40, -32, 32, PLAT_Z0, PLAT_Z1)
plat = plat.cut(cylZ(0, 0, BRG_T_Z0, BRG_T_Z1 + 1, T_OD / 2 + 0.05))  # Ø42 seat
plat = plat.cut(cylZ(0, 0, PLAT_Z0 - 1, BRG_T_Z0, CAP_ID))            # lip + hub_clamp clr
for rx, ry in circle_points(CAP_BC, (45, 135, 225, 315)):             # race-cap bolts
    plat = plat.cut(cylZ(rx, ry, PLAT_Z1 - INSERT_D, PLAT_Z1 + 1, INSERT_R))
plat = plat.cut(cylZ(-34, 26, PLAT_Z0 - 1, PLAT_Z1 + 1, 4.0))         # cable Ø8
yoke = yoke.union(plat)

# SERVO T BRACKET, hung under the platform: face plate + two walls.
# Widened to y -18.5..38.5: the HTS ears run along y (tips at SRV_OFF
# +/-27.19 = -17.2..37.2) and their screws at -14.8/34.7 need real material.
# The DECK (ear-top bearing face at SRV_T_Z - 9.5 = 169.4) merges into the
# face plate above it - one thickened structure 169.4..BRK_Z1.
brk = B(-BRK_HX, BRK_HX, BRK_Y0, BRK_Y1, BRK_Z0, BRK_Z1)
brk = brk.union(B(14, BRK_HX, BRK_Y0, BRK_Y1, BRK_Z1, PLAT_Z0))
brk = brk.union(B(-BRK_HX, -14, BRK_Y0, BRK_Y1, BRK_Z1, PLAT_Z0))
# CASE CUTOUT / CASE-TOP RELIEF, one cut: SRV_L+0.7 x SRV_W+0.6 about the body
# centre at y = SRV_T_YC, from the deck face at 169.4 to the horn-plate
# underside at PLATE_T_Z0 = 179.5. The ears stop on the 169.4 face below it.
brk = brk.cut(B(-(SRV_W + DECK_CLR_W) / 2, (SRV_W + DECK_CLR_W) / 2,
                SRV_T_YC - (SRV_L + DECK_CLR_L) / 2,
                SRV_T_YC + (SRV_L + DECK_CLR_L) / 2,
                BRK_Z0, PLATE_T_Z0))
# and the same window carried across the FULL bracket in y through the dead band
# between deck top and plate underside - nothing lives there and it is 4.1 mm of
# solid otherwise. Deck and plate stay joined by the two x flanks, 6.68 mm each.
brk = brk.cut(B(-(SRV_W + DECK_CLR_W) / 2, (SRV_W + DECK_CLR_W) / 2,
                BRK_Y0 - 1, BRK_Y1 + 1, DECK_T_Z1, PLATE_T_Z0))
# GUSSET RELIEF - 3.0 wide x 2.5 deep along the ear centreline (x = 0)
brk = brk.cut(B(-DECK_GUS_W / 2, DECK_GUS_W / 2, BRK_Y0 - 1, BRK_Y1 + 1,
                BRK_Z0, BRK_Z0 + DECK_GUS_D))
brk = brk.cut(cylZ(0, 0, BRK_Z1 - 1, PLAT_Z0 + 0.5, CAP_ID))          # hub_clamp + horn clr
# SPLINE PATH - Ø13 (r 6.5), FULL DEPTH of the horn plate. The Ø12.5 output boss
# stands BOSS_H proud of the case top, so it occupies z 169.9..171.4 and crosses
# this plate's underside at 170.5; 0.25 mm radial. The Ø7.3 horn hub takes it
# from 171.4 up to the driven face at 177.1; 2.85 mm radial.
brk = brk.cut(cylZ(0, 0, PLATE_T_Z0 - 1, BRK_Z1 + 1, 6.5))            # spline path
# NO HORN POCKET ON THIS PLATE, and that is the twist inversion paying out. The
# other two joints keep a Ø23 relief because their drive screws hang HEAD-DOWN
# under the disc; the twist screws are driven the other way, DOWN from
# hub_clamp's top face into the disc's own threads, so this disc's underside is
# bare metal. Nothing reaches: disc bottom 174.9 against a plate whose top is
# 171.9 once the CAP_ID bore has taken r < 19.5 - 3.0 mm of clear air.
# C. SERVO T EAR INSERTS - 4x M3, TAB_INS_D deep, into the DECK FACE at
#    z = 169.4 on the 49.5 x 10 ear pattern. Mapping for this orientation: the
#    pattern's long axis (hx) runs along +y, its short axis (hy) along -x.
#    Bores land at (x +-5.0, y -14.78) and (x +-5.0, y 34.72), running +Z to
#    174.9 - blind by 0.5 mm under the 175.4 deck top, with 1.62/1.68 mm of
#    bracket outside them in y. THIS IS WHY THE BRACKET WAS WIDENED: at the old
#    y -12..37 the y = -14.78 pair was 2.78 mm outside the part entirely.
#    The 4x M3x8 rise from below through the servo's own Ø4.6 ear slots into
#    open air - servo T hangs clear under the platform and nothing is beneath
#    it at any pitch/roll pose, let alone at assembly time (step 3, and
#    the twist drive is built on top of it in steps 4 and 5).
for hx, hy in tab_points():
    brk = brk.cut(cylZ(-hy, hx, BRK_Z0, BRK_Z0 + TAB_INS_D, INSERT_R))
yoke = yoke.union(brk)
# THE hub_clamp WALL NOTCH IS GONE, and it was measured out, not argued out.
# It cut r < 18.75 over HC_Z0-1.5 .. HC_Z1+1.0 to let the Ø36 clamp descend past
# the bracket walls; every millimetre of that span is already void, because the
# CAP_ID bore takes r < 19.5 from 171.9 to 180.5, the platform takes the same
# r < 19.5 from 179.0 to 184.5, and the Ø42 bearing seat takes r < 21.05 above
# that. Deleting the cut leaves the yoke volume at 120791.611 mm^3 - identical
# to seven figures - so the descent corridor is unchanged and one redundant
# feature is off the part.

# roll idler: M5 shoulder bolt + 625ZZ (bought)
roll_idler = (cylX(-49, -YOKE_X1, 0, C_Z, 5)
              .union(cylX(-YOKE_X1, -31, 0, C_Z, 2.5))
              .union(cylX(-36, -31, 0, C_Z, 8)))

# =============================================================================
# SERVO T  (shaft = humerus axis)
# =============================================================================
servoT_body, servoT_shaft = servo(drilled=False)
servoT_body = servoT_body.rotate((0, 0, 0), (0, 0, 1), 90).translate((0, 0, SRV_T_Z))
servoT_shaft = servoT_shaft.rotate((0, 0, 0), (0, 0, 1), 90).translate((0, 0, SRV_T_Z))

# =============================================================================
# BEARINGS AND RETAINERS
# =============================================================================
# 625ZZ roll idler bearing - a bought part, exported like the others so the
# renders carry its real bore (a hand-built STL had inverted face normals and
# rendered as either a solid puck or an invisible ring, depending on angle).
idler625 = (cylX(-36.5, -31.5, 0, C_Z, 8.0)
            .cut(cylX(-37.0, -31.0, 0, C_Z, 2.5)))

brg_pitch = (cylY(0, BRG_P_Y0, BRG_P_Y1, C_Z, P_OD / 2)
             .cut(cylY(0, BRG_P_Y0 - 1, BRG_P_Y1 + 1, C_Z, P_BORE / 2)))
brg_twist = ring(T_BORE / 2, T_OD / 2, BRG_T_Z0, BRG_T_Z1)

# D. RACE CAP. ID Ø39 grips the 6806 outer race (Ø39..Ø42) and clears the
#    plate's Ø34 rotating shoulder by 2.5 mm. v6.0 had ID Ø36 against a Ø40
#    shoulder - 2828 mm^3 of solid overlap, the cap could not be fitted - and
#    its bolt circle was at r=39, outside its own Ø56 rim, so it had no holes.
#    The four screws are M3x8 COUNTERSUNK and they are driven BEFORE the
#    interface plate goes on - see ASSEMBLY ORDER 8. Socket caps cannot be used
#    here at all: the plate's underside sits 1.5 mm above CAP_Z1 and covers the
#    whole r=25 circle at every twist angle. A 90 deg csk, Ø6.2 at the top face,
#    sinks a DIN 7991 head flush and still leaves 1.6 mm of cap beneath it.
race_cap = ring(CAP_ID, CAP_OD, CAP_Z0, CAP_Z1)
for rx, ry in circle_points(CAP_BC, (45, 135, 225, 315)):
    race_cap = race_cap.cut(cylZ(rx, ry, CAP_Z0 - 1, CAP_Z1 + 1, 1.7))
    race_cap = race_cap.cut(cq.Workplane("XY").add(
        cq.Solid.makeCone(1.7, 3.1, 1.4, cq.Vector(rx, ry, CAP_Z1 - 1.4))))

# E. HUB CLAMP. Bolted to a SOLID Ø30 journal on a r=11 circle (Ø22, was Ø18).
#    The clamp own insert bores span r 8.9..13.1 in the Ø36 clamp, 4.9 mm of
#    wall; the interface plate Ø3.4 through-holes span r 9.3..12.7 in the Ø30
#    journal, a 2.3 mm wall - that 2.3 (from 4.3) is what moving the circle out
#    cost, and it is still the thinnest section in the plate journal. These
#    three M3 carry the whole hanging arm in tension, and in v6.0 they broke
#    straight out through a 4.2 mm journal wall.
#    THE TWIST DRIVE IS INVERTED AND THIS PART IS WHERE IT SHOWS. hub_clamp no
#    longer carries the horn on its underside - it carries the SCREWS. The disc
#    goes on the servo's own spline FIRST, alone, retained by the servo's own
#    factory M3x6 centre screw while that screw is still reachable; hub_clamp
#    then comes DOWN onto the disc's top face and four M3x6 are driven DOWNWARD
#    through it into THE DISC'S OWN TAPPED M3 HOLES. Aluminium threads, in the
#    bought part, used as the maker cut them: no drill-out, no inserts, no
#    bench-fit, and four heat-set positions deleted from this part outright
#    (the module is on 30 inserts now, not 34).
#    Why it can be inverted here and not on pitch or roll: those two DRIVEN
#    PARTS ARE MONOLITHIC. There is no second piece to lower on afterwards, so
#    the horn must be bolted to them on the bench and their centre screws are
#    blind forever. hub_clamp is a separate piece that arrives from above, which
#    is exactly what leaves the centre screw in open air at step 4.
#    The part is 7.4 mm deep (177.1..184.5), set at both ends by parts that are
#    not negotiable: HC_Z1 is the 6806 seat, HC_Z0 is the horn's driven face.
#    The three r=11 bores keep INSERT_D + 0.5, floor 177.5, because that floor
#    is set by the M3x20 tip at 179.0 - not by the thickness. See ASSEMBLY
#    ORDER 4, 5 and 9 and the worked clearance below.
hub_clamp = cylZ(0, 0, HC_Z0, HC_Z1, HC_R)
# CENTRE-SCREW RELIEF, Ø8 x 3.5, up from the underside. The factory M3x6 is
# already in the horn when this part lands on it and NOBODY PUBLISHES how deep
# the horn's own centre recess is, so the clamp does not bet on it: 3.5 mm
# swallows a full 3.0 mm cap head even if the recess turns out to be zero. It
# stops 1.3 mm short of the drive holes' Ø3.4 bores at r 5.3.
hub_clamp = hub_clamp.cut(cylZ(0, 0, HC_Z0, HC_Z0 + 3.5, 4.0))
for rx, ry in circle_points(HC_BC, (0, 120, 240)):
    hub_clamp = hub_clamp.cut(cylZ(rx, ry, HC_Z1 - INSERT_D - 0.5, HC_Z1 + 1, INSERT_R))
# TWIST DRIVE - 4x COUNTERBORED THROUGH-HOLES on Ø14, NOT inserts. Ø3.4 clean
# through 177.1..184.5, with a Ø5.8 x 3.0 counterbore from the TOP face. The
# counterbore is not cosmetic: hub_clamp's top face at r < 15 IS the seat for
# the interface plate's Ø30 journal, so a proud head would hold the whole twist
# stack off its own bearing. Sunk 3.0 the M3 cap head finishes flush with
# HC_Z1 = 184.5.
# LENGTH IS M3x6, AND IT IS NOT M3x8. Stack from the counterbore floor: 4.4 mm
# of clamp, then 2.2 mm of tapped disc and no more - the disc IS the thread and
# it is only 2.2 thick. 6.0 engages 1.6 of that 2.2 and stops 0.6 mm short of
# the disc's far face, so the whole fastener is contained inside the sandwich.
# An M3x8 is NOT a clash here - it takes the full 2.2 and its tip still clears
# the bracket plate top at 171.9 by 1.6 mm. It is a CHOICE. 6 is what pitch and
# roll are FORCED to (2.2 of disc + a 5.5 insert bore bottoms an 8 before its
# head clamps), so taking 6 here makes every horn screw in the module one SKU -
# the same M3x6 the horn's own centre screw is - and leaves no rotating screw
# tip hanging in a gap no sweep in this project can see. Load check: 3.43 N.m
# through four screws on the O14 circle is 122 N each, and 1.6 mm of M3 thread
# in aluminium is an order of magnitude past that.
# CLOCKED 45 deg - HORN_A_T, not HORN_A - and this is still the only horn of the
# three that is, for the same reason as before: these four holes and the three
# r=11 plate-bolt bores now share the TOP 3.0 mm of the part instead of a middle
# 2.5, so they have to clear each other laterally. A 3-hole circle reduces mod
# 90 deg to phi, phi+30, phi+60, so the best any phi can do against a 4-hole
# 90 deg circle is 15 deg - which 0/120/240 against the 45 family already is.
# Closest pair (r=7, 135 deg) against (r=11, 120 deg):
#     d    = sqrt(7^2 + 11^2 - 2*7*11*cos 15) = 4.6095 mm
#     wall = 4.6095 - 2.9 (counterbore) - 1.25 (Ø2.5 pilot) = 0.4595 mm
#            over the top 3.0 mm, and 4.6095 - 1.7 - 1.25 = 1.6595 mm below it
# That 0.46 is the tightest feature in the part - print it solid.
# AND IT IS THE HARD LIMIT ON THIS CIRCLE: with PROTOTYPE_THREADS = False the
# three bores go to Ø4.2 for heat-set inserts, r 2.1, and 4.6095 - 2.9 - 2.1 is
# NEGATIVE. The counterbore breaks into the insert bore - a 1.9 mm wide, 0.23 mm
# deep scallop down its top 3.0 mm. Not a hole into open air, but not a wall
# either. So on the insert build these three either stay thread-formed, or
# HC_BC moves to 12.0 (d = 5.5430, wall 0.543) and the plate journal wall pays
# for it, 2.3 -> 1.3 mm. Decide it with the load, not with the slicer.
for rx, ry in circle_points(HORN_BC, HORN_A_T):
    hub_clamp = hub_clamp.cut(cylZ(rx, ry, HC_Z0 - 1, HC_Z1 + 1, 1.7))
    hub_clamp = hub_clamp.cut(cylZ(rx, ry, HC_Z1 - 3.0, HC_Z1 + 1, 2.9))

# =============================================================================
# INTERFACE PLATE : Ø30 journal through the bearing, bicep tray on top
# =============================================================================
oct_pts = [(29, 22), (18, 33), (-18, 33), (-29, 22),
           (-29, -22), (-18, -33), (18, -33), (29, -22)]
plate = (cq.Workplane("XY", origin=(0, 0, IF_Z0)).polyline(oct_pts).close()
         .extrude(IF_Z1 - IF_Z0))
plate = plate.union(cylZ(0, 0, BRG_T_Z0, IF_Z0, T_BORE / 2))           # journal (solid)
plate = plate.union(cylZ(0, 0, BRG_T_Z1, IF_Z0, SHOULDER_R))           # inner-race shoulder
tray = (cq.Workplane("XY", origin=(0, 0, IF_Z1)).polyline(oct_pts).close()
        .extrude(TRAY_Z1 - IF_Z1)
        .cut(B(-BICEP_X / 2 - 0.3, BICEP_X / 2 + 0.3,
               -BICEP_Y / 2 - 0.3, BICEP_Y / 2 + 0.3, IF_Z1 - 1, TRAY_Z1 + 1)))
plate = plate.union(tray)
# 3x M3x20 down into hub_clamp's bores. O3.4 through the plate, plus a
# O5.8 x 3.0 counterbore in the TOP face: without it the only screw that reaches
# is an M3x25, and 25 into a 6.0 deep bore bottoms 1.5 mm early on a part too
# thin to drill deeper. Sunk 3.0 the head seats at 199.0, an M3x20 tip lands at
# 179.0 against a bore floor of 177.5, and nothing stands proud into the bicep
# cavity.
for rx, ry in circle_points(HC_BC, (0, 120, 240)):
    plate = plate.cut(cylZ(rx, ry, BRG_T_Z0 - 1, IF_Z1 + 0.5, 1.7))
    plate = plate.cut(cylZ(rx, ry, IF_Z1 - 3.0, IF_Z1 + 0.5, 2.9))
# F. BICEP BOLTS, counterbored from below. Driven up into inserts in the bicep,
#    heads buried in the plate so nothing sweeps over race_cap. They still have
#    to be fitted before the plate goes on the bearing - see ASSEMBLY ORDER 6.
for sx in (1, -1):
    for sy in (1, -1):
        plate = plate.cut(cylZ(sx * BOLT_DX, sy * BOLT_DY, IF_Z0 - 1, IF_Z1 + 1, 2.2))
        plate = plate.cut(cylZ(sx * BOLT_DX, sy * BOLT_DY, IF_Z0 - 1, IF_Z0 + 4.5, 3.7))

bicep_ref = B(-BICEP_X / 2, BICEP_X / 2, -BICEP_Y / 2, BICEP_Y / 2,
              IF_Z1 + 0.5, IF_Z1 + 0.5 + BICEP_L)

# =============================================================================
# EXPORT
# =============================================================================
# =============================================================================
C = lambda r, g, b: cq.Color(r / 255, g / 255, b / 255)
PARTS = {
    "mount":           (mount,           C(235, 140, 52)),
    "pitch_retainer":  (pitch_retainer,  C(120, 130, 145)),
    "servoP_body":     (servoP_body,     C(200, 45, 45)),
    "servoP_shaft":    (servoP_shaft,    C(30, 150, 160)),
    "carrier":         (carrier,         C(235, 140, 52)),
    "servoR_body":     (servoR_body,     C(200, 45, 45)),
    "servoR_shaft":    (servoR_shaft,    C(30, 150, 160)),
    "yoke":            (yoke,            C(235, 140, 52)),
    "roll_idler":      (roll_idler,      C(90, 60, 110)),
    "brg_pitch":       (brg_pitch,       C(90, 60, 110)),
    "idler625":        (idler625,        C(107, 71, 140)),
    "brg_twist":       (brg_twist,       C(90, 60, 110)),
    "race_cap":        (race_cap,        C(120, 130, 145)),
    "hub_clamp":       (hub_clamp,       C(120, 130, 145)),
    "servoT_body":     (servoT_body,     C(200, 45, 45)),
    "servoT_shaft":    (servoT_shaft,    C(30, 150, 160)),
    "interface_plate": (plate,           C(70, 72, 78)),
    "bicep_ref":       (bicep_ref,       C(202, 209, 238)),
}

if __name__ == "__main__":
    assy = cq.Assembly(name="shoulder_v6")
    for name, (part, color) in PARTS.items():
        solid = part if SIDE > 0 else part.mirror("XZ")
        assy.add(solid, name=name, color=color)
        cq.exporters.export(solid, os.path.join(OUT, f"{name}.step"))
        cq.exporters.export(solid, os.path.join(OUT, f"{name}.stl"), tolerance=0.05)
    assy.save(os.path.join(OUT, "shoulder_assembly.step"))
    print("exported STEP assembly +", len(PARTS), "parts (step+stl)")

# metadata for sim36/interference_check.py, written next to the STLs so the
# checker reads the kinematics from the design instead of holding its own copy.
#
# EVERY exported part is named in a group. The checker now fails if a part is
# named but not exported AND if a part is exported but not named - the second
# one is how hub_collar, pitch_retainer and bicep_ref went unchecked while
# hub_collar sat 1 mm inside the mount's outer-race lip.
_META = {
    "axes": {"pitch": [0, -1, 0], "roll": [1, 0, 0], "twist": [0, 0, 1]},
    "origin": {"pitch": [0, 0, 100], "roll": [0, 0, 100], "twist": [0, 0, 100]},
    "chain": {"base": [], "carrier": ["pitch"], "yoke": ["pitch", "roll"],
              "plate": ["pitch", "roll", "twist"]},
    "groups": {
        "base":    ["mount", "pitch_retainer", "servoP_body", "brg_pitch"],
        "carrier": ["carrier", "servoP_shaft", "servoR_body",
                    "roll_idler"],
        "yoke":    ["yoke", "servoR_shaft", "servoT_body", "brg_twist",
                    "race_cap"],
        "plate":   ["interface_plate", "hub_clamp", "servoT_shaft", "bicep_ref"],
    },
    # BONDED: pairs that are press-fit or bolted together and are SUPPOSED to
    # touch. The checker no longer just skips these - it puts A's surface points
    # through B's mesh and fails if any of them is more than BOND_TOL inside it.
    # Touching is allowed; interpenetration is not. That is the only reason a
    # servo body may be bonded to the bracket it bolts to.
    "bonded": [
        ["mount", "brg_pitch"], ["mount", "pitch_retainer"],
        ["mount", "servoP_body"],
        ["brg_pitch", "pitch_retainer"],         ["carrier", "brg_pitch"], 
                       ["carrier", "servoP_shaft"], ["carrier", "servoR_body"],
        ["carrier", "roll_idler"],
        ["yoke", "brg_twist"], ["yoke", "race_cap"], ["yoke", "roll_idler"],
        ["yoke", "servoR_shaft"], ["yoke", "servoT_body"],
        ["brg_twist", "race_cap"], ["brg_twist", "hub_clamp"],
        ["interface_plate", "brg_twist"], ["interface_plate", "hub_clamp"],
        ["interface_plate", "bicep_ref"],
        ["hub_clamp", "servoT_shaft"],
        ["servoP_body", "servoP_shaft"], ["servoR_body", "servoR_shaft"],
        ["servoT_body", "servoT_shaft"],
    ],
    # Nothing is excluded any more. Every drive and bearing interface that used
    # to be waved through by name is now an explicit bonded pair with a real
    # interpenetration test behind it.
    "exclude": {"base|carrier": [], "carrier|yoke": [], "yoke|plate": []},
    # DESIGNED running clearances below the checker's global MIN_GAP. THERE ARE
    # NONE LEFT, and that is a result, not an omission. The three entries that
    # used to live here were each a Ø20 horn face turning 0.5 mm over its pocket
    # floor. The hub+disc horn carries its disc 2.2 mm further out on every
    # axis, so the disc now clears the plate it spins over by 2.4 (pitch),
    # 1.6 (roll) and 3.0 mm (twist) and every pair clears MIN_GAP on its own.
    # WHAT IS NOT POLICED HERE: HORN_GAP = 0.5 under the pitch and roll DRIVE
    # SCREW HEADS. Screws are not modelled solids in this build, so no sweep can
    # see them - it is a bench-verify item and the two Ø23 pockets are its only
    # record. If a future edit models the screws, put those pairs back at 0.4.
    "min_override": {},
    "travel": {
        "pitch": [-60, -30, 0, 45, 90, 135, 180],
        "roll": [-25, -15, -5, 0, 30, 60, 90, 120, 140, 150],
        "twist": [-90, -45, 0, 45, 90],
    },
}
with open(os.path.join(OUT, "check_meta.json"), "w") as _f:
    import json as _json
    _json.dump(_META, _f, indent=1)
print("wrote check_meta.json")
