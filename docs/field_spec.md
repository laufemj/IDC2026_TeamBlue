# Field Specification — IDC2026 "AI to Science" (Physical World game)

**Sources, in order of authority:** `[RB pN]` = official Contest Rule Book Ver 1.01
(05 Aug 2026), page N — the only normative source; `[CAD]` = organisers' arena CAD
(`Gametablepart*.stl`); `[TAPE]` = tape-measured on the real artifacts; `[PHOTO]` =
127 venue photos (2026-08-12); `[TEAM]` = decisions/observations by our own team,
unverified by rules. The rulebook contains almost **no dimensions** — most numbers
below are measured, and every unverified one is flagged `TODO-FIELD`.

**Frame note:** all (x, y) below assume we drew the half where Start A sits at the
field corner (+x side) and the robot is placed facing straight down-slope.
Origin = centre of our start position, y = metres forward (robot's initial facing),
x = metres to the robot's initial right. Initial orientation is **our choice, not
specified** `[RB]`. If we draw the mirrored half, the mirror is absorbed by the
frame definition, but the *opponent's* goals swap sides; see section D.

---

## A. Values table

| ref | value | source |
|---|---|---|
| **1.1** field size | plan length **3.586 m** (slope run) x width **2.39 m** (`TODO-FIELD`: STL says 2.70; mat arithmetic 5 x 478 mm says 2.39 — one tape measure settles it). The table is shared, symmetric; "our share" is our half by convention, but **no rule confines the autonomous robot to one half** — only "may not drive off the field" [RB p8-g] | [CAD]+[TEAM] |
| **1.2** slope | **5.0 deg**, uniform single ramp, rise 0.314 m over 3.586 m. **Downhill = +y (robot's forward)** — Start A is at the TOP, goals at the BOTTOM edge. Flat sections: Start A/B pads, and Zone W / Zone P raised **0.20 m** [RB p4] | [CAD] / [RB p4] |
| **1.3** floor | lime/yellow-green EVA foam mats, faint two-tone tile variation, tile pitch 478 mm; slope surface same material | [PHOTO] |
| **1.4** walls | **not specified — measure.** Photos show low table edges, no tall walls around the slope; nothing in the rules about borders | [PHOTO] |
| **2.1** lines | **black tape, ~50 mm wide** (`TODO-FIELD`), on lime floor. (a) One lane per goal, running up-field (constant x) **through each tunnel's bore**: at x = -0.85, -0.55, **+0.55, +0.85** (own pair bold), from the low edge (y ≈ 3.28) up-slope — upper endpoint **not measured**. (b) Cross lines at 478 mm pitch — endpoints **not measured**. **The rulebook never mentions floor lines: they carry no rule meaning, no crossing penalty. Navigation aids only.** | [PHOTO]+[TEAM] |
| **2.2** non-line markings | start-zone perimeter taped in **team colour (red or blue) masking tape** around Start A. No circles/arrows/text observed | [PHOTO]/[TEAM] |
| **2.3** unique vs repeated | UNIQUE: coloured start-zone tape (one per half); the 4-goal row at the low edge (pairs distinguishable by side). REPEATED: the 4 black lanes look alike; cross lines all alike. Figures are unique *objects* but their goal assignment is random (see 4.2) | [PHOTO]+[RB p5] |
| **3.1** start zone | Start A = raised pad at the **top corner** of the slope, our-frame origin; field position ≈ (x=1.05, y=3.40 from low edge) (`TODO-FIELD`). Pad size **not specified — measure.** Required placement inside it and orientation: **not specified** — robot must simply "start from Start A" [RB p5] | [RB p5]+[TEAM] |
| **3.2** two sides | yes, mirrored red (left) / blue (right) halves [RB p4]. Side is **announced >= 5 minutes before the match** [RB p8-f]; sensing it is optional, but we sense the start-tape colour as a config-error guard | [RB p4, p8] |
| **4.1** figures | wooden artist mannequins, standing height **300 mm**, shoulder span **54 mm** [TAPE]. Human figure: grey cloth garments + bare wood; robot figure: silver painted. One of each **per half** (4 total) | [TAPE]/[PHOTO]/[RB p5] |
| **4.2** placement | seated ON Goal A and Goal B tops. **Which figure on which goal is randomized every match**; opponent half is a mirrored copy [RB p5]. Poses freely chosen (by us: opponent's robot figure + our human figure), constrained: **pelvis must contact goal top, must not obstruct ball passage** [RB p5] | [RB p5] |
| **4.3** role | **labels, not obstacles**: the figure on a goal declares that goal's required ball colour (human -> orange, robot -> silver). **Robots must not touch figures** [RB p6] | [RB p5-6] |
| **5.1** goals | two tunnel boxes per half at the **low edge**, ball passes through the bore. Own pair centres (robot frame): **(0.55, 3.28)** and **(0.85, 3.28)**; 0.30 m within pair, pair centres +-0.35 m about the half's centreline (`TODO-FIELD`). Box: 200 W x 195 D mm, wall 12 mm (bore ≈ 176 mm wide), height 191 mm uphill / 212 mm downhill so the **top is level**; tunnel centre ≈ 0.12 m from the low edge (`TODO-FIELD`) | [TAPE]+[TEAM] |
| **5.2** marking | boxes are plain white/wood — **the ONLY colour-assignment marking is the figure sitting on top.** Vision must classify the figure, not the goal | [PHOTO]/[RB p5] |
| **5.3** fixed? | positions fixed & mirrored; **colour assignment random per match** [RB p5] | [RB p5] |
| **6.1** balls | **55 per field, orange + silver mix — the split is not specified.** Diameter: **not in the rulebook** — team believes 60 mm; confirm on the kit ball | [RB p4] |
| **6.2** entry | stacked on a platform per field (the side extensions by Zone W/P per Fig. 1 — position **not measured**). Autonomous robot may **preload 1 orange + 1 silver + cyber-game advantage balls** [RB p5-6]. How balls get from platform to the autonomous robot: **not specified — that's the teleop robot's / our design's job** | [RB p4-6] |
| **7.1** scoring | correct-colour ball in a goal = **1 pt** (orange -> human-figure goal, silver -> robot-figure goal). Goal with **>1 ball, all correct colour -> that goal's score doubles**. Balls outside goals at the end count zero | [RB p6] |
| **7.2** duration | **3 minutes**, no per-phase limits | [RB p6] |
| **7.3** penalties | no point penalties defined. Absolute prohibitions: touch a figure; drive off the field; damage the table; reverse opponent scoring. No rule about lines, zones for the autonomous robot, or entering the opponent half. (Slope / Zone S / Zone P touch-bans apply to the **teleoperated** robot only — the slope is legal ground for us) | [RB p5-8] |
| **7.4** retry | **none** — humans may not affect machines on the table during a round; parts that leave the table stay out [RB p8-e,f]. No mid-match repositioning | [RB p8] |
| **8** | see section D | |
| **9.1** size | autonomous: **fits 50 cm cube at start**; deployed size **not specified**. Weight: **not specified**. (Teleop: 50 x 80 x 80 cm) | [RB p6] |
| **9.2** sensors | no restriction on on-sensor NN or illumination in the rulebook; but **electric components must be authorized by referees** [RB p7-a] — clear active illumination with them. Energy <= 30 kJ total, air <= 0.5 MPa | [RB p7] |
| **9.3** lighting | **not specified.** Venue reality: fluorescent ceiling strips, green doors, windows, cardboard/furniture clutter behind the low edge — i.e., directly behind the goals in our camera's view | [PHOTO] |

**Tie-breaker that changes navigation** [RB p6]: score tie -> cyber-game points;
still tied -> **the team whose autonomous robot ends nearer the centre of its own
Start A wins**. Ending the match back at/near Start A has real value.

---

## B. Top-view sketch (our half drawn; opponent half mirrored to the right)

```
 y=0 at OUR start centre, +y = forward = DOWNHILL, +x = robot's right
 (field coords in brackets)                        DOWNHILL v = +y

        x= -0.15        0        0.55   0.85       1.25  1.55        2.25
  y=-0.19 +-+--------------------------------------------------------+ top edge
          |S|<- Start A pad, tape in team colour                      |
   y=0    |*|  * = ORIGIN, robot faces v                    Start A'  |
          +-+         (top of 5.0 deg slope, z rises 0.314 toward here)
          |                                                           |
          |              :       :      :          :     :            |
          |   cross      :lane   :lane  :lane      :lane :  black     |
          |   lines -----+-------+------+----------+-----+  tape      |
          |   every      :       :      :          :     :  50mm      |
          |   0.478m ----+-------+------+----------+-----+            |
          |              :       :      :          :     :            |
  y=3.28  |            [G2]    [G1]   [G1']      [G2']               |
          |           x=0.85  x=0.55  x=1.25    x=1.55  <- tunnel goals
          |            OURS    OURS    (opponent pair)                |
  y=3.40  +-----------------------------------------------------------+ LOW edge
              each goal: 200mm wide box, figure seated ON TOP
              figure = the goal's colour label (random per match):
              [grey/wood = human -> ORANGE]  [silver = robot -> SILVER]

  (off-table, positions unmeasured: Zone W / Zone P platforms +0.20m,
   ball platforms with 55 balls, Start B — all teleop territory)
```

---

## C. Navigation-relevant rule constraints

- The slope is **legal driving surface for the autonomous robot** (the touch-ban is
  teleop-only). Nothing restricts us to our half; only driving **off the field** is
  forbidden.
- **Never touch a figure.** This is the hard clearance constraint around the goals.
- The floor tape grid has **zero rule meaning** — crossing anything costs nothing.
  Lines exist only as our navigation signal.
- Goal colour identity is carried **only by the figure on top**, randomized per
  match — it must be sensed every match, before the sort flap commits.
- Figure poses are chosen adversarially (the opponent poses one of the figures on
  our goals) within: pelvis on goal top, no ball obstruction. Expect
  weird-but-seated poses.
- Balls not fully in a goal at T=3:00 score nothing; a multi-ball, all-correct goal
  **doubles** — mis-sorts are doubly expensive.
- Nothing that leaves the table comes back; no human touch mid-round — there is
  **no retry**. A navigation dead-end is terminal.
- Tie-breaker: finish the match **near the centre of our own Start A**.
- Robot must fit a 50 cm cube **only at start**; arms may deploy beyond it
  afterwards.

## D. Varies between matches — must be sensed or configured

1. **Side draw (red/blue half)** — announced >= 5 min before; may be handled by
   pre-match config, but sensing the start-tape colour is the safe guard. Mirrors
   the whole map (which lane pair is "ours", goal order).
2. **Figure -> goal assignment** — random every match, mirrored for the opponent.
   **Must be sensed. This is the only per-match unknown that cannot be
   pre-configured.** Decides orange-left/right vs silver-left/right.
3. **Figure poses** — freely chosen per match (partly by the opponent). Detection
   must be pose-robust, seated-pose family.
4. **Number of preloaded balls** — 1+1 plus cyber-game advantage balls; varies with
   our cyber result. Affects feed/sort cycle count.
5. **Opponent robot behaviour** — blocking without touching is legal; dynamic
   obstacle.
6. Everything else — geometry, slope, goal positions, lane layout, ball count on
   platforms, 3-minute clock — is fixed and may be hard-coded (after the
   TODO-FIELD measurements land).

## E. Open points (rulebook is silent — we measure or decide)

- Field width **2.39 vs 2.70 m** — one cross-field tape measure (top TODO).
- Exact Start A pad size/position, our placement and heading inside it (heading is
  free — our choice).
- Goal x-positions and centre-to-low-edge distance (0.30/0.70 pair spacing and
  0.12 m are unverified).
- Tape lane endpoints up-slope, cross-line endpoints, exact tape width.
- Ball diameter (assumed 60 mm) and the orange/silver split within the 55.
- Ball platform locations; how/where the teleop robot hands balls to ours (our
  design decision).
- Zone W / Zone P / Start B / "Zone S" exact geometry — Zone S's extent is
  ambiguous even in Fig. 1 (label points at the slope's upper area); irrelevant to
  us unless the referees rule otherwise, since its restrictions bind the teleop
  robot.
- Wall/border presence at the field edges.
- Venue lighting spec; active illumination legality (ask referees — electric parts
  need authorization).
- No explicit rule on robots **colliding with goals** — assume the table-damage
  rule applies; clarify allowed contact force with referees.
