# Lost Pi session — what was recovered, what is gone

> Archive clarification (2026-11-20): the supplied team report states that the
> competition Pi was returned to the organizers. “Lost” below refers to access
> to its deployed source/configuration. This historical recovery log retains
> earlier assumptions and later recovery waves; read the complete note and the
> [current provenance summary](evidence.md) before treating a value as final.

The competition Raspberry Pi (which ran its own Claude session and held "the
robot repository": `config.py` + `tools/`) is permanently lost. This note
records everything salvageable, found on 2026-08-24 by searching this
machine's transcripts: the user had pasted the Pi session's full **Ball-Sorter
CAD Brief** into the CAD session on 2026-08-17. Verbatim copy:
`docs/pi_session_ball_sorter_brief.txt`.

## Recovered contract values (Pi `config.py`, 2026-08-17)

| key | value | status now |
|---|---|---|
| `WHEEL_DIAMETER_M` | 65 mm | **superseded** — real kit wheel measured Ø173 × 14.5 mm (CAD session) |
| `WHEEL_TRACK_M` | 180 mm | **superseded** — as-built track 242 mm (CAD session) |
| `CAM1_HEIGHT_M` | 300 mm | **adopted** — matches the CAD mast; `field_spec.CAM_HEIGHT` updated 0.25→0.30 |
| `CAM1_PITCH_DEG` | 20° | **adopted** — `field_spec.CAM_TILT_DEG` updated 15→20 |
| `SERVO1_*_DEG` | 10 STOW / 90 ARMS / 140 DISPENSE / 170 VERTICAL(gate closed) | **adopted** as `nav/servo_node.py` defaults (single servo drives arms AND gate — lost-motion design) |
| `SERVO2_*_DEG` (flap) | 35 LEFT / 90 CENTER / 145 RIGHT | **adopted** as flap defaults |
| `BALL_TRAVEL_TIME_S` | 0.8 s | adopted — mission jam-watchdog floor |
| gate open time | 0.30 s | adopted — `servo_node` FEED_CYCLE_S |
| `SORT_MAX_BALLS` | ≥12 | informational |
| full-duty speed | ~0.55 m/s, soft-start 0.15 s, brake-by-shorting | informational for drive calibration |
| Camera-2 ROI | x 0.30–0.70, y 0.55–1.00 @640×480 | matches `ball_color/config.json` philosophy; verify at build |
| driver | MDD3A, sign-magnitude, labels M1A/M1B/M2A/M2B, onboard test buttons | pin NUMBERS not in the brief — still lost |

## SECOND RECOVERY WAVE (2026-08-25): the Pi session's Artifacts

The Pi session published Artifacts to the account — readable even with the
machine gone. Recovered and transcribed into this repo:

| artifact | date | recovered to |
|---|---|---|
| FIELD_DAY.md (field-day playbook) | 08-19 | `docs/pi_field_day_playbook.md` |
| IDC2026 Sorter Backlog (post-field-day full state) | 08-21 | `docs/pi_sorter_backlog.md` |
| Line Detection Probe | 08-20 | key values below |
| Ball-Sorter CAD Brief | 08-17 | already had via paste |
| Ball Photo Triage / Debug Frame Gallery / Ball Divider Frames | 08-19/20 | image evidence, not transcribed |

**Recovered after all — the GPIO pins:** MDD3A signal = physical pins
**32 (M1A), 11 (M1B), 33 (M2A), 16 (M2B), 6 (GND)** = BCM **12, 17, 13, 23**.
Applied as `nav/drive_node.py` defaults (left/right channel assignment and
polarity still need a teleop check).

**Line Detection Probe findings (real field):** the black tape is GLOSSY —
a darkness mask of V≤70 missed it; the Pi raised LINE_MAX_VALUE 70→200 and
LINE_MAX_SAT 90→140. Effective camera pitch measured 16.4° (vs 20° design).
Lane-pair pixel spacing cap 450 px at their band 0.96–1.43 m.

## Still lost (never left the Pi)

- The code itself (`~/idc/robocon_bot`: main.py state machine, tools/*.py,
  sim). Roles and every tuned value are now documented in the two recovered
  docs; the implementation would have to be rewritten.
- The 45 real-run debug-frame folders and logs referenced by the backlog.

## Important context on the brief

The brief describes an **earlier mission concept** (painted 30 mm grid,
collection row, humans dump balls into rear arms, reverse to a firing row,
shoot 1.93 m at goal posts ±600 mm). The real venue (photos 2026-08-12,
`docs/field_spec.md`) replaced this: black 50 mm tape lanes, drive down
between the goal pair, sort at arrival into tunnel goals. **Field numbers in
the brief's §9 are obsolete — `ball_color/field_spec.py` is the field
truth.** The robot-side hardware contract (servo states, camera mount,
electronics-bay practices in §8/§10) remains the build reference and is why
this recovery matters.
