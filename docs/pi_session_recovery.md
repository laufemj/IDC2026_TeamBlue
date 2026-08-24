# Lost Pi session — what was recovered, what is gone

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

## Still lost (never left the Pi)

- `config.py` itself — including **GPIO pin numbers** for MDD3A and servos.
  `nav/drive_node.py` / `servo_node.py` pin params remain placeholders.
- `tools/test_hardware.py`, `calib_drive.py`, `check_geometry.py`,
  `calib_goal_ratio.py`, `calib_hsv.py` implementations. Their *roles* are
  reconstructed in this repo (capture_reference.py, field_spec derivations,
  nav calibration checklist); the code is rewritten, not recovered.
- Any bench-measured values from the motor test the Pi session ran
  ("the test earlier for the motors had worked").
- The only other possible copy is the Remote Control transcript viewable
  from the account at claude.ai (session titled "the test earlier for the
  motors…"). If it opens, pin numbers may be quoted there — paste them into
  this file and update the two nodes.

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
