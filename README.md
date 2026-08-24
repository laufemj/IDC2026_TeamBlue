# IDC2026 Team Blue — Autonomous Robot

Robocon International Design Contest 2026 (*AI to Science*), Institute of
Science Tokyo. This repo holds everything for our **autonomous robot**: the
vision stack, the navigation/mission stack, the synthetic-data pipeline that
trained the dummy detector, and the project documentation.

> New teammate? Read this page, then `docs/field_spec.md` (what the field
> actually is), then the README inside whichever directory you're touching.

---

## 1. The game, in one paragraph

3-minute match on a shared 5°-sloped table. Our autonomous robot starts at
**Start A** (top of the slope) preloaded with orange + silver balls, drives
down between the two black tape lanes to its team's two tunnel **goals**, and
sorts balls with a servo flap: **orange → the goal with the HUMAN figure,
silver → the goal with the ROBOT figure** (1 pt each; a goal holding only
correct-colour balls **doubles**). The two figures are near-identical wooden
mannequins — grey-clothed (human) vs silver-painted (robot) — and **which
figure sits on which goal is randomized every match**, so telling them apart
on camera is the core perception problem. Robots must never touch figures.
Full extraction with provenance tags: **`docs/field_spec.md`**.

## 2. System architecture

```
                     Raspberry Pi 5 (ROS 2)
   ┌─────────────────────────────────────────────────────────┐
   │  PERCEPTION (ball_color/)        NAVIGATION (nav/)      │
   │                                                         │
IMX500 ─ dummy detection (on-sensor) ─► /dummy_detections    │
   │  │                                  └► goal_sides_node ─► /goal_sides
   │  └─ preview ─► lane_node ─► /line_offset ──► mission_node
   │                └► /lanes_visible ─► arrival_node ─► /arrived
   │     team_side (start tape) ─► /team_side ──────┤        │
Cam Mod 3 ─ ball_color_node (chute) ─► /ball_color ─┤        │
   │                                                ▼        │
   │                       mission_node ─► /cmd_vel ─► drive_node ─► MDD3A
   │                                    └► /servo/* ─► servo_node ─► s1 s2 s3
   └─────────────────────────────────────────────────────────┘
```

Design rule used everywhere: **pure-logic module (testable, no ROS/hardware)
+ thin ROS wrapper node**. All field-dependent numbers live in ONE file,
`ball_color/field_spec.py`, provenance-tagged, with `TODO-FIELD` marking the
few numbers still awaiting a tape measure.

## 3. Repository map

| Path | What it is | Docs |
|---|---|---|
| `ball_color/` | Perception: ball colour (chute cam), dual-lane centreline, team-side tape, arrival fusion, **`field_spec.py`** | `ball_color/README.md` |
| `nav/` | Action: PD steering, match state machine, L/R goal verdict, motor/servo drivers, launcher | `nav/README.md` |
| `models/` | Trained dummy-detector weights (`.pt` + `.onnx`) with full metadata | `models/README.md` |
| `dummy_model/` | Synthetic-data pipeline: mannequin URDF generator, MuJoCo pose settling, photo-calibrated Blender scene, dataset renderer, training/eval scripts | file docstrings |
| `docs/` | `field_spec.md` (field truth), `lr_field_report.html` (detector eval on real photos — open in a browser) | — |
| `cad/` | build123d generators for the robot CAD (STEP/GLB regenerate from source) | — |
| `Contest_Rules_final2026.pdf` | Official rulebook Ver 1.01 — the only normative source | — |

## 4. Key results so far

**Dummy detector** (`models/dummy_yolo11n_v25f2.pt`, YOLO11n): trained on
**synthetic Blender renders only** (photo-calibrated materials/lighting,
MuJoCo-settled seated poses, occlusion-checked labels; zero real photos).
On the 127 real venue photos:

- close range (box ≥ ~24 % of frame height): human/robot **100 % correct**
- silver-robot calls correct at **every** range (34/34); the only failure
  mode is far-range human→robot flips
- operating point (already in `field_spec.py`): conf 0.25,
  **agnostic NMS**, 3 % edge filter, L/R latch only at close range

Full analysis with failure causes: `docs/lr_field_report.html`.

**Nav layer**: complete state machine with timeout fallbacks at every wait,
jam watchdog, 0.5 s motor deadman, and mis-sort protection (unclassifiable
balls go to the side that protects the score double). `python3
nav/test_logic.py` — 25 checks, no dependencies.

## 5. Quick start

```bash
# ball-colour classifier on the bundled venue crops (needs numpy+pillow only)
python3 ball_color/ball_color_node.py --images 'ball_color/samples/*.png'

# nav logic tests (no dependencies at all)
python3 nav/test_logic.py

# dummy detector on an image (needs ultralytics)
python3 - <<'EOF'
from ultralytics import YOLO
r = YOLO("models/dummy_yolo11n_v25f2.pt").predict(
    "your.jpg", imgsz=640, conf=0.25, agnostic_nms=True)
EOF

# full stack on the Pi (after calibration, below)
nav/launch_all.sh <n_balls> <red|blue> <HR|RH-fallback>
```

## 6. Status board

| # | Work item | State |
|---|---|---|
| 1 | Synthetic data pipeline (URDF → poses → calibrated renders → labels) | **done** |
| 2 | Dummy detector training + real-photo validation | **done** (v25f2) |
| 3 | Perception nodes (ball colour / lanes / team side / arrival) | **done**, venue calibration pending |
| 4 | Nav layer (drive / mission / goal-sides / servos) | **done**, logic-tested; hardware untested |
| 5 | Field spec + L/R field report docs | **done** |
| 6 | ONNX fallback export | **done** (note: PyTorch was *faster* on x86 — benchmark on the Pi before choosing) |
| 7 | IMX500 on-sensor conversion (INT8 quantization + packaging) | **in progress** — toolchain needs Python ≤3.12 (`~/.venvs/imx`); first quantization pass ran, run died mid-way, retrying |
| 8 | IMX500 detector ROS node publishing `/dummy_detections` | todo (interface contract in `nav/README.md` §5) |
| 9 | Robot assembly, wiring, camera mount (15° down) | hardware side, off-repo |

## 7. At-the-venue checklist (in order)

1. Tape-measure the `TODO-FIELD` numbers → edit `ball_color/field_spec.py`
   (field width 2.39 vs 2.70 first; then goal x-positions, Start A, camera
   height). Every consumer updates automatically.
2. Pins/angles: `nav/drive_node.py` motor pins, `nav/servo_node.py` pins +
   angles (placeholders!), duty scale + `trim` by driving a straight line —
   start slow, the slope helps you downhill.
3. Ball colour: `ball_color/capture_reference.py` 5-minute procedure with
   the real balls in the real pocket. If silver reads "none" → pocket too
   dark → revisit the LED question.
4. Lane geometry: one preview frame of real lanes through the real mount
   confirms `PAIR_SPACING` (derived from camera height/tilt).
5. Dummy detector sanity: run the model on a handful of on-robot frames of
   the placed figures (`models/README.md` snippet).
6. Match config per round: `n_balls`, `cfg_side` (announced ≥5 min ahead),
   `cfg_goal_sides` fallback — parameters of `nav/launch_all.sh`.

## 8. Known gaps / risks

- **IMX500 conversion unfinished** (item 7). Fallback: run the `.pt` on the
  Pi CPU (~ok for 640 px if throttled) or `.onnx` — benchmark both on-Pi.
- Far-range human→robot flips are inherent to the current synthetic human
  (no garment mesh). Mitigated by the close-range latch; real fix is a v2
  clothed mesh + far-scale augmentation if retraining ever happens.
- No encoders: all forward motion is vision + timing. The tie-break
  (end nearest own Start A) is deliberately not attempted.
- `nav/servo_node.py` uses software PWM; if the flap jitters on the Pi 5,
  move servos to the kit's ADA-815 PWM board (topic interface unchanged).
