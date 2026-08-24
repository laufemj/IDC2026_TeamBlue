"""Single source of truth for every field-dependent number in the robot code.

Provenance tags on every value:
  [RB pN]  official Contest Rule Book Ver 1.01 (05 Aug 2026), page N --
           the only normative source.  Full extraction: docs/field_spec.md.
  [CAD]    organisers' arena CAD (IDC2026_mujoco/assets/parts/Gametablepart*.stl)
  [TAPE]   tape-measured on the real artifact (goal box, dummy)
  [PHOTO]  the 127 venue frames (image_20260812_173204_0001, 2026-08-12)
  [TEAM]   our own decision or unverified estimate

TODO-FIELD marks numbers that one tape measure at the venue settles.  Update
the number here and every consumer follows.

Frame convention (matches docs/field_spec.md section B): origin at the centre
of OUR Start A, y forward along the robot's initial facing (straight
down-slope), x to the robot's initial right, z up from the local surface.
"""

import math

# --- arena ------------------------------------------------------------- [CAD]
SLOPE_DEG = 5.0            # uniform ramp, [CAD] exact
FIELD_LENGTH = 3.586       # plan-view slope run, m [CAD]
FIELD_WIDTH = 2.39         # m; TODO-FIELD ([CAD] says 2.70, mat arithmetic
                           # 5 x 478 mm tiles says 2.39 -- measure once)
TILE_PITCH = 0.478         # lime EVA mat tiles, m [PHOTO]

# --- floor tape ------------------------------------------------------ [PHOTO]
# The rulebook NEVER mentions floor lines: they carry no rule meaning and no
# crossing penalty.  Navigation aids only.  [RB, by omission]
TAPE_WIDTH = 0.050         # m; TODO-FIELD
LANE_SPACING = 0.30        # m centre-to-centre within a goal pair; TODO-FIELD
PAIR_CENTRE_X = 0.35       # m, pair centre offset from the half centreline;
                           # TODO-FIELD

# --- goals ------------------------------------------------------------ [TAPE]
GOAL_W = 0.200             # box outer width, m
GOAL_D = 0.195             # box outer depth, m
GOAL_WALL = 0.012          # wall thickness -> bore ~0.176 m wide
GOAL_TOP_Z = 0.201         # mean top height; top is LEVEL on the slope
GOAL_EDGE_Y = 0.12         # tunnel centre from the low edge, m; TODO-FIELD

# --- start ------------------------------------------------------- [RB p5][TEAM]
# Autonomous robot must start from Start A [RB p5]; placement inside the pad
# and heading are NOT specified -- heading is our choice (we face down-slope).
# Start A centre in field coords (x from half centreline, y from low edge):
START_A_FIELD = (1.05, 3.40)   # m; TODO-FIELD
DIST_START_TO_GOALS = START_A_FIELD[1] - GOAL_EDGE_Y   # ~3.28 m of driving

# --- figures (the ONLY per-match unknown that must be sensed) ---- [RB p5][TAPE]
FIGURE_HEIGHT = 0.300      # standing, m [TAPE]
# human figure = grey cloth + bare wood -> its goal takes ORANGE balls
# robot figure = silver paint          -> its goal takes SILVER balls
# Assignment to Goal A/B is randomized EVERY MATCH, mirrored for the
# opponent [RB p5].  Poses adversarial but seated: pelvis on the goal top,
# must not obstruct ball passage [RB p5].  Robots must NOT touch figures
# [RB p6] -- hard clearance constraint at the goal approach.

# --- balls --------------------------------------------------------- [RB][TEAM]
BALL_DIAMETER = 0.060      # m; NOT in the rulebook -- confirm on the kit ball
PRELOAD_BASE = (1, 1)      # (orange, silver) the autonomous robot may install,
                           # PLUS cyber-game advantage balls [RB p5-6]

# --- match --------------------------------------------------------- [RB p6]
MATCH_DURATION_S = 180
# Scoring: 1 pt per correct-colour ball in a goal; a goal holding >1 ball,
# all correct colour, DOUBLES.  Balls outside goals at T=180 s score zero.
# Tie-break chain: cyber points, then whichever autonomous robot ends nearer
# the centre of its own Start A -- finishing near Start A has value.

# --- robot as-built (recovered Pi config + CAD session, see
# docs/pi_session_recovery.md) ------------------------------------------ [TEAM]
WHEEL_DIAMETER = 0.173     # kit wheel, measured (Pi config said 0.065 -- wrong)
WHEEL_TRACK = 0.242        # as-built contact-patch centres, m
FULL_DUTY_SPEED = 0.55     # m/s at duty 1.0, Pi config assumption; recalibrate

# --- camera (IMX500, front) -------------------------------------------- [TEAM]
CAM_HEIGHT = 0.30          # lens above local floor; Pi config CAM1_HEIGHT_M,
                           # matches the CAD mast (z=300).  TODO-FIELD verify
CAM_TILT_DEG = 20.0        # Pi config CAM1_PITCH_DEG, matches CAD mount
CAM_VFOV_DEG = 52.3        # 4:3 capture
CAM_HFOV_DEG = 2 * math.degrees(math.atan(
    math.tan(math.radians(CAM_VFOV_DEG / 2)) * 4 / 3))   # ~66.3 deg


def ground_distance(img_y_frac):
    """Ground distance (m) to the floor point imaged at img_y_frac of the
    frame height (0 = top).  Returns math.inf above the horizon."""
    off = (img_y_frac - 0.5) * 2.0
    below = CAM_TILT_DEG + math.degrees(
        math.atan(off * math.tan(math.radians(CAM_VFOV_DEG / 2))))
    if below <= 0:
        return math.inf
    return CAM_HEIGHT / math.tan(math.radians(below))


def lane_pair_image_fraction(img_y_frac):
    """LANE_SPACING expressed as a fraction of image width at a given image
    row.  This is what lane_tracker.PAIR_SPACING approximates mid-band."""
    d = ground_distance(img_y_frac)
    if not math.isfinite(d):
        return 0.0
    slant = math.hypot(d, CAM_HEIGHT)
    ground_width = 2 * slant * math.tan(math.radians(CAM_HFOV_DEG / 2))
    return LANE_SPACING / ground_width


# --- detector operating point (from the real-photo L/R study, docs/) ---
# v25f2 on 127 venue frames: silver-robot dets 34/34 correct at all ranges;
# grey-human flips to "robot" only when the box is small; both classes 100%
# correct once the box is >= ~24% of frame height.  Hence:
DET_CONF = 0.25            # 0.5 halves recall for no FP benefit
DET_AGNOSTIC_NMS = True    # per-class NMS puts H+R boxes on one dummy
DET_EDGE_MARGIN = 0.03     # drop boxes hugging the frame edge (slivers)
DET_LATCH_MIN_H = 0.25     # only latch the L/R verdict at close range
