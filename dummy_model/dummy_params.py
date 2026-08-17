"""Dimensions and shape profiles for the IDC2026 wooden artist mannequin.

Two numbers are ground truth, measured on the physical figure:
    STANDING_HEIGHT_MM = 300.0
    SHOULDER_SPAN_MM   =  54.0   (centre-to-centre, ball radius 8 mm)

Everything else is photogrammetry from KakaoTalk_20260814_103822986_03.jpg,
a near-orthographic back view.  Pixel measurements were taken on a crop of
that photo and converted with a single scale factor derived by forcing the
standing-height chain to sum to 300 mm:

    foot_thick + shin + thigh + pelvis + chest + neck + head = 300 mm

The scale is only a starting guess: gen_dummy.py measures the built geometry
and iterates a global factor until the mesh really is 300.00 mm tall, so the
chain's job is to fix proportions, not absolute size.

Structure re-checked against _01 (close-up, with a ruler in frame) after the
first build looked wrong:
  * The neck is a small BALL, not a tube.  The first version had a long
    tapered collar and the figure read as giraffe-necked.
  * Shoulders, hips, waist, wrists and ankles are all visible BALLS standing
    proud of the parts they join.  The first version had no ball geometry at
    all, so the joints looked like butt-joined sticks.
  * Hands and feet are small rounded egg knobs, NOT flat paddles.  The first
    version flattened them to 0.45 / 0.35 and grew big duck feet.

Cross-check: shoulder span measured 275 px on the same photo, which that scale
turns into 52.3 mm against the 54.0 mm tape measurement -- 3 % error.  The
proportions are therefore trustworthy to a few percent, which is well inside
what matters here (the figure is ~40-70 px tall in the deployed camera).

Height is parametric: change STANDING_HEIGHT_MM and every length rescales.
SHOULDER_SPAN_MM is applied directly rather than scaled, since it was measured.
"""

STANDING_HEIGHT_MM = 300.0
SHOULDER_SPAN_MM = 54.0
SHOULDER_BALL_R_MM = 8.0

# --- raw pixel measurements from _03 --------------------------------------
# Lengths along each link's own axis.
_PX = dict(
    head=250, neck=38, chest=281, pelvis=220,
    upper_arm=256, forearm=232, hand=104,
    thigh=354, shin=415, foot=122,
    # widths (max diameter perpendicular to the link axis)
    head_w=159, chest_w=226, pelvis_w=208,
    upper_arm_d=73, forearm_d=67, hand_w=73,
    thigh_d=92, shin_d=79, foot_w=85, foot_thick=72,
    # skeleton
    shoulder_span=275, hip_span=147, ankle_z=35,
)

# Height chain that defines the scale.  Standing, from the sole upward.
_CHAIN = ("foot_thick", "shin", "thigh", "pelvis", "chest", "neck", "head")
_PX_TOTAL = sum(_PX[k] for k in _CHAIN)          # 1578 px
PX_PER_MM = _PX_TOTAL / STANDING_HEIGHT_MM       # 5.26

# Where the shoulder sits along the chest, measured from the waist end.
SHOULDER_FRAC_OF_CHEST = 0.70
# Hip joints sit right at the bottom of the pelvis, balls slightly proud.
HIP_FRAC_OF_PELVIS = 0.04


def mm(key):
    """Pixel measurement -> millimetres."""
    return _PX[key] / PX_PER_MM


def m(key):
    """Pixel measurement -> metres (URDF native unit)."""
    return mm(key) / 1000.0


# --- derived dimensions, all metres ---------------------------------------
D = {k: m(k) for k in _PX}
D["shoulder_span"] = SHOULDER_SPAN_MM / 1000.0   # measured, not scaled
D["shoulder_ball_r"] = SHOULDER_BALL_R_MM / 1000.0

# Torso and head are wider than they are deep; ratios eyeballed from the
# side-on views (_00, _02).  Q1 established that exact shape barely matters.
DEPTH_RATIO = dict(head=0.95, chest=0.85, pelvis=0.90)
# Hands and feet are rounded knobs, barely flattened (see _01 close-up).
FLATTEN = dict(hand=0.88, foot=0.85)

# --- visible joint balls ---------------------------------------------------
# Every ball joint on this toy shows as a sphere standing proud of both parts
# it joins, and the elbows/knees/wrists/ankles show a smaller collar.  These
# are load-bearing for us twice over: they are what the detector sees at the
# joint, and they give the solver a smooth convex surface to slide on instead
# of the sharp rim where two lathe bodies butt together.
#
# Radii as a fraction of a reference width, read off the _01 close-up.  The
# shoulder is the exception -- it was tape-measured, so it is absolute.
BALL_FRAC = dict(
    neck=0.19,      # x head_w
    waist=0.12,     # x pelvis_w   -- reads as a wasp waist, not a proud ball
    hip=0.16,       # x pelvis_w
    elbow=0.39,     # x upper_arm_d
    wrist=0.26,     # x forearm_d
    knee=0.32,      # x thigh_d
    ankle=0.22,     # x shin_d
)


# --- lathe profiles -------------------------------------------------------
# Each profile is [(t, r), ...] with t in [0,1] along the link's +Z axis and
# r in [0,1] as a fraction of the link's max radius.  Revolved about +Z.

PROFILES = {
    # Egg.  t=0 at the neck peg, t=1 at the crown.
    "head": [
        (0.00, 0.30), (0.05, 0.42), (0.15, 0.62), (0.30, 0.83),
        (0.45, 0.96), (0.60, 1.00), (0.75, 0.96), (0.88, 0.80),
        (0.95, 0.55), (1.00, 0.00),
    ],
    # Big rounded torso.  t=0 at the waist ball, t=1 at the neck ball.
    #
    # The shoulder end deliberately stays BROAD and then rounds off fast.  The
    # first version tapered gradually to 0.34, and that slow taper plus the old
    # tube neck read as one long thin neck in the renders -- which is what the
    # "neck is too long" complaint was actually pointing at.  On the real
    # figure the chest is still ~0.7 of its max width where the shoulder balls
    # sit, then turns over sharply into the neck ball.
    "chest": [
        (0.00, 0.52), (0.08, 0.72), (0.20, 0.90), (0.35, 1.00),
        (0.55, 1.00), (0.72, 0.95), (0.85, 0.83), (0.94, 0.66),
        (1.00, 0.50),
    ],
    # Smaller, more angular.  t=0 at the hips, t=1 at the waist.
    "pelvis": [
        (0.00, 0.85), (0.15, 0.97), (0.35, 1.00), (0.55, 0.98),
        (0.75, 0.88), (0.90, 0.70), (1.00, 0.55),
    ],
    # Spindle: joint collar, swell, taper to the far joint.  Used for all
    # four limb segments.  t=0 proximal, t=1 distal.
    "spindle": [
        (0.00, 0.40), (0.04, 0.48), (0.10, 0.62), (0.22, 0.82),
        (0.40, 0.97), (0.60, 1.00), (0.78, 0.88), (0.90, 0.68),
        (0.96, 0.52), (1.00, 0.42),
    ],
    # Rounded paddle, flattened after revolving.
    "hand": [
        (0.00, 0.45), (0.15, 0.75), (0.35, 0.95), (0.55, 1.00),
        (0.75, 0.92), (0.90, 0.65), (1.00, 0.00),
    ],
    # Blade that widens toward the toe, flattened after revolving.
    "foot": [
        (0.00, 0.40), (0.20, 0.68), (0.45, 0.88), (0.70, 1.00),
        (0.88, 0.90), (1.00, 0.55),
    ],
}


# --- joint limits, radians ------------------------------------------------
# Real mechanical limits of the toy (Q5c).  Ball joints are decomposed into
# three revolute joints ordered Z (twist) -> X -> Y, with the middle axis kept
# well inside +-90 deg so the gimbal singularity is unreachable.
#
# ESTIMATED from the photographs.  Refine against the physical figure.
import math as _math
_d = _math.radians

LIMITS = {
    "waist_z":  (_d(-45), _d(45)),
    "waist_x":  (_d(-35), _d(35)),
    "waist_y":  (_d(-35), _d(35)),

    "neck_z":   (_d(-60), _d(60)),
    "neck_x":   (_d(-40), _d(40)),
    "neck_y":   (_d(-40), _d(40)),

    "shoulder_z": (_d(-90), _d(90)),
    "shoulder_x": (_d(-85), _d(85)),
    "shoulder_y": (_d(-95), _d(95)),
    "elbow":      (_d(0),   _d(150)),
    "wrist":      (_d(-90), _d(90)),

    "hip_z": (_d(-60), _d(60)),
    "hip_x": (_d(-80), _d(80)),
    "hip_y": (_d(-90), _d(90)),
    "knee":  (_d(-150), _d(0)),
    "ankle": (_d(-45), _d(45)),
}

# Beech / basswood, painted.  Used for inertials only.
#
# ASSUMED, not measured -- the physical figure was not weighed.  Combined with
# the lathe volumes this predicts a total mass of ~119 g, which is plausible
# for a 300 mm wooden mannequin (commercial ones run 150-250 g, though those
# are usually solid hardwood rather than this lighter painted stock).
#
# What it affects: only the settling dynamics.  Every gravity torque and every
# calibrated frictionloss scales linearly with density, so if the real figure
# turns out to weigh 200 g, multiply DEFAULT_FRICTIONLOSS in make_mjcf.py by
# 200/119 and nothing else changes.  Settled *poses* are near-invariant,
# because both the load and the stiction that resists it scale together.
WOOD_DENSITY = 620.0  # kg/m^3
