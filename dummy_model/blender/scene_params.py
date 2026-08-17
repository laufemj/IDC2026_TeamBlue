"""Field / camera / material parameters for the Blender training scene.

Every number here traces to one of three sources, in order of trust:
  1. tape measurements on the physical artifacts (goal box, dummy),
  2. the organisers' arena CAD (IDC2026_mujoco/assets/parts/Gametablepart*.stl,
     cross-checked: slope exactly 5.0 deg),
  3. the venue photo set image_20260812_173204_0001/ (127 IMX500 frames),
     from which venue_swatches.json was sampled.

Anything still unverified is marked TODO-FIELD: confirm at the next field
session and update the one number.
"""

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.dirname(HERE)
REPO = os.path.dirname(MODEL_DIR)

# --- arena ----------------------------------------------------------------
STL_DIR = os.path.join(REPO, "IDC2026_mujoco", "assets", "parts")
FIELD_STL = os.path.join(STL_DIR, "Gametablepart1.stl")   # sloped main table
SIDE_STL = os.path.join(STL_DIR, "Gametablepart2.stl")    # side platform

# Field surface.  Length and rise from the STL (y in [0, 3.586], z rising
# 0.314 -> 5.0 deg).  Width: the STL says 2.70, but the user -- standing at
# the real field -- reads it as narrower, and the mat arithmetic agrees:
# five 478 mm tile columns = 2.39 m.  The imported STL is x-scaled to match.
# TODO-FIELD: one tape measure across the field settles it.
FIELD_W = 2.39
FIELD_W_STL = 2.70
FIELD_L = 3.586
FIELD_RISE = 0.314
SLOPE = math.atan2(FIELD_RISE, FIELD_L)

# Goal tunnel, tape-measured on the real object.  The box is built so its
# top is LEVEL when standing on the slope (191 mm wall on the uphill side,
# 212 mm downhill).
GOAL_W, GOAL_D = 0.200, 0.195
GOAL_WALL = 0.012
GOAL_H_MEAN = 0.201

# Gate x-centres from the organisers' CAD, re-expressed about the field's
# own centreline (the CAD origin is offset by -0.11; the pairs are symmetric
# about it, +-0.35 to each pair centre, 0.30 within a pair).
# TODO-FIELD: confirm with a tape measure.
GOAL_X = (-0.50, -0.20, 0.20, 0.50)
GOAL_EDGE_Y = 0.12          # tunnel centre distance from the low edge
                            # TODO-FIELD: measure

# Start A: the autonomous robot's start corner, top of the slope.
START_A = (1.05, 3.40)      # (x, y) on the field; TODO-FIELD: measure
CAM_HEIGHT = 0.25           # lens height above the LOCAL field surface
                            # TODO-FIELD: actual mount height on the robot

# --- floor tape -----------------------------------------------------------
# Venue mats: lime EVA tiles with slight two-tone variation, black tape in
# long lanes.  Grid pitch and width are the user's measurements.
TAPE_PITCH = 0.478
TAPE_W = 0.050

# --- camera (IMX500) ------------------------------------------------------
# 1/2.3" sensor, 4056 x 3040, 1.55 um pixels.
SENSOR_W_MM = 6.287
SENSOR_H_MM = 4.712
HFOV_DEG = 66.3
# Deployment aspect is UNDECIDED (venue frames were 16:9 1080p; the plan
# assumed 4:3 full sensor letterboxed to 640x640).  Parameterised so the
# whole pipeline flips with one string.
CAPTURE_MODE = "4:3"        # "4:3" (2028x1520 binned) or "16:9" (1080p)
RENDER_W = 1280
RENDER_H = {"4:3": 960, "16:9": 720}[CAPTURE_MODE]

# --- materials ------------------------------------------------------------
SWATCHES = json.load(open(os.path.join(MODEL_DIR, "venue_swatches.json")))

def srgb_to_linear(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

def swatch_linear(name):
    r, g, b = SWATCHES[name]
    return tuple(srgb_to_linear(v) for v in (r, g, b)) + (1.0,)

# The swatches are photographed colour (venue light baked in), not albedo.
# They are used as CALIBRATION TARGETS: render the scene, sample the same
# patches, and adjust these base albedos until the render matches the photo.
# Starting guesses pull the green cast partially out by hand.
ALBEDO = {
    "field_bright": (0.55, 0.75, 0.08, 1.0),
    "field_dark":   (0.46, 0.64, 0.05, 1.0),
    "tape":         (0.033, 0.041, 0.038, 1.0),
    "goal_paint":   (0.85, 0.85, 0.82, 1.0),
    "goal_ply":     (0.52, 0.38, 0.20, 1.0),
    "wood_bare":    (0.62, 0.44, 0.24, 1.0),   # dummy head/hands/feet
    "cloth_grey":   (0.30, 0.31, 0.31, 1.0),   # human garment
    "silver":       (0.75, 0.76, 0.77, 1.0),   # robot paint (metallic)
    "carpet":       (0.42, 0.44, 0.42, 1.0),
}

# calibrate.py writes these; when present they override the guesses above so
# the training scene automatically uses the photo-matched values.
_cal = os.path.join(HERE, "calibrated_albedo.json")
if os.path.exists(_cal):
    for _k, _v in json.load(open(_cal)).items():
        ALBEDO[_k] = tuple(_v) + (1.0,)

_lit = os.path.join(HERE, "calibrated_light.json")
LIGHT = {"energy_scale": 1.0, "color": [1.0, 0.98, 0.92], "ambient": 0.25}
if os.path.exists(_lit):
    LIGHT.update(json.load(open(_lit)))
