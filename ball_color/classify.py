"""Orange-vs-silver ball classification for the intake pocket.  Pure numpy.

No OpenCV, no model: at the decision pocket the ball fills a KNOWN region of
the frame at a few centimetres, so this is a colour-statistics problem, not a
vision problem.  Saturation does almost all of the work:

    orange ball   S >= ~0.45 under venue light (measured on the real balls
                  in the venue frames: S p5=0.47, median 0.59)
    silver ball   near-achromatic, S <= ~0.2, with specular highlights
    empty pocket  whatever the pocket interior is -- captured once as a
                  reference frame and subtracted

Hue is only a guard band, deliberately wide: the venue's lime-green bounce
shifted the balls' hue from true orange (~20 deg) to 31-40 deg -- twenty
degrees of illuminant drift, measured, in one room.  Anything inside 0-60 deg
with high saturation is the orange ball; nothing else on this robot is both
saturated and warm.

All tunables live in config.json next to this file.
"""

import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(HERE, "config.json")


def load_config(path=DEFAULT_CONFIG):
    with open(path) as f:
        return json.load(f)


def rgb_to_hsv(rgb):
    """rgb float [0,1], shape (...,3) -> h (deg), s, v arrays."""
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    mx = rgb.max(axis=-1)
    mn = rgb.min(axis=-1)
    c = mx - mn
    s = np.where(mx > 0, c / np.maximum(mx, 1e-9), 0.0)
    h = np.zeros_like(mx)
    safe = np.maximum(c, 1e-9)
    m = mx == r
    h[m] = (60.0 * ((g - b) / safe))[m] % 360.0
    m = (mx == g) & (c > 0)
    h[m] = (60.0 * (2.0 + (b - r) / safe))[m]
    m = (mx == b) & (c > 0)
    h[m] = (60.0 * (4.0 + (r - g) / safe))[m]
    return h, s, mx


def classify_roi(roi_rgb, cfg, empty_ref=None):
    """One decision from one ROI.

    roi_rgb    uint8 (H, W, 3) crop of the decision pocket
    empty_ref  same-shape uint8 reference of the EMPTY pocket, or None
    returns    (label, info)  label in {"orange", "silver", "none"}
    """
    rgb = roi_rgb.astype(np.float32) / 255.0
    h, s, v = rgb_to_hsv(rgb)

    # --- presence: has anything changed vs the empty pocket? ---------------
    if empty_ref is not None:
        ref = empty_ref.astype(np.float32) / 255.0
        diff = np.abs(rgb - ref).max(axis=-1)
        changed = diff > cfg["presence_diff"]
        presence = float(changed.mean())
        if presence < cfg["presence_min_frac"]:
            return "none", {"presence": presence}
    else:
        changed = np.ones(h.shape, bool)
        presence = 1.0

    # --- colour vote over the changed pixels -------------------------------
    hue_ok = (h < cfg["orange_hue_max"]) | (h > 360.0 - cfg["orange_hue_wrap"])
    orange_px = changed & hue_ok & (s > cfg["orange_sat_min"]) \
        & (v > cfg["orange_val_min"])
    silver_px = changed & (s < cfg["silver_sat_max"]) \
        & (v > cfg["silver_val_min"])

    n = max(int(changed.sum()), 1)
    fo = float(orange_px.sum()) / n
    fs = float(silver_px.sum()) / n
    info = {"presence": presence, "orange_frac": round(fo, 3),
            "silver_frac": round(fs, 3)}

    if fo >= cfg["orange_min_frac"] and fo > fs * cfg["margin"]:
        return "orange", info
    if fs >= cfg["silver_min_frac"] and fs > fo * cfg["margin"]:
        return "silver", info
    return "none", info


class Debouncer:
    """Median-of-N vote with a stability requirement.

    Publishes a decision only after the same label wins `stable` consecutive
    windows -- a ball rolling through the pocket gives a run of consistent
    frames; motion blur and half-entered balls give churn, which this eats.
    """

    def __init__(self, window=5, stable=3):
        self.window = window
        self.stable = stable
        self.buf = []
        self.last_emit = None
        self.run = 0
        self.candidate = None

    def update(self, label):
        """Feed one frame's label; returns a label to EMIT or None."""
        self.buf.append(label)
        if len(self.buf) > self.window:
            self.buf.pop(0)
        vote = max(set(self.buf), key=self.buf.count)
        if vote == self.candidate:
            self.run += 1
        else:
            self.candidate, self.run = vote, 1
        if (self.run >= self.stable and vote != self.last_emit
                and vote != "none"):
            self.last_emit = vote
            return vote
        if vote == "none":
            self.last_emit = None      # pocket cleared; re-arm
        return None
