"""Left/right goal identity from dummy detections.  Pure python, no ROS.

Decides ONE latched verdict per match: is the layout "HR" (human figure on
the left goal, robot on the right) or "RH"?  Everything downstream (the sort
flap) keys off this single string.

Policy, straight from the 127-real-photo study (docs/lr_field_report.html):
  * cross-class dedupe first -- per-class NMS puts an H and an R box on the
    same dummy (27 phantoms in the study); keep the higher-confidence box.
  * drop frame-edge slivers and implausible sizes.
  * a frame VOTES only if it shows exactly one H and one R, BOTH at close
    range (box height >= DET_LATCH_MIN_H): below that the grey-clothed human
    flips to "robot" (100% accuracy above, 75-92% below).  Frames reading
    HH/RR/partial simply don't vote -- with the H->R flip being the only
    failure mode, demanding an {H, R} pair is the strongest filter we have.
  * latch after MIN_VOTES agreeing votes with at least AGREE_FRAC agreement
    in the recent window.
"""

from collections import deque
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "ball_color"))
import field_spec as F

MIN_VOTES = 5           # matching votes needed to latch
WINDOW = 9              # recent frames considered
AGREE_FRAC = 0.8        # required share of the winning verdict in-window
IOU_DEDUPE = 0.4


def _iou(a, b):
    ix = max(0.0, min(a["cx"] + a["w"] / 2, b["cx"] + b["w"] / 2)
             - max(a["cx"] - a["w"] / 2, b["cx"] - b["w"] / 2))
    iy = max(0.0, min(a["cy"] + a["h"] / 2, b["cy"] + b["h"] / 2)
             - max(a["cy"] - a["h"] / 2, b["cy"] - b["h"] / 2))
    i = ix * iy
    return i / (a["w"] * a["h"] + b["w"] * b["h"] - i + 1e-9)


def frame_verdict(dets):
    """dets: list of dicts {cls: 'H'|'R', conf, cx, cy, w, h}, all coords
    normalized to [0, 1].  Returns 'HR', 'RH' or None (frame abstains)."""
    dets = [d for d in dets
            if d["conf"] >= F.DET_CONF
            and F.DET_EDGE_MARGIN < d["cx"] < 1.0 - F.DET_EDGE_MARGIN
            and d["h"] >= F.DET_LATCH_MIN_H]
    dets.sort(key=lambda d: -d["conf"])
    kept = []
    for d in dets:
        if all(_iou(d, k) <= IOU_DEDUPE for k in kept):
            kept.append(d)
    kept = sorted(kept[:2], key=lambda d: d["cx"])
    seq = "".join(d["cls"] for d in kept)
    return seq if seq in ("HR", "RH") else None


class GoalSides:
    def __init__(self):
        self.votes = deque(maxlen=WINDOW)
        self.latched = None

    def update(self, dets):
        """Feed one frame's detections; returns the latched verdict or None
        while still undecided.  Once latched, stays latched."""
        if self.latched:
            return self.latched
        v = frame_verdict(dets)
        if v is not None:
            self.votes.append(v)
            n = self.votes.count(v)
            if n >= MIN_VOTES and n / len(self.votes) >= AGREE_FRAC:
                self.latched = v
        return self.latched
