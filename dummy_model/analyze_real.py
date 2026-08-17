#!/usr/bin/env python3
"""Numeric analysis of a detector's output on the 127 real venue frames.

No GT exists, so every metric leans on structure instead:
  1. det/frame per burst vs the known real figure count
  2. impossible-region rate (above the field horizon nothing can sit)
  3. size plausibility (300 mm figure at 1-4.5 m -> 98-440 px at 1080p)
  4. class balance (fleet is 50/50 by rule)
  5. mutual-exclusivity (a goal pair holds one H + one R -> top-2 should mix)
  6. temporal flicker across adjacent frames of a burst

Usage: analyze with dets json produced by the collection snippet in git log,
or import and call analyze(rows). Baselines: v0_early results 2026-08-17.
"""

import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REAL = os.path.join(os.path.dirname(HERE), "image_20260812_173204_0001")
# Figure counts verified frame-by-frame 2026-08-17.  172904/173126 were
# first assumed empty -- WRONG: both contain small far-field figures
# (standing/slumped) that thumbnail review missed.  Their counts are
# approximate and their poses irregular, so clean metrics use only the
# seated bursts (SEATED set).
EXPECT = {"172414": 2, "172514": 2, "172618": 2, "172810": 2,
          "172904": 3, "173126": 4, "173204": 4}
SEATED = {"172414", "172514", "172618", "172810"}
H = 1080


def collect(weights, conf=0.25):
    from ultralytics import YOLO
    m = YOLO(weights)
    rows = []
    for f in sorted(glob.glob(os.path.join(REAL, "*.jpg"))):
        r = m.predict(f, imgsz=640, conf=conf, verbose=False)[0]
        dets = []
        for b in r.boxes:
            x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
            dets.append(dict(cls=int(b.cls[0]), conf=float(b.conf[0]),
                             cx=(x0 + x1) / 2, cy=(y0 + y1) / 2,
                             w=x1 - x0, h=y1 - y0))
        rows.append(dict(frame=os.path.basename(f),
                         burst=os.path.basename(f).split("_")[2], dets=dets))
    return rows


def _iou(a, b):
    ix = max(0, min(a["cx"] + a["w"] / 2, b["cx"] + b["w"] / 2)
             - max(a["cx"] - a["w"] / 2, b["cx"] - b["w"] / 2))
    iy = max(0, min(a["cy"] + a["h"] / 2, b["cy"] + b["h"] / 2)
             - max(a["cy"] - a["h"] / 2, b["cy"] - b["h"] / 2))
    i = ix * iy
    return i / (a["w"] * a["h"] + b["w"] * b["h"] - i + 1e-9)


def analyze(rows):
    out = {}
    for b in sorted(EXPECT):
        fr = [r for r in rows if r["burst"] == b]
        out[f"det_frame_{b}"] = (
            float(np.mean([len(r["dets"]) for r in fr])),
            float(np.mean([sum(1 for d in r["dets"] if d["conf"] >= 0.5)
                           for r in fr])), EXPECT[b])
    for t in (0.25, 0.5):
        sel = [d for r in rows for d in r["dets"] if d["conf"] >= t]
        out[f"upper_fp_{t}"] = (
            sum(1 for d in sel if d["cy"] < 0.42 * H), len(sel))
        out[f"size_bad_{t}"] = (
            sum(1 for d in sel if not 60 < d["h"] < 600), len(sel))
        out[f"cls_{t}"] = (sum(1 for d in sel if d["cls"] == 0),
                           sum(1 for d in sel if d["cls"] == 1))
    ok = mono = short = 0
    for r in rows:
        if EXPECT[r["burst"]] != 2:
            continue
        low = sorted((d for d in r["dets"] if d["cy"] >= 0.42 * H),
                     key=lambda d: -d["conf"])[:2]
        if len(low) < 2:
            short += 1
        elif {low[0]["cls"], low[1]["cls"]} == {0, 1}:
            ok += 1
        else:
            mono += 1
    out["pair_HR"] = (ok, mono, short)
    flick = tot = 0
    for i in range(len(rows) - 1):
        if rows[i]["burst"] != rows[i + 1]["burst"]:
            continue
        for d in rows[i]["dets"]:
            if d["conf"] < 0.5:
                continue
            tot += 1
            flick += not any(_iou(d, e) > 0.3 for e in rows[i + 1]["dets"])
    out["flicker"] = (flick, tot)
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--save", default=None)
    a = ap.parse_args()
    rows = collect(a.weights)
    res = analyze(rows)
    if a.save:
        json.dump({"rows": rows, "metrics": res}, open(a.save, "w"))
    for k, v in res.items():
        print(k, v)
