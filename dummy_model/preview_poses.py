#!/usr/bin/env python3
"""Contact sheet of settled poses, for eyeballing against the reference photos.

This is the check that matters for the settling work: if the figures land in
plausible sprawls like _04 / _08 / _12 the joint stiction is roughly right; if
they flatten into starfish the friction is too low, and if they stay rigid in
their sampled configuration it is too high.

    python3 preview_poses.py --poses poses.jsonl --n 12
"""

import argparse
import json
import math
import os

import mujoco
import numpy as np
from PIL import Image

import sample_poses as SP

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", default=os.path.join(HERE, "poses.jsonl"))
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--cols", type=int, default=4)
    ap.add_argument("--cell", type=int, default=340)
    ap.add_argument("--only-hard", action="store_true")
    ap.add_argument("--out", default=os.path.join(HERE, "pose_sheet.png"))
    args = ap.parse_args()

    rows = [json.loads(l) for l in open(args.poses)]
    if args.only_hard:
        rows = [r for r in rows if r["hard"]]
    step = max(1, len(rows) // args.n)
    rows = rows[::step][:args.n]

    m = SP.build_scene()
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, args.cell, args.cell)
    cam, opt = mujoco.MjvCamera(), mujoco.MjvOption()
    for g in range(6):
        opt.geomgroup[g] = 0
    opt.geomgroup[2] = 1          # visual meshes only
    opt.geomgroup[0] = 1          # tunnel + field

    cols = args.cols
    rws = math.ceil(len(rows) / cols)
    sheet = Image.new("RGB", (cols * args.cell, rws * args.cell), (30, 30, 34))

    for i, row in enumerate(rows):
        d.qpos[:] = np.array(row["qpos"])
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        # Slight orbit per cell so the sheet is not all one viewpoint.
        cam.lookat[:] = [0, 0, 0.04]
        cam.distance = 0.62
        cam.elevation = -18
        cam.azimuth = 90 + (i * 37) % 360
        r.update_scene(d, cam, opt)
        img = Image.fromarray(r.render())
        sheet.paste(img, ((i % cols) * args.cell, (i // cols) * args.cell))

    sheet.save(args.out)
    print(f"wrote {args.out}  ({len(rows)} poses, "
          f"{sum(1 for r in rows if r['hard'])} hard-case)")


if __name__ == "__main__":
    main()
