#!/usr/bin/env python3
"""Run the trained detector on the 127 real venue frames and build montages.

    ~/.venvs/yolo/bin/python infer_real.py [--weights runs/v1_synth/weights/best.pt]

This is the evaluation that matters: the model has only ever seen synthetic
frames, and these are real IMX500 captures of the real field.  No labels
exist, so the output is visual -- annotated frames tiled into contact sheets
under review/infer_v1/.

Interpreting it honestly:
  * figures found + right class          -> the sim-to-real transfer works
  * figures found + classes confused     -> material gap (expected risk:
    the v1 human has no garment mesh, only a cloth-coloured material)
  * boxes on people/furniture/background -> FP suppression needs more
    negatives/clutter in v2
  * figures missed entirely              -> domain gap too wide; look at
    scale/lighting first
Note the standing-pose bursts (173126/173204) are rule-illegal placements;
misses there are uninformative -- the training pose space excludes standing
on purpose.
"""

import argparse
import glob
import os

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
REAL = os.path.join(REPO, "image_20260812_173204_0001")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights",
                    default=os.path.join(HERE, "runs/v1_synth/weights/best.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--out", default=os.path.join(HERE, "review/infer_v1"))
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    files = sorted(glob.glob(os.path.join(REAL, "*.jpg")))
    os.makedirs(args.out, exist_ok=True)

    names = {0: "human", 1: "robot"}
    colors = {0: (0, 220, 80), 1: (255, 130, 0)}
    annotated = []
    n_det = 0
    for f in files:
        r = model.predict(f, imgsz=640, conf=args.conf, verbose=False)[0]
        im = Image.open(f).convert("RGB")
        d = ImageDraw.Draw(im)
        for b in r.boxes:
            c = int(b.cls[0])
            x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
            d.rectangle([x0, y0, x1, y1], outline=colors[c], width=4)
            d.text((x0 + 4, y0 + 4),
                   f"{names[c]} {float(b.conf[0]):.2f}", fill=colors[c])
            n_det += 1
        annotated.append(im)

    # contact sheets, 24 frames each
    tw = 480
    th = tw * 1080 // 1920
    cols, per = 4, 24
    for s in range((len(annotated) + per - 1) // per):
        batch = annotated[s * per:(s + 1) * per]
        rows = (len(batch) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * tw, rows * (th + 14)), (12, 12, 14))
        dr = ImageDraw.Draw(sheet)
        for i, im in enumerate(batch):
            x, y = (i % cols) * tw, (i // cols) * (th + 14)
            sheet.paste(im.resize((tw, th)), (x, y))
            dr.text((x + 3, y + th + 1),
                    os.path.basename(files[s * per + i])[6:-4],
                    fill=(210, 210, 90))
        sheet.save(os.path.join(args.out, f"sheet_{s}.png"))
    print(f"INFER_DONE {len(files)} frames, {n_det} detections "
          f"-> {args.out}/sheet_*.png")


if __name__ == "__main__":
    main()
