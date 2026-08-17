#!/usr/bin/env python3
"""Train YOLO11n on the synthetic v1 dataset.

    ~/.venvs/yolo/bin/python train_v1.py

Settings notes:
  * imgsz 640: the deployment input (IMX500 letterboxes 640x480 into
    640x640; ultralytics applies the same letterboxing to rectangular
    sources, so training and deployment see identical geometry).
  * mosaic stays ON: it multiplies effective backgrounds and object scales,
    both of which we are short on in v1.
  * hsv_h kept small: hue separates the classes (warm wood vs cool silver),
    and a big hue jitter would erase the one cue that matters.  Exposure-ish
    jitter (hsv_v) can stay generous.
"""

from ultralytics import YOLO, settings

# keep outputs inside the project -- a global runs_dir from an old project
# was silently redirecting weights to another repo
settings.update({"runs_dir": "/home/mj/Desktop/IDC2026/dummy_model/runs"})


def main():
    model = YOLO("yolo11n.pt")
    model.train(
        data="dataset_all/data.yaml",
        epochs=80,
        imgsz=640,
        batch=64,
        device=0,
        workers=8,
        project="runs",
        name="v1_synth",
        hsv_h=0.008,        # protect the wood/silver hue cue
        hsv_s=0.4,
        hsv_v=0.45,
        degrees=6.0,
        translate=0.1,
        scale=0.35,
        fliplr=0.5,
        mosaic=1.0,
        close_mosaic=10,
        patience=20,
    )


if __name__ == "__main__":
    main()
