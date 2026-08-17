#!/usr/bin/env python3
"""On-robot calibration helper.  Run on the Pi with the camera mounted.

    python3 capture_reference.py empty     # pocket EMPTY  -> empty_pocket.npy
    python3 capture_reference.py orange    # orange ball in pocket -> shots/
    python3 capture_reference.py silver    # silver ball in pocket -> shots/
    python3 capture_reference.py preview   # save one full frame to check ROI

Procedure (five minutes, once, at the venue if possible):
  1. `preview`, open the saved frame, adjust "roi" in config.json so the
     pocket fills it.
  2. `empty` with nothing in the pocket.
  3. `orange` / `silver` a few times each, rolling the ball in naturally.
  4. `python3 ball_color_node.py --images 'shots/*.jpg'` and check every
     verdict.  Adjust config.json thresholds only if something misreads.
"""

import os
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import load_config

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "preview"
    cfg = load_config()
    cap = cfg["capture"]

    from picamera2 import Picamera2
    from libcamera import controls
    cam = Picamera2()
    cam.configure(cam.create_still_configuration(
        main={"size": tuple(cap["size"]), "format": "RGB888"}))
    ctrl = {
        "AwbEnable": cap["awb_enable"],
        "ColourGains": tuple(cap["colour_gains"]),
        "AeEnable": False,
        "ExposureTime": cap["exposure_us"],
        "AnalogueGain": cap["analogue_gain"],
    }
    if cap["af_mode_manual"]:
        ctrl["AfMode"] = controls.AfModeEnum.Manual
        ctrl["LensPosition"] = cap["lens_position"]
    cam.set_controls(ctrl)
    cam.start()
    time.sleep(0.5)
    frame = cam.capture_array()

    if mode == "preview":
        out = os.path.join(HERE, "preview_full.png")
        Image.fromarray(frame).save(out)
        x0, y0, x1, y1 = cfg["roi"]
        Image.fromarray(frame[y0:y1, x0:x1]).save(
            os.path.join(HERE, "preview_roi.png"))
        print(f"saved {out} (+ preview_roi.png) -- check the ROI box")
    elif mode == "empty":
        x0, y0, x1, y1 = cfg["roi"]
        np.save(os.path.join(HERE, "empty_pocket.npy"),
                frame[y0:y1, x0:x1])
        print("saved empty_pocket.npy")
    else:
        os.makedirs(os.path.join(HERE, "shots"), exist_ok=True)
        name = f"{mode}_{int(time.time())}.png"
        x0, y0, x1, y1 = cfg["roi"]
        Image.fromarray(frame[y0:y1, x0:x1]).save(
            os.path.join(HERE, "shots", name))
        print(f"saved shots/{name}")


if __name__ == "__main__":
    main()
