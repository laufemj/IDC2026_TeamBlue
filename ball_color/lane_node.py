#!/usr/bin/env python3
"""Lane-centreline node: IMX500 preview stream -> steering offset.

On the Pi:
    python3 lane_node.py
Test mode (files, no ROS):
    python3 lane_node.py --images 'frames/*.jpg'

Topics:
    /line_offset    std_msgs/Float32   lateral offset of the pair centre,
                                       [-1, 1], + = steer right; NaN if lost
    /lanes_visible  std_msgs/Int8      0, 1 or 2

The frame source is the IMX500's PREVIEW stream -- the same camera that runs
the dummy detector network; inference and preview coexist on that module, so
lane tracking costs no extra hardware.  The tracker itself reads only the
lower BAND_FRAC of the image (the ground the robot is about to drive on);
mount tilt decides how much ground that is -- see README.
"""

import argparse
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lane_tracker import track

BAND_FRAC = 0.45        # lower fraction of the frame fed to the tracker
RATE_HZ = 20.0


def band(frame):
    H = frame.shape[0]
    return frame[int(H * (1.0 - BAND_FRAC)):]


def run_on_images(pattern):
    from PIL import Image
    for f in sorted(glob.glob(pattern)):
        frame = np.asarray(Image.open(f).convert("RGB"))
        off, n, centres = track(band(frame))
        print(f"{os.path.basename(f):40s} offset={off} lanes={n}")


def run_node():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Int8
    from picamera2 import Picamera2

    class LaneNode(Node):
        def __init__(self):
            super().__init__("lane_tracker")
            self.pub_off = self.create_publisher(Float32, "/line_offset", 10)
            self.pub_n = self.create_publisher(Int8, "/lanes_visible", 10)
            self.cam = Picamera2()
            self.cam.configure(self.cam.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}))
            self.cam.start()
            self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

        def tick(self):
            frame = self.cam.capture_array()
            off, n, _ = track(band(frame))
            self.pub_off.publish(Float32(
                data=float("nan") if off is None else off))
            self.pub_n.publish(Int8(data=n))

    rclpy.init()
    node = LaneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default=None)
    args = ap.parse_args()
    if args.images:
        run_on_images(args.images)
    else:
        run_node()


if __name__ == "__main__":
    main()
