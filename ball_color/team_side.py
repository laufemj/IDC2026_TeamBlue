#!/usr/bin/env python3
"""Team-side (red/blue) identification at match start.

The start zone is bordered with red or blue masking tape; the arena is
mirror-symmetric, so the very first thing the robot must know is which side
it woke up on -- every subsequent path decision flips with it.

    python3 team_side.py --images 'start_frames/*.jpg'    # test mode
    python3 team_side.py                                  # ROS node

Node behaviour: samples frames for up to `settle_frames`, votes red vs blue
pixels in the lower half (the tape lies on the ground around the robot), and
publishes ONE latched decision on /team_side ("red" | "blue").  Latched +
once: consumers can subscribe late and still get it, and a colour decision
must never flap mid-match.

Hue windows are generous and disjoint (red wraps 0, blue sits ~210-250) --
masking-tape colours are saturated primaries, and nothing else saturated-red
or saturated-blue should dominate the ground around the start zone.
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import rgb_to_hsv

RED_S_MIN = 0.35
BLUE_S_MIN = 0.30
DECISION_RATIO = 2.0        # winner needs 2x the loser's pixels
MIN_FRAC = 0.003            # winner must be at least this much of the band


def vote(frame_rgb):
    """Count red vs blue tape pixels in the lower half of a frame."""
    lower = frame_rgb[frame_rgb.shape[0] // 2:]
    rgb = lower.astype(np.float32) / 255.0
    h, s, v = rgb_to_hsv(rgb)
    red = ((h < 20) | (h > 340)) & (s > RED_S_MIN) & (v > 0.15)
    blue = (h > 195) & (h < 260) & (s > BLUE_S_MIN) & (v > 0.15)
    n = lower.shape[0] * lower.shape[1]
    return float(red.sum()) / n, float(blue.sum()) / n


def decide(fr, fb):
    if fr > fb * DECISION_RATIO and fr > MIN_FRAC:
        return "red"
    if fb > fr * DECISION_RATIO and fb > MIN_FRAC:
        return "blue"
    return None


def run_on_images(pattern):
    from PIL import Image
    for f in sorted(glob.glob(pattern)):
        fr, fb = vote(np.asarray(Image.open(f).convert("RGB")))
        print(f"{os.path.basename(f):40s} red={fr:.4f} blue={fb:.4f} "
              f"-> {decide(fr, fb)}")


def run_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from picamera2 import Picamera2

    class TeamSideNode(Node):
        def __init__(self):
            super().__init__("team_side")
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # latched
            self.pub = self.create_publisher(String, "/team_side", qos)
            self.cam = Picamera2()
            self.cam.configure(self.cam.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"}))
            self.cam.start()
            self.votes = []
            self.timer = self.create_timer(0.2, self.tick)

        def tick(self):
            fr, fb = vote(self.cam.capture_array())
            d = decide(fr, fb)
            if d:
                self.votes.append(d)
            if len(self.votes) >= 5:
                final = max(set(self.votes), key=self.votes.count)
                self.pub.publish(String(data=final))
                self.get_logger().info(f"team side: {final}")
                self.timer.cancel()          # one decision, then silence

    rclpy.init()
    node = TeamSideNode()
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
