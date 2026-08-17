#!/usr/bin/env python3
"""Ball colour node: Camera Module 3 -> /ball_color.

On the Pi (with ROS 2 + picamera2):
    ros2 run ...  or simply:  python3 ball_color_node.py

Off-robot test mode (no ROS, no camera needed -- runs anywhere):
    python3 ball_color_node.py --images 'shots/*.jpg' [--empty shots/empty.jpg]

Topics:
    /ball_color         std_msgs/String, event: "orange" | "silver", published
                        once per ball after debounce (median window + stability
                        run).  Re-arms when the pocket empties.
    /ball_color/state   std_msgs/String, continuous per-frame label at frame
                        rate, for debugging and for anyone who wants raw state.

Camera policy (the part that actually decides reliability):
    * auto-white-balance OFF, fixed colour gains -- the venue's lime bounce
      drags AWB around and moved the balls' hue 20 deg in our measurements,
    * fixed exposure + gain -- the pocket sees its own shadow, not the room,
    * manual focus, locked at the pocket distance -- defocus does not hurt a
      colour vote, but AF hunting changes exposure mid-decision.
    All values in config.json; capture_reference.py helps pick them.
"""

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import Debouncer, classify_roi, load_config


def crop(frame, roi):
    x0, y0, x1, y1 = roi
    return frame[y0:y1, x0:x1]


# --------------------------------------------------------------------------
# test mode: files in, verdicts out, no ROS / no camera
# --------------------------------------------------------------------------

def run_on_images(pattern, empty_path, cfg, full_frame):
    from PIL import Image
    ref = None
    if empty_path:
        ref = np.asarray(Image.open(empty_path).convert("RGB"))
        if full_frame:
            ref = crop(ref, cfg["roi"])
    for f in sorted(glob.glob(pattern)):
        img = np.asarray(Image.open(f).convert("RGB"))
        if full_frame:
            img = crop(img, cfg["roi"])
        label, info = classify_roi(img, cfg, ref)
        print(f"{os.path.basename(f):40s} {label:7s} {info}")


# --------------------------------------------------------------------------
# robot mode: picamera2 capture, ROS 2 publish
# --------------------------------------------------------------------------

def run_node(cfg):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from picamera2 import Picamera2
    from libcamera import controls

    class BallColorNode(Node):
        def __init__(self):
            super().__init__("ball_color")
            self.pub_event = self.create_publisher(String, "/ball_color", 10)
            self.pub_state = self.create_publisher(String,
                                                   "/ball_color/state", 10)
            cap = cfg["capture"]
            self.cam = Picamera2()
            config = self.cam.create_video_configuration(
                main={"size": tuple(cap["size"]), "format": "RGB888"})
            self.cam.configure(config)
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
            self.cam.set_controls(ctrl)
            self.cam.start()

            self.ref = None
            ref_path = os.path.join(os.path.dirname(__file__),
                                    "empty_pocket.npy")
            if os.path.exists(ref_path):
                self.ref = np.load(ref_path)
                self.get_logger().info("empty-pocket reference loaded")
            else:
                self.get_logger().warning(
                    "no empty_pocket.npy -- presence gating disabled; "
                    "run capture_reference.py with the pocket empty")

            self.debounce = Debouncer(cfg["debounce_window"],
                                      cfg["debounce_stable"])
            period = 1.0 / cfg["capture"]["fps"]
            self.timer = self.create_timer(period, self.tick)

        def tick(self):
            frame = self.cam.capture_array()          # RGB888
            roi = crop(frame, cfg["roi"])
            label, _ = classify_roi(roi, cfg, self.ref)
            self.pub_state.publish(String(data=label))
            emit = self.debounce.update(label)
            if emit is not None:
                self.get_logger().info(f"ball detected: {emit}")
                self.pub_event.publish(String(data=emit))

    rclpy.init()
    node = BallColorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--images", default=None,
                    help="glob of test images; enables no-ROS test mode")
    ap.add_argument("--empty", default=None,
                    help="empty-pocket image for presence gating (test mode)")
    ap.add_argument("--full-frame", action="store_true",
                    help="test images are full frames; crop config ROI")
    args = ap.parse_args()

    cfg = load_config(args.config) if args.config else load_config()
    if args.images:
        run_on_images(args.images, args.empty, cfg, args.full_frame)
    else:
        run_node(cfg)


if __name__ == "__main__":
    main()
