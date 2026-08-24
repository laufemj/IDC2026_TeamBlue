#!/usr/bin/env python3
"""Servo node: flap + feeder + arms (MG996R x3 via gpiozero AngularServo).

    python3 servo_node.py [--ros-args -p flap_pin:=12 ...]

Topics:
    /servo/flap  std_msgs/String  "left" | "right"  -- s3 routing flap
    /servo/feed  std_msgs/Empty   release exactly ONE ball -- s2 gate does
                                  one open-close cycle per message
    /servo/arms  std_msgs/Bool    true = extend collecting arms (s1)

Angle defaults are the RECOVERED Pi-session contract values
(docs/pi_session_recovery.md), expressed on gpiozero's -90..+90 scale
(Pi config used 0..180: value_here = pi_value - 90):
  servo-1 (arms+gate, one shaft, lost-motion): STOW 10, ARMS 90,
  DISPENSE 140, VERTICAL/closed 170  ->  -80 / 0 / +50 / +80
  servo-2 (flap): LEFT 35 / CENTER 90 / RIGHT 145  ->  -55 / 0 / +55
Verify on the real linkage (the Pi's test_hardware tool that bench-tuned
them is lost).  PIN numbers are still PLACEHOLDERS -- set from real wiring.
MG996R on gpiozero defaults (1-2 ms pulse); widen min/max_pulse_width if
the real horn travel comes up short.

Note for Pi 5: gpiozero's software PWM jitters; if the flap chatters, move
these three to the kit's PWM board (ADA-815) and keep this node's topic
interface identical.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

FEED_CYCLE_S = 0.30     # gate open time for one ball (recovered Pi contract:
                        # DISPENSE held 0.30 s releases exactly one ball)


class ServoNode(Node):
    def __init__(self):
        super().__init__("servos")
        for name, default in (
                ("flap_pin", 12), ("feed_pin", 13), ("arms_pin", 19),
                ("flap_left_deg", -55.0), ("flap_right_deg", 55.0),
                ("feed_closed_deg", 80.0), ("feed_open_deg", 50.0),
                ("arms_in_deg", -80.0), ("arms_out_deg", 0.0)):
            self.declare_parameter(name, default)
        p = lambda k: self.get_parameter(k).value

        from gpiozero import AngularServo
        mk = lambda pin: AngularServo(int(pin), min_angle=-90, max_angle=90)
        self.flap = mk(p("flap_pin"))
        self.feed = mk(p("feed_pin"))
        self.arms = mk(p("arms_pin"))
        self.angles = {k: float(p(k)) for k in (
            "flap_left_deg", "flap_right_deg", "feed_closed_deg",
            "feed_open_deg", "arms_in_deg", "arms_out_deg")}
        self.feed.angle = self.angles["feed_closed_deg"]

        self.create_subscription(String, "/servo/flap", self.on_flap, 10)
        self.create_subscription(Empty, "/servo/feed", self.on_feed, 10)
        self.create_subscription(Bool, "/servo/arms", self.on_arms, 10)

    def on_flap(self, msg):
        key = "flap_left_deg" if msg.data == "left" else "flap_right_deg"
        self.flap.angle = self.angles[key]

    def on_feed(self, _):
        # blocking is fine: one ball per message, cycle < half a second, and
        # the mission's FLAP_SETTLE ordering already happened upstream
        self.feed.angle = self.angles["feed_open_deg"]
        time.sleep(FEED_CYCLE_S)
        self.feed.angle = self.angles["feed_closed_deg"]

    def on_arms(self, msg):
        key = "arms_out_deg" if msg.data else "arms_in_deg"
        self.arms.angle = self.angles[key]


def main():
    rclpy.init()
    node = ServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
