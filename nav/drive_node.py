#!/usr/bin/env python3
"""Drive node: /cmd_vel (normalized duties) -> Cytron MDD3A -> two DC wheels.

    python3 drive_node.py [--ros-args -p left_pins:="[17,18]" ...]

Topics:
    /cmd_vel   geometry_msgs/Twist   linear.x  = forward duty [-1, 1]
                                     angular.z = turn duty    [-1, 1], + = right

No encoders: duties are open-loop.  linear/angular are mixed per side and
clamped; TRIM compensates a real robot's natural veer (calibrate by driving
a straight line -- TODO-FIELD family).

MDD3A is sign-magnitude per channel (two input pins, PWM one / LOW other);
gpiozero.Motor does exactly that, and works on Pi 5 (lgpio backend).

Deadman: if no /cmd_vel arrives for DEADMAN_S the motors stop -- a crashed
mission node must not send the robot off the table ([RB p8-g]).

Pin numbers are parameters with PLACEHOLDER defaults -- set them from the
real wiring.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

DEADMAN_S = 0.5
TRIM = 0.0              # + slows the left wheel (robot veers left -> raise)


def mix(v, w, trim=TRIM):
    """(forward, turn) duties -> (left, right) wheel duties, clamped."""
    left = v + w
    right = v - w
    m = max(1.0, abs(left), abs(right))
    left, right = left / m, right / m
    left *= (1.0 - max(0.0, trim))
    right *= (1.0 + min(0.0, trim))
    return left, right


class DriveNode(Node):
    def __init__(self):
        super().__init__("drive")
        # RECOVERED wiring (docs/pi_session_recovery.md): MDD3A signals on
        # physical pins 32/11 (M1A/M1B) and 33/16 (M2A/M2B) = BCM 12/17 and
        # 13/23.  Which channel is LEFT and each polarity: verify by teleop,
        # then swap the pin lists / pin order here if needed.
        self.declare_parameter("left_pins", [12, 17])    # M1 (BCM)
        self.declare_parameter("right_pins", [13, 23])   # M2 (BCM)
        self.declare_parameter("trim", TRIM)
        lp = self.get_parameter("left_pins").value
        rp = self.get_parameter("right_pins").value
        self.trim = float(self.get_parameter("trim").value)

        from gpiozero import Motor
        self.left = Motor(forward=lp[0], backward=lp[1], pwm=True)
        self.right = Motor(forward=rp[0], backward=rp[1], pwm=True)

        self.last_cmd = self.get_clock().now()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info(f"drive up, pins L{lp} R{rp}")

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        left, right = mix(float(msg.linear.x), float(msg.angular.z), self.trim)
        for motor, duty in ((self.left, left), (self.right, right)):
            if duty >= 0:
                motor.forward(min(1.0, duty))
            else:
                motor.backward(min(1.0, -duty))

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if dt > DEADMAN_S:
            self.left.stop()
            self.right.stop()


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    finally:
        node.left.stop()
        node.right.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
