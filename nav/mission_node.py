#!/usr/bin/env python3
"""Mission node: the match orchestrator.  Logic lives in mission_logic.py
and steering.py (pure python, testable); this file is the ROS plumbing.

    python3 mission_node.py --ros-args -p n_balls:=2 -p cfg_side:=blue \
                                       -p cfg_goal_sides:=HR

Parameters:
    n_balls         balls loaded this match (1+1 + cyber advantage) [RB p5-6]
    cfg_side        the side draw, announced >= 5 min pre-match [RB p8-f]
    cfg_goal_sides  fallback verdict if /goal_sides never latches
    unknown_side    where an unclassifiable ball goes (protect the DOUBLE
                    of the goal we stuff most; see mission_logic)

Subscribes:  /team_side /line_offset /arrived /goal_sides /ball_color
Publishes:   /cmd_vel (Twist), /servo/flap (String left|right),
             /servo/feed (std_msgs/Empty, one ball per message),
             /mission_state (String, on every transition)
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Float32, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_logic import Mission
from steering import Steering

TICK_HZ = 20.0


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("n_balls", 2)
        self.declare_parameter("cfg_side", "blue")
        self.declare_parameter("cfg_goal_sides", "HR")
        self.declare_parameter("unknown_side", "right")
        p = lambda k: self.get_parameter(k).value

        self.mission = Mission(
            n_balls=int(p("n_balls")), cfg_side=p("cfg_side"),
            cfg_goal_sides=p("cfg_goal_sides"),
            unknown_side=p("unknown_side"))
        self.steering = Steering()
        self.offset = None
        self.last_state = None

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_flap = self.create_publisher(String, "/servo/flap", 10)
        self.pub_feed = self.create_publisher(Empty, "/servo/feed", 10)
        self.pub_state = self.create_publisher(String, "/mission_state", latched)

        self.create_subscription(String, "/team_side",
                                 lambda m: self.mission.on_team_side(m.data),
                                 latched)
        self.create_subscription(String, "/goal_sides",
                                 lambda m: self.mission.on_goal_sides(m.data),
                                 latched)
        self.create_subscription(Bool, "/arrived",
                                 lambda m: m.data and self.mission.on_arrived(),
                                 latched)
        self.create_subscription(Float32, "/line_offset", self.on_offset, 10)
        self.create_subscription(String, "/ball_color", self.on_ball, 10)
        self.create_timer(1.0 / TICK_HZ, self.tick)

    def on_offset(self, msg):
        self.offset = None if math.isnan(msg.data) else float(msg.data)

    def on_ball(self, msg):
        self.execute(self.mission.on_ball_color(msg.data))

    def tick(self):
        self.execute(self.mission.step(self.offset, self.steering))
        if self.mission.state != self.last_state:
            self.last_state = self.mission.state
            self.pub_state.publish(String(data=self.mission.state))
            self.get_logger().info(f"state -> {self.mission.state}")

    def execute(self, cmds):
        for verb, arg in cmds:
            if verb == "drive":
                t = Twist()
                t.linear.x, t.angular.z = float(arg[0]), float(arg[1])
                self.pub_vel.publish(t)
            elif verb == "flap":
                self.pub_flap.publish(String(data=arg))
            elif verb == "feed":
                self.pub_feed.publish(Empty())
            elif verb == "done":
                self.pub_vel.publish(Twist())   # zeros = stop


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
