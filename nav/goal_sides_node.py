#!/usr/bin/env python3
"""Goal-identity node: dummy detections -> ONE latched left/right verdict.

    python3 goal_sides_node.py [--ros-args -p detections_topic:=/dummy_detections]

Topics:
    in   /dummy_detections  vision_msgs/Detection2DArray, NORMALIZED coords
         (bbox center/size in [0, 1] image fractions; class_id "0"/"human"
         = human figure, "1"/"robot" = robot figure)
    out  /goal_sides        std_msgs/String, latched (TRANSIENT_LOCAL):
         "HR" = human figure on the LEFT goal, robot on the RIGHT
         "RH" = the mirror

All filtering/voting policy lives in goal_sides.py (pure python, testable);
this file is only the ROS plumbing.
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from goal_sides import GoalSides

HUMAN_IDS = {"0", "human", "H"}
ROBOT_IDS = {"1", "robot", "R"}


class GoalSidesNode(Node):
    def __init__(self):
        super().__init__("goal_sides")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, "/goal_sides", qos)
        self.declare_parameter("detections_topic", "/dummy_detections")
        topic = self.get_parameter("detections_topic").value

        self.logic = GoalSides()
        self.done = False
        from vision_msgs.msg import Detection2DArray
        self.create_subscription(Detection2DArray, topic, self.on_det, 10)

    def on_det(self, msg):
        if self.done:
            return
        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            hyp = max(d.results, key=lambda r: r.hypothesis.score)
            cid = str(hyp.hypothesis.class_id)
            if cid in HUMAN_IDS:
                cls = "H"
            elif cid in ROBOT_IDS:
                cls = "R"
            else:
                continue
            dets.append(dict(
                cls=cls, conf=float(hyp.hypothesis.score),
                cx=float(d.bbox.center.position.x),
                cy=float(d.bbox.center.position.y),
                w=float(d.bbox.size_x), h=float(d.bbox.size_y)))
        verdict = self.logic.update(dets)
        if verdict:
            self.done = True
            self.pub.publish(String(data=verdict))
            self.get_logger().info(f"goal sides latched: {verdict} "
                                   f"(left={'human' if verdict == 'HR' else 'robot'})")


def main():
    rclpy.init()
    node = GoalSidesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
