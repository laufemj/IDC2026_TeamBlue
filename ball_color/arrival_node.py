#!/usr/bin/env python3
"""Arrival detector: are we parked between the two goals yet?

Fuses the two agreed cues -- no extra hardware:
  (a) the dummy detector's boxes have grown past a size threshold
      (near = big), and
  (b) the lanes have left the near band (/lanes_visible == 0 for a run of
      frames) -- between the goals the tape is behind/beneath the robot.

Publishes /arrived (std_msgs/Bool, latched) once both cues agree.  (b) alone
also fires after a longer run, so a missed detection cannot strand the
robot; (a) alone never fires, so a spurious big box cannot stop it early.

The detections topic is parameterized (default /dummy_detections,
vision_msgs/Detection2DArray) -- wire it to whatever the IMX500 detector
node ends up publishing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int8

# "close enough" bbox height.  Kept above field_spec.DET_LATCH_MIN_H (0.25),
# the range below which the real-photo study showed the detector's
# human/robot call is not yet 100% reliable (docs/lr_field_report.html).
BBOX_H_NEAR = 0.28          # bbox height fraction of image
LANES_GONE_WITH_BOX = 8     # frames of lanes==0 needed when (a) is also true
LANES_GONE_ALONE = 30       # frames of lanes==0 needed on its own


class ArrivalNode(Node):
    def __init__(self):
        super().__init__("arrival")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Bool, "/arrived", qos)
        self.declare_parameter("detections_topic", "/dummy_detections")
        topic = self.get_parameter("detections_topic").value

        self.big_box = False
        self.gone_run = 0
        self.done = False

        self.create_subscription(Int8, "/lanes_visible", self.on_lanes, 10)
        try:
            from vision_msgs.msg import Detection2DArray
            self.create_subscription(Detection2DArray, topic, self.on_det, 10)
        except ImportError:
            self.get_logger().warning(
                "vision_msgs not available -- arrival will use lane loss "
                "alone (longer run)")

    def on_det(self, msg):
        h = max((d.bbox.size_y for d in msg.detections), default=0.0)
        self.big_box = h >= BBOX_H_NEAR

    def on_lanes(self, msg):
        if self.done:
            return
        self.gone_run = self.gone_run + 1 if msg.data == 0 else 0
        need = LANES_GONE_WITH_BOX if self.big_box else LANES_GONE_ALONE
        if self.gone_run >= need:
            self.done = True
            self.pub.publish(Bool(data=True))
            self.get_logger().info(
                f"arrived (lanes gone x{self.gone_run}, "
                f"big_box={self.big_box})")


def main():
    rclpy.init()
    node = ArrivalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
