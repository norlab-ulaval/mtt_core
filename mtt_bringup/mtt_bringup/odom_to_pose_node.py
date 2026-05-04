#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_pose_node")
        self.declare_parameter("odom_topic", "odom_in")
        self.declare_parameter("pose_topic", "pose_out")

        odom_topic = str(self.get_parameter("odom_topic").value)
        pose_topic = str(self.get_parameter("pose_topic").value)

        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 20)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        self.get_logger().info(f"Odom-to-pose bridge ready (odom={odom_topic}, pose={pose_topic})")

    def _on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._pose_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
