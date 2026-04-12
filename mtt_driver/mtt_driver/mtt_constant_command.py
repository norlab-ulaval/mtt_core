#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class MTTConstantCommand(Node):
    def __init__(self):
        super().__init__("mtt_constant_command")

        self.declare_parameter("topic", "controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("publish_zero_on_shutdown", True)

        self.topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.publish_zero_on_shutdown = bool(self.get_parameter("publish_zero_on_shutdown").value)

        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)
        self.timer = self.create_timer(max(1e-3, 1.0 / publish_rate_hz), self.publish_command)

        self.get_logger().info(
            f"Publishing TwistStamped on {self.topic} "
            f"(linear={self.linear_speed:.3f} m/s, angular={self.angular_speed:.3f} rad/s)"
        )

    def _make_message(self, linear_speed: float, angular_speed: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = linear_speed
        msg.twist.angular.z = angular_speed
        return msg

    def publish_command(self):
        self.publisher.publish(self._make_message(self.linear_speed, self.angular_speed))

    def stop(self):
        if not self.publish_zero_on_shutdown:
            return
        self.publisher.publish(self._make_message(0.0, 0.0))


def main(args=None):
    rclpy.init(args=args)
    node = MTTConstantCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
