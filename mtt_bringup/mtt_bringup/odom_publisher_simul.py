#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher_simul')

        self.declare_parameter('base_frame', '')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('pose_topic', 'gz_pose')

        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.robot_frame = base_frame or robot_frame
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.sub = self.create_subscription(Pose, self.pose_topic, self.callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)


    def callback(self, pose_msg):

        current_time = self.get_clock().now().to_msg()

        # Publish TF from odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = pose_msg.position.x
        t.transform.translation.y = pose_msg.position.y
        t.transform.translation.z = pose_msg.position.z
        t.transform.rotation = pose_msg.orientation
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.robot_frame
        odom_msg.pose.pose = pose_msg
        odom_msg.pose.covariance = [
        1e-9, 0,    0,    0,    0,    0,
        0,    1e-9, 0,    0,    0,    0,
        0,    0,    1e3,  0,    0,    0,
        0,    0,    0,    1e3,  0,    0,
        0,    0,    0,    0,    1e3,  0,
        0,    0,    0,    0,    0,    1e-9]

        # [ 
        # 0  1  2  3  4  5
        # 6  7  8  9 10 11
        # 12 13 14 15 16 17
        # 18 19 20 21 22 23
        # 24 25 26 27 28 29
        # 30 31 32 33 34 35 
        # ]

        # 0 Variance of X (m^2)
        # 7 Variance of Y (m^2)
        # 14 Variance of Z (m^2)
        # 21 Variance of Roll (rad^2)
        # 28 Variance of Pitch (rad^2)
        # 35 Variance of Yaw	(rad^2)

        # 1e-9	Very certain
        # 1e3	Not relevant

        # No twist information from Gazebo pose, so leave it zeroed
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
