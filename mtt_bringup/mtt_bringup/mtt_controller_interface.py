#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from mtt_driver.mtt_vehicle_params import get_mtt_params

class MttControllerInterface(Node):

    def __init__(self):
        super().__init__('mtt_controller_interface')
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("wheel_command_topic", "wheel_group_controller/commands")
        self.declare_parameter("yaw_command_topic", "yaw_controller/commands")

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        wheel_command_topic = self.get_parameter("wheel_command_topic").value
        yaw_command_topic = self.get_parameter("yaw_command_topic").value

        self.wheel_publisher = self.create_publisher(Float64MultiArray, wheel_command_topic, 10)
        self.yaw_publisher = self.create_publisher(Float64MultiArray, yaw_command_topic, 10)
        self.subscription = self.create_subscription(TwistStamped, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.mtt_params = get_mtt_params()
        self.gear_ratio = self.mtt_params.mechanical_gear_ratio


    def cmd_vel_callback(self, cmd_vel_msg: TwistStamped):

        # negative sign to have the right direction
        # *20 to make an array for each of the 20 wheels
        wheel_speeds = [-self.gear_ratio*cmd_vel_msg.twist.linear.x] * 20

        wheel_cmd_msg = Float64MultiArray()
        wheel_cmd_msg.data = wheel_speeds
        self.wheel_publisher.publish(wheel_cmd_msg)

        yaw_cmd_msg = Float64MultiArray()
       
        yaw = [-cmd_vel_msg.twist.angular.z]
        yaw_cmd_msg.data = yaw
        self.yaw_publisher.publish(yaw_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MttControllerInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
