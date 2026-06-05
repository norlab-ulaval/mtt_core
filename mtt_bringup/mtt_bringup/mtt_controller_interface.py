#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MttControllerInterface(Node):

    def __init__(self):
        super().__init__('mtt_controller_interface')
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("wheel_command_topic", "wheel_group_controller/commands")
        self.declare_parameter("yaw_command_topic", "yaw_controller/commands")
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("wheel_command_sign", -1.0)
        self.declare_parameter("yaw_command_sign", 1.0)
        self.declare_parameter("yaw_command_mode", "position_servo")
        self.declare_parameter("yaw_joint_name", "yaw")
        self.declare_parameter("yaw_max_articulation_rad", math.radians(60.0))
        self.declare_parameter("yaw_position_kp", 5.0)
        self.declare_parameter("yaw_velocity_limit_rad_s", 2.5)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        wheel_command_topic = self.get_parameter("wheel_command_topic").value
        yaw_command_topic = self.get_parameter("yaw_command_topic").value

        self.wheel_publisher = self.create_publisher(Float64MultiArray, wheel_command_topic, 10)
        self.yaw_publisher = self.create_publisher(Float64MultiArray, yaw_command_topic, 10)
        self.subscription = self.create_subscription(TwistStamped, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.joint_state_subscription = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 20)

        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.wheel_command_sign = float(self.get_parameter("wheel_command_sign").value)
        self.yaw_command_sign = float(self.get_parameter("yaw_command_sign").value)
        self.yaw_command_mode = str(self.get_parameter("yaw_command_mode").value)
        self.yaw_joint_name = str(self.get_parameter("yaw_joint_name").value)
        self.yaw_max_articulation_rad = float(self.get_parameter("yaw_max_articulation_rad").value)
        self.yaw_position_kp = float(self.get_parameter("yaw_position_kp").value)
        self.yaw_velocity_limit_rad_s = float(self.get_parameter("yaw_velocity_limit_rad_s").value)
        self.current_yaw_rad = 0.0
        self.has_yaw_state = False

    def joint_state_callback(self, msg: JointState):
        try:
            index = msg.name.index(self.yaw_joint_name)
        except ValueError:
            return
        if index < len(msg.position):
            self.current_yaw_rad = msg.position[index]
            self.has_yaw_state = True

    def yaw_command_from_normalized_steer(self, normalized_steer: float) -> float:
        normalized_steer = max(-1.0, min(1.0, normalized_steer))
        if self.yaw_command_mode == "velocity":
            return self.yaw_command_sign * normalized_steer

        target_yaw_rad = self.yaw_command_sign * normalized_steer * self.yaw_max_articulation_rad
        if not self.has_yaw_state:
            return 0.0

        error_rad = target_yaw_rad - self.current_yaw_rad
        velocity_cmd = self.yaw_position_kp * error_rad
        return max(-self.yaw_velocity_limit_rad_s, min(self.yaw_velocity_limit_rad_s, velocity_cmd))


    def cmd_vel_callback(self, cmd_vel_msg: TwistStamped):

        # *20 to make an array for each of the 20 wheels
        wheel_speeds = [self.wheel_command_sign * self.gear_ratio * cmd_vel_msg.twist.linear.x] * 20

        wheel_cmd_msg = Float64MultiArray()
        wheel_cmd_msg.data = wheel_speeds
        self.wheel_publisher.publish(wheel_cmd_msg)

        yaw_cmd_msg = Float64MultiArray()
       
        yaw = [self.yaw_command_from_normalized_steer(cmd_vel_msg.twist.angular.z)]
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
