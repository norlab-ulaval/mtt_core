import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from mtt_msgs.msg import MttAuxCommand
import pyinotify
import os
from ament_index_python.packages import get_package_share_directory
import yaml


class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle and winch safety."""

    def __init__(self):
        super().__init__("mtt_teleop_joy")


        self.maximmum_linear_speed = 0.1  # m/s
        self.maximum_angular_speed = 0.1  # rad/s
        self.declare_parameter("max_linear_speed", self.maximmum_linear_speed)
        self.declare_parameter("max_angular_speed", self.maximum_angular_speed) 
        self.maximmum_linear_speed = self.get_parameter("max_linear_speed").value
        self.maximum_angular_speed = self.get_parameter("max_angular_speed").value  


        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel_raw", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)
        self.estop_pub = self.create_publisher(Bool, "teleop_estop", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")


        self.axis_map = {"linear_speed": None, "rotation_speed": None, "brake": None, "winch": None}
        self.button_map = {"deadman": None, "light_toggle": None, "winch_safety": None}
        self.prev_light_btn = 0
        self.light_state = False
        self.safety_state = False

    def _button_pressed(self, msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def _axis_value(self, msg: Joy, index: int, default: float = 0.0) -> float:
        if 0 <= index < len(msg.axes):
            return float(msg.axes[index])
        return default

    def _compute_linear_command(self, axis_value: float) -> float:
        if axis_value < -0.05:
            return -0.3 + axis_value * 0.3
        if axis_value > 0.05:
            return 0.3 + axis_value * 0.3
        return 0.0

    def _compute_angular_command(self, axis_value: float) -> float:
        if axis_value < -0.05 or axis_value > 0.05:
            return axis_value
        return 0.0

    def joy_callback(self, msg: Joy):
        deadman_pressed = self._button_pressed(msg, 5)
        light_btn = self._button_pressed(msg, 2)
        linear_axis = self._axis_value(msg, 4)
        angular_axis = self._axis_value(msg, 3)
        brake_axis = self._axis_value(msg, 5, 1.0)

        twist_msg = TwistStamped()
        aux_msg = MttAuxCommand()
        estop_msg = Bool()
        estop_msg.data = not deadman_pressed
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        aux_msg.light_state = self.light_state

        if light_btn and self.prev_light_btn == 0:
            self.light_state = not self.light_state
        self.prev_light_btn = int(light_btn)
        aux_msg.light_state = self.light_state

        if deadman_pressed:
            twist_msg.twist.linear.x = self._compute_linear_command(linear_axis)
            twist_msg.twist.angular.z = self._compute_angular_command(angular_axis)
            aux_msg.brake = (-brake_axis + 1.0) / 2.0
        else:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            aux_msg.brake = 0.0

        self.estop_pub.publish(estop_msg)
        self.cmd_vel_pub.publish(twist_msg)
        self.aux_cmd_pub.publish(aux_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
