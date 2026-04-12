import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from mtt_msgs.msg import MttAuxCommand
import logging
import pyinotify
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import time
from .mtt_driver import (
    MTTCanDriver,
    WinchState,
    DirectionState,
    SecuritySwitchState,
    LightState
)

import threading



class MTTTeleopJoy(Node):
    """Translates /joy messages to /cmd_vel and /mtt_aux_cmd, including light toggle and winch safety."""

    def __init__(self):
        super().__init__("mtt_teleop_joy")


        self.maximmum_linear_speed = 0.1  # m/s
        self.maximum_angular_speed = 0.1  # rad/s
        self.declare_parameter("max_linear_speed", self.maximmum_linear_speed)
        self.declare_parameter("max_angular_speed", self.maximum_angular_speed) 
        self.declare_parameter("joy_timeout_seconds", 0.5)
        self.maximmum_linear_speed = self.get_parameter("max_linear_speed").value
        self.maximum_angular_speed = self.get_parameter("max_angular_speed").value 
        self.joy_timeout_seconds = float(self.get_parameter("joy_timeout_seconds").value)

 

        try:
            self.driver = MTTCanDriver("can0", log_level=logging.INFO, can_id=0x001)
        except Exception as e:
            raise

        self.driver_lock = threading.RLock()
        self.last_joy_message_time = None
        self.joy_timeout_active = False

        self.can_frame_timer = self.create_timer(0.05, self.send_can_frame)


        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel_raw", 10)
        self.aux_cmd_pub = self.create_publisher(MttAuxCommand, "mtt_aux_cmd", 10)

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.get_logger().info("MTT Teleop Node started.")

        self.mapping_registry = self.load_mapping_registry()

        self.axis_map = {"linear_speed": None, "rotation_speed": None, "brake": None, "winch": None}
        self.button_map = {"nav_safety": None, "light_toggle": None, "winch_safety": None}
        self.prev_light_btn = 0
        self.light_state = False

        self.is_initialized = False

        self.joystick_name = self._get_joystick_name("/dev/input/js0")
        self.get_logger().info(f"Initial joystick: {self.joystick_name}")
        self.set_joystick_mapper(self.joystick_name)

        # pyinotify setup
        self.wm = pyinotify.WatchManager()
        mask = pyinotify.IN_CREATE | pyinotify.IN_DELETE | pyinotify.IN_ATTRIB | pyinotify.IN_MOVED_TO | pyinotify.IN_MOVED_FROM

        # Add watch to /dev/input
        self.notifier = pyinotify.ThreadedNotifier(self.wm, self.event_handler)
        self.wm.add_watch("/dev/input", mask)
        self.notifier.start()

        pkg_share = get_package_share_directory("mtt_driver")
        config_dir = os.path.join(pkg_share, "config")
        self.wm.add_watch(config_dir, mask, rec=False)

    def send_can_frame(self):
        self._apply_joy_timeout_if_needed()
        with self.driver_lock:
            self.driver.send_can_frame()
            # self.get_logger().info(f"MHC")

    def _joy_is_fresh(self):
        if self.last_joy_message_time is None:
            return False
        if self.joy_timeout_seconds <= 0.0:
            return True
        return (time.monotonic() - self.last_joy_message_time) <= self.joy_timeout_seconds

    def _apply_joy_timeout_if_needed(self):
        if self._joy_is_fresh() or self.last_joy_message_time is None:
            return
        if self.joy_timeout_active:
            return

        with self.driver_lock:
            self.driver.set_throttle(0.0)
            self.driver.set_light_state(LightState.Off)
            self.joy_timeout_active = True

        self.get_logger().warning(
            f"No Joy message received for {self.joy_timeout_seconds:.2f}s; neutralizing direct teleop output"
        )


    def _get_joystick_name(self, js_dev="/dev/input/js0"):

        try:
            with open(f"/sys/class/input/{js_dev.split('/')[-1]}/device/name") as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"Could not read joystick name: {e}")
            return "Unknown"
        
    def set_joystick_mapper(self, received_string):
        for name, yaml_file in self.mapping_registry.items():
            if name in received_string:
                self.load_params_from_file(yaml_file)
                return
        self.get_logger().warn(f"No mapping found for joystick: {received_string}")

    def load_mapping_registry(self):
        pkg_share = get_package_share_directory("mtt_driver")
        mapping_path = os.path.join(pkg_share, "config", "joystick_mappings.yaml")
        try:
            with open(mapping_path, 'r') as f:
                data = yaml.safe_load(f)
            return data.get("joystick_mappings", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load joystick mapping registry: {e}")
            return {}
    
    def load_params_from_file(self, yaml_file: str):
        pkg_share = get_package_share_directory("mtt_driver")
        yaml_path = os.path.join(pkg_share, "config", yaml_file)
    
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        self.get_logger().debug(f"Loaded parameters from {yaml_file}")

        params = data["mtt_teleop_joy"]
        self.axis_map["linear_speed"] = params["axis"]["linear_speed"]
        self.axis_map["rotation_speed"] = params["axis"]["rotation_speed"]
        self.axis_map["brake"] = params["axis"]["brake"]
        self.axis_map["winch"] = params["axis"]["winch"]
        
        self.button_map["nav_safety"] = params["buttons"]["nav_safety"]
        self.button_map["winch_safety"] = params["buttons"]["winch_safety"]
        self.button_map["light_toggle"] = params["buttons"]["light_toggle"]

        self.is_initialized = True

    def _get_button(self, msg: Joy, name: str, default: bool = False):
        """Return a button as bool; safe if mapping is missing or out of range."""
        idx = self.button_map.get(name)
        if idx is None:
            return default
        try:
            return bool(msg.buttons[idx])
        except (IndexError, TypeError):
            return default

    def _get_axis(self, msg: Joy, name: str, default: float = 0.0):
        """Return an axis as float; safe if mapping is missing or out of range."""
        idx = self.axis_map.get(name)
        if idx is None:
            return default
        try:
            return float(msg.axes[idx])
        except (IndexError, TypeError, ValueError):
            return default

    def event_handler(self, event):
        basename = os.path.basename(event.pathname)

        # Check if the event concerns js0
        if basename == "js0":
            new_name = self._get_joystick_name()
            if new_name != self.joystick_name:
                self.is_initialized = False
                self.get_logger().info(f"Joystick changed: {self.joystick_name} → {new_name}")
                self.joystick_name = new_name
                self.set_joystick_mapper(new_name)

    def joy_callback(self, msg: Joy):

        if not self.is_initialized:
            return

        deadman_pressed = len(msg.buttons) > 5 and bool(msg.buttons[5])
        throttle_pressed = len(msg.buttons) > 0 and bool(msg.buttons[0])
        stop_pressed = len(msg.buttons) > 1 and bool(msg.buttons[1])

        with self.driver_lock:
            if deadman_pressed and throttle_pressed:
                self.driver.set_throttle(0.4)
                self.driver.set_light_state(LightState.On)
            elif deadman_pressed and stop_pressed:
                self.driver.set_throttle(0.0)
                self.driver.set_light_state(LightState.Off)
            else:
                self.driver.set_throttle(0.0)
                self.driver.set_light_state(LightState.Off)
            self.last_joy_message_time = time.monotonic()
            self.joy_timeout_active = False



        # tmtt_tools/mtt_driver/mtt_driver/mtt_teleop_joy.pywist_msg = Twist()
        # twist_msg.linear.x = self._get_axis(msg, "linear_speed")
        # twist_msg.angular.z = self._get_axis(msg, "rotation_speed")
        # # self.cmd_vel_pub.publish(twist_msg)

        # aux_msg = MttAuxCommand()
        # aux_msg.dead_man_switch = self._get_button(msg, "nav_safety")
        # aux_msg.brake = (-self._get_axis(msg, "brake") + 1 )/2
        
        # # Winch safety button
        # aux_msg.winch_safety_button = self._get_button(msg, "winch_safety")
         
        # # Winch command from D-pad
        # dpad_val = self._get_axis(msg, "winch")
        # if dpad_val > 0.5:
        #     aux_msg.winch_command = MttAuxCommand.WINCH_IN
        # elif dpad_val < -0.5:
        #     aux_msg.winch_command = MttAuxCommand.WINCH_OUT
        # else:
        #     aux_msg.winch_command = MttAuxCommand.WINCH_NEUTRAL

        # # Light toggle logic (rising edge detection)
        # light_btn = self._get_button(msg, "light_toggle")
        # if light_btn == 1 and self.prev_light_btn == 0:
        #     self.light_state = not self.light_state
        # self.prev_light_btn = light_btn

        # # Set light state if supported
        # if hasattr(aux_msg, "light_state"):
        #     aux_msg.light_state = self.light_state

        # self.aux_cmd_pub.publish(aux_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MTTTeleopJoy()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
