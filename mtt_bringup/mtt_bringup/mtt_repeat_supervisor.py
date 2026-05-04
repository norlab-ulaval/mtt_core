#!/usr/bin/env python3

import threading
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from mtt_msgs.msg import MttHealthState
from nav_msgs.msg import Odometry
from norlab_controllers_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from wiln.srv import PlayLoop


class RepeatState(str, Enum):
    IDLE = "idle"
    TEACHING = "teaching"
    READY = "ready_to_replay"
    REPLAYING = "replaying"
    PAUSED = "paused_or_cancelled"
    FAULTED = "faulted"


class MttRepeatSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_repeat_supervisor")

        self.declare_parameter("health_topic", "/mtt_health")
        self.declare_parameter("icp_odom_topic", "/mapping/icp_odom")
        self.declare_parameter("teleop_topic", "cmd_vel/manual")
        self.declare_parameter("deadman_topic", "teleop_deadman")
        self.declare_parameter("controller_cmd_vel_topic", "controller/cmd_vel")
        self.declare_parameter("selected_mode_topic", "selected_mode")
        self.declare_parameter("state_topic", "mtt_repeat/state")
        self.declare_parameter("ready_topic", "mtt_repeat/ready")
        self.declare_parameter("follow_path_action_name", "/follow_path")
        self.declare_parameter("teach_start_service", "/start_recording")
        self.declare_parameter("teach_stop_service", "/stop_recording")
        self.declare_parameter("play_line_service", "/play_line")
        self.declare_parameter("play_loop_service", "/play_loop")
        self.declare_parameter("cancel_service", "/cancel_trajectory")
        self.declare_parameter("request_auto_service", "mtt_control/request_auto")
        self.declare_parameter("request_manual_service", "mtt_control/request_manual")
        self.declare_parameter("health_timeout_s", 1.0)
        self.declare_parameter("icp_timeout_s", 0.75)
        self.declare_parameter("teleop_override_linear_threshold", 0.05)
        self.declare_parameter("teleop_override_angular_threshold", 0.05)
        self.declare_parameter("replay_idle_timeout_s", 1.0)
        self.declare_parameter("replay_startup_timeout_s", 5.0)
        self.declare_parameter("monitor_rate_hz", 10.0)

        self._health_topic = str(self.get_parameter("health_topic").value)
        self._icp_odom_topic = str(self.get_parameter("icp_odom_topic").value)
        self._teleop_topic = str(self.get_parameter("teleop_topic").value)
        self._deadman_topic = str(self.get_parameter("deadman_topic").value)
        self._controller_cmd_vel_topic = str(self.get_parameter("controller_cmd_vel_topic").value)
        self._selected_mode_topic = str(self.get_parameter("selected_mode_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._ready_topic = str(self.get_parameter("ready_topic").value)
        self._follow_path_action_name = str(self.get_parameter("follow_path_action_name").value)
        self._request_auto_service = str(self.get_parameter("request_auto_service").value)
        self._request_manual_service = str(self.get_parameter("request_manual_service").value)
        self._health_timeout_s = float(self.get_parameter("health_timeout_s").value)
        self._icp_timeout_s = float(self.get_parameter("icp_timeout_s").value)
        self._teleop_override_linear_threshold = float(
            self.get_parameter("teleop_override_linear_threshold").value
        )
        self._teleop_override_angular_threshold = float(
            self.get_parameter("teleop_override_angular_threshold").value
        )
        self._replay_idle_timeout_s = float(self.get_parameter("replay_idle_timeout_s").value)
        self._replay_startup_timeout_s = float(self.get_parameter("replay_startup_timeout_s").value)
        self._monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        self._state_lock = threading.Lock()
        self._health_msg: Optional[MttHealthState] = None
        self._health_received_time: Optional[float] = None
        self._icp_odom_msg: Optional[Odometry] = None
        self._icp_received_time: Optional[float] = None
        self._last_teleop_cmd: Optional[TwistStamped] = None
        self._last_teleop_received_time: Optional[float] = None
        self._deadman_active: Optional[bool] = None
        self._deadman_received_time: Optional[float] = None
        self._last_controller_cmd: Optional[TwistStamped] = None
        self._last_controller_motion_time: Optional[float] = None
        self._selected_mode: str = "stop"
        self._state = RepeatState.IDLE
        self._state_reason = "waiting"
        self._trajectory_ready = False
        self._replaying = False
        self._recording = False
        self._replay_started_time: Optional[float] = None
        self._controller_motion_seen = False

        self._health_sub = self.create_subscription(
            MttHealthState, self._health_topic, self._on_health, 20
        )
        self._icp_sub = self.create_subscription(
            Odometry, self._icp_odom_topic, self._on_icp_odom, 20
        )
        self._teleop_sub = self.create_subscription(
            TwistStamped, self._teleop_topic, self._on_teleop_cmd, 20
        )
        self._deadman_sub = self.create_subscription(
            Bool, self._deadman_topic, self._on_deadman, 20
        )
        self._controller_sub = self.create_subscription(
            TwistStamped, self._controller_cmd_vel_topic, self._on_controller_cmd, 20
        )
        self._selected_mode_sub = self.create_subscription(
            String, self._selected_mode_topic, self._on_selected_mode, 20
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(String, self._state_topic, latched_qos)
        self._ready_pub = self.create_publisher(Bool, self._ready_topic, latched_qos)

        teach_start_name = str(self.get_parameter("teach_start_service").value)
        teach_stop_name = str(self.get_parameter("teach_stop_service").value)
        play_line_name = str(self.get_parameter("play_line_service").value)
        play_loop_name = str(self.get_parameter("play_loop_service").value)
        cancel_name = str(self.get_parameter("cancel_service").value)

        self._teach_start_client = self.create_client(
            Empty, teach_start_name, callback_group=self._client_group
        )
        self._teach_stop_client = self.create_client(
            Empty, teach_stop_name, callback_group=self._client_group
        )
        self._play_line_client = self.create_client(
            Empty, play_line_name, callback_group=self._client_group
        )
        self._play_loop_client = self.create_client(
            PlayLoop, play_loop_name, callback_group=self._client_group
        )
        self._cancel_client = self.create_client(
            Empty, cancel_name, callback_group=self._client_group
        )
        self._request_auto_client = self.create_client(
            Trigger, self._request_auto_service, callback_group=self._client_group
        )
        self._request_manual_client = self.create_client(
            Trigger, self._request_manual_service, callback_group=self._client_group
        )
        self._follow_path_client = ActionClient(
            self, FollowPath, self._follow_path_action_name, callback_group=self._client_group
        )

        self._teach_start_srv = self.create_service(
            Trigger,
            "mtt_repeat/teach_start",
            self._handle_teach_start,
            callback_group=self._service_group,
        )
        self._teach_stop_srv = self.create_service(
            Trigger,
            "mtt_repeat/teach_stop",
            self._handle_teach_stop,
            callback_group=self._service_group,
        )
        self._play_line_srv = self.create_service(
            Trigger,
            "mtt_repeat/play_line",
            self._handle_play_line,
            callback_group=self._service_group,
        )
        self._play_loop_srv = self.create_service(
            PlayLoop,
            "mtt_repeat/play_loop",
            self._handle_play_loop,
            callback_group=self._service_group,
        )
        self._cancel_srv = self.create_service(
            Trigger,
            "mtt_repeat/cancel",
            self._handle_cancel,
            callback_group=self._service_group,
        )
        self._mark_ready_srv = self.create_service(
            Trigger,
            "mtt_repeat/mark_ready",
            self._handle_mark_ready,
            callback_group=self._service_group,
        )
        self._mark_idle_srv = self.create_service(
            Trigger,
            "mtt_repeat/mark_idle",
            self._handle_mark_idle,
            callback_group=self._service_group,
        )

        self._timer = self.create_timer(1.0 / max(self._monitor_rate_hz, 1.0), self._monitor)

        self.get_logger().info(
            "MTT repeat supervisor ready "
            f"(health={self._health_topic}, icp={self._icp_odom_topic}, teleop={self._teleop_topic}, "
            f"deadman={self._deadman_topic}, "
            f"follow_path={self._follow_path_action_name})"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_to_seconds(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _set_state(self, state: RepeatState, reason: str) -> None:
        with self._state_lock:
            changed = state != self._state or reason != self._state_reason
            self._state = state
            self._state_reason = reason
        if changed:
            self.get_logger().info(f"repeat state -> {state.value}: {reason}")

    def _on_health(self, msg: MttHealthState) -> None:
        with self._state_lock:
            self._health_msg = msg
            self._health_received_time = self._now_seconds()

    def _on_icp_odom(self, msg: Odometry) -> None:
        with self._state_lock:
            self._icp_odom_msg = msg
            self._icp_received_time = self._now_seconds()

    def _on_teleop_cmd(self, msg: TwistStamped) -> None:
        with self._state_lock:
            self._last_teleop_cmd = msg
            self._last_teleop_received_time = self._now_seconds()

    def _on_deadman(self, msg: Bool) -> None:
        with self._state_lock:
            self._deadman_active = bool(msg.data)
            self._deadman_received_time = self._now_seconds()

    def _on_controller_cmd(self, msg: TwistStamped) -> None:
        now_s = self._now_seconds()
        with self._state_lock:
            self._last_controller_cmd = msg
            if (
                abs(msg.twist.linear.x) > self._teleop_override_linear_threshold
                or abs(msg.twist.angular.z) > self._teleop_override_angular_threshold
            ):
                self._last_controller_motion_time = now_s
                self._controller_motion_seen = True

    def _on_selected_mode(self, msg: String) -> None:
        with self._state_lock:
            self._selected_mode = str(msg.data).strip().lower()

    def _teleop_override_active(self) -> bool:
        with self._state_lock:
            msg = self._last_teleop_cmd
            received_time = self._last_teleop_received_time
            deadman_active = self._deadman_active
            deadman_received_time = self._deadman_received_time
        if msg is None or received_time is None:
            return False
        if self._now_seconds() - received_time > 0.5:
            return False
        if deadman_active is not None and deadman_received_time is not None:
            if self._now_seconds() - deadman_received_time > 0.5:
                return False
            if not deadman_active:
                return False
        return (
            abs(msg.twist.linear.x) > self._teleop_override_linear_threshold
            or abs(msg.twist.angular.z) > self._teleop_override_angular_threshold
        )

    def _health_fresh(self) -> Tuple[bool, Optional[MttHealthState]]:
        with self._state_lock:
            msg = self._health_msg
            received_time = self._health_received_time
        if msg is None or received_time is None:
            return False, msg
        return (self._now_seconds() - received_time) <= self._health_timeout_s, msg

    def _icp_fresh(self) -> Tuple[bool, Optional[Odometry]]:
        with self._state_lock:
            msg = self._icp_odom_msg
            received_time = self._icp_received_time
        if msg is None or received_time is None:
            return False, msg
        header_time = self._stamp_to_seconds(msg.header.stamp)
        now_s = self._now_seconds()
        if header_time > 0.0:
            return (now_s - header_time) <= self._icp_timeout_s, msg
        return (now_s - received_time) <= self._icp_timeout_s, msg

    def _services_ready(self) -> bool:
        return (
            self._teach_start_client.wait_for_service(timeout_sec=0.0)
            and self._teach_stop_client.wait_for_service(timeout_sec=0.0)
            and self._play_line_client.wait_for_service(timeout_sec=0.0)
            and self._cancel_client.wait_for_service(timeout_sec=0.0)
            and self._play_loop_client.wait_for_service(timeout_sec=0.0)
        )

    def _follow_path_ready(self) -> bool:
        return self._follow_path_client.wait_for_server(timeout_sec=0.0)

    def _repeat_ready(self) -> Tuple[bool, str]:
        if not self._trajectory_ready:
            return False, "trajectory not armed"
        if not self._services_ready():
            return False, "WILN services unavailable"
        if not self._follow_path_ready():
            return False, "follow_path action unavailable"
        icp_ok, _ = self._icp_fresh()
        if not icp_ok:
            return False, "ICP odom stale or absent"
        health_ok, health = self._health_fresh()
        if not health_ok or health is None:
            return False, "mtt_health stale or absent"
        if not health.security_unlocked:
            return False, "driver safety locked"
        if health.emergency_stop_active:
            return False, "emergency stop active"
        if self._teleop_override_active():
            return False, "teleop override currently active"
        return True, "ready"

    def _call_empty_client(self, client, timeout_s: float = 2.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.0):
            return False, "service unavailable"
        future = client.call_async(Empty.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "service timeout"
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        return True, "ok"

    def _call_trigger_client(self, client, timeout_s: float = 2.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.0):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "service timeout"
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        if result is None:
            return False, "empty response"
        if not result.success:
            return False, result.message or "request rejected"
        return True, result.message or "ok"

    def _call_play_loop_client(self, loops: int, timeout_s: float = 2.0) -> Tuple[bool, str]:
        if not self._play_loop_client.wait_for_service(timeout_sec=0.0):
            return False, "service unavailable"
        req = PlayLoop.Request()
        req.nb_loops.data = max(1, int(loops))
        future = self._play_loop_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "service timeout"
        try:
            future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        return True, "ok"

    def _start_replay_tracking(self, mode: str) -> None:
        now_s = self._now_seconds()
        with self._state_lock:
            self._replaying = True
            self._recording = False
            self._replay_started_time = now_s
            self._controller_motion_seen = False
            self._last_controller_motion_time = None
        self._set_state(RepeatState.REPLAYING, mode)

    def _cancel_replay(self, reason: str) -> None:
        ok, detail = self._call_empty_client(self._cancel_client, timeout_s=2.0)
        manual_ok, manual_detail = self._call_trigger_client(self._request_manual_client, timeout_s=2.0)
        with self._state_lock:
            self._replaying = False
        next_state = RepeatState.PAUSED if ok else RepeatState.FAULTED
        suffix = reason if ok else f"{reason}; cancel failed: {detail}"
        if not manual_ok:
            suffix = f"{suffix}; manual mode request failed: {manual_detail}"
        self._set_state(next_state, suffix)

    def _handle_teach_start(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._replaying:
                response.success = False
                response.message = "cannot start teaching while replaying"
                return response
        manual_ok, manual_detail = self._call_trigger_client(self._request_manual_client)
        if not manual_ok:
            self._set_state(RepeatState.FAULTED, f"teach_start refused: {manual_detail}")
            response.success = False
            response.message = manual_detail
            return response
        ok, detail = self._call_empty_client(self._teach_start_client)
        if ok:
            with self._state_lock:
                self._recording = True
                self._replaying = False
                self._trajectory_ready = False
            self._set_state(RepeatState.TEACHING, "recording route")
            response.success = True
            response.message = "teach started"
        else:
            self._set_state(RepeatState.FAULTED, f"teach_start failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_teach_stop(self, _, response: Trigger.Response) -> Trigger.Response:
        ok, detail = self._call_empty_client(self._teach_stop_client)
        if ok:
            self._call_trigger_client(self._request_manual_client)
            with self._state_lock:
                self._recording = False
                self._trajectory_ready = True
            self._set_state(RepeatState.READY, "trajectory recorded")
            response.success = True
            response.message = "teach stopped; trajectory armed"
        else:
            self._set_state(RepeatState.FAULTED, f"teach_stop failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_mark_ready(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            self._trajectory_ready = True
            self._recording = False
        self._set_state(RepeatState.READY, "trajectory loaded")
        response.success = True
        response.message = "trajectory marked ready"
        return response

    def _handle_mark_idle(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            self._trajectory_ready = False
            self._recording = False
            self._replaying = False
        self._set_state(RepeatState.IDLE, "trajectory cleared")
        response.success = True
        response.message = "repeat state reset to idle"
        return response

    def _handle_play_line(self, _, response: Trigger.Response) -> Trigger.Response:
        ready, reason = self._repeat_ready()
        if not ready:
            response.success = False
            response.message = reason
            if self._state != RepeatState.TEACHING:
                self._set_state(RepeatState.FAULTED, f"replay refused: {reason}")
            return response
        auto_ok, auto_detail = self._call_trigger_client(self._request_auto_client)
        if not auto_ok:
            self._set_state(RepeatState.FAULTED, f"replay refused: {auto_detail}")
            response.success = False
            response.message = auto_detail
            return response
        ok, detail = self._call_empty_client(self._play_line_client)
        if ok:
            self._start_replay_tracking("line replay active")
            response.success = True
            response.message = "line replay started"
        else:
            self._call_trigger_client(self._request_manual_client)
            self._set_state(RepeatState.FAULTED, f"play_line failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_play_loop(self, request: PlayLoop.Request, response: PlayLoop.Response) -> PlayLoop.Response:
        ready, reason = self._repeat_ready()
        if not ready:
            self._set_state(RepeatState.FAULTED, f"loop replay refused: {reason}")
            return response
        auto_ok, auto_detail = self._call_trigger_client(self._request_auto_client)
        if not auto_ok:
            self._set_state(RepeatState.FAULTED, f"loop replay refused: {auto_detail}")
            return response
        ok, detail = self._call_play_loop_client(int(request.nb_loops.data))
        if ok:
            self._start_replay_tracking(f"loop replay active ({int(request.nb_loops.data)} loops)")
        else:
            self._call_trigger_client(self._request_manual_client)
            self._set_state(RepeatState.FAULTED, f"play_loop failed: {detail}")
        return response

    def _handle_cancel(self, _, response: Trigger.Response) -> Trigger.Response:
        self._cancel_replay("cancelled by operator")
        response.success = True
        response.message = "cancel requested"
        return response

    def _publish_state(self, ready: bool, state: RepeatState, reason: str) -> None:
        state_msg = String()
        state_msg.data = f"{state.value}: {reason}"
        ready_msg = Bool()
        ready_msg.data = ready
        self._state_pub.publish(state_msg)
        self._ready_pub.publish(ready_msg)

    def _monitor(self) -> None:
        ready, ready_reason = self._repeat_ready()
        with self._state_lock:
            state = self._state
            replaying = self._replaying
            replay_started_time = self._replay_started_time
            controller_motion_seen = self._controller_motion_seen
            last_controller_motion_time = self._last_controller_motion_time
            recording = self._recording

        if replaying:
            if self._teleop_override_active():
                self._cancel_replay("teleop override detected")
            else:
                with self._state_lock:
                    selected_mode = self._selected_mode
                if selected_mode != "auto":
                    self._cancel_replay(f"control mode changed to {selected_mode}")
                    ready, ready_reason = self._repeat_ready()
                    with self._state_lock:
                        state = self._state
                        replaying = self._replaying
                        replay_started_time = self._replay_started_time
                        controller_motion_seen = self._controller_motion_seen
                        last_controller_motion_time = self._last_controller_motion_time
                        recording = self._recording
                else:
                    health_ok, health = self._health_fresh()
                    icp_ok, _ = self._icp_fresh()
                    if not health_ok or health is None:
                        self._cancel_replay("mtt_health stale or absent")
                    elif not icp_ok:
                        self._cancel_replay("ICP odom stale or absent")
                    elif not health.security_unlocked:
                        self._cancel_replay("driver safety locked")
                    elif health.emergency_stop_active:
                        self._cancel_replay("emergency stop active")
                    else:
                        now_s = self._now_seconds()
                        if (
                            replay_started_time is not None
                            and now_s - replay_started_time > self._replay_startup_timeout_s
                            and not controller_motion_seen
                        ):
                            self._cancel_replay("replay started but controller produced no motion command")
                        elif (
                            controller_motion_seen
                            and last_controller_motion_time is not None
                            and now_s - last_controller_motion_time > self._replay_idle_timeout_s
                        ):
                            with self._state_lock:
                                self._replaying = False
                            self._set_state(RepeatState.READY, "replay completed or idle")

        with self._state_lock:
            state = self._state
            reason = self._state_reason
        if recording and state != RepeatState.TEACHING:
            state = RepeatState.TEACHING
        if not replaying and state == RepeatState.IDLE and self._trajectory_ready:
            state = RepeatState.READY
            reason = "trajectory armed"
        if state == RepeatState.READY and not ready and ready_reason != "trajectory not armed":
            reason = ready_reason
        self._publish_state(ready, state, reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MttRepeatSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
