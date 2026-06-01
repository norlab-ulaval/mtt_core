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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


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
        # teleop_topic: the *filtered* manual cmd_vel — used only during pre-replay checks
        # (NOT used for override detection during replay — see joystick_topic below)
        self.declare_parameter("teleop_topic", "cmd_vel/manual")
        # joystick_topic: the *raw* joystick output (before the rate-limiter filter).
        # This is what we monitor for intentional operator intervention during replay.
        # cmd_vel/manual_raw has values only when the operator is actively pushing the stick.
        self.declare_parameter("joystick_topic", "cmd_vel/manual_raw")
        self.declare_parameter("deadman_topic", "mtt_control/teleop_deadman")
        self.declare_parameter("controller_cmd_vel_topic", "controller/cmd_vel")
        self.declare_parameter("selected_mode_topic", "mtt_control/selected_mode")
        self.declare_parameter("selected_source_topic", "mtt_control/selected_source")
        self.declare_parameter("auto_enabled_topic", "mtt_control/auto_mode_enabled")
        self.declare_parameter("state_topic", "mtt_repeat/state")
        self.declare_parameter("ready_topic", "mtt_repeat/ready")
        self.declare_parameter("wiln_command_topic", "/wiln/command")
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
        self._joystick_topic = str(self.get_parameter("joystick_topic").value)
        self._deadman_topic = str(self.get_parameter("deadman_topic").value)
        self._controller_cmd_vel_topic = str(self.get_parameter("controller_cmd_vel_topic").value)
        self._selected_mode_topic = str(self.get_parameter("selected_mode_topic").value)
        self._selected_source_topic = str(self.get_parameter("selected_source_topic").value)
        self._auto_enabled_topic = str(self.get_parameter("auto_enabled_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._ready_topic = str(self.get_parameter("ready_topic").value)
        self._wiln_command_topic = str(self.get_parameter("wiln_command_topic").value)
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
        # Raw joystick command (cmd_vel/manual_raw) — not rate-limited.
        # Used for intentional-override detection during replay.
        self._last_joystick_cmd: Optional[TwistStamped] = None
        self._last_joystick_received_time: Optional[float] = None
        self._deadman_active: Optional[bool] = None
        self._deadman_received_time: Optional[float] = None
        self._last_controller_cmd: Optional[TwistStamped] = None
        self._last_controller_motion_time: Optional[float] = None
        self._selected_mode: str = "stop"
        self._selected_source: str = "unknown"
        self._auto_enabled = False
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
        # Subscribe to raw joystick output (before rate-limiter) for override detection
        self._joystick_sub = self.create_subscription(
            TwistStamped, self._joystick_topic, self._on_joystick_cmd, 20
        )
        self._deadman_sub = self.create_subscription(
            Bool, self._deadman_topic, self._on_deadman, 20
        )
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._controller_sub = self.create_subscription(
            TwistStamped, self._controller_cmd_vel_topic, self._on_controller_cmd, best_effort_qos
        )
        self._selected_mode_sub = self.create_subscription(
            String, self._selected_mode_topic, self._on_selected_mode, 20
        )
        self._selected_source_sub = self.create_subscription(
            String, self._selected_source_topic, self._on_selected_source, 20
        )
        self._auto_enabled_sub = self.create_subscription(
            Bool, self._auto_enabled_topic, self._on_auto_enabled, 20
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(String, self._state_topic, latched_qos)
        self._ready_pub = self.create_publisher(Bool, self._ready_topic, latched_qos)
        self._wiln_command_pub = self.create_publisher(String, self._wiln_command_topic, 10)
        self._request_auto_client = self.create_client(
            Trigger, self._request_auto_service, callback_group=self._client_group
        )
        self._request_manual_client = self.create_client(
            Trigger, self._request_manual_service, callback_group=self._client_group
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
            Trigger,
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
            f"(health={self._health_topic}, icp={self._icp_odom_topic}, "
            f"teleop={self._teleop_topic}, joystick_raw={self._joystick_topic}, "
            f"deadman={self._deadman_topic}, "
            f"mode={self._selected_mode_topic}, source={self._selected_source_topic}, "
            f"wiln_command={self._wiln_command_topic})"
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

    def _on_joystick_cmd(self, msg: TwistStamped) -> None:
        """Raw joystick output — published only when deadman is held and stick is moved."""
        with self._state_lock:
            self._last_joystick_cmd = msg
            self._last_joystick_received_time = self._now_seconds()

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

    def _on_selected_source(self, msg: String) -> None:
        with self._state_lock:
            self._selected_source = str(msg.data).strip()

    def _on_auto_enabled(self, msg: Bool) -> None:
        with self._state_lock:
            self._auto_enabled = bool(msg.data)

    def _joystick_override_active(self) -> bool:
        """True only when the operator is *intentionally* pushing the joystick.

        Uses cmd_vel/manual_raw (before the rate-limiter) so we are immune to
        the filter's ramp-down residuals that keep flowing after the stick is
        released.  We also require the deadman to be freshly active so a stale
        raw message cannot latch a false override.
        """
        with self._state_lock:
            joy_msg = self._last_joystick_cmd
            joy_time = self._last_joystick_received_time
            deadman_active = self._deadman_active
            deadman_time = self._deadman_received_time
        now_s = self._now_seconds()
        # Raw joystick message must be fresh (operator node publishes only when
        # deadman is active, so >0.3 s stale means deadman was released).
        if joy_msg is None or joy_time is None:
            return False
        if now_s - joy_time > 0.3:
            return False
        # Deadman must also be confirmed active and fresh.
        if deadman_active is None or deadman_time is None:
            return False
        if now_s - deadman_time > 0.5:
            return False
        if not deadman_active:
            return False
        # Non-zero stick movement above threshold = intentional override.
        return (
            abs(joy_msg.twist.linear.x) > self._teleop_override_linear_threshold
            or abs(joy_msg.twist.angular.z) > self._teleop_override_angular_threshold
        )

    def _teleop_override_active(self) -> bool:
        """Used for pre-replay checks (not during replay — see _joystick_override_active)."""
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
        return self._wiln_command_pub.get_subscription_count() > 0

    def _control_snapshot(self) -> Tuple[str, str, bool, bool]:
        with self._state_lock:
            return (
                self._selected_mode,
                self._selected_source,
                self._auto_enabled,
                self._deadman_active is True,
            )

    def _debug_suffix(self) -> str:
        mode, source, auto_enabled, deadman = self._control_snapshot()
        wiln_subs = self._wiln_command_pub.get_subscription_count()
        return (
            f"mode={mode} auto_enabled={auto_enabled} source={source} "
            f"deadman={deadman} wiln_command_subs={wiln_subs}"
        )

    def _repeat_ready(self) -> Tuple[bool, str]:
        if not self._trajectory_ready:
            return False, f"trajectory not armed; {self._debug_suffix()}"
        if not self._services_ready():
            return False, f"WILN command subscribers unavailable; is wiln container running? {self._debug_suffix()}"
        icp_ok, _ = self._icp_fresh()
        if not icp_ok:
            return False, f"ICP odom stale or absent; {self._debug_suffix()}"
        health_ok, health = self._health_fresh()
        if not health_ok or health is None:
            return False, f"mtt_health stale or absent; {self._debug_suffix()}"
        if not health.security_unlocked:
            return False, f"driver safety locked; unlock robot safety/e-stop; {self._debug_suffix()}"
        if health.emergency_stop_active:
            return False, f"emergency stop active; release e-stop; {self._debug_suffix()}"
        if self._teleop_override_active():
            return False, f"teleop override currently active; release joystick/deadman; {self._debug_suffix()}"
        return True, f"ready; {self._debug_suffix()}"

    def _publish_wiln_command(self, command: str) -> Tuple[bool, str]:
        if self._wiln_command_pub.get_subscription_count() <= 0:
            return False, "no WILN command subscribers"
        msg = String()
        msg.data = command
        self._wiln_command_pub.publish(msg)
        self.get_logger().info(f"WILN command published: {command}")
        return True, f"published {command}"

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
        ok, detail = self._publish_wiln_command("cancel")
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
        ok, detail = self._publish_wiln_command("start_recording")
        if ok:
            with self._state_lock:
                self._recording = True
                self._replaying = False
                self._trajectory_ready = False
            self._set_state(RepeatState.TEACHING, f"recording route; {self._debug_suffix()}")
            response.success = True
            response.message = f"teach started; {self._debug_suffix()}"
        else:
            self._set_state(RepeatState.FAULTED, f"teach_start failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_teach_stop(self, _, response: Trigger.Response) -> Trigger.Response:
        ok, detail = self._publish_wiln_command("stop_recording")
        if ok:
            self._call_trigger_client(self._request_manual_client)
            with self._state_lock:
                self._recording = False
                self._trajectory_ready = True
            self._set_state(RepeatState.READY, f"trajectory recorded; {self._debug_suffix()}")
            response.success = True
            response.message = f"teach stopped; trajectory armed; {self._debug_suffix()}"
        else:
            self._set_state(RepeatState.FAULTED, f"teach_stop failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_mark_ready(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            self._trajectory_ready = True
            self._recording = False
        self._set_state(RepeatState.READY, f"trajectory loaded; {self._debug_suffix()}")
        response.success = True
        response.message = f"trajectory marked ready; {self._debug_suffix()}"
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
            self._set_state(RepeatState.FAULTED, f"replay refused: could not enter AUTO ({auto_detail}); press A or check mtt_mode_manager; {self._debug_suffix()}")
            response.success = False
            response.message = f"could not enter AUTO: {auto_detail}; {self._debug_suffix()}"
            return response
        ok, detail = self._publish_wiln_command("play")
        if ok:
            self._start_replay_tracking("line replay active")
            response.success = True
            response.message = f"line replay started; {self._debug_suffix()}"
        else:
            self._call_trigger_client(self._request_manual_client)
            self._set_state(RepeatState.FAULTED, f"play_line failed: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_play_loop(self, _, response: Trigger.Response) -> Trigger.Response:
        ready, reason = self._repeat_ready()
        if not ready:
            self._set_state(RepeatState.FAULTED, f"loop replay refused: {reason}")
            response.success = False
            response.message = reason
            return response
        auto_ok, auto_detail = self._call_trigger_client(self._request_auto_client)
        if not auto_ok:
            self._set_state(RepeatState.FAULTED, f"loop replay refused: {auto_detail}")
            response.success = False
            response.message = auto_detail
            return response
        ok, detail = self._publish_wiln_command("play")
        if ok:
            self._start_replay_tracking("single replay active (WILN topic API has no native loop command)")
            response.success = True
            response.message = "single replay started"
        else:
            self._call_trigger_client(self._request_manual_client)
            self._set_state(RepeatState.FAULTED, f"play_loop failed: {detail}")
            response.success = False
            response.message = detail
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
            # ----------------------------------------------------------------
            # Priority-1: intentional joystick override (raw stick, deadman
            # confirmed active).  This is the SAFETY path that the operator
            # uses to take back control.  We check the *raw* joystick topic
            # (cmd_vel/manual_raw) rather than the filtered cmd_vel/manual so
            # that rate-limiter ramp-down residuals cannot trigger a false
            # cancel after the stick is released.
            # ----------------------------------------------------------------
            if self._joystick_override_active():
                self.get_logger().warn(
                    "Joystick override detected during replay — cancelling and returning to manual."
                )
                self._cancel_replay("joystick override by operator")
            else:
                with self._state_lock:
                    selected_mode = self._selected_mode
                # --------------------------------------------------------
                # Priority-2: mode changed away from AUTO.
                # The mode_manager can flip to Manual because of
                # manual_activity (even tiny stick drift while holding
                # the deadman).  During an active replay WE requested
                # AUTO via service, so if mode flipped to manual without
                # a real joystick override we silently re-request AUTO
                # instead of cancelling.  A real operator override is
                # caught above (priority-1) before we ever reach here.
                # --------------------------------------------------------
                if selected_mode != "auto":
                    self.get_logger().info(
                        f"Mode drifted to '{selected_mode}' during replay — "
                        "re-requesting AUTO (no joystick override confirmed)."
                    )
                    auto_ok, auto_detail = self._call_trigger_client(
                        self._request_auto_client, timeout_s=1.0
                    )
                    if not auto_ok:
                        # Could not recover auto — genuine problem, cancel.
                        self._cancel_replay(
                            f"could not recover AUTO after mode drift ({auto_detail})"
                        )
                        ready, ready_reason = self._repeat_ready()
                        with self._state_lock:
                            state = self._state
                            replaying = self._replaying
                            replay_started_time = self._replay_started_time
                            controller_motion_seen = self._controller_motion_seen
                            last_controller_motion_time = self._last_controller_motion_time
                            recording = self._recording
                    # else: auto recovered silently, continue replay
                else:
                    # ------------------------------------------------
                    # Priority-3: safety conditions
                    # ------------------------------------------------
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
                            self._cancel_replay(
                                "replay started but controller produced no motion command"
                            )
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
