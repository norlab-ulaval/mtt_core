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
from std_srvs.srv import Empty, Trigger

try:
    from wiln.msg import WilnState as WilnStateMsg
except ImportError:  # pragma: no cover - absent on non-ROS test hosts
    WilnStateMsg = None


class RepeatState(str, Enum):
    IDLE = "idle"
    TEACHING = "teaching"
    READY = "ready_to_replay"
    ARMED = "armed"          # play published, waiting for A press + deadman
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
        self.declare_parameter("replay_state_topic", "/wiln/replay/state")
        self.declare_parameter("follower_state_topic", "/wiln/follower/state")
        self.declare_parameter("replay_ack_timeout_s", 3.0)
        self.declare_parameter("request_auto_service", "mtt_control/request_auto")
        self.declare_parameter("request_manual_service", "mtt_control/request_manual")
        self.declare_parameter("enable_mapping_service", "/mapping/enable_mapping")
        self.declare_parameter("disable_mapping_service", "/mapping/disable_mapping")
        self.declare_parameter("mapping_service_timeout_s", 5.0)
        self.declare_parameter("health_timeout_s", 1.0)
        self.declare_parameter("icp_timeout_s", 0.75)
        self.declare_parameter("teleop_override_linear_threshold", 0.05)
        self.declare_parameter("teleop_override_angular_threshold", 0.05)
        self.declare_parameter("replay_idle_timeout_s", 1.0)
        self.declare_parameter("replay_startup_timeout_s", 5.0)
        self.declare_parameter("armed_timeout_s", 300.0)   # max wait in ARMED state (5 min)
        self.declare_parameter("monitor_rate_hz", 10.0)
        self.declare_parameter("debug", False)  # enable verbose [SUP] logs

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
        self._replay_ack_timeout_s = float(self.get_parameter("replay_ack_timeout_s").value)
        self._request_auto_service = str(self.get_parameter("request_auto_service").value)
        self._request_manual_service = str(self.get_parameter("request_manual_service").value)
        self._mapping_service_timeout_s = float(
            self.get_parameter("mapping_service_timeout_s").value
        )
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
        self._armed_timeout_s = float(self.get_parameter("armed_timeout_s").value)
        self._monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        self._debug = bool(self.get_parameter("debug").value)
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
        self._awaiting_start = False   # armed: play published, waiting for A + deadman
        self._armed_since: Optional[float] = None
        self._recording = False
        self._replay_started_time: Optional[float] = None
        self._controller_motion_seen = False
        self._replay_ack_event = threading.Event()
        self._follower_ack_event = threading.Event()
        self._awaiting_replay_ack = False
        self._awaiting_follower_ack = False
        self._replay_ack_state = 0
        self._replay_ack_detail = ""
        self._follower_ack_state = 0
        self._follower_ack_detail = ""

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
        if WilnStateMsg is not None:
            replay_state_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._replay_state_sub = self.create_subscription(
                WilnStateMsg,
                str(self.get_parameter("replay_state_topic").value),
                self._on_replay_state,
                replay_state_qos,
            )
            self._follower_state_sub = self.create_subscription(
                WilnStateMsg,
                str(self.get_parameter("follower_state_topic").value),
                self._on_follower_state,
                replay_state_qos,
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
        self._enable_mapping_client = self.create_client(
            Empty,
            str(self.get_parameter("enable_mapping_service").value),
            callback_group=self._client_group,
        )
        self._disable_mapping_client = self.create_client(
            Empty,
            str(self.get_parameter("disable_mapping_service").value),
            callback_group=self._client_group,
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
        self._clear_trajectory_srv = self.create_service(
            Trigger,
            "mtt_repeat/clear_trajectory",
            self._handle_clear_trajectory,
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

    def _on_replay_state(self, msg) -> None:
        if WilnStateMsg is None:
            return
        detail = str(msg.detail).strip()
        accepted = msg.state == WilnStateMsg.PLAYING
        refused = msg.state == WilnStateMsg.IDLE and detail.startswith("replay refused")
        if not accepted and not refused:
            return
        with self._state_lock:
            if not self._awaiting_replay_ack:
                return
            self._replay_ack_state = int(msg.state)
            self._replay_ack_detail = detail
            self._awaiting_replay_ack = False
        self._replay_ack_event.set()

    def _on_follower_state(self, msg) -> None:
        if WilnStateMsg is None:
            return
        detail = str(msg.detail).strip()
        accepted = msg.state == WilnStateMsg.PLAYING
        refused = msg.state == WilnStateMsg.IDLE and detail.startswith("play refused")
        if not accepted and not refused:
            return
        with self._state_lock:
            if not self._awaiting_follower_ack:
                return
            self._follower_ack_state = int(msg.state)
            self._follower_ack_detail = detail
            self._awaiting_follower_ack = False
        self._follower_ack_event.set()

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

    def _publish_play_and_wait_for_ack(self) -> Tuple[bool, str]:
        """Publish play and wait for the replay node's authoritative decision.

        Publishing on /wiln/command only proves that DDS accepted a message.  It
        does not prove that the replay node accepted the route, selected a valid
        endpoint, or froze mapping.  The old implementation returned success at
        that point, while wiln_replay_node could simultaneously publish
        ``replay refused``; PathFollower then centered articulation and held zero.
        """
        if WilnStateMsg is None:
            return False, "wiln/State message unavailable; cannot verify replay acceptance"

        self._replay_ack_event.clear()
        self._follower_ack_event.clear()
        with self._state_lock:
            self._awaiting_replay_ack = True
            self._awaiting_follower_ack = True
            self._replay_ack_state = -1
            self._replay_ack_detail = ""
            self._follower_ack_state = -1
            self._follower_ack_detail = ""

        ok, detail = self._publish_wiln_command("play")
        if not ok:
            with self._state_lock:
                self._awaiting_replay_ack = False
                self._awaiting_follower_ack = False
            return False, detail

        deadline = time.monotonic() + self._replay_ack_timeout_s
        replay_ready = self._replay_ack_event.wait(max(0.0, deadline - time.monotonic()))
        follower_ready = self._follower_ack_event.wait(max(0.0, deadline - time.monotonic()))
        if not replay_ready or not follower_ready:
            with self._state_lock:
                self._awaiting_replay_ack = False
                self._awaiting_follower_ack = False
            # The distributed state is unknown. Cancel both replay and follower
            # so a delayed acknowledgement cannot start motion after the service
            # already reported failure.
            self._publish_wiln_command("cancel")
            return False, (
                "replay acknowledgement timeout after "
                f"{self._replay_ack_timeout_s:.1f}s; cancel published"
            )

        with self._state_lock:
            ack_state = self._replay_ack_state
            ack_detail = self._replay_ack_detail
            follower_state = self._follower_ack_state
            follower_detail = self._follower_ack_detail
        if ack_state != WilnStateMsg.PLAYING:
            return False, ack_detail or "replay refused by wiln_replay_node"
        if follower_state != WilnStateMsg.PLAYING:
            self._publish_wiln_command("cancel")
            return False, follower_detail or "replay refused by wiln_path_follower"
        return True, (
            f"{ack_detail or 'replay accepted; mapping frozen'}; "
            f"{follower_detail or 'follower accepted'}"
        )

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

    def _call_empty_client(self, client, timeout_s: float) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=0.0):
            return False, "service unavailable"
        future = client.call_async(Empty.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout_s):
            return False, "service timeout"
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - runtime safety
            return False, str(exc)
        return (result is not None), "ok" if result is not None else "empty response"

    def _start_replay_tracking(self, mode: str) -> None:
        now_s = self._now_seconds()
        with self._state_lock:
            self._replaying = True
            self._recording = False
            self._replay_started_time = now_s
            self._controller_motion_seen = False
            self._last_controller_motion_time = None
        self._set_state(RepeatState.REPLAYING, mode)

    def _cancel_replay(self, reason: str, request_manual: bool = True) -> None:
        """Cancel the active replay.

        Args:
            reason: Human-readable cancel reason (logged and published to state).
            request_manual: If True (default), explicitly switch mode to MANUAL after
                cancel. Set to False for safety stops where the mode cascade would
                cause a request_auto ↔ request_manual ping-pong (e.g., ICP stale).
        """
        ok, detail = self._publish_wiln_command("cancel")
        with self._state_lock:
            self._replaying = False
        next_state = RepeatState.PAUSED if ok else RepeatState.FAULTED
        suffix = reason if ok else f"{reason}; cancel failed: {detail}"
        if request_manual:
            manual_ok, manual_detail = self._call_trigger_client(
                self._request_manual_client, timeout_s=2.0
            )
            if not manual_ok:
                suffix = f"{suffix}; manual mode request failed: {manual_detail}"
        else:
            if self._debug:
                self.get_logger().info(
                    f"[SUP] _cancel_replay({reason}): skipping request_manual to avoid mode cascade."
                )
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
        mapping_ok, mapping_detail = self._call_empty_client(
            self._enable_mapping_client, self._mapping_service_timeout_s
        )
        if not mapping_ok:
            self._set_state(RepeatState.FAULTED, f"teach_start: enable mapping failed: {mapping_detail}")
            response.success = False
            response.message = f"enable mapping failed: {mapping_detail}"
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
            mapping_ok, mapping_detail = self._call_empty_client(
                self._disable_mapping_client, self._mapping_service_timeout_s
            )
            self._call_trigger_client(self._request_manual_client)
            with self._state_lock:
                self._recording = False
                self._trajectory_ready = True
            if mapping_ok:
                self._set_state(
                    RepeatState.READY,
                    f"trajectory recorded; map frozen for return to start; {self._debug_suffix()}",
                )
                response.success = True
                response.message = (
                    f"teach stopped; trajectory armed; map frozen; {self._debug_suffix()}"
                )
            else:
                self._set_state(
                    RepeatState.FAULTED,
                    f"trajectory recorded but map freeze failed: {mapping_detail}",
                )
                response.success = False
                response.message = (
                    f"trajectory recorded, but disable mapping failed: {mapping_detail}; "
                    "do not drive back until mapping is frozen"
                )
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

    def _handle_clear_trajectory(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._recording or self._replaying or self._awaiting_start:
                response.success = False
                response.message = "clear refused while Teach/Replay is active; cancel or stop first"
                return response
        ok, detail = self._publish_wiln_command("clear_trajectory")
        if not ok:
            response.success = False
            response.message = detail
            return response
        with self._state_lock:
            self._trajectory_ready = False
            self._recording = False
            self._replaying = False
            self._awaiting_start = False
            self._armed_since = None
        self._set_state(RepeatState.IDLE, "trajectory cleared by operator")
        response.success = True
        response.message = "active WILN trajectory cleared; saved routes were preserved"
        return response

    def _handle_play_line(self, _, response: Trigger.Response) -> Trigger.Response:
        ready, reason = self._repeat_ready()
        if not ready:
            response.success = False
            response.message = reason
            if self._state != RepeatState.TEACHING:
                self._set_state(RepeatState.FAULTED, f"replay refused: {reason}")
            return response
        # Do NOT request_auto here — the operator must press A and hold deadman.
        # Publishing "play" arms the PathFollower in PLAYING state; cmd_vel is
        # blocked by the arbiter until mode=AUTO (A button) and by PathFollower's
        # deadman gate until deadman is held.
        ok, detail = self._publish_play_and_wait_for_ack()
        if ok:
            now_s = self._now_seconds()
            with self._state_lock:
                self._replaying = False
                self._awaiting_start = True
                self._armed_since = now_s
                self._recording = False
                self._controller_motion_seen = False
                self._last_controller_motion_time = None
                self._replay_started_time = None
            self._set_state(RepeatState.ARMED, "trajectory armed — press A then hold deadman to start")
            response.success = True
            response.message = "armed; press A and hold deadman to start"
        else:
            self._set_state(RepeatState.READY, f"play_line refused: {detail}")
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
        # Same arm-then-start flow as play_line
        ok, detail = self._publish_play_and_wait_for_ack()
        if ok:
            now_s = self._now_seconds()
            with self._state_lock:
                self._replaying = False
                self._awaiting_start = True
                self._armed_since = now_s
                self._recording = False
                self._controller_motion_seen = False
                self._last_controller_motion_time = None
                self._replay_started_time = None
            self._set_state(RepeatState.ARMED, "trajectory armed — press A then hold deadman to start")
            response.success = True
            response.message = "armed; press A and hold deadman to start"
        else:
            self._call_trigger_client(self._request_manual_client)
            self._set_state(RepeatState.READY, f"play_loop refused: {detail}")
            response.success = False
            response.message = detail
        return response

    def _handle_cancel(self, _, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            self._awaiting_start = False
            self._armed_since = None
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
            awaiting_start = self._awaiting_start
            armed_since = self._armed_since
            replay_started_time = self._replay_started_time
            controller_motion_seen = self._controller_motion_seen
            last_controller_motion_time = self._last_controller_motion_time
            recording = self._recording

        # ----------------------------------------------------------------
        # ARMED state: trajectory published to PathFollower, waiting for
        # operator to press A (mode=AUTO) and hold deadman before motion starts.
        # ----------------------------------------------------------------
        if awaiting_start and not replaying:
            now_s = self._now_seconds()
            # Safety abort in armed state (robot still on ground, be cautious)
            health_ok, health = self._health_fresh()
            icp_ok, _ = self._icp_fresh()
            if not health_ok or health is None:
                self.get_logger().error("[SUP] ARMED: health stale — cancelling arm.")
                self._publish_wiln_command("cancel")
                with self._state_lock:
                    self._awaiting_start = False
                    self._armed_since = None
                self._set_state(RepeatState.READY, "arm cancelled: health stale")
            elif health.emergency_stop_active:
                self.get_logger().error("[SUP] ARMED: e-stop active — cancelling arm.")
                self._publish_wiln_command("cancel")
                with self._state_lock:
                    self._awaiting_start = False
                    self._armed_since = None
                self._set_state(RepeatState.READY, "arm cancelled: emergency stop active")
            elif armed_since is not None and now_s - armed_since > self._armed_timeout_s:
                self.get_logger().warn(
                    f"[SUP] ARMED timeout ({self._armed_timeout_s:.0f}s) — disarming."
                )
                self._publish_wiln_command("cancel")
                with self._state_lock:
                    self._awaiting_start = False
                    self._armed_since = None
                self._set_state(RepeatState.READY, "arm timeout — disarmed")
            else:
                # Check if operator has pressed A + is holding deadman → start
                _, _, _, deadman_held = self._control_snapshot()
                with self._state_lock:
                    selected_mode = self._selected_mode
                if selected_mode == "auto" and deadman_held:
                    self.get_logger().info(
                        "[SUP] A pressed + deadman held — starting motion."
                    )
                    now_s = self._now_seconds()
                    with self._state_lock:
                        self._awaiting_start = False
                        self._replaying = True
                        self._replay_started_time = now_s
                        self._controller_motion_seen = False
                        self._last_controller_motion_time = None
                    self._set_state(RepeatState.REPLAYING, "motion started")
                elif self._debug:
                    self.get_logger().info(
                        f"[SUP] ARMED: waiting for A+deadman "
                        f"(mode='{selected_mode}', deadman={deadman_held}) "
                        f"armed for {now_s - (armed_since or now_s):.0f}s"
                    )

        elif replaying:
            # ----------------------------------------------------------------
            # REPLAYING state
            # Priority-1: intentional joystick override → hard cancel
            # Priority-2: deadman released → pause (back to ARMED)
            # Priority-3: safety conditions
            # ----------------------------------------------------------------
            if self._joystick_override_active():
                self.get_logger().warn(
                    "Joystick override detected during replay — cancelling and returning to manual."
                )
                self._cancel_replay("joystick override by operator")
            else:
                _, _, _, deadman_held = self._control_snapshot()
                if not deadman_held:
                    # Deadman released — pause without cancelling the trajectory.
                    # PathFollower's deadman gate already outputs zero cmd_vel.
                    # Go back to ARMED so user can resume by pressing A + deadman.
                    now_s = self._now_seconds()
                    with self._state_lock:
                        self._replaying = False
                        self._awaiting_start = True
                        self._armed_since = now_s
                    self.get_logger().info("[SUP] Deadman released — paused (re-press A + deadman to resume).")
                    self._set_state(RepeatState.ARMED, "paused — press A and hold deadman to resume")
                else:
                    with self._state_lock:
                        selected_mode = self._selected_mode
                    # Priority-2: mode drifted away from AUTO (unexpected — deadman held)
                    if selected_mode != "auto":
                        self.get_logger().info(
                            f"Mode drifted to '{selected_mode}' during replay — "
                            "re-requesting AUTO (deadman held, not an operator override)."
                        )
                        auto_ok, auto_detail = self._call_trigger_client(
                            self._request_auto_client, timeout_s=1.0
                        )
                        if not auto_ok:
                            self._cancel_replay(
                                f"could not recover AUTO after mode drift ({auto_detail})"
                            )
                    else:
                        # Priority-3: safety conditions
                        health_ok, health = self._health_fresh()
                        icp_ok, _ = self._icp_fresh()
                        if not health_ok or health is None:
                            self._cancel_replay("mtt_health stale or absent")
                        elif not icp_ok:
                            self._cancel_replay("ICP odom stale or absent", request_manual=False)
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
                                self._set_state(RepeatState.READY, "replay completed")

        with self._state_lock:
            state = self._state
            reason = self._state_reason
            awaiting_start = self._awaiting_start
        if recording and state != RepeatState.TEACHING:
            state = RepeatState.TEACHING
        elif awaiting_start:
            # Always show ARMED when trajectory is armed and waiting for operator
            state = RepeatState.ARMED
        elif not replaying and state == RepeatState.IDLE and self._trajectory_ready:
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
