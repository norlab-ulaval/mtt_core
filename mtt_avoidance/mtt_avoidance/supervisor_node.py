"""Behavior supervisor for hold, bypass planning, execution, and recovery."""

from __future__ import annotations

import math
import threading

from mtt_interfaces.msg import AvoidanceStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from wiln.msg import WilnState

from .policy import AvoidancePolicy, Inputs, State


class AvoidanceSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("mtt_avoidance_supervisor")
        self.declare_parameter("enabled", False)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("execution_enabled", False)
        self.declare_parameter("reverse_recovery_enabled", False)
        self.declare_parameter("persistent_stop_s", 3.0)
        self.declare_parameter("obstacle_timeout_s", 0.5)
        self.declare_parameter("decision_rate_hz", 10.0)
        self.declare_parameter("replay_state_topic", "/wiln/replay/state")
        self.declare_parameter("obstacle_stop_topic", "/mtt_obstacle/stop_requested")
        self.declare_parameter("slowdown_topic", "/mtt_obstacle/slowdown_scale")
        self.declare_parameter("planner_status_topic", "/mtt_avoidance/planner_status")
        self.declare_parameter("plan_valid_topic", "/mtt_avoidance/plan_valid")
        self.declare_parameter("plan_clearance_topic", "/mtt_avoidance/plan_clearance_m")
        self.declare_parameter("executor_status_topic", "/mtt_avoidance/executor_status")
        self.declare_parameter("recovery_status_topic", "/mtt_avoidance/recovery_status")

        execution = bool(self.get_parameter("execution_enabled").value)
        shadow = bool(self.get_parameter("shadow_mode").value)
        if shadow and execution:
            self.get_logger().warning(
                "shadow_mode=true forces execution_enabled=false regardless of YAML"
            )
            execution = False
        self._policy = AvoidancePolicy(
            persistent_stop_s=float(self.get_parameter("persistent_stop_s").value),
            execution_enabled=execution,
            reverse_recovery_enabled=bool(
                self.get_parameter("reverse_recovery_enabled").value
            ),
        )
        self._execution_enabled = execution
        self._shadow_mode = shadow
        self._replaying = False
        self._obstacle_stop = False
        self._slowdown = 1.0
        self._obstacle_stamp = None
        self._plan_ready = False
        self._plan_failed = False
        self._plan_clearance = math.nan
        self._planner_detail = "idle"
        self._execution_finished = False
        self._recovery_finished = False
        self._lock = threading.Lock()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            WilnState,
            str(self.get_parameter("replay_state_topic").value),
            self._on_replay,
            latched,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("obstacle_stop_topic").value),
            self._on_obstacle,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("slowdown_topic").value),
            self._on_slowdown,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("planner_status_topic").value),
            self._on_planner_status,
            latched,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("plan_valid_topic").value),
            self._on_plan_valid,
            latched,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("plan_clearance_topic").value),
            self._on_clearance,
            latched,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("executor_status_topic").value),
            self._on_executor_status,
            latched,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("recovery_status_topic").value),
            self._on_recovery_status,
            latched,
        )
        self._status_pub = self.create_publisher(
            AvoidanceStatus, "/mtt_avoidance/status", latched
        )
        self._replan_pub = self.create_publisher(
            Bool, "/mtt_avoidance/replan_request", 10
        )
        self._execute_pub = self.create_publisher(
            Bool, "/mtt_avoidance/execute_request", 10
        )
        self._recovery_pub = self.create_publisher(
            Bool, "/mtt_avoidance/recovery_request", 10
        )
        self._override_pub = self.create_publisher(
            Bool, "/mtt_avoidance/override_requested", latched
        )
        rate = max(1.0, float(self.get_parameter("decision_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "Avoidance supervisor: "
            f"enabled={bool(self.get_parameter('enabled').value)} "
            f"shadow={self._shadow_mode} execution={self._execution_enabled} "
            "reverse="
            f"{bool(self.get_parameter('reverse_recovery_enabled').value)}"
        )

    def _on_replay(self, msg: WilnState) -> None:
        with self._lock:
            self._replaying = msg.state == WilnState.PLAYING

    def _on_obstacle(self, msg: Bool) -> None:
        with self._lock:
            self._obstacle_stop = bool(msg.data)
            self._obstacle_stamp = self.get_clock().now()

    def _on_slowdown(self, msg: Float32) -> None:
        with self._lock:
            self._slowdown = max(0.0, min(1.0, float(msg.data)))
            self._obstacle_stamp = self.get_clock().now()

    def _on_planner_status(self, msg: String) -> None:
        with self._lock:
            self._planner_detail = msg.data
            self._plan_failed = msg.data.startswith("failed:")

    def _on_plan_valid(self, msg: Bool) -> None:
        with self._lock:
            self._plan_ready = bool(msg.data)
            if msg.data:
                self._plan_failed = False

    def _on_clearance(self, msg: Float32) -> None:
        with self._lock:
            self._plan_clearance = float(msg.data)

    def _on_executor_status(self, msg: String) -> None:
        with self._lock:
            self._execution_finished = msg.data.startswith(("succeeded", "failed", "canceled"))

    def _on_recovery_status(self, msg: String) -> None:
        with self._lock:
            self._recovery_finished = msg.data.startswith(("succeeded", "failed", "refused"))

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _tick(self) -> None:
        now = self.get_clock().now()
        with self._lock:
            replaying = self._replaying
            obstacle = self._obstacle_stop
            stamp = self._obstacle_stamp
            plan_ready = self._plan_ready
            plan_failed = self._plan_failed
            plan_clearance = self._plan_clearance
            planner_detail = self._planner_detail
            execution_finished = self._execution_finished
            recovery_finished = self._recovery_finished
        obstacle_age = math.inf if stamp is None else (now - stamp).nanoseconds * 1e-9
        fresh = obstacle_age <= float(self.get_parameter("obstacle_timeout_s").value)
        decision = self._policy.update(
            Inputs(
                now_s=now.nanoseconds * 1e-9,
                enabled=bool(self.get_parameter("enabled").value),
                replaying=replaying,
                obstacle_fresh=fresh,
                obstacle_stop=obstacle,
                plan_ready=plan_ready,
                plan_failed=plan_failed,
                execution_finished=execution_finished,
                recovery_finished=recovery_finished,
            )
        )
        if decision.request_plan:
            with self._lock:
                self._plan_ready = False
                self._plan_failed = False
                self._execution_finished = False
                self._recovery_finished = False
            self._publish_bool(self._replan_pub, True)
        if decision.request_execute:
            self._publish_bool(self._execute_pub, True)
        if decision.request_recovery:
            self._publish_bool(self._recovery_pub, True)
        if decision.cancel_motion:
            self._publish_bool(self._execute_pub, False)
            self._publish_bool(self._recovery_pub, False)

        override = decision.state in (State.EXECUTING, State.RECOVERING)
        self._publish_bool(self._override_pub, override)
        status = AvoidanceStatus()
        status.header.stamp = now.to_msg()
        status.header.frame_id = "map"
        status.state = int(decision.state)
        status.shadow_mode = self._shadow_mode
        status.execution_enabled = self._execution_enabled
        status.obstacle_stop = obstacle
        status.plan_available = plan_ready
        status.plan_valid = plan_ready
        status.obstacle_age_s = float(obstacle_age if math.isfinite(obstacle_age) else -1.0)
        status.min_clearance_m = float(
            plan_clearance if math.isfinite(plan_clearance) else -1.0
        )
        status.detail = planner_detail
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
