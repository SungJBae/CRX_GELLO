"""Drives a Robotiq 2F-140 from the GELLO trigger.

The trigger arrives normalised 0 (open) to 1 (closed). This node maps that onto
the GripperCommand action exposed by the Robotiq gripper controller.

The position units depend on which Robotiq stack you run. ros2_robotiq_gripper
commands the finger_joint in radians (roughly 0.0 open to 0.7 closed on a
2F-140); some forks command stroke in metres (0.140 to 0.0). Set
open_position and closed_position to match yours rather than assuming.

Goals are deadbanded and rate limited. When a significant trigger change
arrives while a goal is in flight, the old goal is cancelled (preempted) and
the new one is sent immediately so the gripper tracks the operator's intent
without waiting for the previous motion to finish.
"""

from __future__ import annotations

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64

# Match the leader's best-effort / volatile / depth-1 QoS so the trigger
# subscription connects without a QoS mismatch warning.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class GripperNode(Node):
    def __init__(self) -> None:
        super().__init__("gello_gripper")

        self.declare_parameter("action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("trigger_topic", "/gello_leader/trigger")
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("closed_position", 0.7)
        self.declare_parameter("max_effort", 40.0)
        self.declare_parameter("deadband", 0.02)
        self.declare_parameter("min_period_s", 0.1)

        g = self.get_parameter
        self._open = float(g("open_position").value)
        self._closed = float(g("closed_position").value)
        self._effort = float(g("max_effort").value)
        self._deadband = float(g("deadband").value)
        self._min_period = float(g("min_period_s").value)

        self._last_sent: float | None = None
        self._last_time = 0.0
        self._goal_in_flight = False
        self._active_handle = None
        self._pending_value: float | None = None

        self._client = ActionClient(self, GripperCommand, str(g("action_name").value))
        self.create_subscription(
            Float64, str(g("trigger_topic").value), self._on_trigger, _SENSOR_QOS,
        )
        self.get_logger().info(
            f"waiting for {g('action_name').value} ..."
        )
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "gripper action server not available; trigger input will be ignored"
            )
        else:
            self.get_logger().info("gripper action server connected")

    def _send_goal(self, value: float) -> None:
        """Send a grip goal and mark it in-flight."""
        goal = GripperCommand.Goal()
        goal.command.position = self._open + value * (self._closed - self._open)
        goal.command.max_effort = self._effort

        self._goal_in_flight = True
        self._last_sent = value
        self._last_time = self.get_clock().now().nanoseconds * 1e-9
        self._pending_value = None
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_trigger(self, msg: Float64) -> None:
        value = min(max(float(msg.data), 0.0), 1.0)
        now = self.get_clock().now().nanoseconds * 1e-9

        if now - self._last_time < self._min_period:
            return
        if self._last_sent is not None and abs(value - self._last_sent) < self._deadband:
            return
        if not self._client.server_is_ready():
            return

        if self._goal_in_flight:
            # Preempt: cancel the current goal and queue the new value.
            # The new goal is sent from _on_cancel_done once cancellation
            # is acknowledged, avoiding sending while still in flight.
            self._pending_value = value
            if self._active_handle is not None:
                self._active_handle.cancel_goal_async().add_done_callback(
                    self._on_cancel_done
                )
            return

        self._send_goal(value)

    def _on_cancel_done(self, _future) -> None:
        """Cancellation acknowledged; send the queued goal if one exists."""
        self._goal_in_flight = False
        self._active_handle = None
        if self._pending_value is not None:
            self._send_goal(self._pending_value)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - never let a goal kill the node
            self.get_logger().warn(f"gripper goal failed: {exc}")
            self._goal_in_flight = False
            self._active_handle = None
            return
        if not handle.accepted:
            self.get_logger().warn("gripper goal rejected")
            self._goal_in_flight = False
            self._active_handle = None
            return
        self._active_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future) -> None:
        self._goal_in_flight = False
        self._active_handle = None
        # If a trigger change arrived while the goal was finishing, send it now.
        if self._pending_value is not None:
            self._send_goal(self._pending_value)


def main() -> None:
    rclpy.init()
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
