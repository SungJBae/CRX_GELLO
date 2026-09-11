"""Drives a Robotiq 2F-140 from the GELLO trigger.

The trigger arrives normalised 0 (open) to 1 (closed). This node maps that onto
the GripperCommand action exposed by the Robotiq gripper controller.

The position units depend on which Robotiq stack you run. ros2_robotiq_gripper
commands the finger_joint in radians (roughly 0.0 open to 0.7 closed on a
2F-140); some forks command stroke in metres (0.140 to 0.0). Set
open_position and closed_position to match yours rather than assuming.

Goals are deadbanded and rate limited because the gripper controller will
happily accept goals faster than the hardware can act on them.
"""

from __future__ import annotations

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64


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

        self._client = ActionClient(self, GripperCommand, str(g("action_name").value))
        self.create_subscription(
            Float64, str(g("trigger_topic").value), self._on_trigger, 10
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

    def _on_trigger(self, msg: Float64) -> None:
        value = min(max(float(msg.data), 0.0), 1.0)
        now = self.get_clock().now().nanoseconds * 1e-9

        if self._goal_in_flight:
            return
        if now - self._last_time < self._min_period:
            return
        if self._last_sent is not None and abs(value - self._last_sent) < self._deadband:
            return
        if not self._client.server_is_ready():
            return

        goal = GripperCommand.Goal()
        goal.command.position = self._open + value * (self._closed - self._open)
        goal.command.max_effort = self._effort

        self._goal_in_flight = True
        self._last_sent = value
        self._last_time = now
        self._client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - never let a goal kill the node
            self.get_logger().warn(f"gripper goal failed: {exc}")
            self._goal_in_flight = False
            return
        if not handle.accepted:
            self.get_logger().warn("gripper goal rejected")
            self._goal_in_flight = False
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _future) -> None:
        self._goal_in_flight = False


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
