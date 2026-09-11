"""Publishes GELLO leader state.

Applies the GELLO transfer function
    theta_robot_i = sign_i * (theta_raw_i - offset_i)
and normalises the trigger servo to [0, 1]. No filtering or limiting happens
here; that is the teleop node's job.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from gello_crx.dynamixel_bus import (
    RAD_PER_COUNT,
    DynamixelBus,
    DynamixelBusError,
)

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


class LeaderNode(Node):
    def __init__(self) -> None:
        super().__init__("gello_leader")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("joint_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("trigger_id", 7)
        self.declare_parameter("joint_offsets", [0.0] * 6)
        self.declare_parameter("joint_signs", [1.0] * 6)
        self.declare_parameter("trigger_open_counts", 2048)
        self.declare_parameter("trigger_closed_counts", 2900)
        self.declare_parameter("rate", 100.0)

        g = self.get_parameter
        port = g("port").value
        baud = int(g("baudrate").value)
        self._joint_ids = [int(i) for i in g("joint_ids").value]
        self._trigger_id = int(g("trigger_id").value)
        self._offsets = np.asarray(g("joint_offsets").value, dtype=np.float64)
        self._signs = np.asarray(g("joint_signs").value, dtype=np.float64)
        self._trig_open = float(g("trigger_open_counts").value)
        self._trig_closed = float(g("trigger_closed_counts").value)
        rate = float(g("rate").value)

        if len(self._joint_ids) != 6:
            raise ValueError("joint_ids must have exactly 6 entries")
        if self._offsets.shape != (6,) or self._signs.shape != (6,):
            raise ValueError("joint_offsets and joint_signs must have 6 entries")
        if not np.all(np.abs(self._signs) == 1.0):
            raise ValueError(f"joint_signs must all be +1 or -1, got {self._signs}")
        if self._trig_open == self._trig_closed:
            raise ValueError("trigger open and closed counts must differ")

        ids = self._joint_ids + [self._trigger_id]
        self._bus = DynamixelBus(port, ids, baud)
        self.get_logger().info(f"leader bus open on {port} @ {baud}, ids {ids}")

        self._pub_joints = self.create_publisher(JointState, "~/leader_states", 10)
        self._pub_trigger = self.create_publisher(Float64, "~/trigger", 10)
        self._consecutive_errors = 0
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self) -> None:
        try:
            counts = self._bus.read_counts()
        except DynamixelBusError as exc:
            self._consecutive_errors += 1
            # Stay quiet about the occasional dropped packet; shout if it persists.
            if self._consecutive_errors in (5, 50) or self._consecutive_errors % 500 == 0:
                self.get_logger().error(
                    f"{self._consecutive_errors} consecutive bus errors: {exc}"
                )
            return
        self._consecutive_errors = 0

        raw = counts[:6].astype(np.float64) * RAD_PER_COUNT
        mapped = self._signs * (raw - self._offsets)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in mapped]
        self._pub_joints.publish(msg)

        span = self._trig_closed - self._trig_open
        norm = (float(counts[6]) - self._trig_open) / span
        self._pub_trigger.publish(Float64(data=float(np.clip(norm, 0.0, 1.0))))

    def destroy_node(self) -> bool:
        self._bus.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
