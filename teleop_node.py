"""GELLO -> CRX-10iA/L joint streaming.

The FANUC hardware interface forwards whatever it receives with no velocity
clamping, discontinuity check or delta limit of any kind. Every one of those
guards lives here.

Pipeline per control tick:
    leader state -> first-order low pass -> per-tick delta clamp against the
    URDF velocity limits -> absolute position clamp -> publish.

State machine:
    IDLE     no command published, controller holds its last setpoint
    SYNCING  robot is ramped from where it is to where the leader is
    ENGAGED  leader stream is followed

Enable is a heartbeat, not a latch. The node drops to IDLE if the enable
topic goes quiet, so killing the publisher or losing the terminal stops
motion without anyone having to send a false.

The SYNCING state exists because engaging while the leader and follower
disagree would otherwise command a step of arbitrary size on the first tick.
"""

from __future__ import annotations

from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

# From fanuc_description/crx10ia_l_urdf_macro.xacro
DEFAULT_VEL_LIMITS = [
    np.deg2rad(120.0),
    np.deg2rad(120.0),
    np.deg2rad(180.0),
    np.deg2rad(180.0),
    np.deg2rad(180.0),
    np.deg2rad(180.0),
]
DEFAULT_POS_LIMITS = [179.9, 179.9, 270.0, 190.0, 179.9, 225.0]


class State(Enum):
    IDLE = "IDLE"
    SYNCING = "SYNCING"
    ENGAGED = "ENGAGED"


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("gello_teleop")

        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("velocity_scale", 0.25)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("sync_tolerance_rad", 0.05)
        self.declare_parameter("sync_velocity_scale", 0.05)
        self.declare_parameter("leader_timeout_s", 0.2)
        self.declare_parameter("robot_timeout_s", 0.5)
        self.declare_parameter("enable_timeout_s", 0.5)
        self.declare_parameter("position_margin_deg", 5.0)
        self.declare_parameter("velocity_limits", DEFAULT_VEL_LIMITS)
        self.declare_parameter("position_limits_deg", DEFAULT_POS_LIMITS)
        self.declare_parameter("joint_prefix", "")
        self.declare_parameter("command_topic", "/forward_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("leader_topic", "/gello_leader/leader_states")

        g = self.get_parameter
        self._dt = 1.0 / float(g("control_rate").value)
        self._vel_scale = float(g("velocity_scale").value)
        self._sync_tol = float(g("sync_tolerance_rad").value)
        self._sync_scale = float(g("sync_velocity_scale").value)
        self._leader_timeout = float(g("leader_timeout_s").value)
        self._robot_timeout = float(g("robot_timeout_s").value)
        self._enable_timeout = float(g("enable_timeout_s").value)
        self._prefix = str(g("joint_prefix").value)

        self._vel_limits = np.asarray(g("velocity_limits").value, dtype=np.float64)
        margin = np.deg2rad(float(g("position_margin_deg").value))
        pos = np.deg2rad(np.asarray(g("position_limits_deg").value, dtype=np.float64))
        self._pos_limit = np.maximum(pos - margin, 0.0)

        if not 0.0 < self._vel_scale <= 1.0:
            raise ValueError("velocity_scale must be in (0, 1]")

        cutoff = float(g("filter_cutoff_hz").value)
        tau = 1.0 / (2.0 * np.pi * cutoff)
        self._alpha = self._dt / (tau + self._dt)

        self._state = State.IDLE
        self._enable_requested = False
        self._enable_stamp: float | None = None
        self._leader: np.ndarray | None = None
        self._leader_stamp: float | None = None
        self._robot: np.ndarray | None = None
        self._robot_stamp: float | None = None
        self._filtered: np.ndarray | None = None
        self._command: np.ndarray | None = None

        self._pub_cmd = self.create_publisher(
            Float64MultiArray, str(g("command_topic").value), 10
        )
        self._pub_state = self.create_publisher(String, "~/state", 10)
        self.create_subscription(
            JointState, str(g("leader_topic").value), self._on_leader, 10
        )
        self.create_subscription(
            JointState, str(g("joint_states_topic").value), self._on_robot, 10
        )
        self.create_subscription(Bool, "~/enable", self._on_enable, 10)

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"teleop ready: {1.0 / self._dt:.0f} Hz, velocity_scale="
            f"{self._vel_scale:.2f}, cutoff={cutoff:.1f} Hz. "
            f"Publish true to {self.get_name()}/enable at >"
            f"{1.0 / self._enable_timeout:.0f} Hz to engage; motion stops "
            f"{self._enable_timeout:.1f} s after the heartbeat drops."
        )

    # ---------------------------------------------------------------- inputs

    def _extract(self, msg: JointState) -> np.ndarray | None:
        """Reorder an incoming JointState into J1..J6, tolerating a prefix."""
        lookup = {name: i for i, name in enumerate(msg.name)}
        out = np.zeros(6, dtype=np.float64)
        for i, short in enumerate(JOINT_NAMES):
            idx = lookup.get(self._prefix + short, lookup.get(short))
            if idx is None or idx >= len(msg.position):
                return None
            out[i] = msg.position[idx]
        return out

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_leader(self, msg: JointState) -> None:
        values = self._extract(msg)
        if values is None:
            return
        self._leader = values
        self._leader_stamp = self._now()

    def _on_robot(self, msg: JointState) -> None:
        values = self._extract(msg)
        if values is None:
            return
        self._robot = values
        self._robot_stamp = self._now()

    def _on_enable(self, msg: Bool) -> None:
        if msg.data and not self._enable_requested:
            self.get_logger().info("enable requested")
        elif not msg.data and self._enable_requested:
            self.get_logger().info("disable requested")
        self._enable_requested = bool(msg.data)
        self._enable_stamp = self._now()

    # ----------------------------------------------------------- state logic

    def _stale(self, stamp: float | None, timeout: float) -> bool:
        return stamp is None or (self._now() - stamp) > timeout

    def _to_idle(self, reason: str) -> None:
        if self._state is not State.IDLE:
            self.get_logger().warn(f"-> IDLE ({reason})")
        self._state = State.IDLE
        self._filtered = None
        self._command = None

    def _tick(self) -> None:
        self._pub_state.publish(String(data=self._state.value))

        if self._stale(self._robot_stamp, self._robot_timeout):
            self._to_idle("no /joint_states")
            return
        if self._stale(self._leader_stamp, self._leader_timeout):
            self._to_idle("leader stream stalled")
            return
        if self._stale(self._enable_stamp, self._enable_timeout):
            self._to_idle("enable heartbeat lost")
            return
        if not self._enable_requested:
            self._to_idle("not enabled")
            return

        assert self._leader is not None and self._robot is not None
        target = np.clip(self._leader, -self._pos_limit, self._pos_limit)

        if self._state is State.IDLE:
            # Start the command where the robot actually is, never where the
            # leader is, so the first published value is a no-op.
            self._command = self._robot.copy()
            self._filtered = self._leader.copy()
            self._state = State.SYNCING
            err = float(np.max(np.abs(target - self._robot)))
            self.get_logger().info(
                f"-> SYNCING (max leader/robot error {np.rad2deg(err):.1f} deg)"
            )

        assert self._command is not None and self._filtered is not None

        if self._state is State.SYNCING:
            step = self._vel_limits * self._sync_scale * self._dt
            delta = np.clip(target - self._command, -step, step)
            self._command = self._command + delta
            self._filtered = self._leader.copy()
            if float(np.max(np.abs(target - self._command))) < self._sync_tol:
                self._state = State.ENGAGED
                self.get_logger().info("-> ENGAGED")
        else:
            self._filtered = (
                1.0 - self._alpha
            ) * self._filtered + self._alpha * self._leader
            desired = np.clip(self._filtered, -self._pos_limit, self._pos_limit)
            step = self._vel_limits * self._vel_scale * self._dt
            delta = np.clip(desired - self._command, -step, step)
            self._command = self._command + delta

        self._command = np.clip(self._command, -self._pos_limit, self._pos_limit)
        self._pub_cmd.publish(
            Float64MultiArray(data=[float(v) for v in self._command])
        )


def main() -> None:
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
