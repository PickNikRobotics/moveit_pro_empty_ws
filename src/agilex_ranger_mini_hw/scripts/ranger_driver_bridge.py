#!/usr/bin/env python3
"""Bridge the Ranger Mini's topic-based ros2_control block to the AgileX driver.

Commands: the hardware block publishes planar joint velocities, world frame,
as a JointState. This node rotates them into a body twist for the driver.

State: the driver publishes wheel odometry. This node writes it back as the
planar joint states, with the yaw unwrapped so the continuous joint never jumps.
The steering and wheel joints report zero: the driver publishes no angles.

Gate: a command passes only while the driver's odometry and system state are
recent, the base reports normal CAN control, and planar_base_bridge.py
publishes. Otherwise the driver gets a zero twist. After any stop, motion
waits for a zero command before it may start again.

UNTESTED ON HARDWARE. See the hw package README before the first run.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ranger_msgs.msg import SystemState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

PLANAR_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")
STATE_JOINTS = tuple(f"{c}_steering_joint" for c in ("fl", "fr", "rl", "rr")) + tuple(
    f"{c}_wheel_joint" for c in ("fl", "fr", "rl", "rr")
)
# Far below any commanded speed, far above double rounding of a rotation.
ROUNDING_NOISE = 1e-9


def yaw_from_quaternion(z, w):
    """Heading of a planar orientation quaternion, rad."""
    return 2.0 * math.atan2(z, w)


def unwrap(previous, angle):
    """Return `angle` shifted by whole turns to lie within half a turn of `previous`."""
    if previous is None:
        return angle
    return previous + math.remainder(angle - previous, 2.0 * math.pi)


def commanded_planar_velocity(msg):
    """Read the planar joint velocity commands from the hardware block's JointState.

    @param msg The command JointState. Its velocity list holds the commanded
      joints only, in joint order, so entry i belongs to name i.
    @return The x, y and yaw rates, or None if they are not all present.
    """
    rates = dict(zip(msg.name, msg.velocity))
    if any(name not in rates for name in PLANAR_JOINTS):
        return None
    return tuple(rates[name] for name in PLANAR_JOINTS)


def system_state_fault(state, allowed_error_mask):
    """Say why the driver's reported state forbids motion.

    @param state A ranger_msgs/SystemState from the driver.
    @param allowed_error_mask Error code bits that do not forbid motion.
    @return A reason, or None when the base is normal, under CAN control and
      reports no other error bit.
    """
    if state.vehicle_state != SystemState.VEHICLE_STATE_NORMAL:
        return f"the driver reports vehicle state {state.vehicle_state}, not normal"
    if state.control_mode != SystemState.CONTROL_MODE_CAN:
        return f"the driver reports control mode {state.control_mode}, not CAN"
    faults = state.error_code & ~allowed_error_mask
    if faults:
        return f"the driver reports error code 0x{faults:04x}"
    return None


def is_stop(rates):
    """True when every commanded rate is finite and at rounding level."""
    return all(abs(v) <= ROUNDING_NOISE for v in rates)


def world_to_body(x_dot, y_dot, yaw_dot, yaw):
    """Rotate world-frame planar joint rates into a body-frame twist."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (x_dot * c + y_dot * s, -x_dot * s + y_dot * c, yaw_dot)


def driver_twist(rates, headings, max_linear, max_angular):
    """Turn world-frame planar joint rates into the twist the AgileX driver gets.

    @param rates The x, y and yaw joint rates.
    @param headings Base headings this node published, oldest first, rad.
    @param max_linear Ceiling of the planar speed, m/s.
    @param max_angular Ceiling of the turn rate, rad/s.
    @return The body vx, vy and wz. All zero if anything is not finite.
    @details Controllers build world rates from a heading this node published.
    Rotating back with that heading leaves only rounding noise in an unused
    axis, and that noise is set to exactly zero.

    The match is tried on every command. A world-frame command that lines up
    with a retained heading, such as a joint jog soon after the base first
    turns away from its boot heading, is read as built from that older heading.

    The driver drops the turn when vy is not exactly zero. A command with real
    forward and sideways parts matches no heading, is rotated with the newest
    heading and keeps both parts.
    """
    if not headings or not all(math.isfinite(v) for v in (*rates, *headings)):
        return (0.0, 0.0, 0.0)
    vx, vy, wz = world_to_body(*rates, headings[-1])
    for heading in reversed(headings):
        # A quarter turn swaps the axes and a half turn reverses the command.
        if abs(heading - headings[-1]) >= 0.5 * math.pi:
            break
        bx, by, _ = world_to_body(*rates, heading)
        if min(abs(bx), abs(by)) <= ROUNDING_NOISE:
            vx = 0.0 if abs(bx) <= ROUNDING_NOISE else bx
            vy = 0.0 if abs(by) <= ROUNDING_NOISE else by
            break
    speed = math.hypot(vx, vy)
    if speed > max_linear:
        scale = max_linear / speed
        vx, vy = vx * scale, vy * scale
    wz = max(-max_angular, min(max_angular, wz))
    out = (vx, vy, wz)
    return out if all(math.isfinite(v) for v in out) else (0.0, 0.0, 0.0)


def body_to_world(vx, vy, wz, yaw):
    """Rotate a body-frame twist into world-frame planar joint rates."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (vx * c - vy * s, vx * s + vy * c, wz)


class RangerDriverBridge(Node):
    """Joint commands in, driver twist out; driver odometry in, joint states out."""

    def __init__(self):
        super().__init__("ranger_driver_bridge")
        self.declare_parameter(
            "joint_commands_topic", "/agilex_ranger_mini/joint_commands"
        )
        self.declare_parameter("joint_states_topic", "/agilex_ranger_mini/joint_states")
        self.declare_parameter(
            "driver_cmd_vel_topic", "/agilex_ranger_mini/driver/cmd_vel"
        )
        self.declare_parameter("driver_odom_topic", "/agilex_ranger_mini/driver/odom")
        self.declare_parameter("system_state_topic", "/system_state")
        self.declare_parameter(
            "planar_command_topic", "/planar_velocity_controller/commands"
        )
        # Stop the base when the controller manager stops publishing.
        self.declare_parameter("command_timeout_sec", 0.25)
        # Refuse to drive on a frozen pose when the driver stops publishing odometry.
        self.declare_parameter("odom_timeout_sec", 0.5)
        # Refuse to drive unless the driver recently reported a base fit to move.
        self.declare_parameter("system_state_timeout_sec", 0.5)
        # Battery warning only; see AGENT_COMMENTS.md.
        self.declare_parameter("allowed_error_mask", 0x0002)
        # Refuse to drive when planar_base_bridge.py stops publishing.
        self.declare_parameter("planar_command_timeout_sec", 0.25)
        # The driver runs slower than ros2_control; do not flood its queue.
        self.declare_parameter("min_publish_period_sec", 0.02)
        # Operating limits of config/moveit/joint_limits.yaml; test_hw_config.py checks.
        self.declare_parameter("max_linear_velocity", math.hypot(0.5, 0.5))
        self.declare_parameter("max_angular_velocity", 1.0)
        # Published headings a command may have been built from; see AGENT_COMMENTS.md.
        self.declare_parameter("heading_history", 25)

        self._timeout_ns = int(1e9 * self.get_parameter("command_timeout_sec").value)
        self._odom_timeout_ns = int(1e9 * self.get_parameter("odom_timeout_sec").value)
        self._state_timeout_ns = int(
            1e9 * self.get_parameter("system_state_timeout_sec").value
        )
        self._planar_timeout_ns = int(
            1e9 * self.get_parameter("planar_command_timeout_sec").value
        )
        self._allowed_error_mask = self.get_parameter("allowed_error_mask").value
        self._period_ns = int(1e9 * self.get_parameter("min_publish_period_sec").value)
        self._limits = tuple(
            self.get_parameter(name).value
            for name in ("max_linear_velocity", "max_angular_velocity")
        )
        self._headings = deque(maxlen=self.get_parameter("heading_history").value)
        self._yaw = None
        self._last_command_ns = None
        self._last_odom_ns = None
        self._system_state = None
        self._last_state_ns = None
        self._last_planar_ns = None
        self._last_publish_ns = None
        self._last_twist = None
        # A held command must not restart the base; see AGENT_COMMENTS.md.
        self._needs_zero_command = True

        self._twist_pub = self.create_publisher(
            Twist, self.get_parameter("driver_cmd_vel_topic").value, 10
        )
        self._state_pub = self.create_publisher(
            JointState, self.get_parameter("joint_states_topic").value, 10
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_commands_topic").value,
            self._on_command,
            10,
        )
        self.create_subscription(
            Odometry, self.get_parameter("driver_odom_topic").value, self._on_odom, 10
        )
        self.create_subscription(
            SystemState,
            self.get_parameter("system_state_topic").value,
            self._on_system_state,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("planar_command_topic").value,
            self._on_planar_command,
            10,
        )
        self.create_timer(0.05, self._watchdog)

    def _on_command(self, msg):
        rates = commanded_planar_velocity(msg)
        if rates is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        self._last_command_ns = now_ns
        reason = self._motion_blocked(now_ns)
        if reason is not None:
            self._needs_zero_command = True
        elif self._needs_zero_command:
            if is_stop(rates):
                self._needs_zero_command = False
            else:
                reason = "waiting for a zero command after the last stop"
        if reason is not None:
            self.get_logger().warn(
                f"Holding the base still: {reason}.", throttle_duration_sec=2.0
            )
            self._send((0.0, 0.0, 0.0), now_ns)
            return
        self._send(driver_twist(rates, self._headings, *self._limits), now_ns)

    def _motion_blocked(self, now_ns):
        """Say why the base must not move now, or None when it may."""

        def stale(stamp_ns, timeout_ns):
            return stamp_ns is None or now_ns - stamp_ns > timeout_ns

        if stale(self._last_odom_ns, self._odom_timeout_ns):
            return "no recent driver odometry"
        if stale(self._last_state_ns, self._state_timeout_ns):
            return "no recent driver system state"
        fault = system_state_fault(self._system_state, self._allowed_error_mask)
        if fault is not None:
            return fault
        if stale(self._last_planar_ns, self._planar_timeout_ns):
            return "planar_base_bridge stopped publishing"
        return None

    def _on_system_state(self, msg):
        self._system_state = msg
        self._last_state_ns = self.get_clock().now().nanoseconds

    def _on_planar_command(self, _msg):
        self._last_planar_ns = self.get_clock().now().nanoseconds

    def _send(self, twist, now_ns):
        """Publish a twist, at most once per period unless it newly stops the base."""
        newly_stopped = twist == (0.0, 0.0, 0.0) and self._last_twist != twist
        recent = (
            self._last_publish_ns is not None
            and now_ns - self._last_publish_ns < self._period_ns
        )
        if recent and not newly_stopped:
            return
        self._last_publish_ns = now_ns
        self._last_twist = twist
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = twist
        self._twist_pub.publish(msg)

    def _on_odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn(
                "Dropped driver odometry with a non-finite value.",
                throttle_duration_sec=2.0,
            )
            return
        self._last_odom_ns = self.get_clock().now().nanoseconds
        self._yaw = unwrap(self._yaw, yaw_from_quaternion(q.z, q.w))
        self._headings.append(self._yaw)
        rates = body_to_world(t.linear.x, t.linear.y, t.angular.z, self._yaw)

        state = JointState()
        state.header.stamp = msg.header.stamp
        state.name = list(PLANAR_JOINTS + STATE_JOINTS)
        # Message arrays reject in-place list extension, so assign whole lists.
        state.position = [p.x, p.y, self._yaw] + [0.0] * len(STATE_JOINTS)
        state.velocity = list(rates) + [0.0] * len(STATE_JOINTS)
        self._state_pub.publish(state)

    def _watchdog(self):
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_command_ns is not None
            and now_ns - self._last_command_ns > self._timeout_ns
        ):
            self._twist_pub.publish(Twist())
            self._last_publish_ns, self._last_twist = now_ns, (0.0, 0.0, 0.0)
            self._last_command_ns = None
            self._needs_zero_command = True
            self.get_logger().warn(
                "Joint commands stopped; sent a zero twist to the driver."
            )


def main(args=None):
    rclpy.init(args=args)
    node = RangerDriverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
