#!/usr/bin/env python3
"""Bridge the Ranger Mini's topic-based ros2_control block to the AgileX driver.

Commands: the hardware block publishes planar joint velocities, world frame,
as a JointState. This node rotates them into a body twist for the driver.

State: the driver publishes wheel odometry. This node writes it back as the
planar joint states, with the yaw unwrapped so the continuous joint never jumps.
The steering and wheel joints report zero: the driver publishes no angles.

UNTESTED ON HARDWARE. See the hw package README before the first run.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

PLANAR_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")
STATE_JOINTS = tuple(f"{c}_steering_joint" for c in ("fl", "fr", "rl", "rr")) + tuple(
    f"{c}_wheel_joint" for c in ("fl", "fr", "rl", "rr")
)


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


def world_to_body(x_dot, y_dot, yaw_dot, yaw):
    """Rotate world-frame planar joint rates into a body-frame twist."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (x_dot * c + y_dot * s, -x_dot * s + y_dot * c, yaw_dot)


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
        # Stop the base when the controller manager stops publishing.
        self.declare_parameter("command_timeout_sec", 0.25)

        self._timeout_ns = int(1e9 * self.get_parameter("command_timeout_sec").value)
        self._yaw = None
        self._last_command_ns = None

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
        self.create_timer(0.05, self._watchdog)

    def _on_command(self, msg):
        rates = commanded_planar_velocity(msg)
        if rates is None or self._yaw is None:
            return
        self._last_command_ns = self.get_clock().now().nanoseconds
        vx, vy, wz = world_to_body(*rates, self._yaw)
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        self._twist_pub.publish(twist)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._yaw = unwrap(self._yaw, yaw_from_quaternion(q.z, q.w))
        t = msg.twist.twist
        rates = body_to_world(t.linear.x, t.linear.y, t.angular.z, self._yaw)

        state = JointState()
        state.header.stamp = msg.header.stamp
        state.name = list(PLANAR_JOINTS + STATE_JOINTS)
        state.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, self._yaw]
        state.position += [0.0] * len(STATE_JOINTS)
        state.velocity = list(rates) + [0.0] * len(STATE_JOINTS)
        self._state_pub.publish(state)

    def _watchdog(self):
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_command_ns is not None
            and now_ns - self._last_command_ns > self._timeout_ns
        ):
            self._twist_pub.publish(Twist())
            self._last_command_ns = None
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
