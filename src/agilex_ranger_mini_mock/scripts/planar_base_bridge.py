#!/usr/bin/env python3
"""Connect a body-frame base twist to the Ranger Mini's planar joints.

The base pose is three URDF joints: x and y in the world frame, then yaw. A
twist is in the robot's own frame, so this node turns it into planar joint
rates for planar_velocity_controller, and turns the planar joint states back
into nav_msgs/Odometry for Nav2. The same node runs in mock, sim and hw.

    x_dot = vx * cos(yaw) - vy * sin(yaw)
    y_dot = vx * sin(yaw) + vy * cos(yaw)
    yaw_dot = wz
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

PLANAR_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")


def body_to_world(vx, vy, wz, yaw):
    """Rotate a body-frame planar twist into world-frame joint rates.

    @param vx Forward speed, m/s.
    @param vy Leftward speed, m/s.
    @param wz Yaw rate, rad/s.
    @param yaw Current heading of base_link in the world frame, rad.
    @return The x, y and yaw joint rates.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (vx * c - vy * s, vx * s + vy * c, wz)


def world_to_body(x_dot, y_dot, yaw_dot, yaw):
    """Rotate world-frame planar joint rates into a body-frame twist.

    @param x_dot Rate of the x joint, m/s.
    @param y_dot Rate of the y joint, m/s.
    @param yaw_dot Rate of the yaw joint, rad/s.
    @param yaw Current heading of base_link in the world frame, rad.
    @return Forward speed, leftward speed and yaw rate.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (x_dot * c + y_dot * s, -x_dot * s + y_dot * c, yaw_dot)


def planar_state(msg):
    """Read the planar joints out of a JointState.

    @param msg A sensor_msgs/JointState.
    @return (positions, velocities) as 3-tuples, or None if a joint is missing.
    """
    index = {name: i for i, name in enumerate(msg.name)}
    if any(name not in index for name in PLANAR_JOINTS):
        return None
    ids = [index[name] for name in PLANAR_JOINTS]
    if max(ids) >= len(msg.position):
        return None
    position = tuple(msg.position[i] for i in ids)
    if max(ids) < len(msg.velocity):
        velocity = tuple(msg.velocity[i] for i in ids)
    else:
        velocity = (0.0, 0.0, 0.0)
    return position, velocity


class PlanarBaseBridge(Node):
    """Twist in, planar joint rates out; planar joint states in, odometry out."""

    def __init__(self):
        super().__init__("planar_base_bridge")
        self.declare_parameter("twist_topic", "/base_cmd_vel")
        self.declare_parameter("command_topic", "/planar_velocity_controller/commands")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("command_rate_hz", 50.0)
        # The controller holds its last command, so a silent publisher must stop the base.
        self.declare_parameter("twist_timeout_sec", 0.5)

        self._timeout_ns = int(1e9 * self.get_parameter("twist_timeout_sec").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._twist = None
        self._twist_stamp_ns = 0
        self._yaw = None

        self.create_subscription(
            Twist, self.get_parameter("twist_topic").value, self._on_twist, 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self._command_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("command_topic").value, 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.create_timer(
            1.0 / float(self.get_parameter("command_rate_hz").value), self._tick
        )

    def _on_twist(self, msg):
        self._twist = msg
        self._twist_stamp_ns = self.get_clock().now().nanoseconds

    def _on_joints(self, msg):
        state = planar_state(msg)
        if state is None:
            return
        (x, y, yaw), (x_dot, y_dot, yaw_dot) = state
        self._yaw = yaw
        vx, vy, wz = world_to_body(x_dot, y_dot, yaw_dot, yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(0.5 * yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

    def _tick(self):
        out = Float64MultiArray()
        now_ns = self.get_clock().now().nanoseconds
        fresh = (
            self._twist is not None
            and now_ns - self._twist_stamp_ns <= self._timeout_ns
        )
        if fresh and self._yaw is not None:
            t = self._twist
            out.data = list(
                body_to_world(t.linear.x, t.linear.y, t.angular.z, self._yaw)
            )
        else:
            out.data = [0.0, 0.0, 0.0]
        self._command_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarBaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
