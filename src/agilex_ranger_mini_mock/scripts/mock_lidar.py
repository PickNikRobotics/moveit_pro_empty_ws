#!/usr/bin/env python3
"""Simulate the Ranger Mini's planar lidar against the map, for mock hardware.

Mock hardware has no physics and no sensors, but AMCL and the costmaps need a
scan. This node casts rays through the occupancy grid that map_server serves,
from the base pose held in the planar joints, and publishes a LaserScan in
laser_link. Mock assumes the map frame and the world frame coincide.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan

PLANAR_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")


def cast_rays(grid, resolution, origin_xy, pose, angles, range_min, range_max):
    """Cast planar rays through an occupancy grid.

    @param grid Boolean array [row, col], True where occupied; row 0 is the map's lowest y.
    @param resolution Metres per cell.
    @param origin_xy Map position of cell (0, 0)'s outer corner.
    @param pose Sensor x, y and yaw in the map frame.
    @param angles Beam angles relative to the sensor's +X, rad.
    @param range_min Shortest reported range, m.
    @param range_max Longest reported range, m.
    @return Ranges, one per beam; inf where a beam meets nothing.
    """
    step = 0.5 * resolution
    distances = np.arange(range_min, range_max + step, step)
    heading = pose[2] + np.asarray(angles)
    xs = pose[0] + np.outer(np.cos(heading), distances)
    ys = pose[1] + np.outer(np.sin(heading), distances)
    cols = np.floor((xs - origin_xy[0]) / resolution).astype(int)
    rows = np.floor((ys - origin_xy[1]) / resolution).astype(int)
    inside = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
    hit = np.zeros_like(inside)
    hit[inside] = grid[rows[inside], cols[inside]]
    first = np.argmax(hit, axis=1)
    ranges = np.where(hit.any(axis=1), distances[first], np.inf)
    return ranges


class MockLidar(Node):
    """Publishes a LaserScan ray-cast through /map from the planar joint pose."""

    def __init__(self):
        super().__init__("mock_lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "laser_link")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("beams", 360)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        # Lidar position in base_link; it must match laser_joint in the URDF.
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        # Cells at or above this occupancy count as walls, as in map_server's trinary map.
        self.declare_parameter("occupied_threshold", 65)

        beams = int(self.get_parameter("beams").value)
        self._increment = 2.0 * math.pi / beams
        self._angles = -math.pi + self._increment * np.arange(beams)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._offset = (
            float(self.get_parameter("offset_x").value),
            float(self.get_parameter("offset_y").value),
        )
        self._threshold = int(self.get_parameter("occupied_threshold").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._map = None
        self._pose = None

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self._pub = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, 10
        )
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _on_map(self, msg):
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self._map = (
            data >= self._threshold,
            info.resolution,
            (info.origin.position.x, info.origin.position.y),
        )

    def _on_joints(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if all(name in index for name in PLANAR_JOINTS):
            self._pose = tuple(msg.position[index[name]] for name in PLANAR_JOINTS)

    def _tick(self):
        if self._map is None or self._pose is None:
            return
        x, y, yaw = self._pose
        c, s = math.cos(yaw), math.sin(yaw)
        sensor = (
            x + c * self._offset[0] - s * self._offset[1],
            y + s * self._offset[0] + c * self._offset[1],
            yaw,
        )
        grid, resolution, origin = self._map
        ranges = cast_rays(
            grid,
            resolution,
            origin,
            sensor,
            self._angles,
            self._range_min,
            self._range_max,
        )
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._frame_id
        scan.angle_min = float(self._angles[0])
        scan.angle_max = float(self._angles[-1])
        scan.angle_increment = self._increment
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = [float(r) for r in ranges]
        self._pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = MockLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
