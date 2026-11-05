"""Unit tests for the pure functions in the mock package's scripts."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load("planar_base_bridge")
lidar = load("mock_lidar")


@pytest.mark.parametrize("yaw", [0.0, 0.7, math.pi / 2, -2.5, math.pi])
def test_body_world_round_trip(yaw):
    twist = (0.3, -0.2, 0.4)
    world = bridge.body_to_world(*twist, yaw)
    assert np.allclose(bridge.world_to_body(*world, yaw), twist)


def test_forward_at_quarter_turn_is_world_y():
    x_dot, y_dot, yaw_dot = bridge.body_to_world(1.0, 0.0, 0.0, math.pi / 2)
    assert np.allclose((x_dot, y_dot, yaw_dot), (0.0, 1.0, 0.0))


class FakeJointState:
    def __init__(self, name, position, velocity):
        self.name, self.position, self.velocity = name, position, velocity


def test_planar_state_reads_by_name():
    msg = FakeJointState(
        ["fl_wheel_joint", "rotational_yaw_joint", "linear_x_joint", "linear_y_joint"],
        [9.0, 0.5, 1.0, 2.0],
        [0.0, 0.1, 0.2, 0.3],
    )
    position, velocity = bridge.planar_state(msg)
    assert position == (1.0, 2.0, 0.5)
    assert velocity == (0.2, 0.3, 0.1)


def test_planar_state_missing_joint():
    assert bridge.planar_state(FakeJointState(["linear_x_joint"], [0.0], [])) is None


def test_cast_rays_hits_a_wall():
    # A 10 x 10 m grid at 0.1 m, with a wall column at x = 7 m.
    grid = np.zeros((100, 100), dtype=bool)
    grid[:, 70] = True
    angles = np.array([0.0, math.pi / 2, math.pi])
    ranges = lidar.cast_rays(grid, 0.1, (0.0, 0.0), (5.0, 5.0, 0.0), angles, 0.05, 10.0)
    assert abs(ranges[0] - 2.0) <= 0.1
    assert math.isinf(ranges[1]) and math.isinf(ranges[2])


def test_cast_rays_turns_with_the_sensor():
    grid = np.zeros((100, 100), dtype=bool)
    grid[80, :] = True  # A wall row at y = 8 m.
    ranges = lidar.cast_rays(
        grid, 0.1, (0.0, 0.0), (5.0, 5.0, math.pi / 2), np.array([0.0]), 0.05, 10.0
    )
    assert abs(ranges[0] - 3.0) <= 0.1
