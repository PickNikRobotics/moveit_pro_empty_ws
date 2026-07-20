# MoveIt Pro workspace for the SO-101 arm

MoveIt Pro configuration for the **SO-101** (LeRobot) 5-DOF arm with a parallel-jaw
gripper, driven by Feetech STS3215 smart serial-bus servos. Built on
[MoveIt Pro](https://picknik.ai/moveit-pro/) 9.4.1 (ROS 2 Jazzy).

## Config packages

| Package | Type | Hardware |
|---|---|---|
| `so101_base_config` | Base / mock | `mock_components/GenericSystem` — no physics, commands echo back as state |
| *(planned)* physical config | Real hardware | Feetech STS3215 bus via `feetech_ros2_driver`, over a USB-TTL adapter |

The base config is the foundation; the physical config will inherit from it via
`based_on_package` and swap the mock hardware interface for the real Feetech driver.

## Robot description

The URDF, meshes, and end-effector xacros come from the upstream
[`so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai)
project (`so101_description` package), included here as a git submodule under
`src/so101-ros-physical-ai/`. We build the **follower** variant (the actuated arm).

The MoveIt Pro wrapper (`src/so101_base_config/description/so101.urdf.xacro`) adds
the `world` TF root, a `grasp_link` tool-center-point frame, and a ros2_control
block.

## Setup

```bash
# Clone with submodules (the robot description lives in a submodule)
git clone --recurse-submodules https://github.com/PickNikRobotics/moveit_pro_so-101_ws.git
cd moveit_pro_so-101_ws

# Point MoveIt Pro at this workspace and the base config
moveit_pro configure -w "$(pwd)" -c so101_base_config

# Build and run
moveit_pro build
moveit_pro run
```

Then open the web UI at http://localhost.

> This config targets ROS 2 Jazzy. Ensure `MOVEIT_ROS_DISTRO: jazzy` is set in
> `~/.config/moveit_pro/moveit_pro_config.9.yaml`.

## Basic motion test

From the web UI, run the **Move to Home** or **Move to Ready** objective, or from
a shell:

```bash
ros2 action send_goal --feedback /do_objective \
  moveit_studio_sdk_msgs/action/DoObjectiveSequence \
  "{objective_name: 'Move to Home'}"
```

Gripper: run the **Open Gripper** / **Close Gripper** objectives.

## Stop

```bash
moveit_pro down
```
