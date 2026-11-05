# agilex_ranger_mini_hw

**This config drives the physical AgileX Ranger Mini v3. It has not been run on
a robot.** Everything below is the intended setup, checked only by build, unit
tests and a review of the AgileX driver source.

## How it connects

```text
MoveIt Pro controllers (planar x, y, yaw velocity)
  -> JointStateTopicSystem   /agilex_ranger_mini/joint_commands
  -> ranger_driver_bridge.py world rates -> body twist
  -> ranger_base_node        /agilex_ranger_mini/driver/cmd_vel -> CAN -> firmware
ranger_base_node odometry   /agilex_ranger_mini/driver/odom
  -> ranger_driver_bridge.py -> /agilex_ranger_mini/joint_states -> planar joints
```

The firmware picks the steering mode from the twist; ros2_control never
commands a wheel. The AgileX driver is vendored as the `ranger_ros2` and
`ugv_sdk` submodules and builds with the workspace.

## Before the first run

1. **CAN.** On the robot computer, with the AgileX USB-CAN adapter plugged in:

   ```bash
   sudo modprobe gs_usb
   sudo ip link set can0 up type can bitrate 500000
   ```

   The drivers container uses host networking, so it sees `can0`. Set the
   `can_port` argument of `launch/hw_drivers.launch.py` for another interface.

2. **Lidar.** The Ranger ships without one, and Nav2 needs a scan. Mount a 2D
   lidar, then:
   - measure its pose in `chassis_link` and set `lidar_mount_xyz` and
     `lidar_mount_rpy` in `config/config.yaml`;
   - pass its driver's launch file as `lidar_launch_file` to
     `launch/hw_drivers.launch.py`. It must publish `sensor_msgs/LaserScan` on
     `/scan`, stamped in `laser_link`.

3. **Map.** Nav2 loads the mock config's `maps/suite_106.yaml`. For another site,
   build a map and pass it as the `map` argument of
   `agilex_ranger_mini_mock/launch/navigation.launch.py`.

4. **Localize.** AMCL is not seeded on hardware. After start-up, run the
   `Localize Robot` Objective and place the marker where the base is.

## First motion

Lift the wheels off the floor or keep a hand on the E-stop. Then:

1. Check that `/joint_states` shows the planar joints changing when you push
   the base by hand.
2. Joint jog `linear_x_joint` slowly and confirm the base moves forward.
3. Only then try pose jog, waypoints and Nav2.

## Known limits

- **Sideways plus turning.** The AgileX driver never mixes the two. A twist
  with any sideways speed selects the oblique mode, which drops the turn rate.
  MPPI's omnidirectional model can command both at once, so expect the base
  to slide first and turn after. See `docs/AGENT_COMMENTS.md`.
- **Mode switches.** The firmware ignores motion commands while it re-steers
  for a new mode, so tracking lags at each switch.
- **Driver speed caps.** The driver clips speed and turn rate below the
  limits in the manual.
- **Wheel joints.** The driver reports no steering or wheel angles, so those
  joints stay at zero in the UI.
