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

MoveIt Pro starts `launch/hw_drivers.launch.py` without arguments. To change
a setting below, edit the `default_value` of its argument in that file.

1. **CAN.** On the robot computer, with the AgileX USB-CAN adapter plugged in:

   ```bash
   sudo modprobe gs_usb
   sudo ip link set can0 up type can bitrate 500000
   ```

   The drivers container uses host networking, so it sees `can0`. For another
   interface, edit the `can_port` argument.

2. **Lidar.** The Ranger ships without one, and Nav2 needs a scan. Mount a 2D
   lidar, then:
   - measure its pose in `chassis_link` and set `lidar_mount_xyz` and
     `lidar_mount_rpy` in `config/config.yaml`;
   - set the `lidar_launch_file` argument to its driver's launch file. It must
     publish `sensor_msgs/LaserScan` on `/scan`, stamped in `laser_link`.

3. **Map.** Nav2 loads the mock config's `maps/suite_106.yaml`. For another
   site, build a map and set the `map` argument to its YAML file.

4. **Localize.** Do step 2 first: AMCL publishes the `map` frame only after
   its first scan. It starts at the map origin, because the odometry starts
   there at power-on, so the base first shows at the map corner. Run the
   `Localize Robot` Objective and place the marker where the base really is.

5. **Odometry noise.** The launch file raises AMCL's odometry noise to Nav2's
   default, for real wheel slip. Tune `HW_ODOM_ALPHA` in that file once the
   base has driven on the site's floor.

## First motion

Lift the wheels off the floor or keep a hand on the E-stop. Then:

1. Check that `/joint_states` shows the planar joints changing when you push
   the base by hand.
2. Joint jog `linear_x_joint` slowly and confirm the base moves forward.
3. Joint jog `rotational_yaw_joint` slowly in the positive direction and
   confirm the base turns left, counter-clockwise seen from above.
4. Only then try pose jog, waypoints and Nav2.

**Waypoints are odometry poses.** The planar joints count from where the base
powered on, not from the map. So this config has one waypoint, `Spawn`, at the
power-on spot, and `Move Base to Spawn` drives back there in a straight line
that ignores the map. Save new waypoints only after you have driven to them.

## Known limits

- **Sideways plus turning.** The pinned `ranger_ros2` driver never mixes the
  two. Its stock ROS `Twist` callback ignores the requested turn (`angular.z`)
  whenever the sideways speed (`linear.y`) is not exactly zero: it selects the
  parallel mode, which sends only speed and steering angle. Spinning in place
  and forward-and-turn need exactly zero sideways speed. This is what the
  driver source does with a command. It is not a measured limit of the robot:
  how the Ranger firmware and hardware handle combined sideways and turning
  motion is **UNVERIFIED**.
  - **Kept as is.** The MPPI and pose-jog settings are the same as in mock and
    sim. No speed threshold, Nav2 restriction or bridge policy hides the limit.
    Nav2's MPPI omnidirectional model, while driving and possibly while it
    turns to the goal heading, and pose jog through the velocity-force
    controller, whose own output has a small sideways part when it turns, both
    send sideways speed with a turn, so this driver drops the turn for both.
  - **What the bridge does.** It removes the rounding left by its own frame
    change, so a forward-and-turn twist rotated by `planar_base_bridge.py`
    keeps its turn. It does this by matching the command to a heading it
    published, within the last `heading_history` headings and a quarter turn of
    the newest.
  - **Side effects of the matching.** A world-frame command that lines up with
    a retained heading is read as built from it. A joint jog soon after the
    base first turns away from its boot heading can go along that older heading
    for up to `heading_history` odometry messages. A diagonal command is
    rotated with the newest heading. When it was built from an older heading,
    its original body direction is not recovered exactly.

  See `docs/AGENT_COMMENTS.md`.
- **Mode switches.** The firmware ignores motion commands while it re-steers
  for a new mode, so tracking lags at each switch.
- **Speed caps.** The driver does not cap forward or sideways speed. The
  bridge clamps the planar speed and the turn rate to the URDF limits of the
  planar joints.
- **Stale odometry.** If the driver stops publishing odometry, the bridge
  sends zero twists and warns, rather than drive on a frozen pose.
- **Wheel joints.** The driver reports no steering or wheel angles, so those
  joints stay at zero in the UI.
