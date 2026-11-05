# Agent comments

Detail and justification that does not fit in a code comment, by file and then
by line. Keep each entry short, and keep it true when the code changes.

## src/agilex_ranger_mini_mock/description/agilex_ranger_mini_macro.xacro

- **:6-13 Dimensions.** Wheelbase, track, body size and ground clearance come
  from the AgileX Ranger Mini 3.0 User Manual V1.0.0 (2024-06), specification
  table, p.5. The AgileX driver uses the same wheelbase and track
  (`ranger_ros2`, `ranger_params.hpp`, `RangerMiniV3Params`). The manual's
  "axle track" is the front-to-rear spacing, here `wheelbase`.
- **:10 Pivot height.** AgileX's v3 xacro puts each steering pivot 0.1 m below
  its body mesh origin and each wheel centre 0.12 m below the pivot. With a
  0.1 m wheel radius that places the mesh origin at `mesh_origin_height` above
  the floor, and the mesh's own bounds then give a clearance and height within
  1 cm of the manual.
- **:8 Wheel radius.** No AgileX document states it. The wheel mesh measures
  0.1005 m and AgileX's controller config uses 0.1.
- **:16 Mesh origin.** `chassis_link` is the AgileX `base_link`, so the vendored
  meshes load at their own origins. `base_link` here is the REP 105 frame on
  the floor.
- **:19-22 Masses.** The manual gives the robot's weight. The split between
  chassis and corner modules is an estimate; AgileX's xacro module masses are
  kept.
- **:25-28 Velocity ceilings.** Top speed and spin rate are from the manual,
  p.6 and the 0x111 command limits, p.19-20. The steering range is the widest
  angle any mode commands (oblique, 90 degrees). No document gives a steering
  rate; AgileX's v3 xacro value is kept.
- **:31 Base travel.** The planar x and y joints need a finite range for the
  planners and IK seeds. The range covers far more than any one map.
- **:34 Lidar.** The real Ranger has no lidar. Mock and sim need one for AMCL
  and the costmaps, so the description carries a mount on the deck centre;
  the hw config overrides its pose.
- **:146 Steering sign.** AgileX's xacro uses axis `0 0 -1`, so its positive
  angle steers right. This model uses `+Z`, REP 103. Compare signs before
  feeding AgileX steering feedback into these joints.
- Not modelled: the swing-arm suspension, and any sideways offset of a wheel
  from its pivot. The base is planar.

## src/agilex_ranger_mini_mock/description/agilex_ranger_mini.urdf.xacro

- **:15 Frames above `world`.** `navigation.launch.py` publishes `odom -> world`
  as a static identity, and AMCL publishes `map -> odom`. So the planar joints
  hold the base pose in the odometry frame, and AMCL's correction moves the
  whole robot. Under mock and sim the joints carry the true pose, so AMCL
  converges to an identity; on hw they carry wheel odometry and AMCL corrects
  its drift. Mock and sim assume the map frame and the MuJoCo world coincide.

## src/agilex_ranger_mini_mock/description/agilex_ranger_mini_ros2_control_joints.xacro

- **Steering and wheel joints are state only.** The firmware alone commands
  them. Mock holds them at zero, sim holds them straight with joint stiffness,
  and the AgileX driver publishes no angles, so they are for display.

## src/agilex_ranger_mini_mock/config/control/agilex_ranger_mini.ros2_control.yaml

- **One motion controller at a time.** All four claim the planar velocity
  interfaces. `planar_velocity_controller` is active at start-up so Nav2 can
  drive at once; Teleoperate and the waypoint Objective switch the others in
  with automatic deactivation, and the navigation Objectives switch back.
- **JTC gains.** A velocity interface needs closed-loop position feedback. The
  proportional gain is the one MoveIt Pro's own holonomic base examples use
  for their planar joints.

## src/agilex_ranger_mini_mock/config/moveit/joint_limits.yaml

- **:1-3 Teleop limits.** Below the URDF ceilings and equal to Nav2's MPPI
  limits, so a planned move and a navigated move are equally fast indoors.
  The joint velocity and pose jog controllers use the same numbers.

## src/agilex_ranger_mini_mock/params/nav2_params.yaml

- **AMCL.** The omni motion model matches the base. The odometry noise
  `alpha1-5` is low because mock and sim odometry is exact; raise it on hw.
- **Planners.** NavFn keeps the id `GridBased`, which Nav2's default navigate
  tree uses. SMAC 2D is the SMAC variant for a base that moves in any
  direction: Hybrid-A* assumes a car, and the State Lattice needs a motion
  primitive file.
- **MPPI.** The Omni model samples x, y and yaw independently. `PathAngleCritic`
  mode 0 lets the base drive in any direction along the path.
- **Velocity.** Nav2's final twist goes to `/base_cmd_vel`, not `/cmd_vel`,
  because on hw the AgileX driver owns `/cmd_vel` before its remap.

## src/agilex_ranger_mini_mock/scripts/planar_base_bridge.py

- **Why a bridge.** No Jazzy ros2_controllers drive controller fits a
  four-wheel independent steering base, and on hw the firmware does the wheel
  maths anyway. So every mode drives the planar joints, and this node is the
  only place a body twist becomes world-frame joint rates.
- **Timeout.** `JointGroupVelocityController` holds its last command, so the
  node sends zeros when the twist goes stale.

## src/agilex_ranger_mini_mock/scripts/mock_lidar.py

- **Why ray-cast the map.** Mock hardware has no sensors, but AMCL needs a scan
  to publish `map -> odom`. Casting through `/map` from the planar joint pose
  gives AMCL and the costmaps a scan that matches the map exactly.

## src/agilex_ranger_mini_sim/scripts/generate_mjcf.py

- **:33-38 Actuator gains.** The velocity actuators drive the base mass with a
  time constant of a few hundredths of a second, so the base tracks commands
  without ringing. Yaw has less inertia, so a lower gain gives a similar time
  constant. The steering stiffness keeps the unactuated modules straight while
  the base turns.
- **Lidar on `base_link`.** MoveIt Pro publishes a rangefinder fan's scan frame
  relative to the body its sites sit on. On `base_link`, which
  robot_state_publisher also publishes, the scan frame follows the planar
  joints and needs no re-stamping.
- **Beam count.** Rangefinders are evaluated on the physics thread, so the fan
  is kept to one beam every two degrees.

## src/agilex_ranger_mini_hw/scripts/ranger_driver_bridge.py

- **Driver modes.** `ranger_base` (`ranger_messenger.cpp`, `TwistCmdCallback`)
  picks one firmware mode per twist. Any sideways speed selects the oblique
  mode, which ignores the turn rate. Otherwise a small turn radius selects spin
  mode and a large one Ackermann. MPPI's Omni model can ask for both at once;
  if that misbehaves on the robot, lower `wz_max` or `vy_max` in MPPI for hw.
- **Speed caps.** The driver clips speed and turn rate below the manual's
  ceilings (`RangerMiniV3Params`).
- **Command layout.** `JointStateTopicSystem` lists every joint name but only
  the commanded values, in joint order. The planar joints come first, so the
  first three velocities belong to them.

## Dockerfile

- **rosdep `--skip-keys catkin`.** The vendored `ugv_sdk` declares ROS 1's
  catkin as its build tool. Jazzy has no rosdep key for it, and its CMake
  builds without catkin.
