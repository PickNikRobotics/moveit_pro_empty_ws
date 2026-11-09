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
  `alpha1-5` is low because mock and sim odometry is exact. The hw launch
  raises it and moves the seed to the hw boot pose.
- **Planners.** NavFn keeps the id `GridBased`, which Nav2's default navigate
  tree uses. SMAC 2D is the SMAC variant for a base that moves in any
  direction: Hybrid-A* assumes a car, and the State Lattice needs a motion
  primitive file.
- **MPPI.** The Omni model samples x, y and yaw independently. There is no
  `PathAngleCritic`: every mode of it asks the base to face along the path,
  which an omnidirectional base need not do. `TwirlingCritic` penalises
  needless turning instead, and `GoalAngleCritic` sets the final heading.
- **Arrival.** The goal checker is stateful and the base now holds its
  heading, so it stops as soon as it enters the xy tolerance circle. Measured
  against the true pose it can end just outside that circle, by about the
  AMCL error.
- **Velocity.** Nav2's final twist goes to `/base_cmd_vel`, not `/cmd_vel`,
  because on hw the AgileX driver owns `/cmd_vel` before its remap.
- **:220-238 Global costmap.** Static map and inflation only, by choice. Global
  plans therefore route through obstacles that are not in the map; the local
  costmap and MPPI must avoid them. The `obstacle_layer` block stays,
  inactive, so it can be added back to `plugins`.

## src/agilex_ranger_mini_mock/objectives/localize_robot.xml

- **:54-70 Prior estimate.** Capture the map-frame base pose after marker
  acceptance and before reseeding. Failure to read it stops without changing
  AMCL. No first-start fallback or simulator-frame relabel is added here.
- **:5, :36 Marker coordinates.** The marker starts on the displayed robot,
  which is drawn at the current estimate, and `TransformPoseFrame` maps it back
  to `map` through the old correction. A marker left on the model re-seeds the
  old estimate, so the prompt asks for alignment against the map walls.
- **:53-64 Prior estimate age.** The read lags AMCL by its transform tolerance.
  AMCL does not update a stationary base unless asked, so the lagged
  correction equals the current one.

## src/agilex_ranger_mini_mock/objectives/refine_localization_in_place_subtree.xml

- **:12-25 Input guard.** Reject unbounded or empty loops, a disabled fit gate
  and a tolerance the TF wait cannot cover, before publishing a seed. The gate
  cannot be changed to report-only.
- **:36-58 In-place updates.** Each no-motion request is followed by a new scan
  and a short processing interval. Service replies alone can collapse into one
  pending update. Scan arrival and a wait are not an AMCL acknowledgement.
- **:66-101 Acceptance.** Each check scores a fresh scan against the estimate
  AMCL formed from that scan or later, read through
  `Get AMCL Estimate After Scan Subtree`, never the marker seed or an older
  correction. Pro 10.2.0's ScanMatchResidual also refuses too few returns.
- **:176-187 Provisional gate.** At the recorded suite_106 spawn, every usable
  beam fitted the true pose and about a third fitted the incorrect estimate.
  The defaults separate that capture, but are not calibrated across the room
  or on hardware. Repeated geometry can still admit a wrong pose.
- **:188 Tolerance.** Must equal AMCL's `transform_tolerance` in
  `params/nav2_params.yaml`; `test_amcl_tf_timing.py` checks it. A smaller
  value would accept an older correction.
- **:108-159 Restore.** Re-publish the pre-attempt pose, request an update and
  wait until TF carries AMCL's estimate from after it, so the Objective fails
  only once navigation sees the restored estimate. AMCL can alter
  `map -> odom` during the attempt; restore is not atomic and re-seeds a new
  cloud rather than restoring covariance.
- **:8 Cancellation and motion.** Keep the base stationary. A halt bypasses
  the fallback, so cancellation does not restore; run localization again.
  No controller or odometry reset is performed.
- **:26-32 Version.** Behavior IDs and ports were checked against the Pro 10.2.0
  release source, installed headers, and the node model on Vish. The checked-out
  main worktree has an older Nav behavior set; no new C++ is needed.

## src/agilex_ranger_mini_mock/objectives/get_amcl_estimate_after_scan_subtree.xml

- **Why wait.** AMCL stamps each `map -> odom` at its scan's stamp plus
  `transform_tolerance` (nav2_amcl 1.3.12, `laserReceived`). Pro 10.2.0's
  `GetLatestTransform` reads at the latest common time, about the
  robot_state_publisher time, so a read soon after a scan carries the
  correction from scans one tolerance older.
- **:28-67 Stamp check.** A read whose stamp is at least the scan stamp plus
  the tolerance interpolates only AMCL output from that scan or later. The
  retry polls until TF reaches that time; it does not assume the delay.
  `test_amcl_tf_timing.py` replays AMCL-style stamps through tf2 to show both
  reads. If AMCL stops sending, the stamp never passes and the subtree fails.
- **:28 Retry budget.** Covers the largest tolerance the caller's guard admits,
  plus scan latency.
- **:36-41 Unused Transform output.** Pro 10.2.0 Unpack nodes write every
  field, and an unmapped port defaults to `{<field>}`. Left unmapped, the
  `transform` field would overwrite this subtree's TransformStamped `{transform}`
  output with a Transform, and BT.CPP throws on the type change.

## src/agilex_ranger_mini_mock/scripts/planar_base_bridge.py

- **Why a bridge.** No Jazzy ros2_controllers drive controller fits a
  four-wheel independent steering base, and on hw the firmware does the wheel
  maths anyway. So every mode drives the planar joints, and this node is the
  only place a body twist becomes world-frame joint rates.
- **Timeout.** `JointGroupVelocityController` holds its last command, so the
  node sends zeros when the twist goes stale.
- **:107 Non-finite twists.** A NaN or inf twist would integrate into the mock
  and MuJoCo planar joints for good. The node drops it and stops the base at
  once rather than keep the previous twist until it times out.
- **Liveness.** If this node dies, the controller holds its last command. On
  hw, `ranger_driver_bridge.py` watches this node's command stream and stops
  the base. Mock and sim have no such stop.

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

- **Driver modes.** `ranger_base` (`ranger_messenger.cpp`, `TwistCmdCallback`,
  at the pinned commit) picks one mode per twist. Any non-zero `linear.y`
  selects the parallel mode, which sends speed and steering angle and never
  reads `angular.z`. With zero `linear.y`, a small turn radius selects spin
  mode and a large one Ackermann. The pinned callback therefore drops the turn
  whenever sideways speed is non-zero. That is a limit of this ROS command
  path. What the firmware and hardware do with combined motion is UNVERIFIED.
- **Setup kept as is.** MPPI's Omni model can ask for both at once, and hw
  uses the same MPPI and pose-jog settings as mock and sim. No threshold, Nav2
  restriction or bridge policy hides this; the README lists it as a known limit.
- **Heading history.** A body twist becomes world rates in a controller or in
  `planar_base_bridge.py`, with a heading this node published earlier, and
  comes back here as world rates. Read with a newer heading, a
  forward-and-turn command would gain a small `vy` and select the parallel
  mode. So `driver_twist` tries the recent published headings and keeps the
  one that leaves an axis at rounding level, then sets that axis to exactly
  zero. No delay is assumed. The heading must be among the last
  `heading_history` published ones and within a quarter turn of the newest.
  The real delay on the robot is not measured.
- **World-frame commands.** The match runs on every command, including joint
  jog and joint trajectories, which no heading built. If a retained heading
  leaves a body axis at rounding level, the command is sent along that heading.
  In practice that is the boot heading in the first `heading_history` odometry
  messages after the base first turns, with a direction error up to the heading
  change. Not changed on purpose; the test
  `test_world_frame_command_can_match_an_older_heading` pins it. Whether the
  joint jog leaves unused axes at exactly zero is UNVERIFIED.
- **Why not a deadband.** A speed threshold would need an assumed delay and
  would also erase real small sideways commands. A command with real forward
  and sideways parts matches no heading, is rotated with the newest heading and
  keeps both parts.
- **What still loses the turn.** A replay of mock `/joint_states` through
  `driver_twist` matched pure sideways pose jog at a retained heading. Diagonal
  pose jog kept both parts, rotated with the newest heading, so its original
  body direction is not recovered exactly when it was built from an older
  heading. Forward-and-turn pose jog was not matched: the velocity-force
  controller output keeps a small sideways part at every published heading.
  MPPI's Omni output has real sideways speed while it turns. The driver drops
  the turn for both; no policy is applied.
- **Quarter-turn guard.** A quarter turn swaps the axes and a half turn
  reverses the command, so candidates stop a quarter turn from the newest.
  This also limits how far back a command can match.
- **Non-finite commands.** A ros2_control command is NaN until a controller
  writes it, and the topic interface forwards it. The joints xacro starts the
  commands at zero, and `driver_twist` stops the base on any NaN or inf.
- **Speed caps.** `RangerMiniV3Params` caps only some driver modes, and its
  turn-rate cap is above the manual's. So the bridge clamps to the operating
  limits of `config/moveit/joint_limits.yaml`: planar speed to the hypotenuse
  of the x and y limits, the turn rate to the yaw limit. Every planned, jogged
  or Nav2 command fits; only closed-loop tracking surges, such as JTC catching
  up after a stall, are cut. The URDF ceilings are the base's top speed.
- **:63 System state gate.** The driver publishes `/system_state` every cycle
  from the last CAN feedback. Motion needs a recent message reporting the
  normal vehicle state, CAN control and no error bit outside
  `allowed_error_mask`. The ranger_msgs and ugv_sdk enums disagree on RC and
  standby, but agree on CAN. The default allows only the battery warning bit
  (ugv_sdk `SYSTEM_ERROR_BATTERY_WARNING_MASK`). Whether the firmware sets RC
  signal loss while the remote is off is UNVERIFIED; that bit stops the base.
  Odometry and state come from cached feedback, so neither proves the CAN
  link is alive.
- **:198 Zero command to restart.** `planar_velocity_controller` holds its
  last twist and JTC keeps tracking, so ros2_control repeats a moving command
  through a stop. Without a latch the base would move again on its own when
  an E-stop is released, the remote returns to CAN or a fault bit clears. Any
  gate block, the command watchdog and node start set the latch. Only a
  finite command at rounding level on every planar axis clears it. Only Nav2
  commands pass through `planar_base_bridge.py`, which sends zero once Nav2's
  twist times out, so a cancelled Nav2 goal clears it. IMarker teleop and
  waypoints run on JTC, which is closed-loop and holds a residual command, so
  they clear it only after the JTC goal is cancelled or the Objective or
  controller is switched. Whether a released joint or pose jog sends one is
  UNVERIFIED. Not tested on the robot.
- **:234 Command arrival.** The command watchdog tracks valid planar messages,
  including ones the motion gate blocks. A mode fault must not look like a
  missing command publisher in the log.
- **:251 Liveness gate.** `planar_velocity_controller` holds its last command
  and the topic interface republishes it every cycle, so a dead
  `planar_base_bridge.py` would leave Nav2's last twist on the base. The
  bridge needs that node's command stream to be recent before it passes any
  command, from any controller. A restart of the node alone is not taken as
  proof the base stopped; the gate sends the zero twist.
- **Publish period.** ros2_control writes every cycle, faster than the driver
  updates, and the driver re-selects its mode per message. The bridge passes
  at most one twist per period, stops included; only the first stop after
  motion goes through at once.
- **Odometry.** A message with any non-finite value is dropped before it can
  poison the unwrapped heading.
- **Command layout.** `JointStateTopicSystem` lists every joint name but only
  the commanded values, in joint order. The planar joints come first, so the
  first three velocities belong to them.

## src/agilex_ranger_mini_hw/launch/hw_drivers.launch.py

- **AMCL rewrites.** The hw config shares the mock's Nav2 file, then rewrites
  AMCL by full key path: the seed comes from the hw boot pose and the odometry
  noise goes up to Nav2's default. Full paths keep the rewrite away from any
  other key with the same leaf name.
- **Arguments.** MoveIt Pro includes an `additional_driver_launch_file` with no
  arguments, so each setting is changed by editing its default value.
- **:60 No lidar.** A string comparison, not a Python expression, so a path
  with quotes cannot break or run code. Without a lidar AMCL publishes no
  `map` frame and localization cannot pass, hence the warning.

## Dockerfile

- **rosdep `--skip-keys catkin`.** The vendored `ugv_sdk` declares ROS 1's
  catkin as its build tool. Jazzy has no rosdep key for it, and its CMake
  builds without catkin.

## colcon.meta

- **`ugv_sdk` tests off.** The workspace builds with tests on. `ugv_sdk` then
  needs its nested `test/googletest` submodule, which is empty unless the clone
  is fully recursive. colcon reads this file from the workspace root, so the
  vendored package stays unchanged.
