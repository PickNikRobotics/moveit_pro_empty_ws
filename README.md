# AgileX Ranger Mini for MoveIt Pro

MoveIt Pro configuration packages for the AgileX Ranger Mini v3, a four-wheel
independent steering base that can drive in any direction.

| Package | Hardware | Status |
| --- | --- | --- |
| `agilex_ranger_mini_mock` | ros2_control mock hardware | Base of the chain: robot description, controllers, Objectives, Nav2 |
| `agilex_ranger_mini_sim` | MoveIt Pro MuJoCo simulator, suite_106 room | Inherits from mock |
| `agilex_ranger_mini_hw` | Physical base over CAN, AgileX `ranger_base` driver | Inherits from sim. **Untested on hardware**: read [its README](src/agilex_ranger_mini_hw/README.md) |

Each child overrides only its hardware. Robot semantics live in the mock package.

## What the configs support

Mock and sim:

- Teleoperation of the base from the UI: waypoints, IMarker, pose jog and joint jog
  (the `Teleoperate` Objective).
- Nav2 on the suite_106 map: AMCL localization (`Localize Robot`), NavFn
  (`Navigate to Clicked Point with NavFn`) and SMAC 2D
  (`Navigate to Clicked Point with SMAC`) global planners, and the MPPI
  controller with its omnidirectional motion model.

The base is three planar joints, x, y and yaw. Every controller commands their
velocity, so only one runs at a time; the Objectives switch between them.

### Localization

`Localize Robot` uses the marker as an initial guess, not an accepted location.
Align the marker with the map walls, not with the robot model: the model shows
the current estimate, so a marker left on it re-seeds that estimate. Keep the
base stationary. It requests AMCL updates without moving and accepts
only an estimate whose lidar scans fit the map. Each check waits until TF
carries the estimate AMCL formed from the checked scan. A rejected attempt
re-publishes the previous estimate and returns failure.

This uses the current AMCL instance, so `map -> odom` can change during the
attempt. Cancelling skips the failure restore: run localization again before
navigating. Restore re-seeds the previous pose, not its original particle cloud.
The scan-fit settings are provisional for suite_106; see
[docs/AGENT_COMMENTS.md](docs/AGENT_COMMENTS.md).

## Known limit on the real robot

The pinned AgileX driver ignores the requested turn whenever the sideways
speed is not zero. Nav2's MPPI model and pose jog can ask for both at once, so
the turn may be lost on the physical base. This setup is kept as is, and the
robot's own behaviour is untested. See the
[hw README](src/agilex_ranger_mini_hw/README.md#known-limits).

## Build and run

Build the user workspace, then run one config:

```bash
git clone --recurse-submodules <this repository> ~/agilex_ws
moveit_pro build user_workspace -w ~/agilex_ws
moveit_pro run -c agilex_ranger_mini_sim -w ~/agilex_ws
```

Use `-c agilex_ranger_mini_mock` for mock hardware. The meshes and the
suite_106 map image are stored with Git LFS. Run `git lfs pull` before the
build if the robot shows no body, or if map_server fails to load the map, which
leaves AMCL and Nav2 without a map.

To change the robot model or the map, edit the mock package and regenerate the
sim's MuJoCo files with `scripts/generate_mjcf.py` in `agilex_ranger_mini_sim`.
Its test fails when they are stale.

## Licensing

This repository is BSD-3-Clause ([LICENSE](LICENSE)). External dependencies and
their licences are listed in [NOTICE](NOTICE); their licence texts are in
`src/external_dependencies/licenses/`. The AgileX meshes are included with an
unclear licence: see NOTICE.

For more information, refer to the [MoveIt Pro Documentation](https://docs.picknik.ai/).
