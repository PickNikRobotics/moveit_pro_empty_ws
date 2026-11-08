# Project agent memory

MoveIt Pro user workspace for the AgileX Ranger Mini v3. Overview, package
roles and run commands: [README.md](README.md).

## Layout and sharp edges

- Config chain `agilex_ranger_mini_mock` -> `_sim` -> `_hw`. Robot semantics
  (URDF, SRDF, controllers, Objectives, Nav2) live in mock; a child changes
  only its hardware, through the `hardware_xacro` urdf_param and its own
  `additional_driver_launch_file`.
- `agilex_ranger_mini_sim/mjcf/agilex_ranger_mini.xml` and `suite_106_walls.xml`
  are generated from the mock URDF and map by
  `agilex_ranger_mini_sim/scripts/generate_mjcf.py`. Re-run it after editing
  either; the sim test fails while they are stale.
- Code comments are short; detail and justification go in
  [docs/AGENT_COMMENTS.md](docs/AGENT_COMMENTS.md), by file and line.
- Offline checks are each package's `test/` (`colcon test`). The hw config is
  untested on hardware: see `src/agilex_ranger_mini_hw/README.md`.

## Licensing practice

- `LICENSE` is the BSD 3-Clause License. `NOTICE` lists every external
  dependency with its source URL and exact commit ID.
- Each external dependency's licence text is copied to
  `src/external_dependencies/licenses/LICENSE-<source>`.
- Accepted dependency licences are Apache, BSD, MIT and Boost Software License
  1.0. An unclear licence does not block inclusion: record it in NOTICE and
  tell the reviewer. This does not approve other known licences.
- For SDK-contained libraries, use supplied third-party notices when they
  cover delivered parts; do not add recursive inventories or duplicate copies.
  Inner licences must be compatible with the outer distribution, and required
  licence and copyright notices must remain in delivered source or images.
- Never change files inside `src/external_dependencies/` (submodules and deep
  copies). Adding or removing a dependency, or a file under `licenses/`, is fine.

## Maintaining this file

Keep this file for knowledge useful to almost every future agent session in this project.
Do not repeat what the codebase already shows; point to the authoritative file or command instead.
Prefer rewriting or pruning existing entries over appending new ones.
When updating this file, preserve this bar for all agents and keep entries concise.
