# AGENTS.md

Role
- You are a ROS 2 Humble engineer focused on map tooling for Nav2.
- Keep changes minimal, testable, and compatible with existing `navegacion_gps` runtime behavior.

Package scope
- Package name: `map_tools`.
- Goal: provide tooling to define, validate, store, and publish no-go zones from maps.
- Do not duplicate Nav2 runtime components from `navegacion_gps`; integrate with them.

Workspace context
- This package lives in `src/map_tools` and is a sibling of `src/navegacion_gps`.
- Main navigation package is `navegacion_gps` and uses Nav2 + robot_localization.
- Default target stack is ROS 2 Humble.

Primary design targets
- Inputs:
  - map metadata (`.yaml`) and occupancy image (`.pgm/.png`) when available
  - user-defined polygons for restricted areas (map frame)
- Outputs:
  - machine-readable zone file (`.yaml`) for persistence
  - runtime representation consumable by Nav2 (costmap filter mask or costmap layer config)
  - visualization markers for RViz (`visualization_msgs/MarkerArray`)

Recommended architecture
- `map_tools/zone_server.py`: lifecycle-friendly node owning zone state.
- `map_tools/zone_io.py`: load/save schema for zone files.
- `map_tools/zone_projection.py`: map<->pixel transforms using map origin/resolution.
- Optional UI package later (separate package) if interactive editing is needed.

Data contract for zones
- Use a stable YAML schema:
  - `frame_id`: usually `map`
  - `zones`: list of objects with:
    - `id` (string)
    - `type` (`no_go`, future: `slowdown`)
    - `enabled` (bool)
    - `polygon` (list of `[x, y]` in meters, map frame)
- Keep coordinates in metric map frame as source of truth.

Integration with Nav2 (preferred order)
1. Costmap filters (preferred for semantics)
- Generate/publish mask + metadata compatible with keepout/speed filters.
- Wire resulting topics/config from `map_tools` into Nav2 launch/params.

2. Static layer injection (fallback)
- Convert zones into occupied cells in an auxiliary occupancy grid.
- Merge via costmap plugin chain if filters are not available.

Launch/params guidance
- Every new runtime setting must be a `DeclareLaunchArgument` with default.
- Avoid hard-coded topic names; expose as params/args.
- Keep `map -> odom -> base_link` TF assumptions unchanged.

Safety/behavior expectations
- No-go zones must be conservative by default.
- Provide deterministic behavior on restart (load same zone file -> same mask).
- Validate polygons (closed/non-self-intersecting where possible) before publish.

Validation checklist
- Unit checks:
  - parse and serialize zone YAML without loss
  - coordinate conversion map<->pixel round-trip sanity
- Runtime checks:
  - node starts cleanly with empty and non-empty zone files
  - marker output matches configured zones
  - Nav2 path planning avoids marked no-go regions

Useful commands
- Build package only:
  - `../../tools/compile-ros.sh map_tools`
- Run package node (inside sourced environment):
  - `ros2 run map_tools zone_server`
- Run tests (when present):
  - `colcon test --packages-select map_tools`

Coding style
- Prefer `rclpy` and small modules with pure utility functions for geometry/IO.
- Keep files ASCII-only.
- Prefer `rg` for search.
- Avoid edits to `worlds/`, `models/`, Docker files unless explicitly requested.

Out of scope for this package
- Rewriting existing `navegacion_gps` controllers/planners.
- Sensor drivers and hardware-specific control paths.
- Full GUI framework integration in the first iteration.
