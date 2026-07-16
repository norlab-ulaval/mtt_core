# MTT optional obstacle avoidance

This package prepares a shadow-first obstacle bypass layer around the existing
WILN Teach & Repeat controller. It is **not enabled by default** and the current
WILN command path remains unchanged.

## Safety locks in the shipped configuration

Six independent conditions prevent this package from moving the robot:

1. `avoidance_shadow.launch.py` defaults to `enabled:=false`.
2. Docker Compose only exposes it through the explicit `avoidance` profile.
3. The profile forces `shadow_mode:=true` and `execution_enabled:=false`.
4. Runtime arming and reverse recovery are locked in `avoidance.yaml`.
5. The dispatcher requires the three simultaneous gates `enabled`,
   `!shadow_mode`, and `hardware_output_enabled` before it can publish to
   hardware topics.
6. Its shipped outputs are `/mtt_avoidance/speed_cmd_preview` and
   `/mtt_avoidance/articulation_setpoint_preview`. WILN still publishes through
   its proven direct chain because `external_command_mux` defaults to `false`.

Starting the shadow profile therefore only computes and reports decisions:

```bash
docker compose --profile avoidance up avoidance_shadow
```

Do not combine this with command rewiring until the old repeat mode has first
been validated on the robot and the shadow outputs have been reviewed.

## Architecture

The nodes have deliberately narrow responsibilities:

- `mtt_avoidance_supervisor`: deterministic hold/planning/execution/recovery
  state machine. Missing obstacle data is a sensor fault, never permission to
  move.
- `mtt_rejoin_planner`: selects several points ahead on the recorded Teach path
  and requests feasible Smac Hybrid paths. It scores candidates using path
  length, deviation from Teach, and clearance.
- `mtt_composite_path_validator`: checks the swept tractor and trailer polygons
  against the live Nav2 costmap. It rejects stale poses, stale maps, unknown
  space, frame mismatches, and collisions.
- `mtt_avoidance_executor`: revalidates before starting MPPI FollowPath and every
  0.5 s while moving. Revalidation prunes the path already traversed so an
  obstacle behind the robot cannot spuriously abort the bypass.
- `mtt_recovery_manager`: bounded Nav2 BackUp action behind separate locks. It
  requires fresh rear clearance, fresh articulation, and sufficiently certain
  localization. It is disabled.
- `mtt_articulated_nav_adapter_node`: converts Nav2's signed longitudinal
  velocity and yaw-rate request into the MTT's normalized articulation using
  the shared calibrated MTT curvature/slip model, measured articulation, rate
  limits, and stale-feedback refusal.
- `mtt_avoidance_command_mux`: fail-closed WILN/Nav2 selector. It selects speed
  and articulation as one atomic command and holds the last articulation while
  forcing speed to zero during stale input or a source transition.
- `mtt_articulated_command_dispatcher_node`: the only future owner of the two
  physical outputs. It splits the selected atomic command into longitudinal
  speed and absolute articulation setpoint, with independent activation gates,
  feedback checks, clamping, timeout, and articulation rate limiting.

The canonical command between WILN/Nav2, the mux, and the dispatcher is a
`geometry_msgs/TwistStamped` with deliberately MTT-specific semantics:

```text
linear.x   signed longitudinal speed [m/s], positive or negative
angular.z  normalized articulation [-1, 1], not yaw rate
```

Nav2 supplies the rolling 3-D voxel costmaps, Smac Hybrid planner, MPPI
controller, velocity smoother, behavior server, lifecycle management, and
Collision Monitor. Nav2 calls MPPI's built-in car-like curvature constraint
`Ackermann`; that name does not describe the MTT actuator interface. The actual
conversion to central articulation is performed only by the MTT adapter above.
The planner and MPPI footprint is the steerable tractor;
the composite validator is authoritative for the articulated trailer swept
volume. This avoids treating the whole combination as one rigid 8 m rectangle,
which would make turns and narrow doors unnecessarily impossible.

Normal bypass planning is bidirectional: Smac uses Reeds-Shepp paths, MPPI
follows feasible path orientations across direction inversions, and the MTT
adapter preserves signed velocity. A negative speed therefore produces a
normal planned reverse segment rather than a special actuator hack. The
separate emergency BackUp recovery remains independently locked because it
needs validated rear clearance.

## Trailer estimator contract

No trailer estimator is implemented here. The future estimator can publish:

```text
/trailer/pose_in_map  geometry_msgs/PoseWithCovarianceStamped
```

The pose and covariance must be expressed in `map`. While commissioning, the
validator falls back to measured articulation plus a low-speed kinematic
prediction. Once the estimator is trustworthy, set
`require_trailer_pose: true`; missing or stale trailer pose will then reject
every path. Tractor/trailer dimensions, hitch offset, axle distance, and
uncertainty margin are configurable in `config/avoidance.yaml`.

The fixed Collision Monitor polygon is only a conservative immediate-stop
fallback. The per-path composite validator uses predicted articulation for the
complete maneuver and is the authoritative execution gate.

## Observability

Important outputs are:

```text
/mtt_avoidance/status                  mtt_interfaces/AvoidanceStatus
/mtt_avoidance/planner_status          std_msgs/String
/mtt_avoidance/selected_path           nav_msgs/Path
/mtt_avoidance/plan_clearance_m        std_msgs/Float32
/mtt_avoidance/executor_status         std_msgs/String
/mtt_avoidance/recovery_status         std_msgs/String
/mtt_avoidance/selected_articulated_cmd geometry_msgs/TwistStamped
/mtt_avoidance/speed_cmd_preview        geometry_msgs/TwistStamped
/mtt_avoidance/articulation_setpoint_preview std_msgs/Float64
```

Expected shadow states after a persistent obstacle are `HOLDING`, `PLANNING`,
then either `SHADOW_READY` or `BLOCKED`. `EXECUTING` and `RECOVERING` must never
appear with the shipped settings.

## NVIDIA nvblox preparation

The stack consumes a standard Nav2 costmap, so NVIDIA perception remains an
optional producer rather than a hard dependency. On the Isaac image, first
verify whether `nvblox_nav2` is installed in addition to `nvblox_ros`:

```bash
ros2 pkg prefix nvblox_nav2
```

If available, a full Nav2 parameter variant may replace the voxel layer with
`nvblox::nav2::NvbloxCostmapLayer`; pass that file through
`nav2_params_file:=...`. If only `nvblox_ros` is present, keep the tested Hesai
voxel layers or bridge an nvblox 2-D map output as a standard Nav2 layer. The
composite validator and supervisor do not need to change.

## Future activation checklist

Activation is intentionally not encoded as a ready-to-run Compose profile.
Before creating one, verify at minimum:

- shadow decisions and selected paths over representative obstacles, doors,
  narrow aisles, localization jumps, and stale sensors;
- actual tractor and trailer polygons and all frame conventions;
- rear perception before allowing any BackUp action;
- commanded-vs-measured steering and speed limits at very low speed;
- stop behavior when ICP, costmap, lidar, trailer pose, or Nav2 lifecycle fails;
- a physical low-speed test area with an independent emergency stop.

Only then make all of the following changes together, never partially:

- set WILN's command topic to `/mtt_avoidance/wiln_cmd`;
- set WILN `external_command_mux:=true`, which makes its `angular.z` carry
  normalized articulation and suppresses its direct articulation/speed-servo
  publishers;
- make the dispatcher outputs `/controller/cmd_vel` and
  `/mtt_articulation_setpoint`;
- set `execution_enabled:=true`, `shadow_mode:=false`, and
  `hardware_output_enabled:=true` only for an intentional low-speed test.

This makes the mux/dispatcher the sole owner of both autonomous speed and
articulation. Keep emergency reverse recovery disabled until its rear-clearance
input has been independently validated. Planned Reeds-Shepp reverse segments
must also be physically validated before unrestricted use.
