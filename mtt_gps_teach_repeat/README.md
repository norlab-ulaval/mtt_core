# mtt_gps_teach_repeat

GPS-only teach and repeat: `.traj` recording, ROMEA path matching, and
articulated path following — independent of WILN/LiDAR, but following the
same safety contract (AUTO mode + deadman gating) and the same canonical
command topics (`/mtt_articulation_setpoint`, `controller/cmd_vel`).

## Nodes

| Node | Role |
|---|---|
| `mtt_gps_recorder_node` | Records `localization/odom` samples into a ROMEA `.traj` v2 file |
| `mtt_gps_path_server_node` | Loads a `.traj`, matches the robot pose onto it (`romea::core::PathMatching`), publishes Frenet errors + path curves |
| `mtt_gps_path_follower_node` | Frenet-error control law → articulation setpoint + `controller/cmd_vel` |

## Setting the anchor

`mtt_gps_recorder_node` and `mtt_gps_path_server_node` each take an
`anchor: [latitude_deg, longitude_deg, altitude_m]` parameter — a WGS84
reference point ROMEA's path/matching math is built around. **The two must
be set to the exact same value** (both are commented to say so in
`demos/common/config/gps_recorder.yaml` / `gps_path_server.yaml`) — a
mismatch means the recorded route and the live matching disagree about
where the ground is.

This is a separate, independent reference point from `factor_graph_node`'s
own `gps_origin_lat/lon/alt` (which anchors the `map`/`localization/odom`
ENU frame) — they don't have to match each other, only the two GPS
teach & repeat files have to match one another. The checked-in field anchor
is `[46.778879, -71.277157, 0.0]`, taken from the valid `/gps_front/fix` in
`gps.png`; replace altitude `0.0` when a surveyed altitude is available.

Practical way to get a real value: use any recent fix near the operating
area, ideally RTK Fix/Float quality. From a live session:

```bash
ros2 topic echo /gps_front/fix --once
# read latitude, longitude, altitude from the message
```

From a bag (no live robot needed):

```bash
ros2 bag play <bag_path> --topics /gps_front/fix &
ros2 topic echo /gps_front/fix --once
```

Then set the same three numbers in both
`demos/common/config/gps_recorder.yaml` and
`demos/common/config/gps_path_server.yaml`. The exact point doesn't matter
much (ROMEA builds a local ENU tangent plane around it) — it just needs to
be near where the robot actually operates, not far away, and identical in
both files.

## Recording quality control

Recording a route from raw localization output is only as good as the input,
so the recorder actively defends the taught path:

- **Fix-quality gating** (`min_fix_quality`, default `rtk_float`): samples are
  skipped while GPS precision is below the threshold. Precision is read from
  `NavSatFix.position_covariance`, not `NavSatStatus` — the standard ROS
  status enum cannot distinguish RTK Fix from RTK Float (mtt_gps_driver maps
  both to `STATUS_GBAS_FIX`); covariance is the only reliable signal
  (`logic/gps_fix_quality.hpp`).
- **Jump rejection** (`max_plausible_speed_ms`, default 5.0 m/s): a sample
  implying an impossible speed relative to the last recorded point (GPS
  reacquisition, multipath, ISAM2 relocalization) is rejected instead of
  bending the taught curve.
- **Smoothing** (`smoothing_window_size`, default 2): a centered moving
  average is applied to each section once recording stops, before saving —
  same idea as WILN's teach smoothing.

## Seeing the curve ("the parabola")

`romea::core::PathMatching` fits a continuous curve through the recorded
waypoints (`PathCurve2D`, sampled at `interpolation_window` resolution) —
that dense, smooth curve is exactly what gets published, in two places:

- `/mtt/gps_path_server/full_path` (latched): the **entire** loaded/recorded
  route, republished once on load and on `~/reload` — review the taught path
  before driving it.
- `/mtt/gps_path_server/local_horizon`: the segment ahead of the robot's
  current matched position, refreshed every odom tick — what the follower is
  actually tracking right now.
- `/mtt/gps_path_follower/executed_path`: the localization path actually
  driven during the current GPS Replay. It is cleared when Replay is armed,
  ignores manual motion, and remains latched after Stop/Complete for direct
  comparison against the taught reference.

## Obstacle handling

The GPS follower consumes the same front LiDAR stop and slowdown gate as the
robot stack. Missing/stale obstacle data is fail-closed while Replay is armed.
It does not yet deform the GPS path around an obstacle: WILN's `PathDeformer`
remains the path-avoidance implementation. The extension point is deliberate:
`mtt_gps_path_follower_node`
only ever reads its Frenet input from the parameterized `matched_info_topic`.
A future `mtt_gps_path_deformer_node` could subscribe to `local_horizon` plus
an obstacle cloud, republish a corrected Frenet point on its own topic, and
the follower's `matched_info_topic` parameter gets pointed at it — no change
needed in the path server or the follower.

## Foxglove

See `demos/monitor/layouts/mtt_command_center.json`, tab **🛰️ GPS Teach &
Repeat**: 3D comparison of Teach/reference/local horizon/executed trajectory,
satellite Map panel on `/gps_front/fix`, synchronized lifecycle timeline,
fix-quality and tracking gauges, Teach/Stop/Clear, Replay/Stop,
List/Load/Status and live AUTO speed cap. The **📈 GPS Control** tab provides
dedicated Frenet, articulation/curvature/speed, progress and slowdown plots.

## Running

Via compose (`demos/live_robot` or `demos/data_collection`) — this is what
the Foxglove GPS tab expects to be running:

```bash
# Starts factor graph + recorder + route server + disarmed follower + obstacle gate.
dc up -d
```

`GPS_RECORDER_PARAMS_FILE` / `GPS_PATH_SERVER_PARAMS_FILE` /
`GPS_PATH_FOLLOWER_PARAMS_FILE` override the params files (defaults:
`demos/common/config/gps_{recorder,path_server,path_follower}.yaml` — set
the anchor there, see above).

Direct launch (no compose), e.g. inside `docker compose run --rm bash`:

```bash
ros2 launch mtt_gps_teach_repeat gps_localization.launch.py
```

Do not run this alongside WILN's `wiln_path_follower` — both publish on the
same AUTO command chain.

Canonical services:

- `/mtt_gps/teach_start`, `/mtt_gps/teach_stop`, `/mtt_gps/clear_trajectory`
- `/mtt_gps/list`, `/mtt_gps/load`, `/mtt_gps/status`
- `/mtt_gps/replay`, `/mtt_gps/stop`

Every Teach gets a unique `gps_route_<timestamp>.traj`; Stop saves atomically
and auto-loads it. Replay never starts merely because a route was loaded: the
operator must explicitly call `/mtt_gps/replay`, select AUTO and hold deadman.
