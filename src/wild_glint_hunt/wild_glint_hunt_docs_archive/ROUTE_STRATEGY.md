# Fixed S-Curve Route Strategy

## Goal

The active Gazebo strategy is `route_mode: fixed_s_curve`: the robot follows a short deterministic S-shaped route and only uses RGB vision at column observation points. This removes the previous behavior where visual detections repeatedly interrupted motion and caused long spins.

## Execution Order

1. Start at the second-stage entrance near R4/C4.
2. Rotate 45 degrees counter-clockwise and enter the C3/C4 corridor.
3. At the C4 observation point, scan with a short angle list and strike the C4 orange ball.
4. Continue through the C2/C3 corridor and strike C3.
5. Move to the C1/C2 corridor and strike C2, then C1.
6. After four completed columns, navigate to the upper-left exit.

## Efficiency Rules

- The state machine stays in `FOLLOW_ROUTE`; orange detections during transit do not break the route.
- Alignment tolerance is intentionally loose: `align_tolerance_deg: 15.0`.
- Dynamic strike is enabled: when the target is within `dynamic_strike_trigger_distance_m`, the planner accepts alignment and performs a light touch.
- Strike verification is simplified: if the target remains visible or was visible recently after `strike_success_check_time_s`, the strike is accepted.
- A failed column is skipped after `single_strike_timeout_s` and `max_strike_retries`; the robot continues the route instead of stalling.

## Safety and Recovery

- The route waypoints are generated from the static grid and avoid fixed blue balls.
- Runtime boundary recovery remains available during waypoint/target motion, but does not interrupt pure yaw-facing commands.
- Stuck detection watches pose progress. If movement is below `stuck_detection_distance_m` for `stuck_detection_time_s`, the planner backs up and turns before rejoining the route.
