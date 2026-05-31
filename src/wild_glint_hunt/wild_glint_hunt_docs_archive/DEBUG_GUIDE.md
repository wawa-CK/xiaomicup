# wild_glint_hunt Debug Guide

## 启动验证

仿真、RViz、control 已启动后，在任务终端运行：

```bash
cd /workspace
source /opt/ros/galactic/setup.bash
source /home/cyberdog_sim/install/setup.bash
source install/setup.bash
ros2 launch wild_glint_hunt sim_wild_glint_hunt.launch.py use_sim_time:=true
```

## 必看话题和日志

- `/rgb_camera/image_raw`：Gazebo 真实 RGB 相机输入，不应由 `simulated_sensors_node` 合成。
- `/vision/ball_array`：橙球、蓝球、黄线识别结果。
- `/odom`：当前仿真桥接的 Gazebo 真实位姿。
- `/planner/status`：`SEARCHING`、`TARGETING`、`TARGET_ALIGNED`、`STRIKING`、`BOUNDARY_RECOVERY`、`EXITING` 等规划状态。
- `/state_machine/status`：当前任务状态、已撞击数量、当前路线列、planner 状态。
- `/hunt/success`：完成出口后发布 `success`。

## 常见卡死场景

- C3 后靠近上边界：调大 `route_c3_retreat_y_m` 或降低 `route_c3_retreat_x_m`，确保先回到中部偏下安全点再去 C2/C1。
- 边界附近原地转：确认 `boundary_recovery_enabled=true`；必要时增大 `boundary_recovery_distance_m` 到 `0.25`。
- 连续触边：`boundary_recovery_max_count` 次后会强制导航到 `(boundary_force_center_x_m, boundary_force_center_y_m)`。
- 出口慢或不触发：检查 `exit_x_m`、`exit_y_m`、`exit_clearance_m`、`rear_leg_offset_m` 是否与 Gazebo 地图出口一致。
- 误撞蓝球：增大 `obstacle_avoidance_radius_m` 或 `target_blue_exclusion_radius_m`；但过大可能堵住 R4C3/R4C4 中间通道。
- 总任务超时：`task_total_timeout_s` 到期后会放弃剩余目标并导航出口；用于防止 Gazebo 偶发卡死无限运行。

## 当前新增保护参数

参数位于 `config/competition_tuned_params.yaml`：

- `task_total_timeout_s`
- `state_timeout_s`
- `route_phase_timeout_s`
- `route_c3_retreat_enabled`
- `route_c3_retreat_x_m`
- `route_c3_retreat_y_m`
- `boundary_recovery_enabled`
- `boundary_recovery_trigger_margin_m`
- `boundary_recovery_distance_m`
- `boundary_recovery_max_count`
- `boundary_force_center_x_m`
- `boundary_force_center_y_m`
- `exit_linear_speed_mps`
- `debug_verbose`

## Fast Fixed S-Curve Mode

Active fast-route parameters are in `config/competition_tuned_params.yaml`:

- `route_mode: fixed_s_curve`
- `route_columns_order: [4, 3, 2, 1]`
- `align_tolerance_deg: 15.0`
- `dynamic_strike_enabled: true`
- `dynamic_strike_trigger_distance_m: 0.50`
- `strike_success_check_time_s: 1.0`
- `single_strike_timeout_s: 25.0`
- `task_total_timeout_s: 240.0`
- `stuck_detection_time_s: 15.0`

If the robot spins at an observation point, reduce `route_column_scan_offsets_deg` or increase `align_tolerance_deg`. If it approaches too aggressively, lower `approach_far_speed_mps` and `approach_near_speed_mps`. If it gives up too quickly, increase `route_column_visual_timeout_s` or `single_strike_timeout_s`.
