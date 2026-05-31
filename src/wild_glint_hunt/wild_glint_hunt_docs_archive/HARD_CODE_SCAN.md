# Hard-Coded Data Scan

This scan covers `src/`, `include/`, `config/`, and `launch/` after parameter centralization.

## Source Defaults

These values remain in source only as ROS 2 `declare_parameter` defaults, so they can be overridden from `config/params.yaml` or `--ros-args`.

| File | Line | Current value | YAML-backed |
|---|---:|---|---|
| `src/vision_node.cpp` | constructor | `image_topic` default is `/rgb_camera/image_raw` when `use_rgb_camera=true`, otherwise `/image_left` | yes, `vision_node.image_topic` |
| `src/vision_node.cpp` | 162 | `vision_output_topic = vision/balls` | yes |
| `src/vision_node.cpp` | 163 | `vision_qos_depth = 10` | yes |
| `src/vision_node.cpp` | 167-168 | image size `640 x 480` | yes |
| `src/vision_node.cpp` | constructor | camera height `0.28 m`, target diameter `0.20 m`, RGB FOV `70 deg` | yes |
| `src/vision_node.cpp` | 169-174 | boundary/no-detection/contour thresholds `0.15`, `999.0`, `0.004`, `20.0`, `100.0`, `4.0` | yes |
| `src/vision_node.cpp` | constructor | camera matrix and distortion defaults; fisheye remap controlled by `enable_fisheye_undistortion` | yes |
| `src/vision_node.cpp` | 182-184 | HSV defaults: orange, blue, yellow | yes |
| `src/path_planner_node.cpp` | 77-81 | field and margin defaults `4.0`, `0.15`, `0.20`, `0.10` | yes |
| `src/path_planner_node.cpp` | 82-84 | topics `/odom`, `vision/balls`, `planner/target` | yes |
| `src/state_machine_node.cpp` | 20-26 | backend, state/success topics, required count, period, QoS | yes |
| `src/robot_interface_sim.cpp` | constructor | sim topics `/cmd_vel`, `/odom`, `/imu`, `/obstacle_detection`, `/tof` plus official protocol topic names | yes |
| `src/robot_interface_sim.cpp` | 20-22 | command rate `20.0`, head-butt `0.4 m/s`, `1.0 s` | yes |
| `src/robot_interface_real.cpp` | constructor | real protocol topics `odom_out`, `imu`, `ultrasonic_payload`, `head_tof_payload`, `rear_tof_payload`, `motion_servo_cmd` | yes |

## Configuration Values

These values intentionally live in YAML as the authoritative source.

| File | Line | Current value | Notes |
|---|---:|---|---|
| `config/params.yaml` | vision_node | `/rgb_camera/image_raw`, `/rgb_camera/camera_info` | RGB default; confirm/override on hardware |
| `config/params.yaml` | vision_node | `0.28`, `0.20`, `70.0`, RGB `camera_matrix`, `distortion_coeffs` | TODO: official Cyberdog2 RGB calibration |
| `config/params.yaml` | 13-17 | boundary and contour thresholds | calibrate in field |
| `config/params.yaml` | 18-35 | orange/blue/yellow HSV thresholds | task-specified defaults |
| `config/params.yaml` | 39-42 | field and safety margins | mirrored from field map for ROS params |
| `config/params.yaml` | 43-47 | planner topics and QoS | YAML-backed |
| `config/params.yaml` | 51-64 | state, sim topics, command timing | YAML-backed |
| `config/params.yaml` | 71-82 | official real topics/actions/LCM channels plus TOF TODO | based on scanned `cyberdog_ros2` |
| `config/game_field_map.yaml` | 2-6 | field size, origin, exit, safety margins | exit is TODO pending drawing |
| `config/game_field_map.yaml` | 8-12 | 4x4 grid and fixed blue cells | y-origin requires verification |
| `config/real_hardware_topics.yaml` | all | deployment mapping template | reference only, not loaded |

## Non-Parameter Literals

These are algorithmic constants or labels, not deployment data.

| File | Value | Reason |
|---|---|---|
| `src/vision_node.cpp` | OpenCV matrix sizes `3 x 3`, `1 x 4` | camera calibration shape; fisheye only when enabled |
| `src/vision_node.cpp` | ball labels `orange_ball`, `blue_ball` | message classification labels |
| `src/state_machine_node.cpp` | state enum order | task state machine model |
| `include/wild_glint_hunt/robot_interface.hpp` | zero initialization values | safe default message state |
