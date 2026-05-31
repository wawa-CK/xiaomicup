# wild_glint_hunt

Cyberdog 2 第二赛道“荒野寻球 / 荒野寻珠”ROS 2 功能包。本包负责识别橙色目标球、避开固定蓝球和黄色边界、按路线搜索并撞击目标，最后导航到出口。

当前代码适配本地 `cyberdog_ws` 的官方 `protocol` 接口。本地官方工作区没有 `ception_msgs` 或 `motion_msgs` 包，因此运动控制、TOF 等接口均按 `protocol` 包处理。

## 维护范围

真正需要维护和提交给队友的是最外层 `wild_glint_hunt/`：

```text
wild_glint_hunt/
  CMakeLists.txt
  package.xml
  config/
  include/
  launch/
  msg/
  src/
```

如果目录中还有 `wild_glint_hunt/wild_glint_hunt/...` 这类重复嵌套副本，它们应视为历史副本，不是当前构建入口。当前最外层 `CMakeLists.txt` 只构建最外层 `src/`、`include/`、`msg/`、`launch/` 和 `config/`。

## 核心流程

```text
RGB image
  -> vision_node
  -> /vision/ball_array, /vision/danger_warning
  -> state_machine_node
  -> planner/command
  -> path_planner_node
  -> RobotInterfaceSim / RobotInterfaceReal
  -> /cmd_vel, motion_servo_cmd, LCM gamepad_lcmt
  -> robot motion
  -> /odom or /odom_out
  -> state_machine_node + path_planner_node
```

当前定位和导航主要依赖里程计：仿真用 `/odom`，官方栈/真机用 `/odom_out`。旧的重定位链路和 `/estimated_pose` 不是当前主流程依赖。

## 官方接口

| 功能 | 话题 | 类型 |
| --- | --- | --- |
| 仿真 RGB 图像 | `/rgb_camera/image_raw` | `sensor_msgs/Image` |
| 官方候选相机 | `/image_left`, `/image_right`, `/rgb_camera/image_raw` | `sensor_msgs/Image` |
| RGB 相机信息 | `/rgb_camera/camera_info` 或 `/image_left/camera_info` | `sensor_msgs/CameraInfo` |
| 里程计 | `/odom_out` | `nav_msgs/Odometry` |
| IMU | `/imu` | `sensor_msgs/Imu` |
| 超声波 | `/ultrasonic_payload` | `sensor_msgs/Range` |
| 头部 TOF | `/head_tof_payload` | `protocol/HeadTofPayload` |
| 后部 TOF | `/rear_tof_payload` | `protocol/RearTofPayload` |
| 运动控制 | `/motion_servo_cmd` | `protocol/MotionServoCmd` |
| 底层 LCM 控制 | `robot_control_cmd`, `robot_control_response` | LCM |
| 仿真步态控制 | `gamepad_lcmt` | LCM `gamepad_lcmt` |

完整模板见 `config/official_topics.yaml` 和 `config/real_hardware_topics.yaml`。真机相机实际话题、`camera_info`、相机内参和外参仍需现场确认。

## 构建

先构建官方接口包，再构建本包：

```bash
cd ~/cyberdog_project
colcon build --packages-select protocol
source install/setup.bash
colcon build --packages-select wild_glint_hunt
source install/setup.bash
```

如果工作区路径不同，只替换第一行路径。

## 仿真运行

在 Gazebo、RViz、Cyberdog control/locomotion 相关进程已启动后运行：

```bash
cd ~/cyberdog_project
source install/setup.bash
ros2 launch wild_glint_hunt sim_wild_glint_hunt.launch.py
```

`sim_wild_glint_hunt.launch.py` 默认加载 `config/competition_tuned_params.yaml`，并会：

1. 调用 `reset_robot_pose` 把机器人放到第二赛道入口附近。
2. 启动 `simulated_sensors_node` 桥接 Gazebo 位姿到 `/odom`。
3. 启动 `check_camera_topic` 检查 `/rgb_camera/image_raw`。
4. 启动 `vision_node`、`path_planner_node`、`state_machine_node` 执行任务。

仿真默认使用 Gazebo 真实 RGB 图像和真实 IMU；`simulated_sensors_node` 默认不发布合成图像、假 IMU、假超声波或假 TOF。运动主要通过官方 Gazebo/locomotion 的 LCM `gamepad_lcmt` 后端驱动，`/cmd_vel` 保留为调试输出。

## 官方栈 / 真机集成

在 `cyberdog_bringup`、相机、`sensor_manager`、`motion_manager` 已启动后运行：

```bash
cd ~/cyberdog_project
source install/setup.bash
ros2 launch wild_glint_hunt official_integration.launch.py backend:=real
```

该启动文件只启动本包的 `vision_node`、`path_planner_node`、`state_machine_node`，不会启动 `simulated_sensors_node`。它会覆盖关键话题：

```text
image_topic=/image_left
camera_info_topic=/image_left/camera_info
odom_topic=/odom_out
official_odom_topic=odom_out
official_imu_topic=imu
official_ultrasonic_topic=ultrasonic_payload
official_tof_topic=head_tof_payload
official_rear_tof_topic=rear_tof_payload
official_motion_servo_topic=motion_servo_cmd
```

如果真机 RGB 实际不在 `/image_left`，需要在 launch 参数或 YAML 中覆盖 `vision_node.image_topic`。

## 节点说明

| 节点 / 文件 | 作用 |
| --- | --- |
| `vision_node` / `src/vision_node.cpp` | 订阅相机图像，HSV 检测橙球、蓝球和黄线边界，发布 `/vision/ball_array`、`/vision/tracked_balls`、`/vision/danger_warning`、`vision/undistorted_image`。 |
| `state_machine_node` / `src/state_machine_node.cpp` | 任务状态机。根据视觉、里程计、规划状态决定搜索、巡航、对准、撞击、验证、退出。 |
| `path_planner_node` / `src/path_planner_node.cpp` | 执行状态机命令，生成路线点、目标接近、撞击、退避、边界/卡住恢复等运动控制。 |
| `robot_interface_sim.cpp` | 仿真后端。发布 `/cmd_vel`、`protocol/MotionServoCmd`，并可通过 LCM `gamepad_lcmt` 驱动官方仿真步态控制。 |
| `robot_interface_real.cpp` | 真机后端。订阅官方 `odom_out`、`imu`、超声波、TOF，发布 `protocol/MotionServoCmd`。 |
| `simulated_sensors_node` | 仿真辅助节点。桥接 Gazebo 位姿到 `/odom`，可按参数发布仿真传感器，但默认不伪造关键传感器。 |
| `check_camera_topic` | 启动时检查指定图像话题是否收到相机帧。 |
| `reset_robot_pose` | 仿真中重置机器人模型位姿。 |

## 消息定义

`msg/VisionBall.msg` 表示单个视觉目标：

```text
id, label, pixel_x, pixel_y, distance_m, yaw_deg, confidence, radius_px, safe_to_approach
```

`msg/VisionBallArray.msg` 表示一帧检测结果：

```text
header, balls, orange_balls, blue_balls, boundary_distance_m, boundary_alert
```

## 配置文件

| 文件 | 作用 |
| --- | --- |
| `config/params.yaml` | 通用开发默认参数。 |
| `config/competition_tuned_params.yaml` | 当前 Gazebo/比赛调参基线，仿真 launch 默认使用。 |
| `config/game_field_map.yaml` | 第二赛道场地尺寸、起点、出口、4x4 球柱坐标、固定蓝球位置。 |
| `config/bringup_params.yaml` | 接入官方 bringup 时的真机参数覆盖。 |
| `config/bringup_nodes.yaml` | 给外部 bringup 系统使用的节点清单。 |
| `config/official_topics.yaml` | 本地 `cyberdog_ws` 扫描得到的官方话题模板。 |
| `config/real_hardware_topics.yaml` | 真机话题映射备忘，不会自动被普通 launch 加载。 |

## Launch 文件

| 文件 | 用途 |
| --- | --- |
| `launch/sim_wild_glint_hunt.launch.py` | 仿真主入口，加载 `competition_tuned_params.yaml`。 |
| `launch/official_integration.launch.py` | 官方栈/真机集成入口，适合队友融合总启动时调用。 |
| `launch/integration_real.launch.py` | 另一套真机集成入口，加载 `params.yaml` 和 `bringup_params.yaml`。 |
| `launch/wild_glint_hunt.launch.py` | 最小启动入口，只启动视觉、规划、状态机。 |

## 当前策略

当前主要策略是固定 S 型路线加滚动视觉确认：

1. 从第二赛道入口进入固定过道。
2. 通过静态 4x4 球柱地图避开固定蓝球。
3. 在列观察点使用 RGB 视觉确认橙球。
4. 每次锁定一个可靠橙球后对准并撞击。
5. 撞击后通过视觉变化、目标消失或位姿接触判断成功。
6. 完成目标数量后导航到左上出口。

关键参数集中在 `config/competition_tuned_params.yaml`：

```text
route_mode: fixed_s_curve
route_columns_order: [4, 3, 2, 1]
route_expected_target_ids: ["R2C4", "R3C3", "R4C2", "R1C1"]
required_ball_count: 4
sim_assume_strike_success: false
sim_use_ground_truth_layout: false
target_use_visual_distance_updates: false
target_use_visual_yaw_updates: false
enable_lcm_gamepad_backend: true
sim_publish_images: false
sim_publish_fake_imu: false
sim_publish_fake_range: false
sim_publish_fake_tof: false
```

## 常用观测命令

```bash
ros2 topic echo /vision/ball_array
ros2 topic echo /vision/danger_warning
ros2 topic echo /planner/status
ros2 topic echo /state_machine/status
ros2 topic echo /hunt/success
ros2 topic echo /odom
ros2 topic hz /rgb_camera/image_raw
rqt_image_view
```

成功完成时应看到 `/hunt/success` 发布 `success`，状态机进入 `FINISH`。

## 真机部署前必须确认

- 实际 RGB 图像话题和 `camera_info` 话题。
- RGB 相机内参 `K`、畸变参数 `D`、水平 FOV。
- 相机到 `base_link` 的外参、安装高度和俯仰角。
- 真实比赛光照下的橙色、蓝色、黄色 HSV 阈值。
- 超声波、头部 TOF、后部 TOF 的 frame、安装方向、有效量程和避障阈值。
- `motion_servo_cmd` 在目标机器上的实际响应是否与当前 `MotionServoCmd` 字段匹配。

## 文档归档

调试和历史说明已集中到项目顶层的 `wild_glint_hunt_docs_archive/`：

```text
../wild_glint_hunt_docs_archive/DEBUG_GUIDE.md
../wild_glint_hunt_docs_archive/SIMULATION_GUIDE.md
../wild_glint_hunt_docs_archive/SIM_TOPIC_FLOW.md
../wild_glint_hunt_docs_archive/ROUTE_STRATEGY.md
../wild_glint_hunt_docs_archive/DEVELOPMENT_MEMO.md
../wild_glint_hunt_docs_archive/HARD_CODE_SCAN.md
../wild_glint_hunt_docs_archive/OFFICIAL_CYBERDOG_ROS2_SCAN.md
../wild_glint_hunt_docs_archive/SENSOR_CONNECTIVITY.md
```

其中 `SENSOR_CONNECTIVITY.md` 和 `OFFICIAL_CYBERDOG_ROS2_SCAN.md` 包含部分旧接口调查记录；当前代码以最外层 `wild_glint_hunt/` 的源码和 `config/*.yaml` 为准。

## GitHub 提交建议

上传给队友融合时，建议提交最外层 `wild_glint_hunt/` 以及可选的 `wild_glint_hunt_docs_archive/`。不要提交 `build/`、`install/`、`log/`，也不要提交重复嵌套的 `wild_glint_hunt/wild_glint_hunt/...` 历史副本。
