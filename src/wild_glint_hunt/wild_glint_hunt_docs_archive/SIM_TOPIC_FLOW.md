# Wild Glint Hunt 仿真真实数据链路

当前仿真模式不使用合成相机、假 IMU、假超声波或模型瞬移。`simulated_sensors_node`
只把 Gazebo 中真实机器人模型位姿桥接为定位话题，运动由 Cyberdog 官方
Gazebo/locomotion 的 `gamepad_lcmt` 后端驱动。

## 发布方 / 接收方

| 功能 | 话题 | 类型 | 发布方 | 接收方 | 数据来源 |
| --- | --- | --- | --- | --- | --- |
| RGB 图像 | `/rgb_camera/image_raw` | `sensor_msgs/Image` | `rgb_camera_controller` | `vision_node`, `check_camera_topic` | Gazebo RGB camera plugin |
| 相机参数 | `/rgb_camera/camera_info` | `sensor_msgs/CameraInfo` | `rgb_camera_controller` | 调试/标定工具 | Gazebo RGB camera plugin |
| 调试图像 | `/vision/undistorted_image` | `sensor_msgs/Image` | `vision_node` | `rqt_image_view` | 真实 RGB 图像处理结果 |
| 视觉球列表 | `/vision/ball_array` | `wild_glint_hunt/VisionBallArray` | `vision_node` | `state_machine_node`, `path_planner_node` | HSV 检测真实 RGB 图像 |
| 危险警告 | `/vision/danger_warning` | `std_msgs/Bool` | `vision_node` | `state_machine_node` | 视觉检测结果 |
| Gazebo 位姿桥 | `/odom` | `nav_msgs/Odometry` | `simulated_sensors_node` | `robot_interface_sim` fallback | `gz model -m robot -p` |
| 全局位姿 | `/odom` | `nav_msgs/Odometry` | `simulated_sensors_node` | `state_machine_node`, `path_planner_node` | Gazebo 真实模型位姿 |
| IMU | `/imu` | `sensor_msgs/Imu` | `imu_plugin` | `robot_interface_sim` | Gazebo IMU plugin |
| 任务命令 | `planner/command` | `std_msgs/String` | `state_machine_node` | `path_planner_node` | 状态机 |
| 规划状态 | `planner/status` | `std_msgs/String` | `path_planner_node` | `state_machine_node` | 路径规划器 |
| ROS 调试速度 | `/cmd_vel` | `geometry_msgs/Twist` | `robot_interface_sim` | 调试/桥接节点 | 同步运动命令 |
| 官方仿真控制 | `gamepad_lcmt` | LCM `gamepad_lcmt` | `robot_interface_sim` | `cyberdog_gazebo/lcmhandler` | 官方步态控制器 |

## 当前仿真缺口

- 超声波和 TOF：当前 `cyberdog_simulator-main` 的机器人 xacro 未提供对应 Gazebo
  Range/TOF ROS 插件发布方；`sim_publish_fake_range=false`、`sim_publish_fake_tof=false`，
  所以不会伪造这些数据。
- 运动控制：ROS 话题 `motion_servo_cmd` 在当前仿真中没有 ROS 订阅方；实际可见运动依赖
  官方 Gazebo 控制链路 `gamepad_lcmt`。
- 撞击确认：优先使用真实 RGB 中目标消失/位移；Gazebo 道具摆动不稳定时，使用真实
  Gazebo 位姿判断头部前缘是否到达已视觉确认的目标球位。
