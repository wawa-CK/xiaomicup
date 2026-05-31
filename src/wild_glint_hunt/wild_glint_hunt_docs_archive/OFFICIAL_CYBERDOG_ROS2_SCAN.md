# Official cyberdog_ros2 Scan

Source repository scanned: `https://github.com/MiRoboticsLab/cyberdog_ros2`, cloned at `/tmp/cyberdog_ros2`.

## Topic And Message Mapping

| Function | Topic | Message type | Definition location | Evidence |
|---|---|---|---|---|
| Stereo camera left stream | `image_left` | `sensor_msgs/Image` | `cyberdog_interaction/cyberdog_camera/cyberdog_camera/src/stereo_camera/mono_stream_consumer.cpp:32` | publisher name is `"image_" + m_name`; `m_name` is `left` |
| Stereo camera right stream | `image_right` | `sensor_msgs/Image` | `cyberdog_interaction/cyberdog_camera/cyberdog_camera/src/stereo_camera/stereo_camera.cpp:94` | camera names are `left`, `right` |
| Body state / IMU orientation | `BodyState` | `ception_msgs/BodyState` | `cyberdog_ception/cyberdog_body_state/src/body_state_server.cpp:98` | lifecycle publisher creates topic `BodyState` |
| Body state message | n/a | `ception_msgs/BodyState` | `cyberdog_interfaces/ception_msgs/msg/BodyState.msg:3` | contains `posequat` and `speed_vector` |
| Ultrasonic aggregate | `ObstacleDetection` | `ception_msgs/Around` | `cyberdog_ception/cyberdog_obstacledetection/src/obstacle_detection_server.cpp:110` | lifecycle publisher creates topic `ObstacleDetection` |
| Ultrasonic payload | field in `ObstacleDetection` | `ception_msgs/Ultrasonic` | `cyberdog_interfaces/ception_msgs/msg/Around.msg:3`, `ception_msgs/msg/Ultrasonic.msg:3` | `front_distance.range_info` is `sensor_msgs/Range` |
| TOF | not found as ROS topic | not found | `tools/docs/soft_arch.svg` mentions MCU_Driver_TOF to BodyStateDetection | no concrete msg/topic in scanned source |
| Motion command input | `body_cmd` | `motion_msgs/SE3VelocityCMD` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:170` | `topic_name_rc` defaults to `body_cmd` |
| Motion command output | `cmd_out` | `motion_msgs/SE3VelocityCMD` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:161` | republished for observers |
| Motion odometry output | `odom_out` | `nav_msgs/Odometry` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:158` | motion manager publisher |

## Motion Control

`motion_manager` exposes three action servers:

| Action name | Action type | Definition location |
|---|---|---|
| `checkout_mode` | `motion_msgs/action/ChangeMode` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:137` |
| `checkout_gait` | `motion_msgs/action/ChangeGait` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:142` |
| `exe_monorder` | `motion_msgs/action/ExtMonOrder` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:147` |

Velocity control is not direct `geometry_msgs/Twist`. The ROS-side command is `motion_msgs/SE3VelocityCMD` on `body_cmd`. `motion_manager` validates mode/source/frame/timestamp, writes `motion_control_request_lcmt`, and publishes LCM channel `exec_request`.

LCM channels and types found:

| Channel | Type | Location |
|---|---|---|
| `exec_request` | `motion_control_request_lcmt` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:2140` |
| `exec_response` | `motion_control_response_lcmt` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:221` |
| `state_estimator` | `state_estimator_lcmt` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:223` |
| `motion-list` | `trajectory_command_lcmt` | `cyberdog_decision/decision_maker/src/motion_manager.cpp:1489` |

`motion_control_request_lcmt` fields are `pattern`, `linear[3]`, `angular[3]`, `point[3]`, `quaternion[4]`, `body_height`, `gait_height`, and `order`. A `life_count` field was not found in the scanned `cyberdog_ros2` LCM definitions.

## Fisheye Calibration

No factory fisheye calibration YAML, `camera_info`, `K`, or OpenCV fisheye `D[k1,k2,k3,k4]` file was found in the official repository. The package keeps explicit TODO values in `config/params.yaml`. The expected standard location for deployment is a camera calibration YAML distributed by the device image or camera driver package, typically under a `config/`, `calib/`, or `camera_info/` directory and loaded into `vision_node.camera_matrix` and `vision_node.distortion_coeffs`.
