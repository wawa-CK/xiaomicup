# Sensor Connectivity Report

## Connected In Current Package

| Sensor/interface | Topic parameter | Default topic | Message type | Status |
|---|---|---|---|---|
| RGB camera | `vision_node.image_topic` | `/rgb_camera/image_raw` | `sensor_msgs/Image` | default vision input; synthetic publisher exists in simulation; real topic still must be confirmed |
| RGB camera info | `vision_node.camera_info_topic` | `/rgb_camera/camera_info` | `sensor_msgs/CameraInfo` | template/default only; `vision_node` currently loads K/D from YAML |
| Legacy camera candidate | `vision_node.image_topic` override | `/image_left` or `/image_right` | `sensor_msgs/Image` | official camera-stack candidate if hardware does not expose `/rgb_camera/image_raw` |
| Estimated pose | `estimated_pose_topic` | `/estimated_pose` | `geometry_msgs/PoseWithCovarianceStamped` | used by planner/state machine for global decisions |
| Odometry | `*_odom_topic` | `/odom`, `odom_out` | `nav_msgs/Odometry` | simulation and official protocol bridge supported |
| IMU | `*_imu_topic` | `/imu` | `sensor_msgs/Imu` | connected in simulator and interface layer |
| Ultrasonic | `official_ultrasonic_topic` | `ultrasonic_payload` | `sensor_msgs/Range` | connected at interface level; not primary obstacle classifier yet |
| TOF | `official_tof_topic`, `official_rear_tof_topic` | `head_tof_payload`, `rear_tof_payload` | `protocol/HeadTofPayload`, `protocol/RearTofPayload` | connected at interface level; not primary obstacle classifier yet |
| Motion command | `official_motion_servo_topic` | `motion_servo_cmd` | `protocol/MotionServoCmd` | real/backend interface publishes official motion command; simulation also keeps `/cmd_vel` |

## Missing Parameters For Reliable Ball Recognition

- Real RGB image topic and camera_info topic on Cyberdog2 hardware.
- RGB camera intrinsic matrix `K`, distortion coefficients, and `horizontal_fov_deg` from `/camera_info` or factory calibration.
- RGB camera-to-body transform. Simulator URDF gives `RGB_camera_link` offset `[0.27576, 0.0, 0.125794]` and zero RPY, but hardware must be validated.
- HSV thresholds tuned from actual RGB images under competition lighting.
- Ground-projection calibration for yellow-line distance; current `boundary_meters_per_pixel` is a placeholder.

## Simulator Findings

- `cyberdog_simulator-main` defines `RGB_camera_link`, `D435_camera_link`, and `AI_camera_link`, but `gazebo.xacro` does not add a Gazebo RGB/fisheye camera plugin.
- Therefore Gazebo alone will not publish a real camera image; `simulated_sensors_node` publishes synthetic `/rgb_camera/image_raw` for current closed-loop tests.
