# Simulation Guide

## Dependencies

- Ubuntu 20.04 with ROS 2 Galactic, or a compatible ROS 2 build environment.
- `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `cv_bridge`, OpenCV.
- No `ception_msgs` or `motion_msgs` packages are required for simulation.

## Build

```bash
colcon build --packages-select wild_glint_hunt
source install/setup.bash
```

## Run

If Gazebo, RViz, and `cyberdog_control` are already open, run exactly one task command:

```bash
ros2 launch wild_glint_hunt sim_wild_glint_hunt.launch.py
```

By default this launch now loads `config/competition_tuned_params.yaml`, which is the current
known-good Gazebo full-course parameter set. To test another parameter file:

```bash
ros2 launch wild_glint_hunt sim_wild_glint_hunt.launch.py \
  params_file:=/absolute/path/to/params.yaml
```

The launch resets the robot to the right-bottom second-stage entry pose `(x=3.20, y=0.70, yaw=1.57)`
and then starts the autonomous task. Do not publish manual `/cmd_vel` or joystick commands after
this launch; the planner owns motion until `/hunt/success` is published.

The simulation launch starts:

- `simulated_sensors_node`: publishes `/odom`, `/body_state`, `/imu`, `/obstacle_detection`, and `/tof`; synthetic image publishing is disabled by default.
- `vision_node`: subscribes `/rgb_camera/image_raw` from the Gazebo RGB camera plugin, skips fisheye remap in RGB mode, runs HSV detection, and publishes `/vision/ball_array` plus `vision/undistorted_image`.
- `path_planner_node`: subscribes `/vision/ball_array` and `/odom`.
- `state_machine_node`: runs the task state machine with `backend=sim`, consumes `/odom`, and sends planner commands only once the autonomous loop is active.

## Current Competition-Oriented Behavior

- The fixed route starts from the right-bottom second-stage entry and uses `/odom` for global decisions.
- Search waypoints are greedy-nearest safe viewpoints around the official 4x4 ball-column map, not direct drives through ball centers.
- Fixed blue balls at R3C4, R4C3, and R4C4 are skipped as targets and treated as obstacle exclusion zones.
- The planner keeps a 0.35 m boundary inset and predicts short-horizon motion before sending velocity to avoid stepping on yellow lines.
- Target approach uses visual distance/yaw updates and clamps angular velocity to reduce in-place spinning.
- The current tuned version uses rolling execution: after a short start-corner escape and scan, it can execute one high-confidence visible orange target immediately instead of waiting for a full-map search.
- Orange-target association uses fixed-route column context plus visual range/yaw to keep target selection constrained within the active aisle.
- Strike verification now accepts three success modes: visible ball displacement, visible disappearance after a confirmed pre-strike lock, or pose-contact success when the robot head reaches the target contact envelope.

## Observe

```bash
ros2 topic echo /hunt/state
ros2 topic echo /hunt/success
ros2 topic echo /state_machine/status
ros2 topic echo /planner/status
ros2 topic echo /vision/danger_warning
ros2 topic echo /vision/ball_array
ros2 topic echo /odom
ros2 topic echo /obstacle_detection
```

To inspect the generated and undistorted camera stream:

```bash
rqt_image_view
```

Select `/rgb_camera/image_raw` for the simulated RGB source image or `vision/undistorted_image` for the image used by HSV detection.

## Calibration

`config/params.yaml` keeps placeholder RGB calibration values and legacy fisheye switch parameters:

- `vision_node.camera_matrix`
- `vision_node.distortion_coeffs`
- `simulated_sensors_node.camera_matrix`
- `simulated_sensors_node.distortion_coeffs`

Replace them with Cyberdog2 factory calibration before real deployment. The official `cyberdog_ros2` repository scanned during integration did not include a fisheye calibration YAML or `camera_info` file. On a real robot, check device-image paths such as:

```bash
find /opt/ros -iname '*camera*' -o -iname '*calib*' -o -iname '*fisheye*'
find / -iname '*camera_info*' -o -iname '*fisheye*' 2>/dev/null
```

For RGB mode, expected data are pinhole intrinsics `K=[fx,0,cx,0,fy,cy,0,0,1]`, distortion coefficients from `/rgb_camera/camera_info`, and camera-to-body extrinsics. For fisheye fallback, expected data are OpenCV fisheye `K` and `D=[k1,k2,k3,k4]`.

## Sensor Parameter Checklist

If the dog cannot recognize orange or blue balls, first verify that `vision_node` is receiving a real image:

```bash
ros2 topic hz /rgb_camera/image_raw
ros2 topic echo /vision/ball_array --once
rqt_image_view
```

Current vision input is `vision_node.image_topic=/rgb_camera/image_raw`. The Gazebo RGB camera plugin is attached to `RGB_camera_link`; `simulated_sensors_node` no longer supplies synthetic images by default. If the real robot publishes RGB on `/image_left` or another topic, set `image_topic` accordingly in `config/params.yaml` or override it at launch.

Required camera parameters:

- `image_topic`: actual RGB image topic that can see the race balls; current Gazebo default is `/rgb_camera/image_raw`.
- `camera_info_topic`: matching camera info topic if available; current default is `/rgb_camera/camera_info`.
- `camera_matrix`: RGB pinhole intrinsic matrix `K=[fx,0,cx,0,fy,cy,0,0,1]`; current value is a 640x480 placeholder.
- `distortion_coeffs`: RGB camera distortion coefficients; current value disables distortion. If `enable_fisheye_undistortion=true`, this must be OpenCV fisheye `D=[k1,k2,k3,k4]`.
- `horizontal_fov_deg`: real horizontal FOV after choosing the physical camera stream.
- `camera_height_m`: optical center height relative to ground.
- `rgb_camera_translation_body_m` / `rgb_camera_rpy_body_rad`: simulator values are from `cyberdog_description/xacro/robot.xacro`; real extrinsics must be calibrated or read from TF.
- `target_diameter_m`: ball diameter, currently `0.20`.
- `orange.*`, `blue.*`, `yellow.*`: HSV thresholds; tune with saved images from the actual camera because Gazebo lighting and real lighting differ.
- `min_ball_area_px`, `min_ball_radius_px`: minimum contour filters; lower them if balls are visible but too small in the image.
- `boundary_meters_per_pixel`: placeholder ground-projection scale for yellow line distance; needs calibration from camera height/pitch/FOV.

Required range-sensor parameters before fusing ultrasonic/TOF into safety:

- topic names and message types for front ultrasonic, head TOF, and rear TOF.
- sensor frame IDs and transforms to `base_link`.
- valid range min/max, field of view, and mounting direction.
- conservative stop/avoid thresholds for each sensor.

At the moment, ultrasonic and TOF are published/subscribed for connectivity, but collision avoidance is
still driven by the static blue-ball map plus vision danger warning. They are not yet used as primary
obstacle classifiers.

## Repeatable Scenarios

The simulated ball layout is controlled in `config/params.yaml`:

- `simulated_sensors_node.sim_random_seed`
- `simulated_sensors_node.sim_randomize_balls`
- `simulated_sensors_node.sim_orange_ball_pixels`
- `simulated_sensors_node.sim_blue_ball_pixels`

The state-machine and planner behavior is controlled by:

- `state_machine_node.max_strike_retries`
- `state_machine_node.sim_assume_strike_success`
- `state_machine_node.strike_success_pixel_shift`
- `state_machine_node.strike_success_distance_shift_m`
- `state_machine_node.strike_verify_timeout_s`
- `state_machine_node.strike_accept_pose_contact_success`
- `state_machine_node.target_association_max_world_error_m`
- `state_machine_node.rolling_execute_single_visible_confidence`
- `state_machine_node.rolling_execute_min_travel_m`
- `path_planner_node.target_use_visual_distance_updates`
- `path_planner_node.target_use_visual_yaw_updates`
- `path_planner_node.avoid_back_duration_s`
- `path_planner_node.avoid_turn_duration_s`
- `path_planner_node.approach_stop_distance_m`

For the built-in static-image simulation, keep `sim_assume_strike_success=false` only when the
image generator is extended to remove or move balls after a strike. The default simulation keeps
`sim_assume_strike_success=true`, `target_use_visual_distance_updates=false`, and
`target_use_visual_yaw_updates=false` so the planner can close the loop by integrating its own
commanded motion even though the test image is static. For real hardware or dynamic Gazebo camera
feeds, set both planner visual-update flags to `true` and `sim_assume_strike_success=false`.

## Expected Smoke-Test Progress

A successful launch should show the state machine progressing through:

```text
S0_INIT -> S1_SEARCH -> S2_ALIGN -> S3_STRIKE -> S4_VERIFY
```

After four simulated successful strikes it should enter:

```text
S5_EXIT -> S6_FINISH
```

The launch process keeps running after success so topics remain observable. Stop it with `Ctrl+C`
or wrap it in `timeout` for automated checks.

## Verified Full-Course Run

The current tuned parameter set has completed a full Gazebo run with:

- `vision strike success: count=1`
- `vision strike success: count=2`
- `vision strike success: count=3`
- `vision strike success: count=4`
- `state S5_VERIFY -> S6_EXIT: all targets completed`
- `state S6_EXIT -> S7_FINISH: exit reached`

Use `config/competition_tuned_params.yaml` in this repository as the current known-good Gazebo
baseline. Keep `config/params.yaml` as the general development/default parameter set.

## Remaining Hardware Gaps

- Cyberdog2 RGB physical stream mapping: the simulator URDF defines `RGB_camera_link`, but the scanned Gazebo model does not publish RGB images. On hardware, confirm the actual RGB image and camera_info topics.
- Real RGB/depth/fisheye extrinsics are still required for race-grade distance estimates: camera mounting height, pitch, frame transform to `base_link`, RGB `K/D`, optional fisheye `K/D`, and ground-projection scale.
- Ultrasonic and TOF are connected at the interface level, but the current planner only uses visual/static-map safety. To fuse range sensors into obstacle decisions, provide sensor frames, mounting directions, valid ranges, and official topic/message confirmation.
