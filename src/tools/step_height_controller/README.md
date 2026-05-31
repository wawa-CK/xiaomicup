Step-Height Controller
======================

Purpose
-------
Small helper that listens to a ROS2 depth image and publishes LCM `robot_control_cmd`
messages to temporarily increase the robot `step_height` when a near obstacle/step
is detected in a configurable ROI.

Quick start
-----------
1. Edit `config.yaml` to change 1-2 parameters (e.g. `distance_threshold` or `high_step`).
2. Ensure Python deps installed: `numpy`, `pyyaml`, `lcm`, and ROS2 `rclpy`.
3. Run the script (in a ROS2-enabled environment):

```bash
python3 src/tools/step_height_controller/step_height_controller.py --config src/tools/step_height_controller/config.yaml
```

What to change per run
----------------------
- `distance_threshold`: how close an object must be to trigger higher step.
- `high_step`: step height (m) used when obstacle detected.
- `normal_step`: step height (m) used otherwise.
- `roi`: region of interest (fractions) to look for obstacles.

Notes / Safety
--------------
- The script publishes `robot_control_cmd` LCM messages — make sure your system
  is listening and that publishing is safe for your robot.
- Adjust `high_step` conservatively for a real robot; large values can cause
  collisions or unstable gaits. Test in simulation first.
