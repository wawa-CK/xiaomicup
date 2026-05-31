# DEVELOPMENT_MEMO

## 2026-05-30 route/strike cleanup
- `src/state_machine_node.cpp`
  - Removed runtime dependency on relocalization/`/estimated_pose`; state machine now subscribes to `/odom` directly.
  - Tightened transit logic so `C2/C3` junction is a forced stop point and transit completion requires actual junction arrival.
  - Tightened strike verification: light-touch success is valid only after strike motion actually completed.
- `src/path_planner_node.cpp`
  - Removed planner subscription to `/estimated_pose`; planner now tracks `/odom` directly.
  - Preserved fixed-route waypoint following only; no relocalization input path remains.
  - Kept strike boundary degradation and unconditional emergency backoff after strike.
- `src/robot_interface_sim.cpp`, `src/robot_interface_real.cpp`
  - Added explicit invalid-strike guard before sending strike motion.
  - Kept INFO logs around strike command emission.
- `CMakeLists.txt`, `launch/official_integration.launch.py`
  - Removed `relocalization_node` from build/install/launch.
- `src/relocalization_node.cpp`, `include/wild_glint_hunt/relocalization.hpp`
  - Deleted as obsolete.
- `config/competition_tuned_params.yaml`
  - Removed relocalization block and switched main planner/state-machine pose topic configuration back to `/odom`.
