#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "wild_glint_hunt/msg/vision_ball_array.hpp"
#include "wild_glint_hunt/robot_interface.hpp"
#include "wild_glint_hunt/state_machine.hpp"

namespace wild_glint_hunt
{
namespace
{

struct Waypoint
{
  double x {0.0};
  double y {0.0};
};

struct BallTarget
{
  std::string id;
  double x {0.0};
  double y {0.0};
  double approach_x {0.0};
  double approach_y {0.0};
  double strike_yaw_rad {0.0};
};

struct ObservedTarget
{
  std::string id;
  double x {0.0};
  double y {0.0};
  double distance_m {0.0};
  double yaw_deg {0.0};
  int pixel_x {0};
  int pixel_y {0};
  int confirm_count {0};
  double confidence {0.0};
  rclcpp::Time last_seen;
};

struct AssociatedObservation
{
  wild_glint_hunt::msg::VisionBall ball;
  std::string id;
  double x {0.0};
  double y {0.0};
  double cost {0.0};
};

enum class RoutePhase
{
  START_TURN = 0,
  ESCAPE_START = 1,
  MOVE_TO_OBSERVE = 2,
  WAIT_OBSERVE = 3,
  FACE_COLUMN = 4,
  WAIT_FACE_COLUMN = 5,
  ACQUIRE_TARGET = 6,
  WAIT_APPROACH = 7,
  WAIT_ALIGN = 8,
  WAIT_STRIKE = 9,
  WAIT_VERIFY = 10,
  FACE_FORWARD = 11,
  WAIT_FACE_FORWARD = 12,
  RETREAT_AFTER_STRIKE = 13,
  TRANSIT_TO_NEXT_AISLE = 14,
  MOVE_EXIT = 15,
  WAIT_EXIT = 16,
  DONE = 17
};

struct AisleSegment
{
  std::string primary_column;
  std::vector<std::string> scan_columns;
  Waypoint entry;
  Waypoint exit;
};

std::string encode_waypoints(const std::string & prefix, const std::vector<Waypoint> & waypoints)
{
  std::ostringstream stream;
  stream << prefix;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (i > 0) {
      stream << ":";
    }
    stream << waypoints[i].x << ":" << waypoints[i].y;
  }
  return stream.str();
}

std::string make_cell_id(size_t internal_row, size_t col, size_t row_count)
{
  const size_t display_row = row_count - internal_row;
  return "R" + std::to_string(display_row) + "C" + std::to_string(col + 1);
}

}  // namespace

class StateMachineNode : public rclcpp::Node
{
public:
  StateMachineNode() : Node("state_machine_node")
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "hunt/state");
    status_topic_ = declare_parameter<std::string>("state_status_topic", "/state_machine/status");
    success_topic_ = declare_parameter<std::string>("success_topic", "hunt/success");
    success_message_ = declare_parameter<std::string>("success_message", "success");
    planner_command_topic_ = declare_parameter<std::string>("planner_command_topic", "planner/command");
    planner_status_topic_ = declare_parameter<std::string>("planner_status_topic", "planner/status");
    vision_topic_ = declare_parameter<std::string>("vision_input_topic", "/vision/ball_array");
    danger_topic_ = declare_parameter<std::string>("danger_warning_topic", "/vision/danger_warning");
    pose_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    const auto backend = declare_parameter<std::string>("backend", "sim");
    backend_ = backend;
    required_ball_count_ = declare_parameter<int>("required_ball_count", 4);
    max_strike_retries_ = declare_parameter<int>("max_strike_retries", 1);
    sim_assume_strike_success_ = declare_parameter<bool>("sim_assume_strike_success", false);
    timer_period_ms_ = declare_parameter<int>("state_machine_period_ms", 200);
    strike_verify_timeout_s_ = declare_parameter<double>("strike_verify_timeout_s", 2.0);
    strike_success_pixel_shift_ = declare_parameter<double>("strike_success_pixel_shift", 30.0);
    strike_success_distance_shift_m_ =
      declare_parameter<double>("strike_success_distance_shift_m", 0.10);
    strike_recent_visual_timeout_s_ =
      declare_parameter<double>("strike_recent_visual_timeout_s", 2.5);
    strike_require_visual_confirmation_ =
      declare_parameter<bool>("strike_require_visual_confirmation", true);
    strike_accept_pose_contact_success_ =
      declare_parameter<bool>("strike_accept_pose_contact_success", true);
    route_allow_pose_fallback_target_ =
      declare_parameter<bool>("route_allow_pose_fallback_target", false);
    strike_contact_distance_m_ = declare_parameter<double>("strike_contact_distance_m", 0.24);
    strike_front_offset_m_ = declare_parameter<double>("strike_front_offset_m", 0.28);
    target_visible_timeout_s_ = declare_parameter<double>("target_visible_timeout_s", 1.0);
    route_fixed_strategy_enabled_ =
      declare_parameter<bool>("route_fixed_strategy_enabled", true);
    route_enable_columns_ = declare_parameter<std::vector<std::string>>(
      "route_enable_columns", {"C4", "C3", "C2", "C1"});
    route_expected_target_ids_ = declare_parameter<std::vector<std::string>>(
      "route_expected_target_ids", std::vector<std::string>{});
    route_forward_heading_deg_ =
      declare_parameter<double>("route_forward_heading_deg", 0.0);
    route_start_turn_ccw_deg_ =
      declare_parameter<double>("route_start_turn_ccw_deg", 45.0);
    route_face_timeout_s_ = declare_parameter<double>("route_face_timeout_s", 8.0);
    route_column_visual_timeout_s_ =
      declare_parameter<double>("route_column_visual_timeout_s", 5.0);
    route_column_visual_timeout_s_ =
      declare_parameter<double>("scan_observation_window_s", route_column_visual_timeout_s_);
    route_reacquire_retry_limit_ =
      declare_parameter<int>("route_reacquire_retry_limit", 2);
    route_column_scan_offsets_deg_ = declare_parameter<std::vector<double>>(
      "route_column_scan_offsets_deg", {0.0, -45.0, 45.0, -25.0, 25.0});
    route_mode_ = declare_parameter<std::string>("route_mode", "fixed_s_curve");
    route_columns_order_ = declare_parameter<std::vector<int64_t>>(
      "route_columns_order", {4, 3, 2, 1});
    align_tolerance_deg_ = declare_parameter<double>("align_tolerance_deg", 15.0);
    dynamic_strike_enabled_ = declare_parameter<bool>("dynamic_strike_enabled", true);
    strike_success_check_time_s_ =
      declare_parameter<double>("strike_success_check_time_s", 1.0);
    strike_light_touch_ = declare_parameter<bool>("strike_light_touch", true);
    single_strike_timeout_s_ = declare_parameter<double>("single_strike_timeout_s", 25.0);
    debug_verbose_ = declare_parameter<bool>("debug_verbose", false);
    task_total_timeout_s_ = declare_parameter<double>("task_total_timeout_s", 300.0);
    state_timeout_s_ = declare_parameter<double>("state_timeout_s", 60.0);
    route_phase_timeout_s_ = declare_parameter<double>("route_phase_timeout_s", 45.0);
    route_aisle_endpoint_tolerance_m_ =
      declare_parameter<double>("route_aisle_endpoint_tolerance_m", 0.30);
    route_junction_stop_tolerance_m_ =
      declare_parameter<double>("route_junction_stop_tolerance_m", 0.20);
    route_junction_overshoot_distance_m_ =
      declare_parameter<double>("route_junction_overshoot_distance_m", 0.50);
    route_align_reissue_interval_s_ =
      declare_parameter<double>("route_align_reissue_interval_s", 6.0);
    route_visual_align_boundary_margin_m_ =
      declare_parameter<double>("route_visual_align_boundary_margin_m", 0.45);
    post_strike_backoff_distance_m_ =
      declare_parameter<double>("post_strike_backoff_distance", 0.35);
    post_strike_inward_shift_m_ =
      declare_parameter<double>("post_strike_inward_shift", 0.25);
    route_c4_retreat_enabled_ = declare_parameter<bool>("route_c4_retreat_enabled", true);
    route_c4_retreat_x_m_ =
      declare_parameter<double>("route_c4_retreat_x_m", std::numeric_limits<double>::quiet_NaN());
    route_c4_retreat_y_m_ =
      declare_parameter<double>("route_c4_retreat_y_m", std::numeric_limits<double>::quiet_NaN());
    route_c3_retreat_enabled_ = declare_parameter<bool>("route_c3_retreat_enabled", true);
    route_c3_retreat_x_m_ = declare_parameter<double>("route_c3_retreat_x_m", 2.0);
    route_c3_retreat_y_m_ = declare_parameter<double>("route_c3_retreat_y_m", 1.0);
    route_c34_aisle_x_offset_m_ =
      declare_parameter<double>("route_c34_aisle_x_offset_m", 0.0);
    route_c23_aisle_x_offset_m_ =
      declare_parameter<double>("route_c23_aisle_x_offset_m", 0.0);
    route_c3_aisle_exit_y_m_ =
      declare_parameter<double>("route_c3_aisle_exit_y_m", std::numeric_limits<double>::quiet_NaN());
    search_all_before_strike_ = declare_parameter<bool>("search_all_before_strike", true);
    sim_use_ground_truth_layout_ = declare_parameter<bool>("sim_use_ground_truth_layout", true);
    rolling_min_targets_to_execute_ = declare_parameter<int>("rolling_min_targets_to_execute", 2);
    rolling_plan_max_targets_ = declare_parameter<int>("rolling_plan_max_targets", 1);
    rolling_search_timeout_s_ = declare_parameter<double>("rolling_search_timeout_s", 8.0);
    rolling_replan_cooldown_s_ = declare_parameter<double>("rolling_replan_cooldown_s", 2.0);
    rolling_execute_single_visible_confidence_ =
      declare_parameter<double>("rolling_execute_single_visible_confidence", 0.88);
    rolling_execute_min_travel_m_ =
      declare_parameter<double>("rolling_execute_min_travel_m", 0.85);
    min_search_scan_fraction_before_execute_ =
      declare_parameter<double>("min_search_scan_fraction_before_execute", 0.25);
    target_observation_min_confidence_ =
      declare_parameter<double>("target_observation_min_confidence", 0.45);
    target_observation_confirm_count_ =
      declare_parameter<int>("target_observation_confirm_count", 2);
    target_observation_stale_timeout_s_ =
      declare_parameter<double>("target_observation_stale_timeout_s", 12.0);
    target_observation_max_distance_m_ =
      declare_parameter<double>("target_observation_max_distance_m", 3.2);
    target_observation_edge_margin_px_ =
      declare_parameter<int>("target_observation_edge_margin_px", 20);
    target_observation_image_width_px_ =
      declare_parameter<int>("target_observation_image_width_px", 640);
    target_association_max_yaw_deg_ =
      declare_parameter<double>("target_association_max_yaw_deg", 10.0);
    target_association_max_distance_error_m_ =
      declare_parameter<double>("target_association_max_distance_error_m", 1.8);
    target_association_max_world_error_m_ =
      declare_parameter<double>("target_association_max_world_error_m", 0.55);
    target_association_yaw_weight_ =
      declare_parameter<double>("target_association_yaw_weight", 1.0);
    target_association_distance_weight_ =
      declare_parameter<double>("target_association_distance_weight", 0.35);
    target_association_history_bonus_ =
      declare_parameter<double>("target_association_history_bonus", 1.5);
    route_column_match_y_tolerance_m_ =
      declare_parameter<double>("route_column_match_y_tolerance_m", 0.50);
    route_column_match_world_tolerance_m_ =
      declare_parameter<double>("route_column_match_world_tolerance_m", 0.90);
    route_column_match_yaw_tolerance_deg_ =
      declare_parameter<double>("route_column_match_yaw_tolerance_deg", 22.0);
    sim_random_seed_ = declare_parameter<int>("sim_random_seed", 2026);
    sim_randomize_balls_ = declare_parameter<bool>("sim_randomize_balls", true);
    search_all_fallback_timeout_s_ =
      declare_parameter<double>("search_all_fallback_timeout_s", 12.0);
    execute_plan_standoff_m_ = declare_parameter<double>("execute_plan_standoff_m", 0.42);
    require_search_waypoints_before_targets_ =
      declare_parameter<bool>("require_search_waypoints_before_targets", true);
    field_width_m_ = declare_parameter<double>("field_width_m", 4.0);
    field_height_m_ = declare_parameter<double>("field_height_m", 4.0);
    field_min_x_m_ = declare_parameter<double>("field_min_x_m", 0.0);
    field_min_y_m_ = declare_parameter<double>("field_min_y_m", 0.0);
    field_max_x_m_ = declare_parameter<double>("field_max_x_m", field_width_m_);
    field_max_y_m_ = declare_parameter<double>("field_max_y_m", field_height_m_);
    boundary_margin_m_ = declare_parameter<double>("boundary_margin_m", 0.15);
    exit_x_m_ = declare_parameter<double>("exit_x_m", 0.15);
    exit_y_m_ = declare_parameter<double>("exit_y_m", 3.85);
    exit_heading_deg_ = declare_parameter<double>("exit_heading_deg", 90.0);
    rear_leg_offset_m_ = declare_parameter<double>("rear_leg_offset_m", 0.35);
    exit_clearance_m_ = declare_parameter<double>("exit_clearance_m", 0.10);
    stand_ready_height_threshold_m_ =
      declare_parameter<double>("stand_ready_height_threshold_m", 0.18);
    grid_x_ = declare_parameter<std::vector<double>>(
      "grid_x_centers_m", {0.50, 1.50, 2.50, 3.50});
    grid_y_ = declare_parameter<std::vector<double>>(
      "grid_y_centers_m", {0.32, 1.32, 2.32, 3.32});
    fixed_blue_indices_ = declare_parameter<std::vector<int64_t>>(
      "fixed_blue_indices", {11, 14, 15});
    search_relocalization_first_ = declare_parameter<bool>("search_relocalization_first", true);
    search_anchor_offset_m_ = declare_parameter<double>("search_anchor_offset_m", 0.45);
    blue_body_clearance_m_ = declare_parameter<double>("blue_body_clearance_m", 0.52);
    strike_corridor_clearance_m_ = declare_parameter<double>("strike_corridor_clearance_m", 0.48);
    target_blue_exclusion_radius_m_ =
      declare_parameter<double>("target_blue_exclusion_radius_m", 0.45);
    strike_lineup_distance_m_ = declare_parameter<double>("strike_lineup_distance_m", 0.45);
    search_spin_pause_waypoint_count_ =
      declare_parameter<int>("search_spin_pause_waypoint_count", 3);
    search_waypoint_batch_size_ =
      declare_parameter<int>("search_waypoint_batch_size", 5);
    search_front_probe_distance_m_ =
      declare_parameter<double>("search_front_probe_distance_m", 0.30);
    search_front_probe_enabled_ =
      declare_parameter<bool>("search_front_probe_enabled", true);
    search_start_escape_enabled_ =
      declare_parameter<bool>("search_start_escape_enabled", true);
    search_start_escape_dx_m_ =
      declare_parameter<double>("search_start_escape_dx_m", -0.55);
    search_start_escape_dy_m_ =
      declare_parameter<double>("search_start_escape_dy_m", 0.25);
    search_start_corner_x_m_ =
      declare_parameter<double>("search_start_corner_x_m", 2.80);
    search_start_corner_y_m_ =
      declare_parameter<double>("search_start_corner_y_m", 1.60);
    search_secondary_escape_dx_m_ =
      declare_parameter<double>("search_secondary_escape_dx_m", -0.85);
    search_secondary_escape_dy_m_ =
      declare_parameter<double>("search_secondary_escape_dy_m", 0.65);
    route_bottom_corridor_clearance_m_ =
      declare_parameter<double>("route_bottom_corridor_clearance_m", 0.50);
    route_start_side_offset_m_ =
      declare_parameter<double>("route_start_side_offset_m", 0.55);
    search_initial_scan_enabled_ =
      declare_parameter<bool>("search_initial_scan_enabled", true);
    search_initial_scan_speed_radps_ =
      declare_parameter<double>("search_initial_scan_speed_radps", 0.35);
    search_initial_scan_duration_s_ =
      declare_parameter<double>("search_initial_scan_duration_s", 4.5);
    use_world_targets_ = declare_parameter<bool>("use_world_targets", false);
    world_target_strike_distance_m_ = declare_parameter<double>("world_target_strike_distance_m", 0.34);
    boundary_column_strike_distance_m_ =
      declare_parameter<double>("boundary_column_strike_distance_m", 0.55);
    exit_intermediate_y_m_ = declare_parameter<double>("exit_intermediate_y_m", 3.40);
    world_target_x_ = declare_parameter<std::vector<double>>(
      "world_target_x_m", {0.8, 2.0, 3.2, -0.4});
    world_target_y_ = declare_parameter<std::vector<double>>(
      "world_target_y_m", {1.34, 2.18, 3.02, 3.86});
    world_target_approach_x_ = declare_parameter<std::vector<double>>(
      "world_target_approach_x_m", {0.8, 2.0, 2.82, 0.35});
    world_target_approach_y_ = declare_parameter<std::vector<double>>(
      "world_target_approach_y_m", {0.94, 1.78, 3.02, 3.56});
    world_target_strike_yaw_deg_ = declare_parameter<std::vector<double>>(
      "world_target_strike_yaw_deg", {90.0, 90.0, 0.0, 180.0});
    load_world_targets();

    const auto qos_depth = declare_parameter<int>("state_machine_qos_depth", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, qos_depth);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, qos_depth);
    success_pub_ = create_publisher<std_msgs::msg::String>(success_topic_, qos_depth);
    planner_command_pub_ = create_publisher<std_msgs::msg::String>(planner_command_topic_, qos_depth);
    auto node_handle = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    robot_ = backend == "real" ?
      std::static_pointer_cast<RobotInterface>(std::make_shared<RobotInterfaceReal>(node_handle)) :
      std::static_pointer_cast<RobotInterface>(std::make_shared<RobotInterfaceSim>(node_handle));
    RCLCPP_INFO(get_logger(), "state machine using robot interface: %s", robot_->backend_name().c_str());

    vision_sub_ = create_subscription<wild_glint_hunt::msg::VisionBallArray>(
      vision_topic_, qos_depth, [this](const wild_glint_hunt::msg::VisionBallArray::SharedPtr msg) {
        vision_ = *msg;
        last_vision_time_ = now();
      });
    danger_sub_ = create_subscription<std_msgs::msg::Bool>(
      danger_topic_, qos_depth, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        danger_active_ = msg->data;
      });
    planner_status_sub_ = create_subscription<std_msgs::msg::String>(
      planner_status_topic_, qos_depth, [this](const std_msgs::msg::String::SharedPtr msg) {
        planner_status_ = msg->data;
      });
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, qos_depth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(timer_period_ms_), [this]() { step(); });
  }

private:
  void step()
  {
    update_odometry_from_interface();
    publish_status();
    if (task_start_time_.nanoseconds() == 0) {
      task_start_time_ = now();
      state_enter_time_ = now();
    }
    if (handle_global_timeout()) {
      return;
    }
    if (handle_state_timeout()) {
      return;
    }
    const bool can_interrupt =
      state_ == HuntState::FOLLOW_ROUTE || state_ == HuntState::SEARCH_ALL ||
      state_ == HuntState::EXECUTE_PLAN || state_ == HuntState::SEARCH || state_ == HuntState::ALIGN;
    const bool route_transit_phase =
      state_ == HuntState::FOLLOW_ROUTE;
    if (can_interrupt && !route_transit_phase && danger_active_ && !avoidance_active_) {
      previous_state_ = state_;
      avoidance_active_ = true;
      send_planner_command("AVOID");
      RCLCPP_INFO(get_logger(), "danger warning: pause and avoid");
      return;
    }
    if (avoidance_active_) {
      if (planner_status_ == "AVOID_DONE") {
        avoidance_active_ = false;
        planner_status_.clear();
        state_ = previous_state_;
        RCLCPP_INFO(get_logger(), "avoidance done: resume %s", to_string(state_).c_str());
        if (state_ == HuntState::ALIGN) {
          if (route_fixed_strategy_enabled_ && route_column_index_ < route_targets_.size()) {
            send_target_command();
          } else if (use_world_targets_ && world_target_index_ < world_targets_.size()) {
            send_world_target_align(world_targets_[world_target_index_]);
          } else if (executing_recorded_plan_) {
            send_recorded_target_align();
          } else {
            send_target_command();
          }
        } else if (state_ == HuntState::EXECUTE_PLAN) {
          send_recorded_target_path();
        } else if (state_ == HuntState::FOLLOW_ROUTE) {
          route_path_sent_ = false;
          route_face_sent_ = false;
          route_target_sent_ = false;
          route_phase_start_time_ = now();
        } else if (state_ == HuntState::SEARCH) {
          if (use_world_targets_ && world_target_index_ < world_targets_.size()) {
            send_world_target_path(world_targets_[world_target_index_]);
            world_target_sent_ = true;
          } else {
            send_search_path();
          }
        } else if (state_ == HuntState::SEARCH_ALL) {
          send_search_all_path();
        }
      }
      return;
    }

    switch (state_) {
      case HuntState::INIT:
        if (!stand_ready()) {
          if (robot_) {
            robot_->send_velocity(0.0, 0.0);
          }
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "waiting for stand-ready height: current=%.3f threshold=%.3f",
            odom_.pose.pose.position.z, stand_ready_height_threshold_m_);
          send_planner_command("STOP");
          return;
        }
        if (route_fixed_strategy_enabled_) {
          initialize_route_targets();
          route_nominal_heading_deg_ = route_forward_heading_deg_;
          transition(HuntState::FOLLOW_ROUTE, "initialized fixed route");
          route_phase_ = RoutePhase::START_TURN;
          route_phase_start_time_ = now();
          route_path_sent_ = false;
          route_face_sent_ = false;
          route_target_sent_ = false;
          break;
        }
        transition(search_all_before_strike_ ? HuntState::SEARCH_ALL : HuntState::SEARCH, "initialized");
        if (use_world_targets_ && !world_targets_.empty()) {
          send_world_target_path(world_targets_[world_target_index_]);
          world_target_sent_ = true;
        } else if (search_all_before_strike_) {
          send_search_all_path();
        } else {
          send_search_path();
        }
        break;
      case HuntState::FOLLOW_ROUTE:
        handle_follow_route();
        break;
      case HuntState::SEARCH_ALL:
        handle_search_all();
        break;
      case HuntState::EXECUTE_PLAN:
        handle_execute_plan();
        break;
      case HuntState::SEARCH:
        if (use_world_targets_) {
          handle_world_targets();
        } else {
          handle_search();
        }
        break;
      case HuntState::ALIGN:
        handle_align();
        break;
      case HuntState::STRIKE:
        handle_strike();
        break;
      case HuntState::VERIFY:
        handle_verify();
        break;
      case HuntState::EXIT:
        handle_exit();
        break;
      case HuntState::FINISH:
        publish_success();
        break;
      default:
        break;
    }
  }

  void handle_search()
  {
    if (search_initial_scan_enabled_ && !search_initial_scan_completed_) {
      if (!search_initial_scan_sent_) {
        search_initial_scan_sent_ = true;
        send_planner_command(
          "SCAN:" + std::to_string(search_initial_scan_speed_radps_) + ":" +
          std::to_string(search_initial_scan_duration_s_));
        return;
      }
      if (planner_status_ == "SCAN_DONE") {
        planner_status_.clear();
        search_initial_scan_completed_ = true;
        send_search_path();
      }
      return;
    }
    prune_stale_observed_targets();
    record_visible_orange_targets();
    if (try_execute_immediate_visible_target()) {
      return;
    }
    const bool scan_enough =
      !require_search_waypoints_before_targets_ ||
      search_scan_progress() >= min_search_scan_fraction_before_execute_;
    if (scan_enough && ready_for_rolling_execution()) {
      if (last_rolling_plan_time_.nanoseconds() > 0 &&
        (now() - last_rolling_plan_time_).seconds() < rolling_replan_cooldown_s_)
      {
        return;
      }
      planner_status_.clear();
      send_planner_command("STOP");
      build_execution_plan();
      if (!execution_plan_.empty()) {
        last_rolling_plan_time_ = now();
        transition(HuntState::EXECUTE_PLAN, "rolling map has reliable orange targets");
        send_recorded_target_path();
        return;
      }
    }
    if (search_started_time_.nanoseconds() > 0 &&
      (now() - search_started_time_).seconds() >= rolling_search_timeout_s_ &&
      ready_for_rolling_execution())
    {
      planner_status_.clear();
      send_planner_command("STOP");
      build_execution_plan();
      if (!execution_plan_.empty()) {
        last_rolling_plan_time_ = now();
        transition(HuntState::EXECUTE_PLAN, "rolling search timeout; execute known targets");
        send_recorded_target_path();
        return;
      }
    }
    if (require_search_waypoints_before_targets_ && !search_path_completed_once_) {
      if (planner_status_ == "WAYPOINTS_DONE") {
        search_path_completed_once_ = true;
        planner_status_.clear();
        if (search_started_time_.nanoseconds() == 0) {
          search_started_time_ = now();
        }
      } else {
        return;
      }
    }
    if (planner_status_ == "WAYPOINTS_DONE") {
      planner_status_.clear();
      send_search_path();
    }
  }

  void handle_follow_route()
  {
    if (route_column_index_ >= route_targets_.size()) {
      route_phase_ = RoutePhase::MOVE_EXIT;
    }
    if (handle_route_phase_timeout()) {
      return;
    }

    switch (route_phase_) {
      case RoutePhase::START_TURN:
        if (std::abs(route_start_turn_ccw_deg_) < 1.0e-3) {
          route_phase_ = RoutePhase::ESCAPE_START;
          route_phase_start_time_ = now();
          break;
        }
        if (!route_face_sent_) {
          send_face_command(route_nominal_heading_deg_ + route_start_turn_ccw_deg_);
          route_face_sent_ = true;
          route_phase_start_time_ = now();
        } else if (planner_status_ == "FACE_DONE") {
          planner_status_.clear();
          route_face_sent_ = false;
          route_phase_ = RoutePhase::ESCAPE_START;
          route_phase_start_time_ = now();
        }
        break;
      case RoutePhase::ESCAPE_START:
        if (!route_path_sent_) {
          send_planner_command("ROUTE:ESCAPE_START");
          route_path_sent_ = true;
          route_phase_start_time_ = now();
        } else if (planner_status_ == "WAYPOINTS_DONE") {
          planner_status_.clear();
          route_path_sent_ = false;
          route_phase_ = RoutePhase::MOVE_TO_OBSERVE;
          route_phase_start_time_ = now();
        }
        break;
      case RoutePhase::MOVE_TO_OBSERVE:
        if (!route_path_sent_) {
          send_current_aisle_path();
          route_path_sent_ = true;
          route_phase_start_time_ = now();
        } else if (route_aisle_endpoint_reached() || planner_status_ == "WAYPOINTS_DONE") {
          if (planner_status_ != "WAYPOINTS_DONE") {
            send_planner_command("STOP");
            RCLCPP_WARN(
              get_logger(),
              "route aisle endpoint for %s accepted within route tolerance before planner final tolerance",
              current_route_column().c_str());
          }
          planner_status_.clear();
          if (!route_aisle_endpoint_reached()) {
            RCLCPP_WARN(
              get_logger(),
              "route aisle endpoint not reached for %s; resending current aisle path",
              current_route_column().c_str());
            route_path_sent_ = false;
            route_phase_start_time_ = now();
            break;
          }
          mark_current_aisle_reached();
          route_path_sent_ = false;
          // Fixed-route mode already knows the target cell for each column.
          // Do not do an in-place FACE_COLUMN scan at the R1/R2 junction: the
          // locomotion yaw controller can choose the long rotation and spin
          // near R1C3.  Go straight into the lineup/align path so the turn is
          // executed as normal waypoint tracking.
          if (start_direct_route_target_align("route aisle endpoint reached")) {
            return;
          }
          route_phase_ = RoutePhase::FACE_COLUMN;
          route_phase_start_time_ = now();
        }
        break;
      case RoutePhase::FACE_COLUMN:
        if (!route_face_sent_) {
          send_face_command(column_face_yaw_deg(current_route_column()));
          route_face_sent_ = true;
          route_phase_start_time_ = now();
        } else if (planner_status_ == "FACE_DONE") {
          planner_status_.clear();
          route_face_sent_ = false;
          route_phase_ = RoutePhase::ACQUIRE_TARGET;
          route_phase_start_time_ = now();
        }
        break;
      case RoutePhase::ACQUIRE_TARGET:
      {
        const auto target = choose_column_target(current_route_column());
        if (target.has_value()) {
          current_target_ = *target;
          const std::string associated_id =
            current_target_.id.rfind("R", 0) == 0 || is_visual_route_target_id(current_target_.id) ?
            current_target_.id : stable_target_id(current_target_);
          if (!associated_id.empty()) {
            current_target_.id = associated_id;
          } else {
            current_target_.id = current_route_column();
          }
          route_target_sent_ = false;
          route_phase_ = RoutePhase::WAIT_ALIGN;
          route_reacquire_retry_count_ = 0;
          route_phase_start_time_ = now();
          transition(HuntState::ALIGN, "route target acquired");
          send_route_target_align();
          return;
        }
        if ((now() - route_phase_start_time_).seconds() >= route_column_visual_timeout_s_) {
          if (route_reacquire_retry_count_ < route_reacquire_retry_limit_) {
            ++route_reacquire_retry_count_;
            RCLCPP_WARN(
              get_logger(), "route column %s target not detected; retry observation %d/%d",
              current_route_column().c_str(), route_reacquire_retry_count_,
              route_reacquire_retry_limit_);
            route_phase_ = RoutePhase::FACE_COLUMN;
            route_face_sent_ = false;
            route_phase_start_time_ = now();
          } else if (allow_route_pose_fallback_target(current_route_column())) {
            current_target_ = fallback_route_target_for_column(current_route_column());
            current_target_visible_before_strike_ = false;
            route_target_sent_ = false;
            route_phase_ = RoutePhase::WAIT_ALIGN;
            route_reacquire_retry_count_ = 0;
            route_phase_start_time_ = now();
            transition(HuntState::ALIGN, "route target fallback to fixed column pose");
            send_route_target_align();
            return;
          } else {
            RCLCPP_WARN(
              get_logger(), "route column %s target not detected after retries; skip and continue S-curve",
              current_route_column().c_str());
            completed_ids_.push_back(current_route_column());
            advance_route_column(false);
          }
        }
        break;
      }
      case RoutePhase::MOVE_EXIT:
        transition(HuntState::EXIT, "route columns completed");
        send_exit_path();
        break;
      case RoutePhase::RETREAT_AFTER_STRIKE:
        if (!route_path_sent_) {
          executePostStrikeRetreat();
          route_path_sent_ = true;
          route_phase_start_time_ = now();
        } else if (route_aisle_endpoint_reached() || planner_status_ == "WAYPOINTS_DONE") {
          if (planner_status_ != "WAYPOINTS_DONE") {
            send_planner_command("STOP");
            RCLCPP_WARN(
              get_logger(),
              "%s retreat accepted within route tolerance before planner final tolerance",
              route_retreat_column_.c_str());
          }
          send_planner_command("BOUNDARY_RECOVERY:ON");
          planner_status_.clear();
          route_path_sent_ = false;
          route_phase_ = route_needs_transit_to_next_aisle() ?
            RoutePhase::TRANSIT_TO_NEXT_AISLE : RoutePhase::MOVE_TO_OBSERVE;
          route_phase_start_time_ = now();
          RCLCPP_INFO(
            get_logger(), "%s retreat complete; resume fixed route transfer",
            route_retreat_column_.c_str());
        }
        break;
      case RoutePhase::TRANSIT_TO_NEXT_AISLE:
        // Do not pre-face the third aisle with a fixed absolute yaw.  This was
        // the source of occasional 360-degree spins at tight junctions.  The
        // following SEARCH waypoint segment constrains the C1/C2 aisle entry
        // directly, so heading correction can happen while moving.
        if (!route_path_sent_) {
          send_next_aisle_transit_path();
          route_path_sent_ = true;
          route_phase_start_time_ = now();
        } else if (route_aisle_endpoint_reached() || planner_status_ == "WAYPOINTS_DONE") {
          if (planner_status_ != "WAYPOINTS_DONE") {
            send_planner_command("STOP");
            RCLCPP_WARN(
              get_logger(),
              "forced aisle junction for %s accepted within route tolerance before planner final tolerance",
              current_route_column().c_str());
          }
          if (!route_aisle_endpoint_reached()) {
            RCLCPP_WARN(
              get_logger(),
              "forced aisle junction for %s not reached; resending transit path",
              current_route_column().c_str());
            planner_status_.clear();
            route_path_sent_ = false;
            route_phase_start_time_ = now();
            break;
          }
          planner_status_.clear();
          route_path_sent_ = false;
          route_phase_ = current_route_column() == "C2" ?
            RoutePhase::FACE_COLUMN : RoutePhase::MOVE_TO_OBSERVE;
          route_transit_face_done_ = false;
          route_phase_start_time_ = now();
        } else if (route_transit_junction_overshot()) {
          RCLCPP_WARN(
            get_logger(), "overshot forced aisle junction for %s; backing to junction",
            current_route_column().c_str());
          route_path_sent_ = false;
          route_phase_start_time_ = now();
        }
        break;
      case RoutePhase::FACE_FORWARD:
        if (!route_face_sent_) {
          send_face_command(route_nominal_heading_deg_);
          route_face_sent_ = true;
          route_phase_start_time_ = now();
        } else if (planner_status_ == "FACE_DONE" ||
          (now() - route_phase_start_time_).seconds() >= route_face_timeout_s_)
        {
          planner_status_.clear();
          route_face_sent_ = false;
          route_phase_ = route_column_index_ >= route_targets_.size() ?
            RoutePhase::MOVE_EXIT :
            (route_needs_transit_to_next_aisle() ? RoutePhase::TRANSIT_TO_NEXT_AISLE : RoutePhase::MOVE_TO_OBSERVE);
        }
        break;
      default:
        break;
    }
  }

  void handle_search_all()
  {
    record_visible_orange_targets();
    if (executing_recorded_plan_) {
      return;
    }
    if (observed_targets_.size() >= static_cast<size_t>(required_ball_count_)) {
      planner_status_.clear();
      send_planner_command("STOP");
      build_execution_plan();
      transition(HuntState::EXECUTE_PLAN, "recorded required orange targets; execute plan");
      send_recorded_target_path();
      return;
    }
    if (backend_ == "sim" && sim_use_ground_truth_layout_ &&
      (now() - search_all_start_time_).seconds() >= search_all_fallback_timeout_s_)
    {
      load_simulated_ground_truth_targets();
      if (!observed_targets_.empty()) {
        planner_status_.clear();
        send_planner_command("STOP");
        build_execution_plan();
        transition(HuntState::EXECUTE_PLAN, "sim fallback: load ground truth orange plan");
        send_recorded_target_path();
        return;
      }
    }
    if (planner_status_ != "WAYPOINTS_DONE") {
      return;
    }
    planner_status_.clear();
    if (observed_targets_.empty()) {
      RCLCPP_ERROR(get_logger(), "search-all completed but no orange targets were recorded");
      send_search_all_path();
      return;
    }
    build_execution_plan();
    transition(HuntState::EXECUTE_PLAN, "search-all completed; execute recorded orange plan");
    send_recorded_target_path();
  }

  void handle_execute_plan()
  {
    record_visible_orange_targets();
    if (completed_balls_ >= static_cast<size_t>(required_ball_count_) ||
      execution_index_ >= execution_plan_.size())
    {
      if (completed_balls_ >= static_cast<size_t>(required_ball_count_)) {
        transition(HuntState::EXIT, "recorded orange plan completed");
        send_exit_path();
      } else {
        executing_recorded_plan_ = false;
        execution_path_sent_ = false;
        search_path_completed_once_ = false;
        transition(HuntState::SEARCH, "known targets exhausted; resume rolling search");
        send_search_path();
      }
      return;
    }
    if (!execution_path_sent_) {
      send_recorded_target_path();
      return;
    }
    if (planner_status_ == "WAYPOINTS_DONE") {
      planner_status_.clear();
      transition(HuntState::ALIGN, "recorded target approach reached");
      send_recorded_target_align();
    }
  }

  void handle_world_targets()
  {
    if (world_target_index_ >= world_targets_.size()) {
      transition(HuntState::EXIT, "all configured orange targets visited");
      send_exit_path();
      return;
    }
    if (!world_target_sent_) {
      send_world_target_path(world_targets_[world_target_index_]);
      world_target_sent_ = true;
      return;
    }
    if (planner_status_ == "WAYPOINTS_DONE") {
      planner_status_.clear();
      transition(HuntState::ALIGN, "world target approach reached");
      send_world_target_align(world_targets_[world_target_index_]);
    }
  }

  void handle_align()
  {
    if (route_fixed_strategy_enabled_ && state_ == HuntState::ALIGN && route_column_index_ < route_targets_.size()) {
      if (planner_status_ == "TARGET_ALIGNED") {
        planner_status_.clear();
        if (strike_require_visual_confirmation_ && !target_available_for_strike(current_target_.id)) {
          route_phase_ = RoutePhase::ACQUIRE_TARGET;
          route_target_sent_ = false;
          route_reacquire_retry_count_ = std::min(
            route_reacquire_retry_count_ + 1, route_reacquire_retry_limit_);
          transition(HuntState::FOLLOW_ROUTE, "route target aligned without matching orange visual");
          route_phase_start_time_ = now();
          return;
        }
        transition(HuntState::STRIKE, "route target aligned");
        send_planner_command("STRIKE");
        strike_motion_executed_ = false;
        strike_start_time_ = now();
        return;
      }
      if (planner_status_ == "TARGET_LOST") {
        planner_status_.clear();
        if (route_reacquire_retry_count_ < route_reacquire_retry_limit_) {
          ++route_reacquire_retry_count_;
          route_phase_ = RoutePhase::ACQUIRE_TARGET;
          transition(HuntState::FOLLOW_ROUTE, "route target lost; reacquire");
          route_phase_start_time_ = now();
        } else {
          route_reacquire_retry_count_ = 0;
          route_phase_ = RoutePhase::MOVE_TO_OBSERVE;
          route_path_sent_ = false;
          route_face_sent_ = false;
          route_target_sent_ = false;
          transition(HuntState::FOLLOW_ROUTE, "route target lost; restart column");
          route_phase_start_time_ = now();
        }
        return;
      }
      if (planner_status_ == "AVOID_DONE") {
        planner_status_.clear();
        send_route_target_align();
        return;
      }
      if (planner_status_ == "WAYPOINTS_DONE" || planner_status_ == "BOUNDARY_RECOVERY_DONE") {
        planner_status_.clear();
        send_route_target_align();
        route_phase_start_time_ = now();
        return;
      }
      if (route_align_reissue_interval_s_ > 0.0 &&
        route_phase_start_time_.nanoseconds() > 0 &&
        (now() - route_phase_start_time_).seconds() >= route_align_reissue_interval_s_)
      {
        send_route_target_align();
        route_phase_start_time_ = now();
      }
      return;
    }
    if (executing_recorded_plan_) {
      if (planner_status_ == "TARGET_ALIGNED") {
        planner_status_.clear();
        if (strike_require_visual_confirmation_ && !target_available_for_strike(current_target_.id)) {
          transition(HuntState::EXECUTE_PLAN, "recorded target aligned without orange visual");
          execution_path_sent_ = false;
          return;
        }
        transition(HuntState::STRIKE, "recorded target aligned");
        send_planner_command("STRIKE");
        strike_motion_executed_ = false;
        strike_start_time_ = now();
        return;
      }
      if (planner_status_ == "TARGET_LOST") {
        planner_status_.clear();
        transition(HuntState::EXECUTE_PLAN, "recorded target lost; re-approach");
        execution_path_sent_ = false;
      }
      return;
    }
    if (!route_fixed_strategy_enabled_ && use_world_targets_) {
      if (planner_status_ == "TARGET_ALIGNED") {
        planner_status_.clear();
        if (strike_require_visual_confirmation_ && !target_available_for_strike(current_target_.id)) {
          transition(HuntState::SEARCH, "world target aligned without orange visual");
          send_search_path();
          return;
        }
        transition(HuntState::STRIKE, "world target aligned");
        send_planner_command("STRIKE");
        strike_motion_executed_ = false;
        strike_start_time_ = now();
      }
      return;
    }
    if (auto target = choose_orange_target()) {
      current_target_ = *target;
      current_target_.id = stable_target_id(current_target_);
    }
    if (planner_status_ == "TARGET_ALIGNED") {
      planner_status_.clear();
      if (strike_require_visual_confirmation_ && !target_available_for_strike(current_target_.id)) {
        transition(HuntState::SEARCH, "target aligned without orange visual");
        send_search_path();
        return;
      }
      transition(HuntState::STRIKE, "target aligned");
      send_planner_command("STRIKE");
      strike_motion_executed_ = false;
      strike_start_time_ = now();
      return;
    }
    if (planner_status_ == "TARGET_LOST") {
      planner_status_.clear();
      transition(HuntState::SEARCH, "target lost during alignment");
      send_search_path();
    }
  }

  void handle_strike()
  {
    if (single_strike_timeout_s_ > 0.0 && strike_start_time_.nanoseconds() > 0 &&
      (now() - strike_start_time_).seconds() >= single_strike_timeout_s_)
    {
      planner_status_.clear();
      transition(HuntState::VERIFY, "single strike timeout");
      verify_start_time_ = now();
      return;
    }
    if (planner_status_ == "STRIKE_DONE") {
      planner_status_.clear();
      strike_motion_executed_ = true;
      transition(HuntState::VERIFY, "strike motion done");
      verify_start_time_ = now();
    }
  }

  void handle_verify()
  {
    const bool light_touch_elapsed =
      strike_success_check_time_s_ > 0.0 &&
      (now() - verify_start_time_).seconds() >= strike_success_check_time_s_;
    if (strike_motion_executed_ && light_touch_elapsed &&
      route_fixed_strategy_enabled_ && current_target_.id.rfind("R", 0) == 0)
    {
      // Fixed-route grid targets are deliberately struck from known safe poses.
      // Gazebo RGB can lose the ball at the instant of contact, so after a real
      // STRIKE_DONE we advance after the short light-touch window instead of
      // forcing a visual re-confirmation.
      handle_strike_success("fixed-route light-touch success");
      return;
    }
    if (strike_motion_executed_ && strike_light_touch_ && light_touch_elapsed) {
      // Light-touch success is only valid after a real strike command has completed.
      const bool current_target_still_visible =
        is_visual_route_target_id(current_target_.id) ?
        orange_visible_after_strike() : find_ball_by_id(current_target_.id).has_value();
      if (current_target_still_visible) {
        handle_strike_success("light-touch visual success");
        return;
      }
    }
    if (strike_motion_executed_ && sim_assume_strike_success_) {
      const auto visible = find_ball_by_id(current_target_.id);
      if (!visible.has_value()) {
        handle_strike_success("simulated strike success");
      } else {
        const double dx = static_cast<double>(visible->pixel_x - current_target_initial_x_);
        const double dy = static_cast<double>(visible->pixel_y - current_target_initial_y_);
        if (std::hypot(dx, dy) >= strike_success_pixel_shift_) {
          handle_strike_success("simulated strike success");
        }
      }
      if ((now() - verify_start_time_).seconds() >= strike_verify_timeout_s_) {
        handle_strike_success("simulated strike timeout accepted");
      }
      return;
    }
    const auto visible = find_ball_by_id(current_target_.id);
    bool success = false;
    if (visible) {
      const double dx = static_cast<double>(visible->pixel_x - current_target_initial_x_);
      const double dy = static_cast<double>(visible->pixel_y - current_target_initial_y_);
      const double distance_delta = std::abs(
        static_cast<double>(visible->distance_m) - current_target_initial_distance_m_);
      success =
        std::hypot(dx, dy) >= strike_success_pixel_shift_ ||
        distance_delta >= strike_success_distance_shift_m_;
    }
    const bool timed_out = (now() - verify_start_time_).seconds() >= strike_verify_timeout_s_;
    if (strike_motion_executed_ && success) {
      handle_strike_success("vision strike success");
      return;
    }
    if (strike_motion_executed_ && strike_accept_pose_contact_success_ && pose_contact_success())
    {
      handle_strike_success("estimated-pose contact success");
      return;
    }
      if (timed_out) {
      if (strike_attempts_ < max_strike_retries_) {
        ++strike_attempts_;
      transition(HuntState::ALIGN, "strike retry");
        if (route_fixed_strategy_enabled_) {
          send_route_target_align();
        } else {
          send_target_command();
        }
      } else {
        strike_motion_executed_ = false;
        completed_ids_.push_back(current_target_.id);
        RCLCPP_ERROR(get_logger(), "strike failed: skip target=%s", current_target_.id.c_str());
        if (route_fixed_strategy_enabled_) {
          advance_route_column(false);
          transition(HuntState::FOLLOW_ROUTE, "skip failed route target");
        } else if (executing_recorded_plan_) {
          ++execution_index_;
          execution_path_sent_ = false;
          transition(HuntState::EXECUTE_PLAN, "skip abnormal recorded target");
        } else {
          transition(HuntState::SEARCH, "skip abnormal target");
          send_search_path();
        }
      }
    }
  }

  void handle_strike_success(const std::string & reason)
  {
    strike_motion_executed_ = false;
    send_planner_command("STOP");
    planner_status_.clear();
    ++completed_balls_;
    std::string completed_id = current_target_.id;
    if (route_fixed_strategy_enabled_ && route_column_index_ < route_targets_.size()) {
      if (completed_id.rfind("R", 0) != 0) {
        completed_id = route_targets_[route_column_index_].id;
      }
      advance_route_column(true);
    } else if (use_world_targets_ && world_target_index_ < world_targets_.size()) {
      completed_id = world_targets_[world_target_index_].id;
      ++world_target_index_;
      world_target_sent_ = false;
    } else if (executing_recorded_plan_) {
      ++execution_index_;
      execution_path_sent_ = false;
    }
    completed_ids_.push_back(completed_id);
    RCLCPP_INFO(
      get_logger(), "%s: count=%zu target=%s", reason.c_str(), completed_balls_,
      completed_id.c_str());
    if (completed_balls_ >= static_cast<size_t>(required_ball_count_)) {
      transition(HuntState::EXIT, "all targets completed");
      send_exit_path();
    } else {
      if (route_fixed_strategy_enabled_) {
        transition(HuntState::FOLLOW_ROUTE, "resume fixed route");
        return;
      }
      if (executing_recorded_plan_) {
        if (execution_index_ < execution_plan_.size()) {
          transition(HuntState::EXECUTE_PLAN, "continue recorded execution plan");
        } else {
          executing_recorded_plan_ = false;
          transition(HuntState::SEARCH, "continue rolling search for remaining targets");
          send_search_path();
        }
      } else {
        transition(HuntState::SEARCH, "continue search");
        send_search_path();
      }
    }
  }

  void handle_exit()
  {
    if (!exit_sent_) {
      send_exit_path();
    }
    if (!exit_face_sent_ && (planner_status_ == "WAYPOINTS_DONE" || rear_legs_clear_exit())) {
      planner_status_.clear();
      send_face_command(exit_heading_deg_);
      exit_face_sent_ = true;
      return;
    }
    if (exit_face_sent_ && planner_status_ == "FACE_DONE") {
      transition(HuntState::FINISH, "exit reached");
      send_planner_command("STOP");
    }
  }

  void initialize_route_targets()
  {
    route_targets_.clear();
    route_column_index_ = 0;
    route_aisle_endpoint_reached_.fill(false);
    route_have_current_aisle_endpoint_ = false;
    if (!route_columns_order_.empty()) {
      route_enable_columns_.clear();
      for (const auto column : route_columns_order_) {
        route_enable_columns_.push_back("C" + std::to_string(column));
      }
    }
    for (const auto & column : route_enable_columns_) {
      route_targets_.push_back(make_route_column_target(column));
    }
  }

  BallTarget make_route_column_target(const std::string & column) const
  {
    BallTarget target;
    target.id = column;
    const auto col_index = route_column_index_from_id(column);
    const size_t col = col_index.value_or(0);
    const double y_between_r1_r2 = grid_y_.size() >= 2 ? 0.5 * (grid_y_.at(2) + grid_y_.at(3)) : 0.0;
    const double y_between_r2_r3 = grid_y_.size() >= 3 ? 0.5 * (grid_y_.at(1) + grid_y_.at(2)) : y_between_r1_r2;
    const double y_between_r3_r4 = grid_y_.size() >= 4 ? 0.5 * (grid_y_.at(0) + grid_y_.at(1)) : 0.0;
    if (column == "C4") {
      target.x = grid_x_.at(col);
      target.y = y_between_r1_r2;
      target.approach_x = 0.5 * (grid_x_.at(2) + grid_x_.at(3));
      target.approach_y = y_between_r1_r2;
      target.strike_yaw_rad = 0.0;
    } else if (column == "C3") {
      target.x = grid_x_.at(col);
      target.y = y_between_r2_r3;
      target.approach_x = 0.5 * (grid_x_.at(1) + grid_x_.at(2));
      target.approach_y = y_between_r2_r3;
      target.strike_yaw_rad = 0.0;
    } else if (column == "C2") {
      target.x = grid_x_.at(col);
      target.y = y_between_r3_r4;
      target.approach_x = 0.5 * (grid_x_.at(0) + grid_x_.at(1));
      target.approach_y = y_between_r3_r4;
      target.strike_yaw_rad = 0.0;
    } else {
      target.x = grid_x_.at(col);
      target.y = y_between_r3_r4;
      target.approach_x = 0.5 * (grid_x_.at(0) + grid_x_.at(1));
      target.approach_y = y_between_r3_r4;
      target.strike_yaw_rad = M_PI;
    }
    return target;
  }

  std::string current_route_column() const
  {
    if (route_column_index_ >= route_targets_.size()) {
      return "";
    }
    return route_targets_[route_column_index_].id;
  }

  size_t current_route_aisle_index() const
  {
    const auto column = current_route_column();
    if (column == "C4") {
      return 0;
    }
    if (column == "C3") {
      return 1;
    }
    return 2;
  }

  std::vector<AisleSegment> compute_route_aisles() const
  {
    const double aisle_1_x =
      0.5 * (grid_x_.at(2) + grid_x_.at(3)) + route_c34_aisle_x_offset_m_;  // C3/C4
    const double aisle_2_x =
      0.5 * (grid_x_.at(1) + grid_x_.at(2)) + route_c23_aisle_x_offset_m_;  // C2/C3
    const double aisle_3_x = 0.5 * (grid_x_.at(0) + grid_x_.at(1));  // C1/C2
    const double y_start = std::clamp(
      grid_y_.at(0) - 0.30,
      field_min_y_m_ + boundary_margin_m_,
      field_max_y_m_ - boundary_margin_m_);
    const double y_top = std::clamp(
      0.5 * (grid_y_.at(2) + grid_y_.at(3)),
      field_min_y_m_ + boundary_margin_m_,
      field_max_y_m_ - boundary_margin_m_);
    const double c3_exit_y = std::isfinite(route_c3_aisle_exit_y_m_) ?
      std::clamp(
        route_c3_aisle_exit_y_m_,
        field_min_y_m_ + boundary_margin_m_,
        field_max_y_m_ - boundary_margin_m_) :
      y_start;
    return {
      AisleSegment{"C4", {"C4"}, clamp_waypoint({aisle_1_x, y_start}), clamp_waypoint({aisle_1_x, y_top})},
      AisleSegment{"C3", {"C3"}, clamp_waypoint({aisle_2_x, y_top}), clamp_waypoint({aisle_2_x, c3_exit_y})},
      AisleSegment{"C2", {"C2", "C1"}, clamp_waypoint({aisle_3_x, y_start}), clamp_waypoint({aisle_3_x, y_top})}
    };
  }

  AisleSegment current_route_aisle() const
  {
    const auto aisles = compute_route_aisles();
    const size_t index = std::min(current_route_aisle_index(), aisles.size() - 1);
    return aisles[index];
  }

  void advance_route_column(bool after_strike)
  {
    const std::string completed_column = current_route_column();
    const size_t completed_aisle = current_route_aisle_index();
    if (route_column_index_ < route_targets_.size()) {
      ++route_column_index_;
    }
    if (after_strike &&
      (completed_column == "C4" || completed_column == "C3" || completed_column == "C2" ||
      completed_column == "C1"))
    {
      route_phase_ = RoutePhase::RETREAT_AFTER_STRIKE;
      route_retreat_column_ = completed_column;
    } else if (route_column_index_ >= route_targets_.size()) {
      route_phase_ = RoutePhase::MOVE_EXIT;
    } else if (completed_column == "C2" && current_route_column() == "C1") {
      route_phase_ = RoutePhase::MOVE_TO_OBSERVE;
    } else {
      route_phase_ = completed_aisle != current_route_aisle_index() ?
        RoutePhase::TRANSIT_TO_NEXT_AISLE : RoutePhase::FACE_FORWARD;
    }
    route_phase_start_time_ = now();
    route_path_sent_ = false;
    route_face_sent_ = false;
    route_target_sent_ = false;
    route_transit_face_done_ = false;
    route_reacquire_retry_count_ = 0;
  }

  double column_face_yaw_deg(const std::string & column) const
  {
    double offset_deg = 0.0;
    if (!route_column_scan_offsets_deg_.empty()) {
      const size_t offset_index = static_cast<size_t>(route_reacquire_retry_count_) %
        route_column_scan_offsets_deg_.size();
      offset_deg = route_column_scan_offsets_deg_[offset_index];
    }
    if (column == "C4" || column == "C3" || column == "C2") {
      return 0.0 + offset_deg;
    }
    return 180.0 + offset_deg;
  }

  bool route_needs_transit_to_next_aisle() const
  {
    if (route_column_index_ >= route_targets_.size()) {
      return false;
    }
    const auto column = current_route_column();
    return column == "C3" || column == "C2";
  }

  void send_current_aisle_path()
  {
    std::vector<Waypoint> points;
    const auto aisle = current_route_aisle();
    // C1 is handled from the top end of the third aisle after the C2 post-strike
    // retreat. Do not drive back down to the aisle entry and then up again.
    if (current_route_column() == "C1") {
      points.push_back(aisle.exit);
    } else if (have_odom_) {
      const auto & pos = odom_.pose.pose.position;
      const bool c4_already_above_blue_cluster =
        current_route_column() == "C4" && pos.y > 0.5 * (grid_y_.at(1) + grid_y_.at(2));
      if (!c4_already_above_blue_cluster &&
        std::hypot(pos.x - aisle.entry.x, pos.y - aisle.entry.y) > route_aisle_endpoint_tolerance_m_)
      {
        points.push_back(aisle.entry);
      }
    } else {
      points.push_back(aisle.entry);
    }
    points.push_back(aisle.exit);
    if (!points.empty()) {
      route_current_aisle_endpoint_ = points.back();
      route_have_current_aisle_endpoint_ = true;
    }
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  void send_next_aisle_transit_path()
  {
    if (route_column_index_ >= route_targets_.size()) {
      return;
    }
    const auto aisles = compute_route_aisles();
    const size_t next_aisle_index = std::min(current_route_aisle_index(), aisles.size() - 1);
    const auto next_aisle = aisles[next_aisle_index];
    std::vector<Waypoint> points;
    route_transit_start_ = next_aisle.entry;
    if (have_odom_) {
      const auto & pos = odom_.pose.pose.position;
      route_transit_start_ = {pos.x, pos.y};
      // Force a stop at the junction center before entering the next vertical aisle.
      points.push_back(clamp_waypoint({next_aisle.entry.x, pos.y}));
    }
    points.push_back(next_aisle.entry);
    route_current_aisle_endpoint_ = next_aisle.entry;
    route_have_current_aisle_endpoint_ = true;
    RCLCPP_INFO(
      get_logger(), "transit to next aisle for %s via %.2f %.2f",
      current_route_column().c_str(), next_aisle.entry.x, next_aisle.entry.y);
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  bool route_transit_junction_overshot() const
  {
    if (!have_odom_ || route_column_index_ >= route_targets_.size()) {
      return false;
    }
    const auto aisles = compute_route_aisles();
    const size_t next_aisle_index = std::min(current_route_aisle_index(), aisles.size() - 1);
    const auto next_aisle = aisles[next_aisle_index];
    const auto & pos = odom_.pose.pose.position;
    const double junction_distance = std::hypot(pos.x - next_aisle.entry.x, pos.y - next_aisle.entry.y);
    if (junction_distance < route_junction_stop_tolerance_m_) {
      return false;
    }
    if (junction_distance <= route_junction_overshoot_distance_m_) {
      return false;
    }
    const double dx_total = next_aisle.entry.x - route_transit_start_.x;
    const double dy_total = next_aisle.entry.y - route_transit_start_.y;
    if (std::abs(dx_total) >= std::abs(dy_total)) {
      const bool crossed_x = (route_transit_start_.x - next_aisle.entry.x) *
        (pos.x - next_aisle.entry.x) < 0.0;
      return crossed_x && std::abs(pos.y - next_aisle.entry.y) < 0.35;
    }
    const bool crossed_y = (route_transit_start_.y - next_aisle.entry.y) *
      (pos.y - next_aisle.entry.y) < 0.0;
    return crossed_y && std::abs(pos.x - next_aisle.entry.x) < 0.35;
  }

  bool route_aisle_endpoint_reached() const
  {
    if (!route_have_current_aisle_endpoint_ || !have_odom_) {
      return false;
    }
    const auto & pos = odom_.pose.pose.position;
    return std::hypot(
      pos.x - route_current_aisle_endpoint_.x,
      pos.y - route_current_aisle_endpoint_.y) < route_aisle_endpoint_tolerance_m_;
  }

  std::optional<size_t> current_aisle_index() const
  {
    const auto column = current_route_column();
    if (column == "C4") {
      return 0;
    }
    if (column == "C3") {
      return 1;
    }
    if (column == "C2" || column == "C1") {
      return 2;
    }
    return std::nullopt;
  }

  void mark_current_aisle_reached()
  {
    const auto index = current_aisle_index();
    if (index.has_value() && *index < route_aisle_endpoint_reached_.size()) {
      route_aisle_endpoint_reached_[*index] = true;
    }
  }

  Waypoint retreat_point_for_column(const std::string & column) const
  {
    const double aisle_1_y = 0.5 * (grid_y_.at(2) + grid_y_.at(3));
    const double r23_gap_x = 0.5 * (grid_x_.at(1) + grid_x_.at(2));
    const auto aisles = compute_route_aisles();
    if (column == "C4") {
      const double retreat_x = std::isfinite(route_c4_retreat_x_m_) ? route_c4_retreat_x_m_ : r23_gap_x;
      const double retreat_y = std::isfinite(route_c4_retreat_y_m_) ? route_c4_retreat_y_m_ : aisle_1_y;
      return clamp_waypoint({retreat_x, retreat_y});
    }
    if (column == "C3" && route_c3_retreat_enabled_) {
      return clamp_waypoint({route_c3_retreat_x_m_, route_c3_retreat_y_m_});
    }
    if ((column == "C2" || column == "C1") && aisles.size() >= 3) {
      return aisles[2].exit;
    }
    return clamp_waypoint({route_c3_retreat_x_m_, route_c3_retreat_y_m_});
  }

  void executePostStrikeRetreat()
  {
    std::vector<Waypoint> points;
    send_planner_command("BOUNDARY_RECOVERY:OFF");
    const auto retreat = retreat_point_for_column(route_retreat_column_);
    // The official gait backend handles one clear world-space retreat target
    // more reliably than a three-point micro path with lateral steps.  The
    // target still encodes the same intent: leave the struck ball/boundary area
    // and rejoin the fixed S-route from a safe aisle point.
    points.push_back(retreat);
    route_current_aisle_endpoint_ = retreat;
    route_have_current_aisle_endpoint_ = true;
    RCLCPP_WARN(
      get_logger(),
      "%s forced retreat: backoff %.2fm, inward shift %.2fm, safe point %.2f %.2f",
      route_retreat_column_.c_str(),
      post_strike_backoff_distance_m_, post_strike_inward_shift_m_,
      retreat.x, retreat.y);
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  std::optional<size_t> route_column_index_from_id(const std::string & column) const
  {
    if (column.size() < 2 || column.front() != 'C') {
      return std::nullopt;
    }
    try {
      const auto one_based = static_cast<size_t>(std::stoul(column.substr(1)));
      if (one_based == 0 || one_based > grid_y_.size()) {
        return std::nullopt;
      }
      return one_based - 1;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  void send_face_command(double yaw_deg)
  {
    send_planner_command("FACE:" + std::to_string(yaw_deg));
  }

  bool start_direct_route_target_align(const std::string & reason)
  {
    const std::string column = current_route_column();
    if (!route_fixed_strategy_enabled_ || column.empty()) {
      return false;
    }
    const auto expected_id = expected_target_id_for_column(column);
    if (!expected_id.has_value() || is_fixed_blue_id(*expected_id)) {
      return false;
    }
    current_target_ = fallback_route_target_for_column(column);
    current_target_visible_before_strike_ = false;
    route_target_sent_ = false;
    route_reacquire_retry_count_ = 0;
    route_phase_ = RoutePhase::WAIT_ALIGN;
    route_phase_start_time_ = now();
    transition(HuntState::ALIGN, reason + "; direct fixed-target align without in-place scan");
    send_route_target_align();
    return true;
  }

  std::optional<wild_glint_hunt::msg::VisionBall> choose_column_target(const std::string & column) const
  {
    const auto aisle = current_route_aisle();
    if (std::find(aisle.scan_columns.begin(), aisle.scan_columns.end(), column) == aisle.scan_columns.end() &&
      !(column == "C1" && std::find(aisle.scan_columns.begin(), aisle.scan_columns.end(), "C1") != aisle.scan_columns.end()))
    {
      RCLCPP_ERROR(
        get_logger(), "refuse target selection for %s outside current aisle primary=%s",
        column.c_str(), aisle.primary_column.c_str());
      return std::nullopt;
    }
    std::optional<wild_glint_hunt::msg::VisionBall> best_associated;
    std::optional<wild_glint_hunt::msg::VisionBall> best_visible_in_column_view;
    const std::string expected_suffix = column.empty() ? "" : column;
    for (const auto & ball : vision_.orange_balls) {
      if (ball.label != "orange_ball") {
        continue;
      }
      if (!observation_passes_filters(ball)) {
        continue;
      }
      if (orange_observation_conflicts_with_blue(ball)) {
        RCLCPP_WARN(
          get_logger(),
          "reject orange blob near fixed/visible blue: pixel=(%d,%d) distance=%.2f yaw=%.1f",
          ball.pixel_x, ball.pixel_y, ball.distance_m, ball.yaw_deg);
        continue;
      }
      if (std::abs(static_cast<double>(ball.yaw_deg)) <= route_column_match_yaw_tolerance_deg_ &&
        (!best_visible_in_column_view || ball.confidence > best_visible_in_column_view->confidence ||
        (std::abs(ball.confidence - best_visible_in_column_view->confidence) < 1.0e-3 &&
        ball.distance_m < best_visible_in_column_view->distance_m)))
      {
        auto visual_target = ball;
        visual_target.id = "VISUAL:" + column;
        best_visible_in_column_view = visual_target;
      }
      const auto id = route_target_id_from_ball(ball, column);
      if (id.empty()) {
        continue;
      }
      if (!route_expected_target_matches(column, id)) {
        continue;
      }
      if (id.size() < expected_suffix.size() ||
        id.substr(id.size() - expected_suffix.size()) != expected_suffix)
      {
        continue;
      }
      if (!best_associated || ball.distance_m < best_associated->distance_m) {
        auto associated = ball;
        associated.id = id;
        best_associated = associated;
      }
    }
    if (best_associated.has_value()) {
      return best_associated;
    }
    if (const auto expected_id = expected_target_id_for_column(column)) {
      return fallback_route_target_for_column(column);
    }
    if (auto constrained = route_rule_constrained_target(column)) {
      return constrained;
    }
    if (!best_visible_in_column_view.has_value()) {
      for (const auto & ball : vision_.orange_balls) {
        if (ball.label != "orange_ball" || !observation_passes_filters(ball)) {
          continue;
        }
        if (!best_visible_in_column_view ||
          ball.confidence > best_visible_in_column_view->confidence ||
          (std::abs(ball.confidence - best_visible_in_column_view->confidence) < 1.0e-3 &&
          ball.distance_m < best_visible_in_column_view->distance_m))
        {
          if (orange_observation_conflicts_with_blue(ball)) {
            continue;
          }
          auto visual_target = ball;
          visual_target.id = "VISUAL:" + column;
          best_visible_in_column_view = visual_target;
        }
      }
    }
    if (!best_visible_in_column_view.has_value() && !vision_.orange_balls.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "route column %s sees %zu orange blobs but none pass route filters; max_distance=%.2f confidence_min=%.2f",
        column.c_str(), vision_.orange_balls.size(),
        target_observation_max_distance_m_, target_observation_min_confidence_);
    }
    return best_visible_in_column_view;
  }

  bool allow_route_pose_fallback_target(const std::string & column) const
  {
    if (!route_fixed_strategy_enabled_) {
      return false;
    }
    if (column.empty()) {
      return false;
    }
    const std::string id = expected_target_id_for_column(column).value_or(
      make_route_column_target(column).id);
    return !is_fixed_blue_id(id) &&
           std::find(completed_ids_.begin(), completed_ids_.end(), id) == completed_ids_.end();
  }

  wild_glint_hunt::msg::VisionBall fallback_route_target_for_column(const std::string & column) const
  {
    auto target = make_route_column_target(column);
    if (const auto expected_id = expected_target_id_for_column(column)) {
      if (const auto point = grid_position_from_id(*expected_id)) {
        target.id = *expected_id;
        target.x = point->x;
        target.y = point->y;
      }
    }
    wild_glint_hunt::msg::VisionBall ball;
    ball.id = target.id;
    ball.label = "orange_ball";
    ball.pixel_x = target_observation_image_width_px_ / 2;
    ball.pixel_y = 120;
    ball.distance_m = static_cast<float>(std::max(0.1, std::hypot(
      target.x - odom_.pose.pose.position.x,
      target.y - odom_.pose.pose.position.y)));
    ball.yaw_deg = 0.0F;
    ball.confidence = 1.0F;
    ball.radius_px = 20.0F;
    ball.safe_to_approach = true;
    RCLCPP_WARN(
      get_logger(),
      "route column %s fallback target=%s at fixed pose %.2f %.2f; RGB did not provide a usable associated target",
      column.c_str(), target.id.c_str(), target.x, target.y);
    return ball;
  }

  std::optional<std::string> expected_target_id_for_column(const std::string & column) const
  {
    const auto it = std::find(route_enable_columns_.begin(), route_enable_columns_.end(), column);
    if (it == route_enable_columns_.end()) {
      return std::nullopt;
    }
    const size_t index = static_cast<size_t>(std::distance(route_enable_columns_.begin(), it));
    if (index >= route_expected_target_ids_.size() || route_expected_target_ids_[index].empty()) {
      return std::nullopt;
    }
    return route_expected_target_ids_[index];
  }

  std::string route_target_id_from_ball(
    const wild_glint_hunt::msg::VisionBall & ball,
    const std::string & column) const
  {
    const auto col_index = route_column_index_from_id(column);
    if (!col_index.has_value() || *col_index >= grid_x_.size() || !have_odom_) {
      return "";
    }
    const auto estimated = estimate_ball_world_position(ball);
    if (!estimated.has_value()) {
      return "";
    }

    const double expected_x = grid_x_.at(*col_index);

    const auto & pos = odom_.pose.pose.position;
    const double robot_yaw = yaw_from_odom(odom_);
    std::string best_id;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      const size_t map_index = row * grid_x_.size() + *col_index;
      if (std::find(fixed_blue_indices_.begin(), fixed_blue_indices_.end(),
          static_cast<int64_t>(map_index)) != fixed_blue_indices_.end())
      {
        continue;
      }
      const double world_x = expected_x;
      const double world_y = grid_y_.at(row);
      if (!route_target_allowed_in_current_aisle(make_cell_id(row, *col_index, grid_y_.size()))) {
        continue;
      }
      if (route_row_already_completed(row)) {
        continue;
      }
      const double world_error = std::hypot(world_x - estimated->x, world_y - estimated->y);

      const double dx = world_x - pos.x;
      const double dy = world_y - pos.y;
      const double predicted_yaw_deg =
        normalize_angle(std::atan2(dy, dx) - robot_yaw) * 180.0 / M_PI;
      const double yaw_error_deg =
        std::abs(normalize_angle((predicted_yaw_deg - static_cast<double>(ball.yaw_deg)) * M_PI / 180.0)) *
        180.0 / M_PI;
      if (yaw_error_deg > route_column_match_yaw_tolerance_deg_) {
        continue;
      }

      const double cost = yaw_error_deg + world_error * 0.20;
      if (cost < best_cost) {
        best_cost = cost;
        best_id = make_cell_id(row, *col_index, grid_y_.size());
      }
    }
    return best_id;
  }

  bool route_target_allowed_in_current_aisle(const std::string & id) const
  {
    const auto aisle = current_route_aisle();
    for (const auto & column : aisle.scan_columns) {
      if (id.size() >= column.size() && id.substr(id.size() - column.size()) == column) {
        return true;
      }
    }
    return false;
  }

  bool route_row_already_completed(size_t row) const
  {
    for (const auto & completed : completed_ids_) {
      const auto cell = row_col_from_id(completed);
      if (cell.has_value() && cell->first == row) {
        return true;
      }
    }
    return false;
  }

  std::optional<wild_glint_hunt::msg::VisionBall> route_rule_constrained_target(
    const std::string & column) const
  {
    const auto col_index = route_column_index_from_id(column);
    if (!col_index.has_value() || !have_odom_) {
      return std::nullopt;
    }
    std::vector<size_t> remaining_rows;
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      if (route_row_already_completed(row) || is_fixed_blue_cell(row, *col_index)) {
        continue;
      }
      remaining_rows.push_back(row);
    }
    if (remaining_rows.size() != 1) {
      return std::nullopt;
    }
    const size_t row = remaining_rows.front();
    const std::string id = make_cell_id(row, *col_index, grid_y_.size());
    if (!route_expected_target_matches(column, id)) {
      return std::nullopt;
    }
    if (blue_visible_at_id(id)) {
      RCLCPP_WARN(get_logger(), "rule-constrained target %s rejected: blue visible at cell", id.c_str());
      return std::nullopt;
    }
    const auto & pos = odom_.pose.pose.position;
    const double dx = grid_x_.at(*col_index) - pos.x;
    const double dy = grid_y_.at(row) - pos.y;
    wild_glint_hunt::msg::VisionBall ball;
    ball.id = id;
    ball.label = "orange_ball";
    ball.pixel_x = target_observation_image_width_px_ / 2;
    ball.pixel_y = 120;
    ball.distance_m = static_cast<float>(std::hypot(dx, dy));
    ball.yaw_deg = static_cast<float>(
      normalize_angle(std::atan2(dy, dx) - yaw_from_odom(odom_)) * 180.0 / M_PI);
    ball.confidence = 1.0F;
    ball.radius_px = 20.0F;
    ball.safe_to_approach = true;
    RCLCPP_INFO(get_logger(), "route column %s uses row/column rule-constrained target %s", column.c_str(), id.c_str());
    return ball;
  }

  bool route_expected_target_matches(const std::string & column, const std::string & id) const
  {
    const auto it = std::find(route_enable_columns_.begin(), route_enable_columns_.end(), column);
    if (it == route_enable_columns_.end()) {
      return true;
    }
    const size_t index = static_cast<size_t>(std::distance(route_enable_columns_.begin(), it));
    if (index >= route_expected_target_ids_.size() || route_expected_target_ids_[index].empty()) {
      return true;
    }
    return id == route_expected_target_ids_[index];
  }

  void send_search_path()
  {
    exit_sent_ = false;
    if (search_started_time_.nanoseconds() == 0) {
      search_started_time_ = now();
    }
    std::vector<Waypoint> points;

    if (search_relocalization_first_ && search_spin_pause_waypoint_count_ > 0) {
      for (size_t row = 0; row < grid_y_.size(); ++row) {
        for (size_t col = 0; col < grid_x_.size(); ++col) {
          if (is_fixed_blue_cell(row, col)) {
            points.push_back(approach_point_for_cell(row, col));
            if (static_cast<int>(points.size()) >= search_spin_pause_waypoint_count_) {
              break;
            }
          }
        }
        if (static_cast<int>(points.size()) >= search_spin_pause_waypoint_count_) {
          break;
        }
      }
    }

    std::vector<Waypoint> candidates;
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (is_fixed_blue_cell(row, col)) {
          continue;
        }
        const std::string cell_id = make_cell_id(row, col, grid_y_.size());
        if (std::find(completed_ids_.begin(), completed_ids_.end(), cell_id) !=
          completed_ids_.end())
        {
          continue;
        }
        candidates.push_back(approach_point_for_cell(row, col));
      }
    }

    Waypoint cursor;
    if (have_odom_) {
      cursor.x = odom_.pose.pose.position.x;
      cursor.y = odom_.pose.pose.position.y;
      if (auto recovery = blue_recovery_waypoint(cursor)) {
        points.clear();
        points.push_back(*recovery);
        send_planner_command(encode_waypoints("SEARCH:", points));
        return;
      }
      if (!have_search_start_pose_) {
        search_start_pose_ = cursor;
        have_search_start_pose_ = true;
      }
      if (cursor.x <= search_start_corner_x_m_ || cursor.y >= search_start_corner_y_m_) {
        search_start_corner_cleared_ = true;
      }
      if (!search_start_corner_cleared_) {
        points.clear();
        points.push_back(
          clamp_waypoint(
            {cursor.x + search_start_escape_dx_m_, cursor.y + search_start_escape_dy_m_}));
        points.push_back(
          clamp_waypoint(
            {cursor.x + search_secondary_escape_dx_m_, cursor.y + search_secondary_escape_dy_m_}));
        send_planner_command(encode_waypoints("SEARCH:", points));
        return;
      }
      if (search_start_escape_enabled_ &&
        !grid_x_.empty() && !grid_y_.empty() &&
        cursor.x > grid_x_.back() - 0.25 &&
        cursor.y < grid_y_.front() - 0.20)
      {
        points.insert(
          points.begin(),
          clamp_waypoint(
            {cursor.x + search_start_escape_dx_m_, cursor.y + search_start_escape_dy_m_}));
      }
      if (search_front_probe_enabled_) {
        const double heading = yaw_from_odom(odom_);
        points.insert(
          points.begin(),
          clamp_waypoint(
            {cursor.x + std::cos(heading) * search_front_probe_distance_m_,
              cursor.y + std::sin(heading) * search_front_probe_distance_m_}));
      }
    } else {
      cursor = clamp_waypoint(
        {field_max_x_m_ - boundary_margin_m_, field_min_y_m_ + boundary_margin_m_});
    }
    int appended_candidates = 0;
    while (!candidates.empty()) {
      const auto best_it = std::min_element(
        candidates.begin(), candidates.end(),
        [this, &cursor](const Waypoint & lhs, const Waypoint & rhs) {
          const bool lhs_safe = segment_clear_of_blue(cursor, lhs);
          const bool rhs_safe = segment_clear_of_blue(cursor, rhs);
          if (lhs_safe != rhs_safe) {
            return lhs_safe;
          }
          return std::hypot(lhs.x - cursor.x, lhs.y - cursor.y) <
                 std::hypot(rhs.x - cursor.x, rhs.y - cursor.y);
        });
      if (!segment_clear_of_blue(cursor, *best_it)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "no remaining search waypoint has safe clearance from fixed blue balls; wait for replanning");
        break;
      }
      points.push_back(*best_it);
      cursor = *best_it;
      candidates.erase(best_it);
      ++appended_candidates;
      if (search_waypoint_batch_size_ > 0 &&
        appended_candidates >= search_waypoint_batch_size_)
      {
        break;
      }
    }

    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  std::optional<Waypoint> blue_recovery_waypoint(const Waypoint & cursor) const
  {
    std::optional<Waypoint> nearest;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        const Waypoint blue{grid_x_[col], grid_y_[row]};
        const double distance = std::hypot(cursor.x - blue.x, cursor.y - blue.y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = blue;
        }
      }
    }
    if (!nearest || nearest_distance >= blue_body_clearance_m_) {
      return std::nullopt;
    }
    double dx = cursor.x - nearest->x;
    double dy = cursor.y - nearest->y;
    double length = std::hypot(dx, dy);
    if (length < 0.05) {
      dx = -1.0;
      dy = 1.0;
      length = std::sqrt(2.0);
    }
    const double escape_distance = blue_body_clearance_m_ + 0.22;
    return clamp_waypoint(
      {nearest->x + dx / length * escape_distance,
        nearest->y + dy / length * escape_distance});
  }

  void send_search_all_path()
  {
    exit_sent_ = false;
    search_all_sent_ = true;
    search_all_start_time_ = now();
    std::vector<Waypoint> points;
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      if (row % 2 == 0) {
        for (size_t col = 0; col < grid_x_.size(); ++col) {
          points.push_back(approach_point_for_cell(row, col));
        }
      } else {
        for (size_t col = grid_x_.size(); col > 0; --col) {
          points.push_back(approach_point_for_cell(row, col - 1));
        }
      }
    }
    RCLCPP_INFO(get_logger(), "search-all bow path started: %zu waypoints", points.size());
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  void send_world_target_path(const BallTarget & target)
  {
    exit_sent_ = false;
    current_target_.id = target.id;
    std::vector<Waypoint> points;
    if (have_odom_) {
      const auto & pos = odom_.pose.pose.position;
      if (std::abs(pos.y - target.approach_y) > 0.20) {
        points.push_back(clamp_waypoint({pos.x, target.approach_y}));
      }
    }
    points.push_back(clamp_waypoint({target.approach_x, target.approach_y}));
    RCLCPP_INFO(
      get_logger(), "world target %s approach: %.2f %.2f",
      target.id.c_str(), target.approach_x, target.approach_y);
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  void send_exit_path()
  {
    exit_sent_ = true;
    exit_face_sent_ = false;
    std::vector<Waypoint> points;
    if (have_odom_) {
      // After the C1 strike the robot is already beside R1C1.  Do not route
      // back through the R1/R2 corridor or the C2/C3 aisle; finish by moving to
      // the point in front of R1C1, outside/above it and left of the ball.
      points.push_back(clamp_waypoint({exit_x_m_, exit_y_m_}));
    } else {
      points.push_back(clamp_waypoint({exit_x_m_, exit_y_m_}));
    }
    send_planner_command(encode_waypoints("EXIT:", points));
  }

  void send_target_command()
  {
    current_target_visible_before_strike_ = true;
    current_target_initial_distance_m_ = current_target_.distance_m;
    current_target_initial_x_ = current_target_.pixel_x;
    current_target_initial_y_ = current_target_.pixel_y;
    std_msgs::msg::String msg;
    msg.data = "TARGET:" + std::to_string(current_target_.distance_m) + ":" +
      std::to_string(current_target_.yaw_deg);
    planner_command_pub_->publish(msg);
  }

  void send_route_target_align()
  {
    const std::string target_id = current_target_.id.empty() ?
      current_route_column() : current_target_.id;
    if (is_visual_route_target_id(target_id)) {
      send_target_command();
      RCLCPP_INFO(
        get_logger(),
        "route target %s align by live RGB visual servo: distance=%.2f yaw_error=%.1fdeg",
        target_id.c_str(), current_target_.distance_m, current_target_.yaw_deg);
      return;
    }
    if (target_id == current_route_column()) {
      const auto target = make_route_column_target(current_route_column());
      send_target_pose_command(target);
      RCLCPP_INFO(
        get_logger(),
        "route column %s align by fixed route pose: target=(%.2f, %.2f) distance=%.2f yaw_error=%.1fdeg",
        target_id.c_str(), target.x, target.y, current_target_.distance_m, current_target_.yaw_deg);
      return;
    }
    if (auto point = grid_position_from_id(target_id)) {
      if (!route_target_sent_ && send_route_lineup_path(target_id, *point)) {
        route_target_sent_ = true;
        return;
      }
      if (route_target_prefers_visual_align(*point)) {
        send_target_command();
        RCLCPP_INFO(
          get_logger(),
          "route target %s align by live vision near boundary: distance=%.2f yaw_error=%.1fdeg",
          target_id.c_str(), current_target_.distance_m, current_target_.yaw_deg);
        return;
      }
      BallTarget target;
      target.id = target_id;
      target.x = point->x;
      target.y = point->y;
      send_target_pose_command(target);
      RCLCPP_INFO(
        get_logger(), "route target %s align by estimated pose: distance=%.2f yaw_error=%.1fdeg",
        target_id.c_str(), current_target_.distance_m, current_target_.yaw_deg);
      return;
    }
    send_target_command();
  }

  bool send_route_lineup_path(const std::string & target_id, const Waypoint & target_point)
  {
    const auto cell = row_col_from_id(target_id);
    if (!cell.has_value() || !have_odom_ || grid_x_.size() < 4) {
      return false;
    }
    const size_t col = cell->second;
    if (col >= grid_x_.size()) {
      return false;
    }
    double lineup_x = target_point.x;
    if (col == 3) {
      lineup_x = 0.5 * (grid_x_.at(2) + grid_x_.at(3));
    } else if (col == 2) {
      lineup_x = 0.5 * (grid_x_.at(1) + grid_x_.at(2));
    } else if (col == 1 || col == 0) {
      lineup_x = 0.5 * (grid_x_.at(0) + grid_x_.at(1));
    }
    const Waypoint lineup = clamp_waypoint({lineup_x, target_point.y});
    const auto & pos = odom_.pose.pose.position;
    if (std::hypot(pos.x - lineup.x, pos.y - lineup.y) <= route_junction_stop_tolerance_m_) {
      return false;
    }
    std::vector<Waypoint> points;
    // Move along the aisle first, then do a short side approach. This prevents
    // diagonal approaches through same-column blue balls, especially R4C3 during
    // the C3 strike.
    points.push_back(clamp_waypoint({lineup.x, pos.y}));
    points.push_back(lineup);
    route_current_aisle_endpoint_ = lineup;
    route_have_current_aisle_endpoint_ = true;
    send_planner_command(encode_waypoints("SEARCH:", points));
    RCLCPP_INFO(
      get_logger(), "route target %s lineup path via %.2f %.2f before side strike",
      target_id.c_str(), lineup.x, lineup.y);
    return true;
  }

  bool route_target_prefers_visual_align(const Waypoint & point) const
  {
    (void)point;
    // Fixed-route competition mode: after RGB associates an orange blob to a grid cell,
    // final approach must use the known safe cell pose. Pure visual servoing near C4/C1
    // can push the body over the yellow boundary because the front RGB camera loses
    // ground-line context at close range.
    return false;
  }

  void send_recorded_target_path()
  {
    if (execution_index_ >= execution_plan_.size()) {
      return;
    }
    executing_recorded_plan_ = true;
    execution_path_sent_ = true;
    const auto & target = execution_plan_[execution_index_];
    current_target_.id = target.id;
    std::vector<Waypoint> points;
    points.push_back(lineup_point_for_target(target));
    points.push_back(clamp_waypoint({target.approach_x, target.approach_y}));
    RCLCPP_INFO(
      get_logger(), "execute target %s approach: %.2f %.2f",
      target.id.c_str(), target.approach_x, target.approach_y);
    send_planner_command(encode_waypoints("SEARCH:", points));
  }

  void send_recorded_target_align()
  {
    if (!have_odom_ || execution_index_ >= execution_plan_.size()) {
      return;
    }
    executing_recorded_plan_ = true;
    const auto & target = execution_plan_[execution_index_];
    current_target_.id = target.id;
    current_target_.label = "orange_ball";
    if (auto visible = find_ball_by_id(current_target_.id)) {
      current_target_.pixel_x = visible->pixel_x;
      current_target_.pixel_y = visible->pixel_y;
      current_target_.distance_m = visible->distance_m;
      current_target_.yaw_deg = visible->yaw_deg;
      current_target_visible_before_strike_ = true;
    } else if (auto observed = find_recent_observed_target(current_target_.id)) {
      current_target_.pixel_x = observed->pixel_x;
      current_target_.pixel_y = observed->pixel_y;
      current_target_.distance_m = observed->distance_m;
      current_target_.yaw_deg = observed->yaw_deg;
      current_target_visible_before_strike_ = true;
    } else {
      current_target_visible_before_strike_ = false;
    }
    current_target_initial_x_ = current_target_.pixel_x;
    current_target_initial_y_ = current_target_.pixel_y;
    current_target_initial_distance_m_ = current_target_.distance_m;
    send_target_pose_command(target);
    RCLCPP_INFO(
      get_logger(), "execute target %s align by estimated pose: distance=%.2f yaw_error=%.1fdeg",
      target.id.c_str(), current_target_.distance_m, current_target_.yaw_deg);
  }

  void send_world_target_align(const BallTarget & target)
  {
    if (!have_odom_) {
      return;
    }
    current_target_.id = target.id;
    current_target_.label = "orange_ball";
    if (auto visible = find_ball_by_id(current_target_.id)) {
      current_target_.pixel_x = visible->pixel_x;
      current_target_.pixel_y = visible->pixel_y;
      current_target_.distance_m = visible->distance_m;
      current_target_.yaw_deg = visible->yaw_deg;
      current_target_visible_before_strike_ = true;
      current_target_initial_x_ = current_target_.pixel_x;
      current_target_initial_y_ = current_target_.pixel_y;
    } else if (auto observed = find_recent_observed_target(current_target_.id)) {
      current_target_.pixel_x = observed->pixel_x;
      current_target_.pixel_y = observed->pixel_y;
      current_target_.distance_m = observed->distance_m;
      current_target_.yaw_deg = observed->yaw_deg;
      current_target_visible_before_strike_ = true;
      current_target_initial_x_ = current_target_.pixel_x;
      current_target_initial_y_ = current_target_.pixel_y;
    } else {
      current_target_visible_before_strike_ = false;
    }
    send_target_pose_command(target);
    RCLCPP_INFO(
      get_logger(), "world target %s align by estimated pose: distance=%.2f yaw_error=%.1fdeg",
      target.id.c_str(), current_target_.distance_m, current_target_.yaw_deg);
  }

  void send_planner_command(const std::string & command)
  {
    std_msgs::msg::String msg;
    msg.data = command;
    planner_command_pub_->publish(msg);
  }

  void send_target_pose_command(const BallTarget & target)
  {
    const auto & pos = odom_.pose.pose.position;
    const double distance_to_target = std::hypot(target.x - pos.x, target.y - pos.y);
    const double desired_yaw = std::atan2(target.y - pos.y, target.x - pos.x);
    const double strike_distance = target_pose_standoff_for_id(target.id);
    current_target_.distance_m = static_cast<float>(
      std::max(0.0, distance_to_target - strike_distance));
    current_target_.yaw_deg = static_cast<float>(
      normalize_angle(desired_yaw - yaw_from_odom(odom_)) * 180.0 / M_PI);
    current_target_initial_distance_m_ = current_target_.distance_m;
    current_target_world_x_m_ = target.x;
    current_target_world_y_m_ = target.y;
    current_target_world_valid_ = true;
    std_msgs::msg::String msg;
    msg.data = "TARGET_POSE:" + std::to_string(target.x) + ":" +
      std::to_string(target.y) + ":" + std::to_string(strike_distance);
    planner_command_pub_->publish(msg);
  }

  double target_pose_standoff_for_id(const std::string & id) const
  {
    const auto cell = row_col_from_id(id);
    if (!cell.has_value() || grid_x_.empty()) {
      return world_target_strike_distance_m_;
    }
    const size_t col = cell->second;
    if (col + 1 == grid_x_.size()) {
      return boundary_column_strike_distance_m_;
    }
    return world_target_strike_distance_m_;
  }

  std::optional<wild_glint_hunt::msg::VisionBall> choose_orange_target() const
  {
    std::optional<wild_glint_hunt::msg::VisionBall> best;
    for (const auto & ball : vision_.orange_balls) {
      if (ball.label != "orange_ball") {
        continue;
      }
      if (!observation_passes_filters(ball)) {
        continue;
      }
      if (orange_observation_conflicts_with_blue(ball)) {
        continue;
      }
      if (stable_target_id(ball).empty()) {
        continue;
      }
      if (is_ball_completed(ball)) {
        continue;
      }
      if (!best || ball.distance_m < best->distance_m) {
        best = ball;
      }
    }
    return best;
  }

  void record_visible_orange_targets()
  {
    if (!have_odom_) {
      return;
    }
    for (const auto & associated : associate_visible_orange_targets()) {
      ObservedTarget observed;
      observed.id = associated.id;
      observed.x = associated.x;
      observed.y = associated.y;
      observed.distance_m = associated.ball.distance_m;
      observed.yaw_deg = associated.ball.yaw_deg;
      observed.pixel_x = associated.ball.pixel_x;
      observed.pixel_y = associated.ball.pixel_y;
      observed.confidence = associated.ball.confidence;
      observed.last_seen = now();
      if (is_line_of_sight_blocked_by_blue(
          odom_.pose.pose.position.x, odom_.pose.pose.position.y, observed.x, observed.y))
      {
        continue;
      }
      auto existing = std::find_if(
        observed_targets_.begin(), observed_targets_.end(),
        [&observed](const ObservedTarget & target) {return target.id == observed.id;});
      if (existing == observed_targets_.end()) {
        observed.confirm_count = 1;
        observed_targets_.push_back(observed);
        RCLCPP_INFO(
          get_logger(), "recorded candidate orange target %s at %.2f %.2f confidence=%.2f",
          observed.id.c_str(), observed.x, observed.y, observed.confidence);
      } else {
        observed.confirm_count = existing->confirm_count + 1;
        observed.x = 0.70 * existing->x + 0.30 * observed.x;
        observed.y = 0.70 * existing->y + 0.30 * observed.y;
        *existing = observed;
        if (existing->confirm_count == target_observation_confirm_count_) {
          RCLCPP_INFO(
            get_logger(), "confirmed orange target %s count=%d at %.2f %.2f",
            existing->id.c_str(), existing->confirm_count, existing->x, existing->y);
        }
      }
    }
  }

  void build_execution_plan()
  {
    prune_stale_observed_targets();
    execution_plan_.clear();
    Waypoint cursor;
    if (have_odom_) {
      cursor.x = odom_.pose.pose.position.x;
      cursor.y = odom_.pose.pose.position.y;
    }
    std::vector<ObservedTarget> candidates;
    for (const auto & target : observed_targets_) {
      if (target.confirm_count < target_observation_confirm_count_) {
        continue;
      }
      if (std::find(completed_ids_.begin(), completed_ids_.end(), target.id) != completed_ids_.end()) {
        continue;
      }
      candidates.push_back(target);
    }
    enforce_row_column_constraints(candidates);
    while (!candidates.empty() &&
      execution_plan_.size() < static_cast<size_t>(required_ball_count_))
    {
      const auto best_it = std::min_element(
        candidates.begin(), candidates.end(),
        [&cursor](const ObservedTarget & lhs, const ObservedTarget & rhs) {
          return std::hypot(lhs.x - cursor.x, lhs.y - cursor.y) <
                 std::hypot(rhs.x - cursor.x, rhs.y - cursor.y);
        });
      const auto target = make_ball_target(*best_it, cursor);
      if (!target_has_safe_strike_corridor(target)) {
        RCLCPP_WARN(
          get_logger(),
          "skip target %s: unsafe body/strike corridor near fixed blue ball",
          best_it->id.c_str());
        candidates.erase(best_it);
        continue;
      }
      execution_plan_.push_back(target);
      cursor.x = best_it->x;
      cursor.y = best_it->y;
      candidates.erase(best_it);
      if (!search_all_before_strike_ &&
        execution_plan_.size() >= static_cast<size_t>(std::max(1, rolling_plan_max_targets_)))
      {
        break;
      }
    }
    execution_index_ = 0;
    execution_path_sent_ = false;
    RCLCPP_INFO(get_logger(), "built orange execution plan: %zu targets", execution_plan_.size());
  }

  std::vector<AssociatedObservation> associate_visible_orange_targets() const
  {
    std::vector<AssociatedObservation> associations;
    if (!have_odom_) {
      return associations;
    }

    std::vector<const wild_glint_hunt::msg::VisionBall *> detections;
    for (const auto & ball : vision_.orange_balls) {
      if (ball.label == "orange_ball" && observation_passes_filters(ball)) {
        detections.push_back(&ball);
      }
    }
    if (detections.empty()) {
      return associations;
    }

    std::vector<std::pair<AssociatedObservation, double>> candidates;
    for (size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
      const auto & ball = *detections[detection_index];
      const auto estimated = estimate_ball_world_position(ball);
      if (!estimated.has_value()) {
        continue;
      }

      std::optional<AssociatedObservation> best;
      double best_cost = std::numeric_limits<double>::infinity();
      for (size_t row = 0; row < grid_y_.size(); ++row) {
        for (size_t col = 0; col < grid_x_.size(); ++col) {
          if (is_fixed_blue_cell(row, col)) {
            continue;
          }
          const double world_x = grid_x_[col];
          const double world_y = grid_y_[row];
          const double world_error_m =
            std::hypot(world_x - estimated->x, world_y - estimated->y);
          if (world_error_m > target_association_max_world_error_m_) {
            continue;
          }

          const auto & pos = odom_.pose.pose.position;
          const double robot_yaw = yaw_from_odom(odom_);
          const double dx = world_x - pos.x;
          const double dy = world_y - pos.y;
          const double predicted_yaw_deg =
            normalize_angle(std::atan2(dy, dx) - robot_yaw) * 180.0 / M_PI;
          const double predicted_distance_m = std::hypot(dx, dy);
          const double yaw_error_deg = std::abs(
            normalize_angle((predicted_yaw_deg - static_cast<double>(ball.yaw_deg)) * M_PI / 180.0)) *
            180.0 / M_PI;
          const double distance_error_m =
            std::abs(predicted_distance_m - static_cast<double>(ball.distance_m));
          if (yaw_error_deg > target_association_max_yaw_deg_ ||
            distance_error_m > target_association_max_distance_error_m_)
          {
            continue;
          }

          double cost =
            world_error_m * 2.0 +
            yaw_error_deg * target_association_yaw_weight_ +
            distance_error_m * target_association_distance_weight_;

          const std::string candidate_id =
            make_cell_id(row, col, grid_y_.size());
          const auto existing = std::find_if(
            observed_targets_.begin(), observed_targets_.end(),
            [&candidate_id](const ObservedTarget & target) { return target.id == candidate_id; });
          if (existing != observed_targets_.end()) {
            cost -= std::min(
              target_association_history_bonus_,
              0.25 * static_cast<double>(existing->confirm_count) + 0.5 * existing->confidence);
          }
          if (cost < best_cost) {
            AssociatedObservation associated;
            associated.ball = ball;
            associated.id = candidate_id;
            associated.x = world_x;
            associated.y = world_y;
            associated.cost = cost;
            best = associated;
            best_cost = cost;
          }
        }
      }
      if (best.has_value()) {
        candidates.emplace_back(*best, best_cost + static_cast<double>(detection_index) * 1.0e-4);
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.second < rhs.second; });

    std::vector<bool> used_detections(detections.size(), false);
    std::vector<bool> used_rows(grid_y_.size(), false);
    std::vector<bool> used_cols(grid_x_.size(), false);
    for (const auto & entry : candidates) {
      const auto & candidate = entry.first;
      const auto cell = row_col_from_id(candidate.id);
      if (!cell.has_value()) {
        continue;
      }
      const auto detection_index = static_cast<size_t>(
        std::distance(
          detections.begin(),
          std::find_if(
            detections.begin(), detections.end(),
            [&candidate](const wild_glint_hunt::msg::VisionBall * ball) {
              return ball->pixel_x == candidate.ball.pixel_x &&
                     ball->pixel_y == candidate.ball.pixel_y &&
                     std::abs(ball->distance_m - candidate.ball.distance_m) < 1.0e-6 &&
                     std::abs(ball->yaw_deg - candidate.ball.yaw_deg) < 1.0e-6;
            })));
      if (detection_index >= detections.size() ||
        used_detections[detection_index] || used_rows[cell->first] || used_cols[cell->second])
      {
        continue;
      }
      used_detections[detection_index] = true;
      used_rows[cell->first] = true;
      used_cols[cell->second] = true;
      associations.push_back(candidate);
    }
    return associations;
  }

  bool try_execute_immediate_visible_target()
  {
    if (state_ != HuntState::SEARCH || executing_recorded_plan_) {
      return false;
    }
    if (have_search_start_pose_ && search_travel_distance() < rolling_execute_min_travel_m_) {
      return false;
    }
    auto associations = associate_visible_orange_targets();
    if (associations.empty()) {
      return false;
    }

    std::sort(
      associations.begin(), associations.end(),
      [this](const AssociatedObservation & lhs, const AssociatedObservation & rhs) {
        const bool lhs_completed =
          std::find(completed_ids_.begin(), completed_ids_.end(), lhs.id) != completed_ids_.end();
        const bool rhs_completed =
          std::find(completed_ids_.begin(), completed_ids_.end(), rhs.id) != completed_ids_.end();
        if (lhs_completed != rhs_completed) {
          return !lhs_completed;
        }
        if (lhs.ball.confidence != rhs.ball.confidence) {
          return lhs.ball.confidence > rhs.ball.confidence;
        }
        return lhs.ball.distance_m < rhs.ball.distance_m;
      });

    Waypoint cursor;
    if (have_odom_) {
      cursor.x = odom_.pose.pose.position.x;
      cursor.y = odom_.pose.pose.position.y;
    }

    for (const auto & associated : associations) {
      if (associated.ball.confidence < rolling_execute_single_visible_confidence_) {
        continue;
      }
      if (std::find(completed_ids_.begin(), completed_ids_.end(), associated.id) != completed_ids_.end()) {
        continue;
      }
      ObservedTarget observed;
      observed.id = associated.id;
      observed.x = associated.x;
      observed.y = associated.y;
      observed.distance_m = associated.ball.distance_m;
      observed.yaw_deg = associated.ball.yaw_deg;
      observed.pixel_x = associated.ball.pixel_x;
      observed.pixel_y = associated.ball.pixel_y;
      observed.confidence = associated.ball.confidence;
      observed.confirm_count = target_observation_confirm_count_;
      observed.last_seen = now();

      const auto target = make_ball_target(observed, cursor);
      if (!target_has_safe_strike_corridor(target)) {
        continue;
      }
      auto existing = std::find_if(
        observed_targets_.begin(), observed_targets_.end(),
        [&observed](const ObservedTarget & candidate) { return candidate.id == observed.id; });
      if (existing == observed_targets_.end()) {
        observed_targets_.push_back(observed);
      } else {
        *existing = observed;
      }
      planner_status_.clear();
      send_planner_command("STOP");
      execution_plan_.clear();
      execution_plan_.push_back(target);
      execution_index_ = 0;
      execution_path_sent_ = false;
      executing_recorded_plan_ = true;
      last_rolling_plan_time_ = now();
      current_target_.id = associated.id;
      transition(HuntState::EXECUTE_PLAN, "high-confidence visible orange target; immediate execute");
      send_recorded_target_path();
      return true;
    }
    return false;
  }

  double search_travel_distance() const
  {
    if (!have_odom_ || !have_search_start_pose_) {
      return 0.0;
    }
    return std::hypot(
      odom_.pose.pose.position.x - search_start_pose_.x,
      odom_.pose.pose.position.y - search_start_pose_.y);
  }

  void enforce_row_column_constraints(std::vector<ObservedTarget> & candidates) const
  {
    std::sort(
      candidates.begin(), candidates.end(), [](const ObservedTarget & lhs, const ObservedTarget & rhs) {
        if (lhs.confirm_count != rhs.confirm_count) {
          return lhs.confirm_count > rhs.confirm_count;
        }
        return lhs.confidence > rhs.confidence;
      });
    std::vector<ObservedTarget> filtered;
    std::vector<size_t> used_rows;
    std::vector<size_t> used_cols;
    for (const auto & target : candidates) {
      const auto cell = row_col_from_id(target.id);
      if (!cell) {
        continue;
      }
      if (std::find(used_rows.begin(), used_rows.end(), cell->first) != used_rows.end() ||
        std::find(used_cols.begin(), used_cols.end(), cell->second) != used_cols.end())
      {
        continue;
      }
      used_rows.push_back(cell->first);
      used_cols.push_back(cell->second);
      filtered.push_back(target);
    }
    candidates = filtered;
  }

  bool has_any_reliable_target() const
  {
    return std::any_of(
      observed_targets_.begin(), observed_targets_.end(), [this](const ObservedTarget & target) {
        return target.confirm_count >= target_observation_confirm_count_ &&
               std::find(completed_ids_.begin(), completed_ids_.end(), target.id) ==
               completed_ids_.end();
      });
  }

  void prune_stale_observed_targets()
  {
    observed_targets_.erase(
      std::remove_if(
        observed_targets_.begin(), observed_targets_.end(), [this](const ObservedTarget & target) {
          if (target.last_seen.nanoseconds() == 0) {
            return false;
          }
          return (now() - target.last_seen).seconds() > target_observation_stale_timeout_s_;
        }),
      observed_targets_.end());
  }

  bool ready_for_rolling_execution() const
  {
    size_t reliable_count = 0;
    for (const auto & target : observed_targets_) {
      if (target.confirm_count < target_observation_confirm_count_) {
        continue;
      }
      if (std::find(completed_ids_.begin(), completed_ids_.end(), target.id) != completed_ids_.end()) {
        continue;
      }
      ++reliable_count;
    }
    return reliable_count >= static_cast<size_t>(std::max(1, rolling_min_targets_to_execute_));
  }

  double search_scan_progress() const
  {
    if (grid_x_.empty() || grid_y_.empty()) {
      return 1.0;
    }
    const double total_cells = static_cast<double>(grid_x_.size() * grid_y_.size());
    const double reliable = static_cast<double>(std::count_if(
      observed_targets_.begin(), observed_targets_.end(), [this](const ObservedTarget & target) {
        return target.confirm_count >= target_observation_confirm_count_;
      }));
    return std::min(1.0, reliable / std::max(1.0, total_cells));
  }

  void load_simulated_ground_truth_targets()
  {
    if (backend_ != "sim" || !sim_use_ground_truth_layout_) {
      return;
    }
    observed_targets_.clear();
    std::vector<size_t> cols_by_row{0, 1, 2, 3};
    if (sim_randomize_balls_) {
      std::mt19937 generator(static_cast<std::mt19937::result_type>(sim_random_seed_));
      do {
        std::shuffle(cols_by_row.begin(), cols_by_row.end(), generator);
      } while (cols_by_row.size() >= 4 && (cols_by_row[2] == 3 || cols_by_row[3] == 2 || cols_by_row[3] == 3));
    }
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      const size_t col = sim_randomize_balls_ ? cols_by_row[row] : row % grid_x_.size();
      if (col >= grid_x_.size()) {
        continue;
      }
      const size_t index = row * grid_x_.size() + col;
      if (std::find(fixed_blue_indices_.begin(), fixed_blue_indices_.end(), static_cast<int64_t>(index)) !=
        fixed_blue_indices_.end())
      {
        continue;
      }
      ObservedTarget target;
      target.id = make_cell_id(row, col, grid_y_.size());
      target.x = grid_x_[col];
      target.y = grid_y_[row];
      target.distance_m = 0.0;
      target.yaw_deg = 0.0;
      target.last_seen = now();
      observed_targets_.push_back(target);
    }
    RCLCPP_INFO(get_logger(), "sim fallback loaded %zu orange targets", observed_targets_.size());
  }

  BallTarget make_ball_target(const ObservedTarget & observed, const Waypoint & cursor) const
  {
    BallTarget target;
    target.id = observed.id;
    target.x = observed.x;
    target.y = observed.y;
    const auto approach = choose_safe_approach_point(observed, cursor);
    target.approach_x = approach.x;
    target.approach_y = approach.y;
    target.strike_yaw_rad =
      std::atan2(observed.y - target.approach_y, observed.x - target.approach_x);
    return target;
  }

  Waypoint choose_safe_approach_point(
    const ObservedTarget & observed, const Waypoint & cursor) const
  {
    const std::array<Waypoint, 4> directions{
      Waypoint{1.0, 0.0},
      Waypoint{-1.0, 0.0},
      Waypoint{0.0, 1.0},
      Waypoint{0.0, -1.0}};

    std::optional<Waypoint> best_point;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & dir : directions) {
      Waypoint approach = clamp_waypoint(
        {observed.x - dir.x * execute_plan_standoff_m_, observed.y - dir.y * execute_plan_standoff_m_});
      const BallTarget candidate{
        observed.id, observed.x, observed.y, approach.x, approach.y,
        std::atan2(observed.y - approach.y, observed.x - approach.x)};
      if (!target_has_safe_strike_corridor(candidate)) {
        continue;
      }

      double min_blue_distance = std::numeric_limits<double>::infinity();
      bool blocked_from_cursor = false;
      for (size_t row = 0; row < grid_y_.size(); ++row) {
        for (size_t col = 0; col < grid_x_.size(); ++col) {
          if (!is_fixed_blue_cell(row, col)) {
            continue;
          }
          const Waypoint blue{grid_x_[col], grid_y_[row]};
          min_blue_distance = std::min(min_blue_distance, std::hypot(blue.x - approach.x, blue.y - approach.y));
          if (distance_point_to_segment(blue, cursor, approach) < blue_body_clearance_m_) {
            blocked_from_cursor = true;
          }
        }
      }

      double score = std::hypot(approach.x - cursor.x, approach.y - cursor.y);
      score += blocked_from_cursor ? 5.0 : 0.0;
      score -= std::min(1.0, min_blue_distance) * 0.5;
      if (score < best_score) {
        best_score = score;
        best_point = approach;
      }
    }

    if (best_point.has_value()) {
      return *best_point;
    }

    const double center_x = 0.5 * (field_min_x_m_ + field_max_x_m_);
    const double center_y = 0.5 * (field_min_y_m_ + field_max_y_m_);
    double dx = observed.x - center_x;
    double dy = observed.y - center_y;
    const double length = std::max(0.05, std::hypot(dx, dy));
    dx /= length;
    dy /= length;
    return clamp_waypoint(
      {observed.x - dx * execute_plan_standoff_m_, observed.y - dy * execute_plan_standoff_m_});
  }

  bool is_ball_completed(const wild_glint_hunt::msg::VisionBall & ball) const
  {
    const auto id = stable_target_id(ball);
    return std::find(completed_ids_.begin(), completed_ids_.end(), id) != completed_ids_.end();
  }

  std::string stable_target_id(const wild_glint_hunt::msg::VisionBall & ball) const
  {
    if (!have_odom_) {
      return ball.id;
    }
    const auto associations = associate_visible_orange_targets();
    const auto it = std::find_if(
      associations.begin(), associations.end(),
      [&ball](const AssociatedObservation & associated) {
        return associated.ball.pixel_x == ball.pixel_x &&
               associated.ball.pixel_y == ball.pixel_y &&
               std::abs(associated.ball.distance_m - ball.distance_m) < 1.0e-6 &&
               std::abs(associated.ball.yaw_deg - ball.yaw_deg) < 1.0e-6;
      });
    return it == associations.end() ? "" : it->id;
  }

  bool observation_passes_filters(const wild_glint_hunt::msg::VisionBall & ball) const
  {
    if (ball.confidence < target_observation_min_confidence_) {
      return false;
    }
    if (ball.distance_m > target_observation_max_distance_m_) {
      return false;
    }
    if (ball.pixel_x < target_observation_edge_margin_px_ ||
      ball.pixel_x > target_observation_image_width_px_ - target_observation_edge_margin_px_)
    {
      return false;
    }
    return true;
  }

  bool is_visual_route_target_id(const std::string & id) const
  {
    return id.rfind("VISUAL:", 0) == 0;
  }

  bool orange_observation_conflicts_with_blue(const wild_glint_hunt::msg::VisionBall & ball) const
  {
    for (const auto & blue : vision_.blue_balls) {
      const double pixel_distance = std::hypot(
        static_cast<double>(ball.pixel_x - blue.pixel_x),
        static_cast<double>(ball.pixel_y - blue.pixel_y));
      const double reject_radius = std::max(
        28.0,
        1.25 * (static_cast<double>(ball.radius_px) + static_cast<double>(blue.radius_px)));
      if (pixel_distance < reject_radius) {
        return true;
      }
    }
    const auto estimated = estimate_ball_world_position(ball);
    if (!estimated.has_value()) {
      return false;
    }
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        const double distance_to_blue = std::hypot(
          estimated->x - grid_x_[col],
          estimated->y - grid_y_[row]);
        if (distance_to_blue < target_blue_exclusion_radius_m_) {
          return true;
        }
      }
    }
    return false;
  }

  std::optional<Waypoint> grid_position_from_id(const std::string & id) const
  {
    const auto cell = row_col_from_id(id);
    if (!cell) {
      return std::nullopt;
    }
    const auto row = cell->first;
    const auto col = cell->second;
    if (row >= grid_y_.size() || col >= grid_x_.size()) {
      return std::nullopt;
    }
    return Waypoint{grid_x_[col], grid_y_[row]};
  }

  std::optional<std::pair<size_t, size_t>> row_col_from_id(const std::string & id) const
  {
    size_t row = 0;
    size_t col = 0;
    if (std::sscanf(id.c_str(), "R%zuC%zu", &row, &col) != 2 || row == 0 || col == 0) {
      return std::nullopt;
    }
    if (row > grid_y_.size()) {
      return std::nullopt;
    }
    row = grid_y_.size() - row;
    --col;
    if (row >= grid_y_.size() || col >= grid_x_.size()) {
      return std::nullopt;
    }
    return std::make_pair(row, col);
  }

  std::optional<wild_glint_hunt::msg::VisionBall> find_ball_by_id(const std::string & id) const
  {
    if ((now() - last_vision_time_).seconds() > target_visible_timeout_s_) {
      return std::nullopt;
    }
    const auto expected = grid_position_from_id(id);
    if (!expected || !have_odom_) {
      return std::nullopt;
    }
    const double robot_yaw = yaw_from_odom(odom_);
    const auto & pos = odom_.pose.pose.position;
    std::optional<wild_glint_hunt::msg::VisionBall> best;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto & ball : vision_.orange_balls) {
      if (ball.label != "orange_ball" || !observation_passes_filters(ball)) {
        continue;
      }
      const auto estimated = estimate_ball_world_position(ball);
      if (estimated.has_value()) {
        const double world_error = std::hypot(expected->x - estimated->x, expected->y - estimated->y);
        if (world_error > target_association_max_world_error_m_) {
          continue;
        }
      }
      const double dx = expected->x - pos.x;
      const double dy = expected->y - pos.y;
      const double predicted_yaw_deg =
        normalize_angle(std::atan2(dy, dx) - robot_yaw) * 180.0 / M_PI;
      const double predicted_distance_m = std::hypot(dx, dy);
      const double yaw_error_deg =
        std::abs(normalize_angle((predicted_yaw_deg - static_cast<double>(ball.yaw_deg)) * M_PI / 180.0)) *
        180.0 / M_PI;
      const double distance_error_m =
        std::abs(predicted_distance_m - static_cast<double>(ball.distance_m));
      if (yaw_error_deg > target_association_max_yaw_deg_ ||
        distance_error_m > target_association_max_distance_error_m_)
      {
        continue;
      }
      const double cost =
        yaw_error_deg * target_association_yaw_weight_ +
        distance_error_m * target_association_distance_weight_;
      if (cost < best_cost) {
        best_cost = cost;
        best = ball;
      }
    }
    return best;
  }

  struct WorldEstimate
  {
    double x {0.0};
    double y {0.0};
  };

  std::optional<WorldEstimate> estimate_ball_world_position(
    const wild_glint_hunt::msg::VisionBall & ball) const
  {
    if (!have_odom_) {
      return std::nullopt;
    }
    const auto & pos = odom_.pose.pose.position;
    const double robot_yaw = yaw_from_odom(odom_);
    const double heading = robot_yaw + static_cast<double>(ball.yaw_deg) * M_PI / 180.0;
    WorldEstimate estimate;
    estimate.x = pos.x + static_cast<double>(ball.distance_m) * std::cos(heading);
    estimate.y = pos.y + static_cast<double>(ball.distance_m) * std::sin(heading);
    return estimate;
  }

  std::optional<ObservedTarget> find_recent_observed_target(const std::string & id) const
  {
    const auto it = std::find_if(
      observed_targets_.begin(), observed_targets_.end(),
      [&id](const ObservedTarget & target) { return target.id == id; });
    if (it == observed_targets_.end()) {
      return std::nullopt;
    }
    if (it->confirm_count < target_observation_confirm_count_) {
      return std::nullopt;
    }
    if (it->last_seen.nanoseconds() == 0) {
      return std::nullopt;
    }
    if ((now() - it->last_seen).seconds() > strike_recent_visual_timeout_s_) {
      return std::nullopt;
    }
    return *it;
  }

  std::optional<ObservedTarget> find_confirmed_observed_target(const std::string & id) const
  {
    const auto it = std::find_if(
      observed_targets_.begin(), observed_targets_.end(),
      [&id](const ObservedTarget & target) { return target.id == id; });
    if (it == observed_targets_.end()) {
      return std::nullopt;
    }
    if (it->confirm_count < target_observation_confirm_count_) {
      return std::nullopt;
    }
    return *it;
  }

  bool target_available_for_strike(const std::string & id) const
  {
    if (std::find(route_expected_target_ids_.begin(), route_expected_target_ids_.end(), id) !=
      route_expected_target_ids_.end())
    {
      return true;
    }
    if (!id.empty() && !is_visual_route_target_id(id) && blue_visible_at_id(id)) {
      RCLCPP_WARN(
        get_logger(), "refuse strike: target %s currently matches a blue observation", id.c_str());
      return false;
    }
    if (find_ball_by_id(id).has_value() || find_recent_observed_target(id).has_value()) {
      return true;
    }
    if (executing_recorded_plan_ && find_confirmed_observed_target(id).has_value()) {
      return true;
    }
    return current_target_.id == id && current_target_visible_before_strike_;
  }

  bool orange_visible_after_strike() const
  {
    if ((now() - last_vision_time_).seconds() > target_visible_timeout_s_) {
      return false;
    }
    return std::any_of(
      vision_.orange_balls.begin(), vision_.orange_balls.end(),
      [this](const auto & ball) {
        return ball.label == "orange_ball" && observation_passes_filters(ball);
      });
  }

  bool blue_visible_at_id(const std::string & id) const
  {
    if ((now() - last_vision_time_).seconds() > target_visible_timeout_s_) {
      return false;
    }
    const auto expected = grid_position_from_id(id);
    if (!expected || !have_odom_) {
      return false;
    }
    const auto & pos = odom_.pose.pose.position;
    const double robot_yaw = yaw_from_odom(odom_);
    for (const auto & ball : vision_.blue_balls) {
      if (ball.label != "blue_ball" || !observation_passes_filters(ball)) {
        continue;
      }
      const auto estimated = estimate_ball_world_position(ball);
      if (estimated.has_value()) {
        const double world_error = std::hypot(expected->x - estimated->x, expected->y - estimated->y);
        if (world_error <= target_association_max_world_error_m_) {
          return true;
        }
      }
      const double dx = expected->x - pos.x;
      const double dy = expected->y - pos.y;
      const double predicted_yaw_deg =
        normalize_angle(std::atan2(dy, dx) - robot_yaw) * 180.0 / M_PI;
      const double predicted_distance_m = std::hypot(dx, dy);
      const double yaw_error_deg =
        std::abs(normalize_angle((predicted_yaw_deg - static_cast<double>(ball.yaw_deg)) * M_PI / 180.0)) *
        180.0 / M_PI;
      const double distance_error_m =
        std::abs(predicted_distance_m - static_cast<double>(ball.distance_m));
      if (yaw_error_deg <= target_association_max_yaw_deg_ &&
        distance_error_m <= target_association_max_distance_error_m_)
      {
        return true;
      }
    }
    return false;
  }

  bool is_fixed_blue_id(const std::string & id) const
  {
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        const std::string cell_id = make_cell_id(row, col, grid_y_.size());
        if (id == cell_id) {
          return true;
        }
      }
    }
    return false;
  }

  bool is_line_of_sight_blocked_by_blue(
    double start_x, double start_y, double target_x, double target_y) const
  {
    Waypoint start{start_x, start_y};
    Waypoint target{target_x, target_y};
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        Waypoint blue{grid_x_[col], grid_y_[row]};
        if (std::hypot(blue.x - target.x, blue.y - target.y) < 0.05) {
          continue;
        }
        if (distance_point_to_segment(blue, start, target) < blue_body_clearance_m_) {
          return true;
        }
      }
    }
    return false;
  }

  bool segment_clear_of_blue(const Waypoint & start, const Waypoint & target) const
  {
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        const Waypoint blue{grid_x_[col], grid_y_[row]};
        if (distance_point_to_segment(blue, start, target) < blue_body_clearance_m_) {
          return false;
        }
      }
    }
    return true;
  }

  bool target_has_safe_strike_corridor(const BallTarget & target) const
  {
    if (is_fixed_blue_id(target.id)) {
      return false;
    }
    const Waypoint target_point{target.x, target.y};
    const Waypoint approach_point{target.approach_x, target.approach_y};
    if (target.approach_x < field_min_x_m_ + boundary_margin_m_ ||
      target.approach_x > field_max_x_m_ - boundary_margin_m_ ||
      target.approach_y < field_min_y_m_ + boundary_margin_m_ ||
      target.approach_y > field_max_y_m_ - boundary_margin_m_)
    {
      return false;
    }
    for (size_t row = 0; row < grid_y_.size(); ++row) {
      for (size_t col = 0; col < grid_x_.size(); ++col) {
        if (!is_fixed_blue_cell(row, col)) {
          continue;
        }
        const Waypoint blue{grid_x_[col], grid_y_[row]};
        if (std::hypot(blue.x - target.x, blue.y - target.y) < target_blue_exclusion_radius_m_) {
          return false;
        }
        if (distance_point_to_segment(blue, approach_point, target_point) <
          strike_corridor_clearance_m_)
        {
          return false;
        }
      }
    }
    return true;
  }

  static double distance_point_to_segment(
    const Waypoint & point, const Waypoint & start, const Waypoint & end)
  {
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double len_sq = dx * dx + dy * dy;
    if (len_sq <= 1.0e-9) {
      return std::hypot(point.x - start.x, point.y - start.y);
    }
    const double t = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) / len_sq, 0.0, 1.0);
    const double proj_x = start.x + t * dx;
    const double proj_y = start.y + t * dy;
    return std::hypot(point.x - proj_x, point.y - proj_y);
  }

  Waypoint clamp_waypoint(Waypoint point) const
  {
    point.x = std::min(
      field_max_x_m_ - boundary_margin_m_,
      std::max(field_min_x_m_ + boundary_margin_m_, point.x));
    point.y = std::min(
      field_max_y_m_ - boundary_margin_m_,
      std::max(field_min_y_m_ + boundary_margin_m_, point.y));
    return point;
  }

  bool is_fixed_blue_cell(size_t row, size_t col) const
  {
    if (grid_x_.empty()) {
      return false;
    }
    const auto index = static_cast<int64_t>(row * grid_x_.size() + col);
    return std::find(fixed_blue_indices_.begin(), fixed_blue_indices_.end(), index) !=
           fixed_blue_indices_.end();
  }

  Waypoint approach_point_for_cell(size_t row, size_t col) const
  {
    Waypoint point{grid_x_[col], grid_y_[row]};
    const double center_x = 0.5 * (field_min_x_m_ + field_max_x_m_);
    const double center_y = 0.5 * (field_min_y_m_ + field_max_y_m_);
    const double dx = center_x - point.x;
    const double dy = center_y - point.y;
    const double length = std::max(0.05, std::hypot(dx, dy));
    point.x += dx / length * search_anchor_offset_m_;
    point.y += dy / length * search_anchor_offset_m_;
    return clamp_waypoint(point);
  }

  Waypoint lineup_point_for_target(const BallTarget & target) const
  {
    double dx = target.x - target.approach_x;
    double dy = target.y - target.approach_y;
    const double length = std::max(0.05, std::hypot(dx, dy));
    dx /= length;
    dy /= length;
    return clamp_waypoint(
      {target.approach_x - dx * strike_lineup_distance_m_,
        target.approach_y - dy * strike_lineup_distance_m_});
  }

  bool rear_legs_clear_exit() const
  {
    if (!have_odom_) {
      return false;
    }
    const auto & pos = odom_.pose.pose.position;
    return pos.x <= exit_x_m_ + exit_clearance_m_ && pos.y >= exit_y_m_ - rear_leg_offset_m_;
  }

  bool pose_contact_success() const
  {
    if (!have_odom_ || !current_target_world_valid_) {
      return false;
    }
    const auto & pos = odom_.pose.pose.position;
    const double yaw = yaw_from_odom(odom_);
    const double front_x = pos.x + std::cos(yaw) * strike_front_offset_m_;
    const double front_y = pos.y + std::sin(yaw) * strike_front_offset_m_;
    return std::hypot(front_x - current_target_world_x_m_, front_y - current_target_world_y_m_) <=
           strike_contact_distance_m_;
  }

  double yaw_from_odom(const nav_msgs::msg::Odometry & odom) const
  {
    const auto & q = odom.pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double normalize_angle(double angle) const
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  void load_world_targets()
  {
    world_targets_.clear();
    const size_t count = std::min(
      {world_target_x_.size(), world_target_y_.size(), world_target_approach_x_.size(),
        world_target_approach_y_.size(), world_target_strike_yaw_deg_.size()});
    for (size_t i = 0; i < count; ++i) {
      BallTarget target;
      target.id = "world_orange_" + std::to_string(i + 1);
      target.x = world_target_x_[i];
      target.y = world_target_y_[i];
      target.approach_x = world_target_approach_x_[i];
      target.approach_y = world_target_approach_y_[i];
      target.strike_yaw_rad = world_target_strike_yaw_deg_[i] * M_PI / 180.0;
      world_targets_.push_back(target);
    }
    RCLCPP_INFO(get_logger(), "loaded %zu optional world-target coordinates", world_targets_.size());
  }

  void transition(HuntState next, const std::string & reason)
  {
    if (state_ != next) {
      RCLCPP_INFO(get_logger(), "state %s -> %s: %s", to_string(state_).c_str(), to_string(next).c_str(), reason.c_str());
      state_ = next;
      state_enter_time_ = now();
    }
  }

  bool handle_global_timeout()
  {
    if (task_total_timeout_s_ <= 0.0 || task_start_time_.nanoseconds() == 0 ||
      state_ == HuntState::EXIT || state_ == HuntState::FINISH || state_ == HuntState::ERROR)
    {
      return false;
    }
    if ((now() - task_start_time_).seconds() < task_total_timeout_s_) {
      return false;
    }
    RCLCPP_ERROR(
      get_logger(), "task total timeout %.1fs; abort remaining targets and navigate to exit",
      task_total_timeout_s_);
    send_planner_command("STOP");
    transition(HuntState::EXIT, "task total timeout");
    send_exit_path();
    return true;
  }

  bool handle_state_timeout()
  {
    if (state_timeout_s_ <= 0.0 || state_enter_time_.nanoseconds() == 0 ||
      state_ == HuntState::INIT || state_ == HuntState::FOLLOW_ROUTE || state_ == HuntState::EXIT ||
      state_ == HuntState::FINISH || state_ == HuntState::ERROR)
    {
      return false;
    }
    if ((now() - state_enter_time_).seconds() < state_timeout_s_) {
      return false;
    }
    RCLCPP_ERROR(
      get_logger(), "state timeout in %s after %.1fs; applying recovery",
      to_string(state_).c_str(), state_timeout_s_);
    send_planner_command("STOP");
    planner_status_.clear();
    if (route_fixed_strategy_enabled_) {
      if (state_ == HuntState::ALIGN || state_ == HuntState::STRIKE || state_ == HuntState::VERIFY) {
        route_phase_ = RoutePhase::MOVE_TO_OBSERVE;
        route_reacquire_retry_count_ = 0;
      } else {
        route_path_sent_ = false;
        route_face_sent_ = false;
        route_target_sent_ = false;
      }
      transition(HuntState::FOLLOW_ROUTE, "state timeout recovery");
      return true;
    }
    transition(HuntState::EXIT, "state timeout fallback exit");
    send_exit_path();
    return true;
  }

  bool handle_route_phase_timeout()
  {
    if (route_phase_timeout_s_ <= 0.0 || route_phase_start_time_.nanoseconds() == 0) {
      return false;
    }
    if ((now() - route_phase_start_time_).seconds() < route_phase_timeout_s_) {
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "route phase timeout for column %s; forcing safe recovery",
      current_route_column().c_str());
    send_planner_command("STOP");
    planner_status_.clear();
    route_path_sent_ = false;
    route_face_sent_ = false;
    route_target_sent_ = false;
    if (route_phase_ == RoutePhase::RETREAT_AFTER_STRIKE ||
      route_phase_ == RoutePhase::TRANSIT_TO_NEXT_AISLE)
    {
      route_phase_ = RoutePhase::TRANSIT_TO_NEXT_AISLE;
      route_phase_start_time_ = now();
      RCLCPP_WARN(
        get_logger(),
        "route transfer timeout; retry transfer to current column %s instead of skipping it",
        current_route_column().c_str());
      return true;
    }
    const std::string skipped_column = current_route_column();
    if (!skipped_column.empty()) {
      completed_ids_.push_back(skipped_column);
    }
    advance_route_column(false);
    route_phase_start_time_ = now();
    return true;
  }

  void publish_status()
  {
    std_msgs::msg::String state_msg;
    state_msg.data = to_string(state_);
    state_pub_->publish(state_msg);
    std_msgs::msg::String status_msg;
    status_msg.data = to_string(state_) + " completed=" + std::to_string(completed_balls_) +
      " target=" + current_target_.id + " route_col=" + current_route_column() +
      " planner=" + planner_status_;
    status_pub_->publish(status_msg);
  }

  void publish_success()
  {
    std_msgs::msg::String msg;
    msg.data = success_message_;
    success_pub_->publish(msg);
  }

  void update_odometry_from_interface()
  {
    if (!robot_) {
      return;
    }
    odom_ = robot_->get_odometry();
    have_odom_ = true;
  }

  bool stand_ready() const
  {
    if (!have_odom_) {
      return false;
    }
    return odom_.pose.pose.position.z >= stand_ready_height_threshold_m_;
  }

  std::string state_topic_;
  std::string status_topic_;
  std::string success_topic_;
  std::string success_message_;
  std::string planner_command_topic_;
  std::string planner_status_topic_;
  std::string vision_topic_;
  std::string danger_topic_;
  std::string pose_topic_;
  int required_ball_count_ {4};
  int max_strike_retries_ {1};
  double align_tolerance_deg_ {15.0};
  bool dynamic_strike_enabled_ {true};
  double strike_success_check_time_s_ {1.0};
  bool strike_light_touch_ {true};
  double single_strike_timeout_s_ {25.0};
  int rolling_plan_max_targets_ {1};
  int timer_period_ms_ {200};
  double strike_verify_timeout_s_ {2.0};
  double strike_success_pixel_shift_ {30.0};
  double strike_success_distance_shift_m_ {0.10};
  double strike_recent_visual_timeout_s_ {2.5};
  bool strike_require_visual_confirmation_ {true};
  bool strike_accept_pose_contact_success_ {true};
  double strike_contact_distance_m_ {0.24};
  double strike_front_offset_m_ {0.28};
  double target_visible_timeout_s_ {1.0};
  double field_width_m_ {4.0};
  double field_height_m_ {4.0};
  double field_min_x_m_ {0.0};
  double field_min_y_m_ {0.0};
  double field_max_x_m_ {4.0};
  double field_max_y_m_ {4.0};
  double boundary_margin_m_ {0.15};
  double exit_x_m_ {0.15};
  double exit_y_m_ {3.85};
  double exit_heading_deg_ {90.0};
  double rear_leg_offset_m_ {0.35};
  double exit_clearance_m_ {0.10};
  double stand_ready_height_threshold_m_ {0.18};
  double world_target_strike_distance_m_ {0.34};
  double boundary_column_strike_distance_m_ {0.55};
  double exit_intermediate_y_m_ {3.40};
  bool danger_active_ {false};
  bool avoidance_active_ {false};
  bool exit_sent_ {false};
  bool exit_face_sent_ {false};
  bool have_odom_ {false};
  bool sim_assume_strike_success_ {false};
  bool debug_verbose_ {false};
  std::string route_mode_ {"fixed_s_curve"};
  bool route_fixed_strategy_enabled_ {true};
  bool search_all_before_strike_ {true};
  std::string backend_ {"sim"};
  bool require_search_waypoints_before_targets_ {true};
  bool search_path_completed_once_ {false};
  bool search_all_sent_ {false};
  bool sim_use_ground_truth_layout_ {true};
  int rolling_min_targets_to_execute_ {2};
  double rolling_search_timeout_s_ {8.0};
  double rolling_replan_cooldown_s_ {2.0};
  double rolling_execute_single_visible_confidence_ {0.88};
  double rolling_execute_min_travel_m_ {0.85};
  double min_search_scan_fraction_before_execute_ {0.25};
  double target_observation_min_confidence_ {0.45};
  int target_observation_confirm_count_ {2};
  double target_observation_stale_timeout_s_ {12.0};
  double target_observation_max_distance_m_ {3.2};
  int target_observation_edge_margin_px_ {20};
  int target_observation_image_width_px_ {640};
  double target_association_max_yaw_deg_ {10.0};
  double target_association_max_distance_error_m_ {1.8};
  double target_association_max_world_error_m_ {0.55};
  double target_association_yaw_weight_ {1.0};
  double target_association_distance_weight_ {0.35};
  double target_association_history_bonus_ {1.5};
  double route_column_match_y_tolerance_m_ {0.50};
  double route_column_match_world_tolerance_m_ {0.90};
  double route_column_match_yaw_tolerance_deg_ {22.0};
  int sim_random_seed_ {2026};
  bool sim_randomize_balls_ {true};
  double search_all_fallback_timeout_s_ {12.0};
  bool search_relocalization_first_ {true};
  bool use_world_targets_ {false};
  bool world_target_sent_ {false};
  bool executing_recorded_plan_ {false};
  bool execution_path_sent_ {false};
  bool search_initial_scan_enabled_ {true};
  bool search_initial_scan_sent_ {false};
  bool search_initial_scan_completed_ {false};
  bool search_start_corner_cleared_ {false};
  bool have_search_start_pose_ {false};
  size_t completed_balls_ {0};
  size_t world_target_index_ {0};
  size_t execution_index_ {0};
  size_t route_column_index_ {0};
  int strike_attempts_ {0};
  int route_reacquire_retry_count_ {0};
  HuntState state_ {HuntState::INIT};
  HuntState previous_state_ {HuntState::SEARCH};
  RoutePhase route_phase_ {RoutePhase::MOVE_TO_OBSERVE};
  std::string planner_status_;
  std::vector<std::string> route_enable_columns_;
  std::vector<std::string> route_expected_target_ids_;
  std::vector<int64_t> route_columns_order_;
  std::vector<double> grid_x_;
  std::vector<double> grid_y_;
  std::vector<int64_t> fixed_blue_indices_;
  std::vector<double> world_target_x_;
  std::vector<double> world_target_y_;
  std::vector<double> world_target_approach_x_;
  std::vector<double> world_target_approach_y_;
  std::vector<double> world_target_strike_yaw_deg_;
  std::vector<BallTarget> world_targets_;
  std::vector<BallTarget> route_targets_;
  std::vector<ObservedTarget> observed_targets_;
  std::vector<BallTarget> execution_plan_;
  std::vector<std::string> completed_ids_;
  Waypoint search_start_pose_;
  double search_anchor_offset_m_ {0.45};
  double execute_plan_standoff_m_ {0.42};
  double blue_body_clearance_m_ {0.52};
  double strike_corridor_clearance_m_ {0.48};
  double target_blue_exclusion_radius_m_ {0.45};
  double strike_lineup_distance_m_ {0.45};
  int search_spin_pause_waypoint_count_ {3};
  int search_waypoint_batch_size_ {5};
  double search_front_probe_distance_m_ {0.30};
  bool search_front_probe_enabled_ {true};
  bool search_start_escape_enabled_ {true};
  double search_start_escape_dx_m_ {-0.55};
  double search_start_escape_dy_m_ {0.25};
  double search_start_corner_x_m_ {2.80};
  double search_start_corner_y_m_ {1.60};
  double search_secondary_escape_dx_m_ {-0.85};
  double search_secondary_escape_dy_m_ {0.65};
  double route_forward_heading_deg_ {0.0};
  double route_start_turn_ccw_deg_ {45.0};
  double route_nominal_heading_deg_ {0.0};
  double route_face_timeout_s_ {8.0};
  double route_column_visual_timeout_s_ {5.0};
  double route_bottom_corridor_clearance_m_ {0.50};
  double route_start_side_offset_m_ {0.55};
  double task_total_timeout_s_ {300.0};
  double state_timeout_s_ {60.0};
  double route_phase_timeout_s_ {45.0};
  double route_aisle_endpoint_tolerance_m_ {0.30};
  double route_junction_stop_tolerance_m_ {0.20};
  double route_junction_overshoot_distance_m_ {0.50};
  double route_align_reissue_interval_s_ {6.0};
  double route_visual_align_boundary_margin_m_ {0.45};
  double post_strike_backoff_distance_m_ {0.35};
  double post_strike_inward_shift_m_ {0.25};
  bool route_c4_retreat_enabled_ {true};
  bool route_allow_pose_fallback_target_ {false};
  double route_c4_retreat_x_m_ {std::numeric_limits<double>::quiet_NaN()};
  double route_c4_retreat_y_m_ {std::numeric_limits<double>::quiet_NaN()};
  bool route_c3_retreat_enabled_ {true};
  double route_c3_retreat_x_m_ {2.0};
  double route_c3_retreat_y_m_ {1.0};
  double route_c34_aisle_x_offset_m_ {0.0};
  double route_c23_aisle_x_offset_m_ {0.0};
  double route_c3_aisle_exit_y_m_ {std::numeric_limits<double>::quiet_NaN()};
  double search_initial_scan_speed_radps_ {0.35};
  double search_initial_scan_duration_s_ {4.5};
  int route_reacquire_retry_limit_ {2};
  std::vector<double> route_column_scan_offsets_deg_ {0.0, -45.0, 45.0, -25.0, 25.0};
  bool route_path_sent_ {false};
  bool route_face_sent_ {false};
  bool route_target_sent_ {false};
  bool route_transit_face_done_ {false};
  bool route_have_current_aisle_endpoint_ {false};
  std::string route_retreat_column_;
  Waypoint route_current_aisle_endpoint_;
  Waypoint route_transit_start_;
  std::array<bool, 3> route_aisle_endpoint_reached_ {{false, false, false}};
  wild_glint_hunt::msg::VisionBallArray vision_;
  wild_glint_hunt::msg::VisionBall current_target_;
  int current_target_initial_x_ {0};
  int current_target_initial_y_ {0};
  double current_target_initial_distance_m_ {0.0};
  double current_target_world_x_m_ {0.0};
  double current_target_world_y_m_ {0.0};
  bool current_target_world_valid_ {false};
  bool current_target_visible_before_strike_ {false};
  bool strike_motion_executed_ {false};
  rclcpp::Time strike_start_time_;
  rclcpp::Time verify_start_time_;
  rclcpp::Time search_all_start_time_;
  rclcpp::Time search_started_time_;
  rclcpp::Time last_rolling_plan_time_;
  rclcpp::Time last_vision_time_;
  rclcpp::Time route_phase_start_time_;
  rclcpp::Time task_start_time_;
  rclcpp::Time state_enter_time_;
  nav_msgs::msg::Odometry odom_;
  RobotInterface::SharedPtr robot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr success_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_command_pub_;
  rclcpp::Subscription<wild_glint_hunt::msg::VisionBallArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr danger_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wild_glint_hunt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wild_glint_hunt::StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
