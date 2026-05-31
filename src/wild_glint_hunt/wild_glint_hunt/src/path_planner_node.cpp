#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "wild_glint_hunt/msg/vision_ball_array.hpp"
#include "wild_glint_hunt/robot_interface.hpp"

namespace wild_glint_hunt
{
namespace
{

struct Waypoint
{
  double x {0.0};
  double y {0.0};
};

double yaw_from_odom(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::vector<double> parse_numbers(const std::string & text)
{
  std::stringstream stream(text);
  std::string token;
  std::vector<double> values;
  while (std::getline(stream, token, ':')) {
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception &) {
    }
  }
  return values;
}

std::string make_cell_id(size_t internal_row, size_t col, size_t row_count)
{
  const size_t display_row = row_count - internal_row;
  return "R" + std::to_string(display_row) + "C" + std::to_string(col + 1);
}

}  // namespace

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planner_node")
  {
    field_width_m_ = declare_parameter<double>("field_width_m", 4.0);
    field_height_m_ = declare_parameter<double>("field_height_m", 4.0);
    field_min_x_m_ = declare_parameter<double>("field_min_x_m", 0.0);
    field_min_y_m_ = declare_parameter<double>("field_min_y_m", 0.0);
    field_max_x_m_ = declare_parameter<double>("field_max_x_m", field_width_m_);
    field_max_y_m_ = declare_parameter<double>("field_max_y_m", field_height_m_);
    boundary_margin_m_ = declare_parameter<double>("boundary_margin_m", 0.15);
    obstacle_forbidden_radius_m_ = declare_parameter<double>("obstacle_forbidden_radius_m", 0.20);
    obstacle_avoidance_radius_m_ = declare_parameter<double>("obstacle_avoidance_radius_m", 0.35);
    obstacle_centers_x_m_ = declare_parameter<std::vector<double>>(
      "obstacle_centers_x_m", {3.2, 2.0, 3.2});
    obstacle_centers_y_m_ = declare_parameter<std::vector<double>>(
      "obstacle_centers_y_m", {3.02, 3.86, 3.86});
    grid_x_centers_m_ = declare_parameter<std::vector<double>>(
      "grid_x_centers_m", {-0.4, 0.8, 2.0, 3.2});
    grid_y_centers_m_ = declare_parameter<std::vector<double>>(
      "grid_y_centers_m", {1.34, 2.18, 3.02, 3.86});
    route_expected_target_ids_ = declare_parameter<std::vector<std::string>>(
      "route_expected_target_ids", std::vector<std::string>{});
    expand_all_blue_keepouts_ = declare_parameter<bool>("expand_all_blue_keepouts", false);
    if (expand_all_blue_keepouts_) {
      expand_blue_keepouts_from_expected_targets();
    }
    route_c4_observe_offset_x_m_ =
      declare_parameter<double>("route_c4_observe_offset_x_m", 0.0);
    route_c4_observe_offset_y_m_ =
      declare_parameter<double>("route_c4_observe_offset_y_m", 0.0);
    route_c34_aisle_x_offset_m_ =
      declare_parameter<double>("route_c34_aisle_x_offset_m", 0.0);
    route_c34_bypass_x_offset_m_ =
      declare_parameter<double>("route_c34_bypass_x_offset_m", -0.30);
    route_c34_rejoin_y_m_ =
      declare_parameter<double>("route_c34_rejoin_y_m", std::numeric_limits<double>::quiet_NaN());
    route_c3_observe_offset_x_m_ =
      declare_parameter<double>("route_c3_observe_offset_x_m", 0.0);
    route_c3_observe_offset_y_m_ =
      declare_parameter<double>("route_c3_observe_offset_y_m", 0.0);
    route_c23_aisle_x_offset_m_ =
      declare_parameter<double>("route_c23_aisle_x_offset_m", 0.0);
    route_c3_aisle_exit_y_m_ =
      declare_parameter<double>("route_c3_aisle_exit_y_m", std::numeric_limits<double>::quiet_NaN());
    route_c21_observe_offset_x_m_ =
      declare_parameter<double>("route_c21_observe_offset_x_m", 0.0);
    route_c21_observe_offset_y_m_ =
      declare_parameter<double>("route_c21_observe_offset_y_m", 0.0);
    route_bottom_corridor_clearance_m_ =
      declare_parameter<double>("route_bottom_corridor_clearance_m", 0.50);
    route_start_side_offset_m_ =
      declare_parameter<double>("route_start_side_offset_m", 0.55);
    approach_far_speed_mps_ = declare_parameter<double>("approach_far_speed_mps", 0.20);
    approach_near_speed_mps_ = declare_parameter<double>("approach_near_speed_mps", 0.12);
    approach_slow_distance_m_ = declare_parameter<double>("approach_slow_distance_m", 0.65);
    approach_stop_distance_m_ = declare_parameter<double>("approach_stop_distance_m", 0.30);
    align_yaw_tolerance_deg_ = declare_parameter<double>("align_yaw_tolerance_deg", 7.0);
    dynamic_strike_enabled_ = declare_parameter<bool>("dynamic_strike_enabled", false);
    dynamic_strike_trigger_distance_m_ =
      declare_parameter<double>("dynamic_strike_trigger_distance_m", 0.50);
    angular_gain_ = declare_parameter<double>("planner_angular_gain", 1.1);
    approach_max_angular_speed_radps_ =
      declare_parameter<double>("approach_max_angular_speed_radps", 0.18);
    waypoint_linear_speed_mps_ = declare_parameter<double>("waypoint_linear_speed_mps", 0.22);
    waypoint_linear_speed_mps_ =
      declare_parameter<double>("aisle_travel_speed", waypoint_linear_speed_mps_);
    waypoint_position_tolerance_m_ = declare_parameter<double>("waypoint_position_tolerance_m", 0.18);
    waypoint_yaw_gain_ = declare_parameter<double>("waypoint_yaw_gain", 0.65);
    waypoint_heading_align_threshold_deg_ =
      declare_parameter<double>("waypoint_heading_align_threshold_deg", 18.0);
    waypoint_heading_slow_threshold_deg_ =
      declare_parameter<double>("waypoint_heading_slow_threshold_deg", 7.0);
    waypoint_rotate_only_threshold_deg_ =
      declare_parameter<double>("waypoint_rotate_only_threshold_deg", 55.0);
    waypoint_max_angular_speed_radps_ =
      declare_parameter<double>("waypoint_max_angular_speed_radps", 0.35);
    exit_linear_speed_mps_ = declare_parameter<double>("exit_linear_speed_mps", 0.15);
    boundary_recovery_speed_mps_ =
      declare_parameter<double>("boundary_recovery_speed_mps", 0.08);
    boundary_recovery_enabled_ =
      declare_parameter<bool>("boundary_recovery_enabled", true);
    boundary_recovery_trigger_margin_m_ =
      declare_parameter<double>("boundary_recovery_trigger_margin_m", 0.15);
    boundary_recovery_distance_m_ =
      declare_parameter<double>("boundary_recovery_distance_m", 0.20);
    boundary_recovery_turn_tolerance_deg_ =
      declare_parameter<double>("boundary_recovery_turn_tolerance_deg", 12.0);
    boundary_recovery_max_count_ =
      declare_parameter<int>("boundary_recovery_max_count", 3);
    boundary_force_center_x_m_ =
      declare_parameter<double>("boundary_force_center_x_m", 2.0);
    boundary_force_center_y_m_ =
      declare_parameter<double>("boundary_force_center_y_m", 2.0);
    approach_rotate_only_yaw_deg_ =
      declare_parameter<double>("approach_rotate_only_yaw_deg", 35.0);
    close_target_rotate_backoff_distance_m_ =
      declare_parameter<double>("close_target_rotate_backoff_distance_m", 0.45);
    close_target_backoff_speed_mps_ =
      declare_parameter<double>("close_target_backoff_speed_mps", -0.06);
    avoid_back_speed_mps_ = declare_parameter<double>("avoid_back_speed_mps", -0.12);
    avoid_turn_speed_radps_ = declare_parameter<double>("avoid_turn_speed_radps", 0.45);
    avoid_back_duration_s_ = declare_parameter<double>("avoid_back_duration_s", 0.8);
    avoid_turn_duration_s_ = declare_parameter<double>("avoid_turn_duration_s", 1.2);
    strike_speed_mps_ = declare_parameter<double>("head_butt_speed_mps", 0.4);
    strike_duration_s_ = declare_parameter<double>("head_butt_duration_s", 1.0);
    strike_boundary_slow_margin_m_ =
      declare_parameter<double>("strike_boundary_slow_margin_m", 0.40);
    strike_safe_speed_mps_ = declare_parameter<double>("strike_safe_forward_speed_mps", 0.08);
    strike_safe_duration_s_ = declare_parameter<double>("strike_safe_duration_s", 0.30);
    strike_emergency_backoff_distance_m_ =
      declare_parameter<double>("strike_emergency_backoff_distance_m", 0.20);
    strike_emergency_backoff_speed_mps_ =
      declare_parameter<double>("strike_emergency_backoff_speed_mps", 0.12);
    control_period_ms_ = declare_parameter<int>("planner_control_period_ms", 100);
    target_use_visual_distance_updates_ =
      declare_parameter<bool>("target_use_visual_distance_updates", false);
    target_use_visual_yaw_updates_ =
      declare_parameter<bool>("target_use_visual_yaw_updates", false);
    safety_prediction_dt_s_ = declare_parameter<double>("safety_prediction_dt_s", 0.35);
    target_update_max_yaw_delta_deg_ =
      declare_parameter<double>("target_update_max_yaw_delta_deg", 25.0);
    target_update_max_distance_delta_m_ =
      declare_parameter<double>("target_update_max_distance_delta_m", 0.80);
    target_lost_timeout_s_ = declare_parameter<double>("target_lost_timeout_s", 1.0);
    stuck_detection_time_s_ = declare_parameter<double>("stuck_detection_time_s", 15.0);
    stuck_detection_distance_m_ = declare_parameter<double>("stuck_detection_distance_m", 0.10);
    stuck_recovery_distance_m_ = declare_parameter<double>("stuck_recovery_distance_m", 0.30);
    stuck_recovery_angle_deg_ = declare_parameter<double>("stuck_recovery_angle_deg", 45.0);
    low_height_stop_enabled_ = declare_parameter<bool>("low_height_stop_enabled", true);
    low_height_stop_threshold_m_ = declare_parameter<double>("low_height_stop_threshold_m", 0.16);

    const auto qos_depth = declare_parameter<int>("planner_qos_depth", 10);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto vision_topic = declare_parameter<std::string>("vision_input_topic", "/vision/ball_array");
    const auto command_topic = declare_parameter<std::string>("planner_command_topic", "planner/command");
    const auto status_topic = declare_parameter<std::string>("planner_status_topic", "planner/status");
    const auto backend = declare_parameter<std::string>("backend", "sim");

    auto node_handle = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    robot_ = backend == "real" ?
      std::static_pointer_cast<RobotInterface>(std::make_shared<RobotInterfaceReal>(node_handle)) :
      std::static_pointer_cast<RobotInterface>(std::make_shared<RobotInterfaceSim>(node_handle));
    RCLCPP_INFO(get_logger(), "path planner using robot interface: %s", robot_->backend_name().c_str());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, qos_depth);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos_depth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
      });
    vision_sub_ = create_subscription<wild_glint_hunt::msg::VisionBallArray>(
      vision_topic, qos_depth, [this](const wild_glint_hunt::msg::VisionBallArray::SharedPtr msg) {
        vision_ = *msg;
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic, qos_depth, [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_command(msg->data);
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_), [this]() { control_step(); });
  }

private:
  void handle_command(const std::string & command)
  {
    if (command == "STOP") {
      mode_ = "IDLE";
      exit_mode_ = false;
      publish_stop();
      publish_status("STOPPED");
      return;
    }
    if (command == "AVOID") {
      mode_ = "AVOID_BACK";
      phase_start_ = now();
      publish_status("AVOIDING");
      return;
    }
    if (command == "BOUNDARY_RECOVERY:OFF") {
      boundary_recovery_pause_count_ = std::max(0, boundary_recovery_pause_count_ + 1);
      return;
    }
    if (command == "BOUNDARY_RECOVERY:ON") {
      boundary_recovery_pause_count_ = std::max(0, boundary_recovery_pause_count_ - 1);
      return;
    }
    if (command == "STRIKE") {
      mode_ = "STRIKE";
      phase_start_ = now();
      strike_log_emitted_ = false;
      strike_boundary_degraded_ = have_odom_ &&
        distance_to_field_boundary() < strike_boundary_slow_margin_m_;
      active_strike_speed_mps_ = strike_boundary_degraded_ ? strike_safe_speed_mps_ : strike_speed_mps_;
      active_strike_duration_s_ = strike_boundary_degraded_ ? strike_safe_duration_s_ : strike_duration_s_;
      publish_status("STRIKING");
      return;
    }
    if (command.rfind("SCAN:", 0) == 0) {
      const auto values = parse_numbers(command.substr(std::string("SCAN:").size()));
      if (values.size() >= 2) {
        scan_angular_speed_radps_ = values[0];
        scan_duration_s_ = values[1];
        mode_ = "SCAN";
        phase_start_ = now();
        publish_status("SCANNING");
      }
      return;
    }
    if (command.rfind("SEARCH:", 0) == 0) {
      set_waypoints(command.substr(std::string("SEARCH:").size()));
      mode_ = "WAYPOINTS";
      exit_mode_ = false;
      waypoint_index_ = 0;
      publish_status("SEARCHING");
      return;
    }
    if (command.rfind("ROUTE:", 0) == 0) {
      set_route_waypoints(command.substr(std::string("ROUTE:").size()));
      mode_ = "WAYPOINTS";
      exit_mode_ = false;
      waypoint_index_ = 0;
      publish_status("SEARCHING");
      return;
    }
    if (command.rfind("EXIT:", 0) == 0) {
      set_waypoints(command.substr(std::string("EXIT:").size()));
      mode_ = "WAYPOINTS";
      exit_mode_ = true;
      waypoint_index_ = 0;
      publish_status("EXITING");
      return;
    }
    if (command.rfind("FACE:", 0) == 0) {
      const auto values = parse_numbers(command.substr(std::string("FACE:").size()));
      if (!values.empty()) {
        face_target_yaw_deg_ = values[0];
        mode_ = "FACE";
        publish_status("FACING");
      }
      return;
    }
    if (command.rfind("TARGET:", 0) == 0) {
      const auto values = parse_numbers(command.substr(std::string("TARGET:").size()));
      if (values.size() >= 2) {
        target_pose_initialized_ = false;
        target_distance_m_ = values[0];
        target_yaw_deg_ = values[1];
        target_distance_initialized_ = true;
        target_yaw_initialized_ = true;
        last_control_time_ = now();
        last_target_seen_time_ = now();
        mode_ = "TARGET";
        publish_status("TARGETING");
      }
    }
    if (command.rfind("TARGET_POSE:", 0) == 0) {
      const auto values = parse_numbers(command.substr(std::string("TARGET_POSE:").size()));
      if (values.size() >= 3) {
        target_pose_x_m_ = values[0];
        target_pose_y_m_ = values[1];
        target_pose_standoff_m_ = values[2];
        target_pose_initialized_ = true;
        target_distance_initialized_ = true;
        target_yaw_initialized_ = true;
        last_control_time_ = now();
        last_target_seen_time_ = now();
        update_target_from_pose();
        mode_ = "TARGET";
        publish_status("TARGETING");
      }
    }
  }

  void set_waypoints(const std::string & encoded)
  {
    const auto values = parse_numbers(encoded);
    waypoints_.clear();
    for (size_t i = 0; i + 1 < values.size(); i += 2) {
      const double x = std::min(field_max_x_m_ - boundary_margin_m_, std::max(field_min_x_m_ + boundary_margin_m_, values[i]));
      const double y = std::min(field_max_y_m_ - boundary_margin_m_, std::max(field_min_y_m_ + boundary_margin_m_, values[i + 1]));
      append_safe_waypoint({x, y});
    }
  }

  void set_route_waypoints(const std::string & stage)
  {
    waypoints_.clear();
    if (stage == "ESCAPE_START") {
      if (grid_x_centers_m_.size() < 4 || grid_y_centers_m_.size() < 1) {
        RCLCPP_WARN(get_logger(), "insufficient grid centers for ESCAPE_START");
        return;
      }
      const double c34_gap_x =
        0.5 * (grid_x_centers_m_[2] + grid_x_centers_m_[3]) + route_c34_aisle_x_offset_m_;
      const double row4_y = std::max(
        field_min_y_m_ + boundary_margin_m_,
        grid_y_centers_m_[0] - route_bottom_corridor_clearance_m_);
      const double pre_gap_x = 0.5 * (grid_x_centers_m_[0] + grid_x_centers_m_[1]);
      const double start_x = have_odom_ ? odom_.pose.pose.position.x : pre_gap_x;
      // When the robot is already spawned at the stone-road exit near C4, do not
      // waste time traversing the bottom R4/yellow-line corridor to C1/C2 and back.
      if (start_x < 0.5 * (pre_gap_x + c34_gap_x)) {
        append_safe_waypoint(clamp_waypoint({pre_gap_x, row4_y}));
      }
      append_safe_waypoint(clamp_waypoint({c34_gap_x, row4_y}));
      return;
    }
    const auto point = route_point_for_stage(stage);
    if (!point.has_value()) {
      RCLCPP_WARN(get_logger(), "unknown route stage: %s", stage.c_str());
      return;
    }
    append_safe_waypoint(*point);
  }

  std::optional<Waypoint> route_point_for_stage(const std::string & stage) const
  {
    if (grid_x_centers_m_.size() < 4 || grid_y_centers_m_.size() < 4) {
      return std::nullopt;
    }

    const double r12_gap_y = 0.5 * (grid_y_centers_m_[2] + grid_y_centers_m_[3]);
    const double r4_inside_y = std::max(
      field_min_y_m_ + boundary_margin_m_, grid_y_centers_m_[0] - route_bottom_corridor_clearance_m_);
    const double c3_observe_y = std::isfinite(route_c3_aisle_exit_y_m_) ?
      std::clamp(
        route_c3_aisle_exit_y_m_,
        field_min_y_m_ + boundary_margin_m_,
        field_max_y_m_ - boundary_margin_m_) :
      r4_inside_y;
    const double aisle_1_x =
      0.5 * (grid_x_centers_m_[2] + grid_x_centers_m_[3]) + route_c34_aisle_x_offset_m_;  // C3/C4
    const double aisle_2_x =
      0.5 * (grid_x_centers_m_[1] + grid_x_centers_m_[2]) + route_c23_aisle_x_offset_m_;  // C2/C3
    const double aisle_3_x = 0.5 * (grid_x_centers_m_[0] + grid_x_centers_m_[1]);  // C1/C2

    if (stage == "OBSERVE_C4") {
      return clamp_waypoint({
        aisle_1_x + route_c4_observe_offset_x_m_,
        r12_gap_y + route_c4_observe_offset_y_m_});
    }
    if (stage == "OBSERVE_C3") {
      return clamp_waypoint({
        aisle_2_x + route_c3_observe_offset_x_m_,
        c3_observe_y + route_c3_observe_offset_y_m_});
    }
    if (stage == "OBSERVE_C2_C1") {
      return clamp_waypoint({
        aisle_3_x + route_c21_observe_offset_x_m_,
        r12_gap_y + route_c21_observe_offset_y_m_});
    }
    return std::nullopt;
  }

  void append_safe_waypoint(const Waypoint & requested)
  {
    Waypoint start = requested;
    if (!waypoints_.empty()) {
      start = waypoints_.back();
    } else if (have_odom_) {
      start.x = odom_.pose.pose.position.x;
      start.y = odom_.pose.pose.position.y;
    }

    Waypoint target = keep_out_of_obstacle_forbidden_zones(clamp_waypoint(requested));
    for (size_t i = 0; i < obstacle_count(); ++i) {
      const Waypoint obstacle{obstacle_centers_x_m_[i], obstacle_centers_y_m_[i]};
      if (distance_point_to_segment(obstacle, start, target) >= obstacle_avoidance_radius_m_) {
        continue;
      }
      const double dx = target.x - start.x;
      const double dy = target.y - start.y;
      const double length = std::max(0.05, std::hypot(dx, dy));
      const double px = -dy / length;
      const double py = dx / length;
      const double offset = obstacle_avoidance_radius_m_ + 0.12;
      Waypoint first = clamp_waypoint({obstacle.x + px * offset, obstacle.y + py * offset});
      Waypoint second = clamp_waypoint({obstacle.x - px * offset, obstacle.y - py * offset});
      const double first_cost = std::hypot(first.x - start.x, first.y - start.y) +
        std::hypot(target.x - first.x, target.y - first.y);
      const double second_cost = std::hypot(second.x - start.x, second.y - start.y) +
        std::hypot(target.x - second.x, target.y - second.y);
      waypoints_.push_back(first_cost <= second_cost ? first : second);
      start = waypoints_.back();
    }
    waypoints_.push_back(target);
  }

  Waypoint keep_out_of_obstacle_forbidden_zones(Waypoint point) const
  {
    for (size_t i = 0; i < obstacle_count(); ++i) {
      const Waypoint obstacle{obstacle_centers_x_m_[i], obstacle_centers_y_m_[i]};
      double dx = point.x - obstacle.x;
      double dy = point.y - obstacle.y;
      double distance = std::hypot(dx, dy);
      if (distance >= obstacle_forbidden_radius_m_) {
        continue;
      }
      if (distance < 1.0e-3) {
        dx = 1.0;
        dy = 0.0;
        distance = 1.0;
      }
      const double safe_distance = obstacle_forbidden_radius_m_ + 0.03;
      point.x = obstacle.x + dx / distance * safe_distance;
      point.y = obstacle.y + dy / distance * safe_distance;
      point = clamp_waypoint(point);
      RCLCPP_WARN(
        get_logger(),
        "requested waypoint inside fixed-blue forbidden zone; projected to %.2f %.2f",
        point.x, point.y);
    }
    return point;
  }

  void control_step()
  {
    refresh_odometry_fallback();
    if (mode_ == "IDLE" || !have_odom_) {
      return;
    }
    if (low_height_stop_enabled_ && odom_.pose.pose.position.z < low_height_stop_threshold_m_) {
      publish_stop();
      publish_status("LOW_HEIGHT_STOP");
      mode_ = "IDLE";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "robot height %.3f is below safe locomotion threshold %.3f; stopping planner",
        odom_.pose.pose.position.z, low_height_stop_threshold_m_);
      return;
    }
    if (maybe_start_boundary_recovery()) {
      return;
    }
    if (mode_ == "WAYPOINTS") {
      update_stuck_watchdog();
      follow_waypoints();
    } else if (mode_ == "FACE") {
      face_heading();
    } else if (mode_ == "TARGET") {
      update_stuck_watchdog();
      approach_target();
    } else if (mode_ == "SCAN") {
      timed_velocity(0.0, scan_angular_speed_radps_, scan_duration_s_, "IDLE", "SCAN_DONE");
    } else if (mode_ == "AVOID_BACK") {
      timed_velocity(avoid_back_speed_mps_, 0.0, avoid_back_duration_s_, "AVOID_TURN");
    } else if (mode_ == "AVOID_TURN") {
      timed_velocity(0.0, avoid_turn_speed_radps_, avoid_turn_duration_s_, "IDLE", "AVOID_DONE");
    } else if (mode_ == "STRIKE") {
      execute_strike_motion();
    } else if (mode_ == "STRIKE_BACKOFF") {
      const double backoff_duration =
        strike_emergency_backoff_distance_m_ / std::max(0.02, strike_emergency_backoff_speed_mps_);
      timed_velocity(
        -std::abs(strike_emergency_backoff_speed_mps_), 0.0, backoff_duration,
        "IDLE", "STRIKE_DONE");
    } else if (mode_ == "BOUNDARY_FACE_CENTER") {
      boundary_face_center();
    } else if (mode_ == "BOUNDARY_MOVE_CENTER") {
      boundary_move_center();
    } else if (mode_ == "STUCK_BACK") {
      timed_velocity(
        -std::max(0.02, boundary_recovery_speed_mps_), 0.0,
        stuck_recovery_distance_m_ / std::max(0.02, boundary_recovery_speed_mps_),
        "STUCK_TURN");
    } else if (mode_ == "STUCK_TURN") {
      timed_velocity(
        0.0, avoid_turn_speed_radps_,
        std::abs(stuck_recovery_angle_deg_) * M_PI / 180.0 /
        std::max(0.05, std::abs(avoid_turn_speed_radps_)),
        stuck_return_mode_.empty() ? "IDLE" : stuck_return_mode_, "STUCK_RECOVERED");
    } else if (mode_ == "STRIKE") {
      timed_velocity(strike_speed_mps_, 0.0, strike_duration_s_, "IDLE", "STRIKE_DONE");
    }
  }

  void refresh_odometry_fallback()
  {
    if (have_odom_) {
      return;
    }
    if (!robot_) {
      return;
    }
    const auto fallback = robot_->get_odometry();
    if (fallback.header.stamp.sec == 0 && fallback.header.stamp.nanosec == 0) {
      return;
    }
    odom_ = fallback;
    have_odom_ = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "planner recovered odometry from robot interface fallback");
  }

  void follow_waypoints()
  {
    if (waypoint_index_ >= waypoints_.size()) {
      publish_stop();
      publish_status("WAYPOINTS_DONE");
      mode_ = "IDLE";
      return;
    }
    const auto & point = waypoints_[waypoint_index_];
    const auto & pos = odom_.pose.pose.position;
    const double dx = point.x - pos.x;
    const double dy = point.y - pos.y;
    const double distance = std::hypot(dx, dy);
    if (distance < waypoint_position_tolerance_m_) {
      publish_status("WAYPOINT_REACHED");
      ++waypoint_index_;
      return;
    }
    const double yaw_error = normalize_angle(std::atan2(dy, dx) - yaw_from_odom(odom_));
    const double yaw_error_abs = std::abs(yaw_error);
    const double align_threshold = waypoint_heading_align_threshold_deg_ * M_PI / 180.0;
    const double slow_threshold = waypoint_heading_slow_threshold_deg_ * M_PI / 180.0;
    const double rotate_only_threshold = waypoint_rotate_only_threshold_deg_ * M_PI / 180.0;
    const double base_speed = exit_mode_ ? exit_linear_speed_mps_ : waypoint_linear_speed_mps_;
    const double angular_z = std::clamp(
      waypoint_yaw_gain_ * yaw_error,
      -waypoint_max_angular_speed_radps_,
      waypoint_max_angular_speed_radps_);
    const double world_vx = base_speed * dx / std::max(0.05, distance);
    const double world_vy = base_speed * dy / std::max(0.05, distance);
    const double yaw = yaw_from_odom(odom_);
    double body_x = std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
    double body_y = -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
    if (yaw_error_abs > align_threshold) {
      body_x *= 0.65;
      body_y *= 0.65;
    } else if (yaw_error_abs > slow_threshold) {
      body_x *= 0.85;
      body_y *= 0.85;
    }
    if (yaw_error_abs > rotate_only_threshold) {
      body_x *= 0.35;
      body_y *= 0.35;
    }
    robot_->send_velocity(body_x, body_y, angular_z);
  }

  void face_heading()
  {
    if (!have_odom_) {
      return;
    }
    const double target_yaw_rad = face_target_yaw_deg_ * M_PI / 180.0;
    const double yaw_error = normalize_angle(target_yaw_rad - yaw_from_odom(odom_));
    if (std::abs(yaw_error) <= align_yaw_tolerance_deg_ * M_PI / 180.0) {
      publish_stop();
      publish_status("FACE_DONE");
      mode_ = "IDLE";
      return;
    }
    const double angular_z = std::clamp(
      waypoint_yaw_gain_ * yaw_error,
      -waypoint_max_angular_speed_radps_,
      waypoint_max_angular_speed_radps_);
    send_safe_velocity(0.0, angular_z);
  }

  void approach_target()
  {
    if (target_pose_initialized_) {
      update_target_from_pose();
      const auto & pos = odom_.pose.pose.position;
      const double dx = target_pose_x_m_ - pos.x;
      const double dy = target_pose_y_m_ - pos.y;
      const double distance_to_target = std::hypot(dx, dy);
      const double remaining = std::max(0.0, distance_to_target - target_pose_standoff_m_);
      const double yaw_error = normalize_angle(std::atan2(dy, dx) - yaw_from_odom(odom_));
      if (target_pose_initialized_ && remaining <= 0.25) {
        // Fixed-route pose targets are already selected from known safe lineup
        // cells.  When the head is within striking range, do not keep rotating
        // in place to chase an exact yaw; that caused long spins near R1C1.
        publish_stop();
        publish_status("TARGET_ALIGNED");
        mode_ = "IDLE";
        return;
      }
      if (dynamic_strike_enabled_ && remaining <= dynamic_strike_trigger_distance_m_ &&
        std::abs(yaw_error) <= align_yaw_tolerance_deg_ * M_PI / 180.0)
      {
        publish_stop();
        publish_status("TARGET_ALIGNED");
        mode_ = "IDLE";
        return;
      }
      if (!dynamic_strike_enabled_ && remaining <= approach_stop_distance_m_ &&
        std::abs(yaw_error) <= align_yaw_tolerance_deg_ * M_PI / 180.0)
      {
        publish_stop();
        publish_status("TARGET_ALIGNED");
        mode_ = "IDLE";
        return;
      }
      const double speed = remaining > approach_slow_distance_m_ ? approach_far_speed_mps_ : approach_near_speed_mps_;
      double world_vx = speed * dx / std::max(0.05, distance_to_target);
      double world_vy = speed * dy / std::max(0.05, distance_to_target);
      // Pose-based striking is allowed to use known orange target coordinates, but
      // the body must still stay clear of every non-target blue ball. Add a local
      // repulsive component near blue keepout centers so the head approaches the
      // orange ball while the trunk does not brush adjacent blue balls.
      for (size_t i = 0; i < obstacle_count(); ++i) {
        const double ox = obstacle_centers_x_m_[i];
        const double oy = obstacle_centers_y_m_[i];
        const double away_x = pos.x - ox;
        const double away_y = pos.y - oy;
        const double obstacle_distance = std::hypot(away_x, away_y);
        const double influence = obstacle_avoidance_radius_m_ + 0.18;
        if (obstacle_distance < 1.0e-3 || obstacle_distance >= influence) {
          continue;
        }
        const double scale = (influence - obstacle_distance) / influence * approach_near_speed_mps_;
        world_vx += scale * away_x / obstacle_distance;
        world_vy += scale * away_y / obstacle_distance;
      }
      const double yaw = yaw_from_odom(odom_);
      const double body_x = std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
      const double body_y = -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
      const double angular_z = std::clamp(
        angular_gain_ * yaw_error,
        -approach_max_angular_speed_radps_, approach_max_angular_speed_radps_);
      robot_->send_velocity(body_x, body_y, angular_z);
      return;
    }
    const bool target_seen = update_target_from_vision();
    if (!target_pose_initialized_ && target_use_visual_distance_updates_ && !target_seen &&
      last_target_seen_time_.nanoseconds() > 0 &&
      (now() - last_target_seen_time_).seconds() > target_lost_timeout_s_)
    {
      publish_stop();
      publish_status("TARGET_LOST");
      mode_ = "IDLE";
      return;
    }
    const double yaw_abs = std::abs(target_yaw_deg_);
    if (dynamic_strike_enabled_ &&
      target_distance_m_ <= dynamic_strike_trigger_distance_m_ &&
      yaw_abs <= align_yaw_tolerance_deg_)
    {
      publish_stop();
      publish_status("TARGET_ALIGNED");
      mode_ = "IDLE";
      return;
    }
    if (!dynamic_strike_enabled_ &&
      target_distance_m_ <= approach_stop_distance_m_ && yaw_abs <= align_yaw_tolerance_deg_)
    {
      publish_stop();
      publish_status("TARGET_ALIGNED");
      mode_ = "IDLE";
      return;
    }
    double linear_x = target_distance_m_ > approach_slow_distance_m_ ?
      approach_far_speed_mps_ : approach_near_speed_mps_;
    const double angular_z = std::clamp(
      angular_gain_ * target_yaw_deg_ * M_PI / 180.0,
      -approach_max_angular_speed_radps_, approach_max_angular_speed_radps_);
    if (yaw_abs > approach_rotate_only_yaw_deg_) {
      linear_x = target_distance_m_ < close_target_rotate_backoff_distance_m_ ?
        close_target_backoff_speed_mps_ : 0.0;
    } else if (!dynamic_strike_enabled_ && yaw_abs > align_yaw_tolerance_deg_) {
      linear_x *= 0.4;
    }
    const auto now_time = now();
    double dt = static_cast<double>(control_period_ms_) / 1000.0;
    if (last_control_time_.nanoseconds() > 0) {
      dt = std::max(0.0, (now_time - last_control_time_).seconds());
    }
    last_control_time_ = now_time;
    send_safe_velocity(linear_x, angular_z);
    if (!target_pose_initialized_ && !target_use_visual_distance_updates_) {
      target_distance_m_ = std::max(0.0, target_distance_m_ - std::max(0.0, linear_x) * dt);
    }
    if (!target_pose_initialized_ && !target_use_visual_yaw_updates_) {
      const double yaw_before = target_yaw_deg_;
      target_yaw_deg_ -= angular_z * 180.0 / M_PI * dt;
      if ((yaw_before > 0.0 && target_yaw_deg_ < 0.0) ||
        (yaw_before < 0.0 && target_yaw_deg_ > 0.0))
      {
        target_yaw_deg_ = 0.0;
      }
    }
  }

  void update_target_from_pose()
  {
    if (!target_pose_initialized_ || !have_odom_) {
      return;
    }
    const auto & pos = odom_.pose.pose.position;
    const double dx = target_pose_x_m_ - pos.x;
    const double dy = target_pose_y_m_ - pos.y;
    target_distance_m_ = std::max(0.0, std::hypot(dx, dy) - target_pose_standoff_m_);
    target_yaw_deg_ = normalize_angle(std::atan2(dy, dx) - yaw_from_odom(odom_)) * 180.0 / M_PI;
  }

  bool update_target_from_vision()
  {
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    double best_yaw = target_yaw_deg_;
    double best_distance = target_distance_m_;
    for (const auto & ball : vision_.orange_balls) {
      if (ball.label != "orange_ball") {
        continue;
      }
      const double yaw_delta = std::abs(ball.yaw_deg - target_yaw_deg_);
      const double distance_delta = std::abs(ball.distance_m - target_distance_m_);
      if (target_yaw_initialized_ && yaw_delta > target_update_max_yaw_delta_deg_) {
        continue;
      }
      if (target_distance_initialized_ && distance_delta > target_update_max_distance_delta_m_) {
        continue;
      }
      const double score = yaw_delta + distance_delta * 20.0;
      if (score < best_score) {
        best_score = score;
        best_distance = ball.distance_m;
        best_yaw = ball.yaw_deg;
        found = true;
      }
    }
    if (!found) {
      return false;
    }
    if (target_pose_initialized_) {
      last_target_seen_time_ = now();
      return true;
    }
    if (target_use_visual_yaw_updates_ || !target_yaw_initialized_) {
      target_yaw_deg_ = best_yaw;
      target_yaw_initialized_ = true;
    }
    if (target_use_visual_distance_updates_ || !target_distance_initialized_) {
      target_distance_m_ = best_distance;
      target_distance_initialized_ = true;
    }
    last_target_seen_time_ = now();
    return true;
  }

  void timed_velocity(
    double linear_x,
    double angular_z,
    double duration_s,
    const std::string & next_mode,
    const std::string & done_status = "")
  {
    const double elapsed = (now() - phase_start_).seconds();
    if (elapsed >= duration_s) {
      publish_stop();
      mode_ = next_mode;
      phase_start_ = now();
      if (!done_status.empty()) {
        publish_status(done_status);
      }
      return;
    }
    robot_->send_velocity(linear_x, angular_z);
  }

  void execute_strike_motion()
  {
    if (!strike_log_emitted_) {
      RCLCPP_INFO(
        get_logger(),
        "STRIKE: sending velocity command: linear=%.2f, duration=%.2f%s",
        active_strike_speed_mps_, active_strike_duration_s_,
        strike_boundary_degraded_ ? " [boundary-safe]" : "");
      strike_log_emitted_ = true;
    }
    const double elapsed = (now() - phase_start_).seconds();
    if (elapsed >= active_strike_duration_s_) {
      publish_stop();
      mode_ = "STRIKE_BACKOFF";
      phase_start_ = now();
      RCLCPP_INFO(get_logger(), "STRIKE: velocity command sent successfully");
      return;
    }
    send_safe_velocity(active_strike_speed_mps_, 0.0);
  }

  double distance_to_field_boundary() const
  {
    const auto & pos = odom_.pose.pose.position;
    return std::min(
      {pos.x - field_min_x_m_, field_max_x_m_ - pos.x,
        pos.y - field_min_y_m_, field_max_y_m_ - pos.y});
  }

  bool mode_allows_boundary_recovery() const
  {
    return mode_ == "WAYPOINTS" || mode_ == "TARGET";
  }

  void update_stuck_watchdog()
  {
    if (stuck_detection_time_s_ <= 0.0 || !have_odom_) {
      return;
    }
    const auto & pos = odom_.pose.pose.position;
    if (stuck_watch_start_time_.nanoseconds() == 0) {
      stuck_watch_start_time_ = now();
      stuck_watch_x_m_ = pos.x;
      stuck_watch_y_m_ = pos.y;
      return;
    }
    const double moved = std::hypot(pos.x - stuck_watch_x_m_, pos.y - stuck_watch_y_m_);
    if (moved >= stuck_detection_distance_m_) {
      stuck_watch_start_time_ = now();
      stuck_watch_x_m_ = pos.x;
      stuck_watch_y_m_ = pos.y;
      return;
    }
    if ((now() - stuck_watch_start_time_).seconds() < stuck_detection_time_s_) {
      return;
    }
    stuck_return_mode_ = mode_;
    mode_ = "STUCK_BACK";
    phase_start_ = now();
    stuck_watch_start_time_ = now();
    publish_status("STUCK_RECOVERY");
    RCLCPP_WARN(
      get_logger(), "stuck recovery at pose %.2f %.2f after %.1fs with movement %.2fm",
      pos.x, pos.y, stuck_detection_time_s_, moved);
  }

  bool maybe_start_boundary_recovery()
  {
    if (boundary_recovery_pause_count_ > 0) {
      return false;
    }
    if (!boundary_recovery_enabled_ || !mode_allows_boundary_recovery()) {
      return false;
    }
    if (distance_to_field_boundary() >= boundary_recovery_trigger_margin_m_) {
      return false;
    }
    boundary_return_mode_ = mode_;
    boundary_recovery_start_time_ = now();
    mode_ = "BOUNDARY_FACE_CENTER";
    publish_stop();
    ++boundary_recovery_count_;
    publish_status("BOUNDARY_RECOVERY");
    RCLCPP_WARN(
      get_logger(),
      "boundary recovery %d/%d at pose %.2f %.2f; turning toward field center",
      boundary_recovery_count_, boundary_recovery_max_count_,
      odom_.pose.pose.position.x, odom_.pose.pose.position.y);
    if (boundary_recovery_count_ >= boundary_recovery_max_count_) {
      force_center_waypoint();
      return true;
    }
    return true;
  }

  void boundary_face_center()
  {
    const auto & pos = odom_.pose.pose.position;
    const double center_x = std::clamp(
      boundary_force_center_x_m_, field_min_x_m_ + boundary_margin_m_,
      field_max_x_m_ - boundary_margin_m_);
    const double center_y = std::clamp(
      boundary_force_center_y_m_, field_min_y_m_ + boundary_margin_m_,
      field_max_y_m_ - boundary_margin_m_);
    const double desired_yaw = std::atan2(center_y - pos.y, center_x - pos.x);
    const double yaw_error = normalize_angle(desired_yaw - yaw_from_odom(odom_));
    if (std::abs(yaw_error) <= boundary_recovery_turn_tolerance_deg_ * M_PI / 180.0 ||
      (now() - boundary_recovery_start_time_).seconds() > 4.0)
    {
      mode_ = "BOUNDARY_MOVE_CENTER";
      boundary_recovery_start_time_ = now();
      return;
    }
    const double angular_z = std::clamp(
      waypoint_yaw_gain_ * yaw_error,
      -waypoint_max_angular_speed_radps_, waypoint_max_angular_speed_radps_);
    robot_->send_velocity(0.0, angular_z);
  }

  void boundary_move_center()
  {
    const double duration_s = std::max(
      0.2, boundary_recovery_distance_m_ / std::max(0.02, boundary_recovery_speed_mps_));
    if ((now() - boundary_recovery_start_time_).seconds() >= duration_s) {
      publish_stop();
      mode_ = boundary_return_mode_.empty() ? "IDLE" : boundary_return_mode_;
      publish_status("BOUNDARY_RECOVERY_DONE");
      return;
    }
    robot_->send_velocity(boundary_recovery_speed_mps_, 0.0);
  }

  void force_center_waypoint()
  {
    waypoints_.clear();
    append_safe_waypoint(clamp_waypoint({boundary_force_center_x_m_, boundary_force_center_y_m_}));
    waypoint_index_ = 0;
    exit_mode_ = false;
    boundary_recovery_count_ = 0;
    mode_ = "WAYPOINTS";
    publish_status("FORCE_CENTER");
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

  size_t obstacle_count() const
  {
    return std::min(obstacle_centers_x_m_.size(), obstacle_centers_y_m_.size());
  }

  void expand_blue_keepouts_from_expected_targets()
  {
    if (grid_x_centers_m_.empty() || grid_y_centers_m_.empty() ||
      route_expected_target_ids_.empty())
    {
      return;
    }
    const std::set<std::string> expected_orange(
      route_expected_target_ids_.begin(), route_expected_target_ids_.end());
    for (size_t row = 0; row < grid_y_centers_m_.size(); ++row) {
      for (size_t col = 0; col < grid_x_centers_m_.size(); ++col) {
        const std::string cell_id = make_cell_id(row, col, grid_y_centers_m_.size());
        if (expected_orange.count(cell_id) != 0U) {
          continue;
        }
        const double x = grid_x_centers_m_[col];
        const double y = grid_y_centers_m_[row];
        bool already_present = false;
        for (size_t i = 0; i < obstacle_count(); ++i) {
          if (std::hypot(obstacle_centers_x_m_[i] - x, obstacle_centers_y_m_[i] - y) < 1.0e-3) {
            already_present = true;
            break;
          }
        }
        if (!already_present) {
          obstacle_centers_x_m_.push_back(x);
          obstacle_centers_y_m_.push_back(y);
        }
      }
    }
    RCLCPP_INFO(
      get_logger(), "blue keepout map expanded to %zu cells using route_expected_target_ids",
      obstacle_count());
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

  void send_safe_velocity(double linear_x, double angular_z)
  {
    const auto & pos = odom_.pose.pose.position;
    const double yaw = yaw_from_odom(odom_);
    const double next_x = pos.x + std::cos(yaw) * linear_x * safety_prediction_dt_s_;
    const double next_y = pos.y + std::sin(yaw) * linear_x * safety_prediction_dt_s_;
    const double safe_min_x = field_min_x_m_ + boundary_margin_m_;
    const double safe_max_x = field_max_x_m_ - boundary_margin_m_;
    const double safe_min_y = field_min_y_m_ + boundary_margin_m_;
    const double safe_max_y = field_max_y_m_ - boundary_margin_m_;

    if (next_x < safe_min_x || next_x > safe_max_x || next_y < safe_min_y || next_y > safe_max_y) {
      const double center_x = 0.5 * (safe_min_x + safe_max_x);
      const double center_y = 0.5 * (safe_min_y + safe_max_y);
      const double desired_yaw = std::atan2(center_y - pos.y, center_x - pos.x);
      const double yaw_error_to_center = normalize_angle(desired_yaw - yaw);
      angular_z = std::clamp(
        waypoint_yaw_gain_ * yaw_error_to_center,
        -waypoint_max_angular_speed_radps_, waypoint_max_angular_speed_radps_);
      const bool currently_outside =
        pos.x < safe_min_x || pos.x > safe_max_x || pos.y < safe_min_y || pos.y > safe_max_y;
      if (currently_outside &&
        std::abs(yaw_error_to_center) < 120.0 * M_PI / 180.0)
      {
        linear_x = boundary_recovery_speed_mps_;
      } else {
        linear_x = 0.0;
      }
    }

    for (size_t i = 0; i < obstacle_count(); ++i) {
      const double dx = next_x - obstacle_centers_x_m_[i];
      const double dy = next_y - obstacle_centers_y_m_[i];
      if (std::hypot(dx, dy) >= obstacle_avoidance_radius_m_) {
        continue;
      }
      const double away_yaw = std::atan2(pos.y - obstacle_centers_y_m_[i], pos.x - obstacle_centers_x_m_[i]);
      const double yaw_error = normalize_angle(away_yaw - yaw);
      angular_z = std::clamp(
        waypoint_yaw_gain_ * yaw_error,
        -waypoint_max_angular_speed_radps_, waypoint_max_angular_speed_radps_);
      linear_x = -0.04;
      break;
    }
    robot_->send_velocity(linear_x, angular_z);
  }

  void publish_stop()
  {
    if (robot_) {
      robot_->stop();
    }
  }

  void publish_status(const std::string & text)
  {
    if (text == last_status_) {
      return;
    }
    last_status_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "planner status: %s", text.c_str());
  }

  double field_width_m_ {4.0};
  double field_height_m_ {4.0};
  double field_min_x_m_ {0.0};
  double field_min_y_m_ {0.0};
  double field_max_x_m_ {4.0};
  double field_max_y_m_ {4.0};
  double boundary_margin_m_ {0.15};
  double obstacle_forbidden_radius_m_ {0.20};
  double obstacle_avoidance_radius_m_ {0.35};
  std::vector<double> obstacle_centers_x_m_;
  std::vector<double> obstacle_centers_y_m_;
  std::vector<double> grid_x_centers_m_;
  std::vector<double> grid_y_centers_m_;
  std::vector<std::string> route_expected_target_ids_;
  bool expand_all_blue_keepouts_ {false};
  double route_c4_observe_offset_x_m_ {0.0};
  double route_c4_observe_offset_y_m_ {0.0};
  double route_c34_aisle_x_offset_m_ {0.0};
  double route_c34_bypass_x_offset_m_ {-0.30};
  double route_c34_rejoin_y_m_ {std::numeric_limits<double>::quiet_NaN()};
  double route_c3_observe_offset_x_m_ {0.0};
  double route_c3_observe_offset_y_m_ {0.0};
  double route_c23_aisle_x_offset_m_ {0.0};
  double route_c3_aisle_exit_y_m_ {std::numeric_limits<double>::quiet_NaN()};
  double route_c21_observe_offset_x_m_ {0.0};
  double route_c21_observe_offset_y_m_ {0.0};
  double route_bottom_corridor_clearance_m_ {0.50};
  double route_start_side_offset_m_ {0.55};
  double approach_far_speed_mps_ {0.20};
  double approach_near_speed_mps_ {0.12};
  double approach_slow_distance_m_ {0.65};
  double approach_stop_distance_m_ {0.30};
  double align_yaw_tolerance_deg_ {7.0};
  bool dynamic_strike_enabled_ {false};
  double dynamic_strike_trigger_distance_m_ {0.50};
  double angular_gain_ {1.1};
  double approach_max_angular_speed_radps_ {0.18};
  double waypoint_linear_speed_mps_ {0.22};
  double waypoint_position_tolerance_m_ {0.18};
  double waypoint_yaw_gain_ {0.65};
  double waypoint_heading_align_threshold_deg_ {18.0};
  double waypoint_heading_slow_threshold_deg_ {7.0};
  double waypoint_rotate_only_threshold_deg_ {55.0};
  double waypoint_max_angular_speed_radps_ {0.35};
  double exit_linear_speed_mps_ {0.15};
  double boundary_recovery_speed_mps_ {0.08};
  bool boundary_recovery_enabled_ {true};
  int boundary_recovery_pause_count_ {0};
  double boundary_recovery_trigger_margin_m_ {0.15};
  double boundary_recovery_distance_m_ {0.20};
  double boundary_recovery_turn_tolerance_deg_ {12.0};
  int boundary_recovery_max_count_ {3};
  double boundary_force_center_x_m_ {2.0};
  double boundary_force_center_y_m_ {2.0};
  double approach_rotate_only_yaw_deg_ {35.0};
  double close_target_rotate_backoff_distance_m_ {0.45};
  double close_target_backoff_speed_mps_ {-0.06};
  double avoid_back_speed_mps_ {-0.12};
  double avoid_turn_speed_radps_ {0.45};
  double avoid_back_duration_s_ {0.8};
  double avoid_turn_duration_s_ {1.2};
  double strike_speed_mps_ {0.4};
  double strike_duration_s_ {1.0};
  double strike_boundary_slow_margin_m_ {0.40};
  double strike_safe_speed_mps_ {0.08};
  double strike_safe_duration_s_ {0.30};
  double strike_emergency_backoff_distance_m_ {0.20};
  double strike_emergency_backoff_speed_mps_ {0.12};
  double active_strike_speed_mps_ {0.4};
  double active_strike_duration_s_ {1.0};
  double target_distance_m_ {0.0};
  double target_yaw_deg_ {0.0};
  double target_pose_x_m_ {0.0};
  double target_pose_y_m_ {0.0};
  double target_pose_standoff_m_ {0.0};
  int control_period_ms_ {100};
  bool target_use_visual_distance_updates_ {false};
  bool target_use_visual_yaw_updates_ {false};
  double safety_prediction_dt_s_ {0.35};
  double target_update_max_yaw_delta_deg_ {25.0};
  double target_update_max_distance_delta_m_ {0.80};
  double target_lost_timeout_s_ {1.0};
  double stuck_detection_time_s_ {15.0};
  double stuck_detection_distance_m_ {0.10};
  double stuck_recovery_distance_m_ {0.30};
  double stuck_recovery_angle_deg_ {45.0};
  bool low_height_stop_enabled_ {true};
  double low_height_stop_threshold_m_ {0.16};
  double scan_angular_speed_radps_ {0.35};
  double scan_duration_s_ {4.5};
  bool target_distance_initialized_ {false};
  bool target_yaw_initialized_ {false};
  bool target_pose_initialized_ {false};
  bool strike_boundary_degraded_ {false};
  bool strike_log_emitted_ {false};
  bool exit_mode_ {false};
  int boundary_recovery_count_ {0};
  std::string boundary_return_mode_ {"IDLE"};
  rclcpp::Time boundary_recovery_start_time_;
  std::string stuck_return_mode_ {"IDLE"};
  rclcpp::Time stuck_watch_start_time_;
  double stuck_watch_x_m_ {0.0};
  double stuck_watch_y_m_ {0.0};
  bool have_odom_ {false};
  size_t waypoint_index_ {0};
  double face_target_yaw_deg_ {0.0};
  std::string mode_ {"IDLE"};
  std::string last_status_;
  rclcpp::Time phase_start_;
  rclcpp::Time last_control_time_;
  rclcpp::Time last_target_seen_time_;
  nav_msgs::msg::Odometry odom_;
  wild_glint_hunt::msg::VisionBallArray vision_;
  std::vector<Waypoint> waypoints_;
  RobotInterface::SharedPtr robot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<wild_glint_hunt::msg::VisionBallArray>::SharedPtr vision_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace wild_glint_hunt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wild_glint_hunt::PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
