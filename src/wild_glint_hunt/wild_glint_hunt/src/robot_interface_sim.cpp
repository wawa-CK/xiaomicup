#include "wild_glint_hunt/robot_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <thread>

#include "wild_glint_hunt/lcm/gamepad_lcmt.hpp"

namespace wild_glint_hunt
{
namespace
{

template<typename T>
T read_parameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const T & fallback)
{
  T value;
  if (node->get_parameter(name, value)) {
    return value;
  }
  return node->declare_parameter<T>(name, fallback);
}

gamepad_lcmt make_zero_gamepad_command()
{
  gamepad_lcmt command;
  command.leftBumper = 0;
  command.rightBumper = 0;
  command.leftTriggerButton = 0;
  command.rightTriggerButton = 0;
  command.back = 0;
  command.start = 0;
  command.a = 0;
  command.b = 0;
  command.x = 0;
  command.y = 0;
  command.leftStickButton = 0;
  command.rightStickButton = 0;
  command.leftTriggerAnalog = 0.0F;
  command.rightTriggerAnalog = 0.0F;
  command.leftStickAnalog[0] = 0.0F;
  command.leftStickAnalog[1] = 0.0F;
  command.rightStickAnalog[0] = 0.0F;
  command.rightStickAnalog[1] = 0.0F;
  return command;
}

}  // namespace

double yaw_from_odom(const nav_msgs::msg::Odometry & odom)
{
  const auto & q = odom.pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

RobotInterfaceSim::RobotInterfaceSim(const rclcpp::Node::SharedPtr & node) : node_(node)
{
  const auto cmd_vel_topic = read_parameter<std::string>(node_, "sim_cmd_vel_topic", "/cmd_vel");
  const auto official_cmd_topic =
    read_parameter<std::string>(node_, "official_motion_servo_topic", "motion_servo_cmd");
  apply_force_topic_ = read_parameter<std::string>(node_, "apply_force_topic", "/apply_force");
  apply_force_link_name_ = read_parameter<std::string>(node_, "apply_force_link_name", "base_link");
  gazebo_wrench_service_ =
    read_parameter<std::string>(node_, "gazebo_wrench_service", "/apply_link_wrench");
  gazebo_wrench_link_name_ =
    read_parameter<std::string>(node_, "gazebo_wrench_link_name", "cyberdog::base_link");
  enable_apply_force_backend_ = read_parameter<bool>(node_, "enable_apply_force_backend", false);
  enable_gazebo_wrench_backend_ = read_parameter<bool>(node_, "enable_gazebo_wrench_backend", false);
  enable_lcm_gamepad_backend_ = read_parameter<bool>(node_, "enable_lcm_gamepad_backend", true);
  lcm_gamepad_channel_ = read_parameter<std::string>(node_, "lcm_gamepad_channel", "gamepad_lcmt");
  const auto odom_topic = read_parameter<std::string>(node_, "odom_topic", "/odom");
  const auto imu_topic = read_parameter<std::string>(node_, "imu_topic", "/imu");
  const auto ultrasonic_topic =
    read_parameter<std::string>(node_, "ultrasonic_topic", "ultrasonic_payload");
  const auto tof_topic = read_parameter<std::string>(node_, "tof_topic", "/tof");
  const auto qos_depth = read_parameter<int>(node_, "robot_interface_qos_depth", 10);
  publish_rate_hz_ = read_parameter<double>(node_, "motion_command_rate_hz", 20.0);
  head_butt_speed_mps_ = read_parameter<double>(node_, "head_butt_speed_mps", 0.4);
  head_butt_duration_s_ = read_parameter<double>(node_, "head_butt_duration_s", 1.0);
  apply_force_duration_s_ = read_parameter<double>(node_, "apply_force_duration_s", 0.05);
  linear_force_gain_n_per_mps_ =
    read_parameter<double>(node_, "linear_force_gain_n_per_mps", 250.0);
  angular_force_gain_n_per_radps_ =
    read_parameter<double>(node_, "angular_force_gain_n_per_radps", 120.0);
  turn_force_arm_m_ = read_parameter<double>(node_, "turn_force_arm_m", 0.25);
  gazebo_wrench_duration_s_ = read_parameter<double>(node_, "gazebo_wrench_duration_s", 0.15);
  gazebo_linear_wrench_gain_n_per_mps_ =
    read_parameter<double>(node_, "gazebo_linear_wrench_gain_n_per_mps", 3000.0);
  gazebo_angular_wrench_gain_nm_per_radps_ =
    read_parameter<double>(node_, "gazebo_angular_wrench_gain_nm_per_radps", 800.0);
  lcm_gamepad_linear_scale_ = read_parameter<double>(node_, "lcm_gamepad_linear_scale", 3.0);
  lcm_gamepad_angular_scale_ = read_parameter<double>(node_, "lcm_gamepad_angular_scale", 2.0);
  lcm_gamepad_init_cycles_ = read_parameter<int>(node_, "lcm_gamepad_init_cycles", 20);
  lcm_gamepad_init_rest_cycles_ = read_parameter<int>(node_, "lcm_gamepad_init_rest_cycles", 20);
  lcm_gamepad_init_period_ms_ = read_parameter<int>(node_, "lcm_gamepad_init_period_ms", 20);
  stand_ready_height_threshold_m_ =
    read_parameter<double>(node_, "stand_ready_height_threshold_m", 0.18);
  stand_ready_wait_ms_ = read_parameter<int>(node_, "stand_ready_wait_ms", 6000);
  lcm_gamepad_retry_cycles_ = read_parameter<int>(node_, "lcm_gamepad_retry_cycles", 120);
  read_parameter<int>(node_, "official_walk_motion_id", 303);

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos_depth);
  official_cmd_pub_ =
    node_->create_publisher<protocol::msg::MotionServoCmd>(official_cmd_topic, qos_depth);
#ifdef WGH_HAS_CYBERDOG_MSG
  apply_force_pub_ =
    node_->create_publisher<cyberdog_msg::msg::ApplyForce>(apply_force_topic_, qos_depth);
  yaml_param_pub_ =
    node_->create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter", qos_depth);
#else
  RCLCPP_WARN(
    node_->get_logger(),
    "cyberdog_msg is unavailable at build time; Gazebo /apply_force output is disabled");
#endif
  if (enable_lcm_gamepad_backend_) {
    gamepad_lcm_ = std::make_shared<lcm::LCM>();
    if (!gamepad_lcm_->good()) {
      RCLCPP_ERROR(node_->get_logger(), "LCM gamepad backend requested but lcm::LCM is not good");
      gamepad_lcm_.reset();
    }
  }
#ifdef WGH_HAS_GAZEBO_MSGS
  gazebo_wrench_client_ =
    node_->create_client<gazebo_msgs::srv::ApplyLinkWrench>(gazebo_wrench_service_);
#else
  if (enable_gazebo_wrench_backend_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "gazebo_msgs is unavailable at build time; Gazebo wrench output is disabled");
  }
#endif
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, qos_depth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom_ = *msg;
    });
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      latest_imu_ = *msg;
    });
  ultrasonic_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    ultrasonic_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Range::SharedPtr msg) {
      latest_ultrasonic_ = *msg;
    });
  tof_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    tof_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Range::SharedPtr msg) {
      latest_tof_ = *msg;
  });
}

void RobotInterfaceSim::publish_use_rc_parameter() const
{
#ifdef WGH_HAS_CYBERDOG_MSG
  if (!yaml_param_pub_) {
    return;
  }
  cyberdog_msg::msg::YamlParam msg;
  msg.name = "use_rc";
  msg.kind = cyberdog_msg::msg::YamlParam::S64;
  msg.s64_value = 0;
  msg.is_user = 0;
  yaml_param_pub_->publish(msg);
#endif
}

void RobotInterfaceSim::publish_control_parameter(const std::string & name, int64_t value) const
{
#ifdef WGH_HAS_CYBERDOG_MSG
  if (!yaml_param_pub_) {
    return;
  }
  cyberdog_msg::msg::YamlParam msg;
  msg.name = name;
  msg.kind = cyberdog_msg::msg::YamlParam::S64;
  msg.s64_value = value;
  msg.is_user = 0;
  yaml_param_pub_->publish(msg);
#else
  (void)name;
  (void)value;
#endif
}

void RobotInterfaceSim::initialize_gamepad_mode() const
{
  if (!enable_lcm_gamepad_backend_ || !gamepad_lcm_ || lcm_gamepad_mode_initialized_) {
    return;
  }
  const auto init_start = std::chrono::steady_clock::now();
  publish_use_rc_parameter();
  publish_gamepad_button('b', lcm_gamepad_init_cycles_, lcm_gamepad_init_rest_cycles_);
  publish_gamepad_button('x', lcm_gamepad_init_cycles_, lcm_gamepad_init_rest_cycles_);
  publish_gamepad_button('y', lcm_gamepad_init_cycles_, lcm_gamepad_init_rest_cycles_);

  auto is_standing = [this]() {
      return latest_odom_.pose.pose.position.z >= stand_ready_height_threshold_m_;
    };

  while (rclcpp::ok() && !is_standing()) {
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - init_start).count();
    if (elapsed_ms >= stand_ready_wait_ms_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "robot height %.3f below stand-ready threshold %.3f after %d ms; retrying locomotion button",
        latest_odom_.pose.pose.position.z, stand_ready_height_threshold_m_, stand_ready_wait_ms_);
      publish_gamepad_button('x', lcm_gamepad_retry_cycles_, lcm_gamepad_init_rest_cycles_);
      publish_gamepad_button('y', lcm_gamepad_retry_cycles_, lcm_gamepad_init_rest_cycles_);
      break;
    }
    gamepad_lcmt command = make_zero_gamepad_command();
    gamepad_lcm_->publish(lcm_gamepad_channel_, &command);
    std::this_thread::sleep_for(std::chrono::milliseconds(lcm_gamepad_init_period_ms_));
  }

  lcm_gamepad_mode_initialized_ = true;
  RCLCPP_INFO(
    node_->get_logger(),
    "Initialized Gazebo LCM gamepad control: height=%.3f threshold=%.3f",
    latest_odom_.pose.pose.position.z, stand_ready_height_threshold_m_);
}

void RobotInterfaceSim::publish_gamepad_velocity(double linear_x, double lateral_y, double angular_z) const
{
  if (!enable_lcm_gamepad_backend_ || !gamepad_lcm_) {
    return;
  }
  initialize_gamepad_mode();
  gamepad_lcmt command = make_zero_gamepad_command();
  command.leftStickAnalog[1] =
    static_cast<float>(std::clamp(linear_x * lcm_gamepad_linear_scale_, -1.0, 1.0));
  command.leftStickAnalog[0] =
    static_cast<float>(std::clamp(lateral_y * lcm_gamepad_linear_scale_, -1.0, 1.0));
  command.rightStickAnalog[0] =
    static_cast<float>(std::clamp(-angular_z * lcm_gamepad_angular_scale_, -1.0, 1.0));
  command.rightStickAnalog[1] = 0.0F;
  publish_use_rc_parameter();
  gamepad_lcm_->publish(lcm_gamepad_channel_, &command);
}

void RobotInterfaceSim::publish_gamepad_button(char button, int cycles, int rest_cycles) const
{
  if (!enable_lcm_gamepad_backend_ || !gamepad_lcm_) {
    return;
  }
  gamepad_lcmt command = make_zero_gamepad_command();
  switch (button) {
    case 'a':
      command.a = 1;
      break;
    case 'b':
      command.b = 1;
      break;
    case 'x':
      command.x = 1;
      break;
    case 'y':
      command.y = 1;
      break;
    default:
      return;
  }
  const auto period = std::chrono::milliseconds(lcm_gamepad_init_period_ms_);
  for (int i = 0; i < cycles; ++i) {
    publish_use_rc_parameter();
    gamepad_lcm_->publish(lcm_gamepad_channel_, &command);
    std::this_thread::sleep_for(period);
  }
  command = make_zero_gamepad_command();
  for (int i = 0; i < rest_cycles; ++i) {
    gamepad_lcm_->publish(lcm_gamepad_channel_, &command);
    std::this_thread::sleep_for(period);
  }
}

void RobotInterfaceSim::publish_apply_force(double linear_x, double angular_z) const
{
#ifdef WGH_HAS_CYBERDOG_MSG
  if (!enable_apply_force_backend_) {
    return;
  }
  if (!apply_force_pub_) {
    return;
  }

  const double yaw = yaw_from_odom(latest_odom_);
  const double forward_force = linear_force_gain_n_per_mps_ * linear_x;
  const double turn_force = angular_force_gain_n_per_radps_ * angular_z;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  auto publish_force = [this](
    const std::array<double, 3> & force, const std::array<double, 3> & rel_pos) {
      cyberdog_msg::msg::ApplyForce msg;
      msg.link_name = apply_force_link_name_;
      msg.time = apply_force_duration_s_;
      for (size_t i = 0; i < 3; ++i) {
        msg.force[i] = force[i];
        msg.rel_pos[i] = rel_pos[i];
      }
      apply_force_pub_->publish(msg);
    };

  if (std::abs(forward_force) > 1e-6) {
    publish_force({forward_force * cos_yaw, forward_force * sin_yaw, 0.0}, {0.0, 0.0, 0.0});
  }
  if (std::abs(turn_force) > 1e-6) {
    publish_force({-turn_force * sin_yaw, turn_force * cos_yaw, 0.0}, {turn_force_arm_m_, 0.0, 0.0});
  }
  if (std::abs(forward_force) <= 1e-6 && std::abs(turn_force) <= 1e-6) {
    publish_force({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  }
#else
  (void)linear_x;
  (void)angular_z;
#endif
}

void RobotInterfaceSim::publish_gazebo_wrench(double linear_x, double angular_z) const
{
#ifdef WGH_HAS_GAZEBO_MSGS
  if (!enable_gazebo_wrench_backend_ || !gazebo_wrench_client_ ||
    !gazebo_wrench_client_->service_is_ready())
  {
    return;
  }
  const double yaw = yaw_from_odom(latest_odom_);
  const double force = gazebo_linear_wrench_gain_n_per_mps_ * linear_x;
  auto request = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
  request->link_name = gazebo_wrench_link_name_;
  request->reference_frame = "world";
  request->reference_point.x = 0.0;
  request->reference_point.y = 0.0;
  request->reference_point.z = 0.0;
  request->wrench.force.x = force * std::cos(yaw);
  request->wrench.force.y = force * std::sin(yaw);
  request->wrench.force.z = 0.0;
  request->wrench.torque.x = 0.0;
  request->wrench.torque.y = 0.0;
  request->wrench.torque.z = gazebo_angular_wrench_gain_nm_per_radps_ * angular_z;
  request->start_time.sec = 0;
  request->start_time.nanosec = 0;
  request->duration.sec = static_cast<int32_t>(gazebo_wrench_duration_s_);
  request->duration.nanosec =
    static_cast<uint32_t>((gazebo_wrench_duration_s_ - request->duration.sec) * 1e9);
  (void)gazebo_wrench_client_->async_send_request(request);
#else
  (void)linear_x;
  (void)angular_z;
#endif
}

void RobotInterfaceSim::publish_for_duration(double linear_x, double angular_z, double duration_s) const
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.angular.z = angular_z;
  const bool use_wrench_only = enable_gazebo_wrench_backend_;
  const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
  rclcpp::Rate rate(publish_rate_hz_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
    cmd_pub_->publish(twist);
    publish_official_velocity(linear_x, 0.0, angular_z);
    publish_gazebo_wrench(linear_x, angular_z);
    if (!use_wrench_only) {
      publish_gamepad_velocity(linear_x, 0.0, angular_z);
      publish_apply_force(linear_x, angular_z);
    }
    rate.sleep();
  }
}

void RobotInterfaceSim::publish_official_velocity(double linear_x, double lateral_y, double angular_z) const
{
  protocol::msg::MotionServoCmd cmd;
  int motion_id = 303;
  if (!node_->get_parameter("official_walk_motion_id", motion_id)) {
    motion_id = 303;
  }
  cmd.motion_id = motion_id;
  cmd.cmd_type = protocol::msg::MotionServoCmd::SERVO_DATA;
  cmd.cmd_source = protocol::msg::MotionServoCmd::VIS;
  cmd.value = 0;
  cmd.vel_des = {
    static_cast<float>(linear_x),
    static_cast<float>(lateral_y),
    static_cast<float>(angular_z)};
  cmd.rpy_des = {0.0F, 0.0F, 0.0F};
  cmd.pos_des = {0.0F, 0.0F, 0.0F};
  cmd.acc_des = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  cmd.ctrl_point = {0.0F, 0.0F, 0.0F};
  cmd.foot_pose = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  cmd.step_height = {0.05F, 0.05F};
  official_cmd_pub_->publish(cmd);
}

void RobotInterfaceSim::move_forward(double speed, double duration)
{
  publish_for_duration(speed, 0.0, duration);
  stop();
}

void RobotInterfaceSim::turn(double angular_velocity, double duration)
{
  publish_for_duration(0.0, angular_velocity, duration);
  stop();
}

void RobotInterfaceSim::stop()
{
  geometry_msgs::msg::Twist twist;
  cmd_pub_->publish(twist);
  publish_official_velocity(0.0, 0.0, 0.0);
  publish_gazebo_wrench(0.0, 0.0);
  if (!enable_gazebo_wrench_backend_) {
    publish_gamepad_velocity(0.0, 0.0, 0.0);
    publish_apply_force(0.0, 0.0);
  }
}

nav_msgs::msg::Odometry RobotInterfaceSim::get_odometry() const { return latest_odom_; }

sensor_msgs::msg::Imu RobotInterfaceSim::get_imu() const { return latest_imu_; }

sensor_msgs::msg::Range RobotInterfaceSim::get_ultrasonic() const { return latest_ultrasonic_; }

sensor_msgs::msg::Range RobotInterfaceSim::get_tof() const { return latest_tof_; }

void RobotInterfaceSim::execute_head_butt()
{
  if (std::abs(head_butt_speed_mps_) < 1.0e-6 || head_butt_duration_s_ <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "STRIKE: invalid strike command, linear=%.3f duration=%.3f",
      head_butt_speed_mps_, head_butt_duration_s_);
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "STRIKE: sending velocity command: linear=%.2f, duration=%.2f",
    head_butt_speed_mps_, head_butt_duration_s_);
  move_forward(head_butt_speed_mps_, head_butt_duration_s_);
  RCLCPP_INFO(node_->get_logger(), "STRIKE: velocity command sent successfully");
}

void RobotInterfaceSim::send_velocity(double linear_x, double angular_z)
{
  send_velocity(linear_x, 0.0, angular_z);
}

void RobotInterfaceSim::send_velocity(double linear_x, double lateral_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_x;
  twist.linear.y = lateral_y;
  twist.angular.z = angular_z;
  cmd_pub_->publish(twist);
  publish_official_velocity(linear_x, lateral_y, angular_z);
  publish_gazebo_wrench(linear_x, angular_z);
  if (!enable_gazebo_wrench_backend_) {
    publish_gamepad_velocity(linear_x, lateral_y, angular_z);
    publish_apply_force(linear_x, angular_z);
  }
}

std::string RobotInterfaceSim::backend_name() const { return "sim_twist_lcm_gamepad_backend"; }

}  // namespace wild_glint_hunt
