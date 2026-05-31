#include "wild_glint_hunt/robot_interface.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"

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

sensor_msgs::msg::Range tof_payload_to_range(
  const protocol::msg::SingleTofPayload & payload,
  const std::string & frame_id)
{
  sensor_msgs::msg::Range range;
  range.header = payload.header;
  range.header.frame_id = frame_id;
  range.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range.field_of_view = 0.25F;
  range.min_range = 0.15F;
  range.max_range = 0.66F;
  if (!payload.data_available || payload.data.empty()) {
    range.range = std::numeric_limits<float>::quiet_NaN();
    return range;
  }
  range.range = *std::min_element(payload.data.begin(), payload.data.end());
  return range;
}

}  // namespace

RobotInterfaceReal::RobotInterfaceReal(const rclcpp::Node::SharedPtr & node) : node_(node)
{
  const auto odom_topic = read_parameter<std::string>(node_, "official_odom_topic", "odom_out");
  const auto imu_topic = read_parameter<std::string>(node_, "official_imu_topic", "imu");
  const auto ultrasonic_topic =
    read_parameter<std::string>(node_, "official_ultrasonic_topic", "ultrasonic_payload");
  const auto tof_topic = read_parameter<std::string>(node_, "official_tof_topic", "head_tof_payload");
  const auto rear_tof_topic =
    read_parameter<std::string>(node_, "official_rear_tof_topic", "rear_tof_payload");
  const auto motion_cmd_topic =
    read_parameter<std::string>(node_, "official_motion_servo_topic", "motion_servo_cmd");
  read_parameter<int>(node_, "official_walk_motion_id", 303);
  const auto qos_depth = read_parameter<int>(node_, "robot_interface_qos_depth", 10);
  publish_rate_hz_ = read_parameter<double>(node_, "motion_command_rate_hz", 30.0);
  head_butt_speed_mps_ = read_parameter<double>(node_, "head_butt_speed_mps", 0.4);
  head_butt_duration_s_ = read_parameter<double>(node_, "head_butt_duration_s", 1.0);

  real_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, qos_depth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      latest_odom_ = *msg;
    });
  real_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      latest_imu_ = *msg;
    });
  real_ultrasonic_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    ultrasonic_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Range::SharedPtr msg) {
      latest_ultrasonic_ = *msg;
    });
  real_head_tof_sub_ = node_->create_subscription<protocol::msg::HeadTofPayload>(
    tof_topic, qos_depth, [this](const protocol::msg::HeadTofPayload::SharedPtr msg) {
      latest_tof_ = tof_payload_to_range(msg->left_head, "left_head_tof");
    });
  real_rear_tof_sub_ = node_->create_subscription<protocol::msg::RearTofPayload>(
    rear_tof_topic, qos_depth, [this](const protocol::msg::RearTofPayload::SharedPtr msg) {
      latest_tof_ = tof_payload_to_range(msg->left_rear, "left_rear_tof");
    });
  (void)real_tof_sub_;
  real_motion_pub_ =
    node_->create_publisher<protocol::msg::MotionServoCmd>(motion_cmd_topic, qos_depth);
  real_motion_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_)),
    [this]() { send_velocity(commanded_linear_x_, commanded_lateral_y_, commanded_angular_z_); });
}

void RobotInterfaceReal::move_forward(double speed, double duration)
{
  send_velocity(speed, 0.0);
  const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
  rclcpp::Rate rate(publish_rate_hz_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
    send_velocity(speed, 0.0);
    rate.sleep();
  }
  stop();
}

void RobotInterfaceReal::turn(double angular_velocity, double duration)
{
  send_velocity(0.0, angular_velocity);
  const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
  rclcpp::Rate rate(publish_rate_hz_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
    send_velocity(0.0, angular_velocity);
    rate.sleep();
  }
  stop();
}

void RobotInterfaceReal::stop()
{
  send_velocity(0.0, 0.0);
}

nav_msgs::msg::Odometry RobotInterfaceReal::get_odometry() const { return latest_odom_; }
sensor_msgs::msg::Imu RobotInterfaceReal::get_imu() const { return latest_imu_; }
sensor_msgs::msg::Range RobotInterfaceReal::get_ultrasonic() const { return latest_ultrasonic_; }
sensor_msgs::msg::Range RobotInterfaceReal::get_tof() const { return latest_tof_; }

void RobotInterfaceReal::execute_head_butt()
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

void RobotInterfaceReal::send_velocity(double linear_x, double angular_z)
{
  send_velocity(linear_x, 0.0, angular_z);
}

void RobotInterfaceReal::send_velocity(double linear_x, double lateral_y, double angular_z)
{
  commanded_linear_x_ = linear_x;
  commanded_lateral_y_ = lateral_y;
  commanded_angular_z_ = angular_z;

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
  real_motion_pub_->publish(cmd);
}

std::string RobotInterfaceReal::backend_name() const
{
  return "official_protocol_motion_servo_backend";
}

}  // namespace wild_glint_hunt
