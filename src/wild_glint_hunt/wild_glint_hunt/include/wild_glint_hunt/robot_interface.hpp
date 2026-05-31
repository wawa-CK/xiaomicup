#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "lcm/lcm-cpp.hpp"
#ifdef WGH_HAS_CYBERDOG_MSG
#include "cyberdog_msg/msg/apply_force.hpp"
#include "cyberdog_msg/msg/yaml_param.hpp"
#endif
#ifdef WGH_HAS_GAZEBO_MSGS
#include "gazebo_msgs/srv/apply_link_wrench.hpp"
#endif
#include "protocol/msg/head_tof_payload.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/rear_tof_payload.hpp"

namespace wild_glint_hunt
{

struct MotionCommand
{
  double linear_x {0.0};
  double angular_z {0.0};
  double duration_s {0.0};
};

class RobotInterface
{
public:
  using SharedPtr = std::shared_ptr<RobotInterface>;
  virtual ~RobotInterface() = default;

  virtual void move_forward(double speed, double duration) = 0;
  virtual void turn(double angular_velocity, double duration) = 0;
  virtual void stop() = 0;
  virtual nav_msgs::msg::Odometry get_odometry() const = 0;
  virtual sensor_msgs::msg::Imu get_imu() const = 0;
  virtual sensor_msgs::msg::Range get_ultrasonic() const = 0;
  virtual sensor_msgs::msg::Range get_tof() const = 0;
  virtual void execute_head_butt() = 0;
  virtual void send_velocity(double linear_x, double angular_z) = 0;
  virtual void send_velocity(double linear_x, double lateral_y, double angular_z) = 0;
  virtual std::string backend_name() const = 0;
};

class RobotInterfaceSim : public RobotInterface
{
public:
  explicit RobotInterfaceSim(const rclcpp::Node::SharedPtr & node);

  void move_forward(double speed, double duration) override;
  void turn(double angular_velocity, double duration) override;
  void stop() override;
  nav_msgs::msg::Odometry get_odometry() const override;
  sensor_msgs::msg::Imu get_imu() const override;
  sensor_msgs::msg::Range get_ultrasonic() const override;
  sensor_msgs::msg::Range get_tof() const override;
  void execute_head_butt() override;
  void send_velocity(double linear_x, double angular_z) override;
  void send_velocity(double linear_x, double lateral_y, double angular_z) override;
  std::string backend_name() const override;

private:
  void publish_for_duration(double linear_x, double angular_z, double duration_s) const;
  void publish_official_velocity(double linear_x, double lateral_y, double angular_z) const;
  void publish_apply_force(double linear_x, double angular_z) const;
  void publish_gazebo_wrench(double linear_x, double angular_z) const;
  void publish_gamepad_velocity(double linear_x, double lateral_y, double angular_z) const;
  void publish_gamepad_button(char button, int cycles, int rest_cycles) const;
  void publish_use_rc_parameter() const;
  void publish_control_parameter(const std::string & name, int64_t value) const;
  void initialize_gamepad_mode() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr official_cmd_pub_;
#ifdef WGH_HAS_CYBERDOG_MSG
  rclcpp::Publisher<cyberdog_msg::msg::ApplyForce>::SharedPtr apply_force_pub_;
  rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr yaml_param_pub_;
#endif
#ifdef WGH_HAS_GAZEBO_MSGS
  rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr gazebo_wrench_client_;
#endif
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::Imu latest_imu_;
  sensor_msgs::msg::Range latest_ultrasonic_;
  sensor_msgs::msg::Range latest_tof_;
  std::string apply_force_topic_ {"/apply_force"};
  std::string apply_force_link_name_ {"base_link"};
  std::string gazebo_wrench_service_ {"/apply_link_wrench"};
  std::string gazebo_wrench_link_name_ {"robot::base_link"};
  bool enable_apply_force_backend_ {false};
  bool enable_gazebo_wrench_backend_ {true};
  bool enable_lcm_gamepad_backend_ {true};
  mutable bool lcm_gamepad_mode_initialized_ {false};
  std::string lcm_gamepad_channel_ {"gamepad_lcmt"};
  double publish_rate_hz_ {20.0};
  double head_butt_speed_mps_ {0.4};
  double head_butt_duration_s_ {1.0};
  double apply_force_duration_s_ {0.05};
  double linear_force_gain_n_per_mps_ {250.0};
  double angular_force_gain_n_per_radps_ {120.0};
  double turn_force_arm_m_ {0.25};
  double gazebo_wrench_duration_s_ {0.15};
  double gazebo_linear_wrench_gain_n_per_mps_ {3000.0};
  double gazebo_angular_wrench_gain_nm_per_radps_ {800.0};
  double lcm_gamepad_linear_scale_ {3.0};
  double lcm_gamepad_angular_scale_ {2.0};
  int lcm_gamepad_init_cycles_ {20};
  int lcm_gamepad_init_rest_cycles_ {20};
  int lcm_gamepad_init_period_ms_ {20};
  double stand_ready_height_threshold_m_ {0.18};
  int stand_ready_wait_ms_ {6000};
  int lcm_gamepad_retry_cycles_ {120};
  std::shared_ptr<lcm::LCM> gamepad_lcm_;
};

class RobotInterfaceReal : public RobotInterface
{
public:
  explicit RobotInterfaceReal(const rclcpp::Node::SharedPtr & node);

  void move_forward(double speed, double duration) override;
  void turn(double angular_velocity, double duration) override;
  void stop() override;
  nav_msgs::msg::Odometry get_odometry() const override;
  sensor_msgs::msg::Imu get_imu() const override;
  sensor_msgs::msg::Range get_ultrasonic() const override;
  sensor_msgs::msg::Range get_tof() const override;
  void execute_head_butt() override;
  void send_velocity(double linear_x, double angular_z) override;
  void send_velocity(double linear_x, double lateral_y, double angular_z) override;
  std::string backend_name() const override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr real_motion_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr real_tof_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr real_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr real_ultrasonic_sub_;
  rclcpp::Subscription<protocol::msg::HeadTofPayload>::SharedPtr real_head_tof_sub_;
  rclcpp::Subscription<protocol::msg::RearTofPayload>::SharedPtr real_rear_tof_sub_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr real_motion_pub_;
  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::Imu latest_imu_;
  sensor_msgs::msg::Range latest_ultrasonic_;
  sensor_msgs::msg::Range latest_tof_;
  double publish_rate_hz_ {30.0};
  double head_butt_speed_mps_ {0.4};
  double head_butt_duration_s_ {1.0};
  double commanded_linear_x_ {0.0};
  double commanded_lateral_y_ {0.0};
  double commanded_angular_z_ {0.0};
};

}  // namespace wild_glint_hunt
