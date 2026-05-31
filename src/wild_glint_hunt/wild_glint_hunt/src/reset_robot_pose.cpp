#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("reset_robot_pose");
  const std::string model_name = node->declare_parameter<std::string>("model_name", "robot");
  const double x = node->declare_parameter<double>("spawn_x", -0.45);
  const double y = node->declare_parameter<double>("spawn_y", 0.70);
  const double z = node->declare_parameter<double>("spawn_z", 0.32);
  const double roll = node->declare_parameter<double>("spawn_roll", 0.0);
  const double pitch = node->declare_parameter<double>("spawn_pitch", 0.0);
  const double yaw = node->declare_parameter<double>("spawn_yaw", 0.0);
  const int delay_ms = node->declare_parameter<int>("delay_ms", 3000);

  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  const std::string command =
    "gz model -m " + model_name +
    " -x " + std::to_string(x) +
    " -y " + std::to_string(y) +
    " -z " + std::to_string(z) +
    " -R " + std::to_string(roll) +
    " -P " + std::to_string(pitch) +
    " -Y " + std::to_string(yaw);
  const int ret = std::system(command.c_str());
  if (ret != 0) {
    RCLCPP_ERROR(node->get_logger(), "failed to reset robot pose with: %s", command.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "reset robot pose to second-stage entry: x=%.3f y=%.3f", x, y);
  }
  rclcpp::shutdown();
  return ret;
}
