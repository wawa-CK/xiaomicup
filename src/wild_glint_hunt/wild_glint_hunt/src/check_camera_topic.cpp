#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CheckCameraTopicNode : public rclcpp::Node
{
public:
  CheckCameraTopicNode() : Node("check_camera_topic")
  {
    topic_ = declare_parameter<std::string>("camera_topic", "/image_left");
    timeout_s_ = declare_parameter<double>("camera_timeout_s", 5.0);
    start_time_ = now();
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (received_) {
          return;
        }
        received_ = true;
        RCLCPP_INFO(
          get_logger(), "camera topic %s is active: %ux%u encoding=%s frame=%s",
          topic_.c_str(), msg->width, msg->height, msg->encoding.c_str(),
          msg->header.frame_id.c_str());
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (received_) {
        return;
      }
      if ((now() - start_time_).seconds() >= timeout_s_) {
        RCLCPP_ERROR(
          get_logger(),
          "No image received on %s within %.1fs. Rebuild/source cyberdog_description after "
          "editing gazebo.xacro, then restart Gazebo. Do not enable simulated_sensors_node "
          "image publishing for real-camera tests.",
          topic_.c_str(), timeout_s_);
        received_ = true;
      }
    });
  }

private:
  std::string topic_;
  double timeout_s_ {5.0};
  bool received_ {false};
  rclcpp::Time start_time_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CheckCameraTopicNode>());
  rclcpp::shutdown();
  return 0;
}
