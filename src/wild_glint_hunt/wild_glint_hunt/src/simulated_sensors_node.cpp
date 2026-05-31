#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "protocol/msg/head_tof_payload.hpp"
#include "protocol/msg/rear_tof_payload.hpp"

namespace wild_glint_hunt
{
namespace
{

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std::vector<cv::Point> read_points(const std::vector<int64_t> & raw)
{
  std::vector<cv::Point> points;
  for (size_t i = 0; i + 1 < raw.size(); i += 2) {
    points.emplace_back(static_cast<int>(raw[i]), static_cast<int>(raw[i + 1]));
  }
  return points;
}

std::vector<cv::Point> make_grid_points(
  const std::vector<int64_t> & x_centers,
  const std::vector<int64_t> & y_centers,
  const std::vector<size_t> & cols_by_row)
{
  std::vector<cv::Point> points;
  for (size_t row = 0; row < y_centers.size() && row < cols_by_row.size(); ++row) {
    const auto col = cols_by_row[row];
    if (col < x_centers.size()) {
      points.emplace_back(static_cast<int>(x_centers[col]), static_cast<int>(y_centers[row]));
    }
  }
  return points;
}

struct WorldBall
{
  std::string label;
  int id {0};
  double x_m {0.0};
  double y_m {0.0};
  bool visible {true};
};

}  // namespace

class SimulatedSensorsNode : public rclcpp::Node
{
public:
  SimulatedSensorsNode() : Node("simulated_sensors_node")
  {
    image_topic_ = declare_parameter<std::string>("sim_image_topic", "/image_left");
    camera_info_topic_ = declare_parameter<std::string>("sim_camera_info_topic", "/image_left/camera_info");
    rgb_image_topic_ = declare_parameter<std::string>("sim_rgb_image_topic", "/rgb_camera/image_raw");
    rgb_camera_info_topic_ =
      declare_parameter<std::string>("sim_rgb_camera_info_topic", "/rgb_camera/camera_info");
    odom_topic_ = declare_parameter<std::string>("sim_odom_topic", "/odom");
    body_state_topic_ = declare_parameter<std::string>("sim_body_state_topic", "/body_state");
    official_ultrasonic_topic_ =
      declare_parameter<std::string>("official_ultrasonic_topic", "ultrasonic_payload");
    official_tof_topic_ = declare_parameter<std::string>("official_tof_topic", "head_tof_payload");
    official_rear_tof_topic_ =
      declare_parameter<std::string>("official_rear_tof_topic", "rear_tof_payload");
    imu_topic_ = declare_parameter<std::string>("sim_imu_topic", "/imu");
    obstacle_topic_ = declare_parameter<std::string>("sim_obstacle_topic", "/obstacle_detection");
    tof_topic_ = declare_parameter<std::string>("sim_tof_topic", "/tof");
    cmd_vel_topic_ = declare_parameter<std::string>("sim_cmd_vel_topic", "/cmd_vel");
    frame_id_ = declare_parameter<std::string>("sim_camera_frame_id", "RGB_camera_link");
    odom_frame_id_ = declare_parameter<std::string>("sim_odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("sim_base_frame_id", "base_link");
    image_width_ = declare_parameter<int>("sim_image_width", 640);
    image_height_ = declare_parameter<int>("sim_image_height", 480);
    camera_rate_hz_ = declare_parameter<double>("sim_camera_rate_hz", 10.0);
    state_rate_hz_ = declare_parameter<double>("sim_state_rate_hz", 30.0);
    publish_images_ = declare_parameter<bool>("sim_publish_images", false);
    sync_gazebo_pose_ = declare_parameter<bool>("sim_sync_gazebo_model_pose", false);
    drive_gazebo_pose_ = declare_parameter<bool>("sim_drive_gazebo_model_pose", true);
    publish_fake_imu_ = declare_parameter<bool>("sim_publish_fake_imu", false);
    publish_fake_range_ = declare_parameter<bool>("sim_publish_fake_range", false);
    publish_fake_tof_ = declare_parameter<bool>("sim_publish_fake_tof", false);
    gazebo_pose_poll_period_s_ = declare_parameter<double>("sim_gazebo_pose_poll_period_s", 0.20);
    gazebo_cli_pose_poll_period_s_ =
      declare_parameter<double>("sim_gazebo_cli_pose_poll_period_s", 0.20);
    gazebo_pose_drive_period_s_ = declare_parameter<double>("sim_gazebo_pose_drive_period_s", 0.10);
    gazebo_model_name_ = declare_parameter<std::string>("sim_gazebo_model_name", "cyberdog");
    gazebo_world_name_ = declare_parameter<std::string>("sim_gazebo_world_name", "earth");
    gazebo_pose_topic_ = declare_parameter<std::string>(
      "sim_gazebo_pose_topic", "/gazebo/earth/pose/info");
    x_ = declare_parameter<double>("sim_initial_x", 1.55);
    y_ = declare_parameter<double>("sim_initial_y", 0.70);
    yaw_ = declare_parameter<double>("sim_initial_yaw", 1.57);
    z_ = declare_parameter<double>("sim_model_z", 0.24);
    range_m_ = declare_parameter<double>("sim_safe_range_m", 0.50);
    tof_range_m_ = declare_parameter<double>("sim_tof_range_m", 0.80);
    wheel_base_m_ = declare_parameter<double>("sim_wheel_base_m", 0.50);
    yellow_border_px_ = declare_parameter<int>("sim_yellow_border_px", 20);
    ball_radius_px_ = declare_parameter<int>("sim_ball_radius_px", 24);
    randomize_balls_ = declare_parameter<bool>("sim_randomize_balls", true);
    random_seed_ = declare_parameter<int>("sim_random_seed", 2026);
    auto grid_x = declare_parameter<std::vector<int64_t>>(
      "sim_grid_x_pixels", {110, 250, 390, 530});
    auto grid_y = declare_parameter<std::vector<int64_t>>(
      "sim_grid_y_pixels", {110, 200, 290, 380});
    auto orange_raw = declare_parameter<std::vector<int64_t>>(
      "sim_orange_ball_pixels", {530, 110, 390, 200, 250, 290, 110, 380});
    auto blue_raw = declare_parameter<std::vector<int64_t>>(
      "sim_blue_ball_pixels", {530, 290, 390, 380, 530, 380});
    camera_matrix_ = declare_parameter<std::vector<double>>(
      "camera_matrix", {260.0, 0.0, 320.0, 0.0, 260.0, 240.0, 0.0, 0.0, 1.0});
    distortion_coeffs_ = declare_parameter<std::vector<double>>(
      "distortion_coeffs", {0.01, -0.01, 0.001, 0.0});
    horizontal_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 110.0);
    target_diameter_m_ = declare_parameter<double>("target_diameter_m", 0.20);
    use_world_layout_ = declare_parameter<bool>("sim_use_world_layout", true);
    world_grid_x_m_ = declare_parameter<std::vector<double>>(
      "sim_grid_x_centers_m", {-0.4, 0.8, 2.0, 3.2});
    world_grid_y_m_ = declare_parameter<std::vector<double>>(
      "sim_grid_y_centers_m", {1.34, 2.18, 3.02, 3.86});
    fixed_blue_indices_ = declare_parameter<std::vector<int64_t>>(
      "sim_fixed_blue_indices", {2, 3, 7});

    if (use_world_layout_ && world_grid_x_m_.size() == 4 && world_grid_y_m_.size() == 4) {
      build_world_layout();
    } else if (randomize_balls_ && grid_x.size() == 4 && grid_y.size() == 4) {
      std::vector<size_t> cols_by_row{0, 1, 2, 3};
      std::mt19937 generator(static_cast<std::mt19937::result_type>(random_seed_));
      do {
        std::shuffle(cols_by_row.begin(), cols_by_row.end(), generator);
      } while (cols_by_row[2] == 3 || cols_by_row[3] == 2 || cols_by_row[3] == 3);
      orange_points_ = make_grid_points(grid_x, grid_y, cols_by_row);
    } else {
      orange_points_ = read_points(orange_raw);
    }
    blue_points_ = read_points(blue_raw);

    const auto qos_depth = declare_parameter<int>("sim_qos_depth", 10);
    const auto qos = rclcpp::QoS(qos_depth).reliable();
    const auto sensor_qos = rclcpp::SensorDataQoS();
    if (publish_images_) {
      image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
      rgb_image_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_image_topic_, qos);
      rgb_camera_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(rgb_camera_info_topic_, qos);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "simulated_sensors_node image publishing disabled; /image_left must come from Gazebo RGB camera");
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);
    body_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(body_state_topic_, qos);
    if (publish_fake_imu_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos);
    }
    if (publish_fake_range_) {
      obstacle_pub_ = create_publisher<sensor_msgs::msg::Range>(obstacle_topic_, sensor_qos);
      official_ultrasonic_pub_ =
        create_publisher<sensor_msgs::msg::Range>(official_ultrasonic_topic_, sensor_qos);
    }
    if (publish_fake_tof_) {
      tof_pub_ = create_publisher<sensor_msgs::msg::Range>(tof_topic_, sensor_qos);
      head_tof_pub_ = create_publisher<protocol::msg::HeadTofPayload>(official_tof_topic_, qos);
      rear_tof_pub_ = create_publisher<protocol::msg::RearTofPayload>(official_rear_tof_topic_, qos);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, qos, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
      });

    last_state_update_ = std::chrono::steady_clock::now();
    if (publish_images_) {
      image_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / camera_rate_hz_)),
        [this]() { publish_image(); });
    }
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / state_rate_hz_)),
      [this]() { publish_state(); });
    if (sync_gazebo_pose_ && !drive_gazebo_pose_) {
      start_gazebo_pose_subscription();
    } else if (!sync_gazebo_pose_ && !drive_gazebo_pose_) {
      RCLCPP_INFO(
        get_logger(),
        "sim_sync_gazebo_model_pose is disabled; integrating /cmd_vel into odom directly");
    }
    if (drive_gazebo_pose_) {
      gazebo_pose_driver_running_.store(true);
      gazebo_pose_driver_thread_ = std::thread([this]() {
        rclcpp::Rate rate(1.0 / std::max(0.01, gazebo_pose_drive_period_s_));
        while (rclcpp::ok() && gazebo_pose_driver_running_.load()) {
          drive_gazebo_pose();
          rate.sleep();
        }
      });
      RCLCPP_WARN(
        get_logger(),
        "sim_drive_gazebo_model_pose is enabled: /cmd_vel directly teleports Gazebo model %s "
        "for visible closed-loop validation.",
        gazebo_model_name_.c_str());
    } else if (sync_gazebo_pose_) {
      gazebo_pose_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(gazebo_cli_pose_poll_period_s_)),
      [this]() { poll_gazebo_pose(); });
    }
  }

  ~SimulatedSensorsNode() override
  {
    gazebo_pose_driver_running_.store(false);
    if (gazebo_pose_driver_thread_.joinable()) {
      gazebo_pose_driver_thread_.join();
    }
    gazebo_pose_sub_.reset();
    gazebo_node_.reset();
    if (gazebo_client_initialized_) {
      gazebo::client::shutdown();
    }
  }

private:
  void publish_image()
  {
    if (!publish_images_) {
      return;
    }
    cv::Mat image(image_height_, image_width_, CV_8UC3, cv::Scalar(40, 40, 40));
    cv::rectangle(
      image, cv::Rect(0, 0, image_width_, image_height_), cv::Scalar(0, 255, 255),
      std::max(1, yellow_border_px_));
    update_ball_visibility_from_command();
    if (use_world_layout_) {
      draw_world_layout(image);
    } else {
      for (const auto & point : orange_points_) {
        cv::circle(image, point, ball_radius_px_, cv::Scalar(0, 140, 255), -1);
      }
      for (const auto & point : blue_points_) {
        cv::circle(image, point, ball_radius_px_, cv::Scalar(255, 180, 0), -1);
      }
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(image.cols * image.elemSize());
    msg.data.assign(image.datastart, image.dataend);
    if (image_pub_) {
      image_pub_->publish(msg);
    }
    if (rgb_image_pub_) {
      rgb_image_pub_->publish(msg);
    }

    sensor_msgs::msg::CameraInfo info;
    info.header = msg.header;
    info.height = msg.height;
    info.width = msg.width;
    info.distortion_model = "plumb_bob";
    info.d = distortion_coeffs_;
    for (size_t i = 0; i < std::min<size_t>(camera_matrix_.size(), info.k.size()); ++i) {
      info.k[i] = camera_matrix_[i];
    }
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
    info.p[0] = info.k[0];
    info.p[2] = info.k[2];
    info.p[5] = info.k[4];
    info.p[6] = info.k[5];
    info.p[10] = 1.0;
    if (camera_info_pub_) {
      camera_info_pub_->publish(info);
    }
    if (rgb_camera_info_pub_) {
      rgb_camera_info_pub_->publish(info);
    }
  }

  void build_world_layout()
  {
    orange_world_balls_.clear();
    blue_world_balls_.clear();
    std::vector<size_t> cols_by_row{0, 1, 2, 3};
    std::mt19937 generator(static_cast<std::mt19937::result_type>(random_seed_));
    do {
      std::shuffle(cols_by_row.begin(), cols_by_row.end(), generator);
    } while (cols_by_row[2] == 3 || cols_by_row[3] == 2 || cols_by_row[3] == 3);

    for (size_t row = 0; row < world_grid_y_m_.size(); ++row) {
      for (size_t col = 0; col < world_grid_x_m_.size(); ++col) {
      WorldBall ball;
      const size_t index = row * world_grid_x_m_.size() + col;
        const bool fixed_blue =
          std::find(fixed_blue_indices_.begin(), fixed_blue_indices_.end(), static_cast<int64_t>(index)) !=
          fixed_blue_indices_.end();
      if (fixed_blue) {
          ball.id = static_cast<int>(index);
          ball.x_m = world_grid_x_m_[col];
          ball.y_m = world_grid_y_m_[row];
          ball.label = "blue_ball";
          blue_world_balls_.push_back(ball);
        }
      }
    }

    for (size_t row = 0; row < world_grid_y_m_.size(); ++row) {
      const size_t col = cols_by_row[row];
      if (col >= world_grid_x_m_.size()) {
        continue;
      }
      const size_t index = row * world_grid_x_m_.size() + col;
      if (std::find(fixed_blue_indices_.begin(), fixed_blue_indices_.end(), static_cast<int64_t>(index)) !=
        fixed_blue_indices_.end())
      {
        continue;
      }
      WorldBall ball;
      ball.id = static_cast<int>(index);
      ball.label = "orange_ball";
      ball.x_m = world_grid_x_m_[col];
      ball.y_m = world_grid_y_m_[row];
      orange_world_balls_.push_back(ball);
    }
  }

  cv::Point project_ball(const WorldBall & ball) const
  {
    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_x = x_;
      current_y = y_;
      current_yaw = yaw_;
    }
    const double dx = ball.x_m - current_x;
    const double dy = ball.y_m - current_y;
    const double range = std::hypot(dx, dy);
    const double relative_yaw = normalize_angle(std::atan2(dy, dx) - current_yaw);
    const double yaw_deg = relative_yaw * 180.0 / M_PI;
    const double image_center_x = static_cast<double>(image_width_) * 0.5;
    const double image_center_y = static_cast<double>(image_height_) * 0.5;
    const double px = image_center_x + (yaw_deg / (horizontal_fov_deg_ * 0.5)) * image_center_x;
    const double py = image_center_y + std::clamp(range - 1.2, -1.3, 1.3) * 55.0;
    return cv::Point(
      std::clamp(static_cast<int>(std::round(px)), 0, image_width_ - 1),
      std::clamp(static_cast<int>(std::round(py)), 0, image_height_ - 1));
  }

  int project_radius_px(const WorldBall & ball) const
  {
    double current_x = 0.0;
    double current_y = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_x = x_;
      current_y = y_;
    }
    const double distance = std::hypot(ball.x_m - current_x, ball.y_m - current_y);
    const double focal_px = camera_matrix_.empty() ? 260.0 : camera_matrix_.front();
    const double radius = (target_diameter_m_ * focal_px) / (2.0 * std::max(0.25, distance));
    return std::clamp(static_cast<int>(std::round(radius)), 7, 40);
  }

  void draw_world_layout(cv::Mat & image) const
  {
    for (const auto & ball : orange_world_balls_) {
      if (!ball_visible(ball)) {
        continue;
      }
      const auto pixel = project_ball(ball);
      cv::circle(image, pixel, project_radius_px(ball), cv::Scalar(0, 140, 255), -1);
    }
    for (const auto & ball : blue_world_balls_) {
      if (!ball_visible(ball)) {
        continue;
      }
      const auto pixel = project_ball(ball);
      cv::circle(image, pixel, project_radius_px(ball), cv::Scalar(255, 180, 0), -1);
    }
  }

  bool ball_visible(const WorldBall & ball) const
  {
    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_x = x_;
      current_y = y_;
      current_yaw = yaw_;
    }
    const double dx = ball.x_m - current_x;
    const double dy = ball.y_m - current_y;
    const double range = std::hypot(dx, dy);
    const double yaw_error = std::abs(normalize_angle(std::atan2(dy, dx) - current_yaw));
    return range > 0.20 && range < 4.5 && yaw_error <= horizontal_fov_deg_ * M_PI / 360.0;
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

  void publish_state()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(current_time - last_state_update_).count();
    last_state_update_ = current_time;

    const double left_wheel_velocity =
      latest_cmd_.linear.x - latest_cmd_.angular.z * wheel_base_m_ * 0.5;
    const double right_wheel_velocity =
      latest_cmd_.linear.x + latest_cmd_.angular.z * wheel_base_m_ * 0.5;
    const double linear_velocity = (left_wheel_velocity + right_wheel_velocity) * 0.5;
    const double angular_velocity = (right_wheel_velocity - left_wheel_velocity) / wheel_base_m_;

    if (!sync_gazebo_pose_ || drive_gazebo_pose_) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      yaw_ += angular_velocity * dt;
      yaw_ = normalize_angle(yaw_);
      x_ += linear_velocity * std::cos(yaw_) * dt;
      y_ += linear_velocity * std::sin(yaw_) * dt;
    }
    if (suppress_ball_updates_ > 0) {
      --suppress_ball_updates_;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = z_;
      odom.pose.pose.orientation = yaw_to_quaternion(yaw_);
    }
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;
    odom_pub_->publish(odom);
    body_state_pub_->publish(odom);
    publish_tf(odom);

    if (imu_pub_) {
      sensor_msgs::msg::Imu imu;
      imu.header = odom.header;
      imu.header.frame_id = base_frame_id_;
      imu.orientation = odom.pose.pose.orientation;
      imu.angular_velocity.z = angular_velocity;
      imu.linear_acceleration.z = 0.0;
      imu_pub_->publish(imu);
    }

    if (obstacle_pub_ && official_ultrasonic_pub_) {
      const auto ultrasonic_range = make_range(odom.header.stamp, obstacle_topic_, range_m_);
      obstacle_pub_->publish(ultrasonic_range);
      auto official_ultrasonic = ultrasonic_range;
      official_ultrasonic.header.frame_id = official_ultrasonic_topic_;
      official_ultrasonic_pub_->publish(official_ultrasonic);
    }
    if (tof_pub_) {
      tof_pub_->publish(make_range(odom.header.stamp, tof_topic_, tof_range_m_));
    }
    if (head_tof_pub_ && rear_tof_pub_) {
      publish_official_tof(odom.header.stamp);
    }
  }

  void update_ball_visibility_from_command()
  {
    if (latest_cmd_.linear.x > 0.25 && std::abs(latest_cmd_.angular.z) < 0.12) {
      suppress_ball_updates_ = std::max(suppress_ball_updates_, 5);
    }
    if (suppress_ball_updates_ == 0) {
      return;
    }
    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_x = x_;
      current_y = y_;
      current_yaw = yaw_;
    }
    const double strike_front_x = current_x + std::cos(current_yaw) * 0.22;
    const double strike_front_y = current_y + std::sin(current_yaw) * 0.22;
    orange_world_balls_.erase(
      std::remove_if(
        orange_world_balls_.begin(), orange_world_balls_.end(),
        [&](const WorldBall & ball) {
          return ball.label == "orange_ball" &&
                 std::hypot(ball.x_m - strike_front_x, ball.y_m - strike_front_y) < 0.35;
        }),
      orange_world_balls_.end());
  }

  void poll_gazebo_pose()
  {
    if (!sync_gazebo_pose_ || drive_gazebo_pose_) {
      return;
    }
    if (gazebo_pose_valid_) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_gazebo_pose_time_).count();
      if (elapsed <= 1.0) {
        return;
      }
    }
    if (!gazebo_pose_valid_ && poll_gazebo_pose_from_cli()) {
      return;
    }
    if (!gazebo_pose_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "waiting for Gazebo pose topic %s model=%s; keeping last odometry",
        gazebo_pose_topic_.c_str(), gazebo_model_name_.c_str());
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_gazebo_pose_time_).count();
    if (elapsed > 1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Gazebo pose topic %s has not updated for %.2fs; keeping last pose",
        gazebo_pose_topic_.c_str(), elapsed);
    }
  }

  bool poll_gazebo_pose_from_cli()
  {
    const std::string command = "gz model -m " + gazebo_model_name_ + " -i 2>/dev/null";
    FILE * pipe = popen(command.c_str(), "r");
    if (!pipe) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to run Gazebo pose command: %s", command.c_str());
      return false;
    }

    std::string output;
    char buffer[512];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      output += buffer;
      if (output.size() > 65536) {
        break;
      }
    }
    const int rc = pclose(pipe);
    if (output.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Gazebo model pose command failed for model=%s rc=%d", gazebo_model_name_.c_str(), rc);
      return false;
    }

    const auto find_number_after = [&](const std::string & marker, size_t & cursor, double & value) {
        const size_t marker_pos = output.find(marker, cursor);
        if (marker_pos == std::string::npos) {
          return false;
        }
        const size_t value_pos = output.find_first_of("-0123456789.", marker_pos + marker.size());
        if (value_pos == std::string::npos) {
          return false;
        }
        char * end_ptr = nullptr;
        value = std::strtod(output.c_str() + value_pos, &end_ptr);
        if (end_ptr == output.c_str() + value_pos) {
          return false;
        }
        cursor = static_cast<size_t>(end_ptr - output.c_str());
        return true;
      };

    size_t cursor = output.find("pose");
    if (cursor == std::string::npos) {
      cursor = 0;
    }
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!find_number_after("x:", cursor, px) ||
      !find_number_after("y:", cursor, py) ||
      !find_number_after("z:", cursor, pz) ||
      !find_number_after("x:", cursor, qx) ||
      !find_number_after("y:", cursor, qy) ||
      !find_number_after("z:", cursor, qz) ||
      !find_number_after("w:", cursor, qw))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to parse Gazebo model pose for model=%s", gazebo_model_name_.c_str());
      return false;
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double pose_yaw = std::atan2(siny_cosp, cosy_cosp);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      x_ = px;
      y_ = py;
      z_ = pz;
      yaw_ = pose_yaw;
    }
    last_gazebo_pose_time_ = std::chrono::steady_clock::now();
    gazebo_pose_valid_ = true;
    return true;
  }

  void start_gazebo_pose_subscription()
  {
    if (gazebo_client_initialized_) {
      return;
    }
    if (!gazebo::client::setup()) {
      RCLCPP_WARN(get_logger(), "failed to initialize Gazebo client transport");
      return;
    }
    gazebo_client_initialized_ = true;
    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(gazebo_world_name_);
    gazebo_pose_sub_ =
      gazebo_node_->Subscribe<gazebo::msgs::PosesStamped>(
        gazebo_pose_topic_, &SimulatedSensorsNode::on_gazebo_pose, this);
    RCLCPP_INFO(
      get_logger(), "subscribed Gazebo true pose topic %s for model %s",
      gazebo_pose_topic_.c_str(), gazebo_model_name_.c_str());
  }

  void on_gazebo_pose(const boost::shared_ptr<const gazebo::msgs::PosesStamped> & msg)
  {
    if (!msg) {
      return;
    }
    for (int i = 0; i < msg->pose_size(); ++i) {
      const auto & pose = msg->pose(i);
      if (pose.name() != gazebo_model_name_) {
        continue;
      }
      const auto & position = pose.position();
      const auto & orientation = pose.orientation();
      const double siny_cosp =
        2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y());
      const double cosy_cosp =
        1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z());
      const double pose_yaw = std::atan2(siny_cosp, cosy_cosp);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_ = position.x();
        y_ = position.y();
        z_ = position.z();
        yaw_ = pose_yaw;
      }
      last_gazebo_pose_time_ = std::chrono::steady_clock::now();
      gazebo_pose_valid_ = true;
      return;
    }
    if (!gazebo_pose_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Gazebo pose topic received but model '%s' was not found in current pose batch",
        gazebo_model_name_.c_str());
    }
  }

  void drive_gazebo_pose()
  {
    if (!drive_gazebo_pose_) {
      return;
    }
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;
    double current_yaw = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_x = x_;
      current_y = y_;
      current_z = z_;
      current_yaw = yaw_;
    }
    const std::string command =
      "gz model -m " + gazebo_model_name_ +
      " -x " + std::to_string(current_x) +
      " -y " + std::to_string(current_y) +
      " -z " + std::to_string(current_z) +
      " -R 0.0 -P 0.0 -Y " + std::to_string(current_yaw) +
      " >/dev/null 2>&1";
    const int ret = std::system(command.c_str());
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to drive Gazebo model pose with gz model command");
      return;
    }
    gazebo_pose_valid_ = true;
  }

  void publish_official_tof(const builtin_interfaces::msg::Time & stamp)
  {
    protocol::msg::SingleTofPayload left_head;
    left_head.header.stamp = stamp;
    left_head.header.frame_id = "left_head_tof";
    left_head.data_available = true;
    left_head.tof_position = protocol::msg::SingleTofPayload::LEFT_HEAD;
    left_head.data = std::vector<float>(64, static_cast<float>(tof_range_m_));
    left_head.intensity = std::vector<float>(64, 1.0F);

    protocol::msg::SingleTofPayload right_head = left_head;
    right_head.header.frame_id = "right_head_tof";
    right_head.tof_position = protocol::msg::SingleTofPayload::RIGHT_HEAD;
    protocol::msg::HeadTofPayload head;
    head.left_head = left_head;
    head.right_head = right_head;
    head_tof_pub_->publish(head);

    protocol::msg::SingleTofPayload left_rear = left_head;
    left_rear.header.frame_id = "left_rear_tof";
    left_rear.tof_position = protocol::msg::SingleTofPayload::LEFT_REAR;
    protocol::msg::SingleTofPayload right_rear = left_head;
    right_rear.header.frame_id = "right_rear_tof";
    right_rear.tof_position = protocol::msg::SingleTofPayload::RIGHT_REAR;
    protocol::msg::RearTofPayload rear;
    rear.left_rear = left_rear;
    rear.right_rear = right_rear;
    rear_tof_pub_->publish(rear);
  }

  void publish_tf(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  sensor_msgs::msg::Range make_range(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame,
    double range) const
  {
    sensor_msgs::msg::Range msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.5;
    msg.min_range = 0.05;
    msg.max_range = 2.0;
    msg.range = static_cast<float>(range);
    return msg;
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string rgb_image_topic_;
  std::string rgb_camera_info_topic_;
  std::string odom_topic_;
  std::string body_state_topic_;
  std::string official_ultrasonic_topic_;
  std::string official_tof_topic_;
  std::string official_rear_tof_topic_;
  std::string imu_topic_;
  std::string obstacle_topic_;
  std::string tof_topic_;
  std::string cmd_vel_topic_;
  std::string frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  int image_width_ {640};
  int image_height_ {480};
  int yellow_border_px_ {20};
  int ball_radius_px_ {24};
  double camera_rate_hz_ {10.0};
  double state_rate_hz_ {30.0};
  double gazebo_pose_poll_period_s_ {0.20};
  double gazebo_cli_pose_poll_period_s_ {0.20};
  double gazebo_pose_drive_period_s_ {0.10};
  double range_m_ {0.50};
  double tof_range_m_ {0.80};
  double wheel_base_m_ {0.50};
  double horizontal_fov_deg_ {110.0};
  double target_diameter_m_ {0.20};
  bool sync_gazebo_pose_ {true};
  bool drive_gazebo_pose_ {true};
  bool publish_fake_imu_ {false};
  bool publish_fake_range_ {false};
  bool publish_fake_tof_ {false};
  bool publish_images_ {false};
  bool use_world_layout_ {true};
  std::string gazebo_model_name_ {"cyberdog"};
  std::string gazebo_world_name_ {"earth"};
  std::string gazebo_pose_topic_ {"/gazebo/earth/pose/info"};
  double x_ {1.55};
  double y_ {0.70};
  double z_ {0.24};
  double yaw_ {1.57};
  bool randomize_balls_ {true};
  int random_seed_ {2026};
  bool gazebo_pose_valid_ {false};
  bool gazebo_client_initialized_ {false};
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coeffs_;
  std::vector<cv::Point> orange_points_;
  std::vector<cv::Point> blue_points_;
  std::vector<double> world_grid_x_m_;
  std::vector<double> world_grid_y_m_;
  std::vector<int64_t> fixed_blue_indices_;
  std::vector<WorldBall> orange_world_balls_;
  std::vector<WorldBall> blue_world_balls_;
  mutable std::mutex state_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  int suppress_ball_updates_ {0};
  std::chrono::steady_clock::time_point last_state_update_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camera_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr official_ultrasonic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pub_;
  rclcpp::Publisher<protocol::msg::HeadTofPayload>::SharedPtr head_tof_pub_;
  rclcpp::Publisher<protocol::msg::RearTofPayload>::SharedPtr rear_tof_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr gazebo_pose_timer_;
  std::thread gazebo_pose_driver_thread_;
  std::atomic_bool gazebo_pose_driver_running_ {false};
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr gazebo_pose_sub_;
  std::chrono::steady_clock::time_point last_gazebo_pose_time_;
};

}  // namespace wild_glint_hunt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wild_glint_hunt::SimulatedSensorsNode>());
  rclcpp::shutdown();
  return 0;
}
