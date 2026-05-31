#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include "wild_glint_hunt/msg/vision_ball_array.hpp"
#include "wild_glint_hunt/vision_processor.hpp"

namespace wild_glint_hunt
{

static cv::Mat read_matrix(const std::vector<double> & values, int rows, int cols)
{
  cv::Mat mat(rows, cols, CV_64F);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      mat.at<double>(r, c) = values.at(static_cast<size_t>(r * cols + c));
    }
  }
  return mat;
}

VisionProcessor::VisionProcessor(
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coeffs,
  const cv::Size & image_size,
  const ColorThreshold & orange_threshold,
  const ColorThreshold & blue_threshold,
    const ColorThreshold & yellow_threshold,
    double camera_height_m,
    double target_diameter_m,
    double horizontal_fov_deg,
    double no_detection_distance_m,
    double boundary_meters_per_pixel,
    double min_boundary_contour_area_px,
    double min_ball_area_px,
    double min_ball_radius_px,
    double orange_value_boost,
    double orange_saturation_boost,
    int orange_morph_close_iterations,
    bool enable_fisheye_undistortion)
: camera_matrix_(camera_matrix), distortion_coeffs_(distortion_coeffs), image_size_(image_size),
  orange_threshold_(orange_threshold), blue_threshold_(blue_threshold), yellow_threshold_(yellow_threshold),
  camera_height_m_(camera_height_m), target_diameter_m_(target_diameter_m),
  horizontal_fov_deg_(horizontal_fov_deg), no_detection_distance_m_(no_detection_distance_m),
  boundary_meters_per_pixel_(boundary_meters_per_pixel),
  min_boundary_contour_area_px_(min_boundary_contour_area_px), min_ball_area_px_(min_ball_area_px),
  min_ball_radius_px_(min_ball_radius_px), orange_value_boost_(orange_value_boost),
  orange_saturation_boost_(orange_saturation_boost),
  orange_morph_close_iterations_(orange_morph_close_iterations),
  enable_fisheye_undistortion_(enable_fisheye_undistortion)
{
  if (enable_fisheye_undistortion_) {
    cv::fisheye::initUndistortRectifyMap(
      camera_matrix_, distortion_coeffs_, cv::Mat::eye(3, 3, CV_64F), camera_matrix_,
      image_size_, CV_16SC2, map1_, map2_);
  }
}

cv::Scalar VisionProcessor::threshold_min(const ColorThreshold & threshold) const
{
  return cv::Scalar(
    static_cast<double>(threshold.h_min),
    static_cast<double>(threshold.s_min),
    static_cast<double>(threshold.v_min));
}

cv::Scalar VisionProcessor::threshold_max(const ColorThreshold & threshold) const
{
  return cv::Scalar(
    static_cast<double>(threshold.h_max),
    static_cast<double>(threshold.s_max),
    static_cast<double>(threshold.v_max));
}

double VisionProcessor::estimate_distance_m(double radius_px) const
{
  if (radius_px <= 1.0) return no_detection_distance_m_;
  const double focal_px = camera_matrix_.at<double>(0, 0);
  return (target_diameter_m_ * focal_px) / (2.0 * radius_px);
}

double VisionProcessor::estimate_yaw_deg(double center_x_px, int image_width) const
{
  const double half_width = static_cast<double>(image_width) * 0.5;
  const double normalized = (center_x_px - half_width) / half_width;
  return normalized * (horizontal_fov_deg_ * 0.5);
}

double VisionProcessor::detect_boundary_distance_m(const cv::Mat & hsv, cv::Mat & debug) const
{
  cv::Mat mask;
  cv::inRange(hsv, threshold_min(yellow_threshold_), threshold_max(yellow_threshold_), mask);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const cv::Point2d image_center(
    static_cast<double>(hsv.cols) * 0.5, static_cast<double>(hsv.rows) * 0.5);
  double min_distance = no_detection_distance_m_;
  for (const auto & contour : contours) {
    const auto rect = cv::boundingRect(contour);
    if (rect.area() < min_boundary_contour_area_px_) continue;
    cv::rectangle(debug, rect, cv::Scalar(0, 255, 255), 2);
  }
  std::vector<cv::Point> yellow_pixels;
  cv::findNonZero(mask, yellow_pixels);
  for (const auto & pixel : yellow_pixels) {
    const double pixel_distance =
      std::hypot(static_cast<double>(pixel.x) - image_center.x, static_cast<double>(pixel.y) - image_center.y);
    min_distance = std::min(min_distance, pixel_distance * boundary_meters_per_pixel_);
  }
  return min_distance;
}

std::vector<wild_glint_hunt::msg::VisionBall> VisionProcessor::detect_balls(
  const cv::Mat & bgr,
  const cv::Mat & hsv,
  const ColorThreshold & threshold,
  const std::string & label,
  const cv::Scalar & draw_color,
  cv::Mat & debug)
{
  cv::Mat mask;
  cv::inRange(hsv, threshold_min(threshold), threshold_max(threshold), mask);
  if (label == "orange_ball" && orange_morph_close_iterations_ > 0) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), orange_morph_close_iterations_);
  }
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<wild_glint_hunt::msg::VisionBall> results;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < min_ball_area_px_) continue;
    cv::Point2f center;
    float radius = 0.0F;
    cv::minEnclosingCircle(contour, center, radius);
    if (radius < min_ball_radius_px_) continue;
    wild_glint_hunt::msg::VisionBall ball;
    ball.label = label;
    ball.pixel_x = static_cast<int>(center.x);
    ball.pixel_y = static_cast<int>(center.y);
    ball.radius_px = radius;
    ball.distance_m = static_cast<float>(estimate_distance_m(radius));
    ball.yaw_deg = static_cast<float>(estimate_yaw_deg(center.x, bgr.cols));
    ball.confidence = static_cast<float>(std::min(1.0, area / (CV_PI * radius * radius)));
    ball.safe_to_approach = true;
    ball.id = label;
    cv::circle(debug, center, static_cast<int>(radius), draw_color, 2);
    cv::putText(debug, label, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 1);
    results.push_back(ball);
  }
  std::sort(results.begin(), results.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.pixel_y == rhs.pixel_y) {
      return lhs.pixel_x < rhs.pixel_x;
    }
    return lhs.pixel_y < rhs.pixel_y;
  });
  for (size_t i = 0; i < results.size(); ++i) {
    results[i].id = label + "_" + std::to_string(i + 1);
  }
  return results;
}

VisionFrameResult VisionProcessor::process(const cv::Mat & frame_bgr)
{
  VisionFrameResult result;
  if (enable_fisheye_undistortion_) {
    cv::remap(frame_bgr, result.undistorted, map1_, map2_, cv::INTER_LINEAR);
  } else {
    result.undistorted = frame_bgr.clone();
  }
  result.debug = result.undistorted.clone();
  cv::Mat hsv;
  cv::cvtColor(result.undistorted, hsv, cv::COLOR_BGR2HSV);
  if (orange_value_boost_ > 1.0 || orange_saturation_boost_ > 1.0) {
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    if (orange_saturation_boost_ > 1.0) {
      channels[1].convertTo(channels[1], -1, orange_saturation_boost_, 0.0);
    }
    if (orange_value_boost_ > 1.0) {
      channels[2].convertTo(channels[2], -1, orange_value_boost_, 0.0);
    }
    cv::merge(channels, hsv);
  }
  result.orange_balls =
    detect_balls(result.undistorted, hsv, orange_threshold_, "orange_ball", {0, 140, 255}, result.debug);
  result.blue_balls =
    detect_balls(result.undistorted, hsv, blue_threshold_, "blue_ball", {255, 180, 0}, result.debug);
  result.balls = result.orange_balls;
  result.balls.insert(result.balls.end(), result.blue_balls.begin(), result.blue_balls.end());
  result.boundary_distance_m = detect_boundary_distance_m(hsv, result.debug);
  return result;
}

class VisionNode : public rclcpp::Node
{
public:
  VisionNode() : Node("vision_node")
  {
    use_rgb_camera_ = declare_parameter<bool>("use_rgb_camera", true);
    enable_fisheye_undistortion_ =
      declare_parameter<bool>("enable_fisheye_undistortion", false);
    const auto default_image_topic = "/image_left";
    image_topic_ = declare_parameter<std::string>("image_topic", default_image_topic);
    output_topic_ = declare_parameter<std::string>("vision_output_topic", "/vision/ball_array");
    undistorted_topic_ = declare_parameter<std::string>(
      "undistorted_image_topic", "vision/undistorted_image");
    qos_depth_ = declare_parameter<int>("vision_qos_depth", 10);
    camera_calibration_is_placeholder_ =
      declare_parameter<bool>("camera_calibration_is_placeholder", true);
    camera_height_m_ = declare_parameter<double>("camera_height_m", 0.28);
    target_diameter_m_ = declare_parameter<double>("target_diameter_m", 0.20);
    horizontal_fov_deg_ = declare_parameter<double>("horizontal_fov_deg", 70.0);
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    blue_danger_distance_m_ = declare_parameter<double>("blue_danger_distance_m", 0.20);
    blue_danger_yaw_abs_deg_ = declare_parameter<double>("blue_danger_yaw_abs_deg", 18.0);
    enable_blue_danger_warning_ =
      declare_parameter<bool>("enable_blue_danger_warning", false);
    boundary_alert_distance_m_ = declare_parameter<double>("boundary_alert_distance_m", 0.15);
    enable_boundary_danger_warning_ =
      declare_parameter<bool>("enable_boundary_danger_warning", false);
    no_detection_distance_m_ = declare_parameter<double>("no_detection_distance_m", 999.0);
    boundary_meters_per_pixel_ = declare_parameter<double>("boundary_meters_per_pixel", 0.004);
    min_boundary_contour_area_px_ = declare_parameter<double>("min_boundary_contour_area_px", 20.0);
    min_ball_area_px_ = declare_parameter<double>("min_ball_area_px", 100.0);
    min_ball_radius_px_ = declare_parameter<double>("min_ball_radius_px", 4.0);
    const auto orange_value_boost = declare_parameter<double>("orange_value_boost", 1.15);
    const auto orange_saturation_boost = declare_parameter<double>("orange_saturation_boost", 1.10);
    const auto orange_morph_close_iterations =
      declare_parameter<int>("orange_morph_close_iterations", 1);
    auto camera_matrix = read_matrix(
      declare_parameter<std::vector<double>>(
        "camera_matrix", {260.0, 0.0, 320.0, 0.0, 260.0, 240.0, 0.0, 0.0, 1.0}),
      3, 3);
    auto distortion = read_matrix(
      declare_parameter<std::vector<double>>("distortion_coeffs", {0.01, -0.01, 0.001, 0.0}),
      1, 4);
    if (camera_calibration_is_placeholder_) {
      RCLCPP_WARN(
        get_logger(),
        "Using placeholder camera calibration parameters for %s. Replace camera_matrix, "
        "distortion_coeffs and camera extrinsics before real deployment.",
        use_rgb_camera_ ? "RGB camera" : "fisheye camera");
    }
    RCLCPP_INFO(
      get_logger(), "vision input=%s use_rgb=%d fisheye_undistort=%d",
      image_topic_.c_str(), use_rgb_camera_, enable_fisheye_undistortion_);
    ColorThreshold orange{5, 80, 60, 25, 255, 255};
    ColorThreshold blue{90, 70, 80, 110, 255, 255};
    ColorThreshold yellow{20, 80, 120, 40, 255, 255};
    orange_threshold_.h_min = declare_parameter<int>("orange.h_min", orange.h_min);
    orange_threshold_.s_min = declare_parameter<int>("orange.s_min", orange.s_min);
    orange_threshold_.v_min = declare_parameter<int>("orange.v_min", orange.v_min);
    orange_threshold_.h_max = declare_parameter<int>("orange.h_max", orange.h_max);
    orange_threshold_.s_max = declare_parameter<int>("orange.s_max", orange.s_max);
    orange_threshold_.v_max = declare_parameter<int>("orange.v_max", orange.v_max);
    blue_threshold_.h_min = declare_parameter<int>("blue.h_min", blue.h_min);
    blue_threshold_.s_min = declare_parameter<int>("blue.s_min", blue.s_min);
    blue_threshold_.v_min = declare_parameter<int>("blue.v_min", blue.v_min);
    blue_threshold_.h_max = declare_parameter<int>("blue.h_max", blue.h_max);
    blue_threshold_.s_max = declare_parameter<int>("blue.s_max", blue.s_max);
    blue_threshold_.v_max = declare_parameter<int>("blue.v_max", blue.v_max);
    yellow_threshold_.h_min = declare_parameter<int>("yellow.h_min", yellow.h_min);
    yellow_threshold_.s_min = declare_parameter<int>("yellow.s_min", yellow.s_min);
    yellow_threshold_.v_min = declare_parameter<int>("yellow.v_min", yellow.v_min);
    yellow_threshold_.h_max = declare_parameter<int>("yellow.h_max", yellow.h_max);
    yellow_threshold_.s_max = declare_parameter<int>("yellow.s_max", yellow.s_max);
    yellow_threshold_.v_max = declare_parameter<int>("yellow.v_max", yellow.v_max);
    processor_ = std::make_unique<VisionProcessor>(
      camera_matrix, distortion, cv::Size(image_width_, image_height_), orange_threshold_,
      blue_threshold_, yellow_threshold_, camera_height_m_, target_diameter_m_, horizontal_fov_deg_,
      no_detection_distance_m_, boundary_meters_per_pixel_, min_boundary_contour_area_px_,
      min_ball_area_px_, min_ball_radius_px_, orange_value_boost, orange_saturation_boost,
      orange_morph_close_iterations, enable_fisheye_undistortion_);
    pub_ = create_publisher<wild_glint_hunt::msg::VisionBallArray>(output_topic_, qos_depth_);
    tracking_pub_ = create_publisher<wild_glint_hunt::msg::VisionBallArray>(
      declare_parameter<std::string>("vision_tracking_topic", "vision/tracked_balls"), qos_depth_);
    danger_pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("danger_warning_topic", "/vision/danger_warning"), qos_depth_);
    undistorted_pub_ = create_publisher<sensor_msgs::msg::Image>(undistorted_topic_, qos_depth_);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      const auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat frame_bgr;
      if (msg->encoding == "rgb8") {
        cv::cvtColor(cv_ptr->image, frame_bgr, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "bgr8") {
        frame_bgr = cv_ptr->image;
      } else {
        cv::cvtColor(cv_ptr->image, frame_bgr, cv::COLOR_GRAY2BGR);
      }
      auto result = processor_->process(frame_bgr);
      wild_glint_hunt::msg::VisionBallArray out;
      out.header = msg->header;
      out.boundary_distance_m = static_cast<float>(result.boundary_distance_m);
      out.boundary_alert =
        enable_boundary_danger_warning_ && result.boundary_distance_m < boundary_alert_distance_m_;
      out.balls = result.balls;
      out.orange_balls = result.orange_balls;
      out.blue_balls = result.blue_balls;
      pub_->publish(out);
      tracking_pub_->publish(out);
      std_msgs::msg::Bool danger_msg;
      danger_msg.data = out.boundary_alert;
      for (const auto & ball : out.blue_balls) {
        if (enable_blue_danger_warning_ &&
          ball.label == "blue_ball" &&
          std::abs(static_cast<double>(ball.yaw_deg)) <= blue_danger_yaw_abs_deg_ &&
          ball.distance_m < blue_danger_distance_m_)
        {
          danger_msg.data = true;
          break;
        }
      }
      danger_pub_->publish(danger_msg);
      auto undistorted_msg = cv_bridge::CvImage(msg->header, "bgr8", result.undistorted).toImageMsg();
      undistorted_pub_->publish(*undistorted_msg);
    });
  }

private:
  std::string image_topic_;
  std::string output_topic_;
  std::string undistorted_topic_;
  int qos_depth_;
  bool camera_calibration_is_placeholder_;
  bool use_rgb_camera_;
  bool enable_fisheye_undistortion_;
  bool enable_blue_danger_warning_ {false};
  bool enable_boundary_danger_warning_ {false};
  int image_width_;
  int image_height_;
  double camera_height_m_;
  double target_diameter_m_;
  double horizontal_fov_deg_;
  double blue_danger_distance_m_;
  double blue_danger_yaw_abs_deg_;
  double boundary_alert_distance_m_;
  double no_detection_distance_m_;
  double boundary_meters_per_pixel_;
  double min_boundary_contour_area_px_;
  double min_ball_area_px_;
  double min_ball_radius_px_;
  ColorThreshold orange_threshold_;
  ColorThreshold blue_threshold_;
  ColorThreshold yellow_threshold_;
  std::unique_ptr<VisionProcessor> processor_;
  rclcpp::Publisher<wild_glint_hunt::msg::VisionBallArray>::SharedPtr pub_;
  rclcpp::Publisher<wild_glint_hunt::msg::VisionBallArray>::SharedPtr tracking_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace wild_glint_hunt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wild_glint_hunt::VisionNode>());
  rclcpp::shutdown();
  return 0;
}
