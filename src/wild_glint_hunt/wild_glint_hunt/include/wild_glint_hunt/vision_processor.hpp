#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "wild_glint_hunt/msg/vision_ball.hpp"

namespace wild_glint_hunt
{

struct ColorThreshold
{
  int h_min {0};
  int s_min {0};
  int v_min {0};
  int h_max {179};
  int s_max {255};
  int v_max {255};
};

struct VisionFrameResult
{
  std::vector<wild_glint_hunt::msg::VisionBall> balls;
  std::vector<wild_glint_hunt::msg::VisionBall> orange_balls;
  std::vector<wild_glint_hunt::msg::VisionBall> blue_balls;
  double boundary_distance_m {999.0};
  cv::Mat undistorted;
  cv::Mat debug;
};

class VisionProcessor
{
public:
  VisionProcessor(
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
    bool enable_fisheye_undistortion);

  VisionFrameResult process(const cv::Mat & frame_bgr);

private:
  std::vector<wild_glint_hunt::msg::VisionBall> detect_balls(
    const cv::Mat & bgr,
    const cv::Mat & hsv,
    const ColorThreshold & threshold,
    const std::string & label,
    const cv::Scalar & draw_color,
    cv::Mat & debug);

  double estimate_distance_m(double radius_px) const;
  double estimate_yaw_deg(double center_x_px, int image_width) const;
  double detect_boundary_distance_m(const cv::Mat & hsv, cv::Mat & debug) const;
  cv::Scalar threshold_min(const ColorThreshold & threshold) const;
  cv::Scalar threshold_max(const ColorThreshold & threshold) const;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  cv::Mat map1_;
  cv::Mat map2_;
  cv::Size image_size_;
  ColorThreshold orange_threshold_;
  ColorThreshold blue_threshold_;
  ColorThreshold yellow_threshold_;
  double camera_height_m_;
  double target_diameter_m_;
  double horizontal_fov_deg_;
  double no_detection_distance_m_;
  double boundary_meters_per_pixel_;
  double min_boundary_contour_area_px_;
  double min_ball_area_px_;
  double min_ball_radius_px_;
  double orange_value_boost_ {1.0};
  double orange_saturation_boost_ {1.0};
  int orange_morph_close_iterations_ {0};
  bool enable_fisheye_undistortion_ {false};
};

}  // namespace wild_glint_hunt
