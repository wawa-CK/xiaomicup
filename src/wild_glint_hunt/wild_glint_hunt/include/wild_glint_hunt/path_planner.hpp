#pragma once

#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "wild_glint_hunt/msg/vision_ball.hpp"

namespace wild_glint_hunt
{

struct PlannedTarget
{
  wild_glint_hunt::msg::VisionBall ball;
  double score {0.0};
};

class PathPlanner
{
public:
  PathPlanner(
    double field_width_m,
    double field_height_m,
    double boundary_margin_m,
    double obstacle_margin_m,
    double minimum_target_distance_m);

  std::optional<PlannedTarget> choose_target(
    const std::vector<wild_glint_hunt::msg::VisionBall> & detections,
    const std::vector<geometry_msgs::msg::Point> & blocked_points,
    const nav_msgs::msg::Odometry & odom,
    const std::vector<std::string> & completed_ids) const;

  bool is_boundary_safe(double x_m, double y_m) const;
  bool is_path_safe(
    double target_distance_m,
    double target_yaw_deg,
    const std::vector<geometry_msgs::msg::Point> & blocked_points,
    const nav_msgs::msg::Odometry & odom) const;

private:
  double distance_to_point(double x_m, double y_m, const geometry_msgs::msg::Point & point) const;
  bool contains_id(const std::vector<std::string> & completed_ids, const std::string & id) const;

  double field_width_m_;
  double field_height_m_;
  double boundary_margin_m_;
  double obstacle_margin_m_;
  double minimum_target_distance_m_;
};

}  // namespace wild_glint_hunt
