#include "path_planning_pkg/path_planner_node.hpp"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_trajectory_generation/timing.h>

#include <cmath>

bool PathPlannerNode::buildTrajectoryFromPath(
    const std::vector<Eigen::Vector3d> &path,
    mav_trajectory_generation::Trajectory *trajectory) {
  // --- Trajectory generation ---
  if (!trajectory || path.size() < 2) {
    return false;
  }

  // Build a smooth polynomial trajectory (optionally with yaw).
  std::vector<Eigen::Vector3d> waypoints = path;
  if (waypoints.size() == 2) {
    // Avoid degenerate trajectories with only two vertices.
    const Eigen::Vector3d mid = 0.5 * (waypoints.front() + waypoints.back());
    waypoints.insert(waypoints.begin() + 1, mid);
  }

  const int dimension = use_yaw_ ? 4 : 3;
  mav_trajectory_generation::Vertex::Vector vertices;

  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  if (use_yaw_) {
    double fallback_yaw = last_yaw_;
    const double vel_xy =
        std::hypot(current_velocity_.x(), current_velocity_.y());
    if (vel_xy > yaw_speed_threshold_) {
      // Use velocity heading when moving fast enough.
      fallback_yaw =
          std::atan2(current_velocity_.y(), current_velocity_.x()) + yaw_offset_;
    }
    const std::vector<double> yaw_values =
        computeYawFromPath(waypoints, fallback_yaw);

    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    Eigen::Vector4d start_pos_yaw;
    start_pos_yaw << waypoints.front().x(), waypoints.front().y(),
        waypoints.front().z(), yaw_values.front();
    start.makeStartOrEnd(start_pos_yaw, derivative_to_optimize);
    Eigen::Vector4d start_vel_yaw;
    start_vel_yaw << current_velocity_.x(), current_velocity_.y(),
        current_velocity_.z(), 0.0;
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_yaw);
    vertices.push_back(start);

    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      mav_trajectory_generation::Vertex middle(dimension);
      Eigen::Vector4d pos_yaw;
      pos_yaw << waypoints[i].x(), waypoints[i].y(), waypoints[i].z(),
          yaw_values[i];
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                           pos_yaw);
      vertices.push_back(middle);
    }

    Eigen::Vector4d end_pos_yaw;
    end_pos_yaw << waypoints.back().x(), waypoints.back().y(),
        waypoints.back().z(), yaw_values.back();
    end.makeStartOrEnd(end_pos_yaw, derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      Eigen::Vector4d::Zero());
    vertices.push_back(end);
  } else {
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.makeStartOrEnd(waypoints.front(), derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);
    vertices.push_back(start);

    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      mav_trajectory_generation::Vertex middle(dimension);
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                           waypoints[i]);
      vertices.push_back(middle);
    }

    end.makeStartOrEnd(waypoints.back(), derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      Eigen::Vector3d::Zero());
    vertices.push_back(end);
  }

  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, max_v_, max_a_);
  if (segment_times.size() + 1 != vertices.size()) {
    RCLCPP_WARN(get_logger(), "Invalid segment times (%zu) for %zu vertices.",
                segment_times.size(), vertices.size());
    return false;
  }

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension,
                                                                    parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  opt.optimize();
  opt.getTrajectory(trajectory);

  return true;
}

std::vector<double> PathPlannerNode::computeYawFromPath(
    const std::vector<Eigen::Vector3d> &waypoints,
    double fallback_yaw) const {
  // --- Yaw profile ---
  // Derive a continuous yaw profile from segment directions.
  const size_t n = waypoints.size();
  std::vector<double> yaw_values(n, fallback_yaw);
  if (n < 2) {
    return yaw_values;
  }

  double last_yaw = fallback_yaw;
  for (size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Vector3d delta = waypoints[i + 1] - waypoints[i];
    const double dx = delta.x();
    const double dy = delta.y();
    const double norm_xy = std::hypot(dx, dy);
    if (norm_xy > 1e-3) {
      last_yaw = std::atan2(dy, dx) + yaw_offset_;
    }
    yaw_values[i] = last_yaw;
  }
  yaw_values.back() = last_yaw;
  return yaw_values;
}

bool PathPlannerNode::publishTrajectory(
    const mav_trajectory_generation::Trajectory &trajectory) {
  // --- Trajectory publish ---
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  if (!mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                      &msg)) {
    RCLCPP_WARN(get_logger(), "Failed to convert trajectory to 4D message.");
    return false;
  }
  msg.header.frame_id = world_frame_;
  msg.header.stamp = get_clock()->now();
  trajectory_pub_->publish(msg);
  return true;
}
