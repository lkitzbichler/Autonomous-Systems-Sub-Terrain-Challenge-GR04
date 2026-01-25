#ifndef BASIC_WAYPOINT_PKG__PLANNER_HPP_
#define BASIC_WAYPOINT_PKG__PLANNER_HPP_

#include <memory>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "mav_msgs/conversions.hpp"
#include "mav_planning_msgs/msg/polynomial_trajectory4_d.hpp"
#include <tf2_eigen/tf2_eigen.hpp>


#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "mav_trajectory_generation/ros_conversions.h"

class BasicPlanner
{
public:
  explicit BasicPlanner(const rclcpp::Node::SharedPtr & node);

  void uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  void setMaxSpeed(double max_v);

  bool planTrajectory(mav_trajectory_generation::Trajectory * trajectory);

  bool planTrajectory(
    const Eigen::VectorXd & start_pos,
    const Eigen::VectorXd & start_vel,
    double v_max, double a_max,
    mav_trajectory_generation::Trajectory * trajectory);

  bool publishTrajectory(const mav_trajectory_generation::Trajectory & trajectory);

  void drawMavTrajectory(
    const mav_trajectory_generation::Trajectory & trajectory,
    double distance, const std::string & frame_id,
    visualization_msgs::msg::MarkerArray * marker_array);

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;

  double max_v_;      // m/s
  double max_a_;      // m/s^2
  double max_ang_v_;
  double max_ang_a_;
};

#endif  // BASIC_WAYPOINT_PKG__PLANNER_HPP_
