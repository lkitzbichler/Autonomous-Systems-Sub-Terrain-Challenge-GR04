#pragma once

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <Eigen/Dense>
#include <tf2/utils.h>

#ifndef PI
#define PI M_PI
#endif

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

  // ROS callbacks
  void onDesiredState(
    const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr des_state_msg);
  void onCurrentState(
    const nav_msgs::msg::Odometry::SharedPtr cur_state_msg);

  // Main control loop
  void controlLoop();

  // Utility functions
  static Eigen::Vector3d Vee(const Eigen::Matrix3d & in);
  static double signed_sqrt(double val);

private:
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 | ROS interface
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr desired_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Controller parameters
  double kx, kv, kr, komega;  // controller gains

  // Physical constants
  double m;    // mass of the UAV
  double g;    // gravity acceleration
  double d;    // distance from the center of propellers to the c.o.m.
  double cf;   // propeller lift coefficient
  double cd;   // propeller drag coefficient

  Eigen::Matrix3d J;        // inertia matrix
  Eigen::Vector3d e3;       // [0, 0, 1]
  Eigen::MatrixXd F2W;      // wrench-to-rotor-speeds map

  // Current state
  Eigen::Vector3d x;        // position of c.o.m. in world frame
  Eigen::Vector3d v;        // velocity of c.o.m. in world frame
  Eigen::Matrix3d R;        // orientation
  Eigen::Vector3d omega;    // angular velocity in body frame

  // Desired state
  Eigen::Vector3d xd;       // desired position
  Eigen::Vector3d vd;       // desired velocity
  Eigen::Vector3d ad;       // desired acceleration
  double yawd;              // desired yaw angle

  double hz;                // control loop frequency [Hz]
  bool received_desired;    // flag to check if desired state has been received
};
