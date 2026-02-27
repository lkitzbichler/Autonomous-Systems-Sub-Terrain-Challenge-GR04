#ifndef BASIC_WAYPOINT_PKG__PLANNER_HPP_
#define BASIC_WAYPOINT_PKG__PLANNER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

#include "mav_msgs/conversions.hpp"
#include "mav_planning_msgs/msg/polynomial_trajectory4_d.hpp"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/ros_conversions.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief Simple planner that generates polynomial trajectories for a UAV.
 *
 * The planner reads current state from an odometry topic and uses either
 * parameters or explicitly provided waypoints to construct a trajectory that
 * respects configured velocity/acceleration limits.
 */
class BasicPlanner {
   public:
    /**
     * @brief Construct a planner bound to a ROS2 node instance.
     *
     * @param node Shared pointer to the ROS2 node used for parameter access,
     *             publishers and subscriptions.
     */
    explicit BasicPlanner(const rclcpp::Node::SharedPtr& node);

    /**
     * @brief Odometry callback used to update the planner's internal state.
     *
     * @param odom Pointer to the incoming odometry message.
     */
    void uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    /**
     * @brief Configure the maximum translational speed for trajectory planning.
     *
     * @param max_speed_mps Maximum velocity in meters per second (m/s).
     */
    void setMaxSpeed(double max_speed_mps);

    /**
     * @brief Plan a trajectory using waypoints supplied as ROS parameters.
     *
     * The method internally reads the \"waypoints\" and \"stop_index\"
     * parameters from the node and forwards them to
     * planTrajectoryWithWaypoints.
     *
     * @param trajectory Pointer to output trajectory object.
     * @return true if a valid trajectory was generated, false otherwise.
     */
    bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);

    /**
     * @brief Plan trajectory using an explicit waypoint list.
     *
     * @param waypoint_flat_list Flat list [x1, y1, z1, x2, y2, z2, ...] of
     *        Cartesian waypoints in meters.
     * @param stop_waypoint_index Index of the waypoint where the vehicle must
     *        come to a complete stop (-1 disables the stop constraint).
     * @param trajectory Output trajectory object to populate.
     * @return true if planning succeeded and trajectory is valid.
     */
    bool planTrajectoryWithWaypoints(const std::vector<double>& waypoint_flat_list, int stop_waypoint_index,
                                     mav_trajectory_generation::Trajectory* trajectory);

    /**
     * @brief Alternate planning interface with explicit start state and limits.
     *
     * @param start_pos Initial position vector in meters.
     * @param start_vel Initial velocity vector in meters per second.
     * @param v_max Maximum allowed velocity (m/s).
     * @param a_max Maximum allowed acceleration (m/s^2).
     * @param trajectory Output trajectory object.
     * @return true if planning succeeded.
     * @note Currently a thin wrapper that defers to the parameter-based planner.
     */
    bool planTrajectory(const Eigen::VectorXd& start_pos, const Eigen::VectorXd& start_vel, double v_max,
                        double a_max, mav_trajectory_generation::Trajectory* trajectory);

    /**
     * @brief Publish a previously generated trajectory to RVIZ markers and
     *        the \"trajectory\" topic.
     *
     * @param trajectory Trajectory to be published.
     * @return true on success.
     */
    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    /**
     * @brief Utility to draw a trajectory as a set of markers.
     *
     * @param trajectory Trajectory to sample and draw.
     * @param marker_spacing_m Spacing between orientation markers in meters.
     * @param frame_id TF frame in which markers are expressed.
     * @param marker_array Output array that will be populated.
     */
    void drawMavTrajectory(const mav_trajectory_generation::Trajectory& trajectory, double marker_spacing_m,
                           const std::string& frame_id, visualization_msgs::msg::MarkerArray* marker_array);

   private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    bool has_current_pose_{false};

    double max_speed_mps_;             // m/s
    double max_accel_mps2_;            // m/s^2
    double max_angular_speed_radps_;   // rad/s
    double max_angular_accel_radps2_;  // rad/s^2
};

#endif  // BASIC_WAYPOINT_PKG__PLANNER_HPP_
