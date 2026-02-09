#include "basic_waypoint_pkg/planner.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <utility>
#include <vector>

BasicPlanner::BasicPlanner(const rclcpp::Node::SharedPtr & node)
: node_(node),
  current_pose_(Eigen::Affine3d::Identity()),
  current_velocity_(Eigen::Vector3d::Zero()),
  current_angular_velocity_(Eigen::Vector3d::Zero()),
  max_v_(0.2),
  max_a_(0.2),
  max_ang_v_(0.0),
  max_ang_a_(0.0)
{   
  // declare parameters
  node_->declare_parameter("max_v", max_v_);
  node_->declare_parameter("max_a", max_a_);
  node_->declare_parameter<int>("stop_index", -1);
  node_->declare_parameter("waypoints", std::vector<double>());

  // get parameters
  node_->get_parameter("max_v", max_v_);
  node_->get_parameter("max_a", max_a_);


  // Publishers
  pub_markers_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trajectory_markers", 10);

  pub_trajectory_ =
    node_->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "trajectory", 10);

  // Subscriber for Odometry
  sub_odom_ =
    node_->create_subscription<nav_msgs::msg::Odometry>(
      "current_state_est", 10,
      std::bind(&BasicPlanner::uavOdomCallback, this, std::placeholders::_1));
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // store current position in our planner
  tf2::fromMsg(odom->pose.pose, current_pose_);

  // store current velocity
  tf2::fromMsg(odom->twist.twist.linear, current_velocity_);

  // mark pose as valid
  has_current_pose_ = true;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v)
{
  max_v_ = max_v;
}

// Plans a trajectory from the current position to a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(
  mav_trajectory_generation::Trajectory * trajectory)
{
  // Step 0: Require a valid current pose
  if (!has_current_pose_) {
    RCLCPP_WARN(node_->get_logger(), "No valid odometry yet, cannot plan trajectory.");
    return false;
  }

  // Step 1: Load waypoints and stop index from parameters
  std::vector<double> waypoint_list;
  int stop_index = -1;
  node_->get_parameter("stop_index", stop_index);
  node_->get_parameter("waypoints", waypoint_list);

  // Step 2: Delegate to explicit waypoint planner
  return planTrajectoryWithWaypoints(waypoint_list, stop_index, trajectory);
}

// Plans a trajectory from the current position using an explicit waypoint list
bool BasicPlanner::planTrajectoryWithWaypoints(
  const std::vector<double> & waypoint_list,
  int stop_index,
  mav_trajectory_generation::Trajectory * trajectory)
{
  // Step 0: Require a valid current pose
  if (!has_current_pose_) {
    RCLCPP_WARN(node_->get_logger(), "No valid odometry yet, cannot plan trajectory.");
    return false;
  }

  // 3 Dimensional trajectory => through Cartesian space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimize up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
    mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // End   = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(
    current_pose_.translation(),
    derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY,
    current_velocity_);

  // add waypoint to list
  vertices.push_back(start);

  /******* Configure trajectory (intermediate waypoints) *******/
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  if (waypoint_list.size() < 3 || (waypoint_list.size() % 3) != 0) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid waypoints param (size=%zu)", waypoint_list.size());
    return false;
  }

  const size_t waypoint_count = waypoint_list.size() / 3;
  for (size_t i = 0; i + 1 < waypoint_count; ++i) {
    mav_trajectory_generation::Vertex middle(dimension);

    Eigen::Vector3d pos;
    pos << waypoint_list[i * 3],
      waypoint_list[i * 3 + 1],
      waypoint_list[i * 3 + 2];

    middle.addConstraint(
      mav_trajectory_generation::derivative_order::POSITION, pos);

    if (static_cast<int>(i) == stop_index) {
      middle.addConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY,
        Eigen::Vector3d::Zero());
      middle.addConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION,
        Eigen::Vector3d::Zero());
    }

    vertices.push_back(middle);
  }

  /******* Configure end point *******/
  Eigen::Vector3d goal_pos;
  goal_pos << waypoint_list[(waypoint_count - 1) * 3],
    waypoint_list[(waypoint_count - 1) * 3 + 1],
    waypoint_list[(waypoint_count - 1) * 3 + 2];

  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(
    goal_pos,
    derivative_to_optimize);

  // set end point's velocity to be constrained to desired velocity
  end.addConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY,
    Eigen::Vector3d::Zero());

  if (stop_index == static_cast<int>(waypoint_count - 1)) {
    end.addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION,
      Eigen::Vector3d::Zero());
  }

  // add waypoint to list
  vertices.push_back(end);

  // estimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
    dimension, parameters);
  opt.setupFromVertices(
    vertices, segment_times,
    derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(trajectory);

  return true;
}

// Overload using explicit start state and limits (currently just a stub, same as above)
bool BasicPlanner::planTrajectory(
  const Eigen::VectorXd & start_pos,
  const Eigen::VectorXd & start_vel,
  double v_max, double a_max,
  mav_trajectory_generation::Trajectory * trajectory)
{
  // You can either implement a different variant or simply reuse the other method.
  (void)start_pos;
  (void)start_vel;
  (void)v_max;
  (void)a_max;
  return planTrajectory(trajectory);
}

bool BasicPlanner::publishTrajectory(
  const mav_trajectory_generation::Trajectory & trajectory)
{
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::msg::MarkerArray markers;
  double distance = 0.2;  // distance between markers; 0.0 to disable
  std::string frame_id = "world";

  drawMavTrajectory(
    trajectory, distance, frame_id, &markers);
  pub_markers_->publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
    trajectory, &msg);
  msg.header.frame_id = "world";
  // optionally: msg.header.stamp = node_->now();
  pub_trajectory_->publish(msg);

  return true;
}

void BasicPlanner::drawMavTrajectory(
    const mav_trajectory_generation::Trajectory& trajectory,
    double distance, const std::string& frame_id,
    visualization_msgs::msg::MarkerArray* marker_array) {
    // sample trajectory
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    bool success = mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, 0.1, &trajectory_points);
    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Could not sample trajectory.");
        return;
    }

    // draw trajectory
    marker_array->markers.clear();

    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // orange
    std_msgs::msg::ColorRGBA line_strip_color;
    line_strip_color.r = 1.0;
    line_strip_color.g = 0.5;
    line_strip_color.b = 0.0;
    line_strip_color.a = 1.0;
    line_strip.color = line_strip_color;
    line_strip.scale.x = 0.01;
    line_strip.ns = "path";

    double accumulated_distance = 0.0;
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
    double scale = 0.3;
    double diameter = 0.3;
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const mav_msgs::EigenTrajectoryPoint& point = trajectory_points[i];

        accumulated_distance += (last_position - point.position_W).norm();
        if (accumulated_distance > distance) {
            accumulated_distance = 0.0;
            mav_msgs::EigenMavState mav_state;
            mav_msgs::EigenMavStateFromEigenTrajectoryPoint(point, &mav_state);
            mav_state.orientation_W_B = point.orientation_W_B;

            visualization_msgs::msg::MarkerArray axes_arrows;
            axes_arrows.markers.resize(3);

            // x axis
            visualization_msgs::msg::Marker arrow_marker = axes_arrows.markers[0];
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color;
            color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
            arrow_marker.color = color;  // x - red
            arrow_marker.points.resize(2);
            arrow_marker.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitX() * scale)
            );
            arrow_marker.scale.x = diameter * 0.1;
            arrow_marker.scale.y = diameter * 0.2;
            arrow_marker.scale.z = 0;

            // y axis
            visualization_msgs::msg::Marker arrow_marker_y = axes_arrows.markers[1];
            arrow_marker_y.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_y.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_y;
            color_y.r = 0.0; color_y.g = 1.0; color_y.b = 0.0; color_y.a = 1.0;
            arrow_marker_y.color = color_y;  // y - green
            arrow_marker_y.points.resize(2);
            arrow_marker_y.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_y.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitY() * scale)
            );
            arrow_marker_y.scale.x = diameter * 0.1;
            arrow_marker_y.scale.y = diameter * 0.2;
            arrow_marker_y.scale.z = 0;

            // z axis
            visualization_msgs::msg::Marker arrow_marker_z = axes_arrows.markers[2];
            arrow_marker_z.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker_z.action = visualization_msgs::msg::Marker::ADD;
            std_msgs::msg::ColorRGBA color_z;
            color_z.r = 0.0; color_z.g = 0.0; color_z.b = 1.0; color_z.a = 1.0;
            arrow_marker_z.color = color_z;  // z - blue
            arrow_marker_z.points.resize(2);
            arrow_marker_z.points[0] = tf2::toMsg(
                mav_state.position_W);
            arrow_marker_z.points[1] = tf2::toMsg(
                Eigen::Vector3d(mav_state.position_W +
                mav_state.orientation_W_B * Eigen::Vector3d::UnitZ() * scale)
            );
            arrow_marker_z.scale.x = diameter * 0.1;
            arrow_marker_z.scale.y = diameter * 0.2;
            arrow_marker_z.scale.z = 0;

            // append to marker array
            for (size_t j = 0; j < axes_arrows.markers.size(); ++j) {
                axes_arrows.markers[j].header.frame_id = frame_id;
                axes_arrows.markers[j].ns = "pose";
                marker_array->markers.push_back(axes_arrows.markers[j]);
            }
        }
        last_position = point.position_W;
        geometry_msgs::msg::Point last_position_msg;
        last_position_msg = tf2::toMsg(last_position);
        line_strip.points.push_back(last_position_msg);
    }
    marker_array->markers.push_back(line_strip);

    std_msgs::msg::Header header;
    header.frame_id = frame_id;
    header.stamp = node_->now();
    for (size_t i = 0; i < marker_array->markers.size(); ++i) {
        marker_array->markers[i].header = header;
        marker_array->markers[i].id = i;
        marker_array->markers[i].lifetime = rclcpp::Duration::from_seconds(0.0);
        marker_array->markers[i].action = visualization_msgs::msg::Marker::ADD;
    }
}
