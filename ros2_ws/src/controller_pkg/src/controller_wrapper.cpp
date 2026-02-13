#include "controller_pkg/controller_node.hpp"
#include "protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <chrono>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}

Eigen::Vector3d ControllerNode::Vee(const Eigen::Matrix3d& in){
  Eigen::Vector3d out;
  out << in(2,1), in(0,2), in(1,0);
  return out;
}

double ControllerNode::signed_sqrt(double val){
  return val>0?sqrt(val):-sqrt(-val);
}

ControllerNode::ControllerNode()
: rclcpp::Node("controller_node"),
  e3(0.0, 0.0, 1.0),
  F2W(4,4),
  hz(1000.0)
{
  received_desired = false;

  // Declare parameters
  kx = this->declare_parameter<double>("kx", 1.0);
  kv = this->declare_parameter<double>("kv", 1.0);
  kr = this->declare_parameter<double>("kr", 1.0);
  komega = this->declare_parameter<double>("komega", 1.0);
  command_topic_ = this->declare_parameter<std::string>("command_topic", "statemachine/cmd");
  heartbeat_topic_ = this->declare_parameter<std::string>("heartbeat_topic", "heartbeat");
  heartbeat_period_sec_ = this->declare_parameter<double>("heartbeat_period_sec", 1.0);
  yaw_from_vel_min_xy_ = this->declare_parameter<double>("yaw_from_vel_min_xy", yaw_from_vel_min_xy_);

  // Initialization of ROS elements
  motor_pub_= this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
  desired_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    "command/trajectory", rclcpp::QoS(10),
    std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));
  current_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state_est", rclcpp::QoS(10),
    std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));
  command_sub_ = this->create_subscription<statemachine_pkg::msg::Command>(
    command_topic_, 10,
    std::bind(&ControllerNode::onCommand, this, std::placeholders::_1));
  auto period = std::chrono::duration<double>(1.0 / hz);
  timer_= this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControllerNode::controlLoop, this) );
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(heartbeat_period_sec_),
    std::bind(&ControllerNode::publishHeartbeat, this));

  // Get parameters
  kx = this->get_parameter("kx").as_double();
  kv = this->get_parameter("kv").as_double();             
  kr = this->get_parameter("kr").as_double();
  komega = this->get_parameter("komega").as_double();

  // Initialize constants
  m = 1.0;
  cd = 1e-5;
  cf = 1e-3;
  g = 9.81;
  d = 0.3;
  J << 1.0,0.0,0.0,
        0.0,1.0,0.0,
        0.0,0.0,1.0;

  RCLCPP_INFO(this->get_logger(),
              "Controller gains: kx=%.3f, kv=%.3f, kr=%.3f, komega=%.3f",
              kx, kv, kr, komega);

  RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
}

void ControllerNode::onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr des_state_msg){
  if (!des_state_msg || des_state_msg->points.empty()) {
    return;
  }

  const auto & point = des_state_msg->points[0];

  // desired position
  xd << point.transforms[0].translation.x,
        point.transforms[0].translation.y,
        point.transforms[0].translation.z;

  // desired velocity
  vd << point.velocities[0].linear.x,
        point.velocities[0].linear.y,
        point.velocities[0].linear.z;

  // desired acceleration
  ad << point.accelerations[0].linear.x,
        point.accelerations[0].linear.y,
        point.accelerations[0].linear.z;

  // desired yaw: align with desired velocity when moving
  const double vxy = std::hypot(vd.x(), vd.y());
  if (vxy > std::max(1e-3, yaw_from_vel_min_xy_)) {
    yawd = std::atan2(vd.y(), vd.x());
    yaw_initialized_ = true;
  } else {
    // Keep yaw stable while nearly stationary to avoid camera "look-away" flips.
    if (!yaw_initialized_) {
      Eigen::Quaterniond q(
        point.transforms[0].rotation.w,
        point.transforms[0].rotation.x,
        point.transforms[0].rotation.y,
        point.transforms[0].rotation.z);
      Eigen::Matrix3d R_des = q.toRotationMatrix();
      yawd = std::atan2(R_des(1, 0), R_des(0, 0));
      yaw_initialized_ = true;
    }
  }

  received_desired = true;
}

void ControllerNode::onCurrentState(const nav_msgs::msg::Odometry::SharedPtr cur_state_msg){
    //current position
    x << cur_state_msg->pose.pose.position.x, 
          cur_state_msg->pose.pose.position.y, 
          cur_state_msg->pose.pose.position.z;

    //current velocity
    v << cur_state_msg->twist.twist.linear.x, 
          cur_state_msg->twist.twist.linear.y, 
          cur_state_msg->twist.twist.linear.z;

    const auto &qmsg = cur_state_msg->pose.pose.orientation;
    Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    q.normalize();
    // rotation matrix from the body-fixed frame to the inertial frame
    R = q.toRotationMatrix();
    Eigen::Vector3d omega_world(cur_state_msg->twist.twist.angular.x, cur_state_msg->twist.twist.angular.y, cur_state_msg->twist.twist.angular.z);
    // angular velocity in the body-fixed frame
    omega = R.transpose() * omega_world;
}

void ControllerNode::onCommand(const statemachine_pkg::msg::Command::SharedPtr cmd_msg){
    // Step 1: Validate incoming command and filter by target name.
    if (!cmd_msg) {
      return;
    }
    if (cmd_msg->target != this->get_name()) {
      return;
    }

    // Step 2: Toggle controller output according to mission-level command.
    switch (cmd_msg->command) {
      case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::START):
        control_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "[cmd] START");
        break;
      case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::HOLD):
      case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::LAND):
      case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::ABORT):
        control_enabled_ = false;
        publishZeroMotors();
        RCLCPP_INFO(this->get_logger(), "[cmd] HOLD/LAND/ABORT");
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "[cmd] ignored id=%u", cmd_msg->command);
        break;
    }
}

void ControllerNode::publishHeartbeat(){
    statemachine_pkg::msg::Answer hb;
    hb.node_name = this->get_name();
    hb.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
    hb.info = control_enabled_ ? "RUNNING_ACTIVE" : "RUNNING_HOLD";
    const auto now = this->now();
    hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    heartbeat_pub_->publish(hb);
}

void ControllerNode::publishZeroMotors(){
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.assign(4, 0.0);
    motor_pub_->publish(cmd);
}

void ControllerNode::controlLoop(){
    if (!control_enabled_) {
      return;
    }
    if (!received_desired) {
      return;
    }
    Eigen::Vector3d ex, ev, er, eomega;
    //position and velocity errors
    ex = this->x - this->xd;
    ev = this->v - this->vd;

    //calulation of cordnates of body fixed frame to world frame
    Eigen::Vector3d a = -kx * ex - kv * ev + m * g * e3 + m * ad;
    Eigen::Vector3d b3d = a.normalized();
    Eigen::Vector3d yaw_vec(std::cos(this->yawd), std::sin(this->yawd), 0.0);
    Eigen::Vector3d b2d = b3d.cross(yaw_vec).normalized();
    Eigen::Vector3d b1d = b2d.cross(b3d).normalized();

    //rotation matrix body fixed to world frame
    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;
  
    //computation of orientation error (er) and the rotation-rate error (eomega)
    Eigen::Matrix3d eR_mat = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    er = Vee(eR_mat);
    eomega = omega;
//     eomega = -omega;

    //computation of the desired wrench (force + torques) to control the UAV
    double F = a.dot(R * e3);
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J * omega);

    //wrench to force and torques
    Eigen::Vector4d wrench;
    wrench << F, M(0), M(1), M(2);

    double d_hat = d / std::sqrt(2.0);
    Eigen::Matrix4d F2W_local;
    F2W_local << cf,         cf,         cf,         cf,
                 cf * d_hat, cf * d_hat, -cf * d_hat, -cf * d_hat,
                -cf * d_hat, cf * d_hat,  cf * d_hat, -cf * d_hat,
                 cd,        -cd,         cd,        -cd;

    Eigen::Vector4d omega_squared = F2W_local.inverse() * wrench;

    //publishing the motorspeeds
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      cmd.angular_velocities[i] = signed_sqrt(omega_squared(i));
    }
    motor_pub_->publish(cmd);
  }
