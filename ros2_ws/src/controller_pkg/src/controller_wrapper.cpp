#include "controller_pkg/controller_node.hpp"
#include <rclcpp/rclcpp.hpp>

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
  e3(0,0,1),
  F2W(4,4),
  hz(1000.0)
{
  // Declare parameters
  kx = this->declare_parameter<double>("kx", 1.0);
  kv = this->declare_parameter<double>("kv", 1.0);
  kr = this->declare_parameter<double>("kr", 1.0);
  komega = this->declare_parameter<double>("komega", 1.0);

  // Initialization of ROS elements
  motor_pub_= this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);
  desired_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("command/trajectory", 10, std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));
  current_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( "current_state", 10, std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));
  timer_= this->create_wall_timer( std::chrono::duration<double>(1.0 / hz), std::bind(&ControllerNode::controlLoop, this) );

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

  RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
}

void ControllerNode::onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr des_state_msg){
  // desired position
  xd << des_state_msg->points[0].transforms[0].translation.x, 
        des_state_msg->points[0].transforms[0].translation.y, 
        des_state_msg->points[0].transforms[0].translation.z;

  // desired velocity
  vd << des_state_msg->points[0].velocities[0].linear.x, 
        des_state_msg->points[0].velocities[0].linear.y, 
        des_state_msg->points[0].velocities[0].linear.z,

  // desired acceleration
  ad << des_state_msg->points[0].accelerations[0].linear.x, 
        des_state_msg->points[0].accelerations[0].linear.y, 
        des_state_msg->points[0].accelerations[0].linear.z;

  Eigen::Quaterniond q(
    des_state_msg->points[0].transforms[0].rotation.w,
    des_state_msg->points[0].transforms[0].rotation.x,
    des_state_msg->points[0].transforms[0].rotation.y,
    des_state_msg->points[0].transforms[0].rotation.z);
  q.normalize();
  
  // desired yaw
  this->yawd = PI + tf2::getYaw(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
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

void ControllerNode::controlLoop(){
    Eigen::Vector3d ex, ev, er, eomega;
    //position and velocity errors
    ex = this->x - this->xd;
    ev = this->v - this->vd;

    //calulation of cordnates of body fixed frame to world frame
    Eigen::Vector3d b3d = (-kx*ex - kv*ev + m*g*e3 + m*ad).normalized();
    Eigen::Vector3d b1c(std::cos(this->yawd), std::sin(this->yawd), 0.0);
    Eigen::Vector3d b2d = b3d.cross(b1c).normalized();
    Eigen::Vector3d b1d = b2d.cross(b3d).normalized();

    //rotation matrix body fixed to world frame
    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;
  
    //computation of orientation error (er) and the rotation-rate error (eomega)
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
    eomega = this->omega;

    //computation of the desired wrench (force + torques) to control the UAV
    double F = (-kx*ex - kv*ev + m*g*e3 + m*ad).dot(R.col(2));
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J * omega);

    //wrench to force and torques
    F2W << cf, cf, cf, cf,
          std::sqrt(2)/2 * d * cf,  std::sqrt(2)/2 * d * cf, -std::sqrt(2)/2 * d * cf, -std::sqrt(2)/2 * d * cf, 
          - std::sqrt(2)/2 * d * cf, std::sqrt(2)/2 * d * cf, std::sqrt(2)/2 * d * cf,  - std::sqrt(2)/2 * d * cf,
          cd,  -cd, cd,  -cd;

    //calculation of wrench of the propellers
    Eigen::Vector4d w = F2W.inverse() * Eigen::Vector4d (F, M(0), M(1), M(2));

    double w1 = signed_sqrt(w(0));
    double w2 = signed_sqrt(w(1));
    double w3 = signed_sqrt(w(2));
    double w4 = signed_sqrt(w(3));

    //publishing the motorspeeds
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);
    cmd.angular_velocities[0] = w1;
    cmd.angular_velocities[1] = w2;
    cmd.angular_velocities[2] = w3;
    cmd.angular_velocities[3] = w4;
    motor_pub_->publish(cmd);
  }
