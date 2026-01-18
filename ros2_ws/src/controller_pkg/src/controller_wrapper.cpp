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
  kx = this->declare_parameter<double>("kx", 20.0);
  kv = this->declare_parameter<double>("kv", 10.0);
  kr = this->declare_parameter<double>("kr", 12.0);
  komega = this->declare_parameter<double>("komega", 1.5);

  // Initialization of ROS elements
  motor_pub_= this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);
  desired_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("desired_state", 10, std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));
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
  this->yawd = tf2::getYaw(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));
}