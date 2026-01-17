// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point_cloud_with_pose.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PointCloudWithPose_cloud_in_sensor_frame
{
public:
  explicit Init_PointCloudWithPose_cloud_in_sensor_frame(::mav_planning_msgs::msg::PointCloudWithPose & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PointCloudWithPose cloud_in_sensor_frame(::mav_planning_msgs::msg::PointCloudWithPose::_cloud_in_sensor_frame_type arg)
  {
    msg_.cloud_in_sensor_frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PointCloudWithPose msg_;
};

class Init_PointCloudWithPose_sensor_pose
{
public:
  explicit Init_PointCloudWithPose_sensor_pose(::mav_planning_msgs::msg::PointCloudWithPose & msg)
  : msg_(msg)
  {}
  Init_PointCloudWithPose_cloud_in_sensor_frame sensor_pose(::mav_planning_msgs::msg::PointCloudWithPose::_sensor_pose_type arg)
  {
    msg_.sensor_pose = std::move(arg);
    return Init_PointCloudWithPose_cloud_in_sensor_frame(msg_);
  }

private:
  ::mav_planning_msgs::msg::PointCloudWithPose msg_;
};

class Init_PointCloudWithPose_header
{
public:
  Init_PointCloudWithPose_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PointCloudWithPose_sensor_pose header(::mav_planning_msgs::msg::PointCloudWithPose::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PointCloudWithPose_sensor_pose(msg_);
  }

private:
  ::mav_planning_msgs::msg::PointCloudWithPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PointCloudWithPose>()
{
  return mav_planning_msgs::msg::builder::Init_PointCloudWithPose_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__BUILDER_HPP_
