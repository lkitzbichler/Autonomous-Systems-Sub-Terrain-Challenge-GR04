// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point_cloud_with_pose.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__TRAITS_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'sensor_pose'
#include "geometry_msgs/msg/detail/transform_stamped__traits.hpp"
// Member 'cloud_in_sensor_frame'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace mav_planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointCloudWithPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: sensor_pose
  {
    out << "sensor_pose: ";
    to_flow_style_yaml(msg.sensor_pose, out);
    out << ", ";
  }

  // member: cloud_in_sensor_frame
  {
    out << "cloud_in_sensor_frame: ";
    to_flow_style_yaml(msg.cloud_in_sensor_frame, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointCloudWithPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: sensor_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor_pose:\n";
    to_block_style_yaml(msg.sensor_pose, out, indentation + 2);
  }

  // member: cloud_in_sensor_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_in_sensor_frame:\n";
    to_block_style_yaml(msg.cloud_in_sensor_frame, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointCloudWithPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::msg::PointCloudWithPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::msg::PointCloudWithPose & msg)
{
  return mav_planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::msg::PointCloudWithPose>()
{
  return "mav_planning_msgs::msg::PointCloudWithPose";
}

template<>
inline const char * name<mav_planning_msgs::msg::PointCloudWithPose>()
{
  return "mav_planning_msgs/msg/PointCloudWithPose";
}

template<>
struct has_fixed_size<mav_planning_msgs::msg::PointCloudWithPose>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::TransformStamped>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mav_planning_msgs::msg::PointCloudWithPose>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::TransformStamped>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mav_planning_msgs::msg::PointCloudWithPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__TRAITS_HPP_
