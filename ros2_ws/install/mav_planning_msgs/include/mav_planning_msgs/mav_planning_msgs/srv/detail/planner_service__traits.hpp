// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/planner_service.hpp"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__TRAITS_HPP_
#define MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'start_velocity'
// Member 'goal_velocity'
// Member 'bounding_box'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mav_planning_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlannerService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_pose
  {
    out << "start_pose: ";
    to_flow_style_yaml(msg.start_pose, out);
    out << ", ";
  }

  // member: start_velocity
  {
    out << "start_velocity: ";
    to_flow_style_yaml(msg.start_velocity, out);
    out << ", ";
  }

  // member: goal_pose
  {
    out << "goal_pose: ";
    to_flow_style_yaml(msg.goal_pose, out);
    out << ", ";
  }

  // member: goal_velocity
  {
    out << "goal_velocity: ";
    to_flow_style_yaml(msg.goal_velocity, out);
    out << ", ";
  }

  // member: bounding_box
  {
    out << "bounding_box: ";
    to_flow_style_yaml(msg.bounding_box, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlannerService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_pose:\n";
    to_block_style_yaml(msg.start_pose, out, indentation + 2);
  }

  // member: start_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_velocity:\n";
    to_block_style_yaml(msg.start_velocity, out, indentation + 2);
  }

  // member: goal_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_pose:\n";
    to_block_style_yaml(msg.goal_pose, out, indentation + 2);
  }

  // member: goal_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_velocity:\n";
    to_block_style_yaml(msg.goal_velocity, out, indentation + 2);
  }

  // member: bounding_box
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bounding_box:\n";
    to_block_style_yaml(msg.bounding_box, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlannerService_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::srv::PlannerService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::srv::PlannerService_Request & msg)
{
  return mav_planning_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::srv::PlannerService_Request>()
{
  return "mav_planning_msgs::srv::PlannerService_Request";
}

template<>
inline const char * name<mav_planning_msgs::srv::PlannerService_Request>()
{
  return "mav_planning_msgs/srv/PlannerService_Request";
}

template<>
struct has_fixed_size<mav_planning_msgs::srv::PlannerService_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<mav_planning_msgs::srv::PlannerService_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<mav_planning_msgs::srv::PlannerService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'polynomial_plan'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__traits.hpp"
// Member 'polynomial_plan_4d'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__traits.hpp"
// Member 'sampled_plan'
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__traits.hpp"

namespace mav_planning_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlannerService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: polynomial_plan
  {
    out << "polynomial_plan: ";
    to_flow_style_yaml(msg.polynomial_plan, out);
    out << ", ";
  }

  // member: polynomial_plan_4d
  {
    out << "polynomial_plan_4d: ";
    to_flow_style_yaml(msg.polynomial_plan_4d, out);
    out << ", ";
  }

  // member: sampled_plan
  {
    out << "sampled_plan: ";
    to_flow_style_yaml(msg.sampled_plan, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlannerService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: polynomial_plan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "polynomial_plan:\n";
    to_block_style_yaml(msg.polynomial_plan, out, indentation + 2);
  }

  // member: polynomial_plan_4d
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "polynomial_plan_4d:\n";
    to_block_style_yaml(msg.polynomial_plan_4d, out, indentation + 2);
  }

  // member: sampled_plan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sampled_plan:\n";
    to_block_style_yaml(msg.sampled_plan, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlannerService_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::srv::PlannerService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::srv::PlannerService_Response & msg)
{
  return mav_planning_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::srv::PlannerService_Response>()
{
  return "mav_planning_msgs::srv::PlannerService_Response";
}

template<>
inline const char * name<mav_planning_msgs::srv::PlannerService_Response>()
{
  return "mav_planning_msgs/srv/PlannerService_Response";
}

template<>
struct has_fixed_size<mav_planning_msgs::srv::PlannerService_Response>
  : std::integral_constant<bool, has_fixed_size<mav_planning_msgs::msg::PolynomialTrajectory4D>::value && has_fixed_size<mav_planning_msgs::msg::PolynomialTrajectory>::value && has_fixed_size<trajectory_msgs::msg::MultiDOFJointTrajectory>::value> {};

template<>
struct has_bounded_size<mav_planning_msgs::srv::PlannerService_Response>
  : std::integral_constant<bool, has_bounded_size<mav_planning_msgs::msg::PolynomialTrajectory4D>::value && has_bounded_size<mav_planning_msgs::msg::PolynomialTrajectory>::value && has_bounded_size<trajectory_msgs::msg::MultiDOFJointTrajectory>::value> {};

template<>
struct is_message<mav_planning_msgs::srv::PlannerService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mav_planning_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PlannerService_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlannerService_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlannerService_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::srv::PlannerService_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::srv::PlannerService_Event & msg)
{
  return mav_planning_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::srv::PlannerService_Event>()
{
  return "mav_planning_msgs::srv::PlannerService_Event";
}

template<>
inline const char * name<mav_planning_msgs::srv::PlannerService_Event>()
{
  return "mav_planning_msgs/srv/PlannerService_Event";
}

template<>
struct has_fixed_size<mav_planning_msgs::srv::PlannerService_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_planning_msgs::srv::PlannerService_Event>
  : std::integral_constant<bool, has_bounded_size<mav_planning_msgs::srv::PlannerService_Request>::value && has_bounded_size<mav_planning_msgs::srv::PlannerService_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mav_planning_msgs::srv::PlannerService_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mav_planning_msgs::srv::PlannerService>()
{
  return "mav_planning_msgs::srv::PlannerService";
}

template<>
inline const char * name<mav_planning_msgs::srv::PlannerService>()
{
  return "mav_planning_msgs/srv/PlannerService";
}

template<>
struct has_fixed_size<mav_planning_msgs::srv::PlannerService>
  : std::integral_constant<
    bool,
    has_fixed_size<mav_planning_msgs::srv::PlannerService_Request>::value &&
    has_fixed_size<mav_planning_msgs::srv::PlannerService_Response>::value
  >
{
};

template<>
struct has_bounded_size<mav_planning_msgs::srv::PlannerService>
  : std::integral_constant<
    bool,
    has_bounded_size<mav_planning_msgs::srv::PlannerService_Request>::value &&
    has_bounded_size<mav_planning_msgs::srv::PlannerService_Response>::value
  >
{
};

template<>
struct is_service<mav_planning_msgs::srv::PlannerService>
  : std::true_type
{
};

template<>
struct is_service_request<mav_planning_msgs::srv::PlannerService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mav_planning_msgs::srv::PlannerService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__TRAITS_HPP_
