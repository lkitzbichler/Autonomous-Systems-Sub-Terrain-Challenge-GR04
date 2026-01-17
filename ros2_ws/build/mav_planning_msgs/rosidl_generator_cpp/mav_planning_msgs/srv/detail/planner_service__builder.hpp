// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/planner_service.hpp"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__BUILDER_HPP_
#define MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PlannerService_Request_bounding_box
{
public:
  explicit Init_PlannerService_Request_bounding_box(::mav_planning_msgs::srv::PlannerService_Request & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::srv::PlannerService_Request bounding_box(::mav_planning_msgs::srv::PlannerService_Request::_bounding_box_type arg)
  {
    msg_.bounding_box = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Request msg_;
};

class Init_PlannerService_Request_goal_velocity
{
public:
  explicit Init_PlannerService_Request_goal_velocity(::mav_planning_msgs::srv::PlannerService_Request & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Request_bounding_box goal_velocity(::mav_planning_msgs::srv::PlannerService_Request::_goal_velocity_type arg)
  {
    msg_.goal_velocity = std::move(arg);
    return Init_PlannerService_Request_bounding_box(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Request msg_;
};

class Init_PlannerService_Request_goal_pose
{
public:
  explicit Init_PlannerService_Request_goal_pose(::mav_planning_msgs::srv::PlannerService_Request & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Request_goal_velocity goal_pose(::mav_planning_msgs::srv::PlannerService_Request::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return Init_PlannerService_Request_goal_velocity(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Request msg_;
};

class Init_PlannerService_Request_start_velocity
{
public:
  explicit Init_PlannerService_Request_start_velocity(::mav_planning_msgs::srv::PlannerService_Request & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Request_goal_pose start_velocity(::mav_planning_msgs::srv::PlannerService_Request::_start_velocity_type arg)
  {
    msg_.start_velocity = std::move(arg);
    return Init_PlannerService_Request_goal_pose(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Request msg_;
};

class Init_PlannerService_Request_start_pose
{
public:
  Init_PlannerService_Request_start_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerService_Request_start_velocity start_pose(::mav_planning_msgs::srv::PlannerService_Request::_start_pose_type arg)
  {
    msg_.start_pose = std::move(arg);
    return Init_PlannerService_Request_start_velocity(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PlannerService_Request>()
{
  return mav_planning_msgs::srv::builder::Init_PlannerService_Request_start_pose();
}

}  // namespace mav_planning_msgs


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PlannerService_Response_sampled_plan
{
public:
  explicit Init_PlannerService_Response_sampled_plan(::mav_planning_msgs::srv::PlannerService_Response & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::srv::PlannerService_Response sampled_plan(::mav_planning_msgs::srv::PlannerService_Response::_sampled_plan_type arg)
  {
    msg_.sampled_plan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Response msg_;
};

class Init_PlannerService_Response_polynomial_plan_4d
{
public:
  explicit Init_PlannerService_Response_polynomial_plan_4d(::mav_planning_msgs::srv::PlannerService_Response & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Response_sampled_plan polynomial_plan_4d(::mav_planning_msgs::srv::PlannerService_Response::_polynomial_plan_4d_type arg)
  {
    msg_.polynomial_plan_4d = std::move(arg);
    return Init_PlannerService_Response_sampled_plan(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Response msg_;
};

class Init_PlannerService_Response_polynomial_plan
{
public:
  explicit Init_PlannerService_Response_polynomial_plan(::mav_planning_msgs::srv::PlannerService_Response & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Response_polynomial_plan_4d polynomial_plan(::mav_planning_msgs::srv::PlannerService_Response::_polynomial_plan_type arg)
  {
    msg_.polynomial_plan = std::move(arg);
    return Init_PlannerService_Response_polynomial_plan_4d(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Response msg_;
};

class Init_PlannerService_Response_success
{
public:
  Init_PlannerService_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerService_Response_polynomial_plan success(::mav_planning_msgs::srv::PlannerService_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlannerService_Response_polynomial_plan(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PlannerService_Response>()
{
  return mav_planning_msgs::srv::builder::Init_PlannerService_Response_success();
}

}  // namespace mav_planning_msgs


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PlannerService_Event_response
{
public:
  explicit Init_PlannerService_Event_response(::mav_planning_msgs::srv::PlannerService_Event & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::srv::PlannerService_Event response(::mav_planning_msgs::srv::PlannerService_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Event msg_;
};

class Init_PlannerService_Event_request
{
public:
  explicit Init_PlannerService_Event_request(::mav_planning_msgs::srv::PlannerService_Event & msg)
  : msg_(msg)
  {}
  Init_PlannerService_Event_response request(::mav_planning_msgs::srv::PlannerService_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlannerService_Event_response(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Event msg_;
};

class Init_PlannerService_Event_info
{
public:
  Init_PlannerService_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerService_Event_request info(::mav_planning_msgs::srv::PlannerService_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlannerService_Event_request(msg_);
  }

private:
  ::mav_planning_msgs::srv::PlannerService_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PlannerService_Event>()
{
  return mav_planning_msgs::srv::builder::Init_PlannerService_Event_info();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__BUILDER_HPP_
