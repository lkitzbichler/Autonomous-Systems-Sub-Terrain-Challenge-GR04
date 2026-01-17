// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:srv/PolygonService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/polygon_service.hpp"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__POLYGON_SERVICE__BUILDER_HPP_
#define MAV_PLANNING_MSGS__SRV__DETAIL__POLYGON_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/srv/detail/polygon_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PolygonService_Request_polygon
{
public:
  Init_PolygonService_Request_polygon()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mav_planning_msgs::srv::PolygonService_Request polygon(::mav_planning_msgs::srv::PolygonService_Request::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PolygonService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PolygonService_Request>()
{
  return mav_planning_msgs::srv::builder::Init_PolygonService_Request_polygon();
}

}  // namespace mav_planning_msgs


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PolygonService_Response_success
{
public:
  Init_PolygonService_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mav_planning_msgs::srv::PolygonService_Response success(::mav_planning_msgs::srv::PolygonService_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PolygonService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PolygonService_Response>()
{
  return mav_planning_msgs::srv::builder::Init_PolygonService_Response_success();
}

}  // namespace mav_planning_msgs


namespace mav_planning_msgs
{

namespace srv
{

namespace builder
{

class Init_PolygonService_Event_response
{
public:
  explicit Init_PolygonService_Event_response(::mav_planning_msgs::srv::PolygonService_Event & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::srv::PolygonService_Event response(::mav_planning_msgs::srv::PolygonService_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::srv::PolygonService_Event msg_;
};

class Init_PolygonService_Event_request
{
public:
  explicit Init_PolygonService_Event_request(::mav_planning_msgs::srv::PolygonService_Event & msg)
  : msg_(msg)
  {}
  Init_PolygonService_Event_response request(::mav_planning_msgs::srv::PolygonService_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PolygonService_Event_response(msg_);
  }

private:
  ::mav_planning_msgs::srv::PolygonService_Event msg_;
};

class Init_PolygonService_Event_info
{
public:
  Init_PolygonService_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolygonService_Event_request info(::mav_planning_msgs::srv::PolygonService_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PolygonService_Event_request(msg_);
  }

private:
  ::mav_planning_msgs::srv::PolygonService_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::srv::PolygonService_Event>()
{
  return mav_planning_msgs::srv::builder::Init_PolygonService_Event_info();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__POLYGON_SERVICE__BUILDER_HPP_
