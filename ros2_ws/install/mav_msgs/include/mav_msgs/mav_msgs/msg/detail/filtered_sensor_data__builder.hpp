// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/filtered_sensor_data.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/filtered_sensor_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_FilteredSensorData_barometer
{
public:
  explicit Init_FilteredSensorData_barometer(::mav_msgs::msg::FilteredSensorData & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::FilteredSensorData barometer(::mav_msgs::msg::FilteredSensorData::_barometer_type arg)
  {
    msg_.barometer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::FilteredSensorData msg_;
};

class Init_FilteredSensorData_magnetometer
{
public:
  explicit Init_FilteredSensorData_magnetometer(::mav_msgs::msg::FilteredSensorData & msg)
  : msg_(msg)
  {}
  Init_FilteredSensorData_barometer magnetometer(::mav_msgs::msg::FilteredSensorData::_magnetometer_type arg)
  {
    msg_.magnetometer = std::move(arg);
    return Init_FilteredSensorData_barometer(msg_);
  }

private:
  ::mav_msgs::msg::FilteredSensorData msg_;
};

class Init_FilteredSensorData_gyroscope
{
public:
  explicit Init_FilteredSensorData_gyroscope(::mav_msgs::msg::FilteredSensorData & msg)
  : msg_(msg)
  {}
  Init_FilteredSensorData_magnetometer gyroscope(::mav_msgs::msg::FilteredSensorData::_gyroscope_type arg)
  {
    msg_.gyroscope = std::move(arg);
    return Init_FilteredSensorData_magnetometer(msg_);
  }

private:
  ::mav_msgs::msg::FilteredSensorData msg_;
};

class Init_FilteredSensorData_accelerometer
{
public:
  explicit Init_FilteredSensorData_accelerometer(::mav_msgs::msg::FilteredSensorData & msg)
  : msg_(msg)
  {}
  Init_FilteredSensorData_gyroscope accelerometer(::mav_msgs::msg::FilteredSensorData::_accelerometer_type arg)
  {
    msg_.accelerometer = std::move(arg);
    return Init_FilteredSensorData_gyroscope(msg_);
  }

private:
  ::mav_msgs::msg::FilteredSensorData msg_;
};

class Init_FilteredSensorData_header
{
public:
  Init_FilteredSensorData_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredSensorData_accelerometer header(::mav_msgs::msg::FilteredSensorData::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FilteredSensorData_accelerometer(msg_);
  }

private:
  ::mav_msgs::msg::FilteredSensorData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::FilteredSensorData>()
{
  return mav_msgs::msg::builder::Init_FilteredSensorData_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__BUILDER_HPP_
