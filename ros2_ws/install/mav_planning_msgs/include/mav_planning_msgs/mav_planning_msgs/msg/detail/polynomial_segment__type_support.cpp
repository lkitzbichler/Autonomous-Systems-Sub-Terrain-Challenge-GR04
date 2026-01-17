// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_planning_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PolynomialSegment_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::msg::PolynomialSegment(_init);
}

void PolynomialSegment_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::msg::PolynomialSegment *>(message_memory);
  typed_message->~PolynomialSegment();
}

size_t size_function__PolynomialSegment__x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__x(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__x(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__y(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__y(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__z(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__z(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__z(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__z(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__z(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__z(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__rx(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__rx(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__rx(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__rx(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__rx(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__rx(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__rx(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__rx(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__ry(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__ry(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__ry(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__ry(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__ry(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__ry(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__ry(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__ry(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__rz(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__rz(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__rz(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__rz(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__rz(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__rz(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__rz(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__rz(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PolynomialSegment__yaw(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PolynomialSegment__yaw(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__PolynomialSegment__yaw(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__PolynomialSegment__yaw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__PolynomialSegment__yaw(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__PolynomialSegment__yaw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__PolynomialSegment__yaw(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__PolynomialSegment__yaw(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PolynomialSegment_message_member_array[10] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "num_coeffs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, num_coeffs),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "segment_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Duration>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, segment_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, x),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__x,  // size() function pointer
    get_const_function__PolynomialSegment__x,  // get_const(index) function pointer
    get_function__PolynomialSegment__x,  // get(index) function pointer
    fetch_function__PolynomialSegment__x,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__x,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__x  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, y),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__y,  // size() function pointer
    get_const_function__PolynomialSegment__y,  // get_const(index) function pointer
    get_function__PolynomialSegment__y,  // get(index) function pointer
    fetch_function__PolynomialSegment__y,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__y,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__y  // resize(index) function pointer
  },
  {
    "z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, z),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__z,  // size() function pointer
    get_const_function__PolynomialSegment__z,  // get_const(index) function pointer
    get_function__PolynomialSegment__z,  // get(index) function pointer
    fetch_function__PolynomialSegment__z,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__z,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__z  // resize(index) function pointer
  },
  {
    "rx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, rx),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__rx,  // size() function pointer
    get_const_function__PolynomialSegment__rx,  // get_const(index) function pointer
    get_function__PolynomialSegment__rx,  // get(index) function pointer
    fetch_function__PolynomialSegment__rx,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__rx,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__rx  // resize(index) function pointer
  },
  {
    "ry",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, ry),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__ry,  // size() function pointer
    get_const_function__PolynomialSegment__ry,  // get_const(index) function pointer
    get_function__PolynomialSegment__ry,  // get(index) function pointer
    fetch_function__PolynomialSegment__ry,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__ry,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__ry  // resize(index) function pointer
  },
  {
    "rz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, rz),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__rz,  // size() function pointer
    get_const_function__PolynomialSegment__rz,  // get_const(index) function pointer
    get_function__PolynomialSegment__rz,  // get(index) function pointer
    fetch_function__PolynomialSegment__rz,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__rz,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__rz  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolynomialSegment, yaw),  // bytes offset in struct
    nullptr,  // default value
    size_function__PolynomialSegment__yaw,  // size() function pointer
    get_const_function__PolynomialSegment__yaw,  // get_const(index) function pointer
    get_function__PolynomialSegment__yaw,  // get(index) function pointer
    fetch_function__PolynomialSegment__yaw,  // fetch(index, &value) function pointer
    assign_function__PolynomialSegment__yaw,  // assign(index, value) function pointer
    resize_function__PolynomialSegment__yaw  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PolynomialSegment_message_members = {
  "mav_planning_msgs::msg",  // message namespace
  "PolynomialSegment",  // message name
  10,  // number of fields
  sizeof(mav_planning_msgs::msg::PolynomialSegment),
  false,  // has_any_key_member_
  PolynomialSegment_message_member_array,  // message members
  PolynomialSegment_init_function,  // function to initialize message memory (memory has to be allocated)
  PolynomialSegment_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PolynomialSegment_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PolynomialSegment_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_hash,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_description,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::msg::PolynomialSegment>()
{
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::PolynomialSegment_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, msg, PolynomialSegment)() {
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::PolynomialSegment_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
