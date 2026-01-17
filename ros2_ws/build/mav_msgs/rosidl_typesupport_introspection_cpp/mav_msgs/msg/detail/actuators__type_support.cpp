// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_msgs/msg/detail/actuators__functions.h"
#include "mav_msgs/msg/detail/actuators__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Actuators_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_msgs::msg::Actuators(_init);
}

void Actuators_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_msgs::msg::Actuators *>(message_memory);
  typed_message->~Actuators();
}

size_t size_function__Actuators__angles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Actuators__angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Actuators__angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Actuators__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Actuators__angles(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Actuators__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Actuators__angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Actuators__angles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Actuators__angular_velocities(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Actuators__angular_velocities(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Actuators__angular_velocities(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Actuators__angular_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Actuators__angular_velocities(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Actuators__angular_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Actuators__angular_velocities(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Actuators__angular_velocities(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__Actuators__normalized(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Actuators__normalized(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__Actuators__normalized(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__Actuators__normalized(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__Actuators__normalized(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__Actuators__normalized(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__Actuators__normalized(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__Actuators__normalized(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Actuators_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs::msg::Actuators, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs::msg::Actuators, angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__Actuators__angles,  // size() function pointer
    get_const_function__Actuators__angles,  // get_const(index) function pointer
    get_function__Actuators__angles,  // get(index) function pointer
    fetch_function__Actuators__angles,  // fetch(index, &value) function pointer
    assign_function__Actuators__angles,  // assign(index, value) function pointer
    resize_function__Actuators__angles  // resize(index) function pointer
  },
  {
    "angular_velocities",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs::msg::Actuators, angular_velocities),  // bytes offset in struct
    nullptr,  // default value
    size_function__Actuators__angular_velocities,  // size() function pointer
    get_const_function__Actuators__angular_velocities,  // get_const(index) function pointer
    get_function__Actuators__angular_velocities,  // get(index) function pointer
    fetch_function__Actuators__angular_velocities,  // fetch(index, &value) function pointer
    assign_function__Actuators__angular_velocities,  // assign(index, value) function pointer
    resize_function__Actuators__angular_velocities  // resize(index) function pointer
  },
  {
    "normalized",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs::msg::Actuators, normalized),  // bytes offset in struct
    nullptr,  // default value
    size_function__Actuators__normalized,  // size() function pointer
    get_const_function__Actuators__normalized,  // get_const(index) function pointer
    get_function__Actuators__normalized,  // get(index) function pointer
    fetch_function__Actuators__normalized,  // fetch(index, &value) function pointer
    assign_function__Actuators__normalized,  // assign(index, value) function pointer
    resize_function__Actuators__normalized  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Actuators_message_members = {
  "mav_msgs::msg",  // message namespace
  "Actuators",  // message name
  4,  // number of fields
  sizeof(mav_msgs::msg::Actuators),
  false,  // has_any_key_member_
  Actuators_message_member_array,  // message members
  Actuators_init_function,  // function to initialize message memory (memory has to be allocated)
  Actuators_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Actuators_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Actuators_message_members,
  get_message_typesupport_handle_function,
  &mav_msgs__msg__Actuators__get_type_hash,
  &mav_msgs__msg__Actuators__get_type_description,
  &mav_msgs__msg__Actuators__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mav_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_msgs::msg::Actuators>()
{
  return &::mav_msgs::msg::rosidl_typesupport_introspection_cpp::Actuators_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_msgs, msg, Actuators)() {
  return &::mav_msgs::msg::rosidl_typesupport_introspection_cpp::Actuators_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
