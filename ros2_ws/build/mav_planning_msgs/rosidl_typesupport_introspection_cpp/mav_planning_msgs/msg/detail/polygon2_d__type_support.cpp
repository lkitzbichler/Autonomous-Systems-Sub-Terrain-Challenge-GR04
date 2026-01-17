// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/msg/detail/polygon2_d__functions.h"
#include "mav_planning_msgs/msg/detail/polygon2_d__struct.hpp"
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

void Polygon2D_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::msg::Polygon2D(_init);
}

void Polygon2D_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::msg::Polygon2D *>(message_memory);
  typed_message->~Polygon2D();
}

size_t size_function__Polygon2D__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mav_planning_msgs::msg::Point2D> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Polygon2D__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mav_planning_msgs::msg::Point2D> *>(untyped_member);
  return &member[index];
}

void * get_function__Polygon2D__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mav_planning_msgs::msg::Point2D> *>(untyped_member);
  return &member[index];
}

void fetch_function__Polygon2D__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mav_planning_msgs::msg::Point2D *>(
    get_const_function__Polygon2D__points(untyped_member, index));
  auto & value = *reinterpret_cast<mav_planning_msgs::msg::Point2D *>(untyped_value);
  value = item;
}

void assign_function__Polygon2D__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mav_planning_msgs::msg::Point2D *>(
    get_function__Polygon2D__points(untyped_member, index));
  const auto & value = *reinterpret_cast<const mav_planning_msgs::msg::Point2D *>(untyped_value);
  item = value;
}

void resize_function__Polygon2D__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mav_planning_msgs::msg::Point2D> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Polygon2D_message_member_array[1] = {
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::msg::Point2D>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::Polygon2D, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__Polygon2D__points,  // size() function pointer
    get_const_function__Polygon2D__points,  // get_const(index) function pointer
    get_function__Polygon2D__points,  // get(index) function pointer
    fetch_function__Polygon2D__points,  // fetch(index, &value) function pointer
    assign_function__Polygon2D__points,  // assign(index, value) function pointer
    resize_function__Polygon2D__points  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Polygon2D_message_members = {
  "mav_planning_msgs::msg",  // message namespace
  "Polygon2D",  // message name
  1,  // number of fields
  sizeof(mav_planning_msgs::msg::Polygon2D),
  false,  // has_any_key_member_
  Polygon2D_message_member_array,  // message members
  Polygon2D_init_function,  // function to initialize message memory (memory has to be allocated)
  Polygon2D_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Polygon2D_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Polygon2D_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__Polygon2D__get_type_hash,
  &mav_planning_msgs__msg__Polygon2D__get_type_description,
  &mav_planning_msgs__msg__Polygon2D__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::msg::Polygon2D>()
{
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::Polygon2D_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, msg, Polygon2D)() {
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::Polygon2D_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
