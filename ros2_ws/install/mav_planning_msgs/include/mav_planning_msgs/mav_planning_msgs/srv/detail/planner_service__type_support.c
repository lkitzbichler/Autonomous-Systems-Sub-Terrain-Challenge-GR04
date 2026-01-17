// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_introspection_c.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_planning_msgs/srv/detail/planner_service__functions.h"
#include "mav_planning_msgs/srv/detail/planner_service__struct.h"


// Include directives for member types
// Member `start_pose`
// Member `goal_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `start_pose`
// Member `goal_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `start_velocity`
// Member `goal_velocity`
// Member `bounding_box`
#include "geometry_msgs/msg/vector3.h"
// Member `start_velocity`
// Member `goal_velocity`
// Member `bounding_box`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__srv__PlannerService_Request__init(message_memory);
}

void mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_fini_function(void * message_memory)
{
  mav_planning_msgs__srv__PlannerService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[5] = {
  {
    "start_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Request, start_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Request, start_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Request, goal_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Request, goal_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_box",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Request, bounding_box),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_members = {
  "mav_planning_msgs__srv",  // message namespace
  "PlannerService_Request",  // message name
  5,  // number of fields
  sizeof(mav_planning_msgs__srv__PlannerService_Request),
  false,  // has_any_key_member_
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array,  // message members
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle = {
  0,
  &mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Request)() {
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"


// Include directives for member types
// Member `polynomial_plan`
#include "mav_planning_msgs/msg/polynomial_trajectory.h"
// Member `polynomial_plan`
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__rosidl_typesupport_introspection_c.h"
// Member `polynomial_plan_4d`
#include "mav_planning_msgs/msg/polynomial_trajectory4_d.h"
// Member `polynomial_plan_4d`
#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__rosidl_typesupport_introspection_c.h"
// Member `sampled_plan`
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.h"
// Member `sampled_plan`
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__srv__PlannerService_Response__init(message_memory);
}

void mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_fini_function(void * message_memory)
{
  mav_planning_msgs__srv__PlannerService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_member_array[4] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polynomial_plan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Response, polynomial_plan),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polynomial_plan_4d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Response, polynomial_plan_4d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sampled_plan",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Response, sampled_plan),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_members = {
  "mav_planning_msgs__srv",  // message namespace
  "PlannerService_Response",  // message name
  4,  // number of fields
  sizeof(mav_planning_msgs__srv__PlannerService_Response),
  false,  // has_any_key_member_
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_member_array,  // message members
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle = {
  0,
  &mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Response)() {
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolynomialTrajectory)();
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolynomialTrajectory4D)();
  mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, trajectory_msgs, msg, MultiDOFJointTrajectory)();
  if (!mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "mav_planning_msgs/srv/planner_service.h"
// Member `request`
// Member `response`
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__srv__PlannerService_Event__init(message_memory);
}

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_fini_function(void * message_memory)
{
  mav_planning_msgs__srv__PlannerService_Event__fini(message_memory);
}

size_t mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__size_function__PlannerService_Event__request(
  const void * untyped_member)
{
  const mav_planning_msgs__srv__PlannerService_Request__Sequence * member =
    (const mav_planning_msgs__srv__PlannerService_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__request(
  const void * untyped_member, size_t index)
{
  const mav_planning_msgs__srv__PlannerService_Request__Sequence * member =
    (const mav_planning_msgs__srv__PlannerService_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__request(
  void * untyped_member, size_t index)
{
  mav_planning_msgs__srv__PlannerService_Request__Sequence * member =
    (mav_planning_msgs__srv__PlannerService_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__fetch_function__PlannerService_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mav_planning_msgs__srv__PlannerService_Request * item =
    ((const mav_planning_msgs__srv__PlannerService_Request *)
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__request(untyped_member, index));
  mav_planning_msgs__srv__PlannerService_Request * value =
    (mav_planning_msgs__srv__PlannerService_Request *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__assign_function__PlannerService_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mav_planning_msgs__srv__PlannerService_Request * item =
    ((mav_planning_msgs__srv__PlannerService_Request *)
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__request(untyped_member, index));
  const mav_planning_msgs__srv__PlannerService_Request * value =
    (const mav_planning_msgs__srv__PlannerService_Request *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__resize_function__PlannerService_Event__request(
  void * untyped_member, size_t size)
{
  mav_planning_msgs__srv__PlannerService_Request__Sequence * member =
    (mav_planning_msgs__srv__PlannerService_Request__Sequence *)(untyped_member);
  mav_planning_msgs__srv__PlannerService_Request__Sequence__fini(member);
  return mav_planning_msgs__srv__PlannerService_Request__Sequence__init(member, size);
}

size_t mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__size_function__PlannerService_Event__response(
  const void * untyped_member)
{
  const mav_planning_msgs__srv__PlannerService_Response__Sequence * member =
    (const mav_planning_msgs__srv__PlannerService_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__response(
  const void * untyped_member, size_t index)
{
  const mav_planning_msgs__srv__PlannerService_Response__Sequence * member =
    (const mav_planning_msgs__srv__PlannerService_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__response(
  void * untyped_member, size_t index)
{
  mav_planning_msgs__srv__PlannerService_Response__Sequence * member =
    (mav_planning_msgs__srv__PlannerService_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__fetch_function__PlannerService_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mav_planning_msgs__srv__PlannerService_Response * item =
    ((const mav_planning_msgs__srv__PlannerService_Response *)
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__response(untyped_member, index));
  mav_planning_msgs__srv__PlannerService_Response * value =
    (mav_planning_msgs__srv__PlannerService_Response *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__assign_function__PlannerService_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mav_planning_msgs__srv__PlannerService_Response * item =
    ((mav_planning_msgs__srv__PlannerService_Response *)
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__response(untyped_member, index));
  const mav_planning_msgs__srv__PlannerService_Response * value =
    (const mav_planning_msgs__srv__PlannerService_Response *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__resize_function__PlannerService_Event__response(
  void * untyped_member, size_t size)
{
  mav_planning_msgs__srv__PlannerService_Response__Sequence * member =
    (mav_planning_msgs__srv__PlannerService_Response__Sequence *)(untyped_member);
  mav_planning_msgs__srv__PlannerService_Response__Sequence__fini(member);
  return mav_planning_msgs__srv__PlannerService_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Event, request),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__size_function__PlannerService_Event__request,  // size() function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__request,  // get_const(index) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__request,  // get(index) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__fetch_function__PlannerService_Event__request,  // fetch(index, &value) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__assign_function__PlannerService_Event__request,  // assign(index, value) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__resize_function__PlannerService_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mav_planning_msgs__srv__PlannerService_Event, response),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__size_function__PlannerService_Event__response,  // size() function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_const_function__PlannerService_Event__response,  // get_const(index) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__get_function__PlannerService_Event__response,  // get(index) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__fetch_function__PlannerService_Event__response,  // fetch(index, &value) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__assign_function__PlannerService_Event__response,  // assign(index, value) function pointer
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__resize_function__PlannerService_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_members = {
  "mav_planning_msgs__srv",  // message namespace
  "PlannerService_Event",  // message name
  3,  // number of fields
  sizeof(mav_planning_msgs__srv__PlannerService_Event),
  false,  // has_any_key_member_
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_member_array,  // message members
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_type_support_handle = {
  0,
  &mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Event)() {
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Request)();
  mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Response)();
  if (!mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_members = {
  "mav_planning_msgs__srv",  // service namespace
  "PlannerService",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle,
  NULL,  // response message
  // mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle
  NULL  // event_message
  // mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle
};


static rosidl_service_type_support_t mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_type_support_handle = {
  0,
  &mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_members,
  get_service_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Request__rosidl_typesupport_introspection_c__PlannerService_Request_message_type_support_handle,
  &mav_planning_msgs__srv__PlannerService_Response__rosidl_typesupport_introspection_c__PlannerService_Response_message_type_support_handle,
  &mav_planning_msgs__srv__PlannerService_Event__rosidl_typesupport_introspection_c__PlannerService_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mav_planning_msgs,
    srv,
    PlannerService
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mav_planning_msgs,
    srv,
    PlannerService
  ),
  &mav_planning_msgs__srv__PlannerService__get_type_hash,
  &mav_planning_msgs__srv__PlannerService__get_type_description,
  &mav_planning_msgs__srv__PlannerService__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService)(void) {
  if (!mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Event)()->data;
  }

  return &mav_planning_msgs__srv__detail__planner_service__rosidl_typesupport_introspection_c__PlannerService_service_type_support_handle;
}
