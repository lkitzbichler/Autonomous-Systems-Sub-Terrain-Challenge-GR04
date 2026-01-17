// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/planner_service.h"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_H_
#define MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'start_velocity'
// Member 'goal_velocity'
// Member 'bounding_box'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in srv/PlannerService in the package mav_planning_msgs.
typedef struct mav_planning_msgs__srv__PlannerService_Request
{
  /// start pose for the planner
  geometry_msgs__msg__PoseStamped start_pose;
  geometry_msgs__msg__Vector3 start_velocity;
  /// start pose for the planner
  geometry_msgs__msg__PoseStamped goal_pose;
  geometry_msgs__msg__Vector3 goal_velocity;
  geometry_msgs__msg__Vector3 bounding_box;
} mav_planning_msgs__srv__PlannerService_Request;

// Struct for a sequence of mav_planning_msgs__srv__PlannerService_Request.
typedef struct mav_planning_msgs__srv__PlannerService_Request__Sequence
{
  mav_planning_msgs__srv__PlannerService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__srv__PlannerService_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'polynomial_plan'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__struct.h"
// Member 'polynomial_plan_4d'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__struct.h"
// Member 'sampled_plan'
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__struct.h"

/// Struct defined in srv/PlannerService in the package mav_planning_msgs.
typedef struct mav_planning_msgs__srv__PlannerService_Response
{
  bool success;
  /// Either contains a polynomial trajectory:
  mav_planning_msgs__msg__PolynomialTrajectory polynomial_plan;
  mav_planning_msgs__msg__PolynomialTrajectory4D polynomial_plan_4d;
  /// or a MultiDOFJointTrajectory containing a sampled path (or straight-line
  /// waypoints, depending on the planner).
  /// Only one of these should be non-empty.
  trajectory_msgs__msg__MultiDOFJointTrajectory sampled_plan;
} mav_planning_msgs__srv__PlannerService_Response;

// Struct for a sequence of mav_planning_msgs__srv__PlannerService_Response.
typedef struct mav_planning_msgs__srv__PlannerService_Response__Sequence
{
  mav_planning_msgs__srv__PlannerService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__srv__PlannerService_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mav_planning_msgs__srv__PlannerService_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mav_planning_msgs__srv__PlannerService_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/PlannerService in the package mav_planning_msgs.
typedef struct mav_planning_msgs__srv__PlannerService_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mav_planning_msgs__srv__PlannerService_Request__Sequence request;
  mav_planning_msgs__srv__PlannerService_Response__Sequence response;
} mav_planning_msgs__srv__PlannerService_Event;

// Struct for a sequence of mav_planning_msgs__srv__PlannerService_Event.
typedef struct mav_planning_msgs__srv__PlannerService_Event__Sequence
{
  mav_planning_msgs__srv__PlannerService_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__srv__PlannerService_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_H_
