// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

#include "mav_planning_msgs/srv/detail/planner_service__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PlannerService__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0x30, 0xf8, 0x7d, 0x7b, 0xc1, 0xe7, 0xee,
      0x2d, 0x37, 0x43, 0xb4, 0xfb, 0x0f, 0xaf, 0xe4,
      0xd5, 0xb6, 0x2f, 0x13, 0x74, 0x8f, 0x57, 0x7a,
      0x87, 0xc6, 0x30, 0x3f, 0x87, 0x9d, 0xb2, 0x38,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PlannerService_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x94, 0x64, 0x6d, 0xe9, 0xde, 0x46, 0xa4, 0x9f,
      0xa1, 0xce, 0x49, 0xbd, 0x34, 0x39, 0x5b, 0x80,
      0x99, 0xa7, 0x84, 0xd8, 0x56, 0xb0, 0xbb, 0x9a,
      0x1f, 0xc6, 0xf6, 0xba, 0xa6, 0x7c, 0x39, 0x42,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PlannerService_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb4, 0x2e, 0xa2, 0x98, 0x3f, 0xa6, 0x8a, 0x71,
      0x4d, 0x19, 0x72, 0xb3, 0x5b, 0x76, 0x02, 0x9a,
      0x35, 0xa8, 0x92, 0x5e, 0xd3, 0x50, 0xed, 0xe5,
      0xf9, 0x84, 0x5f, 0xa7, 0xb7, 0x96, 0x45, 0xbb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PlannerService_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcb, 0x3a, 0x49, 0x7b, 0xa7, 0x47, 0xe9, 0x5c,
      0x2a, 0xc6, 0x8e, 0xcf, 0x13, 0x4d, 0x0b, 0xc3,
      0x18, 0xec, 0xd9, 0x79, 0xa6, 0xe9, 0xec, 0x2c,
      0x0a, 0x08, 0x9a, 0xe4, 0x66, 0x4c, 0x09, 0xfb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "geometry_msgs/msg/detail/transform__functions.h"
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__functions.h"
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory_point__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment4_d__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__functions.h"
#include "geometry_msgs/msg/detail/twist__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Duration__EXPECTED_HASH = {1, {
    0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75,
    0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9, 0xca, 0x5b,
    0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14,
    0xc7, 0xf9, 0x37, 0x34, 0x95, 0x99, 0xd9, 0x3b,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__PoseStamped__EXPECTED_HASH = {1, {
    0x10, 0xf3, 0x78, 0x6d, 0x7d, 0x40, 0xfd, 0x2b,
    0x54, 0x36, 0x78, 0x35, 0x61, 0x4b, 0xff, 0x85,
    0xd4, 0xad, 0x3b, 0x5d, 0xab, 0x62, 0xbf, 0x8b,
    0xca, 0x0c, 0xc2, 0x32, 0xd7, 0x3b, 0x4c, 0xd8,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Transform__EXPECTED_HASH = {1, {
    0xbe, 0xb8, 0x3f, 0xbe, 0x69, 0x86, 0x36, 0x35,
    0x14, 0x61, 0xf6, 0xf3, 0x5d, 0x1a, 0xbb, 0x20,
    0x01, 0x0c, 0x43, 0xd5, 0x53, 0x74, 0xd8, 0x1b,
    0xd0, 0x41, 0xf1, 0xba, 0x25, 0x81, 0xfd, 0xdc,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Twist__EXPECTED_HASH = {1, {
    0x9c, 0x45, 0xbf, 0x16, 0xfe, 0x09, 0x83, 0xd8,
    0x0e, 0x3c, 0xfe, 0x75, 0x0d, 0x68, 0x35, 0x84,
    0x3d, 0x26, 0x5a, 0x9a, 0x6c, 0x46, 0xbd, 0x2e,
    0x60, 0x9f, 0xcd, 0xdd, 0xe6, 0xfb, 0x8d, 0x2a,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolynomialSegment__EXPECTED_HASH = {1, {
    0x51, 0x2d, 0xa6, 0x54, 0x36, 0x5a, 0x4e, 0x04,
    0x5f, 0x07, 0x6f, 0xf9, 0x8b, 0x61, 0x84, 0x30,
    0x3d, 0x5c, 0xe2, 0xe8, 0x5b, 0x13, 0x6c, 0x28,
    0x14, 0xae, 0x1d, 0xe1, 0x7a, 0x92, 0xcb, 0xfc,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolynomialSegment4D__EXPECTED_HASH = {1, {
    0x2f, 0x23, 0xc1, 0xe2, 0xdf, 0xb4, 0x61, 0x4d,
    0xe7, 0xd3, 0x5f, 0x3a, 0x97, 0x84, 0xe7, 0x35,
    0x5c, 0xfe, 0x4e, 0xf1, 0x45, 0x67, 0xe5, 0xd6,
    0x47, 0x4a, 0x0f, 0x0d, 0xec, 0xb8, 0xd7, 0xa6,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolynomialTrajectory__EXPECTED_HASH = {1, {
    0x43, 0x37, 0x6f, 0x41, 0x8f, 0xca, 0xbb, 0xa8,
    0x6d, 0xde, 0xe1, 0xdc, 0xaf, 0xe7, 0x05, 0xa0,
    0x1a, 0xc6, 0xf7, 0x5a, 0x94, 0x1f, 0x1b, 0xaa,
    0x28, 0x93, 0xd6, 0x87, 0x98, 0x86, 0xeb, 0xf9,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolynomialTrajectory4D__EXPECTED_HASH = {1, {
    0xfe, 0x02, 0x77, 0xde, 0xf4, 0x48, 0xcc, 0x3f,
    0x8e, 0x15, 0x22, 0xac, 0x64, 0xef, 0x94, 0x07,
    0xab, 0xd4, 0x07, 0xbc, 0xa9, 0xf5, 0xde, 0x09,
    0x23, 0x9b, 0x2f, 0xa7, 0x65, 0xfe, 0xc3, 0x14,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t trajectory_msgs__msg__MultiDOFJointTrajectory__EXPECTED_HASH = {1, {
    0x3a, 0x18, 0xfd, 0x09, 0x52, 0x92, 0xa6, 0x5c,
    0xfd, 0xe8, 0x83, 0x3c, 0x72, 0x98, 0x5a, 0x30,
    0xaf, 0x98, 0x1f, 0x3e, 0xc4, 0x44, 0x94, 0x65,
    0x5c, 0x62, 0x67, 0x26, 0x2b, 0x44, 0x3a, 0x4a,
  }};
static const rosidl_type_hash_t trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__EXPECTED_HASH = {1, {
    0x6a, 0xda, 0x10, 0x85, 0xb5, 0xee, 0x64, 0xea,
    0xa0, 0x69, 0xb0, 0x74, 0x96, 0x8e, 0x69, 0xf0,
    0xe2, 0x7c, 0x8c, 0x5e, 0x6f, 0x5b, 0xb0, 0x58,
    0x6d, 0xd1, 0xc8, 0x34, 0xef, 0x0e, 0x32, 0xb8,
  }};
#endif

static char mav_planning_msgs__srv__PlannerService__TYPE_NAME[] = "mav_planning_msgs/srv/PlannerService";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__PoseStamped__TYPE_NAME[] = "geometry_msgs/msg/PoseStamped";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char geometry_msgs__msg__Transform__TYPE_NAME[] = "geometry_msgs/msg/Transform";
static char geometry_msgs__msg__Twist__TYPE_NAME[] = "geometry_msgs/msg/Twist";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME[] = "mav_planning_msgs/msg/PolynomialSegment";
static char mav_planning_msgs__msg__PolynomialSegment4D__TYPE_NAME[] = "mav_planning_msgs/msg/PolynomialSegment4D";
static char mav_planning_msgs__msg__PolynomialTrajectory__TYPE_NAME[] = "mav_planning_msgs/msg/PolynomialTrajectory";
static char mav_planning_msgs__msg__PolynomialTrajectory4D__TYPE_NAME[] = "mav_planning_msgs/msg/PolynomialTrajectory4D";
static char mav_planning_msgs__srv__PlannerService_Event__TYPE_NAME[] = "mav_planning_msgs/srv/PlannerService_Event";
static char mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME[] = "mav_planning_msgs/srv/PlannerService_Request";
static char mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME[] = "mav_planning_msgs/srv/PlannerService_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char trajectory_msgs__msg__MultiDOFJointTrajectory__TYPE_NAME[] = "trajectory_msgs/msg/MultiDOFJointTrajectory";
static char trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__TYPE_NAME[] = "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint";

// Define type names, field names, and default values
static char mav_planning_msgs__srv__PlannerService__FIELD_NAME__request_message[] = "request_message";
static char mav_planning_msgs__srv__PlannerService__FIELD_NAME__response_message[] = "response_message";
static char mav_planning_msgs__srv__PlannerService__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PlannerService__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PlannerService__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PlannerService_Event__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PlannerService__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Transform__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment4D__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory4D__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Event__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectory__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PlannerService__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PlannerService__TYPE_NAME, 36, 36},
      {mav_planning_msgs__srv__PlannerService__FIELDS, 3, 3},
    },
    {mav_planning_msgs__srv__PlannerService__REFERENCED_TYPE_DESCRIPTIONS, 20, 20},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseStamped__EXPECTED_HASH, geometry_msgs__msg__PoseStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__PoseStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Transform__EXPECTED_HASH, geometry_msgs__msg__Transform__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Transform__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = mav_planning_msgs__msg__PolynomialSegment__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = mav_planning_msgs__msg__PolynomialSegment4D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = mav_planning_msgs__msg__PolynomialTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[13].fields = mav_planning_msgs__srv__PlannerService_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[14].fields = mav_planning_msgs__srv__PlannerService_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[15].fields = mav_planning_msgs__srv__PlannerService_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[16].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[17].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectory__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[18].fields = trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[19].fields = trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__start_pose[] = "start_pose";
static char mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__start_velocity[] = "start_velocity";
static char mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__goal_pose[] = "goal_pose";
static char mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__goal_velocity[] = "goal_velocity";
static char mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__bounding_box[] = "bounding_box";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PlannerService_Request__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__start_pose, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__start_velocity, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__goal_pose, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__goal_velocity, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__FIELD_NAME__bounding_box, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PlannerService_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PlannerService_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
      {mav_planning_msgs__srv__PlannerService_Request__FIELDS, 5, 5},
    },
    {mav_planning_msgs__srv__PlannerService_Request__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseStamped__EXPECTED_HASH, geometry_msgs__msg__PoseStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__PoseStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__success[] = "success";
static char mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__polynomial_plan[] = "polynomial_plan";
static char mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__polynomial_plan_4d[] = "polynomial_plan_4d";
static char mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__sampled_plan[] = "sampled_plan";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PlannerService_Response__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__polynomial_plan, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__msg__PolynomialTrajectory__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__polynomial_plan_4d, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__msg__PolynomialTrajectory4D__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Response__FIELD_NAME__sampled_plan, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {trajectory_msgs__msg__MultiDOFJointTrajectory__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PlannerService_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Transform__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment4D__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory4D__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectory__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PlannerService_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
      {mav_planning_msgs__srv__PlannerService_Response__FIELDS, 4, 4},
    },
    {mav_planning_msgs__srv__PlannerService_Response__REFERENCED_TYPE_DESCRIPTIONS, 13, 13},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Transform__EXPECTED_HASH, geometry_msgs__msg__Transform__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Transform__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = mav_planning_msgs__msg__PolynomialSegment__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = mav_planning_msgs__msg__PolynomialSegment4D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = mav_planning_msgs__msg__PolynomialTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectory__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__info[] = "info";
static char mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__request[] = "request";
static char mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PlannerService_Event__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PlannerService_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__PoseStamped__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Transform__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Twist__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment4D__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialTrajectory4D__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectory__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PlannerService_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PlannerService_Event__TYPE_NAME, 42, 42},
      {mav_planning_msgs__srv__PlannerService_Event__FIELDS, 3, 3},
    },
    {mav_planning_msgs__srv__PlannerService_Event__REFERENCED_TYPE_DESCRIPTIONS, 19, 19},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__PoseStamped__EXPECTED_HASH, geometry_msgs__msg__PoseStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__PoseStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Transform__EXPECTED_HASH, geometry_msgs__msg__Transform__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Transform__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Twist__EXPECTED_HASH, geometry_msgs__msg__Twist__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Twist__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = mav_planning_msgs__msg__PolynomialSegment__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialSegment4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialSegment4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = mav_planning_msgs__msg__PolynomialSegment4D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = mav_planning_msgs__msg__PolynomialTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolynomialTrajectory4D__EXPECTED_HASH, mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = mav_planning_msgs__msg__PolynomialTrajectory4D__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[13].fields = mav_planning_msgs__srv__PlannerService_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[14].fields = mav_planning_msgs__srv__PlannerService_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[15].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[16].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectory__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[17].fields = trajectory_msgs__msg__MultiDOFJointTrajectory__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__EXPECTED_HASH, trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[18].fields = trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#request fields\n"
  "geometry_msgs/PoseStamped start_pose #start pose for the planner\n"
  "geometry_msgs/Vector3 start_velocity\n"
  "geometry_msgs/PoseStamped goal_pose #start pose for the planner\n"
  "geometry_msgs/Vector3 goal_velocity\n"
  "geometry_msgs/Vector3 bounding_box\n"
  "---\n"
  "# True on success, false on planning failure\n"
  "bool success\n"
  "# Either contains a polynomial trajectory:\n"
  "mav_planning_msgs/PolynomialTrajectory polynomial_plan\n"
  "mav_planning_msgs/PolynomialTrajectory4D polynomial_plan_4d\n"
  "# or a MultiDOFJointTrajectory containing a sampled path (or straight-line\n"
  "# waypoints, depending on the planner).\n"
  "# Only one of these should be non-empty.\n"
  "trajectory_msgs/MultiDOFJointTrajectory sampled_plan";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PlannerService__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PlannerService__TYPE_NAME, 36, 36},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 682, 682},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PlannerService_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PlannerService_Request__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PlannerService_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PlannerService_Response__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PlannerService_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PlannerService_Event__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PlannerService__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[21];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 21, 21};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PlannerService__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__PoseStamped__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Transform__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[9] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[10] = *mav_planning_msgs__msg__PolynomialSegment__get_individual_type_description_source(NULL);
    sources[11] = *mav_planning_msgs__msg__PolynomialSegment4D__get_individual_type_description_source(NULL);
    sources[12] = *mav_planning_msgs__msg__PolynomialTrajectory__get_individual_type_description_source(NULL);
    sources[13] = *mav_planning_msgs__msg__PolynomialTrajectory4D__get_individual_type_description_source(NULL);
    sources[14] = *mav_planning_msgs__srv__PlannerService_Event__get_individual_type_description_source(NULL);
    sources[15] = *mav_planning_msgs__srv__PlannerService_Request__get_individual_type_description_source(NULL);
    sources[16] = *mav_planning_msgs__srv__PlannerService_Response__get_individual_type_description_source(NULL);
    sources[17] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[18] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[19] = *trajectory_msgs__msg__MultiDOFJointTrajectory__get_individual_type_description_source(NULL);
    sources[20] = *trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PlannerService_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PlannerService_Request__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__PoseStamped__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PlannerService_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[14];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 14, 14};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PlannerService_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Transform__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[7] = *mav_planning_msgs__msg__PolynomialSegment__get_individual_type_description_source(NULL);
    sources[8] = *mav_planning_msgs__msg__PolynomialSegment4D__get_individual_type_description_source(NULL);
    sources[9] = *mav_planning_msgs__msg__PolynomialTrajectory__get_individual_type_description_source(NULL);
    sources[10] = *mav_planning_msgs__msg__PolynomialTrajectory4D__get_individual_type_description_source(NULL);
    sources[11] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[12] = *trajectory_msgs__msg__MultiDOFJointTrajectory__get_individual_type_description_source(NULL);
    sources[13] = *trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PlannerService_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[20];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 20, 20};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PlannerService_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__PoseStamped__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Transform__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Twist__get_individual_type_description_source(NULL);
    sources[9] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[10] = *mav_planning_msgs__msg__PolynomialSegment__get_individual_type_description_source(NULL);
    sources[11] = *mav_planning_msgs__msg__PolynomialSegment4D__get_individual_type_description_source(NULL);
    sources[12] = *mav_planning_msgs__msg__PolynomialTrajectory__get_individual_type_description_source(NULL);
    sources[13] = *mav_planning_msgs__msg__PolynomialTrajectory4D__get_individual_type_description_source(NULL);
    sources[14] = *mav_planning_msgs__srv__PlannerService_Request__get_individual_type_description_source(NULL);
    sources[15] = *mav_planning_msgs__srv__PlannerService_Response__get_individual_type_description_source(NULL);
    sources[16] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[17] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[18] = *trajectory_msgs__msg__MultiDOFJointTrajectory__get_individual_type_description_source(NULL);
    sources[19] = *trajectory_msgs__msg__MultiDOFJointTrajectoryPoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
