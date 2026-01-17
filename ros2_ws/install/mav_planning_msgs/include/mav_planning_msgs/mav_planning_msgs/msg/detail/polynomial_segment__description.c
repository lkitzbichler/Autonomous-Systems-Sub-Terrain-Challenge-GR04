// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice

#include "mav_planning_msgs/msg/detail/polynomial_segment__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__msg__PolynomialSegment__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0x2d, 0xa6, 0x54, 0x36, 0x5a, 0x4e, 0x04,
      0x5f, 0x07, 0x6f, 0xf9, 0x8b, 0x61, 0x84, 0x30,
      0x3d, 0x5c, 0xe2, 0xe8, 0x5b, 0x13, 0x6c, 0x28,
      0x14, 0xae, 0x1d, 0xe1, 0x7a, 0x92, 0xcb, 0xfc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME[] = "mav_planning_msgs/msg/PolynomialSegment";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__header[] = "header";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__num_coeffs[] = "num_coeffs";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__segment_time[] = "segment_time";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__x[] = "x";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__y[] = "y";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__z[] = "z";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__rx[] = "rx";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__ry[] = "ry";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__rz[] = "rz";
static char mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__msg__PolynomialSegment__FIELDS[] = {
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__num_coeffs, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__segment_time, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__rx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__ry, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__rz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolynomialSegment__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__msg__PolynomialSegment__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__msg__PolynomialSegment__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME, 39, 39},
      {mav_planning_msgs__msg__PolynomialSegment__FIELDS, 10, 10},
    },
    {mav_planning_msgs__msg__PolynomialSegment__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "int32 num_coeffs        # order of the polynomial + 1, should match size of x[]\n"
  "builtin_interfaces/Duration segment_time   # duration of the segment\n"
  "float64[] x             # coefficients for the x-axis, INCREASING order\n"
  "float64[] y             # coefficients for the y-axis, INCREASING order\n"
  "float64[] z             # coefficients for the z-axis, INCREASING order\n"
  "float64[] rx            # coefficients for the rotation x-vector, INCREASING order\n"
  "float64[] ry            # coefficients for the rotation y-vector, INCREASING order\n"
  "float64[] rz            # coefficients for the rotation z-vector, INCREASING order\n"
  "## For backwards compatibility with underactuated (4DOF) commands):\n"
  "float64[] yaw           # coefficients for the yaw, INCREASING order \n"
  "                        ";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__msg__PolynomialSegment__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__msg__PolynomialSegment__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 800, 800},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__msg__PolynomialSegment__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__msg__PolynomialSegment__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
