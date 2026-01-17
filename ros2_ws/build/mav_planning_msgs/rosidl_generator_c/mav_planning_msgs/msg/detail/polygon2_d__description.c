// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

#include "mav_planning_msgs/msg/detail/polygon2_d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__msg__Polygon2D__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x24, 0x78, 0xb9, 0xe4, 0x31, 0x9a, 0x26, 0xf1,
      0xff, 0x77, 0x96, 0x75, 0xe7, 0x58, 0x28, 0xc4,
      0x4a, 0xbe, 0x80, 0xa1, 0x99, 0x77, 0x9e, 0xa7,
      0x93, 0x23, 0xca, 0x9b, 0x1b, 0x82, 0x23, 0x83,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "mav_planning_msgs/msg/detail/point2_d__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t mav_planning_msgs__msg__Point2D__EXPECTED_HASH = {1, {
    0x4e, 0x74, 0xe6, 0xd4, 0x8f, 0x43, 0xe8, 0x8a,
    0x5d, 0xb6, 0x83, 0xb9, 0xd3, 0x72, 0xaf, 0x09,
    0xa6, 0xbb, 0x33, 0x1d, 0xd0, 0x54, 0xdc, 0xc0,
    0x6b, 0x7f, 0x23, 0xbf, 0x3d, 0x03, 0x99, 0x0a,
  }};
#endif

static char mav_planning_msgs__msg__Polygon2D__TYPE_NAME[] = "mav_planning_msgs/msg/Polygon2D";
static char mav_planning_msgs__msg__Point2D__TYPE_NAME[] = "mav_planning_msgs/msg/Point2D";

// Define type names, field names, and default values
static char mav_planning_msgs__msg__Polygon2D__FIELD_NAME__points[] = "points";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__msg__Polygon2D__FIELDS[] = {
  {
    {mav_planning_msgs__msg__Polygon2D__FIELD_NAME__points, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__msg__Polygon2D__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__msg__Polygon2D__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__msg__Polygon2D__TYPE_NAME, 31, 31},
      {mav_planning_msgs__msg__Polygon2D__FIELDS, 1, 1},
    },
    {mav_planning_msgs__msg__Polygon2D__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&mav_planning_msgs__msg__Point2D__EXPECTED_HASH, mav_planning_msgs__msg__Point2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = mav_planning_msgs__msg__Point2D__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A specification of a 2D polygon where the first and last points are assumed to be connected.\n"
  "mav_planning_msgs/Point2D[] points";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__msg__Polygon2D__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__msg__Polygon2D__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 130, 130},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__msg__Polygon2D__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__msg__Polygon2D__get_individual_type_description_source(NULL),
    sources[1] = *mav_planning_msgs__msg__Point2D__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
