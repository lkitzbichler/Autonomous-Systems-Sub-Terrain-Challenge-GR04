// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_planning_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice

#include "mav_planning_msgs/msg/detail/point2_d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__msg__Point2D__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4e, 0x74, 0xe6, 0xd4, 0x8f, 0x43, 0xe8, 0x8a,
      0x5d, 0xb6, 0x83, 0xb9, 0xd3, 0x72, 0xaf, 0x09,
      0xa6, 0xbb, 0x33, 0x1d, 0xd0, 0x54, 0xdc, 0xc0,
      0x6b, 0x7f, 0x23, 0xbf, 0x3d, 0x03, 0x99, 0x0a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mav_planning_msgs__msg__Point2D__TYPE_NAME[] = "mav_planning_msgs/msg/Point2D";

// Define type names, field names, and default values
static char mav_planning_msgs__msg__Point2D__FIELD_NAME__x[] = "x";
static char mav_planning_msgs__msg__Point2D__FIELD_NAME__y[] = "y";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__msg__Point2D__FIELDS[] = {
  {
    {mav_planning_msgs__msg__Point2D__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Point2D__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__msg__Point2D__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
      {mav_planning_msgs__msg__Point2D__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# This contains the position of a 2D point.\n"
  "float64 x\n"
  "float64 y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__msg__Point2D__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 64, 64},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__msg__Point2D__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__msg__Point2D__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
