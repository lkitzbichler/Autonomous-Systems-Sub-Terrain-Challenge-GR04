// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_msgs:msg/RollPitchYawrateThrust.idl
// generated code does not contain a copyright notice

#include "mav_msgs/msg/detail/roll_pitch_yawrate_thrust__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_type_hash_t *
mav_msgs__msg__RollPitchYawrateThrust__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1f, 0x64, 0x41, 0xc1, 0x6d, 0x03, 0x7a, 0xa1,
      0xc2, 0xf0, 0x8a, 0x01, 0x72, 0xf2, 0x23, 0x73,
      0x80, 0xe0, 0xed, 0x40, 0x75, 0x1f, 0xce, 0xa2,
      0x1a, 0xdc, 0xad, 0xe9, 0x3d, 0x90, 0x39, 0x84,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "geometry_msgs/msg/detail/vector3__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mav_msgs__msg__RollPitchYawrateThrust__TYPE_NAME[] = "mav_msgs/msg/RollPitchYawrateThrust";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__header[] = "header";
static char mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__roll[] = "roll";
static char mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__pitch[] = "pitch";
static char mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__yaw_rate[] = "yaw_rate";
static char mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__thrust[] = "thrust";

static rosidl_runtime_c__type_description__Field mav_msgs__msg__RollPitchYawrateThrust__FIELDS[] = {
  {
    {mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__yaw_rate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__RollPitchYawrateThrust__FIELD_NAME__thrust, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_msgs__msg__RollPitchYawrateThrust__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
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
mav_msgs__msg__RollPitchYawrateThrust__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_msgs__msg__RollPitchYawrateThrust__TYPE_NAME, 35, 35},
      {mav_msgs__msg__RollPitchYawrateThrust__FIELDS, 5, 5},
    },
    {mav_msgs__msg__RollPitchYawrateThrust__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# We use the coordinate frames with the following convention:\n"
  "#   x: forward\n"
  "#   y: left\n"
  "#   z: up\n"
  "\n"
  "# rotation convention (z-y'-x''):\n"
  "# yaw rotates around fixed frame's z axis\n"
  "# pitch rotates around new y-axis (y')\n"
  "# roll rotates around new x-axis (x'')\n"
  "\n"
  "# This is a convenience-message to support that low-level (microcontroller-based) state\n"
  "# estimators may not have knowledge about the absolute yaw.\n"
  "# Roll- and pitch-angle should be specified in the header/frame_id frame\n"
  "float64 roll                   # Roll angle [rad]\n"
  "float64 pitch                  # Pitch angle  [rad]\n"
  "float64 yaw_rate               # Yaw rate around z-axis [rad/s]\n"
  "\n"
  "geometry_msgs/Vector3 thrust   # Thrust [N] expressed in the body frame.\n"
  "                               # For a fixed-wing, usually the x-component is used.\n"
  "                               # For a multi-rotor, usually the z-component is used.\n"
  "                               # Set all un-used components to 0.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_msgs__msg__RollPitchYawrateThrust__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_msgs__msg__RollPitchYawrateThrust__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 975, 975},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_msgs__msg__RollPitchYawrateThrust__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_msgs__msg__RollPitchYawrateThrust__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
