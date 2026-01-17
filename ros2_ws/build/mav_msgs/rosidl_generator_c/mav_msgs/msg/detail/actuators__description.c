// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

#include "mav_msgs/msg/detail/actuators__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_type_hash_t *
mav_msgs__msg__Actuators__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x22, 0xbc, 0x02, 0xa4, 0x7d, 0xb3, 0xe8, 0x92,
      0x18, 0xa4, 0x72, 0xf8, 0x4e, 0x23, 0x48, 0x28,
      0x6e, 0xc0, 0xf1, 0x23, 0x0a, 0x5d, 0x9f, 0xfc,
      0xca, 0x24, 0xf9, 0x7e, 0xe0, 0x79, 0x6b, 0x21,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
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

static char mav_msgs__msg__Actuators__TYPE_NAME[] = "mav_msgs/msg/Actuators";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mav_msgs__msg__Actuators__FIELD_NAME__header[] = "header";
static char mav_msgs__msg__Actuators__FIELD_NAME__angles[] = "angles";
static char mav_msgs__msg__Actuators__FIELD_NAME__angular_velocities[] = "angular_velocities";
static char mav_msgs__msg__Actuators__FIELD_NAME__normalized[] = "normalized";

static rosidl_runtime_c__type_description__Field mav_msgs__msg__Actuators__FIELDS[] = {
  {
    {mav_msgs__msg__Actuators__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Actuators__FIELD_NAME__angles, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Actuators__FIELD_NAME__angular_velocities, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Actuators__FIELD_NAME__normalized, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_msgs__msg__Actuators__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
mav_msgs__msg__Actuators__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_msgs__msg__Actuators__TYPE_NAME, 22, 22},
      {mav_msgs__msg__Actuators__FIELDS, 4, 4},
    },
    {mav_msgs__msg__Actuators__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# This message defines lowest level commands to be sent to the actuator(s). \n"
  "\n"
  "float64[] angles             # Angle of the actuator in [rad]. \n"
  "                             # E.g. servo angle of a control surface(not angle of the surface!), orientation-angle of a thruster.      \n"
  "float64[] angular_velocities # Angular velocities of the actuator in [rad/s].\n"
  "                             # E.g. \"rpm\" of rotors, propellers, thrusters \n"
  "float64[] normalized         # Everything that does not fit the above, normalized between [-1 ... 1].";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_msgs__msg__Actuators__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_msgs__msg__Actuators__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 557, 557},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_msgs__msg__Actuators__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_msgs__msg__Actuators__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
