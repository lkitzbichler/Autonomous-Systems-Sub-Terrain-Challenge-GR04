// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice

#include "mav_msgs/msg/detail/status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_type_hash_t *
mav_msgs__msg__Status__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2f, 0x14, 0xd6, 0x7b, 0xa8, 0x8e, 0xf7, 0x0a,
      0x49, 0x9b, 0x55, 0x5c, 0x79, 0x27, 0x73, 0xe6,
      0xbe, 0x98, 0x31, 0x16, 0xa3, 0xbc, 0x7e, 0xd3,
      0x29, 0xbb, 0x59, 0xe5, 0xfb, 0x39, 0x71, 0x04,
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

static char mav_msgs__msg__Status__TYPE_NAME[] = "mav_msgs/msg/Status";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mav_msgs__msg__Status__FIELD_NAME__header[] = "header";
static char mav_msgs__msg__Status__FIELD_NAME__vehicle_name[] = "vehicle_name";
static char mav_msgs__msg__Status__FIELD_NAME__vehicle_type[] = "vehicle_type";
static char mav_msgs__msg__Status__FIELD_NAME__battery_voltage[] = "battery_voltage";
static char mav_msgs__msg__Status__FIELD_NAME__rc_command_mode[] = "rc_command_mode";
static char mav_msgs__msg__Status__FIELD_NAME__command_interface_enabled[] = "command_interface_enabled";
static char mav_msgs__msg__Status__FIELD_NAME__flight_time[] = "flight_time";
static char mav_msgs__msg__Status__FIELD_NAME__system_uptime[] = "system_uptime";
static char mav_msgs__msg__Status__FIELD_NAME__cpu_load[] = "cpu_load";
static char mav_msgs__msg__Status__FIELD_NAME__motor_status[] = "motor_status";
static char mav_msgs__msg__Status__FIELD_NAME__in_air[] = "in_air";
static char mav_msgs__msg__Status__FIELD_NAME__gps_status[] = "gps_status";
static char mav_msgs__msg__Status__FIELD_NAME__gps_num_satellites[] = "gps_num_satellites";

static rosidl_runtime_c__type_description__Field mav_msgs__msg__Status__FIELDS[] = {
  {
    {mav_msgs__msg__Status__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__vehicle_name, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__vehicle_type, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__battery_voltage, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__rc_command_mode, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__command_interface_enabled, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__flight_time, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__system_uptime, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__cpu_load, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__motor_status, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__in_air, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__gps_status, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mav_msgs__msg__Status__FIELD_NAME__gps_num_satellites, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_msgs__msg__Status__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
mav_msgs__msg__Status__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_msgs__msg__Status__TYPE_NAME, 19, 19},
      {mav_msgs__msg__Status__FIELDS, 13, 13},
    },
    {mav_msgs__msg__Status__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
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
  "# If values are not known / available, set to -1 or empty string.\n"
  "string      vehicle_name\n"
  "string      vehicle_type                  # E.g. firefly, pelican ...\n"
  "float32     battery_voltage               # Battery voltage in V.\n"
  "string      rc_command_mode               # Command mode set on the 3 position switch on the rc.\n"
  "bool        command_interface_enabled     # Reports whether the serial command interface is enabled.\n"
  "float32     flight_time                   # Flight time in s.\n"
  "float32     system_uptime\\t\\t\\t\\t\\t\\t\\t\\t\\t# MAV uptime in s.\n"
  "float32     cpu_load                      # MAV CPU load: 0.0 ... 1.0\n"
  "\n"
  "string      motor_status                  # Current motor status: running, stopped, starting, stopping.\n"
  "bool        in_air                        # True if vehicle is actually in air, false otherwise\n"
  "\n"
  "string      gps_status                    # GPS status: lock, no_lock\n"
  "int32       gps_num_satellites            # Number of visible satellites\n"
  "\n"
  "string RC_COMMAND_ATTITUDE=\"attitude_thrust\"\n"
  "string RC_COMMAND_ATTITUDE_HEIGHT=\"attitude_height\"\n"
  "string RC_COMMAND_POSITION=\"position\"\n"
  "\n"
  "string MOTOR_STATUS_RUNNING=\"running\"\n"
  "string MOTOR_STATUS_STOPPED=\"stopped\"\n"
  "string MOTOR_STATUS_STARTING=\"starting\"\n"
  "string MOTOR_STATUS_STOPPING=\"stopping\"\n"
  "\n"
  "string GPS_STATUS_LOCK=\"lock\"\n"
  "string GPS_STATUS_NO_LOCK=\"no_lock\"";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_msgs__msg__Status__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_msgs__msg__Status__TYPE_NAME, 19, 19},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1339, 1339},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_msgs__msg__Status__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_msgs__msg__Status__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
