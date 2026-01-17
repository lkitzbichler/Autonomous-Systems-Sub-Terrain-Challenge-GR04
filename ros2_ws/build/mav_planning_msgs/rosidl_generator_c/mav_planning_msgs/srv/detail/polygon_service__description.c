// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mav_planning_msgs:srv/PolygonService.idl
// generated code does not contain a copyright notice

#include "mav_planning_msgs/srv/detail/polygon_service__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PolygonService__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x06, 0x74, 0xae, 0x76, 0x50, 0x6b, 0x38, 0xd4,
      0xfa, 0x2e, 0xab, 0xcc, 0x1c, 0xba, 0x77, 0x7a,
      0x9c, 0x77, 0xa3, 0xb0, 0x7f, 0xff, 0xda, 0xed,
      0x88, 0x51, 0x25, 0xf0, 0x0f, 0xa4, 0xb3, 0x49,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PolygonService_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0xfb, 0x1a, 0xf2, 0x22, 0xfa, 0x93, 0x0a,
      0xa8, 0xe8, 0x59, 0x07, 0x52, 0xcb, 0x2f, 0x06,
      0x02, 0xa8, 0x74, 0x52, 0xb5, 0xa9, 0x9b, 0x12,
      0xcc, 0xf1, 0xde, 0x73, 0xc9, 0x42, 0x89, 0x94,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PolygonService_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x51, 0xf4, 0x17, 0x7a, 0x2c, 0xfa, 0xfe, 0x42,
      0x2a, 0x22, 0x09, 0x05, 0xc9, 0xb9, 0x73, 0x9b,
      0x60, 0x6d, 0x37, 0xd5, 0xaf, 0x27, 0x5f, 0x85,
      0xef, 0xc7, 0x82, 0x2c, 0x28, 0x30, 0x64, 0xc3,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__srv__PolygonService_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0xa1, 0x3b, 0xf8, 0x68, 0xd2, 0x24, 0x93,
      0xb7, 0xb2, 0x4f, 0x7a, 0x00, 0x54, 0x64, 0xa9,
      0x99, 0xa9, 0x48, 0x11, 0x9b, 0x58, 0x4f, 0x65,
      0xfc, 0xe5, 0xc2, 0x9a, 0x46, 0x81, 0x9a, 0x80,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__functions.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "mav_planning_msgs/msg/detail/polygon2_d__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "mav_planning_msgs/msg/detail/point2_d__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__Point2D__EXPECTED_HASH = {1, {
    0x4e, 0x74, 0xe6, 0xd4, 0x8f, 0x43, 0xe8, 0x8a,
    0x5d, 0xb6, 0x83, 0xb9, 0xd3, 0x72, 0xaf, 0x09,
    0xa6, 0xbb, 0x33, 0x1d, 0xd0, 0x54, 0xdc, 0xc0,
    0x6b, 0x7f, 0x23, 0xbf, 0x3d, 0x03, 0x99, 0x0a,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__Polygon2D__EXPECTED_HASH = {1, {
    0x24, 0x78, 0xb9, 0xe4, 0x31, 0x9a, 0x26, 0xf1,
    0xff, 0x77, 0x96, 0x75, 0xe7, 0x58, 0x28, 0xc4,
    0x4a, 0xbe, 0x80, 0xa1, 0x99, 0x77, 0x9e, 0xa7,
    0x93, 0x23, 0xca, 0x9b, 0x1b, 0x82, 0x23, 0x83,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolygonWithHoles__EXPECTED_HASH = {1, {
    0x35, 0xff, 0xbf, 0x99, 0xbe, 0xbd, 0x18, 0xbf,
    0x71, 0x85, 0x49, 0x2e, 0x98, 0x47, 0x82, 0xb7,
    0x5d, 0x8a, 0x1f, 0xc8, 0x06, 0xcb, 0x9b, 0x17,
    0x0f, 0x00, 0x54, 0x10, 0xf1, 0xe7, 0x32, 0x40,
  }};
static const rosidl_type_hash_t mav_planning_msgs__msg__PolygonWithHolesStamped__EXPECTED_HASH = {1, {
    0x61, 0x9c, 0x6b, 0x56, 0x19, 0x3e, 0x8b, 0xf2,
    0x66, 0xb5, 0xe1, 0xb8, 0x17, 0xce, 0x60, 0x55,
    0xfb, 0xc4, 0x45, 0x75, 0x2e, 0x74, 0xfe, 0x1c,
    0x83, 0x8f, 0xd4, 0xb1, 0x0f, 0x9c, 0xd4, 0x2f,
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
#endif

static char mav_planning_msgs__srv__PolygonService__TYPE_NAME[] = "mav_planning_msgs/srv/PolygonService";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mav_planning_msgs__msg__Point2D__TYPE_NAME[] = "mav_planning_msgs/msg/Point2D";
static char mav_planning_msgs__msg__Polygon2D__TYPE_NAME[] = "mav_planning_msgs/msg/Polygon2D";
static char mav_planning_msgs__msg__PolygonWithHoles__TYPE_NAME[] = "mav_planning_msgs/msg/PolygonWithHoles";
static char mav_planning_msgs__msg__PolygonWithHolesStamped__TYPE_NAME[] = "mav_planning_msgs/msg/PolygonWithHolesStamped";
static char mav_planning_msgs__srv__PolygonService_Event__TYPE_NAME[] = "mav_planning_msgs/srv/PolygonService_Event";
static char mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME[] = "mav_planning_msgs/srv/PolygonService_Request";
static char mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME[] = "mav_planning_msgs/srv/PolygonService_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mav_planning_msgs__srv__PolygonService__FIELD_NAME__request_message[] = "request_message";
static char mav_planning_msgs__srv__PolygonService__FIELD_NAME__response_message[] = "response_message";
static char mav_planning_msgs__srv__PolygonService__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PolygonService__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PolygonService__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__srv__PolygonService_Event__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PolygonService__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Polygon2D__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHoles__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHolesStamped__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Event__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
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
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PolygonService__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PolygonService__TYPE_NAME, 36, 36},
      {mav_planning_msgs__srv__PolygonService__FIELDS, 3, 3},
    },
    {mav_planning_msgs__srv__PolygonService__REFERENCED_TYPE_DESCRIPTIONS, 10, 10},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Point2D__EXPECTED_HASH, mav_planning_msgs__msg__Point2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mav_planning_msgs__msg__Point2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Polygon2D__EXPECTED_HASH, mav_planning_msgs__msg__Polygon2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mav_planning_msgs__msg__Polygon2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHoles__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHoles__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mav_planning_msgs__msg__PolygonWithHoles__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHolesStamped__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mav_planning_msgs__srv__PolygonService_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = mav_planning_msgs__srv__PolygonService_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[7].fields = mav_planning_msgs__srv__PolygonService_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PolygonService_Request__FIELD_NAME__polygon[] = "polygon";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PolygonService_Request__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PolygonService_Request__FIELD_NAME__polygon, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mav_planning_msgs__msg__PolygonWithHolesStamped__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PolygonService_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Polygon2D__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHoles__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHolesStamped__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PolygonService_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
      {mav_planning_msgs__srv__PolygonService_Request__FIELDS, 1, 1},
    },
    {mav_planning_msgs__srv__PolygonService_Request__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Point2D__EXPECTED_HASH, mav_planning_msgs__msg__Point2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mav_planning_msgs__msg__Point2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Polygon2D__EXPECTED_HASH, mav_planning_msgs__msg__Polygon2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mav_planning_msgs__msg__Polygon2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHoles__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHoles__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mav_planning_msgs__msg__PolygonWithHoles__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHolesStamped__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PolygonService_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PolygonService_Response__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PolygonService_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PolygonService_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
      {mav_planning_msgs__srv__PolygonService_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__info[] = "info";
static char mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__request[] = "request";
static char mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mav_planning_msgs__srv__PolygonService_Event__FIELDS[] = {
  {
    {mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
    },
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mav_planning_msgs__srv__PolygonService_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Point2D__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__Polygon2D__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHoles__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__msg__PolygonWithHolesStamped__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
    {NULL, 0, 0},
  },
  {
    {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
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
};

const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__srv__PolygonService_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mav_planning_msgs__srv__PolygonService_Event__TYPE_NAME, 42, 42},
      {mav_planning_msgs__srv__PolygonService_Event__FIELDS, 3, 3},
    },
    {mav_planning_msgs__srv__PolygonService_Event__REFERENCED_TYPE_DESCRIPTIONS, 9, 9},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Point2D__EXPECTED_HASH, mav_planning_msgs__msg__Point2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mav_planning_msgs__msg__Point2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__Polygon2D__EXPECTED_HASH, mav_planning_msgs__msg__Polygon2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mav_planning_msgs__msg__Polygon2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHoles__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHoles__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mav_planning_msgs__msg__PolygonWithHoles__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mav_planning_msgs__msg__PolygonWithHolesStamped__EXPECTED_HASH, mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mav_planning_msgs__srv__PolygonService_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = mav_planning_msgs__srv__PolygonService_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A service to set a new polygon with holes.\n"
  "# Request fields:\n"
  "mav_planning_msgs/PolygonWithHolesStamped polygon # The new polygon.\n"
  "---\n"
  "# Response fields:\n"
  "bool success # True on success, false on polygon error.";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PolygonService__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PolygonService__TYPE_NAME, 36, 36},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 211, 211},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PolygonService_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PolygonService_Request__TYPE_NAME, 44, 44},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PolygonService_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PolygonService_Response__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__srv__PolygonService_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mav_planning_msgs__srv__PolygonService_Event__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PolygonService__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[11];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 11, 11};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PolygonService__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mav_planning_msgs__msg__Point2D__get_individual_type_description_source(NULL);
    sources[3] = *mav_planning_msgs__msg__Polygon2D__get_individual_type_description_source(NULL);
    sources[4] = *mav_planning_msgs__msg__PolygonWithHoles__get_individual_type_description_source(NULL);
    sources[5] = *mav_planning_msgs__msg__PolygonWithHolesStamped__get_individual_type_description_source(NULL);
    sources[6] = *mav_planning_msgs__srv__PolygonService_Event__get_individual_type_description_source(NULL);
    sources[7] = *mav_planning_msgs__srv__PolygonService_Request__get_individual_type_description_source(NULL);
    sources[8] = *mav_planning_msgs__srv__PolygonService_Response__get_individual_type_description_source(NULL);
    sources[9] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[10] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PolygonService_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PolygonService_Request__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mav_planning_msgs__msg__Point2D__get_individual_type_description_source(NULL);
    sources[3] = *mav_planning_msgs__msg__Polygon2D__get_individual_type_description_source(NULL);
    sources[4] = *mav_planning_msgs__msg__PolygonWithHoles__get_individual_type_description_source(NULL);
    sources[5] = *mav_planning_msgs__msg__PolygonWithHolesStamped__get_individual_type_description_source(NULL);
    sources[6] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PolygonService_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PolygonService_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__srv__PolygonService_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[10];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 10, 10};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mav_planning_msgs__srv__PolygonService_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mav_planning_msgs__msg__Point2D__get_individual_type_description_source(NULL);
    sources[3] = *mav_planning_msgs__msg__Polygon2D__get_individual_type_description_source(NULL);
    sources[4] = *mav_planning_msgs__msg__PolygonWithHoles__get_individual_type_description_source(NULL);
    sources[5] = *mav_planning_msgs__msg__PolygonWithHolesStamped__get_individual_type_description_source(NULL);
    sources[6] = *mav_planning_msgs__srv__PolygonService_Request__get_individual_type_description_source(NULL);
    sources[7] = *mav_planning_msgs__srv__PolygonService_Response__get_individual_type_description_source(NULL);
    sources[8] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[9] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
