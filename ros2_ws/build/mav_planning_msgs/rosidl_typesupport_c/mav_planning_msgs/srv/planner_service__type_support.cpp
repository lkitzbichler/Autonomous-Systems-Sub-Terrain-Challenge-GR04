// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mav_planning_msgs/srv/detail/planner_service__struct.h"
#include "mav_planning_msgs/srv/detail/planner_service__type_support.h"
#include "mav_planning_msgs/srv/detail/planner_service__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PlannerService_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlannerService_Request_type_support_ids_t;

static const _PlannerService_Request_type_support_ids_t _PlannerService_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlannerService_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlannerService_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlannerService_Request_type_support_symbol_names_t _PlannerService_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Request)),
  }
};

typedef struct _PlannerService_Request_type_support_data_t
{
  void * data[2];
} _PlannerService_Request_type_support_data_t;

static _PlannerService_Request_type_support_data_t _PlannerService_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlannerService_Request_message_typesupport_map = {
  2,
  "mav_planning_msgs",
  &_PlannerService_Request_message_typesupport_ids.typesupport_identifier[0],
  &_PlannerService_Request_message_typesupport_symbol_names.symbol_name[0],
  &_PlannerService_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlannerService_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlannerService_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mav_planning_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mav_planning_msgs, srv, PlannerService_Request)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_c::PlannerService_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__type_support.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PlannerService_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlannerService_Response_type_support_ids_t;

static const _PlannerService_Response_type_support_ids_t _PlannerService_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlannerService_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlannerService_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlannerService_Response_type_support_symbol_names_t _PlannerService_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Response)),
  }
};

typedef struct _PlannerService_Response_type_support_data_t
{
  void * data[2];
} _PlannerService_Response_type_support_data_t;

static _PlannerService_Response_type_support_data_t _PlannerService_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlannerService_Response_message_typesupport_map = {
  2,
  "mav_planning_msgs",
  &_PlannerService_Response_message_typesupport_ids.typesupport_identifier[0],
  &_PlannerService_Response_message_typesupport_symbol_names.symbol_name[0],
  &_PlannerService_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlannerService_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlannerService_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mav_planning_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mav_planning_msgs, srv, PlannerService_Response)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_c::PlannerService_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__type_support.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _PlannerService_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlannerService_Event_type_support_ids_t;

static const _PlannerService_Event_type_support_ids_t _PlannerService_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlannerService_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlannerService_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlannerService_Event_type_support_symbol_names_t _PlannerService_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService_Event)),
  }
};

typedef struct _PlannerService_Event_type_support_data_t
{
  void * data[2];
} _PlannerService_Event_type_support_data_t;

static _PlannerService_Event_type_support_data_t _PlannerService_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlannerService_Event_message_typesupport_map = {
  2,
  "mav_planning_msgs",
  &_PlannerService_Event_message_typesupport_ids.typesupport_identifier[0],
  &_PlannerService_Event_message_typesupport_symbol_names.symbol_name[0],
  &_PlannerService_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t PlannerService_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlannerService_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mav_planning_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mav_planning_msgs, srv, PlannerService_Event)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_c::PlannerService_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _PlannerService_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _PlannerService_type_support_ids_t;

static const _PlannerService_type_support_ids_t _PlannerService_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _PlannerService_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _PlannerService_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _PlannerService_type_support_symbol_names_t _PlannerService_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, srv, PlannerService)),
  }
};

typedef struct _PlannerService_type_support_data_t
{
  void * data[2];
} _PlannerService_type_support_data_t;

static _PlannerService_type_support_data_t _PlannerService_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _PlannerService_service_typesupport_map = {
  2,
  "mav_planning_msgs",
  &_PlannerService_service_typesupport_ids.typesupport_identifier[0],
  &_PlannerService_service_typesupport_symbol_names.symbol_name[0],
  &_PlannerService_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t PlannerService_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_PlannerService_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &PlannerService_Request_message_type_support_handle,
  &PlannerService_Response_message_type_support_handle,
  &PlannerService_Event_message_type_support_handle,
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

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mav_planning_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, mav_planning_msgs, srv, PlannerService)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_c::PlannerService_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
