// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ros2cli_test_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ShortVariedMultiNested_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ShortVariedMultiNested_Request_type_support_ids_t;

static const _ShortVariedMultiNested_Request_type_support_ids_t _ShortVariedMultiNested_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ShortVariedMultiNested_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ShortVariedMultiNested_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ShortVariedMultiNested_Request_type_support_symbol_names_t _ShortVariedMultiNested_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)),
  }
};

typedef struct _ShortVariedMultiNested_Request_type_support_data_t
{
  void * data[2];
} _ShortVariedMultiNested_Request_type_support_data_t;

static _ShortVariedMultiNested_Request_type_support_data_t _ShortVariedMultiNested_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ShortVariedMultiNested_Request_message_typesupport_map = {
  2,
  "ros2cli_test_interfaces",
  &_ShortVariedMultiNested_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ShortVariedMultiNested_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ShortVariedMultiNested_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ShortVariedMultiNested_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ShortVariedMultiNested_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>()
{
  return &::ros2cli_test_interfaces::srv::rosidl_typesupport_cpp::ShortVariedMultiNested_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)() {
  return get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros2cli_test_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ShortVariedMultiNested_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ShortVariedMultiNested_Response_type_support_ids_t;

static const _ShortVariedMultiNested_Response_type_support_ids_t _ShortVariedMultiNested_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ShortVariedMultiNested_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ShortVariedMultiNested_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ShortVariedMultiNested_Response_type_support_symbol_names_t _ShortVariedMultiNested_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)),
  }
};

typedef struct _ShortVariedMultiNested_Response_type_support_data_t
{
  void * data[2];
} _ShortVariedMultiNested_Response_type_support_data_t;

static _ShortVariedMultiNested_Response_type_support_data_t _ShortVariedMultiNested_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ShortVariedMultiNested_Response_message_typesupport_map = {
  2,
  "ros2cli_test_interfaces",
  &_ShortVariedMultiNested_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ShortVariedMultiNested_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ShortVariedMultiNested_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ShortVariedMultiNested_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ShortVariedMultiNested_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>()
{
  return &::ros2cli_test_interfaces::srv::rosidl_typesupport_cpp::ShortVariedMultiNested_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)() {
  return get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros2cli_test_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ShortVariedMultiNested_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ShortVariedMultiNested_Event_type_support_ids_t;

static const _ShortVariedMultiNested_Event_type_support_ids_t _ShortVariedMultiNested_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ShortVariedMultiNested_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ShortVariedMultiNested_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ShortVariedMultiNested_Event_type_support_symbol_names_t _ShortVariedMultiNested_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)),
  }
};

typedef struct _ShortVariedMultiNested_Event_type_support_data_t
{
  void * data[2];
} _ShortVariedMultiNested_Event_type_support_data_t;

static _ShortVariedMultiNested_Event_type_support_data_t _ShortVariedMultiNested_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ShortVariedMultiNested_Event_message_typesupport_map = {
  2,
  "ros2cli_test_interfaces",
  &_ShortVariedMultiNested_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ShortVariedMultiNested_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ShortVariedMultiNested_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ShortVariedMultiNested_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ShortVariedMultiNested_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>()
{
  return &::ros2cli_test_interfaces::srv::rosidl_typesupport_cpp::ShortVariedMultiNested_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)() {
  return get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ros2cli_test_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _ShortVariedMultiNested_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ShortVariedMultiNested_type_support_ids_t;

static const _ShortVariedMultiNested_type_support_ids_t _ShortVariedMultiNested_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ShortVariedMultiNested_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ShortVariedMultiNested_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ShortVariedMultiNested_type_support_symbol_names_t _ShortVariedMultiNested_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested)),
  }
};

typedef struct _ShortVariedMultiNested_type_support_data_t
{
  void * data[2];
} _ShortVariedMultiNested_type_support_data_t;

static _ShortVariedMultiNested_type_support_data_t _ShortVariedMultiNested_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ShortVariedMultiNested_service_typesupport_map = {
  2,
  "ros2cli_test_interfaces",
  &_ShortVariedMultiNested_service_typesupport_ids.typesupport_identifier[0],
  &_ShortVariedMultiNested_service_typesupport_symbol_names.symbol_name[0],
  &_ShortVariedMultiNested_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ShortVariedMultiNested_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ShortVariedMultiNested_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<ros2cli_test_interfaces::srv::ShortVariedMultiNested>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<ros2cli_test_interfaces::srv::ShortVariedMultiNested>,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested>()
{
  return &::ros2cli_test_interfaces::srv::rosidl_typesupport_cpp::ShortVariedMultiNested_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, ros2cli_test_interfaces, srv, ShortVariedMultiNested)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<ros2cli_test_interfaces::srv::ShortVariedMultiNested>();
}

#ifdef __cplusplus
}
#endif
