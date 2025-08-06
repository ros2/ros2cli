// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__rosidl_typesupport_introspection_c.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"


// Include directives for member types
// Member `short_varied_nested`
#include "ros2cli_test_interfaces/msg/short_varied_nested.h"
// Member `short_varied_nested`
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__init(message_memory);
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_fini_function(void * message_memory)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_member_array[1] = {
  {
    "short_varied_nested",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request, short_varied_nested),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_members = {
  "ros2cli_test_interfaces__srv",  // message namespace
  "ShortVariedMultiNested_Request",  // message name
  1,  // number of fields
  sizeof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request),
  false,  // has_any_key_member_
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_member_array,  // message members
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle = {
  0,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)() {
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, msg, ShortVariedNested)();
  if (!ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__init(message_memory);
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_fini_function(void * message_memory)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_member_array[1] = {
  {
    "bool_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response, bool_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_members = {
  "ros2cli_test_interfaces__srv",  // message namespace
  "ShortVariedMultiNested_Response",  // message name
  1,  // number of fields
  sizeof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response),
  false,  // has_any_key_member_
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_member_array,  // message members
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle = {
  0,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)() {
  if (!ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "ros2cli_test_interfaces/srv/short_varied_multi_nested.h"
// Member `request`
// Member `response`
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__init(message_memory);
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_fini_function(void * message_memory)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__fini(message_memory);
}

size_t ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__size_function__ShortVariedMultiNested_Event__request(
  const void * untyped_member)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence * member =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__request(
  const void * untyped_member, size_t index)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence * member =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__request(
  void * untyped_member, size_t index)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence * member =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__fetch_function__ShortVariedMultiNested_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * item =
    ((const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request *)
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__request(untyped_member, index));
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * value =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request *)(untyped_value);
  *value = *item;
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__assign_function__ShortVariedMultiNested_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * item =
    ((ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request *)
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__request(untyped_member, index));
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request * value =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request *)(untyped_value);
  *item = *value;
}

bool ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__resize_function__ShortVariedMultiNested_Event__request(
  void * untyped_member, size_t size)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence * member =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence *)(untyped_member);
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence__fini(member);
  return ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__Sequence__init(member, size);
}

size_t ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__size_function__ShortVariedMultiNested_Event__response(
  const void * untyped_member)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence * member =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__response(
  const void * untyped_member, size_t index)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence * member =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__response(
  void * untyped_member, size_t index)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence * member =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__fetch_function__ShortVariedMultiNested_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * item =
    ((const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response *)
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__response(untyped_member, index));
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * value =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response *)(untyped_value);
  *value = *item;
}

void ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__assign_function__ShortVariedMultiNested_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * item =
    ((ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response *)
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__response(untyped_member, index));
  const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response * value =
    (const ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response *)(untyped_value);
  *item = *value;
}

bool ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__resize_function__ShortVariedMultiNested_Event__response(
  void * untyped_member, size_t size)
{
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence * member =
    (ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence *)(untyped_member);
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence__fini(member);
  return ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event, request),  // bytes offset in struct
    NULL,  // default value
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__size_function__ShortVariedMultiNested_Event__request,  // size() function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__request,  // get_const(index) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__request,  // get(index) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__fetch_function__ShortVariedMultiNested_Event__request,  // fetch(index, &value) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__assign_function__ShortVariedMultiNested_Event__request,  // assign(index, value) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__resize_function__ShortVariedMultiNested_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event, response),  // bytes offset in struct
    NULL,  // default value
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__size_function__ShortVariedMultiNested_Event__response,  // size() function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_const_function__ShortVariedMultiNested_Event__response,  // get_const(index) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__get_function__ShortVariedMultiNested_Event__response,  // get(index) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__fetch_function__ShortVariedMultiNested_Event__response,  // fetch(index, &value) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__assign_function__ShortVariedMultiNested_Event__response,  // assign(index, value) function pointer
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__resize_function__ShortVariedMultiNested_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_members = {
  "ros2cli_test_interfaces__srv",  // message namespace
  "ShortVariedMultiNested_Event",  // message name
  3,  // number of fields
  sizeof(ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event),
  false,  // has_any_key_member_
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_member_array,  // message members
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_type_support_handle = {
  0,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)() {
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)();
  ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)();
  if (!ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_members = {
  "ros2cli_test_interfaces__srv",  // service namespace
  "ShortVariedMultiNested",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle,
  NULL,  // response message
  // ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle
  NULL  // event_message
  // ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle
};


static rosidl_service_type_support_t ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_type_support_handle = {
  0,
  &ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_members,
  get_service_typesupport_handle_function,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Request_message_type_support_handle,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Response_message_type_support_handle,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event__rosidl_typesupport_introspection_c__ShortVariedMultiNested_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    srv,
    ShortVariedMultiNested
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    srv,
    ShortVariedMultiNested
  ),
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_hash,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description,
  &ros2cli_test_interfaces__srv__ShortVariedMultiNested__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested)(void) {
  if (!ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, srv, ShortVariedMultiNested_Event)()->data;
  }

  return &ros2cli_test_interfaces__srv__detail__short_varied_multi_nested__rosidl_typesupport_introspection_c__ShortVariedMultiNested_service_type_support_handle;
}
