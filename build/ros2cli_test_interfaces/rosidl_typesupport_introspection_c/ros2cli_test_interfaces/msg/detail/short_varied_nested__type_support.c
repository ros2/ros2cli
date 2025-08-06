// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__rosidl_typesupport_introspection_c.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.h"


// Include directives for member types
// Member `short_varied`
#include "ros2cli_test_interfaces/msg/short_varied.h"
// Member `short_varied`
#include "ros2cli_test_interfaces/msg/detail/short_varied__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2cli_test_interfaces__msg__ShortVariedNested__init(message_memory);
}

void ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_fini_function(void * message_memory)
{
  ros2cli_test_interfaces__msg__ShortVariedNested__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_member_array[1] = {
  {
    "short_varied",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces__msg__ShortVariedNested, short_varied),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_members = {
  "ros2cli_test_interfaces__msg",  // message namespace
  "ShortVariedNested",  // message name
  1,  // number of fields
  sizeof(ros2cli_test_interfaces__msg__ShortVariedNested),
  false,  // has_any_key_member_
  ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_member_array,  // message members
  ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_type_support_handle = {
  0,
  &ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, msg, ShortVariedNested)() {
  ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, msg, ShortVaried)();
  if (!ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2cli_test_interfaces__msg__ShortVariedNested__rosidl_typesupport_introspection_c__ShortVariedNested_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
