// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2cli_test_interfaces/msg/detail/short_varied__rosidl_typesupport_introspection_c.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.h"


// Include directives for member types
// Member `bool_values`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2cli_test_interfaces__msg__ShortVaried__init(message_memory);
}

void ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_fini_function(void * message_memory)
{
  ros2cli_test_interfaces__msg__ShortVaried__fini(message_memory);
}

size_t ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__size_function__ShortVaried__bool_values(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_const_function__ShortVaried__bool_values(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_function__ShortVaried__bool_values(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__fetch_function__ShortVaried__bool_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_const_function__ShortVaried__bool_values(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__assign_function__ShortVaried__bool_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_function__ShortVaried__bool_values(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__resize_function__ShortVaried__bool_values(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_member_array[2] = {
  {
    "bool_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces__msg__ShortVaried, bool_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bool_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    true,  // is upper bound
    offsetof(ros2cli_test_interfaces__msg__ShortVaried, bool_values),  // bytes offset in struct
    NULL,  // default value
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__size_function__ShortVaried__bool_values,  // size() function pointer
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_const_function__ShortVaried__bool_values,  // get_const(index) function pointer
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__get_function__ShortVaried__bool_values,  // get(index) function pointer
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__fetch_function__ShortVaried__bool_values,  // fetch(index, &value) function pointer
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__assign_function__ShortVaried__bool_values,  // assign(index, value) function pointer
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__resize_function__ShortVaried__bool_values  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_members = {
  "ros2cli_test_interfaces__msg",  // message namespace
  "ShortVaried",  // message name
  2,  // number of fields
  sizeof(ros2cli_test_interfaces__msg__ShortVaried),
  false,  // has_any_key_member_
  ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_member_array,  // message members
  ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_type_support_handle = {
  0,
  &ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2cli_test_interfaces, msg, ShortVaried)() {
  if (!ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_type_support_handle.typesupport_identifier) {
    ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2cli_test_interfaces__msg__ShortVaried__rosidl_typesupport_introspection_c__ShortVaried_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
