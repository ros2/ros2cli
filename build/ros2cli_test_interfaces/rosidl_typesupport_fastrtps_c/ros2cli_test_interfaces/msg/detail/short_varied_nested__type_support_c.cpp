// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"  // short_varied

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__msg__ShortVaried(
  const ros2cli_test_interfaces__msg__ShortVaried * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__msg__ShortVaried(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__msg__ShortVaried * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVaried(
  const ros2cli_test_interfaces__msg__ShortVaried * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, msg, ShortVaried)();


using _ShortVariedNested__ros_msg_type = ros2cli_test_interfaces__msg__ShortVariedNested;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__msg__ShortVariedNested(
  const ros2cli_test_interfaces__msg__ShortVariedNested * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: short_varied
  {
    cdr_serialize_ros2cli_test_interfaces__msg__ShortVaried(
      &ros_message->short_varied, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__msg__ShortVariedNested(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__msg__ShortVariedNested * ros_message)
{
  // Field name: short_varied
  {
    cdr_deserialize_ros2cli_test_interfaces__msg__ShortVaried(cdr, &ros_message->short_varied);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedNested__ros_msg_type * ros_message = static_cast<const _ShortVariedNested__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: short_varied
  current_alignment += get_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
    &(ros_message->short_varied), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: short_varied
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__msg__ShortVariedNested;
    is_plain =
      (
      offsetof(DataType, short_varied) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  const ros2cli_test_interfaces__msg__ShortVariedNested * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: short_varied
  {
    cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVaried(
      &ros_message->short_varied, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedNested__ros_msg_type * ros_message = static_cast<const _ShortVariedNested__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: short_varied
  current_alignment += get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
    &(ros_message->short_varied), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: short_varied
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__msg__ShortVariedNested;
    is_plain =
      (
      offsetof(DataType, short_varied) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedNested__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__msg__ShortVariedNested * ros_message = static_cast<const ros2cli_test_interfaces__msg__ShortVariedNested *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__msg__ShortVariedNested(ros_message, cdr);
}

static bool _ShortVariedNested__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__msg__ShortVariedNested * ros_message = static_cast<ros2cli_test_interfaces__msg__ShortVariedNested *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__msg__ShortVariedNested(cdr, ros_message);
}

static uint32_t _ShortVariedNested__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
      untyped_ros_message, 0));
}

static size_t _ShortVariedNested__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedNested = {
  "ros2cli_test_interfaces::msg",
  "ShortVariedNested",
  _ShortVariedNested__cdr_serialize,
  _ShortVariedNested__cdr_deserialize,
  _ShortVariedNested__get_serialized_size,
  _ShortVariedNested__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedNested__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedNested,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVariedNested__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, msg, ShortVariedNested)() {
  return &_ShortVariedNested__type_support;
}

#if defined(__cplusplus)
}
#endif
