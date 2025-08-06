// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/msg/detail/short_varied__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // bool_values
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // bool_values

// forward declare type support functions


using _ShortVaried__ros_msg_type = ros2cli_test_interfaces__msg__ShortVaried;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__msg__ShortVaried(
  const ros2cli_test_interfaces__msg__ShortVaried * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_value
  {
    cdr << (ros_message->bool_value ? true : false);
  }

  // Field name: bool_values
  {
    size_t size = ros_message->bool_values.size;
    auto array_ptr = ros_message->bool_values.data;
    if (size > 3) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__msg__ShortVaried(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__msg__ShortVaried * ros_message)
{
  // Field name: bool_value
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bool_value = tmp ? true : false;
  }

  // Field name: bool_values
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->bool_values.data) {
      rosidl_runtime_c__boolean__Sequence__fini(&ros_message->bool_values);
    }
    if (!rosidl_runtime_c__boolean__Sequence__init(&ros_message->bool_values, size)) {
      fprintf(stderr, "failed to create array for field 'bool_values'");
      return false;
    }
    auto array_ptr = ros_message->bool_values.data;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVaried__ros_msg_type * ros_message = static_cast<const _ShortVaried__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: bool_value
  {
    size_t item_size = sizeof(ros_message->bool_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: bool_values
  {
    size_t array_size = ros_message->bool_values.size;
    auto array_ptr = ros_message->bool_values.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
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

  // Field name: bool_value
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: bool_values
  {
    size_t array_size = 3;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__msg__ShortVaried;
    is_plain =
      (
      offsetof(DataType, bool_values) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVaried(
  const ros2cli_test_interfaces__msg__ShortVaried * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_value
  {
    cdr << (ros_message->bool_value ? true : false);
  }

  // Field name: bool_values
  {
    size_t size = ros_message->bool_values.size;
    auto array_ptr = ros_message->bool_values.data;
    if (size > 3) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVaried__ros_msg_type * ros_message = static_cast<const _ShortVaried__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: bool_value
  {
    size_t item_size = sizeof(ros_message->bool_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: bool_values
  {
    size_t array_size = ros_message->bool_values.size;
    auto array_ptr = ros_message->bool_values.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVaried(
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
  // Field name: bool_value
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: bool_values
  {
    size_t array_size = 3;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__msg__ShortVaried;
    is_plain =
      (
      offsetof(DataType, bool_values) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVaried__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__msg__ShortVaried * ros_message = static_cast<const ros2cli_test_interfaces__msg__ShortVaried *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__msg__ShortVaried(ros_message, cdr);
}

static bool _ShortVaried__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__msg__ShortVaried * ros_message = static_cast<ros2cli_test_interfaces__msg__ShortVaried *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__msg__ShortVaried(cdr, ros_message);
}

static uint32_t _ShortVaried__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
      untyped_ros_message, 0));
}

static size_t _ShortVaried__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__msg__ShortVaried(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVaried = {
  "ros2cli_test_interfaces::msg",
  "ShortVaried",
  _ShortVaried__cdr_serialize,
  _ShortVaried__cdr_deserialize,
  _ShortVaried__get_serialized_size,
  _ShortVaried__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVaried__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVaried,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, msg, ShortVaried)() {
  return &_ShortVaried__type_support;
}

#if defined(__cplusplus)
}
#endif
