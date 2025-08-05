// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2cli_test_interfaces:action/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
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

#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__functions.h"  // short_varied_nested

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__msg__ShortVariedNested(
  const ros2cli_test_interfaces__msg__ShortVariedNested * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__msg__ShortVariedNested(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__msg__ShortVariedNested * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  const ros2cli_test_interfaces__msg__ShortVariedNested * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, msg, ShortVariedNested)();


using _ShortVariedMultiNested_Goal__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: short_varied_nested
  {
    cdr_serialize_ros2cli_test_interfaces__msg__ShortVariedNested(
      &ros_message->short_varied_nested, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message)
{
  // Field name: short_varied_nested
  {
    cdr_deserialize_ros2cli_test_interfaces__msg__ShortVariedNested(cdr, &ros_message->short_varied_nested);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Goal__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Goal__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: short_varied_nested
  current_alignment += get_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
    &(ros_message->short_varied_nested), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
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

  // Field name: short_varied_nested
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__msg__ShortVariedNested(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal;
    is_plain =
      (
      offsetof(DataType, short_varied_nested) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: short_varied_nested
  {
    cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVariedNested(
      &ros_message->short_varied_nested, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Goal__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Goal__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: short_varied_nested
  current_alignment += get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
    &(ros_message->short_varied_nested), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
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
  // Field name: short_varied_nested
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedNested(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal;
    is_plain =
      (
      offsetof(DataType, short_varied_nested) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_Goal__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(ros_message, cdr);
}

static bool _ShortVariedMultiNested_Goal__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_Goal__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_Goal__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_Goal = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_Goal",
  _ShortVariedMultiNested_Goal__cdr_serialize,
  _ShortVariedMultiNested_Goal__cdr_deserialize,
  _ShortVariedMultiNested_Goal__get_serialized_size,
  _ShortVariedMultiNested_Goal__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_Goal__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_Goal,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Goal)() {
  return &_ShortVariedMultiNested_Goal__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


// forward declare type support functions


using _ShortVariedMultiNested_Result__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_value
  {
    cdr << (ros_message->bool_value ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message)
{
  // Field name: bool_value
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bool_value = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Result__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Result__ros_msg_type *>(untyped_ros_message);
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

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
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


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result;
    is_plain =
      (
      offsetof(DataType, bool_value) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_value
  {
    cdr << (ros_message->bool_value ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Result__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Result__ros_msg_type *>(untyped_ros_message);
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

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Result;
    is_plain =
      (
      offsetof(DataType, bool_value) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_Result__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(ros_message, cdr);
}

static bool _ShortVariedMultiNested_Result__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_Result *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_Result__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_Result__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_Result = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_Result",
  _ShortVariedMultiNested_Result__cdr_serialize,
  _ShortVariedMultiNested_Result__cdr_deserialize,
  _ShortVariedMultiNested_Result__get_serialized_size,
  _ShortVariedMultiNested_Result__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_Result__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_Result,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Result)() {
  return &_ShortVariedMultiNested_Result__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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


// forward declare type support functions


using _ShortVariedMultiNested_Feedback__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_values
  {
    size_t size = 3;
    auto array_ptr = ros_message->bool_values;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message)
{
  // Field name: bool_values
  {
    size_t size = 3;
    auto array_ptr = ros_message->bool_values;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Feedback__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Feedback__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: bool_values
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->bool_values;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
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

  // Field name: bool_values
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback;
    is_plain =
      (
      offsetof(DataType, bool_values) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: bool_values
  {
    size_t size = 3;
    auto array_ptr = ros_message->bool_values;
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_Feedback__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_Feedback__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: bool_values
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->bool_values;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
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
  // Field name: bool_values
  {
    size_t array_size = 3;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback;
    is_plain =
      (
      offsetof(DataType, bool_values) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_Feedback__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(ros_message, cdr);
}

static bool _ShortVariedMultiNested_Feedback__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_Feedback__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_Feedback__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_Feedback = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_Feedback",
  _ShortVariedMultiNested_Feedback__cdr_serialize,
  _ShortVariedMultiNested_Feedback__cdr_deserialize,
  _ShortVariedMultiNested_Feedback__get_serialized_size,
  _ShortVariedMultiNested_Feedback__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_Feedback__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_Feedback,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Feedback)() {
  return &_ShortVariedMultiNested_Feedback__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"  // goal
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"  // goal_id

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Goal)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_unique_identifier_msgs__msg__UUID(
  eprosima::fastcdr::Cdr & cdr,
  unique_identifier_msgs__msg__UUID * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unique_identifier_msgs, msg, UUID)();


using _ShortVariedMultiNested_SendGoal_Request__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  // Field name: goal
  {
    cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
      &ros_message->goal, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message)
{
  // Field name: goal_id
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->goal_id);
  }

  // Field name: goal
  {
    cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(cdr, &ros_message->goal);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Request__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  // Field name: goal
  current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
    &(ros_message->goal), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
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

  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request;
    is_plain =
      (
      offsetof(DataType, goal) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  // Field name: goal
  {
    cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
      &ros_message->goal, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Request__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  // Field name: goal
  current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
    &(ros_message->goal), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
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
  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request;
    is_plain =
      (
      offsetof(DataType, goal) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_SendGoal_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(ros_message, cdr);
}

static bool _ShortVariedMultiNested_SendGoal_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_SendGoal_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_SendGoal_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_SendGoal_Request = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_SendGoal_Request",
  _ShortVariedMultiNested_SendGoal_Request__cdr_serialize,
  _ShortVariedMultiNested_SendGoal_Request__cdr_deserialize,
  _ShortVariedMultiNested_SendGoal_Request__get_serialized_size,
  _ShortVariedMultiNested_SendGoal_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_SendGoal_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_SendGoal_Request,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Request)() {
  return &_ShortVariedMultiNested_SendGoal_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "builtin_interfaces/msg/detail/time__functions.h"  // stamp

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_builtin_interfaces__msg__Time(
  const builtin_interfaces__msg__Time * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_builtin_interfaces__msg__Time(
  eprosima::fastcdr::Cdr & cdr,
  builtin_interfaces__msg__Time * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_builtin_interfaces__msg__Time(
  const builtin_interfaces__msg__Time * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _ShortVariedMultiNested_SendGoal_Response__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: stamp
  {
    cdr_serialize_builtin_interfaces__msg__Time(
      &ros_message->stamp, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message)
{
  // Field name: accepted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accepted = tmp ? true : false;
  }

  // Field name: stamp
  {
    cdr_deserialize_builtin_interfaces__msg__Time(cdr, &ros_message->stamp);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Response__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stamp
  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
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

  // Field name: accepted
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stamp
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response;
    is_plain =
      (
      offsetof(DataType, stamp) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: accepted
  {
    cdr << (ros_message->accepted ? true : false);
  }

  // Field name: stamp
  {
    cdr_serialize_key_builtin_interfaces__msg__Time(
      &ros_message->stamp, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Response__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: accepted
  {
    size_t item_size = sizeof(ros_message->accepted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: stamp
  current_alignment += get_serialized_size_key_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
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
  // Field name: accepted
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: stamp
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_builtin_interfaces__msg__Time(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response;
    is_plain =
      (
      offsetof(DataType, stamp) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_SendGoal_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(ros_message, cdr);
}

static bool _ShortVariedMultiNested_SendGoal_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_SendGoal_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_SendGoal_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_SendGoal_Response = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_SendGoal_Response",
  _ShortVariedMultiNested_SendGoal_Response__cdr_serialize,
  _ShortVariedMultiNested_SendGoal_Response__cdr_deserialize,
  _ShortVariedMultiNested_SendGoal_Response__get_serialized_size,
  _ShortVariedMultiNested_SendGoal_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_SendGoal_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_SendGoal_Response,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Response)() {
  return &_ShortVariedMultiNested_SendGoal_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Request)();

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _ShortVariedMultiNested_SendGoal_Event__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
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

    if (ros_message->request.data) {
      ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__fini(&ros_message->request);
    }
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
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

    if (ros_message->response.data) {
      ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__fini(&ros_message->response);
    }
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Event__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
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

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_SendGoal_Event__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_SendGoal_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
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
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_SendGoal_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(ros_message, cdr);
}

static bool _ShortVariedMultiNested_SendGoal_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_SendGoal_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_SendGoal_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_SendGoal_Event = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_SendGoal_Event",
  _ShortVariedMultiNested_SendGoal_Event__cdr_serialize,
  _ShortVariedMultiNested_SendGoal_Event__cdr_deserialize,
  _ShortVariedMultiNested_SendGoal_Event__get_serialized_size,
  _ShortVariedMultiNested_SendGoal_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_SendGoal_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_SendGoal_Event,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Event)() {
  return &_ShortVariedMultiNested_SendGoal_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2cli_test_interfaces/action/short_varied_multi_nested.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ShortVariedMultiNested_SendGoal__callbacks = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_SendGoal",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal_Response)(),
};

static rosidl_service_type_support_t ShortVariedMultiNested_SendGoal__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ShortVariedMultiNested_SendGoal__callbacks,
  get_service_typesupport_handle_function,
  &_ShortVariedMultiNested_SendGoal_Request__type_support,
  &_ShortVariedMultiNested_SendGoal_Response__type_support,
  &_ShortVariedMultiNested_SendGoal_Event__type_support,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    action,
    ShortVariedMultiNested_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    action,
    ShortVariedMultiNested_SendGoal
  ),
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal__get_type_description_sources,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_SendGoal)() {
  return &ShortVariedMultiNested_SendGoal__handle;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"  // goal_id

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_unique_identifier_msgs__msg__UUID(
  eprosima::fastcdr::Cdr & cdr,
  unique_identifier_msgs__msg__UUID * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unique_identifier_msgs, msg, UUID)();


using _ShortVariedMultiNested_GetResult_Request__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message)
{
  // Field name: goal_id
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->goal_id);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Request__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
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

  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request;
    is_plain =
      (
      offsetof(DataType, goal_id) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Request__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
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
  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request;
    is_plain =
      (
      offsetof(DataType, goal_id) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_GetResult_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(ros_message, cdr);
}

static bool _ShortVariedMultiNested_GetResult_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_GetResult_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_GetResult_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_GetResult_Request = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_GetResult_Request",
  _ShortVariedMultiNested_GetResult_Request__cdr_serialize,
  _ShortVariedMultiNested_GetResult_Request__cdr_deserialize,
  _ShortVariedMultiNested_GetResult_Request__get_serialized_size,
  _ShortVariedMultiNested_GetResult_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_GetResult_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_GetResult_Request,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Request)() {
  return &_ShortVariedMultiNested_GetResult_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"  // result

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Result)();


using _ShortVariedMultiNested_GetResult_Response__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: status
  {
    cdr << ros_message->status;
  }

  // Field name: result
  {
    cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
      &ros_message->result, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message)
{
  // Field name: status
  {
    cdr >> ros_message->status;
  }

  // Field name: result
  {
    cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(cdr, &ros_message->result);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Response__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: status
  {
    size_t item_size = sizeof(ros_message->status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: result
  current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
    &(ros_message->result), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
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

  // Field name: status
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: result
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response;
    is_plain =
      (
      offsetof(DataType, result) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: status
  {
    cdr << ros_message->status;
  }

  // Field name: result
  {
    cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
      &ros_message->result, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Response__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: status
  {
    size_t item_size = sizeof(ros_message->status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: result
  current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
    &(ros_message->result), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
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
  // Field name: status
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: result
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Result(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response;
    is_plain =
      (
      offsetof(DataType, result) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_GetResult_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(ros_message, cdr);
}

static bool _ShortVariedMultiNested_GetResult_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_GetResult_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_GetResult_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_GetResult_Response = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_GetResult_Response",
  _ShortVariedMultiNested_GetResult_Response__cdr_serialize,
  _ShortVariedMultiNested_GetResult_Response__cdr_deserialize,
  _ShortVariedMultiNested_GetResult_Response__get_serialized_size,
  _ShortVariedMultiNested_GetResult_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_GetResult_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_GetResult_Response,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Response)() {
  return &_ShortVariedMultiNested_GetResult_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Request)();

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _ShortVariedMultiNested_GetResult_Event__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
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

    if (ros_message->request.data) {
      ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__fini(&ros_message->request);
    }
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
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

    if (ros_message->response.data) {
      ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__fini(&ros_message->response);
    }
    if (!ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Event__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
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

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_GetResult_Event__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_GetResult_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
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
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_GetResult_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(ros_message, cdr);
}

static bool _ShortVariedMultiNested_GetResult_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_GetResult_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_GetResult_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_GetResult_Event = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_GetResult_Event",
  _ShortVariedMultiNested_GetResult_Event__cdr_serialize,
  _ShortVariedMultiNested_GetResult_Event__cdr_deserialize,
  _ShortVariedMultiNested_GetResult_Event__get_serialized_size,
  _ShortVariedMultiNested_GetResult_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_GetResult_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_GetResult_Event,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Event)() {
  return &_ShortVariedMultiNested_GetResult_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/short_varied_multi_nested.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ShortVariedMultiNested_GetResult__callbacks = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_GetResult",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult_Response)(),
};

static rosidl_service_type_support_t ShortVariedMultiNested_GetResult__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ShortVariedMultiNested_GetResult__callbacks,
  get_service_typesupport_handle_function,
  &_ShortVariedMultiNested_GetResult_Request__type_support,
  &_ShortVariedMultiNested_GetResult_Response__type_support,
  &_ShortVariedMultiNested_GetResult_Event__type_support,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    action,
    ShortVariedMultiNested_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    ros2cli_test_interfaces,
    action,
    ShortVariedMultiNested_GetResult
  ),
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult__get_type_description_sources,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_GetResult)() {
  return &ShortVariedMultiNested_GetResult__handle;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__functions.h"  // feedback
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"  // goal_id

// forward declare type support functions

bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message);

size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_Feedback)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_deserialize_unique_identifier_msgs__msg__UUID(
  eprosima::fastcdr::Cdr & cdr,
  unique_identifier_msgs__msg__UUID * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
bool cdr_serialize_key_unique_identifier_msgs__msg__UUID(
  const unique_identifier_msgs__msg__UUID * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t get_serialized_size_key_unique_identifier_msgs__msg__UUID(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
size_t max_serialized_size_key_unique_identifier_msgs__msg__UUID(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, unique_identifier_msgs, msg, UUID)();


using _ShortVariedMultiNested_FeedbackMessage__ros_msg_type = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  // Field name: feedback
  {
    cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
      &ros_message->feedback, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * ros_message)
{
  // Field name: goal_id
  {
    cdr_deserialize_unique_identifier_msgs__msg__UUID(cdr, &ros_message->goal_id);
  }

  // Field name: feedback
  {
    cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(cdr, &ros_message->feedback);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_FeedbackMessage__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_FeedbackMessage__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  // Field name: feedback
  current_alignment += get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
    &(ros_message->feedback), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
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

  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: feedback
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage;
    is_plain =
      (
      offsetof(DataType, feedback) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: goal_id
  {
    cdr_serialize_key_unique_identifier_msgs__msg__UUID(
      &ros_message->goal_id, cdr);
  }

  // Field name: feedback
  {
    cdr_serialize_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
      &ros_message->feedback, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ShortVariedMultiNested_FeedbackMessage__ros_msg_type * ros_message = static_cast<const _ShortVariedMultiNested_FeedbackMessage__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: goal_id
  current_alignment += get_serialized_size_key_unique_identifier_msgs__msg__UUID(
    &(ros_message->goal_id), current_alignment);

  // Field name: feedback
  current_alignment += get_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
    &(ros_message->feedback), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
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
  // Field name: goal_id
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_unique_identifier_msgs__msg__UUID(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: feedback
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback(
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
    using DataType = ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage;
    is_plain =
      (
      offsetof(DataType, feedback) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ShortVariedMultiNested_FeedbackMessage__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * ros_message = static_cast<const ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(ros_message, cdr);
}

static bool _ShortVariedMultiNested_FeedbackMessage__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * ros_message = static_cast<ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(cdr, ros_message);
}

static uint32_t _ShortVariedMultiNested_FeedbackMessage__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
      untyped_ros_message, 0));
}

static size_t _ShortVariedMultiNested_FeedbackMessage__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ShortVariedMultiNested_FeedbackMessage = {
  "ros2cli_test_interfaces::action",
  "ShortVariedMultiNested_FeedbackMessage",
  _ShortVariedMultiNested_FeedbackMessage__cdr_serialize,
  _ShortVariedMultiNested_FeedbackMessage__cdr_deserialize,
  _ShortVariedMultiNested_FeedbackMessage__get_serialized_size,
  _ShortVariedMultiNested_FeedbackMessage__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested_FeedbackMessage__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ShortVariedMultiNested_FeedbackMessage,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__get_type_hash,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__get_type_description,
  &ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, action, ShortVariedMultiNested_FeedbackMessage)() {
  return &_ShortVariedMultiNested_FeedbackMessage__type_support;
}

#if defined(__cplusplus)
}
#endif
