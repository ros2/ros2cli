// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/msg/detail/short_varied__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace ros2cli_test_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_serialize(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bool_value
  cdr << (ros_message.bool_value ? true : false);

  // Member: bool_values
  {
    size_t size = ros_message.bool_values.size();
    if (size > 3) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << (ros_message.bool_values[i] ? true : false);
    }
  }

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces::msg::ShortVaried & ros_message)
{
  // Member: bool_value
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.bool_value = tmp ? true : false;
  }

  // Member: bool_values
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

    ros_message.bool_values.resize(size);
    for (size_t i = 0; i < size; i++) {
      uint8_t tmp;
      cdr >> tmp;
      ros_message.bool_values[i] = tmp ? true : false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bool_value
  {
    size_t item_size = sizeof(ros_message.bool_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: bool_values
  {
    size_t array_size = ros_message.bool_values.size();
    if (array_size > 3) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.bool_values[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_ShortVaried(
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

  // Member: bool_value
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // Member: bool_values
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
    using DataType = ros2cli_test_interfaces::msg::ShortVaried;
    is_plain =
      (
      offsetof(DataType, bool_values) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_serialize_key(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bool_value
  cdr << (ros_message.bool_value ? true : false);

  // Member: bool_values
  {
    size_t size = ros_message.bool_values.size();
    if (size > 3) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << (ros_message.bool_values[i] ? true : false);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size_key(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bool_value
  {
    size_t item_size = sizeof(ros_message.bool_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: bool_values
  {
    size_t array_size = ros_message.bool_values.size();
    if (array_size > 3) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.bool_values[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_key_ShortVaried(
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

  // Member: bool_value
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: bool_values
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
    using DataType = ros2cli_test_interfaces::msg::ShortVaried;
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
  auto typed_message =
    static_cast<const ros2cli_test_interfaces::msg::ShortVaried *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ShortVaried__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2cli_test_interfaces::msg::ShortVaried *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ShortVaried__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2cli_test_interfaces::msg::ShortVaried *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ShortVaried__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ShortVaried(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ShortVaried__callbacks = {
  "ros2cli_test_interfaces::msg",
  "ShortVaried",
  _ShortVaried__cdr_serialize,
  _ShortVaried__cdr_deserialize,
  _ShortVaried__get_serialized_size,
  _ShortVaried__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVaried__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ShortVaried__callbacks,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::msg::ShortVaried>()
{
  return &ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::_ShortVaried__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, msg, ShortVaried)() {
  return &ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::_ShortVaried__handle;
}

#ifdef __cplusplus
}
#endif
