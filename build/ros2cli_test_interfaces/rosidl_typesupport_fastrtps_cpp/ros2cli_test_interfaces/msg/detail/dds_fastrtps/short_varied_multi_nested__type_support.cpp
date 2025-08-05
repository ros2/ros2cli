// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros2cli_test_interfaces:msg/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice
#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__struct.hpp"

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
bool cdr_serialize(
  const ros2cli_test_interfaces::msg::ShortVariedNested &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros2cli_test_interfaces::msg::ShortVariedNested &);
size_t get_serialized_size(
  const ros2cli_test_interfaces::msg::ShortVariedNested &,
  size_t current_alignment);
size_t
max_serialized_size_ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const ros2cli_test_interfaces::msg::ShortVariedNested &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const ros2cli_test_interfaces::msg::ShortVariedNested &,
  size_t current_alignment);
size_t
max_serialized_size_key_ShortVariedNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros2cli_test_interfaces


namespace ros2cli_test_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_serialize(
  const ros2cli_test_interfaces::msg::ShortVariedMultiNested & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: short_varied_nested
  ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.short_varied_nested,
    cdr);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces::msg::ShortVariedMultiNested & ros_message)
{
  // Member: short_varied_nested
  ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.short_varied_nested);

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size(
  const ros2cli_test_interfaces::msg::ShortVariedMultiNested & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: short_varied_nested
  current_alignment +=
    ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.short_varied_nested, current_alignment);

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_ShortVariedMultiNested(
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

  // Member: short_varied_nested
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_ShortVariedNested(
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
    using DataType = ros2cli_test_interfaces::msg::ShortVariedMultiNested;
    is_plain =
      (
      offsetof(DataType, short_varied_nested) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_serialize_key(
  const ros2cli_test_interfaces::msg::ShortVariedMultiNested & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: short_varied_nested
  ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.short_varied_nested,
    cdr);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size_key(
  const ros2cli_test_interfaces::msg::ShortVariedMultiNested & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: short_varied_nested
  current_alignment +=
    ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.short_varied_nested, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_key_ShortVariedMultiNested(
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

  // Member: short_varied_nested
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_key_ShortVariedNested(
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
    using DataType = ros2cli_test_interfaces::msg::ShortVariedMultiNested;
    is_plain =
      (
      offsetof(DataType, short_varied_nested) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _ShortVariedMultiNested__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros2cli_test_interfaces::msg::ShortVariedMultiNested *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ShortVariedMultiNested__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros2cli_test_interfaces::msg::ShortVariedMultiNested *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ShortVariedMultiNested__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros2cli_test_interfaces::msg::ShortVariedMultiNested *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ShortVariedMultiNested__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ShortVariedMultiNested(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ShortVariedMultiNested__callbacks = {
  "ros2cli_test_interfaces::msg",
  "ShortVariedMultiNested",
  _ShortVariedMultiNested__cdr_serialize,
  _ShortVariedMultiNested__cdr_deserialize,
  _ShortVariedMultiNested__get_serialized_size,
  _ShortVariedMultiNested__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ShortVariedMultiNested__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ShortVariedMultiNested__callbacks,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVariedMultiNested__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2cli_test_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros2cli_test_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::msg::ShortVariedMultiNested>()
{
  return &ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::_ShortVariedMultiNested__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, msg, ShortVariedMultiNested)() {
  return &ros2cli_test_interfaces::msg::typesupport_fastrtps_cpp::_ShortVariedMultiNested__handle;
}

#ifdef __cplusplus
}
#endif
