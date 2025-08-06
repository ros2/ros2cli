// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.hpp"

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

#include "fastcdr/Cdr.h"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros2cli_test_interfaces::msg::ShortVaried & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_ShortVaried(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
cdr_serialize_key(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
get_serialized_size_key(
  const ros2cli_test_interfaces::msg::ShortVaried & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
max_serialized_size_key_ShortVaried(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros2cli_test_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros2cli_test_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros2cli_test_interfaces, msg, ShortVaried)();

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
