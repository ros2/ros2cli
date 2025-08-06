// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from ros2cli_test_interfaces:msg/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice
#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2cli_test_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  const ros2cli_test_interfaces__msg__ShortVariedMultiNested * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_deserialize_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  eprosima::fastcdr::Cdr &,
  ros2cli_test_interfaces__msg__ShortVariedMultiNested * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
bool cdr_serialize_key_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  const ros2cli_test_interfaces__msg__ShortVariedMultiNested * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t get_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
size_t max_serialized_size_key_ros2cli_test_interfaces__msg__ShortVariedMultiNested(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2cli_test_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2cli_test_interfaces, msg, ShortVariedMultiNested)();

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
