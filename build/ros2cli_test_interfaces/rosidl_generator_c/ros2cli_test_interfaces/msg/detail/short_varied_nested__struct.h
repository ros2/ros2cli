// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied_nested.h"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_H_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'short_varied'
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.h"

/// Struct defined in msg/ShortVariedNested in the package ros2cli_test_interfaces.
/**
  * A short, varied type
 */
typedef struct ros2cli_test_interfaces__msg__ShortVariedNested
{
  /// Comment - Nesting Level 2: 1 of 1
  ros2cli_test_interfaces__msg__ShortVaried short_varied;
} ros2cli_test_interfaces__msg__ShortVariedNested;

// Struct for a sequence of ros2cli_test_interfaces__msg__ShortVariedNested.
typedef struct ros2cli_test_interfaces__msg__ShortVariedNested__Sequence
{
  ros2cli_test_interfaces__msg__ShortVariedNested * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__msg__ShortVariedNested__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_H_
