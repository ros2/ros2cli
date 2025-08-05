// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2cli_test_interfaces:msg/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied_multi_nested.h"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'short_varied_nested'
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.h"

/// Struct defined in msg/ShortVariedMultiNested in the package ros2cli_test_interfaces.
/**
  * A short, varied, and nested type
 */
typedef struct ros2cli_test_interfaces__msg__ShortVariedMultiNested
{
  /// Comment - Nesting Level 3: 1 of 1
  ros2cli_test_interfaces__msg__ShortVariedNested short_varied_nested;
} ros2cli_test_interfaces__msg__ShortVariedMultiNested;

// Struct for a sequence of ros2cli_test_interfaces__msg__ShortVariedMultiNested.
typedef struct ros2cli_test_interfaces__msg__ShortVariedMultiNested__Sequence
{
  ros2cli_test_interfaces__msg__ShortVariedMultiNested * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__msg__ShortVariedMultiNested__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_
