// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied.h"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_H_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'BOOL_CONST'.
/**
  * Comment - Nesting Level 1: 1 of 2
 */
static const bool ros2cli_test_interfaces__msg__ShortVaried__BOOL_CONST = true;

// Include directives for member types
// Member 'bool_values'
#include "rosidl_runtime_c/primitives_sequence.h"

// constants for array fields with an upper bound
// bool_values
enum
{
  ros2cli_test_interfaces__msg__ShortVaried__bool_values__MAX_SIZE = 3
};

/// Struct defined in msg/ShortVaried in the package ros2cli_test_interfaces.
/**
  * A constant
 */
typedef struct ros2cli_test_interfaces__msg__ShortVaried
{
  /// Bool and array of bools
  bool bool_value;
  /// Comment - Nesting Level 1: 2 of 2
  rosidl_runtime_c__boolean__Sequence bool_values;
} ros2cli_test_interfaces__msg__ShortVaried;

// Struct for a sequence of ros2cli_test_interfaces__msg__ShortVaried.
typedef struct ros2cli_test_interfaces__msg__ShortVaried__Sequence
{
  ros2cli_test_interfaces__msg__ShortVaried * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__msg__ShortVaried__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_H_
