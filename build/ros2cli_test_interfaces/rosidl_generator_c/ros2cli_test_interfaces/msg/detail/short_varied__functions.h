// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied.h"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__FUNCTIONS_H_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "ros2cli_test_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.h"

/// Initialize msg/ShortVaried message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ros2cli_test_interfaces__msg__ShortVaried
 * )) before or use
 * ros2cli_test_interfaces__msg__ShortVaried__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__init(ros2cli_test_interfaces__msg__ShortVaried * msg);

/// Finalize msg/ShortVaried message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
void
ros2cli_test_interfaces__msg__ShortVaried__fini(ros2cli_test_interfaces__msg__ShortVaried * msg);

/// Create msg/ShortVaried message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ros2cli_test_interfaces__msg__ShortVaried__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
ros2cli_test_interfaces__msg__ShortVaried *
ros2cli_test_interfaces__msg__ShortVaried__create(void);

/// Destroy msg/ShortVaried message.
/**
 * It calls
 * ros2cli_test_interfaces__msg__ShortVaried__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
void
ros2cli_test_interfaces__msg__ShortVaried__destroy(ros2cli_test_interfaces__msg__ShortVaried * msg);

/// Check for msg/ShortVaried message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__are_equal(const ros2cli_test_interfaces__msg__ShortVaried * lhs, const ros2cli_test_interfaces__msg__ShortVaried * rhs);

/// Copy a msg/ShortVaried message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__copy(
  const ros2cli_test_interfaces__msg__ShortVaried * input,
  ros2cli_test_interfaces__msg__ShortVaried * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_type_hash_t *
ros2cli_test_interfaces__msg__ShortVaried__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
ros2cli_test_interfaces__msg__ShortVaried__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_runtime_c__type_description__TypeSource *
ros2cli_test_interfaces__msg__ShortVaried__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/ShortVaried messages.
/**
 * It allocates the memory for the number of elements and calls
 * ros2cli_test_interfaces__msg__ShortVaried__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__Sequence__init(ros2cli_test_interfaces__msg__ShortVaried__Sequence * array, size_t size);

/// Finalize array of msg/ShortVaried messages.
/**
 * It calls
 * ros2cli_test_interfaces__msg__ShortVaried__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
void
ros2cli_test_interfaces__msg__ShortVaried__Sequence__fini(ros2cli_test_interfaces__msg__ShortVaried__Sequence * array);

/// Create array of msg/ShortVaried messages.
/**
 * It allocates the memory for the array and calls
 * ros2cli_test_interfaces__msg__ShortVaried__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
ros2cli_test_interfaces__msg__ShortVaried__Sequence *
ros2cli_test_interfaces__msg__ShortVaried__Sequence__create(size_t size);

/// Destroy array of msg/ShortVaried messages.
/**
 * It calls
 * ros2cli_test_interfaces__msg__ShortVaried__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
void
ros2cli_test_interfaces__msg__ShortVaried__Sequence__destroy(ros2cli_test_interfaces__msg__ShortVaried__Sequence * array);

/// Check for msg/ShortVaried message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__Sequence__are_equal(const ros2cli_test_interfaces__msg__ShortVaried__Sequence * lhs, const ros2cli_test_interfaces__msg__ShortVaried__Sequence * rhs);

/// Copy an array of msg/ShortVaried messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ros2cli_test_interfaces
bool
ros2cli_test_interfaces__msg__ShortVaried__Sequence__copy(
  const ros2cli_test_interfaces__msg__ShortVaried__Sequence * input,
  ros2cli_test_interfaces__msg__ShortVaried__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__FUNCTIONS_H_
