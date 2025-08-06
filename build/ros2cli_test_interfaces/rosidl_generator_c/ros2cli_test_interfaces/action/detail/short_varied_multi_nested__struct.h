// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2cli_test_interfaces:action/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/action/short_varied_multi_nested.h"


#ifndef ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_
#define ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_

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

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal
{
  /// Comment - Nesting Level 3: 1 of 2
  ros2cli_test_interfaces__msg__ShortVariedNested short_varied_nested;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Result
{
  /// Comment - Nesting Level 3: 2 of 2
  bool bool_value;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Result;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_Result.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback
{
  bool bool_values[3];
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Goal goal;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Request__Sequence request;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Response__Sequence response;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response
{
  int8_t status;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Result result;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Request__Sequence request;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Response__Sequence response;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.h"

/// Struct defined in action/ShortVariedMultiNested in the package ros2cli_test_interfaces.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ros2cli_test_interfaces__action__ShortVariedMultiNested_Feedback feedback;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage;

// Struct for a sequence of ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage.
typedef struct ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence
{
  ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2cli_test_interfaces__action__ShortVariedMultiNested_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_H_
