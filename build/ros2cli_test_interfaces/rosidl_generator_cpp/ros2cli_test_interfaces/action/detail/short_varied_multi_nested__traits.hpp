// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2cli_test_interfaces:action/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/action/short_varied_multi_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_
#define ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'short_varied_nested'
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: short_varied_nested
  {
    out << "short_varied_nested: ";
    to_flow_style_yaml(msg.short_varied_nested, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: short_varied_nested
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "short_varied_nested:\n";
    to_block_style_yaml(msg.short_varied_nested, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_Goal";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::msg::ShortVariedNested>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::msg::ShortVariedNested>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: bool_value
  {
    out << "bool_value: ";
    rosidl_generator_traits::value_to_yaml(msg.bool_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bool_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bool_value: ";
    rosidl_generator_traits::value_to_yaml(msg.bool_value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_Result";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_Result";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: bool_values
  {
    if (msg.bool_values.size() == 0) {
      out << "bool_values: []";
    } else {
      out << "bool_values: [";
      size_t pending_items = msg.bool_values.size();
      for (auto item : msg.bool_values) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bool_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bool_values.size() == 0) {
      out << "bool_values: []\n";
    } else {
      out << "bool_values:\n";
      for (auto item : msg.bool_values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_Feedback";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_SendGoal_Request";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_SendGoal_Response";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_SendGoal_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_SendGoal_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_SendGoal_Event";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>::value && has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_SendGoal";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>::value &&
    has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>::value &&
    has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2cli_test_interfaces::action::ShortVariedMultiNested_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_GetResult_Request";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_GetResult_Response";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_GetResult_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_GetResult_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_GetResult_Event";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>::value && has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_GetResult";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>::value &&
    has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>::value &&
    has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>::value
  >
{
};

template<>
struct is_service<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2cli_test_interfaces::action::ShortVariedMultiNested_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "ros2cli_test_interfaces/action/detail/short_varied_multi_nested__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedMultiNested_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedMultiNested_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested_FeedbackMessage";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::action::ShortVariedMultiNested_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::action::ShortVariedMultiNested>()
{
  return "ros2cli_test_interfaces::action::ShortVariedMultiNested";
}

template<>
inline const char * name<ros2cli_test_interfaces::action::ShortVariedMultiNested>()
{
  return "ros2cli_test_interfaces/action/ShortVariedMultiNested";
}

template<>
struct is_action<ros2cli_test_interfaces::action::ShortVariedMultiNested>
  : std::true_type
{
};

template<>
struct is_action_goal<ros2cli_test_interfaces::action::ShortVariedMultiNested_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<ros2cli_test_interfaces::action::ShortVariedMultiNested_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<ros2cli_test_interfaces::action::ShortVariedMultiNested_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // ROS2CLI_TEST_INTERFACES__ACTION__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_
