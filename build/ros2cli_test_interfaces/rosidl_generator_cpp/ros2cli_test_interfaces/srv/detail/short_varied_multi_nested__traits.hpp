// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/srv/short_varied_multi_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_
#define ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'short_varied_nested'
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Request & msg,
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
  const ShortVariedMultiNested_Request & msg,
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

inline std::string to_yaml(const ShortVariedMultiNested_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>()
{
  return "ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request";
}

template<>
inline const char * name<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>()
{
  return "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Request";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::msg::ShortVariedNested>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::msg::ShortVariedNested>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2cli_test_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Response & msg,
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
  const ShortVariedMultiNested_Response & msg,
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

inline std::string to_yaml(const ShortVariedMultiNested_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>()
{
  return "ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response";
}

template<>
inline const char * name<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>()
{
  return "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Response";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ShortVariedMultiNested_Event & msg,
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
  const ShortVariedMultiNested_Event & msg,
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

inline std::string to_yaml(const ShortVariedMultiNested_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>()
{
  return "ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event";
}

template<>
inline const char * name<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>()
{
  return "ros2cli_test_interfaces/srv/ShortVariedMultiNested_Event";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>::value && has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::srv::ShortVariedMultiNested>()
{
  return "ros2cli_test_interfaces::srv::ShortVariedMultiNested";
}

template<>
inline const char * name<ros2cli_test_interfaces::srv::ShortVariedMultiNested>()
{
  return "ros2cli_test_interfaces/srv/ShortVariedMultiNested";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>::value &&
    has_fixed_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>::value &&
    has_bounded_size<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>::value
  >
{
};

template<>
struct is_service<ros2cli_test_interfaces::srv::ShortVariedMultiNested>
  : std::true_type
{
};

template<>
struct is_service_request<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__TRAITS_HPP_
