// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__TRAITS_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'short_varied'
#include "ros2cli_test_interfaces/msg/detail/short_varied__traits.hpp"

namespace ros2cli_test_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ShortVariedNested & msg,
  std::ostream & out)
{
  out << "{";
  // member: short_varied
  {
    out << "short_varied: ";
    to_flow_style_yaml(msg.short_varied, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ShortVariedNested & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: short_varied
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "short_varied:\n";
    to_block_style_yaml(msg.short_varied, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ShortVariedNested & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros2cli_test_interfaces

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2cli_test_interfaces::msg::ShortVariedNested>()
{
  return "ros2cli_test_interfaces::msg::ShortVariedNested";
}

template<>
inline const char * name<ros2cli_test_interfaces::msg::ShortVariedNested>()
{
  return "ros2cli_test_interfaces/msg/ShortVariedNested";
}

template<>
struct has_fixed_size<ros2cli_test_interfaces::msg::ShortVariedNested>
  : std::integral_constant<bool, has_fixed_size<ros2cli_test_interfaces::msg::ShortVaried>::value> {};

template<>
struct has_bounded_size<ros2cli_test_interfaces::msg::ShortVariedNested>
  : std::integral_constant<bool, has_bounded_size<ros2cli_test_interfaces::msg::ShortVaried>::value> {};

template<>
struct is_message<ros2cli_test_interfaces::msg::ShortVariedNested>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__TRAITS_HPP_
