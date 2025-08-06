// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__BUILDER_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2cli_test_interfaces
{

namespace msg
{

namespace builder
{

class Init_ShortVaried_bool_values
{
public:
  explicit Init_ShortVaried_bool_values(::ros2cli_test_interfaces::msg::ShortVaried & msg)
  : msg_(msg)
  {}
  ::ros2cli_test_interfaces::msg::ShortVaried bool_values(::ros2cli_test_interfaces::msg::ShortVaried::_bool_values_type arg)
  {
    msg_.bool_values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2cli_test_interfaces::msg::ShortVaried msg_;
};

class Init_ShortVaried_bool_value
{
public:
  Init_ShortVaried_bool_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ShortVaried_bool_values bool_value(::ros2cli_test_interfaces::msg::ShortVaried::_bool_value_type arg)
  {
    msg_.bool_value = std::move(arg);
    return Init_ShortVaried_bool_values(msg_);
  }

private:
  ::ros2cli_test_interfaces::msg::ShortVaried msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2cli_test_interfaces::msg::ShortVaried>()
{
  return ros2cli_test_interfaces::msg::builder::Init_ShortVaried_bool_value();
}

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__BUILDER_HPP_
