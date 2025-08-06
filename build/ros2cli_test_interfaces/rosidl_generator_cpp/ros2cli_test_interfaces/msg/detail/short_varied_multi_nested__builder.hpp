// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied_multi_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2cli_test_interfaces/msg/detail/short_varied_multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2cli_test_interfaces
{

namespace msg
{

namespace builder
{

class Init_ShortVariedMultiNested_short_varied_nested
{
public:
  Init_ShortVariedMultiNested_short_varied_nested()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2cli_test_interfaces::msg::ShortVariedMultiNested short_varied_nested(::ros2cli_test_interfaces::msg::ShortVariedMultiNested::_short_varied_nested_type arg)
  {
    msg_.short_varied_nested = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2cli_test_interfaces::msg::ShortVariedMultiNested msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2cli_test_interfaces::msg::ShortVariedMultiNested>()
{
  return ros2cli_test_interfaces::msg::builder::Init_ShortVariedMultiNested_short_varied_nested();
}

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_
