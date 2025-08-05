// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/srv/short_varied_multi_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_
#define ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2cli_test_interfaces/srv/detail/short_varied_multi_nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2cli_test_interfaces
{

namespace srv
{

namespace builder
{

class Init_ShortVariedMultiNested_Request_short_varied_nested
{
public:
  Init_ShortVariedMultiNested_Request_short_varied_nested()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request short_varied_nested(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request::_short_varied_nested_type arg)
  {
    msg_.short_varied_nested = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request>()
{
  return ros2cli_test_interfaces::srv::builder::Init_ShortVariedMultiNested_Request_short_varied_nested();
}

}  // namespace ros2cli_test_interfaces


namespace ros2cli_test_interfaces
{

namespace srv
{

namespace builder
{

class Init_ShortVariedMultiNested_Response_bool_value
{
public:
  Init_ShortVariedMultiNested_Response_bool_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response bool_value(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response::_bool_value_type arg)
  {
    msg_.bool_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response>()
{
  return ros2cli_test_interfaces::srv::builder::Init_ShortVariedMultiNested_Response_bool_value();
}

}  // namespace ros2cli_test_interfaces


namespace ros2cli_test_interfaces
{

namespace srv
{

namespace builder
{

class Init_ShortVariedMultiNested_Event_response
{
public:
  explicit Init_ShortVariedMultiNested_Event_response(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event & msg)
  : msg_(msg)
  {}
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event response(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event msg_;
};

class Init_ShortVariedMultiNested_Event_request
{
public:
  explicit Init_ShortVariedMultiNested_Event_request(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event & msg)
  : msg_(msg)
  {}
  Init_ShortVariedMultiNested_Event_response request(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ShortVariedMultiNested_Event_response(msg_);
  }

private:
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event msg_;
};

class Init_ShortVariedMultiNested_Event_info
{
public:
  Init_ShortVariedMultiNested_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ShortVariedMultiNested_Event_request info(::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ShortVariedMultiNested_Event_request(msg_);
  }

private:
  ::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event>()
{
  return ros2cli_test_interfaces::srv::builder::Init_ShortVariedMultiNested_Event_info();
}

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__BUILDER_HPP_
