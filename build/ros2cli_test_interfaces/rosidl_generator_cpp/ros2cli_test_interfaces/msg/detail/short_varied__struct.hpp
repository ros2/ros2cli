// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2cli_test_interfaces__msg__ShortVaried __attribute__((deprecated))
#else
# define DEPRECATED__ros2cli_test_interfaces__msg__ShortVaried __declspec(deprecated)
#endif

namespace ros2cli_test_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ShortVaried_
{
  using Type = ShortVaried_<ContainerAllocator>;

  explicit ShortVaried_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bool_value = false;
    }
  }

  explicit ShortVaried_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bool_value = false;
    }
  }

  // field types and members
  using _bool_value_type =
    bool;
  _bool_value_type bool_value;
  using _bool_values_type =
    rosidl_runtime_cpp::BoundedVector<bool, 3, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _bool_values_type bool_values;

  // setters for named parameter idiom
  Type & set__bool_value(
    const bool & _arg)
  {
    this->bool_value = _arg;
    return *this;
  }
  Type & set__bool_values(
    const rosidl_runtime_cpp::BoundedVector<bool, 3, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->bool_values = _arg;
    return *this;
  }

  // constant declarations
  static constexpr bool BOOL_CONST =
    1;

  // pointer types
  using RawPtr =
    ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2cli_test_interfaces__msg__ShortVaried
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2cli_test_interfaces__msg__ShortVaried
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShortVaried_ & other) const
  {
    if (this->bool_value != other.bool_value) {
      return false;
    }
    if (this->bool_values != other.bool_values) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShortVaried_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShortVaried_

// alias to use template instance with default allocator
using ShortVaried =
  ros2cli_test_interfaces::msg::ShortVaried_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr bool ShortVaried_<ContainerAllocator>::BOOL_CONST;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED__STRUCT_HPP_
