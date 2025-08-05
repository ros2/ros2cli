// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2cli_test_interfaces:msg/ShortVariedNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/msg/short_varied_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_HPP_
#define ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'short_varied'
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2cli_test_interfaces__msg__ShortVariedNested __attribute__((deprecated))
#else
# define DEPRECATED__ros2cli_test_interfaces__msg__ShortVariedNested __declspec(deprecated)
#endif

namespace ros2cli_test_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ShortVariedNested_
{
  using Type = ShortVariedNested_<ContainerAllocator>;

  explicit ShortVariedNested_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : short_varied(_init)
  {
    (void)_init;
  }

  explicit ShortVariedNested_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : short_varied(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _short_varied_type =
    ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator>;
  _short_varied_type short_varied;

  // setters for named parameter idiom
  Type & set__short_varied(
    const ros2cli_test_interfaces::msg::ShortVaried_<ContainerAllocator> & _arg)
  {
    this->short_varied = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2cli_test_interfaces__msg__ShortVariedNested
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2cli_test_interfaces__msg__ShortVariedNested
    std::shared_ptr<ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShortVariedNested_ & other) const
  {
    if (this->short_varied != other.short_varied) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShortVariedNested_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShortVariedNested_

// alias to use template instance with default allocator
using ShortVariedNested =
  ros2cli_test_interfaces::msg::ShortVariedNested_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__MSG__DETAIL__SHORT_VARIED_NESTED__STRUCT_HPP_
