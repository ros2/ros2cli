// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2cli_test_interfaces:srv/ShortVariedMultiNested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ros2cli_test_interfaces/srv/short_varied_multi_nested.hpp"


#ifndef ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_HPP_
#define ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'short_varied_nested'
#include "ros2cli_test_interfaces/msg/detail/short_varied_nested__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request __declspec(deprecated)
#endif

namespace ros2cli_test_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShortVariedMultiNested_Request_
{
  using Type = ShortVariedMultiNested_Request_<ContainerAllocator>;

  explicit ShortVariedMultiNested_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : short_varied_nested(_init)
  {
    (void)_init;
  }

  explicit ShortVariedMultiNested_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : short_varied_nested(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _short_varied_nested_type =
    ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator>;
  _short_varied_nested_type short_varied_nested;

  // setters for named parameter idiom
  Type & set__short_varied_nested(
    const ros2cli_test_interfaces::msg::ShortVariedNested_<ContainerAllocator> & _arg)
  {
    this->short_varied_nested = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Request
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShortVariedMultiNested_Request_ & other) const
  {
    if (this->short_varied_nested != other.short_varied_nested) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShortVariedMultiNested_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShortVariedMultiNested_Request_

// alias to use template instance with default allocator
using ShortVariedMultiNested_Request =
  ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2cli_test_interfaces


#ifndef _WIN32
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response __declspec(deprecated)
#endif

namespace ros2cli_test_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShortVariedMultiNested_Response_
{
  using Type = ShortVariedMultiNested_Response_<ContainerAllocator>;

  explicit ShortVariedMultiNested_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bool_value = false;
    }
  }

  explicit ShortVariedMultiNested_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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

  // setters for named parameter idiom
  Type & set__bool_value(
    const bool & _arg)
  {
    this->bool_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Response
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShortVariedMultiNested_Response_ & other) const
  {
    if (this->bool_value != other.bool_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShortVariedMultiNested_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShortVariedMultiNested_Response_

// alias to use template instance with default allocator
using ShortVariedMultiNested_Response =
  ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2cli_test_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event __attribute__((deprecated))
#else
# define DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event __declspec(deprecated)
#endif

namespace ros2cli_test_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ShortVariedMultiNested_Event_
{
  using Type = ShortVariedMultiNested_Event_<ContainerAllocator>;

  explicit ShortVariedMultiNested_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ShortVariedMultiNested_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2cli_test_interfaces__srv__ShortVariedMultiNested_Event
    std::shared_ptr<ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ShortVariedMultiNested_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const ShortVariedMultiNested_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ShortVariedMultiNested_Event_

// alias to use template instance with default allocator
using ShortVariedMultiNested_Event =
  ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2cli_test_interfaces

namespace ros2cli_test_interfaces
{

namespace srv
{

struct ShortVariedMultiNested
{
  using Request = ros2cli_test_interfaces::srv::ShortVariedMultiNested_Request;
  using Response = ros2cli_test_interfaces::srv::ShortVariedMultiNested_Response;
  using Event = ros2cli_test_interfaces::srv::ShortVariedMultiNested_Event;
};

}  // namespace srv

}  // namespace ros2cli_test_interfaces

#endif  // ROS2CLI_TEST_INTERFACES__SRV__DETAIL__SHORT_VARIED_MULTI_NESTED__STRUCT_HPP_
