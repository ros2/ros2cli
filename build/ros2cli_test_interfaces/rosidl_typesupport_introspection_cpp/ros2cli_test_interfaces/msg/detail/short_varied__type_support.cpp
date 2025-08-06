// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros2cli_test_interfaces:msg/ShortVaried.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__functions.h"
#include "ros2cli_test_interfaces/msg/detail/short_varied__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros2cli_test_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ShortVaried_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros2cli_test_interfaces::msg::ShortVaried(_init);
}

void ShortVaried_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros2cli_test_interfaces::msg::ShortVaried *>(message_memory);
  typed_message->~ShortVaried();
}

size_t size_function__ShortVaried__bool_values(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__ShortVaried__bool_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__ShortVaried__bool_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__ShortVaried__bool_values(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ShortVaried_message_member_array[2] = {
  {
    "bool_value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2cli_test_interfaces::msg::ShortVaried, bool_value),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bool_values",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    true,  // is upper bound
    offsetof(ros2cli_test_interfaces::msg::ShortVaried, bool_values),  // bytes offset in struct
    nullptr,  // default value
    size_function__ShortVaried__bool_values,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__ShortVaried__bool_values,  // fetch(index, &value) function pointer
    assign_function__ShortVaried__bool_values,  // assign(index, value) function pointer
    resize_function__ShortVaried__bool_values  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ShortVaried_message_members = {
  "ros2cli_test_interfaces::msg",  // message namespace
  "ShortVaried",  // message name
  2,  // number of fields
  sizeof(ros2cli_test_interfaces::msg::ShortVaried),
  false,  // has_any_key_member_
  ShortVaried_message_member_array,  // message members
  ShortVaried_init_function,  // function to initialize message memory (memory has to be allocated)
  ShortVaried_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ShortVaried_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ShortVaried_message_members,
  get_message_typesupport_handle_function,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_hash,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description,
  &ros2cli_test_interfaces__msg__ShortVaried__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros2cli_test_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros2cli_test_interfaces::msg::ShortVaried>()
{
  return &::ros2cli_test_interfaces::msg::rosidl_typesupport_introspection_cpp::ShortVaried_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros2cli_test_interfaces, msg, ShortVaried)() {
  return &::ros2cli_test_interfaces::msg::rosidl_typesupport_introspection_cpp::ShortVaried_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
