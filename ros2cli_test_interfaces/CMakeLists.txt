cmake_minimum_required(VERSION 3.5)
project(ros2cli_test_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ShortVariedMultiNested.action"
  "msg/ShortVaried.msg"
  "msg/ShortVariedMultiNested.msg"
  "msg/ShortVariedNested.msg"
  "srv/ShortVariedMultiNested.srv"
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
