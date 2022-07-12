cmake_minimum_required(VERSION 3.5)
project(netperf_tool_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # About -Wno-sign-conversion: With Clang, -Wconversion implies -Wsign-conversion. There are a number of
  # implicit sign conversions in rclcpp and gtest.cc, see https://ci.ros2.org/job/ci_osx/9265/.
  # Hence disabling -Wsign-conversion for now until all those have eventually been fixed.
  # (from https://github.com/ros2/rclcpp/pull/1188#issuecomment-650229140)
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/Bytes.msg"
  "srv/GetResults.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
