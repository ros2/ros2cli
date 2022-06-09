cmake_minimum_required(VERSION 3.5)
project(netperf_tool)

# Default to C++17
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

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  # Force FindPython3 to use the debug interpreter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
_ament_cmake_python_get_python_install_dir()
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(netperf_tool_interfaces REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(netperf_tool_impl SHARED
  src/client_node.cpp
  src/node_runner.cpp
  src/netperf_tool.cpp
  src/server_node.cpp
  src/utils.cpp
)
target_include_directories(netperf_tool_impl PRIVATE src)
target_link_libraries(netperf_tool_impl PRIVATE
    rclcpp::rclcpp
    ${netperf_tool_interfaces_TARGETS})

# Install cython modules as sub-modules of the project
install(
  TARGETS netperf_tool_impl
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
