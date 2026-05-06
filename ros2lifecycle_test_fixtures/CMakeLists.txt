cmake_minimum_required(VERSION 3.20)

project(ros2lifecycle_test_fixtures)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_executable(simple_lifecycle_node
  src/simple_lifecycle_node.cpp)

target_link_libraries(simple_lifecycle_node PRIVATE
  rclcpp_lifecycle::rclcpp_lifecycle
  ament_cmake_ros_core::ament_ros_defaults)

install(TARGETS
  simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
