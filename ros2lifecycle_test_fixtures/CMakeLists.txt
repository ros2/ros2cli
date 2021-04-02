cmake_minimum_required(VERSION 3.5)

project(ros2lifecycle_test_fixtures)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_executable(simple_lifecycle_node
  src/simple_lifecycle_node.cpp)
target_link_libraries(simple_lifecycle_node
  ${rclcpp_lifecycle_LIBRARIES}
  rclcpp::rclcpp
)

install(TARGETS
  simple_lifecycle_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
