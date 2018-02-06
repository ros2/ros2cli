cmake_minimum_required(VERSION 3.5)
project(@(project_name))

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
@[if cpp_library_name]@
find_package(ament_cmake_ros REQUIRED)
@[end if]@
@[if dependencies]@
@[  for dep in dependencies]@
find_package(@dep REQUIRED)
@[  end for]@
@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
@[end if]@
@[if cpp_library_name]@

add_library(${PROJECT_NAME} src/@(cpp_library_name))
target_include_directories(${PROJECT_NAME} PUBLIC include)
@[  if dependencies]@
ament_target_dependencies(
  ${PROJECT_NAME}
@[    for dep in dependencies]@
  "@(dep)"
@[    end for]@
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "@(project_name.upper())_BUILDING_LIBRARY")
@[  end if]@

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
@[end if]@
@[if cpp_node_name]@

add_executable(${PROJECT_NAME}_node src/@(cpp_node_name))
target_include_directories(${PROJECT_NAME}_node PUBLIC include)
@[  if dependencies]@
ament_target_dependencies(
  ${PROJECT_NAME}_node
@[    for dep in dependencies]@
  "@(dep)"
@[    end for]@
)
@[  end if]@

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
@[end if]@

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
@[if cpp_library_name]@

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
@[end if]@

ament_package()
