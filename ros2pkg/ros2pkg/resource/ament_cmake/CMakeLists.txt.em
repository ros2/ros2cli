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

add_library(@(cpp_library_name) src/@(cpp_library_name).cpp)
target_include_directories(@(cpp_library_name) PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
@[  if dependencies]@
ament_target_dependencies(
  @(cpp_library_name)
@[    for dep in dependencies]@
  "@(dep)"
@[    end for]@
)
@[  end if]@

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(@(cpp_library_name) PRIVATE "@(project_name.upper())_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS @(cpp_library_name)
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
@[end if]@
@[if cpp_node_name]@

add_executable(@(cpp_node_name) src/@(cpp_node_name).cpp)
target_include_directories(@(cpp_node_name) PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
@[  if cpp_library_name]@
target_link_libraries(@(cpp_node_name) @(cpp_library_name))
@[  else]@
@[    if dependencies]@
ament_target_dependencies(
  @(cpp_node_name)
@[      for dep in dependencies]@
  "@(dep)"
@[      end for]@
)
@[    end if]@
@[  end if]@

install(TARGETS @(cpp_node_name)
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
@[end if]@

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
@[if cpp_library_name]@

ament_export_include_directories(
  include
)
ament_export_libraries(
  @(cpp_library_name)
)
ament_export_targets(
  export_${PROJECT_NAME}
)
@[end if]@

ament_package()
