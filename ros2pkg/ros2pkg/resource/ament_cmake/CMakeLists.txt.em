cmake_minimum_required(VERSION 3.8)
project(@(project_name))

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
add_library(@(project_name)::@(cpp_library_name) ALIAS @(cpp_library_name))
target_compile_features(@(cpp_library_name) PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(@(cpp_library_name) PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
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
  DESTINATION include/${PROJECT_NAME}
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
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
@[  if cpp_library_name]@
target_link_libraries(@(cpp_node_name) @(cpp_library_name))
@[  else]@
target_compile_features(@(cpp_node_name) PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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
  DESTINATION lib/${PROJECT_NAME})
@[end if]@

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
@[if cpp_library_name]@

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  @(cpp_library_name)
)
ament_export_targets(
  export_${PROJECT_NAME}
)
@[end if]@

ament_package()
