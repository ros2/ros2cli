cmake_minimum_required(VERSION 3.5)
project(@(project_name))

set(@(project_name)_MAJOR_VERSION 0)
set(@(project_name)_MINOR_VERSION 0)
set(@(project_name)_PATCH_VERSION 0)
set(@(project_name)_VERSION
  ${@(project_name)_MAJOR_VERSION}.${@(project_name)_MINOR_VERSION}.${@(project_name)_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

@[if dependencies]@

# find dependencies
@[  for dep in dependencies]@
find_package(@dep REQUIRED)
@[  end for]@
@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
@[end if]@
@[if cpp_library_name]@

add_library(@(cpp_library_name) SHARED src/@(cpp_library_name).cpp)
target_compile_features(@(cpp_library_name) PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(@(cpp_library_name) PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
@[  if dependencies]@
@[    for dep in dependencies]@
  PUBLIC ${@(dep)_INCLUDE_DIRS}
@[    end for]@
@[  end if]@
)
@[  if dependencies]@
target_link_libraries(@(cpp_library_name)
@[    for dep in dependencies]@
 ${@(dep)_LIBRARIES}
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
  EXPORT export_@(project_name)
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
@[end if]@
@[if cpp_node_name]@

add_executable(@(cpp_node_name) src/@(cpp_node_name).cpp)
@[  if dependencies]@
target_include_directories(@(cpp_node_name) PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
@[      for dep in dependencies]@
  PUBLIC ${@(dep)_INCLUDE_DIRS}
@[      end for]@
)
@[  end if]@
@[  if cpp_library_name]@
target_link_libraries(@(cpp_node_name) @(cpp_library_name))
@[  else]@
target_compile_features(@(cpp_node_name) PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
@[    if dependencies]@
target_link_libraries(@(cpp_node_name)
@[      for dep in dependencies]@
 ${@(dep)_LIBRARIES}
@[      end for]@
)
@[    end if]@
@[  end if]@

install(
  TARGETS @(cpp_node_name)
  EXPORT export_@(project_name)
  DESTINATION lib/${PROJECT_NAME})
@[end if]@
@[if cpp_library_name or cpp_node_name]@

# export targets
@[  if cpp_library_name]@
set(export_targets ${export_targets};@(cpp_library_name))
@[  end if]@
@[  if cpp_node_name]@
set(export_targets ${export_targets};@(cpp_node_name))
@[  end if]@
export(EXPORT export_@(project_name)
  FILE "${PROJECT_BINARY_DIR}/export_@(project_name).cmake")

# Create the @(project_name)Config.cmake
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(@(project_name)Config.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake" @@ONLY)

# Create the @(project_name)ConfigVersion.cmake
configure_file(@(project_name)ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake" @@ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake"
  "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev)
install(EXPORT export_@(project_name)
  DESTINATION "share/${PROJECT_NAME}/cmake"
  FILE export_@(project_name).cmake
  COMPONENT dev)
@[end if]@
