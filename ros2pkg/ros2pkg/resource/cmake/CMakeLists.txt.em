cmake_minimum_required(VERSION 3.5)
project(@(project_name))

set(@(project_name)_MAJOR_VERSION 0)
set(@(project_name)_MINOR_VERSION 0)
set(@(project_name)_PATCH_VERSION 0)
set(@(project_name)_VERSION
  ${@(project_name)_MAJOR_VERSION}.${@(project_name)_MINOR_VERSION}.${@(project_name)_PATCH_VERSION})

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
  EXPORT @(project_name)Targets
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
  EXPORT @(project_name)Targets
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
export(EXPORT @(project_name)Targets
  FILE "${PROJECT_BINARY_DIR}/@(project_name)Targets.cmake")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()
# Create the @(project_name)Config.cmake
file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}"
   "${INSTALL_INCLUDE_DIR}")
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(@(project_name)Config.cmake.in
  "${PROJECT_BINARY_DIR}/@(project_name)Config.cmake" @@ONLY)
set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
configure_file(@(project_name)Config.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake" @@ONLY)
configure_file(@(project_name)ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake" @@ONLY)
install(FILES
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake"
  "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake" COMPONENT dev)
install(EXPORT @(project_name)Targets DESTINATION
  "share/${PROJECT_NAME}/cmake" COMPONENT dev)
@[end if]@
