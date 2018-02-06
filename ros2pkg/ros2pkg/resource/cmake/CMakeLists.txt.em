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
@[end if]@
@[if cpp_library_name]@

add_library(${PROJECT_NAME} SHARED src/@(cpp_library_name))
@[if dependencies]@
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
@[  for dep in dependencies]@
  PUBLIC ${@(dep)_INLCUDE_DIRS}
@[  end for]@
)
@[    for dep in dependencies]@
target_link_libraries(${PROJECT_NAME} ${@(dep)_LIBRARIES})
@[    end for]@
@[else]@
  target_include_directories(${PROJECT_NAME} PUBLIC include)
@[end if]@

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT @(project_name)Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
@[end if]@
@[if cpp_node_name]@

add_executable(${PROJECT_NAME}_node src/@(cpp_node_name))
@[  if dependencies]@
@[    for dep in dependencies]@
target_link_libraries(${PROJECT_NAME}_node ${@(dep)_LIBRARIES})
@[    end for]@
@[  end if]@

install(TARGETS ${PROJECT_NAME}_node
  EXPORT @(project_name)Targets
  DESTINATION lib/${PROJECT_NAME})
@[end if]@
@[if cpp_library_name or cpp_node_name]@

# export targets
@[  if cpp_library_name]@
set(export_targets ${export_targets};${PROJECT_NAME})
@[  end if]@
@[  if cpp_node_name]@
set(export_targets ${export_targets};${PROJECT_NAME}_node)
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
