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

# find dependencies
@[if dependencies]@
@[for deb in dependencies]@
find_package(@deb REQUIRED)
@[end for]@

include_directories(
@[if create_cpp_library]@
  include
@[end if]@
@[for deb in dependencies]@
  ${@(deb)_INCLUDE_DIRS}
@[end for]@
)

@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> [<REQUIRED>|<QUIET>])

# include_directories($<dependency>_INCLUDE_DIRS})

@[end if]@
@[if create_cpp_library]@
add_library(${PROJECT_NAME} src/@(cpp_library_name))

@[if dependencies]@
@[for deb in dependencies]@
target_link_libraries(${PROJECT_NAME} ${@(deb)_LIBRARIES})
@[end for]@

@[end if]@
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

@[end if]@
@[if create_cpp_exe]@
add_executable(${PROJECT_NAME}_main src/@(cpp_exe_name))

@[if dependencies]@
@[for deb in dependencies]@
target_link_libraries(${PROJECT_NAME}_main ${@(deb)_LIBRARIES})
@[end for]@

@[end if]@
install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME})

@[end if]@
if(BUILD_TESTING)
# testing specific code
endif()

@[if create_cpp_library or create_cpp_exe]@
@[if create_cpp_library]@
set(export_targets ${export_targets};${PROJECT_NAME})
@[end if]@
@[if create_cpp_exe]@
set(export_targets ${export_targets};${PROJECT_NAME}_main)
@[end if]@
export(TARGETS ${export_targets}
  FILE "${PROJECT_BINARY_DIR}/@(project_name)Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE @(project_name))
# Create the @(project_name)Config.cmake
# file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}"
#    "${INSTALL_INCLUDE_DIR}")
# set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
# configure_file(@(project_name)Config.cmake.in
#   "${PROJECT_BINARY_DIR}/@(project_name)Config.cmake" @@ONLY)
# set(CONF_INCLUDE_DIRS "\${@(project_name)_CMAKE_DIR}/${REL_INCLUDE_DIR}")
# configure_file(@(project_name)Config.cmake.in
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake" @@ONLY)
# configure_file(@(project_name)ConfigVersion.cmake.in
#   "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake" @@ONLY)
# install(FILES
#   "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/@(project_name)Config.cmake"
#   "${PROJECT_BINARY_DIR}/@(project_name)ConfigVersion.cmake"
#   DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)
# install(EXPORT @(project_name)Targets DESTINATION
#   "${INSTALL_CMAKE_DIR}" COMPONENT dev)
@[end if]@
