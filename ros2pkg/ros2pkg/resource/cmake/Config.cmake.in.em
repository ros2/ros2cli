# - Config file for the @(project_name) package
# It defines the following variables
#  @(project_name)_INCLUDE_DIRS - include directories for FooBar
#  @(project_name)_LIBRARIES    - libraries to link against
#  @(project_name)_EXECUTABLE   - the bar executable
 
# Compute paths
get_filename_component(@(project_name)_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(@(project_name)_INCLUDE_DIRS "@@CONF_INCLUDE_DIRS@@")

# Our library dependencies (contains definitions for IMPORTED targets)
include("${@(project_name)_CMAKE_DIR}/@(project_name)Targets.cmake")

# These are IMPORTED targets created by @(project_name)Targets.cmake
set(@(project_name)_LIBRARIES foo)
set(@(project_name)_EXECUTABLE bar)
