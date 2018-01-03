# - Config file for the @(project_name) package
# It defines the following variables
#  @(project_name)_INCLUDE_DIRS - include directories for FooBar
#  @(project_name)_LIBRARIES    - libraries to link against
#  @(project_name)_EXECUTABLE   - the bar executable

set(@(project_name)_INCLUDE_DIRS "@@CONF_INCLUDE_DIRS@@")

# Our library dependencies (contains definitions for IMPORTED targets)
include("${@(project_name)_DIR}/@(project_name)Targets.cmake")

# These are IMPORTED targets created by @(project_name)Targets.cmake
@[if create_cpp_library]@
set(@(project_name)_LIBRARIES @(project_name))
@[end if]
@[if create_cpp_exe]@
set(@(project_name)_EXECUTABLE @(project_name)_node)
@[end if]
