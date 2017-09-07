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
find_package(ament_cmake REQUIRED)
@[if dependencies]@
@[for deb in dependencies]@
find_package(@deb REQUIRED)
@[end for]@

@[if dependencies or create_cpp_library]@
include_directories(
@[if create_cpp_library]@
  include
@[end if]@
)

@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> [<REQUIRED>|<QUIET>])

@[end if]@
@[end if]@
@[if create_cpp_library]@
add_library(${PROJECT_NAME} src/@(cpp_library_name))

@[if dependencies]@
ament_target_dependencies(
  ${PROJECT_NAME}
@[for deb in dependencies]@
  ${@(deb)_LIBRARIES}
@[end for]@
)

@[end if]@
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

@[end if]@
@[if create_cpp_exe]@
add_executable(${PROJECT_NAME}_main src/@(cpp_exe_name))

@[if dependencies]@
ament_target_dependencies(
  ${PROJECT_NAME}_main
@[for deb in dependencies]@
  ${@(deb)_LIBRARIES}
@[end for]@
)

@[end if]@
install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME})

@[end if]@
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

@[if create_cpp_library]@
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
@[end if]@
ament_package()
