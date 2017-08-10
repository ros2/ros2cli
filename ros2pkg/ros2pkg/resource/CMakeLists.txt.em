cmake_minimum_required(VERSION 3.5)
project(@(project_name))

@[for deb in dependencies]find_package(@deb REQUIRED)
@[end for]

include_directories(
@[for deb in dependencies]  @(deb)_INCLUDE_DIRS
@[end for]
)

@[if create_cpp_exe]add_executable(${PROJECT_NAME} src/@(cpp_exe_name))
@[for deb in dependencies]target_link_libraries(${PROJECT_NAME} ${@(deb)_LIBRARIES})
@[end for]
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
@[end if]

ament_package()
