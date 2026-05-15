cmake_minimum_required(VERSION 3.20)
project(@(project_name))

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
@[if dependencies]@
@[  for dep in dependencies]@
find_package(@dep REQUIRED)
@[  end for]@
@[else]@
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
@[end if]@

ament_python_install_package(${PROJECT_NAME})
@[if node_name]@

install(PROGRAMS
  scripts/@(node_name)
  DESTINATION lib/${PROJECT_NAME}
)
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

ament_package()
