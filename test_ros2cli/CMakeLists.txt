cmake_minimum_required(VERSION 3.5)

project(test_ros2cli)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ros_testing REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)

  macro(custom_test_process_output test_name config_file_name)
    # Set variables for configuring the test files.
    set(TEST_NAME ${test_name})  # test name used in the test cases
    set(TEST_CONFIGURATION_FILE ${config_file_name})  # config file to use

    configure_file(
      test/test_process_output_customizable.py.in
      ${test_name}${target_suffix}.py.genexp
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
    )
    file(INSTALL test/${config_file_name}.py
      DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
    )
    add_ros_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET ${test_name}${target_suffix}
      ENV RMW_IMPLEMENTATION=${rmw_implementation}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 120
    )
    list(
      APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
    )
  endmacro()

  set(generated_python_files)
  macro(tests)
    custom_test_process_output(test_ros2action config_ros2action_test)
    custom_test_process_output(test_ros2msg config_ros2msg_test)
  endmacro()
  file(INSTALL test/test_config.py
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
  )

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    # the generated code might contain longer lines for templated types
    MAX_LINE_LENGTH 999
    ${generated_python_files}
  )
endif()

ament_package()
