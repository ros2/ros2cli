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

  macro(add_custom_cli_test test_name)
    # Set variables for configuring the test files.
    cmake_parse_arguments(TEST "" "CONFIG_MODULE;SETUP_DELAY;TIMEOUT" "" ${ARGN})
    # Name for test cases
    set(TEST_NAME ${test_name})
    # RMW implementation for test cases
    set(TEST_RMW_IMPLEMENTATION ${rmw_implementation})
    # Module carrying test configuration
    if(NOT DEFINED TEST_CONFIG_MODULE)
      set(TEST_CONFIG_MODULE ${test_name}_config)
    endif()
    # Setup delay for test fixture actions
    if(NOT DEFINED TEST_SETUP_DELAY)
      set(TEST_SETUP_DELAY 2.0)
      if("${TEST_RMW_IMPLEMENTATION}" STREQUAL "rmw_connext_cpp")
        # Connext startup is too slow. It needs a few extra seconds till
        # discovery completes.
        set(TEST_SETUP_DELAY 5.0)
      endif()
    endif()
    # Timeout for test itself
    if(NOT DEFINED TEST_TIMEOUT)
      set(TEST_TIMEOUT 0)
    endif()



    configure_file(
      test/test_cli.py.in
      ${test_name}${target_suffix}.py.genexp
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.genexp"
    )
    install(
      FILES test/${TEST_CONFIG_MODULE}.py
      DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
    )
    add_ros_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET ${test_name}${target_suffix}
      ENV RMW_IMPLEMENTATION=${rmw_implementation}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT ${TEST_TIMEOUT}
    )
    list(
      APPEND generated_python_files
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
    )
  endmacro()

  set(generated_python_files)
  macro(custom_cli_tests)
    add_custom_cli_test(
      test_ros2action
      CONFIG_MODULE ros2action_test_configuration
      TIMEOUT 300)
    add_custom_cli_test(
      test_ros2topic
      CONFIG_MODULE ros2topic_test_configuration
      TIMEOUT 300)
    add_custom_cli_test(
      test_ros2service
      CONFIG_MODULE ros2service_test_configuration
      TIMEOUT 300)
    add_custom_cli_test(
      test_ros2node
      CONFIG_MODULE ros2node_test_configuration
      TIMEOUT 240)
    add_custom_cli_test(
      test_ros2msg
      CONFIG_MODULE ros2msg_test_configuration
      TIMEOUT 240)
    add_custom_cli_test(
      test_ros2srv
      CONFIG_MODULE ros2srv_test_configuration
      TIMEOUT 240)
    add_custom_cli_test(
      test_ros2interface
      CONFIG_MODULE ros2interface_test_configuration
      TIMEOUT 240)
  endmacro()
  install(
    FILES
      test/cli_test_configuration.py
      test/fixture_actions.py
    DESTINATION "${CMAKE_CURRENT_BINARY_DIR}"
  )

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(custom_cli_tests)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    TESTNAME "flake8_generated_launch"
    # the generated code might contain longer lines for templated types
    MAX_LINE_LENGTH 999
    ${generated_python_files}
  )
endif()

ament_package()
