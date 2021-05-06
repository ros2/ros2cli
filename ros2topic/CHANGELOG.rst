^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* QoS autodetection. (`#613 <https://github.com/ros2/ros2cli/issues/613>`_)
  * Added autodetection
  * Changes based con review
  * Fixed linter, removed comments
  * Fixed bug, changes based on review
  * Fixed bug
  * Changes based on review
  * Changes based on review
  * Fixed style
* Make Lost Messages option ON by default. (`#633 <https://github.com/ros2/ros2cli/issues/633>`_)
  * lost messages option ON by default
  * Changes based on review. Removed warning on lost_messages not supported by implementation
  * Removed import
  * changes based on review
* Contributors: Gonzo

0.14.0 (2021-04-26)
-------------------

0.13.0 (2021-04-06)
-------------------
* Add verbose info for topic list. (`#351 <https://github.com/ros2/ros2cli/issues/351>`_)
  * Add help argument for verbose in topic list.
  * Able to show verbose info of topic list.
  * Refactor the code and use function instead.
  * Use underscore to replace unused variables
  Co-Authored-By: Claire Wang <clairewang@openrobotics.org>
  * Use the same indentation for consistency
  Co-Authored-By: Claire Wang <clairewang@openrobotics.org>
  * Use NodeStrategy instead of DirectNode for showing topic list.
  We also need to register functions for NodeStrategy: count_publishers and
  count_subscribers.
  * Remove blank line.
  * Add test for topic list verbose.
  * Restruct the code.
  * Restruct the code.
  Co-authored-by: Claire Wang <clairewang@openrobotics.org>
* Contributors: ChenYing Kuo

0.12.0 (2021-03-02)
-------------------
* 0.12.0
* remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* add option to support use_sim_time. (`#581 <https://github.com/ros2/ros2cli/issues/581>`_)
  * add option to support use_sim_time.
  * requirement is delay, not bandwidth.
  * add test for DirectNode Class.
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang, Tomoya Fujita

0.11.0 (2021-01-25)
-------------------

0.10.1 (2020-12-08)
-------------------
* Add filter option to ros2topic . (`#575 <https://github.com/ros2/ros2cli/issues/575>`_)
  * Refactor ros2topic echo verb implementation
  * Move subscription functions into class for easier reuse arguments.
  * Prefix functions with an underscore to indicate they are private.
  * Add filter option to ros2topic
  A new '--filter' option makes it easy to echo only part of a message.
  For example, to only echo the position part of an odometry message:
  ros2 topic echo /odom --filter pose/pose/position
  The filter option takes one value that must not be empty (or only forward slashes '/').
  * Change option name and syntax
  Option is now named '--field' and the '.' character is used as the delimiter.
  Also updated the help message with an example and added a test for accessing a nested field.
* Contributors: Jacob Perron

0.10.0 (2020-11-02)
-------------------
* Update deprecated qos policy value names. (`#571 <https://github.com/ros2/ros2cli/issues/571>`_)
* update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
  * update maintainers
  * add authors, update setup.py
  * remove trailing whitespace
* Fix the test in here to use the topic name. (`#566 <https://github.com/ros2/ros2cli/issues/566>`_)
* improve the error message for invalid message types. (`#558 <https://github.com/ros2/ros2cli/issues/558>`_)
  * improve the error message for invalid message types
  * also catch exception for detected message types
* Use reliable QoS for ros2topic tests. (`#555 <https://github.com/ros2/ros2cli/issues/555>`_)
  The TestROS2TopicCLI tests perform feature testing of the ros2topic
  command line interface. If the system is under stress during these
  tests, messages may be lost (by design). If that happens, there is a
  fairly high likelihood that the test_topic_pub_once test will fail
  because there is only one opportunity for the message to be successfully
  transported. We're likely dropping other messages in this suite, but the
  other tests continuously publish until one of the messages is received
  (or a timeout occurs), making them significantly less likely to fail.
  Using the 'reliable' setting for QoS reliability seems to make the tests
  consistently pass, even when the system is placed under additional
  stress.
* [ros2topic] Add option to echo serialized messages. (`#470 <https://github.com/ros2/ros2cli/issues/470>`_)
  I found this feature useful when doing some debugging.
* Enable --no-daemon flag for some cli tools. (`#514 <https://github.com/ros2/ros2cli/issues/514>`_)
  Fixes `#511 <https://github.com/ros2/ros2cli/issues/511>`_
* use transient_local and longer keep-alive for pub tests. (`#546 <https://github.com/ros2/ros2cli/issues/546>`_)
  * use transient_local and longer keep-alive for pub tests
  * add comment to document unit of --keep-alive
* add --keep-alive option to 'topic pub'. (`#544 <https://github.com/ros2/ros2cli/issues/544>`_)
* Add option to ros2 topic echo to report lost messages. (`#542 <https://github.com/ros2/ros2cli/issues/542>`_)
* support QoS Depth and History via ros2 topic pub/echo. (`#528 <https://github.com/ros2/ros2cli/issues/528>`_)
  * support QoS Depth and History via ros2 topic pub/echo.
  Also keep depth at least 1 with transient_local setting.
* Contributors: Chris Lalancette, Claire Wang, Dereck Wonnacott, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Scott K Logan, tomoya

0.9.5 (2020-06-01)
------------------
* Guard against passing None to rclpy subscriber. (`#520 <https://github.com/ros2/ros2cli/issues/520>`_)
  * Guard against passing None to rclpy subscriber
  Fixes `#519 <https://github.com/ros2/ros2cli/issues/519>`_
  * Add regression test
* Contributors: Jacob Perron

0.9.4 (2020-05-26)
------------------
* Use consistent quotes in help messages. (`#517 <https://github.com/ros2/ros2cli/issues/517>`_)
  Using single quotes inside double quotes is consistent with the other CLI help messages.
* Fix typo in `ros2 topic delay` help. (`#510 <https://github.com/ros2/ros2cli/issues/510>`_)
* Contributors: Audrow Nash, Jacob Perron

0.9.3 (2020-05-13)
------------------
* Make CLI more robust to discovery latency. (`#494 <https://github.com/ros2/ros2cli/issues/494>`_)
* Contributors: Michel Hidalgo

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------
* Fix expectation of "Incompatible QoS" messages in unit test. (`#496 <https://github.com/ros2/ros2cli/issues/496>`_)
* Contributors: Miaofei Mei

0.9.0 (2020-04-29)
------------------
* Implement times for ros2 topic pub. (`#491 <https://github.com/ros2/ros2cli/issues/491>`_)
  * Implement times for ros2 topic pub
* Stop using 'node_name' and 'node_namespace' in tests. (`#498 <https://github.com/ros2/ros2cli/issues/498>`_)
  They are both deprecated, and print warnings in CI like:
  Warning: The parameter 'node_name' is deprecated, use 'name' instead
* [ros2topic bw] Monotonic clock, units, fstring. (`#455 <https://github.com/ros2/ros2cli/issues/455>`_)
  * Use monotonic clock to avoid system time jumps
  * Fix units on message sizes
  * Make bw message easier to understand
  * Use f strings
  * Add back []
  * Update BW test regex
* Replace deprecated launch_ros usage. (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
  The Node parameter 'node_executable' has been deprecated and replaced
  with the parameter 'executable'.
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* Fix formatting of "ros2 topic info -v" output. (`#473 <https://github.com/ros2/ros2cli/issues/473>`_)
  * fix formatting of "ros2 topic info -v" output
  * improve test strictness vertical spacing of "ros2 topic info -v" output
* Added incompatible event support to ros2 topic echo and ros2 topic pub. (`#410 <https://github.com/ros2/ros2cli/issues/410>`_)
  Co-authored-by: Miaofei <miaofei@amazon.com>
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions. (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* used get_available_rmw_implementations from rclpy. (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon. (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
  This is to give time for discovery to happen between the daemon node and the test fixture nodes.
* Enhanced ros2 topic info to display node name, node namespace, topic type and qos profile of the publishers and subscribers. (`#385 <https://github.com/ros2/ros2cli/issues/385>`_)
  Co-authored-by: Miaofei Mei <ameision@hotmail.com>
* use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * use f-string
  * remove unused variable
* only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * extend API to exclude extensions from loading
  * add add_subparsers_on_demand() function
  * update all extensions to use the new API
  * deprecate old API, add deprecation warnings
* Add support for showing info of hidden topic. (`#423 <https://github.com/ros2/ros2cli/issues/423>`_)
* [ros2topic] Use import message logic provided by rosidl_runtime_py. (`#415 <https://github.com/ros2/ros2cli/issues/415>`_)
  Connects to `#218 <https://github.com/ros2/ros2cli/issues/218>`_.
  Note that the action feedback logic in the echo verb was incorrect, resulting in a ModuleImportError.
  The new logic added in https://github.com/ros2/rosidl_runtime_py/pull/9 should fix the error.
* Use imperative mood in constructor docstring. (`#422 <https://github.com/ros2/ros2cli/issues/422>`_)
* Add timestamp to ros2topic test where needed. (`#416 <https://github.com/ros2/ros2cli/issues/416>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo, Chris Lalancette, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Miaofei Mei, Peter Baughman, Shane Loretz, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------
* [ros2topic] show default values for --qos-* options. (`#400 <https://github.com/ros2/ros2cli/issues/400>`_)
* fix new linter warnings as of flake8-comprehensions 3.1.0. (`#399 <https://github.com/ros2/ros2cli/issues/399>`_)
* Contributors: Dirk Thomas

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Fix ros2 topic pub --node-name. (`#398 <https://github.com/ros2/ros2cli/issues/398>`_)
  * Fix ros2 topic pub --node-name
  * Give DirectNode node_name kwarg
  * not node_name -> node_name is None
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* Merge pull request `#396 <https://github.com/ros2/ros2cli/issues/396>`_ from ros2/BMarchi/assert_from_output_tests
  [ros2topic] Assert on listener node output for ros2topic cli test
* Assert on listener node output for ros2topic cli test
* Fix ros2topic test_echo_pub.py test suite. (`#384 <https://github.com/ros2/ros2cli/issues/384>`_)
* [ros2topic] make info verb display the type of the topic. (`#379 <https://github.com/ros2/ros2cli/issues/379>`_)
* Contributors: Brian Ezequiel Marchi, Brian Marchi, Michael Carroll, Michel Hidalgo, Mikael Arguedas

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Refactor test_echo_pub.py pytest into a launch test. (`#377 <https://github.com/ros2/ros2cli/issues/377>`_)
* End-to-end test coverage for CLI commands output. (`#304 <https://github.com/ros2/ros2cli/issues/304>`_)
  * Add end-to-end CLI output tests for ros2:
  - ros2action
  - ros2service
  - ros2topic
  - ros2msg
  - ros2srv
  - ros2interface
  - ros2node
  - ros2pkg
* [ros2topic] Add test timeout for tests using subprocess. (`#374 <https://github.com/ros2/ros2cli/issues/374>`_)
  In case a subprocess hangs, then we are not waiting forever.
* Move rosidl implementation details to rosidl_runtime_py. (`#371 <https://github.com/ros2/ros2cli/issues/371>`_)
  * Move rosidl implementation details to rosidl_runtime_py
  This resolves several TODOs.
  Here is the PR moving the related functions to their new home: https://github.com/ros2/rosidl_runtime_py/pull/3
  * Remove dependencies on deprecated packages
  * Remove obsolete test
  * make linters happy :)
* Expose qos durability and reliability to ros2topic echo. (`#283 <https://github.com/ros2/ros2cli/issues/283>`_)
  * Expose durability, reliability, and preset profile QoS options to 'topic echo'.
  Also add pytests for 'topic echo' and 'topic pub' to prevent future regressions against these new features
  * Simplify echo and pub tests to not use a timer, explicitly specify timeout parameter for subprocess calls
  * Patch stdin for windows test, and increase echo timeout for arm build
  * Disable tests for now on Windows until we figure out a proper workaround
* Contributors: Emerson Knapp, Jacob Perron, Michel Hidalgo, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* Contributors: Michael Carroll

0.8.0 (2019-09-26)
------------------
* install resource marker file for packages. (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Pass keyword arguments by name. (`#317 <https://github.com/ros2/ros2cli/issues/317>`_)
* add topic pub prototype completer. (`#299 <https://github.com/ros2/ros2cli/issues/299>`_)
* Fix ros2 topic bw output units. (`#306 <https://github.com/ros2/ros2cli/issues/306>`_)
* Add no_str and no_arr options for ros2 topic echo. (`#216 <https://github.com/ros2/ros2cli/issues/216>`_)
  * Add no_str and no_arr options for ros2 topic echo
  * Modify argument help
* print all types. (`#275 <https://github.com/ros2/ros2cli/issues/275>`_)
* Add 'topic find' verb. (`#271 <https://github.com/ros2/ros2cli/issues/271>`_)
  * add 'topic find' verb
  * alphabetical order
  * use TopicTypeCompleter
  * replace TopicTypeCompleter with message_type_completer
* add 'topic type' verb. (`#272 <https://github.com/ros2/ros2cli/issues/272>`_)
  * add 'topic type' verb
  * fix doc
  * add func return code
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Michel Hidalgo, Vinnam Kim

0.7.4 (2019-05-29)
------------------
* only allow window sizes of 1 and higher. (`#252 <https://github.com/ros2/ros2cli/issues/252>`_)
* use system_default as qos for ros2 topic pub. (`#245 <https://github.com/ros2/ros2cli/issues/245>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.7.3 (2019-05-20)
------------------
* Use rclpy qos name translations instead of defining here. (`#240 <https://github.com/ros2/ros2cli/issues/240>`_)
  * Use rclpy qos name translations instead of defining here
  * Use revised name mapping APIs
* [ros2topic] Update pub to use qos command line settings. (`#238 <https://github.com/ros2/ros2cli/issues/238>`_)
  * Update pub to use qos command line settings.
  * Clean up logic, remove type=str, add comment.
  * Address deprecation warnings.
* [ros2topic] Handle multiple namespace parts in message type. (`#237 <https://github.com/ros2/ros2cli/issues/237>`_)
  Fixes `#235 <https://github.com/ros2/ros2cli/issues/235>`_.
  Now the 'bw', 'hz', and 'delay' verbs work again.
* Fix deprecation warnings. (`#234 <https://github.com/ros2/ros2cli/issues/234>`_)
* Contributors: Emerson Knapp, Jacob Perron, Michael Carroll

0.7.2 (2019-05-08)
------------------
* separate the yaml of messages with three dashes. (`#230 <https://github.com/ros2/ros2cli/issues/230>`_)
* add xmllint linter test. (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * add xmllint test to ament_python packages
  * cover new packages as well
* Remove unused test dependency
* Contributors: Dirk Thomas, Jacob Perron, Mikael Arguedas

0.7.1 (2019-04-17)
------------------
* Port rostopic bw. (`#190 <https://github.com/ros2/ros2cli/issues/190>`_)
  * Copy original file for ros2topic bw porting
  This file is originally copied from: https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init_\_.py
  * add rostopic bw original file link
  * port rostopic bw to ros2topic
  enable ros2topic bw to display bandwidth used by topic.
* Contributors: Chris Ye

0.7.0 (2019-04-14)
------------------
* Use migrated message utility functions
  These functions are more generally useful outside of ros2topic and so they have been moved to rosidl_runtime_py.
* use safe_load instead of deprecated load. (`#212 <https://github.com/ros2/ros2cli/issues/212>`_)
* support array.array and numpy.ndarray field types. (`#211 <https://github.com/ros2/ros2cli/issues/211>`_)
* duplicate --include-hidden-topics in list verb. (`#196 <https://github.com/ros2/ros2cli/issues/196>`_)
* Contributors: Dirk Thomas, Jacob Perron, Mikael Arguedas

0.6.3 (2019-02-08)
------------------
* Fix overindentation flake8 error. (`#192 <https://github.com/ros2/ros2cli/issues/192>`_)
* Consistent node naming. (`#158 <https://github.com/ros2/ros2cli/issues/158>`_)
  * Support for easy integration with ros2 security features by starting CLI nodes with a consistent prefix.
  * Removing unneeded comment
  * Making DirectNode visible (removing hidden node prefix) to have consistent node naming for ros2cli.
  * Start all CLI nodes as hidden.
  * Shortening the default CLI node name prefix from '_ros2cli_node' to '_ros2cli'
  * Importing HIDDEN_NODE_PREFIX from rclpy, renaming CLI_NODE_NAME_PREFIX -> NODE_NAME_PREFIX.
  * ros2node - Importing HIDDEN_NODE_PREFIX from rclpy
  * Linter fixes.
* Contributors: AAlon, Shane Loretz

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* delay/hz/pub/echo work with action feedback topic
* Fix delay/echo/hz with hidden topics
  hz, delay, echo always check hidden topics
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* move get_msg_class to API module
  The two methods get_msg_class and _get_msg_class are both used in delay and hz module, avoid cop-n-paste the code but move it into the api module and reuse it in both locations.
* Small changes to optimize code
  * remove confused "string to" on help
  * move import to the top of the file
  * use local variable instead of multiple funcion call.
* Major function changes for hz cmd porting
  * remove irrelevant code and reserve hz related code
  * port rostopic hz to ros2topic based on ROS2 API format
* add ros2topic hz original file link
* copy original code for ros2topic hz porting
  Copy file from ROS1 and port to ros2. This file is originally from: https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init_\_.py
* port rostopic delay to ros2topic
  * remove irrelevant code and reserve hz code (ros has only one __init_\_.py file include all topic commands, ros2 has splitted commands to isolated file)
  * major functional changes of delay cmd with ROS2 API
  * update license format to pass test_copyright
  * use Time duration to compute the delay
  * check window_size as non-negative integer, fix no print when set window as 1
* add rostopic delay original file link
* Copy original file for ros2topic delay porting
  This file is originally copied from: https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init_\_.py
* [ros2topic] use a timer instead of time.sleep. (`#141 <https://github.com/ros2/ros2cli/issues/141>`_)
  time.sleep will add the time the publish call takes to each cycle. Use a timer to avoid pub rate loss.
* Contributors: Chris Ye

0.5.4 (2018-08-20)
------------------
* Don't truncate dictionary keys. (`#137 <https://github.com/ros2/ros2cli/issues/137>`_)
* Fix echo sometimes printing ..... (`#135 <https://github.com/ros2/ros2cli/issues/135>`_)
* [ros2topic] add missing rclpy dependency. (`#134 <https://github.com/ros2/ros2cli/issues/134>`_)
* Fix echo for big array messages. (`#126 <https://github.com/ros2/ros2cli/issues/126>`_)
  Issue1: ros2 topic echo pointcould2(big arrays), has no response, updated the logical to make more sensible.
  a. (by default) full_length=false, truncate_length=128, then print max 128 (fix big arrays issue)
  b. pass truncate_length=X, then print max X.
  c. pass full_length=true (whatever truncate_length), then set truncate_length=None and print full_length.
  Issue2: missed truncate_length to _convert_value().
  Since truncate_length is a key argument, pass it explicitly to _convert_value()
* Contributors: Chris Ye, Mikael Arguedas, Shane Loretz

0.5.3 (2018-07-17)
------------------
* Merge pull request `#123 <https://github.com/ros2/ros2cli/issues/123>`_ from ros2/limit_printing
  [topic pub] add option to limit printing published msgs
* remove default node name
* [topic pub] add option to limit printing published msgs
* Contributors: Dirk Thomas

0.5.2 (2018-06-28)
------------------
* fix echo for nested messages. (`#119 <https://github.com/ros2/ros2cli/issues/119>`_)
  * fix echo for nested messages
  * use string representation for bytes
* Contributors: Dirk Thomas

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* pass actual node object to subscriber function. (`#116 <https://github.com/ros2/ros2cli/issues/116>`_)
* add pytest markers to linter tests
* ignore F841 from latest Pyflakes release. (`#93 <https://github.com/ros2/ros2cli/issues/93>`_)
* Info verb for ros2topic. (`#88 <https://github.com/ros2/ros2cli/issues/88>`_)
  * info verb for ros2 topic
  * Fix flake8 issues with the existing code in info.py
  * Add unit test for test_info()
  * Count publishers and subscribers in topic into
  * Add test for `topic info`
  * Fix flake8 issues.
  * Address PR feedback:
  - Update the output text
  - Rename the test topic name
  - Delete obsolete code
  * Use contextlib.redirect_stdout instead of a custom decorator
  * remove single use vars
* set zip_safe to avoid warning during installation. (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* allow to pass a node name to ros2 topic pub. (`#82 <https://github.com/ros2/ros2cli/issues/82>`_)
* print full help when no command is passed. (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Nick Medveditskov

0.4.0 (2017-12-08)
------------------
* [ros2topic] pub: add --repeat. (`#66 <https://github.com/ros2/ros2cli/issues/66>`_)
  * first shot at passing -r argument
  * [ros2topic] add once and rate parameters
  * [ros2service] add once and rate parameters
  * simplify logic, add sleepd for once publisher and remove argparse
  * fix spelling
  * format default the same as argparse does
  * format default the same as argparse does
  * move logic to the right function
  * mimic ros2topic and remove extra logic
  * consistent with services
* Merge pull request `#64 <https://github.com/ros2/ros2cli/issues/64>`_ from ros2/add_type_completer
  add type completer for 'topic pub' and 'service call'
* add type completer for 'topic pub' and 'service call'
* remove test_suite, add pytest as test_requires
* Make sure to check errors when expanding the topic name. (`#58 <https://github.com/ros2/ros2cli/issues/58>`_)
  * Make sure to check errors when expanding the topic name.
  We need to catch ValueErrors when actually doing the expansion,
  then InvalidTopicNameException when doing the validation.
  * Switch to using the string from the original exception.
* Support non-absolute topic names. (`#57 <https://github.com/ros2/ros2cli/issues/57>`_)
  * Support non-absolute topic names.
  If the user passes "/topic_name" to the ros2 echo
  command, it works properly.  If they pass "topic_name"
  to the ros2 echo command, it fails to match.  This
  change just allows us to deal with non-absolute topic
  names.
* 0.0.3
* Fix request message population. (`#56 <https://github.com/ros2/ros2cli/issues/56>`_)
  * use set_msg_fields
  * remove unused comment
  * move function and error definition to api module
  * use message filling method from ros2topic
  * alphabetical order
* Merge pull request `#48 <https://github.com/ros2/ros2cli/issues/48>`_ from ros2/improve_error_message
  better error message when passing an invalid value to ros2 topic pub
* better error message when passing an invalid value to ros2 topic pub
* use test_msgs. (`#47 <https://github.com/ros2/ros2cli/issues/47>`_)
  * use test_msgs instead of test_communication
  * remove unused message
  * test all messages with fixtures
* Merge pull request `#46 <https://github.com/ros2/ros2cli/issues/46>`_ from ros2/flake8_plugins
  update style to satisfy new flake8 plugins
* update style to satisfy new flake8 plugins
* implicitly inherit from object. (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Merge pull request `#36 <https://github.com/ros2/ros2cli/issues/36>`_ from ros2/improve_error_message
  better error message
* better error message
* update test
* truncate arrays, bytes, and strings by default, add option to show in full or use custom threshold. (`#31 <https://github.com/ros2/ros2cli/issues/31>`_)
  * truncate arrays, bytes, and strings by default, add option to show in full or use custom threshold
  * add short options
* Merge pull request `#27 <https://github.com/ros2/ros2cli/issues/27>`_ from ros2/also_catch_value_errors
  also handle ValueError nicely
* also handle ValueError nicely
* Merge pull request `#24 <https://github.com/ros2/ros2cli/issues/24>`_ from ros2/recursive_msg_population
  fix population of recursive message fields
* fix population of recursive message fields
* use yaml for parsing msg and srv values. (`#19 <https://github.com/ros2/ros2cli/issues/19>`_)
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
  various fixes and improvements
* add missing dependency on yaml
* various fixes and improvements
* revert no_demangle option until it can be fixed. (`#9 <https://github.com/ros2/ros2cli/issues/9>`_)
* Refactor get topic names and types. (`#4 <https://github.com/ros2/ros2cli/issues/4>`_)
  * ros2topic: use rclpy utility
  * ros2topic: fixup
  * ros2topic: support multiple types
  * ros2service: initial commit
  * ros2topic: support no_demangle
  * fix include order
  * missed a commit
  * ros2service: add pep257 tests
  * fix echo to support multiple types
  * improve shutdown behavior of call, add loop option
  * address comments
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
  add pep257 tests
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
  Entry point, plugin system, daemon, existing tools
* add ros2topic echo, list, pub including previous tests for yaml/csv output
* Contributors: Chris Lalancette, Dirk Thomas, Mikael Arguedas, William Woodall
