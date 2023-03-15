^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2topic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added changelogs
* Contributors: Dharini Dutia

0.9.12 (2022-09-12)
-------------------

0.9.11 (2022-01-31)
-------------------
* support QoS Depth and History via ros2 topic pub/echo. (`#528 <https://github.com/ros2/ros2cli/issues/528>`_) (`#674 <https://github.com/ros2/ros2cli/issues/674>`_)
* Contributors: Nikos Koukis

0.9.10 (2021-10-05)
-------------------

0.9.9 (2021-03-24)
------------------
* 0.9.9
* Contributors: Audrow Nash

0.9.8 (2020-12-08)
------------------
* [Foxy backport] add --keep-alive option to 'topic pub' (`#544 <https://github.com/ros2/ros2cli/issues/544>`_)  use transient_local and longer keep-alive for pub tests (`#546 <https://github.com/ros2/ros2cli/issues/546>`_) Use reliable QoS for ros2topic tests (`#555 <https://github.com/ros2/ros2cli/issues/555>`_) (`#565 <https://github.com/ros2/ros2cli/issues/565>`_)
* Contributors: Shane Loretz

0.9.7 (2020-07-07)
------------------

0.9.6 (2020-06-23)
------------------

0.9.5 (2020-06-01)
------------------
* Guard against passing None to rclpy subscriber (`#520 <https://github.com/ros2/ros2cli/issues/520>`_)
* Contributors: Jacob Perron

0.9.4 (2020-05-26)
------------------
* Use consistent quotes in help messages (`#517 <https://github.com/ros2/ros2cli/issues/517>`_)
* Fix typo in `ros2 topic delay` help (`#510 <https://github.com/ros2/ros2cli/issues/510>`_)
* Contributors: Audrow Nash, Jacob Perron

0.9.3 (2020-05-13)
------------------
* Make CLI more robust to discovery latency. (`#494 <https://github.com/ros2/ros2cli/issues/494>`_)
* Contributors: Michel Hidalgo

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------
* Fix expectation of "Incompatible QoS" messages in unit test (`#496 <https://github.com/ros2/ros2cli/issues/496>`_)
* Contributors: Miaofei Mei

0.9.0 (2020-04-29)
------------------
* Implement times for ros2 topic pub (`#491 <https://github.com/ros2/ros2cli/issues/491>`_)
* Stop using 'node_name' and 'node_namespace' in tests. (`#498 <https://github.com/ros2/ros2cli/issues/498>`_)
* [ros2topic bw] Monotonic clock, units, fstring (`#455 <https://github.com/ros2/ros2cli/issues/455>`_)
* Replace deprecated launch_ros usage (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* Fix formatting of "ros2 topic info -v" output (`#473 <https://github.com/ros2/ros2cli/issues/473>`_)
* Added incompatible event support to ros2 topic echo and ros2 topic pub (`#410 <https://github.com/ros2/ros2cli/issues/410>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* used get_available_rmw_implementations from rclpy (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
* Enhanced ros2 topic info to display node name, node namespace, topic type and qos profile of the publishers and subscribers. (`#385 <https://github.com/ros2/ros2cli/issues/385>`_)
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
* Add support for showing info of hidden topic (`#423 <https://github.com/ros2/ros2cli/issues/423>`_)
* [ros2topic] Use import message logic provided by rosidl_runtime_py (`#415 <https://github.com/ros2/ros2cli/issues/415>`_)
* Use imperative mood in constructor docstring. (`#422 <https://github.com/ros2/ros2cli/issues/422>`_)
* Add timestamp to ros2topic test where needed (`#416 <https://github.com/ros2/ros2cli/issues/416>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo, Chris Lalancette, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Miaofei Mei, Peter Baughman, Shane Loretz, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------
* [ros2topic] show default values for --qos-* options (`#400 <https://github.com/ros2/ros2cli/issues/400>`_)
* fix new linter warnings as of flake8-comprehensions 3.1.0 (`#399 <https://github.com/ros2/ros2cli/issues/399>`_)
* Contributors: Dirk Thomas

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Fix ros2 topic pub --node-name (`#398 <https://github.com/ros2/ros2cli/issues/398>`_)
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* Merge pull request `#396 <https://github.com/ros2/ros2cli/issues/396>`_ from ros2/BMarchi/assert_from_output_tests
* Assert on listener node output for ros2topic cli test
* Fix ros2topic test_echo_pub.py test suite (`#384 <https://github.com/ros2/ros2cli/issues/384>`_)
* [ros2topic] make info verb display the type of the topic (`#379 <https://github.com/ros2/ros2cli/issues/379>`_)
* Contributors: Brian Ezequiel Marchi, Brian Marchi, Michael Carroll, Michel Hidalgo, Mikael Arguedas

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Refactor test_echo_pub.py pytest into a launch test (`#377 <https://github.com/ros2/ros2cli/issues/377>`_)
* End-to-end test coverage for CLI commands output (`#304 <https://github.com/ros2/ros2cli/issues/304>`_)
* [ros2topic] Add test timeout for tests using subprocess (`#374 <https://github.com/ros2/ros2cli/issues/374>`_)
* Move rosidl implementation details to rosidl_runtime_py (`#371 <https://github.com/ros2/ros2cli/issues/371>`_)
* Expose qos durability and reliability to ros2topic echo (`#283 <https://github.com/ros2/ros2cli/issues/283>`_)
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
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Pass keyword arguments by name (`#317 <https://github.com/ros2/ros2cli/issues/317>`_)
* add topic pub prototype completer (`#299 <https://github.com/ros2/ros2cli/issues/299>`_)
* Fix ros2 topic bw output units. (`#306 <https://github.com/ros2/ros2cli/issues/306>`_)
* Add no_str and no_arr options for ros2 topic echo (`#216 <https://github.com/ros2/ros2cli/issues/216>`_)
* print all types (`#275 <https://github.com/ros2/ros2cli/issues/275>`_)
* Add 'topic find' verb (`#271 <https://github.com/ros2/ros2cli/issues/271>`_)
* add 'topic type' verb (`#272 <https://github.com/ros2/ros2cli/issues/272>`_)
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Michel Hidalgo, Vinnam Kim

0.7.4 (2019-05-29)
------------------
* only allow window sizes of 1 and higher (`#252 <https://github.com/ros2/ros2cli/issues/252>`_)
* use system_default as qos for ros2 topic pub (`#245 <https://github.com/ros2/ros2cli/issues/245>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.7.3 (2019-05-20)
------------------
* Use rclpy qos name translations instead of defining here (`#240 <https://github.com/ros2/ros2cli/issues/240>`_)
* [ros2topic] Update pub to use qos command line settings. (`#238 <https://github.com/ros2/ros2cli/issues/238>`_)
* [ros2topic] Handle multiple namespace parts in message type (`#237 <https://github.com/ros2/ros2cli/issues/237>`_)
* Fix deprecation warnings (`#234 <https://github.com/ros2/ros2cli/issues/234>`_)
* Contributors: Emerson Knapp, Jacob Perron, Michael Carroll

0.7.2 (2019-05-08)
------------------
* separate the yaml of messages with three dashes (`#230 <https://github.com/ros2/ros2cli/issues/230>`_)
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
* Remove unused test dependency
* Contributors: Dirk Thomas, Jacob Perron, Mikael Arguedas

0.7.1 (2019-04-17)
------------------
* Port rostopic bw (`#190 <https://github.com/ros2/ros2cli/issues/190>`_)
* Contributors: Chris Ye

0.7.0 (2019-04-14)
------------------
* Use migrated message utility functions
* use safe_load instead of deprecated load (`#212 <https://github.com/ros2/ros2cli/issues/212>`_)
* support array.array and numpy.ndarray field types (`#211 <https://github.com/ros2/ros2cli/issues/211>`_)
* duplicate --include-hidden-topics in list verb (`#196 <https://github.com/ros2/ros2cli/issues/196>`_)
* Contributors: Dirk Thomas, Jacob Perron, Mikael Arguedas

0.6.3 (2019-02-08)
------------------
* Fix overindentation flake8 error (`#192 <https://github.com/ros2/ros2cli/issues/192>`_)
* Consistent node naming (`#158 <https://github.com/ros2/ros2cli/issues/158>`_)
* Contributors: AAlon, Shane Loretz

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------
* 0.6.1
* delay/hz/pub/echo work with action feedback topic
* Fix delay/echo/hz with hidden topics
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* move get_msg_class to API module
* Small changes to optimize code
* Major function changes for hz cmd porting
* add ros2topic hz original file link
* copy original code for ros2topic hz porting
* port rostopic delay to ros2topic
* add rostopic delay original file link
* Copy original file for ros2topic delay porting
* [ros2topic] use a timer instead of time.sleep (`#141 <https://github.com/ros2/ros2cli/issues/141>`_)
* Contributors: Chris Ye

0.5.4 (2018-08-20)
------------------
* Don't truncate dictionary keys (`#137 <https://github.com/ros2/ros2cli/issues/137>`_)
* Fix echo sometimes printing ..... (`#135 <https://github.com/ros2/ros2cli/issues/135>`_)
* [ros2topic] add missing rclpy dependency (`#134 <https://github.com/ros2/ros2cli/issues/134>`_)
* Fix echo for big array messages (`#126 <https://github.com/ros2/ros2cli/issues/126>`_)
* Contributors: Chris Ye, Mikael Arguedas, Shane Loretz

0.5.3 (2018-07-17)
------------------
* Merge pull request `#123 <https://github.com/ros2/ros2cli/issues/123>`_ from ros2/limit_printing
* remove default node name
* [topic pub] add option to limit printing published msgs
* Contributors: Dirk Thomas

0.5.2 (2018-06-28)
------------------
* fix echo for nested messages (`#119 <https://github.com/ros2/ros2cli/issues/119>`_)
* Contributors: Dirk Thomas

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* pass actual node object to subscriber function (`#116 <https://github.com/ros2/ros2cli/issues/116>`_)
* add pytest markers to linter tests
* ignore F841 from latest Pyflakes release (`#93 <https://github.com/ros2/ros2cli/issues/93>`_)
* Info verb for ros2topic (`#88 <https://github.com/ros2/ros2cli/issues/88>`_)
* set zip_safe to avoid warning during installation (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* allow to pass a node name to ros2 topic pub (`#82 <https://github.com/ros2/ros2cli/issues/82>`_)
* print full help when no command is passed (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Nick Medveditskov

0.4.0 (2017-12-08)
------------------
* [ros2topic] pub: add --repeat (`#66 <https://github.com/ros2/ros2cli/issues/66>`_)
* Merge pull request `#64 <https://github.com/ros2/ros2cli/issues/64>`_ from ros2/add_type_completer
* add type completer for 'topic pub' and 'service call'
* remove test_suite, add pytest as test_requires
* Make sure to check errors when expanding the topic name. (`#58 <https://github.com/ros2/ros2cli/issues/58>`_)
* Support non-absolute topic names. (`#57 <https://github.com/ros2/ros2cli/issues/57>`_)
* 0.0.3
* Fix request message population (`#56 <https://github.com/ros2/ros2cli/issues/56>`_)
* Merge pull request `#48 <https://github.com/ros2/ros2cli/issues/48>`_ from ros2/improve_error_message
* better error message when passing an invalid value to ros2 topic pub
* use test_msgs (`#47 <https://github.com/ros2/ros2cli/issues/47>`_)
* Merge pull request `#46 <https://github.com/ros2/ros2cli/issues/46>`_ from ros2/flake8_plugins
* update style to satisfy new flake8 plugins
* implicitly inherit from object (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Merge pull request `#36 <https://github.com/ros2/ros2cli/issues/36>`_ from ros2/improve_error_message
* better error message
* update test
* truncate arrays, bytes, and strings by default, add option to show in full or use custom threshold (`#31 <https://github.com/ros2/ros2cli/issues/31>`_)
* Merge pull request `#27 <https://github.com/ros2/ros2cli/issues/27>`_ from ros2/also_catch_value_errors
* also handle ValueError nicely
* Merge pull request `#24 <https://github.com/ros2/ros2cli/issues/24>`_ from ros2/recursive_msg_population
* fix population of recursive message fields
* use yaml for parsing msg and srv values (`#19 <https://github.com/ros2/ros2cli/issues/19>`_)
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
* add missing dependency on yaml
* various fixes and improvements
* revert no_demangle option until it can be fixed (`#9 <https://github.com/ros2/ros2cli/issues/9>`_)
* Refactor get topic names and types (`#4 <https://github.com/ros2/ros2cli/issues/4>`_)
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
* add ros2topic echo, list, pub including previous tests for yaml/csv output
* Contributors: Chris Lalancette, Dirk Thomas, Mikael Arguedas, William Woodall
