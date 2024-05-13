^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2service
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.32.0 (2024-04-16)
-------------------

0.31.2 (2024-03-27)
-------------------

0.31.1 (2024-02-07)
-------------------

0.31.0 (2024-01-24)
-------------------

0.30.1 (2023-12-26)
-------------------

0.30.0 (2023-11-06)
-------------------
* Add ros2 service info (`#771 <https://github.com/ros2/ros2cli/issues/771>`_)
* Contributors: Minju, Lee

0.29.1 (2023-10-04)
-------------------

0.29.0 (2023-08-21)
-------------------
* Load a message/request/goal from standard input (`#844 <https://github.com/ros2/ros2cli/issues/844>`_)
* Contributors: ymd-stella

0.28.0 (2023-07-11)
-------------------
* Fix tests with get_type_description service and param present (`#838 <https://github.com/ros2/ros2cli/issues/838>`_)
* Contributors: Emerson Knapp

0.27.0 (2023-06-07)
-------------------
* [service introspection] ros2 service echo (`#745 <https://github.com/ros2/ros2cli/issues/745>`_)
* Contributors: Brian

0.26.1 (2023-05-11)
-------------------

0.26.0 (2023-04-28)
-------------------

0.25.0 (2023-04-18)
-------------------
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`_)
* Contributors: Chris Lalancette

0.24.1 (2023-04-12)
-------------------

0.24.0 (2023-04-11)
-------------------

0.23.0 (2023-03-02)
-------------------

0.22.0 (2023-02-14)
-------------------
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`_)
* Contributors: Audrow Nash

0.21.0 (2022-11-02)
-------------------

0.20.0 (2022-09-13)
-------------------

0.19.0 (2022-04-29)
-------------------

0.18.3 (2022-04-08)
-------------------

0.18.2 (2022-03-30)
-------------------
* Add timeout to kill hanging tests (`#701 <https://github.com/ros2/ros2cli/issues/701>`_)
* Contributors: Audrow Nash

0.18.1 (2022-03-28)
-------------------
* Also provide --include-hidden-services for `ros2 service list` verb (`#551 <https://github.com/ros2/ros2cli/issues/551>`_)
* Contributors: Karsten Knese

0.18.0 (2022-03-01)
-------------------

0.17.1 (2022-01-25)
-------------------

0.17.0 (2022-01-25)
-------------------

0.16.1 (2022-01-14)
-------------------

0.16.0 (2022-01-14)
-------------------
* Depend on launch packages instead of ros_testing to avoid circular dependency (`#685 <https://github.com/ros2/ros2cli/issues/685>`_)
* Contributors: Shane Loretz

0.15.0 (2021-11-18)
-------------------
* Update maintainers to Aditya Pande, Audrow Nash, and Michael Jeronimo (`#673 <https://github.com/ros2/ros2cli/issues/673>`_)
* Updated maintainers (`#670 <https://github.com/ros2/ros2cli/issues/670>`_)
* Add changelogs (`#635 <https://github.com/ros2/ros2cli/issues/635>`_)
* Contributors: Aditya Pande, Audrow Nash, Ivan Santiago Paunovic

0.14.0 (2021-04-26)
-------------------

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* Remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang

0.11.0 (2021-01-25)
-------------------

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* Update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
* Check that passed type is actually a service. (`#559 <https://github.com/ros2/ros2cli/issues/559>`_)
* Contributors: Claire Wang, Dirk Thomas

0.9.5 (2020-06-01)
------------------

0.9.4 (2020-05-26)
------------------
* Use consistent quotes in help messages. (`#517 <https://github.com/ros2/ros2cli/issues/517>`_)
  Using single quotes inside double quotes is consistent with the other CLI help messages.
* Contributors: Jacob Perron

0.9.3 (2020-05-13)
------------------

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------

0.9.0 (2020-04-29)
------------------
* Stop using 'node_name' and 'node_namespace' in tests. (`#498 <https://github.com/ros2/ros2cli/issues/498>`_)
  They are both deprecated, and print warnings in CI like:
  Warning: The parameter 'node_name' is deprecated, use 'name' instead
* Replace deprecated launch_ros usage. (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
  The Node parameter 'node_executable' has been deprecated and replaced
  with the parameter 'executable'.
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* Update tests to expect no launch_ros node. (`#474 <https://github.com/ros2/ros2cli/issues/474>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions. (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* Used get_available_rmw_implementations from rclpy. (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon. (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
  This is to give time for discovery to happen between the daemon node and the test fixture nodes.
* Use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * Use f-string
  * Remove unused variable
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Jacob Perron, Peter Baughman, Shane Loretz, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* Contributors: Michael Carroll

0.8.3 (2019-10-23)
------------------
* 0.8.3
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
* Move rosidl implementation details to rosidl_runtime_py. (`#371 <https://github.com/ros2/ros2cli/issues/371>`_)
  * Move rosidl implementation details to rosidl_runtime_py
  This resolves several TODOs.
  Here is the PR moving the related functions to their new home: https://github.com/ros2/rosidl_runtime_py/pull/3
  * Remove dependencies on deprecated packages
  * Remove obsolete test
  * Make linters happy :)
* Contributors: Jacob Perron, Michel Hidalgo, Shane Loretz

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
* Install resource marker file for packages. (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Add service call prototype completer. (`#300 <https://github.com/ros2/ros2cli/issues/300>`_)
* Add 'service find' verb. (`#274 <https://github.com/ros2/ros2cli/issues/274>`_)
  * Add 'service find' verb
  * Add ServiceTypeCompleter
  * Replace ServiceTypeCompleter with service_type_completer
  * Use strings literals
  * Use single quotes
  * Fix import order
* Add 'service type' verb. (`#273 <https://github.com/ros2/ros2cli/issues/273>`_)
  * Add 'service type' verb
  * Print all types
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray

0.7.4 (2019-05-29)
------------------
* Add support for service type name without 'srv' namespace part. (`#247 <https://github.com/ros2/ros2cli/issues/247>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------
* Use new type identification for service calls. (`#242 <https://github.com/ros2/ros2cli/issues/242>`_)
  * Use new type identification for service calls
  * Address middle_module logic
  * Fix typo
  * Use review suggestions
* Contributors: Karsten Knese

0.7.2 (2019-05-08)
------------------
* Add xmllint linter test. (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * Add xmllint test to ament_python packages
  * Cover new packages as well
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* Use migrated message utility functions
  These functions are more generally useful outside of ros2topic and so they have been moved to rosidl_runtime_py.
* Use safe_load instead of deprecated load. (`#212 <https://github.com/ros2/ros2cli/issues/212>`_)
* Contributors: Jacob Perron, Mikael Arguedas

0.6.3 (2019-02-08)
------------------
* Consistent node naming. (`#158 <https://github.com/ros2/ros2cli/issues/158>`_)
  * Support for easy integration with ros2 security features by starting CLI nodes with a consistent prefix.
  * Removing unneeded comment
  * Making DirectNode visible (removing hidden node prefix) to have consistent node naming for ros2cli.
  * Start all CLI nodes as hidden.
  * Shortening the default CLI node name prefix from '_ros2cli_node' to '_ros2cli'
  * Importing HIDDEN_NODE_PREFIX from rclpy, renaming CLI_NODE_NAME_PREFIX -> NODE_NAME_PREFIX.
  * Ros2node - Importing HIDDEN_NODE_PREFIX from rclpy
  * Linter fixes.
* Contributors: AAlon

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Check in action module if action service. (`#163 <https://github.com/ros2/ros2cli/issues/163>`_)
* List services symmetric with topics. (`#162 <https://github.com/ros2/ros2cli/issues/162>`_)
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------

0.5.4 (2018-08-20)
------------------

0.5.3 (2018-07-17)
------------------

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* Add missing arg to ros2 service list. (`#99 <https://github.com/ros2/ros2cli/issues/99>`_)
* Add pytest markers to linter tests
* Ignore F841 from latest Pyflakes release. (`#93 <https://github.com/ros2/ros2cli/issues/93>`_)
* `ros2 service` Use new client api. (`#77 <https://github.com/ros2/ros2cli/issues/77>`_)
  * Use new client api
  * Try_shutdown() -> shutdown()
* Set zip_safe to avoid warning during installation. (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* Print full help when no command is passed. (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Shane Loretz

0.4.0 (2017-12-08)
------------------
* [ros2service] call only once by default. (`#67 <https://github.com/ros2/ros2cli/issues/67>`_)
  * [ros2service] call only once by default
  * Remove once completely
* [ros2topic] pub: add --repeat. (`#66 <https://github.com/ros2/ros2cli/issues/66>`_)
  * First shot at passing -r argument
  * [ros2topic] add once and rate parameters
  * [ros2service] add once and rate parameters
  * Simplify logic, add sleepd for once publisher and remove argparse
  * Fix spelling
  * Format default the same as argparse does
  * Format default the same as argparse does
  * Move logic to the right function
  * Mimic ros2topic and remove extra logic
  * Consistent with services
* Merge pull request `#64 <https://github.com/ros2/ros2cli/issues/64>`_ from ros2/add_type_completer
  add type completer for 'topic pub' and 'service call'
* Merge pull request `#65 <https://github.com/ros2/ros2cli/issues/65>`_ from ros2/wait_for_service_before_calling
  wait for service before calling it
* Wait for service before calling it
* Add type completer for 'topic pub' and 'service call'
* Remove test_suite, add pytest as test_requires
* 0.0.3
* Fix request message population. (`#56 <https://github.com/ros2/ros2cli/issues/56>`_)
  * Use set_msg_fields
  * Remove unused comment
  * Move function and error definition to api module
  * Use message filling method from ros2topic
  * Alphabetical order
* Implicitly inherit from object. (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Merge pull request `#36 <https://github.com/ros2/ros2cli/issues/36>`_ from ros2/improve_error_message
  better error message
* Better error message
* Use yaml for parsing msg and srv values. (`#19 <https://github.com/ros2/ros2cli/issues/19>`_)
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
  various fixes and improvements
* Various fixes and improvements
* Refactor get topic names and types. (`#4 <https://github.com/ros2/ros2cli/issues/4>`_)
  * Ros2topic: use rclpy utility
  * Ros2topic: fixup
  * Ros2topic: support multiple types
  * Ros2service: initial commit
  * Ros2topic: support no_demangle
  * Fix include order
  * Missed a commit
  * Ros2service: add pep257 tests
  * Fix echo to support multiple types
  * Improve shutdown behavior of call, add loop option
  * Address comments
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
