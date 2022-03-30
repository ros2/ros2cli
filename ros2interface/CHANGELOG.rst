^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.2 (2022-03-30)
-------------------
* Add timeout to kill hanging tests (`#701 <https://github.com/ros2/ros2cli/issues/701>`_)
* Contributors: Audrow Nash

0.18.1 (2022-03-28)
-------------------

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
* Remove ros2interface test dependencies on builtin interface. (`#579 <https://github.com/ros2/ros2cli/issues/579>`_)
* Contributors: Audrow Nash

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* Update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
* Handle inline comments on constants correctly. (`#548 <https://github.com/ros2/ros2cli/issues/548>`_)
* Update quoted comments in the test (`#540 <https://github.com/ros2/ros2cli/issues/540>`_)
* Add option to include/remove whitespace and comments. (`#527 <https://github.com/ros2/ros2cli/issues/527>`_)
* Show "expanded" message definition. (`#524 <https://github.com/ros2/ros2cli/issues/524>`_)
* Contributors: Audrow, Audrow Nash, Claire Wang, Tully Foote

0.9.5 (2020-06-01)
------------------
* [ros2interface] Allow stdin input for 'ros2 interface show'. (`#387 <https://github.com/ros2/ros2cli/issues/387>`_)
  * Allow stdin input for 'ros2 interface show'
  * Just use help for all the information
  * Fix import order
  * Simplify logic
  * Catch empty values in case stdin doesn't contain output
  * Add test for 'ros2 interface show' with stdin
  * Use test_msgs instead of std_msgs for stdin test
  * Use example_interfaces in help for show
  Co-authored-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Contributors: Nursharmin Ramli

0.9.4 (2020-05-26)
------------------
* [ros2interface] Remove usage of deprecated std_msgs and std_srvs packages. (`#516 <https://github.com/ros2/ros2cli/issues/516>`_)
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
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions. (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
* Add docblock for ros2interface command. (`#434 <https://github.com/ros2/ros2cli/issues/434>`_)
* Contributors: Dirk Thomas, Jacob Perron, Peter Baughman, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* [ros2interface] Fix tests. (`#386 <https://github.com/ros2/ros2cli/issues/386>`_)
  Broken in https://github.com/ros2/rosidl_runtime_py/pull/6
* Contributors: Jacob Perron, Michael Carroll

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
* Make ros2 interface show fail gracefully (no traceback). (`#372 <https://github.com/ros2/ros2cli/issues/372>`_)
* Move rosidl implementation details to rosidl_runtime_py. (`#371 <https://github.com/ros2/ros2cli/issues/371>`_)
  * Move rosidl implementation details to rosidl_runtime_py
  This resolves several TODOs.
  Here is the PR moving the related functions to their new home: https://github.com/ros2/rosidl_runtime_py/pull/3
  * Remove dependencies on deprecated packages
  * Remove obsolete test
  * Make linters happy :)
* Handle bad or missing package on ros2 interface show. (`#366 <https://github.com/ros2/ros2cli/issues/366>`_)
* Ensure ros2 interface show has trailing newline. (`#368 <https://github.com/ros2/ros2cli/issues/368>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo, Shane Loretz

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
* Add interface proto . (`#298 <https://github.com/ros2/ros2cli/issues/298>`_)
  * Add interface proto
  * Use rosidl_runtime_py.utilities
  * No-hyphens -> no-quotes
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Ros2interface fixes/changes. (`#308 <https://github.com/ros2/ros2cli/issues/308>`_)
  * Minor changes to ros2interface
* Add actions to interface type completer. (`#303 <https://github.com/ros2/ros2cli/issues/303>`_)
  Fix `#302 <https://github.com/ros2/ros2cli/issues/302>`_
* [ros2interface] Fix output formatting. (`#289 <https://github.com/ros2/ros2cli/issues/289>`_)
  * Add missing indentation to list verb output
  * Format show verb error message
* Added ros2interface to replace ros2 msg/srv. (`#288 <https://github.com/ros2/ros2cli/issues/288>`_)
  * Adding ros2 interface command line tool
  Signed off by: Siddharth Kucheria
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Siddharth Kucheria

0.7.4 (2019-05-29)
------------------

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------

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

0.4.0 (2017-12-08)
------------------
