^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Contributors: Claire Wang

0.9.5 (2020-06-01)
------------------

0.9.4 (2020-05-26)
------------------

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
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
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
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Jacob Perron, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* [ros2lifecycle] Add test coverage for CLI. (`#391 <https://github.com/ros2/ros2cli/issues/391>`_)
  * Add test coverage for ros2lifecycle CLI.
  * Add ros2lifecycle_test_fixtures package.
  * Use ros2lifecycle_test_fixtures package instead of demo lifecycle package.
  * Add a few more ros2lifecycle test cases.
  * Address last few peer review comments.
  * Rename ros2lifecycle tests fixture node.
* [ros2lifecycle] Misc fixes. (`#395 <https://github.com/ros2/ros2cli/issues/395>`_)
* Contributors: Michael Carroll, Michel Hidalgo

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Contributors: Shane Loretz

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
* Contributors: Dirk Thomas, Jacob Perron

0.7.4 (2019-05-29)
------------------
* [ros2lifecycle] Only return the state for the node requested. (`#266 <https://github.com/ros2/ros2cli/issues/266>`_)
  * [ros2lifecycle] Only return the state for the node requested
  Previously, the 'get' verb was returning the state for all lifecycle nodes.
  * Fix logic to handle optional positional argument
  * Clarfiy 'lifecycle get' behavior in documentation
* Contributors: Jacob Perron

0.7.3 (2019-05-20)
------------------
* Use new type identification for lifecycle nodes. (`#241 <https://github.com/ros2/ros2cli/issues/241>`_)
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
* Ros2lifecycle `nodes` and `get` verbs, now showing hidden nodes when requested. (`#208 <https://github.com/ros2/ros2cli/issues/208>`_)
  * Fixed issue 169
  * Nipick: remove newline
* Contributors: ivanpauno

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------
* Add slash for node name. (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
  * Add slash for node name
  * Check for forward slash in ros2param
  * Use get_absolute_node_name function
* Remove unused cli option. (`#174 <https://github.com/ros2/ros2cli/issues/174>`_)
  * Remove --transition option from lifecycle get
  * Check for leading slash on node name
  * No new line between ifs
  * Review comments
* Fix ros2 lifecycle get. (`#167 <https://github.com/ros2/ros2cli/issues/167>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* Lifecycle refactor. (`#150 <https://github.com/ros2/ros2cli/issues/150>`_)
  * Remove node name from lifecycle messages
  * New verb _nodes\_ for lifecycle
* Comply with new node representation. (`#149 <https://github.com/ros2/ros2cli/issues/149>`_)
  * Comply with new node representation
  * Python oneliner
  * Get_node_names return full struct
* Contributors: Karsten Knese

0.5.4 (2018-08-20)
------------------
* Remove apparently unused yaml dependency. (`#130 <https://github.com/ros2/ros2cli/issues/130>`_)
* Contributors: Mikael Arguedas

0.5.3 (2018-07-17)
------------------

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* Ros2lifecycle: fix dep and import. (`#103 <https://github.com/ros2/ros2cli/issues/103>`_)
* Make flake8 import order happy
* Add ros2 lifecycle. (`#97 <https://github.com/ros2/ros2cli/issues/97>`_)
  * Add ros2 lifecycle
  * Remove debug print
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
