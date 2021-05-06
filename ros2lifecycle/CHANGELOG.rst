^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.14.0 (2021-04-26)
-------------------

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* 0.12.0
* remove maintainer (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add Audrow as a maintainer (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang

0.11.0 (2021-01-25)
-------------------

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* update maintainers (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
  * update maintainers
  * add authors, update setup.py
  * remove trailing whitespace
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
* Replace deprecated launch_ros usage (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
  The Node parameter 'node_executable' has been deprecated and replaced
  with the parameter 'executable'.
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* used get_available_rmw_implementations from rclpy (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
  This is to give time for discovery to happen between the daemon node and the test fixture nodes.
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * use f-string
  * remove unused variable
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * extend API to exclude extensions from loading
  * add add_subparsers_on_demand() function
  * update all extensions to use the new API
  * deprecate old API, add deprecation warnings
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
* [ros2lifecycle] Add test coverage for CLI (`#391 <https://github.com/ros2/ros2cli/issues/391>`_)
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
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.7.4 (2019-05-29)
------------------
* [ros2lifecycle] Only return the state for the node requested (`#266 <https://github.com/ros2/ros2cli/issues/266>`_)
  * [ros2lifecycle] Only return the state for the node requested
  Previously, the 'get' verb was returning the state for all lifecycle nodes.
  * Fix logic to handle optional positional argument
  * Clarfiy 'lifecycle get' behavior in documentation
* Contributors: Jacob Perron

0.7.3 (2019-05-20)
------------------
* use new type identification for lifecycle nodes (`#241 <https://github.com/ros2/ros2cli/issues/241>`_)
* Contributors: Karsten Knese

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * add xmllint test to ament_python packages
  * cover new packages as well
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* ros2lifecycle `nodes` and `get` verbs, now showing hidden nodes when requested (`#208 <https://github.com/ros2/ros2cli/issues/208>`_)
  * Fixed issue 169
  * nipick: remove newline
* Contributors: ivanpauno

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------
* add slash for node name (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
  * add slash for node name
  * check for forward slash in ros2param
  * use get_absolute_node_name function
* Remove unused cli option (`#174 <https://github.com/ros2/ros2cli/issues/174>`_)
  * remove --transition option from lifecycle get
  * check for leading slash on node name
  * no new line between ifs
  * review comments
* fix ros2 lifecycle get (`#167 <https://github.com/ros2/ros2cli/issues/167>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* Lifecycle refactor (`#150 <https://github.com/ros2/ros2cli/issues/150>`_)
  * remove node name from lifecycle messages
  * new verb _nodes\_ for lifecycle
* comply with new node representation (`#149 <https://github.com/ros2/ros2cli/issues/149>`_)
  * comply with new node representation
  * python oneliner
  * get_node_names return full struct
* Contributors: Karsten Knese

0.5.4 (2018-08-20)
------------------
* remove apparently unused yaml dependency (`#130 <https://github.com/ros2/ros2cli/issues/130>`_)
* Contributors: Mikael Arguedas

0.5.3 (2018-07-17)
------------------

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* ros2lifecycle: fix dep and import (`#103 <https://github.com/ros2/ros2cli/issues/103>`_)
* make flake8 import order happy
* add ros2 lifecycle (`#97 <https://github.com/ros2/ros2cli/issues/97>`_)
  * add ros2 lifecycle
  * remove debug print
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
