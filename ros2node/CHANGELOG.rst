^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.30.0 (2023-11-06)
-------------------

0.29.1 (2023-10-04)
-------------------

0.29.0 (2023-08-21)
-------------------

0.28.0 (2023-07-11)
-------------------

0.27.0 (2023-06-07)
-------------------

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
* Enable document generation using rosdoc2 (`#811 <https://github.com/ros2/ros2cli/issues/811>`_)
  * Fix warnings for ros2component, ros2doctor, ros2interface, and ros2node
* Contributors: Yadu

0.23.0 (2023-03-02)
-------------------
* Fix linters (`#808 <https://github.com/ros2/ros2cli/issues/808>`_)
* add timeout option for ros2param to find node. (`#802 <https://github.com/ros2/ros2cli/issues/802>`_)
* Contributors: Cristóbal Arroyo, Tomoya Fujita

0.22.0 (2023-02-14)
-------------------
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`_)
* Updated wording in list.py (`#775 <https://github.com/ros2/ros2cli/issues/775>`_)
* Contributors: Audrow Nash, Michael Wrock

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
* Make CLI more robust to discovery latency. (`#494 <https://github.com/ros2/ros2cli/issues/494>`_)
* Remove uses of deprecated ready_fn. (`#506 <https://github.com/ros2/ros2cli/issues/506>`_)
* Contributors: Chris Lalancette, Michel Hidalgo

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
* Just add warning at the top of the node names list. (`#462 <https://github.com/ros2/ros2cli/issues/462>`_)
* Add a warning to ros2 node info when there is more than one node with the same name. (`#463 <https://github.com/ros2/ros2cli/issues/463>`_)
  * Add a warning to ros2 node info when there is more than one node with the given name
  * Add test for nonunique node names
  * Update use of removed API
  * Add newline at top of code
  Co-Authored-By: William Woodall <william@osrfoundation.org>
  Co-authored-by: William Woodall <william@osrfoundation.org>
* Remove ready_fn from test descriptions. (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* Used get_available_rmw_implementations from rclpy. (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon. (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
  This is to give time for discovery to happen between the daemon node and the test fixture nodes.
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
* Merge branch 'master' of github.com:ros2/ros2cli
* Make use of include-hidden flag for ros2node info verb. (`#401 <https://github.com/ros2/ros2cli/issues/401>`_)
* Contributors: Alejandro Hernández Cordero, Brian Marchi, Chris Lalancette, Dirk Thomas, Emerson Knapp, Jacob Perron, Peter Baughman, Shane Loretz, Steven! Ragnarök, claireyywang

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* Add service clients to ros2node info. (`#392 <https://github.com/ros2/ros2cli/issues/392>`_)
* Contributors: Michael Carroll, Mikael Arguedas

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
* [ros2node] Add option to info verb to display hidden names. (`#373 <https://github.com/ros2/ros2cli/issues/373>`_)
  * [ros2node] Add option to info verb to display hidden names
  Changes behavior so that hidden names are not shown by default.
  * Update ros2component to request hidden service names
  Which it uses for identifier component containers.
* [ros2node] Update info headings for actions. (`#357 <https://github.com/ros2/ros2cli/issues/357>`_)
* Contributors: Jacob Perron, Michel Hidalgo, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* Ros2node info: add action category. (`#345 <https://github.com/ros2/ros2cli/issues/345>`_)
  * Add action category
  * Correct node param
  * Split action into server and client
  * Fix indentation
* Contributors: Claire Wang, Michael Carroll

0.8.0 (2019-09-26)
------------------
* Install resource marker file for packages. (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Convert list comprehension to generator. (`#314 <https://github.com/ros2/ros2cli/issues/314>`_)
  Addresses flake8 C412 errors introduced by flake8-comprehension 2.2.0
* Alphasort ros2 node list output. (`#305 <https://github.com/ros2/ros2cli/issues/305>`_)
  For node name order to be deterministic.
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo, Scott K Logan

0.7.4 (2019-05-29)
------------------

0.7.3 (2019-05-20)
------------------

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
* Solved bug when trying to find node info in namespaces. (`#206 <https://github.com/ros2/ros2cli/issues/206>`_)
  * Solved bug when trying to find node info in namespaces
  * Added test
  * Accepted non fully qualified names. Used better terminology for variables.
  * Modified with PR comments
  * Simplify logic
* Contributors: ivanpauno

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
* Add completer for node info <node-name>. (`#189 <https://github.com/ros2/ros2cli/issues/189>`_)
* Fix node info verb description. (`#186 <https://github.com/ros2/ros2cli/issues/186>`_)
* Contributors: AAlon, Dirk Thomas, Jacob Perron

0.6.2 (2018-12-12)
------------------
* Add slash for node name. (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
  * Add slash for node name
  * Check for forward slash in ros2param
  * Use get_absolute_node_name function
* Contributors: Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Add ros2 node info verb. (`#159 <https://github.com/ros2/ros2cli/issues/159>`_)
  * Add ros node info verb
  * Fix copyright headers
  * Fix flake8 issues
  * Addresses comments for exceptional case
  * Simplify error message when we can't find the node.
* Contributors: Ross Desmond, Shane Loretz

0.6.0 (2018-11-19)
------------------
* Node name with namespace. (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
* Contributors: Dirk Thomas

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
* Add pytest markers to linter tests
* Ignore empty or None node names. (`#76 <https://github.com/ros2/ros2cli/issues/76>`_)
* Set zip_safe to avoid warning during installation. (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* Print full help when no command is passed. (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* Remove test_suite, add pytest as test_requires
* 0.0.3
* Implicitly inherit from object. (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
  various fixes and improvements
* Various fixes and improvements
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
  add pep257 tests
* Add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
  Entry point, plugin system, daemon, existing tools
* Add ros2node list
* Contributors: Dirk Thomas, Mikael Arguedas
