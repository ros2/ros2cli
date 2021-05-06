^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2param
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.14.0 (2021-04-26)
-------------------

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* 0.12.0
* Make the ros2param --filter test more reliable. (`#606 <https://github.com/ros2/ros2cli/issues/606>`_)
* Add wildcard loading to ros2 param load. (`#602 <https://github.com/ros2/ros2cli/issues/602>`_)
* ros2 param dump/load should use fully qualified node names. (`#600 <https://github.com/ros2/ros2cli/issues/600>`_)
* Add --filter options for 'ros2 param list'. (`#592 <https://github.com/ros2/ros2cli/issues/592>`_)
* remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add rosparam verb load. (`#590 <https://github.com/ros2/ros2cli/issues/590>`_)
  * Add rosparam verb load
  * Move load_parameter_file implementation to api, add test_verb_load
  * Apply fixes for linter
  * Remove TODO comment
  * Fix linter errors
  Co-authored-by: Audrow Nash <audrow.nash@gmail.com>
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang, Ivan Santiago Paunovic, Victor Lopez

0.11.0 (2021-01-25)
-------------------

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* add "--param-type" option to ros2param list. (`#572 <https://github.com/ros2/ros2cli/issues/572>`_)
  * add "--param-type" option to ros2param list.
  Co-authored-by: Chris Lalancette <clalancette@openrobotics.org>
* update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
  * update maintainers
  * add authors, update setup.py
  * remove trailing whitespace
* Contributors: Claire Wang, tomoya

0.9.5 (2020-06-01)
------------------

0.9.4 (2020-05-26)
------------------
* Avoid array.array in ros2 param output. (`#508 <https://github.com/ros2/ros2cli/issues/508>`_)
* Contributors: Michel Hidalgo

0.9.3 (2020-05-13)
------------------

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------

0.9.0 (2020-04-29)
------------------
* [ros2param] Convert test_verb_dump into launch test. (`#485 <https://github.com/ros2/ros2cli/issues/485>`_)
  Fixes https://github.com/ros2/ros2cli/issues/480
  The actual tests are the same, except with the use of launch_testing we ensure the CLI daemon
  is restarted between tests. This follows a similar pattern as the other ros2cli tests.
  In addition to converting to launch tests, this change also runs the tests for all RMW implementations.
  For now, we are skipping tests on Windows. Other CLI tests are skipped on Windows since https://github.com/ros2/ros2cli/pull/489. To be reverted when https://github.com/ros2/build_farmer/issues/248 is resolved.
* Do not wait for entire timeout. (`#486 <https://github.com/ros2/ros2cli/issues/486>`_)
  Follow-up to `#481 <https://github.com/ros2/ros2cli/issues/481>`_
  This makes the tests faster.
* [ros2param] Add timeout to ros2param list. (`#469 <https://github.com/ros2/ros2cli/issues/469>`_)
* [ros2param] Wait for discovery before running tests. (`#481 <https://github.com/ros2/ros2cli/issues/481>`_)
  Fixes `#480 <https://github.com/ros2/ros2cli/issues/480>`_.
  Catch expected exceptions from rclpy (or transitively as xmlrpc.client.Fault) while we wait for discovery in the test setup.
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * use f-string
  * remove unused variable
* fix ros2param tests. (`#441 <https://github.com/ros2/ros2cli/issues/441>`_)
* only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * extend API to exclude extensions from loading
  * add add_subparsers_on_demand() function
  * update all extensions to use the new API
  * deprecate old API, add deprecation warnings
* Contributors: Dirk Thomas, DongheeYe, Jacob Perron

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
* add ros2 param describe. (`#367 <https://github.com/ros2/ros2cli/issues/367>`_)
  * add ros2 param describe
  * fix linter
* add completion for parameter name arguments. (`#364 <https://github.com/ros2/ros2cli/issues/364>`_)
  * add completion for parameter name arguments
  * style
  * add parameter name completion for set
* Contributors: Dirk Thomas, Shane Loretz

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
* Adjusting ros2param tests to take into account automatic declaration of 'use_sim_time' parameter. (`#307 <https://github.com/ros2/ros2cli/issues/307>`_)
* Add param dump <node-name>. (`#285 <https://github.com/ros2/ros2cli/issues/285>`_)
  * wip param dump
  * default path & cleanup
  * wip test verb dump
  * rm spin_once
  * nested namespaces
  * cleaning up
  * multithread the test
  * todo use PARAMETER_SEPARATOR_STRING
  * test comp generate<->expected param file
  * lipstick
  * use proper PARAMETER_SEPARATOR_STRING
  * mv common code to api
  * rename param output-dir
  * rm line breaks
  * raise rather than print
  * rm useless import
  * raise rather than print
  * add --print option
  * prepend node namespace to output filename
  * preempted -> preempt
  * "w" -> 'w'
  * Output file using fully qualified node name
  * fix linter tests
  * relaxe --print preempt test
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Juan Ignacio Ubeira

0.7.4 (2019-05-29)
------------------
* fix param list for hidden nodes. (`#268 <https://github.com/ros2/ros2cli/issues/268>`_)
* fix param list for nodes which don't have the service. (`#265 <https://github.com/ros2/ros2cli/issues/265>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* add xmllint linter test. (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * add xmllint test to ament_python packages
  * cover new packages as well
* use yaml.safe_load (round2). (`#229 <https://github.com/ros2/ros2cli/issues/229>`_)
  * use yaml.safe_load (round2)
  * without the typo
* Add capability to use ros2 param set for array types. (`#199 <https://github.com/ros2/ros2cli/issues/199>`_)
  * Add tests for converting string values to parameter types
  * Use YAML parsing to convert parameters to correct type
  * Do not perform redundant type conversions
  * Fix import ordering
  * Use single quotes instead of double
  * Prevent unnecessary list comprehensions
  * Consolidate similar tests into single paramatrized function
  * remove obsolete functions
  * expect array.array for numerics
* Contributors: Mikael Arguedas, sgvandijk

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------
* add slash for node name. (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
  * add slash for node name
  * check for forward slash in ros2param
  * use get_absolute_node_name function
* Contributors: Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* node name with namespace. (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
* Contributors: Dirk Thomas

0.5.4 (2018-08-20)
------------------
* add support for parameter prefixes in ros2 param list. (`#131 <https://github.com/ros2/ros2cli/issues/131>`_)
  * add support for parameter prefix in ros2 param list
  * require at least 1 prefix and simplify logic
* remove apparently unused yaml dependency. (`#130 <https://github.com/ros2/ros2cli/issues/130>`_)
* Contributors: Mikael Arguedas

0.5.3 (2018-07-17)
------------------

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* Specific message for unset parameters. (`#104 <https://github.com/ros2/ros2cli/issues/104>`_)
* update ros2 param list output for a specific node. (`#98 <https://github.com/ros2/ros2cli/issues/98>`_)
* add ros2 param. (`#95 <https://github.com/ros2/ros2cli/issues/95>`_)
  * add ros2 param
  * remove debug output
  * add rcl_interfaces dependency instead of relying on it transitively
  * typo
  * check if value is None regardless of the hide type value
  * return error when requested paraemeter is not set
  * remove condition
* Contributors: Dirk Thomas, dhood

0.4.0 (2017-12-08)
------------------
