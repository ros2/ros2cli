^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2param
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.32.0 (2024-04-16)
-------------------

0.31.2 (2024-03-27)
-------------------
* ros2 param dump should handle empty list as exception. (`#881 <https://github.com/ros2/ros2cli/issues/881>`_)
* Contributors: Tomoya Fujita

0.31.1 (2024-02-07)
-------------------

0.31.0 (2024-01-24)
-------------------

0.30.1 (2023-12-26)
-------------------

0.30.0 (2023-11-06)
-------------------
* Warning: get_parameter_value() is deprecated. (`#866 <https://github.com/ros2/ros2cli/issues/866>`_)
* Contributors: Tomoya Fujita

0.29.1 (2023-10-04)
-------------------

0.29.0 (2023-08-21)
-------------------

0.28.0 (2023-07-11)
-------------------
* Fix tests with get_type_description service and param present (`#838 <https://github.com/ros2/ros2cli/issues/838>`_)
* Update ros2 param dump dosctring. (`#837 <https://github.com/ros2/ros2cli/issues/837>`_)
* Contributors: Emerson Knapp, Murilo M Marinho

0.27.0 (2023-06-07)
-------------------

0.26.1 (2023-05-11)
-------------------

0.26.0 (2023-04-28)
-------------------

0.25.0 (2023-04-18)
-------------------
* remove deprecated options (`#824 <https://github.com/ros2/ros2cli/issues/824>`_)
* Make all of the dependencies in pure Python packages exec_depend. (`#823 <https://github.com/ros2/ros2cli/issues/823>`_)
* Contributors: Chris Lalancette, Tomoya Fujita

0.24.1 (2023-04-12)
-------------------

0.24.0 (2023-04-11)
-------------------

0.23.0 (2023-03-02)
-------------------
* add timeout option for ros2param to find node. (`#802 <https://github.com/ros2/ros2cli/issues/802>`_)
* Contributors: Tomoya Fujita

0.22.0 (2023-02-14)
-------------------
* Fix printing of integer and double arrays. (`#804 <https://github.com/ros2/ros2cli/issues/804>`_)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.21.0 (2022-11-02)
-------------------

0.20.0 (2022-09-13)
-------------------
* refactor: make ros2param use rclpy.parameter_client (`#716 <https://github.com/ros2/ros2cli/issues/716>`_)
* Contributors: Brian

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
* Fix how ros2 param interprets command-line arguments. (`#684 <https://github.com/ros2/ros2cli/issues/684>`_)
* Contributors: Chris Lalancette

0.15.0 (2021-11-18)
-------------------
* Update maintainers to Aditya Pande, Audrow Nash, and Michael Jeronimo (`#673 <https://github.com/ros2/ros2cli/issues/673>`_)
* Updated maintainers (`#670 <https://github.com/ros2/ros2cli/issues/670>`_)
* Removed redundant code (`#666 <https://github.com/ros2/ros2cli/issues/666>`_)
* Reapply `#659 <https://github.com/ros2/ros2cli/issues/659>`_ (`#661 <https://github.com/ros2/ros2cli/issues/661>`_)
* Fix flaky ros2 param list (`#656 <https://github.com/ros2/ros2cli/issues/656>`_)
* Skip None Result (`#646 <https://github.com/ros2/ros2cli/issues/646>`_)
* add '--write' option to avoid an unintentional data loss. (`#638 <https://github.com/ros2/ros2cli/issues/638>`_)
* Add changelogs (`#635 <https://github.com/ros2/ros2cli/issues/635>`_)
* Contributors: Aditya Pande, Audrow Nash, Ivan Santiago Paunovic, Jacob Perron, Jay Wang, Tomoya Fujita

0.14.0 (2021-04-26)
-------------------

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* Make the ros2param --filter test more reliable. (`#606 <https://github.com/ros2/ros2cli/issues/606>`_)
* Add wildcard loading to ros2 param load. (`#602 <https://github.com/ros2/ros2cli/issues/602>`_)
* Ros2 param dump/load should use fully qualified node names. (`#600 <https://github.com/ros2/ros2cli/issues/600>`_)
* Add --filter options for 'ros2 param list'. (`#592 <https://github.com/ros2/ros2cli/issues/592>`_)
* Remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add rosparam verb load. (`#590 <https://github.com/ros2/ros2cli/issues/590>`_)
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang, Ivan Santiago Paunovic, Victor Lopez

0.11.0 (2021-01-25)
-------------------

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* Add "--param-type" option to ros2param list. (`#572 <https://github.com/ros2/ros2cli/issues/572>`_)
* Update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
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
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * Use f-string
  * Remove unused variable
* Fix ros2param tests. (`#441 <https://github.com/ros2/ros2cli/issues/441>`_)
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
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
* Add ros2 param describe. (`#367 <https://github.com/ros2/ros2cli/issues/367>`_)
  * Add ros2 param describe
  * Fix linter
* Add completion for parameter name arguments. (`#364 <https://github.com/ros2/ros2cli/issues/364>`_)
  * Add completion for parameter name arguments
  * Style
  * Add parameter name completion for set
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
* Install resource marker file for packages. (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Adjusting ros2param tests to take into account automatic declaration of 'use_sim_time' parameter. (`#307 <https://github.com/ros2/ros2cli/issues/307>`_)
* Add param dump <node-name>. (`#285 <https://github.com/ros2/ros2cli/issues/285>`_)
  * Wip param dump
  * Default path & cleanup
  * Wip test verb dump
  * Rm spin_once
  * Nested namespaces
  * Cleaning up
  * Multithread the test
  * Todo use PARAMETER_SEPARATOR_STRING
  * Test comp generate<->expected param file
  * Lipstick
  * Use proper PARAMETER_SEPARATOR_STRING
  * Mv common code to api
  * Rename param output-dir
  * Rm line breaks
  * Raise rather than print
  * Rm useless import
  * Raise rather than print
  * Add --print option
  * Prepend node namespace to output filename
  * Preempted -> preempt
  * "w" -> 'w'
  * Output file using fully qualified node name
  * Fix linter tests
  * Relaxe --print preempt test
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Juan Ignacio Ubeira

0.7.4 (2019-05-29)
------------------
* Fix param list for hidden nodes. (`#268 <https://github.com/ros2/ros2cli/issues/268>`_)
* Fix param list for nodes which don't have the service. (`#265 <https://github.com/ros2/ros2cli/issues/265>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* Add xmllint linter test. (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * Add xmllint test to ament_python packages
  * Cover new packages as well
* Use yaml.safe_load (round2). (`#229 <https://github.com/ros2/ros2cli/issues/229>`_)
  * Use yaml.safe_load (round2)
  * Without the typo
* Add capability to use ros2 param set for array types. (`#199 <https://github.com/ros2/ros2cli/issues/199>`_)
  * Add tests for converting string values to parameter types
  * Use YAML parsing to convert parameters to correct type
  * Do not perform redundant type conversions
  * Fix import ordering
  * Use single quotes instead of double
  * Prevent unnecessary list comprehensions
  * Consolidate similar tests into single paramatrized function
  * Remove obsolete functions
  * Expect array.array for numerics
* Contributors: Mikael Arguedas, sgvandijk

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------

0.6.3 (2019-02-08)
------------------

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
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* Node name with namespace. (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
* Contributors: Dirk Thomas

0.5.4 (2018-08-20)
------------------
* Add support for parameter prefixes in ros2 param list. (`#131 <https://github.com/ros2/ros2cli/issues/131>`_)
  * Add support for parameter prefix in ros2 param list
  * Require at least 1 prefix and simplify logic
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
* Specific message for unset parameters. (`#104 <https://github.com/ros2/ros2cli/issues/104>`_)
* Update ros2 param list output for a specific node. (`#98 <https://github.com/ros2/ros2cli/issues/98>`_)
* Add ros2 param. (`#95 <https://github.com/ros2/ros2cli/issues/95>`_)
  * Add ros2 param
  * Remove debug output
  * Add rcl_interfaces dependency instead of relying on it transitively
  * Typo
  * Check if value is None regardless of the hide type value
  * Return error when requested paraemeter is not set
  * Remove condition
* Contributors: Dirk Thomas, dhood

0.4.0 (2017-12-08)
------------------
