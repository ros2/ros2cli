^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2doctor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Improve ros2 doctor on Windows (`#631 <https://github.com/ros2/ros2cli/issues/631>`_)
  Improve ros2 doctor on Windows:
  * Print a more useful error message when failing to load plugin
  * Print a clearer error message when failing to find either a loopback or no loopback interface on Windows.
  * Print "latest version" instead of "required version".
  * Print the corrent name of the python executable according to the platform.
  Co-authored-by: Chris Lalancette <clalancette@openrobotics.org>
  Co-authored-by: Christophe Bedard <bedard.christophe@gmail.com>
* Contributors: Ivan Santiago Paunovic

0.14.0 (2021-04-26)
-------------------
* [ros2doctor] Add QoS compatibility check and report (`#621 <https://github.com/ros2/ros2cli/issues/621>`_)
  * Add QoS compatibility check and report for ROS2 Doctor
  * Make linter happy
  * Handle case when report has nothing to show
  * Remove unused function
  * Make the option to skip topics when getting topics
  * Output reason with QoS warning or error
  * Update report category
  * Add tests and test fixtures
  * Update copyright year in test fixtures
  * Fix quotes
  * Retry tests on failure
  * Make tests weaker
  * Remove re import
* Continue to next iteration after exceptions in generate_reports (`#623 <https://github.com/ros2/ros2cli/issues/623>`_)
  run_checks loops
* Contributors: Alberto Soragna, Audrow Nash

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
* Support Python 3.8-provided importlib.metadata (`#585 <https://github.com/ros2/ros2cli/issues/585>`_)
  The importlib_metadata package is a backport of the importlib.metadata
  module from Python 3.8. Fedora (and possibly others) no longer package
  importlib_metadata because they ship Python versions which have the
  functionality built-in.
* Contributors: Scott K Logan

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* update maintainers (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
  * update maintainers
  * add authors, update setup.py
  * remove trailing whitespace
* Remove pkg_resources from ros2doctor.
  In combination with some other work, this will make the
  command-line tools faster.
* Make ros2doctor depend on ros_environment and fix platform.py bug on error. (`#538 <https://github.com/ros2/ros2cli/issues/538>`_)
  * Make ros2doctor depend on ros_environment.
  Since it technically needs ROS_DISTRO defined for it to work,
  that means it also has an exec_depend on ros2doctor.  Put that
  in here.  Also make sure RosDistroReport returns a valid report on error.
* Refactor ros2doctor hello verb. (`#521 <https://github.com/ros2/ros2cli/issues/521>`_)
* Contributors: Chris Lalancette, Claire Wang, Michel Hidalgo

0.9.5 (2020-06-01)
------------------

0.9.4 (2020-05-26)
------------------
* Ensure ros2doctor ROS nodes have valid names. (`#513 <https://github.com/ros2/ros2cli/issues/513>`_)
* Contributors: Michel Hidalgo

0.9.3 (2020-05-13)
------------------

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------

0.9.0 (2020-04-29)
------------------
* Make sure to add ros2doctor verbs to the extension points. (`#495 <https://github.com/ros2/ros2cli/issues/495>`_)
* [ros2doctor] Only report topic warnings if publisher or subscription count is zero (`#472 <https://github.com/ros2/ros2cli/issues/472>`_)
  Before the tool was generating many false positives as it is a valid configuration
  to have an unequal number of publishers and subscriptions on a given topic.
  This change makes it so we only provide a warning if one of the counts is zero and the
  other is not. Although this is still a legitimate configuration, it seems more likely to be
  a problem and worth reporting.
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Prefix ros2doctor node names with 'ros2doctor\_' (`#451 <https://github.com/ros2/ros2cli/issues/451>`_)
  This fixes an error when the hostname starts
  with an illegal character for a node name
  (e.g. a number).
* [ros2doctor] Handle non-metapackages in rosdistro check (`#452 <https://github.com/ros2/ros2cli/issues/452>`_)
  Otherwise, we get a KeyError and get warnings about not being able to find versions for certain packages.
* [ros2doctor] Improve doctor_warn()  (`#445 <https://github.com/ros2/ros2cli/issues/445>`_)
  * decouple warning/error msg from result to show traceback
  * Update print statement to use f-string
  * remove None return and add error catching
  * specify stacklevel; add doctor_error
* Multimachine communication (`#432 <https://github.com/ros2/ros2cli/issues/432>`_)
  * add verb call
  * add verb entry point
  * put executor.spin in its own thread
  * update args
  * add missing whitespace
  * add summary table doc string
  * fix exception ignore traceback
  * fix mixed up sub/receive dicts
  * reapply changes
  * add option and verbose name, enforce node name uniqueness, add context lock. prefix variables, add infinite loop
  * change verb naem
  * add single host test case
  * add rclpy dependency
  * correct typo, remove stderr from test
  * restart daemon to resolve CI runtime error
  * replace ready_fn with ReadyToTest()
* [ros2doctor] Check for deprecated packages (`#388 <https://github.com/ros2/ros2cli/issues/388>`_)
  * check local package versions against rosdistro
* Revert "simplify helper function"
  This reverts commit 2c1768d1f464aa6e8d7f4cb162a5e36647036a43.
* simplify helper function
* Update network check to fix missing flags on Windows  (`#404 <https://github.com/ros2/ros2cli/issues/404>`_)
  * add no flags scenario
* Contributors: Chris Lalancette, Claire Wang, Dirk Thomas, Jacob Perron, claireyywang

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* update headline to capital letters (`#383 <https://github.com/ros2/ros2cli/issues/383>`_)
* Contributors: Claire Wang, Michael Carroll

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Update failed modules message (`#380 <https://github.com/ros2/ros2cli/issues/380>`_)
  * Update failed modules message
  The message previously said, for example
  ```
  Failed modules are  network
  ```
  Which is confusing when there's only one module
  Changing to:
  ```
  Failed modules: network
  ```
  Which works if there's one or more failed modules
  * remove double space
* fix AttributeError (`#370 <https://github.com/ros2/ros2cli/issues/370>`_)
* add new args (`#354 <https://github.com/ros2/ros2cli/issues/354>`_)
* Contributors: Claire Wang, Marya Belanger, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* ros2doctor: add topic check (`#341 <https://github.com/ros2/ros2cli/issues/341>`_)
  * add topic check&report
  * add topic check&report
  * add topic test, tbc
  * add topic report unit test
  * correct docstring
* Contributors: Claire Wang, Michael Carroll

0.8.0 (2019-09-26)
------------------
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* ros2doctor: add `--include-warning` arg (`#338 <https://github.com/ros2/ros2cli/issues/338>`_)
  * add include-warning arg
  * rm whitespace
  * update arg usage
  * simplify error/warning mechanism
  * simplify run_checks param
* Add warning and error handling for `ifcfg` import on Windows and OSX (`#332 <https://github.com/ros2/ros2cli/issues/332>`_)
  * add error handling for ifcfg and check/report type
  * modify check/report warning msgs
  * fix code format
  * fix grammar
  * fix var refed before declared exception
  * remove type check
  * update network check/report rtypes
  * move report if/else block
  * remove report inits
* Add RMW name to report  (`#335 <https://github.com/ros2/ros2cli/issues/335>`_)
  * add rmw library info
  * add middleware name
  * add entry point
* Make network check case-insensitive (`#334 <https://github.com/ros2/ros2cli/issues/334>`_)
  * make network check case-insensitive
  * update case insensitive function call
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* update README entry point examples (`#329 <https://github.com/ros2/ros2cli/issues/329>`_)
* Update report feature with new argument, add temp fix for ifcfg module  (`#324 <https://github.com/ros2/ros2cli/issues/324>`_)
  * add network checks and report
  * network shenanigens
  * network shenanigens
  * network shenanigens
  * add network check and report
  * update code format
  * revised code format
  * added rosdep key ifcfg-pip
  * revise code
  * working on report format
  * improving report
  * update platform report format
  * update network report format
  * add format print
  * add --report_failed feature
  * improving report format
  * temp fix ifcfg import module
  * update build dep
  * fix flake8
  * fix flake8
  * add abc and Report class
  * Implement ABC for each check and report and udpate format print
  * update ifcfg import error, fix code format
  * add newlines
  * update warning msgs
  * fix code format
  * update report of failed checks
  * update run_check
  * udpate generate_report
  * add sphinx style docstring and type annotations
  * add context manager for custom warning msg
  * fixed flakey issues
  * update Check and Report class error handling
  * fix report refed before assigned mistake
  * add failed entry point name
  * remove pass from try/except
  * add error handling for check/report
  * change ValueError to Exception
* removing ifcfg_vendor (`#323 <https://github.com/ros2/ros2cli/issues/323>`_)
* Add network configuration check and report to ros2doctor (`#319 <https://github.com/ros2/ros2cli/issues/319>`_)
  * add network checks and report
  * network shenanigens
  * network shenanigens
  * network shenanigens
  * add network check and report
  * update code format
  * revised code format
  * add ifcfg-pip rosdep key
  waiting for rosdistro PR approval https://github.com/ros/rosdistro/pull/22071
  * added rosdep key ifcfg-pip
  * revise code
* add ros2doctor README (`#318 <https://github.com/ros2/ros2cli/issues/318>`_)
  * add README
  * update sentence to new line
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammar
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
* Add distribution check and report feature to `ros2 doctor` command   (`#311 <https://github.com/ros2/ros2cli/issues/311>`_)
  * created ros2debug package
  * created setup verb, need revision
  * added simple setup check
  * added simple setup check, need testing
  * added four standard tests
  * add new line to end of file
  * corrected code format
  * update debug api Signed-off by: Claire Wang clairewang@openrobotics.org
  * update code format
  * added rosdistro
  * fixed style and added rosdistro
  * fixed code style
  * corrected code style
  * added network interface print command
  * leave out network verb, change cmd name to doctor, add alias wtf
  * remove network.py
  * add version, rosdistro, platformdist, fallback checks, fallback checks
  * add wtf alias, separate checks and report
  * remove duplicates, correct grammer
  * add entrypoints for checks and report, output failed checks
  * corrected code format
  * reformat report, correct typo
* Contributors: Claire Wang, Dirk Thomas

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
