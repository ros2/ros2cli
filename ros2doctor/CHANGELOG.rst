^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2doctor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.5 (2022-12-02)
-------------------

0.13.4 (2022-06-30)
-------------------

0.13.3 (2022-04-28)
-------------------

0.13.2 (2021-05-07)
-------------------
* Improve ros2 doctor on Windows. (`#631 <https://github.com/ros2/ros2cli/issues/631>`_) (`#634 <https://github.com/ros2/ros2cli/issues/634>`_)
* Add changelog. (`#636 <https://github.com/ros2/ros2cli/issues/636>`_)
* Contributors: Ivan Santiago Paunovic, mergify[bot]

0.13.1 (2021-04-26)
-------------------
* Continue to next iteration after exceptions in generate_reports. (`#623 <https://github.com/ros2/ros2cli/issues/623>`_)
* Contributors: Alberto Soragna

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* Remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang

0.11.0 (2021-01-25)
-------------------
* Support Python 3.8-provided importlib.metadata. (`#585 <https://github.com/ros2/ros2cli/issues/585>`_)
* Contributors: Scott K Logan

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* Update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
* Remove pkg_resources from ros2doctor. (`#537 <https://github.com/ros2/ros2cli/pull/537>`_)
* Make ros2doctor depend on ros_environment and fix platform.py bug on error. (`#538 <https://github.com/ros2/ros2cli/issues/538>`_)
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
* [ros2doctor] Only report topic warnings if publisher or subscription count is zero. (`#472 <https://github.com/ros2/ros2cli/issues/472>`_)
  Before the tool was generating many false positives as it is a valid configuration
  to have an unequal number of publishers and subscriptions on a given topic.
  This change makes it so we only provide a warning if one of the counts is zero and the
  other is not. Although this is still a legitimate configuration, it seems more likely to be
  a problem and worth reporting.
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Prefix ros2doctor node names with 'ros2doctor\_'. (`#451 <https://github.com/ros2/ros2cli/issues/451>`_)
  This fixes an error when the hostname starts
  with an illegal character for a node name
  (e.g. a number).
* [ros2doctor] Handle non-metapackages in rosdistro check. (`#452 <https://github.com/ros2/ros2cli/issues/452>`_)
  Otherwise, we get a KeyError and get warnings about not being able to find versions for certain packages.
* [ros2doctor] Improve doctor_warn() . (`#445 <https://github.com/ros2/ros2cli/issues/445>`_)
  * Decouple warning/error msg from result to show traceback
  * Update print statement to use f-string
  * Remove None return and add error catching
  * Specify stacklevel; add doctor_error
* Multimachine communication. (`#432 <https://github.com/ros2/ros2cli/issues/432>`_)
  * Add verb call
  * Add verb entry point
  * Put executor.spin in its own thread
  * Update args
  * Add missing whitespace
  * Add summary table doc string
  * Fix exception ignore traceback
  * Fix mixed up sub/receive dicts
  * Reapply changes
  * Add option and verbose name, enforce node name uniqueness, add context lock. prefix variables, add infinite loop
  * Change verb naem
  * Add single host test case
  * Add rclpy dependency
  * Correct typo, remove stderr from test
  * Restart daemon to resolve CI runtime error
  * Replace ready_fn with ReadyToTest()
* [ros2doctor] Check for deprecated packages. (`#388 <https://github.com/ros2/ros2cli/issues/388>`_)
  * Check local package versions against rosdistro
* Revert "simplify helper function"
  This reverts commit 2c1768d1f464aa6e8d7f4cb162a5e36647036a43.
* Simplify helper function
* Update network check to fix missing flags on Windows . (`#404 <https://github.com/ros2/ros2cli/issues/404>`_)
  * Add no flags scenario
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
* Update headline to capital letters. (`#383 <https://github.com/ros2/ros2cli/issues/383>`_)
* Contributors: Claire Wang, Michael Carroll

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Update failed modules message. (`#380 <https://github.com/ros2/ros2cli/issues/380>`_)
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
  * Remove double space
* Fix AttributeError. (`#370 <https://github.com/ros2/ros2cli/issues/370>`_)
* Add new args. (`#354 <https://github.com/ros2/ros2cli/issues/354>`_)
* Contributors: Claire Wang, Marya Belanger, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* Ros2doctor: add topic check. (`#341 <https://github.com/ros2/ros2cli/issues/341>`_)
  * Add topic check&report
  * Add topic check&report
  * Add topic test, tbc
  * Add topic report unit test
  * Correct docstring
* Contributors: Claire Wang, Michael Carroll

0.8.0 (2019-09-26)
------------------
* Install resource marker file for packages. (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Ros2doctor: add `--include-warning` arg. (`#338 <https://github.com/ros2/ros2cli/issues/338>`_)
  * Add include-warning arg
  * Rm whitespace
  * Update arg usage
  * Simplify error/warning mechanism
  * Simplify run_checks param
* Add warning and error handling for `ifcfg` import on Windows and OSX. (`#332 <https://github.com/ros2/ros2cli/issues/332>`_)
  * Add error handling for ifcfg and check/report type
  * Modify check/report warning msgs
  * Fix code format
  * Fix grammar
  * Fix var refed before declared exception
  * Remove type check
  * Update network check/report rtypes
  * Move report if/else block
  * Remove report inits
* Add RMW name to report . (`#335 <https://github.com/ros2/ros2cli/issues/335>`_)
  * Add rmw library info
  * Add middleware name
  * Add entry point
* Make network check case-insensitive. (`#334 <https://github.com/ros2/ros2cli/issues/334>`_)
  * Make network check case-insensitive
  * Update case insensitive function call
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Update README entry point examples. (`#329 <https://github.com/ros2/ros2cli/issues/329>`_)
* Update report feature with new argument, add temp fix for ifcfg module . (`#324 <https://github.com/ros2/ros2cli/issues/324>`_)
  * Add network checks and report
  * Network shenanigens
  * Network shenanigens
  * Network shenanigens
  * Add network check and report
  * Update code format
  * Revised code format
  * Added rosdep key ifcfg-pip
  * Revise code
  * Working on report format
  * Improving report
  * Update platform report format
  * Update network report format
  * Add format print
  * Add --report_failed feature
  * Improving report format
  * Temp fix ifcfg import module
  * Update build dep
  * Fix flake8
  * Fix flake8
  * Add abc and Report class
  * Implement ABC for each check and report and udpate format print
  * Update ifcfg import error, fix code format
  * Add newlines
  * Update warning msgs
  * Fix code format
  * Update report of failed checks
  * Update run_check
  * Udpate generate_report
  * Add sphinx style docstring and type annotations
  * Add context manager for custom warning msg
  * Fixed flakey issues
  * Update Check and Report class error handling
  * Fix report refed before assigned mistake
  * Add failed entry point name
  * Remove pass from try/except
  * Add error handling for check/report
  * Change ValueError to Exception
* Removing ifcfg_vendor. (`#323 <https://github.com/ros2/ros2cli/issues/323>`_)
* Add network configuration check and report to ros2doctor. (`#319 <https://github.com/ros2/ros2cli/issues/319>`_)
  * Add network checks and report
  * Network shenanigens
  * Network shenanigens
  * Network shenanigens
  * Add network check and report
  * Update code format
  * Revised code format
  * Add ifcfg-pip rosdep key
  waiting for rosdistro PR approval https://github.com/ros/rosdistro/pull/22071
  * Added rosdep key ifcfg-pip
  * Revise code
* Add ros2doctor README. (`#318 <https://github.com/ros2/ros2cli/issues/318>`_)
  * Add README
  * Update sentence to new line
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammer
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
  * Update grammar
  Co-Authored-By: Marya Belanger <marya@openrobotics.org>
* Add distribution check and report feature to `ros2 doctor` command  . (`#311 <https://github.com/ros2/ros2cli/issues/311>`_)
  * Created ros2debug package
  * Created setup verb, need revision
  * Added simple setup check
  * Added simple setup check, need testing
  * Added four standard tests
  * Add new line to end of file
  * Corrected code format
  * Update debug api Signed-off by: Claire Wang clairewang@openrobotics.org
  * Update code format
  * Added rosdistro
  * Fixed style and added rosdistro
  * Fixed code style
  * Corrected code style
  * Added network interface print command
  * Leave out network verb, change cmd name to doctor, add alias wtf
  * Remove network.py
  * Add version, rosdistro, platformdist, fallback checks, fallback checks
  * Add wtf alias, separate checks and report
  * Remove duplicates, correct grammer
  * Add entrypoints for checks and report, output failed checks
  * Corrected code format
  * Reformat report, correct typo
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
