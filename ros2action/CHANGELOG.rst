^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.11 (2024-07-26)
--------------------

0.18.10 (2024-05-15)
--------------------
* call get_action_interfaces() properly. (`#898 <https://github.com/ros2/ros2cli/issues/898>`_) (`#902 <https://github.com/ros2/ros2cli/issues/902>`_)
* Contributors: mergify[bot]

0.18.9 (2024-02-16)
-------------------

0.18.8 (2024-01-24)
-------------------

0.18.7 (2023-07-18)
-------------------

0.18.6 (2023-04-25)
-------------------

0.18.5 (2023-01-10)
-------------------

0.18.4 (2022-11-07)
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
* Contributors: Michel Hidalgo

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------

0.9.0 (2020-04-29)
------------------
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
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
* [ros2action] Refactor send_goal implementation. (`#406 <https://github.com/ros2/ros2cli/issues/406>`_)
  Resolve a TODO and use a function from rosidl_runtime_py to get the action interface.
* Merge branch 'master' of github.com:ros2/ros2cli
* [ros2action] Remove show verb. (`#405 <https://github.com/ros2/ros2cli/issues/405>`_)
  The verb is redundant with 'ros2 interface show'.
  Equivalent tests for the ones removed already exist in ros2interface.
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Jacob Perron, Peter Baughman, Steven! Ragnarök, claireyywang

0.8.6 (2019-11-19)
------------------
* Fix new linter warnings as of flake8-comprehensions 3.1.0. (`#399 <https://github.com/ros2/ros2cli/issues/399>`_)
* Contributors: Dirk Thomas

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
* Ensure ros2 interface show has trailing newline. (`#368 <https://github.com/ros2/ros2cli/issues/368>`_)
* Contributors: Dirk Thomas, Michel Hidalgo, Shane Loretz

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
* Pass keyword arguments by name. (`#317 <https://github.com/ros2/ros2cli/issues/317>`_)
* Add action send_goal prototype completer. (`#301 <https://github.com/ros2/ros2cli/issues/301>`_)
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray

0.7.4 (2019-05-29)
------------------
* [ros2action] Support multiple part action type names for 'send_goal' verb. (`#261 <https://github.com/ros2/ros2cli/issues/261>`_)
  Similar to the changes made in `#247 <https://github.com/ros2/ros2cli/issues/247>`_ and `#259 <https://github.com/ros2/ros2cli/issues/259>`_.
* Use three-part interface names in msg/srv/action show and msg/srv/ list. (`#259 <https://github.com/ros2/ros2cli/issues/259>`_)
* Reset goal_handle to avoid attempt to cancel. (`#254 <https://github.com/ros2/ros2cli/issues/254>`_)
  * Reset goal_handle to avoid attempt to cancel
  * Fix spelling
* Contributors: Dirk Thomas, Jacob Perron

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
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* Add Action CLI. (`#214 <https://github.com/ros2/ros2cli/issues/214>`_)
  * Add ros2action package
  Contains ros2cli command 'action' with verbs: list and show.
  The list verb lists action names for any running action servers and action clients.
  The show verb prints the definition for a given action type.
  * Add 'info' verb to action command
  Prints a list of node names that have an action client or server for a given action name.
  * Use None as argument to test node
  * Add TODOs to move action query functions to rclpy (and rcl_action)
  The tool shouldn't need to know details about the implementation of actions.
  * Add dependency to rclpy
  * Add 'send_goal' verb to action command
  * Migrate message utility functions to rosidl_runtime_py
  * Make use of rclpy functions
  * Fix lint
  * Fix tests
  * Fix test
  * Add autocompletion to verbs
  * Update year
  * Expand and validate action name
  This also has the side-effect of making the forward slash optional for the action name.
  * Print goal ID when sendind a goal
  * Cancel goal on SIGINT
  Wrapped send goal logic in try-finally clause.
  This ensures that any active goal will be canceled before the CLI command terminates and also ensure that the ROS node is shutdown.
  * Fix typos
  * Change maintainer
  * Move try-except to verb
  * Catch expected exceptions only
* Contributors: Jacob Perron

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
