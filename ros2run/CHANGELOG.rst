^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2run
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.10 (2024-05-15)
--------------------

0.18.9 (2024-02-16)
-------------------
* Conditionally use valid_signals (`#880 <https://github.com/ros2/ros2cli/issues/880>`_)
* Contributors: Chris Lalancette

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

0.15.0 (2021-11-18)
-------------------
* Update maintainers to Aditya Pande, Audrow Nash, and Michael Jeronimo (`#673 <https://github.com/ros2/ros2cli/issues/673>`_)
* Updated maintainers (`#670 <https://github.com/ros2/ros2cli/issues/670>`_)
* check subprocess.returncode to print error message. (`#639 <https://github.com/ros2/ros2cli/issues/639>`_)
* Add changelogs (`#635 <https://github.com/ros2/ros2cli/issues/635>`_)
* Contributors: Aditya Pande, Audrow Nash, Ivan Santiago Paunovic, Tomoya Fujita

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
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * Use f-string
  * Remove unused variable
* Contributors: Dirk Thomas

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

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
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
* Add pytest markers to linter tests
* Set zip_safe to avoid warning during installation. (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* Improve parameters passing to node in ros2run. (`#61 <https://github.com/ros2/ros2cli/issues/61>`_)
  * Improve parameter passing to node in ros2run
  * Enforce PEP
  * Fix remaining warning
  * Move argparse import
* Remove test_suite, add pytest as test_requires
* 0.0.3
* Merge pull request `#53 <https://github.com/ros2/ros2cli/issues/53>`_ from ros2/invoke_python_script_on_windows_with_interpreter
  invoke Python scripts on Windows with interpreter
* Invoke Python scripts on Windows with interpreter
* Merge pull request `#50 <https://github.com/ros2/ros2cli/issues/50>`_ from ros2/fix_sigint_ros2_run
  fix SIGINT handling in ros2 run
* Fix SIGINT handling in ros2 run
* Merge pull request `#46 <https://github.com/ros2/ros2cli/issues/46>`_ from ros2/flake8_plugins
  update style to satisfy new flake8 plugins
* Update style to satisfy new flake8 plugins
* Implicitly inherit from object. (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* Merge pull request `#43 <https://github.com/ros2/ros2cli/issues/43>`_ from ros2/run_suppress_prefix_completion
  run: suppress prefix completion
* Run: suppress prefix completion
* Added --prefix argument to ros2 run. (`#41 <https://github.com/ros2/ros2cli/issues/41>`_)
  * Added --prefix argument to ros2 run
  * Fixed issues from review by @dirk-thomas and @wjwwood
  * Rephrased help message
  * Added space in help message
* 0.0.2
* Add ros2 pkg executables. (`#23 <https://github.com/ros2/ros2cli/issues/23>`_)
  * Add ros2 pkg executables
  * Print basenames by default, option to print full path
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
  various fixes and improvements
* Various fixes and improvements
* Removed node from call to get_executable path in ros2 run command,. (`#13 <https://github.com/ros2/ros2cli/issues/13>`_)
  see `ros2/ros2cli#12 <https://github.com/ros2/ros2cli/issues/12>`_.
* Merge pull request `#12 <https://github.com/ros2/ros2cli/issues/12>`_ from ros2/improve_executable_selection
  Improve executable selection
* Consider PATHEXT on Windows
* Remove unused node arg
* Merge pull request `#3 <https://github.com/ros2/ros2cli/issues/3>`_ from ros2/more_commands
  add more commands
* Add ros2run
* Contributors: Dirk Thomas, Hunter Allen, Mikael Arguedas, alexandre eudes, volkandre
