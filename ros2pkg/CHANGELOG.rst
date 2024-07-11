^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2pkg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.25.6 (2024-04-19)
-------------------

0.25.5 (2024-02-07)
-------------------

0.25.4 (2023-11-17)
-------------------

0.25.3 (2023-09-08)
-------------------

0.25.2 (2023-07-14)
-------------------

0.25.1 (2023-05-11)
-------------------
* Fix typo in ros2pkg warning message. (`#828 <https://github.com/ros2/ros2cli/issues/828>`_)
* Contributors: Tomoya Fujita

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
* resolve `#790 <https://github.com/ros2/ros2cli/issues/790>`_ (`#801 <https://github.com/ros2/ros2cli/issues/801>`_)
* Add alias library targets for CMake (`#718 <https://github.com/ros2/ros2cli/issues/718>`_)
* [rolling] Update maintainers - 2022-11-07 (`#776 <https://github.com/ros2/ros2cli/issues/776>`_)
* Contributors: Audrow Nash, Kenji Brameld, RFRIEDM-Trimble

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
* Use local git config instead of global (`#693 <https://github.com/ros2/ros2cli/issues/693>`_)
* Contributors: Amro Al-Baali

0.17.1 (2022-01-25)
-------------------

0.16.1 (2022-01-14)
-------------------

0.16.0 (2022-01-14)
-------------------
* Depend on launch packages instead of ros_testing to avoid circular dependency (`#685 <https://github.com/ros2/ros2cli/issues/685>`_)
* [ros2pkg] Skip copyright tests in template packages (`#676 <https://github.com/ros2/ros2cli/issues/676>`_)
* Contributors: Abrar Rahman Protyasha, Shane Loretz

0.15.0 (2021-11-18)
-------------------
* Update maintainers to Aditya Pande, Audrow Nash, and Michael Jeronimo (`#673 <https://github.com/ros2/ros2cli/issues/673>`_)
* Updated maintainers (`#670 <https://github.com/ros2/ros2cli/issues/670>`_)
* Generate LICENSE files on ros2 pkg create. (`#650 <https://github.com/ros2/ros2cli/issues/650>`_)
* Handle ValueError (`#643 <https://github.com/ros2/ros2cli/issues/643>`_)
* Pass package exports to template in pkg create api (`#619 <https://github.com/ros2/ros2cli/issues/619>`_) (`#628 <https://github.com/ros2/ros2cli/issues/628>`_)
* Add changelogs (`#635 <https://github.com/ros2/ros2cli/issues/635>`_)
* Contributors: Aditya Pande, Audrow Nash, Chris Lalancette, Ivan Santiago Paunovic, rob-clarke, tim-fan

0.14.0 (2021-04-26)
-------------------
* Use underscores in setup.cfg.em instead of dashes. (`#627 <https://github.com/ros2/ros2cli/issues/627>`_)
* Contributors: Ivan Santiago Paunovic

0.13.0 (2021-04-06)
-------------------
* Add space for "ROS 2". (`#617 <https://github.com/ros2/ros2cli/issues/617>`_)
* Use target_compile_features for c/c++ standards. (`#615 <https://github.com/ros2/ros2cli/issues/615>`_)
* Contributors: Chris Lalancette, Shane Loretz

0.12.0 (2021-03-02)
-------------------
* Remove maintainer. (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* Add Audrow as a maintainer. (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang

0.11.0 (2021-01-25)
-------------------
* Declare missing dependency on python3-importlib-resources. (`#584 <https://github.com/ros2/ros2cli/issues/584>`_)
* Contributors: Scott K Logan

0.10.1 (2020-12-08)
-------------------

0.10.0 (2020-11-02)
-------------------
* Update maintainers. (`#568 <https://github.com/ros2/ros2cli/issues/568>`_)
* Fix incorrect EXPORT for executables. (`#545 <https://github.com/ros2/ros2cli/issues/545>`_)
* Switch ros2pkg to using importlib.
* Contributors: Chris Lalancette, Claire Wang, Dirk Thomas

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
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* Use ament_export_targets(). (`#478 <https://github.com/ros2/ros2cli/issues/478>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions. (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* Use f-string. (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * Use f-string
  * Remove unused variable
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
* Merge pull request `#428 <https://github.com/ros2/ros2cli/issues/428>`_ from ros2/tfoote-patch-1
  fix create_pkg dependencies for python
* Consistent interpretation of dependency type
* Contributors: Dirk Thomas, Peter Baughman, Steven! Ragnarök, Tully Foote

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
* Contributors: Michel Hidalgo, Shane Loretz

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
* Add ros2 pkg create for ament python. (`#296 <https://github.com/ros2/ros2cli/issues/296>`_)
  * Add pkg create for ament_python.
  * Add entries for setup.py to make it closer to package.xml.
  * Fix yucky formatting.
  * Provide default node name for python if one is not provided. Refactor how cpp node names are handle so we do sane things if using python build tool, but pass in a cpp node name.
  * Fix formatting issues brought up by colcon test.
  * PR feedback.
  * Replace --cpp-node-name and --python-node-name options with --node-name option.
  * Handle library_name option for ament_python packages.
  * Add default linters to ament_python packages.
  * Abort when package name = test.
  * PR feedback: Remove unneccessary new lines.
  * PR Feedback: Improve error message.
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Rename package-format to match other flags. (`#291 <https://github.com/ros2/ros2cli/issues/291>`_)
* Add pkg xml verb. (`#280 <https://github.com/ros2/ros2cli/issues/280>`_)
  * Add pkg xml verb
  * Fix helper wording
  * Fix logic & default tag opt
  * Typo
  * Fix pkg name autocomplete
  * Exit earlier & use None actual type
  * Minor updates
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Ted Kern, jpace121

0.7.4 (2019-05-29)
------------------
* Abort pkg create if destination directory exists. (`#258 <https://github.com/ros2/ros2cli/issues/258>`_)
  * Abort pkg create if destination directory exists
  * Move check up
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------
* Add support for creating a package with format 3. (`#239 <https://github.com/ros2/ros2cli/issues/239>`_)
  * Add support for creating a package with format 3
  * Default to format 3
* Contributors: Jacob Perron

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
* Use all linters by default. (`#194 <https://github.com/ros2/ros2cli/issues/194>`_)
* Contributors: Dirk Thomas

0.6.2 (2018-12-12)
------------------
* Use collections.abc.Iterable. (`#177 <https://github.com/ros2/ros2cli/issues/177>`_)
* Contributors: Dirk Thomas

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
* Fix tests to use packages which register themselves at the index. (`#118 <https://github.com/ros2/ros2cli/issues/118>`_)
* Contributors: Dirk Thomas

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* Ros2pkg: avoid requiring git. (`#111 <https://github.com/ros2/ros2cli/issues/111>`_)
* Implementation for `#89 <https://github.com/ros2/ros2cli/issues/89>`_. (`#96 <https://github.com/ros2/ros2cli/issues/96>`_)
  * Implementation for `#89 <https://github.com/ros2/ros2cli/issues/89>`_
  ``` bash
  $ ros2 pkg prefix ament_flake8
  /home/nick/ros2_ws/install
  $ ros2 pkg prefix --share ament_flake8
  /home/nick/ros2_ws/install
  Share dir: /home/nick/ros2_ws/install/share/ament_flake8
  $ ros2 pkg prefix -h
  usage: ros2 pkg prefix [-h] [--share] package_name
  Output the prefix path of a package
  positional arguments:
  package_name  The package name
  optional arguments:
  -h, --help    show this help message and exit
  --share       show share directory for the package
  ```
  * Revert "Implementation for `#89 <https://github.com/ros2/ros2cli/issues/89>`_"
  This reverts commit 8bec852
  * Implementation for `#89 <https://github.com/ros2/ros2cli/issues/89>`_, addressing PR feedback
* Use catkin_pkg. (`#94 <https://github.com/ros2/ros2cli/issues/94>`_)
* Add pytest markers to linter tests
* Add ament_package dependency. (`#90 <https://github.com/ros2/ros2cli/issues/90>`_)
* Fix installation of templates for ros2pkg create. (`#87 <https://github.com/ros2/ros2cli/issues/87>`_)
  * Move resource dir out of package
  * Install templates to share dir as data_files
  * Remove unnecessary __file_\_ prefix
  * Revert "Move resource dir out of package"
  This reverts commit 90556f6313c0f2ad996488c6a8b873c658d6627c.
  * Swap to package_data installation
  Will install to e.g. install_isolated/ros2pkg/lib/python3.5/site-packages/ros2pkg/resource
  which is where it was being looked for previously
  * Add exec_depends for third party python packages
  * Set zip_safe to True
  * Add/use _get_template_path
  Prevents resource_filename from extracting whole directory if
  installation is zipped
  * Embed _get_template_path contents
  * Exec depend on python3-pkg-resources
* Merge pull request `#85 <https://github.com/ros2/ros2cli/issues/85>`_ from ros2/avoid_builtin_use
  Avoid use of license as variable name
* Avoid use of license as variable name
* Set zip_safe to avoid warning during installation. (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* Ros2 pkg create. (`#42 <https://github.com/ros2/ros2cli/issues/42>`_)
  * Initial commit for ros2 pkg create
  * Managed interpreter
  * Cleanup cmakelists.txt.em
  * Single quote prints
  * Alpha order
  * Copy paste error correction
  * Ament-cmake -> ament_cmake
  * Fix typo
  * Style
  * Correct line breaks in cmake
  * Enhance CMakeLists.txt with testing section
  * Clear separation between ament_cmake and plain cmake
  * Whitespace
  * Alpha order
  * Import order
  * Use platform for uname
  * Address style comments
  * Add include and header file if building library
  * Use git config to get email and username
  * Use os.curdir
  * Cleanup cmake config
  * Address style comments
  * Disable some linters
  * Cleanup prints
  * Print error message in a single statement
  * Consolidate block
  * Add comment about disabled linters
  * Switch back to format 2 for now
  * Use build type in help text rather than build tool
  * Build type
  * Deb --> dep
  * Ament_common --> ament_lint_common
  * Use target_include_directories
  * Target_include_dir and export
  * Export symbols on plain cmake
  * Use library/node name for targets
  * Naming convention for export targets
  * Rethink nargs
  * Using ament_package data types
  * Linters
  * Set correct values in package.xml
  * Clean up package.xml with buldtool_depends and test_depends
* Print full help when no command is passed. (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Karsten Knese, Mikael Arguedas, Nick Medveditskov, dhood

0.4.0 (2017-12-08)
------------------
* Remove test_suite, add pytest as test_requires
* 0.0.3
* Implicitly inherit from object. (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Add ros2 pkg executables. (`#23 <https://github.com/ros2/ros2cli/issues/23>`_)
  * Add ros2 pkg executables
  * Print basenames by default, option to print full path
* Merge pull request `#14 <https://github.com/ros2/ros2cli/issues/14>`_ from ros2/add_tests
  add unit tests
* Add unit tests
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
  add pep257 tests
* Add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
  Entry point, plugin system, daemon, existing tools
* Add ros2pkg for listing packages and retrieve their prefix
* Contributors: Dirk Thomas, Mikael Arguedas
