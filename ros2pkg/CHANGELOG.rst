^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2pkg
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added changelogs
* Contributors: Dharini Dutia

0.9.12 (2022-09-12)
-------------------

0.9.11 (2022-01-31)
-------------------

0.9.10 (2021-10-05)
-------------------

0.9.9 (2021-03-24)
------------------
* 0.9.9
* Contributors: Audrow Nash

0.9.8 (2020-12-08)
------------------

0.9.7 (2020-07-07)
------------------
* fix incorrect EXPORT for executables (`#545 <https://github.com/ros2/ros2cli/issues/545>`_) (`#550 <https://github.com/ros2/ros2cli/issues/550>`_)
* Contributors: Dirk Thomas

0.9.6 (2020-06-23)
------------------

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
* use ament_export_targets() (`#478 <https://github.com/ros2/ros2cli/issues/478>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove ready_fn from test descriptions (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
* Merge pull request `#428 <https://github.com/ros2/ros2cli/issues/428>`_ from ros2/tfoote-patch-1
* Consistent interpretation of dependency type
* Contributors: Dirk Thomas, Peter Baughman, Steven! Ragnarök, Tully Foote

0.8.6 (2019-11-19)
------------------
* fix new linter warnings as of flake8-comprehensions 3.1.0 (`#399 <https://github.com/ros2/ros2cli/issues/399>`_)
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
* End-to-end test coverage for CLI commands output (`#304 <https://github.com/ros2/ros2cli/issues/304>`_)
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
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Add ros2 pkg create for ament python (`#296 <https://github.com/ros2/ros2cli/issues/296>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* rename package-format to match other flags (`#291 <https://github.com/ros2/ros2cli/issues/291>`_)
* Add pkg xml verb (`#280 <https://github.com/ros2/ros2cli/issues/280>`_)
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Ted Kern, jpace121

0.7.4 (2019-05-29)
------------------
* abort pkg create if destination directory exists (`#258 <https://github.com/ros2/ros2cli/issues/258>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------
* Add support for creating a package with format 3 (`#239 <https://github.com/ros2/ros2cli/issues/239>`_)
* Contributors: Jacob Perron

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------

0.6.3 (2019-02-08)
------------------
* use all linters by default (`#194 <https://github.com/ros2/ros2cli/issues/194>`_)
* Contributors: Dirk Thomas

0.6.2 (2018-12-12)
------------------
* use collections.abc.Iterable (`#177 <https://github.com/ros2/ros2cli/issues/177>`_)
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
* fix tests to use packages which register themselves at the index (`#118 <https://github.com/ros2/ros2cli/issues/118>`_)
* Contributors: Dirk Thomas

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* ros2pkg: avoid requiring git (`#111 <https://github.com/ros2/ros2cli/issues/111>`_)
* Implementation for `#89 <https://github.com/ros2/ros2cli/issues/89>`_ (`#96 <https://github.com/ros2/ros2cli/issues/96>`_)
* use catkin_pkg (`#94 <https://github.com/ros2/ros2cli/issues/94>`_)
* add pytest markers to linter tests
* add ament_package dependency (`#90 <https://github.com/ros2/ros2cli/issues/90>`_)
* Fix installation of templates for ros2pkg create (`#87 <https://github.com/ros2/ros2cli/issues/87>`_)
* Merge pull request `#85 <https://github.com/ros2/ros2cli/issues/85>`_ from ros2/avoid_builtin_use
* Avoid use of license as variable name
* set zip_safe to avoid warning during installation (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* ros2 pkg create (`#42 <https://github.com/ros2/ros2cli/issues/42>`_)
* print full help when no command is passed (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Karsten Knese, Mikael Arguedas, Nick Medveditskov, dhood

0.4.0 (2017-12-08)
------------------
* remove test_suite, add pytest as test_requires
* 0.0.3
* implicitly inherit from object (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* add ros2 pkg executables (`#23 <https://github.com/ros2/ros2cli/issues/23>`_)
* Merge pull request `#14 <https://github.com/ros2/ros2cli/issues/14>`_ from ros2/add_tests
* add unit tests
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
* add ros2pkg for listing packages and retrieve their prefix
* Contributors: Dirk Thomas, Mikael Arguedas
