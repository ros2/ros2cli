^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.9.6 (2020-06-23)
------------------

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
* Replace deprecated launch_ros usage (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* Update tests to expect no launch_ros node (`#474 <https://github.com/ros2/ros2cli/issues/474>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Just add warning at the top of the node names list (`#462 <https://github.com/ros2/ros2cli/issues/462>`_)
* Add a warning to ros2 node info when there is more than one node with the same name (`#463 <https://github.com/ros2/ros2cli/issues/463>`_)
* Remove ready_fn from test descriptions (`#376 <https://github.com/ros2/ros2cli/issues/376>`_)
* used get_available_rmw_implementations from rclpy (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
* Merge branch 'master' of github.com:ros2/ros2cli
* Make use of include-hidden flag for ros2node info verb (`#401 <https://github.com/ros2/ros2cli/issues/401>`_)
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
* add service clients to ros2node info (`#392 <https://github.com/ros2/ros2cli/issues/392>`_)
* Contributors: Michael Carroll, Mikael Arguedas

0.8.3 (2019-10-23)
------------------
* 0.8.3
* End-to-end test coverage for CLI commands output (`#304 <https://github.com/ros2/ros2cli/issues/304>`_)
* [ros2node] Add option to info verb to display hidden names (`#373 <https://github.com/ros2/ros2cli/issues/373>`_)
* [ros2node] Update info headings for actions (`#357 <https://github.com/ros2/ros2cli/issues/357>`_)
* Contributors: Jacob Perron, Michel Hidalgo, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* ros2node info: add action category (`#345 <https://github.com/ros2/ros2cli/issues/345>`_)
* Contributors: Claire Wang, Michael Carroll

0.8.0 (2019-09-26)
------------------
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Convert list comprehension to generator (`#314 <https://github.com/ros2/ros2cli/issues/314>`_)
* Alphasort ros2 node list output. (`#305 <https://github.com/ros2/ros2cli/issues/305>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo, Scott K Logan

0.7.4 (2019-05-29)
------------------

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* Solved bug when trying to find node info in namespaces (`#206 <https://github.com/ros2/ros2cli/issues/206>`_)
* Contributors: ivanpauno

0.6.3 (2019-02-08)
------------------
* Consistent node naming (`#158 <https://github.com/ros2/ros2cli/issues/158>`_)
* add completer for node info <node-name> (`#189 <https://github.com/ros2/ros2cli/issues/189>`_)
* Fix node info verb description (`#186 <https://github.com/ros2/ros2cli/issues/186>`_)
* Contributors: AAlon, Dirk Thomas, Jacob Perron

0.6.2 (2018-12-12)
------------------
* add slash for node name (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
* Contributors: Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
* Add ros2 node info verb (`#159 <https://github.com/ros2/ros2cli/issues/159>`_)
* Contributors: Ross Desmond, Shane Loretz

0.6.0 (2018-11-19)
------------------
* node name with namespace (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
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
* add pytest markers to linter tests
* ignore empty or None node names (`#76 <https://github.com/ros2/ros2cli/issues/76>`_)
* set zip_safe to avoid warning during installation (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* print full help when no command is passed (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* remove test_suite, add pytest as test_requires
* 0.0.3
* implicitly inherit from object (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
* various fixes and improvements
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
* add ros2node list
* Contributors: Dirk Thomas, Mikael Arguedas
