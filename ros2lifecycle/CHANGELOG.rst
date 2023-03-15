^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

0.9.2 (2020-05-08)
------------------

0.9.1 (2020-05-06)
------------------

0.9.0 (2020-04-29)
------------------
* Stop using 'node_name' and 'node_namespace' in tests. (`#498 <https://github.com/ros2/ros2cli/issues/498>`_)
* Replace deprecated launch_ros usage (`#487 <https://github.com/ros2/ros2cli/issues/487>`_)
* Skip CLI tests on Windows until we resolve the blocking/hanging isuse. (`#489 <https://github.com/ros2/ros2cli/issues/489>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* used get_available_rmw_implementations from rclpy (`#461 <https://github.com/ros2/ros2cli/issues/461>`_)
* Add delay when retrying tests involving the CLI daemon (`#459 <https://github.com/ros2/ros2cli/issues/459>`_)
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Jacob Perron, Steven! Ragnarök

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* [ros2lifecycle] Add test coverage for CLI (`#391 <https://github.com/ros2/ros2cli/issues/391>`_)
* [ros2lifecycle] Misc fixes. (`#395 <https://github.com/ros2/ros2cli/issues/395>`_)
* Contributors: Michael Carroll, Michel Hidalgo

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
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.7.4 (2019-05-29)
------------------
* [ros2lifecycle] Only return the state for the node requested (`#266 <https://github.com/ros2/ros2cli/issues/266>`_)
* Contributors: Jacob Perron

0.7.3 (2019-05-20)
------------------
* use new type identification for lifecycle nodes (`#241 <https://github.com/ros2/ros2cli/issues/241>`_)
* Contributors: Karsten Knese

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* ros2lifecycle `nodes` and `get` verbs, now showing hidden nodes when requested (`#208 <https://github.com/ros2/ros2cli/issues/208>`_)
* Contributors: ivanpauno

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------
* add slash for node name (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
* Remove unused cli option (`#174 <https://github.com/ros2/ros2cli/issues/174>`_)
* fix ros2 lifecycle get (`#167 <https://github.com/ros2/ros2cli/issues/167>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* Lifecycle refactor (`#150 <https://github.com/ros2/ros2cli/issues/150>`_)
* comply with new node representation (`#149 <https://github.com/ros2/ros2cli/issues/149>`_)
* Contributors: Karsten Knese

0.5.4 (2018-08-20)
------------------
* remove apparently unused yaml dependency (`#130 <https://github.com/ros2/ros2cli/issues/130>`_)
* Contributors: Mikael Arguedas

0.5.3 (2018-07-17)
------------------

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* ros2lifecycle: fix dep and import (`#103 <https://github.com/ros2/ros2cli/issues/103>`_)
* make flake8 import order happy
* add ros2 lifecycle (`#97 <https://github.com/ros2/ros2cli/issues/97>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
