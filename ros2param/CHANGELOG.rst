^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2param
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fix flaky ros2 param list (`#656 <https://github.com/ros2/ros2cli/issues/656>`_) (`#664 <https://github.com/ros2/ros2cli/issues/664>`_)
* Contributors: Jacob Perron

0.9.9 (2021-03-24)
------------------
* 0.9.9
* Add wildcard loading to ros2 param load (`#602 <https://github.com/ros2/ros2cli/issues/602>`_)
* ros2 param dump/load should use fully qualified node names (`#600 <https://github.com/ros2/ros2cli/issues/600>`_)
* Add rosparam verb load (`#590 <https://github.com/ros2/ros2cli/issues/590>`_)
* Contributors: Audrow Nash, Ivan Santiago Paunovic, Victor Lopez

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
* [ros2param] Convert test_verb_dump into launch test (`#485 <https://github.com/ros2/ros2cli/issues/485>`_)
* Do not wait for entire timeout (`#486 <https://github.com/ros2/ros2cli/issues/486>`_)
* [ros2param] Add timeout to ros2param list (`#469 <https://github.com/ros2/ros2cli/issues/469>`_)
* [ros2param] Wait for discovery before running tests (`#481 <https://github.com/ros2/ros2cli/issues/481>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
* fix ros2param tests (`#441 <https://github.com/ros2/ros2cli/issues/441>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
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
* add ros2 param describe (`#367 <https://github.com/ros2/ros2cli/issues/367>`_)
* add completion for parameter name arguments (`#364 <https://github.com/ros2/ros2cli/issues/364>`_)
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
* install resource marker file for packages (`#339 <https://github.com/ros2/ros2cli/issues/339>`_)
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Adjusting ros2param tests to take into account automatic declaration of 'use_sim_time' parameter. (`#307 <https://github.com/ros2/ros2cli/issues/307>`_)
* Add param dump <node-name> (`#285 <https://github.com/ros2/ros2cli/issues/285>`_)
* Contributors: Dirk Thomas, Jacob Perron, Jeremie Deray, Juan Ignacio Ubeira

0.7.4 (2019-05-29)
------------------
* fix param list for hidden nodes (`#268 <https://github.com/ros2/ros2cli/issues/268>`_)
* fix param list for nodes which don't have the service (`#265 <https://github.com/ros2/ros2cli/issues/265>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
* use yaml.safe_load (round2) (`#229 <https://github.com/ros2/ros2cli/issues/229>`_)
* Add capability to use ros2 param set for array types (`#199 <https://github.com/ros2/ros2cli/issues/199>`_)
* Contributors: Mikael Arguedas, sgvandijk

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------

0.6.3 (2019-02-08)
------------------

0.6.2 (2018-12-12)
------------------
* add slash for node name (`#179 <https://github.com/ros2/ros2cli/issues/179>`_)
* Contributors: Karsten Knese

0.6.1 (2018-12-06)
------------------
* 0.6.1
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* node name with namespace (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
* Contributors: Dirk Thomas

0.5.4 (2018-08-20)
------------------
* add support for parameter prefixes in ros2 param list (`#131 <https://github.com/ros2/ros2cli/issues/131>`_)
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
* Specific message for unset parameters (`#104 <https://github.com/ros2/ros2cli/issues/104>`_)
* update ros2 param list output for a specific node (`#98 <https://github.com/ros2/ros2cli/issues/98>`_)
* add ros2 param (`#95 <https://github.com/ros2/ros2cli/issues/95>`_)
* Contributors: Dirk Thomas, dhood

0.4.0 (2017-12-08)
------------------
