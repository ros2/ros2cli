^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2component
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Ensure consistent timeout in ros2component list. (`#526 <https://github.com/ros2/ros2cli/issues/526>`_)
* Contributors: Claire Wang, Michel Hidalgo

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
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Only load required entry points which improves the performance. (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * Extend API to exclude extensions from loading
  * Add add_subparsers_on_demand() function
  * Update all extensions to use the new API
  * Deprecate old API, add deprecation warnings
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
* Add service clients to ros2node info. (`#392 <https://github.com/ros2/ros2cli/issues/392>`_)
* Contributors: Michael Carroll, Mikael Arguedas

0.8.3 (2019-10-23)
------------------
* 0.8.3
* [ros2node] Add option to info verb to display hidden names. (`#373 <https://github.com/ros2/ros2cli/issues/373>`_)
  * [ros2node] Add option to info verb to display hidden names
  Changes behavior so that hidden names are not shown by default.
  * Update ros2component to request hidden service names
  Which it uses for identifier component containers.
* Contributors: Jacob Perron, Shane Loretz

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
* Handle find_container_node_names error. (`#322 <https://github.com/ros2/ros2cli/issues/322>`_)
* Update setup.py version. (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* Install package manifest. (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Use of -r/--remap flags where appropriate. (`#325 <https://github.com/ros2/ros2cli/issues/325>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo, ivanpauno

0.7.4 (2019-05-29)
------------------
* [ros2component] Stop the standalone container when load fails. (`#269 <https://github.com/ros2/ros2cli/issues/269>`_)
  * Stop the standalone container when load fails
  This prevents zombie container processes from hanging around when a
  component fails to load.
  Closes: `ros2/ros2cli#260 <https://github.com/ros2/ros2cli/issues/260>`_
* Fix service names to contain 'srv' namespace part. (`#249 <https://github.com/ros2/ros2cli/issues/249>`_)
* Contributors: Dirk Thomas, Michael Carroll

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* Add xmllint linter test. (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * Add xmllint test to ament_python packages
  * Cover new packages as well
* Fix name of standalone verb. (`#227 <https://github.com/ros2/ros2cli/issues/227>`_)
* Improve ros2 component CLI. (`#226 <https://github.com/ros2/ros2cli/issues/226>`_)
  * Refactor ros2component internal API for reuse.
  In preparation for a standalone verb.
  * Add ros2component standalone verb.
  * Clean up ros2component dependencies.
  * Improve ros2component CLI verbs usability.
  * Address peer review comments.
* Contributors: Michel Hidalgo, Mikael Arguedas

0.7.1 (2019-04-17)
------------------
* Fix component package version.
* Fix component arguments to be inline with service. (`#220 <https://github.com/ros2/ros2cli/issues/220>`_)
* Add Component CLI. (`#217 <https://github.com/ros2/ros2cli/issues/217>`_)
  * Add ros2 component CLI verb package.
  * Fix ros2 component CLI verbs.
  * Improve ros2component CLI verbs and api documentation and style.
  * Add ros2component dummy API tests.
* Contributors: Michael Carroll, Michel Hidalgo

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
