^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.14.0 (2021-04-26)
-------------------
* Ensure only one daemon can run at a time (`#622 <https://github.com/ros2/ros2cli/issues/622>`_)
  * Substitute SO_REUSEADDR by SO_LINGER.
  * Wait for daemon shutdown before testing NodeStrategy
* Contributors: Michel Hidalgo

0.13.0 (2021-04-06)
-------------------

0.12.0 (2021-03-02)
-------------------
* 0.12.0
* remove maintainer (`#597 <https://github.com/ros2/ros2cli/issues/597>`_)
* add option to support use_sim_time. (`#581 <https://github.com/ros2/ros2cli/issues/581>`_)
  * add option to support use_sim_time.
  * requirement is delay, not bandwidth.
  * add test for DirectNode Class.
* bugfix for `#563 <https://github.com/ros2/ros2cli/issues/563>`_ (`#570 <https://github.com/ros2/ros2cli/issues/570>`_)
* Add Audrow as a maintainer (`#591 <https://github.com/ros2/ros2cli/issues/591>`_)
* Contributors: Audrow Nash, Claire Wang, Daisuke Sato, Tomoya Fujita

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
* Added dependency to python3-argcomplete to ros2cli (`#564 <https://github.com/ros2/ros2cli/issues/564>`_)
  * Added dependency to python3-argcomplete to ros2cli
* Remove use of pkg_resources from ros2cli.
  Replace it with the use of the more modern importlib*
  libraries.  There are a couple of reasons to do this:
  1.  pkg_resources is quite slow to import; on my machine,
  just firing up the python interpreter takes ~35ms, while
  firing up the python interpreter and importing pkg_resources
  takes ~175ms.  Firing up the python interpreter and importing
  importlib_metadata takes ~70ms.  Removing 100ms per invocation
  of the command-line both makes it speedier for users, and
  will speed up our tests (which call out to the command-line
  quite a lot).
  2.  pkg_resources is somewhat deprecated and being replaced
  by importlib.  https://importlib-metadata.readthedocs.io/en/latest/using.html
  describes some of it
  Note: this change definitively breaks doing releases with
  Ubuntu Bionic, as python3-importlib-metadata is not available in Bionic
  (though it is available via pip).
  Note: By itself, this change is not enough to completely remove our
  dependence on pkg_resources.  We'll also have to do something about
  the console_scripts that setup.py generates.  That will be
  a separate effort.
* Contributors: Chris Lalancette, Claire Wang, Yoan Mollard

0.9.5 (2020-06-01)
------------------

0.9.4 (2020-05-26)
------------------

0.9.3 (2020-05-13)
------------------
* Fix new flake8 errors. (`#509 <https://github.com/ros2/ros2cli/issues/509>`_)
* Improve NodeStrategy to use the right node seamlessly. (`#499 <https://github.com/ros2/ros2cli/issues/499>`_)
* Contributors: Michel Hidalgo

0.9.2 (2020-05-08)
------------------
* Make test_daemon.py robust to discovery latency. (`#504 <https://github.com/ros2/ros2cli/issues/504>`_)
* Contributors: Michel Hidalgo

0.9.1 (2020-05-06)
------------------
* add support for get_node_names_and_namespaces_with_enclaves (`#501 <https://github.com/ros2/ros2cli/issues/501>`_)
* Contributors: Mikael Arguedas

0.9.0 (2020-04-29)
------------------
* Extend CLI daemon's API (`#493 <https://github.com/ros2/ros2cli/issues/493>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* use f-string (`#448 <https://github.com/ros2/ros2cli/issues/448>`_)
  * use f-string
  * remove unused variable
* override parse_known_args for completion to work with partial argparse tree (`#446 <https://github.com/ros2/ros2cli/issues/446>`_)
  * override parse_known_args for completion to work with partial argparse tree
  * fix completion of partial first level command
  * fix spelling in comment
* Avoid passing None to rclpy.init (`#433 <https://github.com/ros2/ros2cli/issues/433>`_)
  Otherwise, all CLI arguments will be parsed as ROS arguments, which can lead
  to rcl warnings or incorrect legacy remapping behavior.
  This change does not pass arguments to rclpy.init from any of the CLI
  tools, but it leaves the opportunity to do so in the future by setting the
  parser argument 'argv'. For example, we could take the remaining arguments
  and pass them to rclpy.init, similar to what is done in ros2run:
  https://github.com/ros2/ros2cli/blob/4c5d9327026ecb2ea10a16b3429908b4f6f64ca6/ros2run/ros2run/command/run.py#L51-L53
  Fixes `#336 <https://github.com/ros2/ros2cli/issues/336>`_.
* fix linter warning about bad quotes (`#438 <https://github.com/ros2/ros2cli/issues/438>`_)
* pass argv to CommandExtension.add_arguments if available (`#437 <https://github.com/ros2/ros2cli/issues/437>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
  * extend API to exclude extensions from loading
  * add add_subparsers_on_demand() function
  * update all extensions to use the new API
  * deprecate old API, add deprecation warnings
* Extend CLI daemon based features (`#420 <https://github.com/ros2/ros2cli/issues/420>`_)
  * Extend CLI daemon ROS graph API support.
  * Add --no-daemon option for strategy nodes.
  * Drop redundant default for --no-daemon.
  * Make ROS graph API support complete.
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Fix ros2 topic pub --node-name (`#398 <https://github.com/ros2/ros2cli/issues/398>`_)
  * Fix ros2 topic pub --node-name
  * Give DirectNode node_name kwarg
  * not node_name -> node_name is None
* Contributors: Shane Loretz

0.8.4 (2019-11-13)
------------------
* 0.8.4
* Contributors: Michael Carroll

0.8.3 (2019-10-23)
------------------
* 0.8.3
* Make daemon “reset” itself when the IP address changes (`#284 <https://github.com/ros2/ros2cli/issues/284>`_)
* Contributors: Ivan Santiago Paunovic, Shane Loretz

0.8.2 (2019-10-08)
------------------
* 0.8.2
* fix sourcing completion scripts in Debian package (`#353 <https://github.com/ros2/ros2cli/issues/353>`_)
  * fix sourcing completion scripts in Debian package
  * fix path
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* Contributors: Michael Carroll

0.8.0 (2019-09-26)
------------------
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
  Versions now match latest tag and package.xml.
* install package manifest (`#330 <https://github.com/ros2/ros2cli/issues/330>`_)
* Contributors: Dirk Thomas, Jacob Perron

0.7.4 (2019-05-29)
------------------
* update help of --spin-time (`#253 <https://github.com/ros2/ros2cli/issues/253>`_)
* don't start parameter service in daemon (`#251 <https://github.com/ros2/ros2cli/issues/251>`_)
* fix sourcing argcomplete script in zsh (`#243 <https://github.com/ros2/ros2cli/issues/243>`_)
* Contributors: Dirk Thomas

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* add xmllint linter test (`#232 <https://github.com/ros2/ros2cli/issues/232>`_)
  * add xmllint test to ament_python packages
  * cover new packages as well
* Contributors: Mikael Arguedas

0.7.1 (2019-04-17)
------------------

0.7.0 (2019-04-14)
------------------
* update logger.warn (deprecated) to logger.warning (`#205 <https://github.com/ros2/ros2cli/issues/205>`_)
* Contributors: Dirk Thomas

0.6.3 (2019-02-08)
------------------
* Consistent node naming (`#158 <https://github.com/ros2/ros2cli/issues/158>`_)
  * Support for easy integration with ros2 security features by starting CLI nodes with a consistent prefix.
  * Removing unneeded comment
  * Making DirectNode visible (removing hidden node prefix) to have consistent node naming for ros2cli.
  * Start all CLI nodes as hidden.
  * Shortening the default CLI node name prefix from '_ros2cli_node' to '_ros2cli'
  * Importing HIDDEN_NODE_PREFIX from rclpy, renaming CLI_NODE_NAME_PREFIX -> NODE_NAME_PREFIX.
  * ros2node - Importing HIDDEN_NODE_PREFIX from rclpy
  * Linter fixes.
* Contributors: AAlon

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------
* 0.6.1
  bump package.xml, setup.py and setup.cfg versions
* Contributors: Shane Loretz

0.6.0 (2018-11-19)
------------------
* node name with namespace (`#146 <https://github.com/ros2/ros2cli/issues/146>`_)
* Contributors: Dirk Thomas

0.5.4 (2018-08-20)
------------------

0.5.3 (2018-07-17)
------------------
* Check rmw identifier (`#121 <https://github.com/ros2/ros2cli/issues/121>`_)
  * daemin -> daemon
  * check rmw implementation identifier before calling the daemon
  This allows to raise with an appropriate error message if the default rmw implementation is not installed on the system
  * trailing spaces
* Contributors: Mikael Arguedas

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* hide window of spawned daemon (`#113 <https://github.com/ros2/ros2cli/issues/113>`_)
* cancel timer before letting caller use the node to avoid spurious wakeups for consumers (`#115 <https://github.com/ros2/ros2cli/issues/115>`_)
  * cancel timer before letting caller use the node to avoid spurious wakeups for consumers
  * make timer local and destroy it after use
* use (bash)compinit for zsh completion (`#102 <https://github.com/ros2/ros2cli/issues/102>`_)
* add colcon.pkg file to source completion scripts (`#101 <https://github.com/ros2/ros2cli/issues/101>`_)
* add pytest markers to linter tests
* ignore F841 from latest Pyflakes release (`#93 <https://github.com/ros2/ros2cli/issues/93>`_)
* source bash completion script from setup file (`#84 <https://github.com/ros2/ros2cli/issues/84>`_)
  * source bash completion script from setup file
  * add zsh specific local_setup file
* set zip_safe to avoid warning during installation (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* use rmw agnostic daemon URL (`#80 <https://github.com/ros2/ros2cli/issues/80>`_)
* print full help when no command is passed (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* fix import order (`#79 <https://github.com/ros2/ros2cli/issues/79>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* Merge pull request `#71 <https://github.com/ros2/ros2cli/issues/71>`_ from ros2/wait_until_daemon_has_started
  'daemon start' waits until it has been started before returning
* 'daemon start' waits until it has been started before returning
* remove test_suite, add pytest as test_requires
* 0.0.3
* Merge pull request `#49 <https://github.com/ros2/ros2cli/issues/49>`_ from ros2/msg_stopping_daemon_diff_rmw
  add error message when trying to stop a daemon using a different rmw implementation
* add error message when trying to stop a daemon using a different rmw implementation
* Merge pull request `#46 <https://github.com/ros2/ros2cli/issues/46>`_ from ros2/flake8_plugins
  update style to satisfy new flake8 plugins
* update style to satisfy new flake8 plugins
* implicitly inherit from object (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* fix daemon verb
* Merge pull request `#38 <https://github.com/ros2/ros2cli/issues/38>`_ from ros2/add_daemon_command
  add daemon command with verbs status, start, stop
* Merge pull request `#33 <https://github.com/ros2/ros2cli/issues/33>`_ from ros2/improve_windows_daemon
  Improve windows daemon
* add daemon command with verbs status, start, stop
* use different cwd for daemon to prevent holding folder handle
* detach daemon on Windows
* add exec_depend on python3-pkg-resources (`#30 <https://github.com/ros2/ros2cli/issues/30>`_)
* Merge pull request `#29 <https://github.com/ros2/ros2cli/issues/29>`_ from ros2/hide_help_from_completion
  hide help options from completion
* hide help options from completion
* Merge pull request `#26 <https://github.com/ros2/ros2cli/issues/26>`_ from ros2/support_argcomplete_py3
  support python3-argcomplete
* support python3-argcomplete
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
  various fixes and improvements
* various fixes and improvements
* Merge pull request `#11 <https://github.com/ros2/ros2cli/issues/11>`_ from ros2/daemon_rmw_impl
  update daemon to only handle local requests for matching rmw impl
* update daemon to only handle requests from localhost
* update daemon to only handle requests from matching rmw impl.
* Merge pull request `#7 <https://github.com/ros2/ros2cli/issues/7>`_ from ros2/zsh_argcomplete
  add argcomplete script for zsh
* add argcomplete script for zsh
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
  add pep257 tests
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
  Entry point, plugin system, daemon, existing tools
* append pid / domain id to node names
* add suffix to node name in daemon (`#2 <https://github.com/ros2/ros2cli/issues/2>`_)
* add linter tests
* add rclpy node interface and xml-rpc based daemon
* add argcomplete-based completion
* add hidden extension commands
* add ros2cli plugin system, interface for commands and verbs, and cli
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
