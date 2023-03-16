^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.13 (2023-03-16)
-------------------
* Added changelogs
* Contributors: Dharini Dutia

0.9.12 (2022-09-12)
-------------------
* XMLRPC server accepts request from all local IP addresses. (`#729 <https://github.com/ros2/ros2cli/issues/729>`_) (`#735 <https://github.com/ros2/ros2cli/issues/735>`_)
* Improve performance of commands for Foxy on Windows by 4x (`#686 <https://github.com/ros2/ros2cli/issues/686>`_)
* Contributors: Tomoya Fujita, Akash, mergify[bot]

0.9.11 (2022-01-31)
-------------------

0.9.10 (2021-10-05)
-------------------

0.9.9 (2021-03-24)
------------------
* 0.9.9
* bugfix for `#563 <https://github.com/ros2/ros2cli/issues/563>`_ (`#570 <https://github.com/ros2/ros2cli/issues/570>`_) (`#609 <https://github.com/ros2/ros2cli/issues/609>`_)
* Contributors: Daisuke Sato, Audrow Nash, Tomoya Fujita

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
* override parse_known_args for completion to work with partial argparse tree (`#446 <https://github.com/ros2/ros2cli/issues/446>`_)
* Avoid passing None to rclpy.init (`#433 <https://github.com/ros2/ros2cli/issues/433>`_)
* fix linter warning about bad quotes (`#438 <https://github.com/ros2/ros2cli/issues/438>`_)
* pass argv to CommandExtension.add_arguments if available (`#437 <https://github.com/ros2/ros2cli/issues/437>`_)
* only load required entry points which improves the performance (`#436 <https://github.com/ros2/ros2cli/issues/436>`_)
* Extend CLI daemon based features (`#420 <https://github.com/ros2/ros2cli/issues/420>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo

0.8.6 (2019-11-19)
------------------

0.8.5 (2019-11-14)
------------------
* 0.8.5
* Fix ros2 topic pub --node-name (`#398 <https://github.com/ros2/ros2cli/issues/398>`_)
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
* Contributors: Dirk Thomas

0.8.1 (2019-10-04)
------------------
* 0.8.1
* Contributors: Michael Carroll

0.8.0 (2019-09-26)
------------------
* Update setup.py version (`#331 <https://github.com/ros2/ros2cli/issues/331>`_)
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
* Contributors: AAlon

0.6.2 (2018-12-12)
------------------

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

0.5.3 (2018-07-17)
------------------
* Check rmw identifier (`#121 <https://github.com/ros2/ros2cli/issues/121>`_)
* Contributors: Mikael Arguedas

0.5.2 (2018-06-28)
------------------

0.5.1 (2018-06-27 12:27)
------------------------

0.5.0 (2018-06-27 12:17)
------------------------
* hide window of spawned daemon (`#113 <https://github.com/ros2/ros2cli/issues/113>`_)
* cancel timer before letting caller use the node to avoid spurious wakeups for consumers (`#115 <https://github.com/ros2/ros2cli/issues/115>`_)
* use (bash)compinit for zsh completion (`#102 <https://github.com/ros2/ros2cli/issues/102>`_)
* add colcon.pkg file to source completion scripts (`#101 <https://github.com/ros2/ros2cli/issues/101>`_)
* add pytest markers to linter tests
* ignore F841 from latest Pyflakes release (`#93 <https://github.com/ros2/ros2cli/issues/93>`_)
* source bash completion script from setup file (`#84 <https://github.com/ros2/ros2cli/issues/84>`_)
* set zip_safe to avoid warning during installation (`#83 <https://github.com/ros2/ros2cli/issues/83>`_)
* use rmw agnostic daemon URL (`#80 <https://github.com/ros2/ros2cli/issues/80>`_)
* print full help when no command is passed (`#81 <https://github.com/ros2/ros2cli/issues/81>`_)
* fix import order (`#79 <https://github.com/ros2/ros2cli/issues/79>`_)
* Contributors: Dirk Thomas, Mikael Arguedas

0.4.0 (2017-12-08)
------------------
* Merge pull request `#71 <https://github.com/ros2/ros2cli/issues/71>`_ from ros2/wait_until_daemon_has_started
* 'daemon start' waits until it has been started before returning
* remove test_suite, add pytest as test_requires
* 0.0.3
* Merge pull request `#49 <https://github.com/ros2/ros2cli/issues/49>`_ from ros2/msg_stopping_daemon_diff_rmw
* add error message when trying to stop a daemon using a different rmw implementation
* Merge pull request `#46 <https://github.com/ros2/ros2cli/issues/46>`_ from ros2/flake8_plugins
* update style to satisfy new flake8 plugins
* implicitly inherit from object (`#45 <https://github.com/ros2/ros2cli/issues/45>`_)
* 0.0.2
* fix daemon verb
* Merge pull request `#38 <https://github.com/ros2/ros2cli/issues/38>`_ from ros2/add_daemon_command
* Merge pull request `#33 <https://github.com/ros2/ros2cli/issues/33>`_ from ros2/improve_windows_daemon
* add daemon command with verbs status, start, stop
* use different cwd for daemon to prevent holding folder handle
* detach daemon on Windows
* add exec_depend on python3-pkg-resources (`#30 <https://github.com/ros2/ros2cli/issues/30>`_)
* Merge pull request `#29 <https://github.com/ros2/ros2cli/issues/29>`_ from ros2/hide_help_from_completion
* hide help options from completion
* Merge pull request `#26 <https://github.com/ros2/ros2cli/issues/26>`_ from ros2/support_argcomplete_py3
* support python3-argcomplete
* Merge pull request `#15 <https://github.com/ros2/ros2cli/issues/15>`_ from ros2/various_fixes
* various fixes and improvements
* Merge pull request `#11 <https://github.com/ros2/ros2cli/issues/11>`_ from ros2/daemon_rmw_impl
* update daemon to only handle requests from localhost
* update daemon to only handle requests from matching rmw impl.
* Merge pull request `#7 <https://github.com/ros2/ros2cli/issues/7>`_ from ros2/zsh_argcomplete
* add argcomplete script for zsh
* Merge pull request `#5 <https://github.com/ros2/ros2cli/issues/5>`_ from ros2/pep257
* add pep257 tests
* Merge pull request `#1 <https://github.com/ros2/ros2cli/issues/1>`_ from ros2/initial_features
* append pid / domain id to node names
* add suffix to node name in daemon (`#2 <https://github.com/ros2/ros2cli/issues/2>`_)
* add linter tests
* add rclpy node interface and xml-rpc based daemon
* add argcomplete-based completion
* add hidden extension commands
* add ros2cli plugin system, interface for commands and verbs, and cli
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
