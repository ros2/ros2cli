^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2log
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.42.0 (2026-09-01)
-------------------
* ros2log new package introduction (`#1217 <https://github.com/ros2/ros2cli/issues/1217>`_)
  * 1st draft bring up for "ros2 log watch" sub-command.
  clean up the package and implementation.
  fix ros2log test_watch.py.
  support QoS configuration argument for ros2 log watch.
  Call content filtering API for logger name and level if available.
  add test_cli.py to test "ros2 log watch".
  remove ros2log/README.md.
  :construction: add list subcommand
  bug: fix test
  :recycle: delete non-necessary fixture node for test
  :bug: fix to print one by one
  support "ros2 log levels".
  :zap: add get and set subcommand
  :bug: align logger's initial state during test
  FIX: add sleep to reduce cpu consumption
  FIX: rewrite waiting acync process
  :FIX: delete --include-hidden-nodes option
  FIX: import from api
  FIX: add ValueError when empty string is inputted in get_absolute_node_name
  * fix copyright year
  * fix to destroy clients
  * fix to remove dependencies
  * add missing dependencies
  * fix type annotation
  * add type hints to node
  * Update ros2log/ros2log/verb/watch.py
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  * Update ros2log/ros2log/verb/watch.py
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  * Update ros2log/ros2log/verb/watch.py
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  * Update ros2log/ros2log/api/__init_\_.py
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  * add checking negative timeout_sec
  * LogWatcher constructor should raise the exception instead of sys.exit().
  * address copilot review comments.
  * address review comments.
  ---------
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
  Co-authored-by: Decwest <fumiyaonishi1016@gmail.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Tomoya Fujita
