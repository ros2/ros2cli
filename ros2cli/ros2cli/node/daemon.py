# Copyright 2017-2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import errno
import functools
import socket

import rclpy

import ros2cli.daemon as daemon
from ros2cli.daemon.daemonize import daemonize

from ros2cli.helpers import get_ros_domain_id
from ros2cli.helpers import wait_for

from ros2cli.xmlrpc.client import ServerProxy


class DaemonNode:

    def __init__(self, args):
        self._args = args
        self._proxy = ServerProxy(
            daemon.get_xmlrpc_server_url(),
            allow_none=True)
        self._methods = []

    @property
    def connected(self):
        try:
            self._methods = [
                method
                for method in self._proxy.system.listMethods()
                if not method.startswith('system.')
            ]
        except ConnectionRefusedError:
            return False
        return True

    @property
    def methods(self):
        return self._methods

    def __enter__(self):
        self._proxy.__enter__()
        return self

    def __getattr__(self, name):
        return getattr(self._proxy, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self._proxy.__exit__(exc_type, exc_value, traceback)


def is_daemon_running(args):
    """
    Check if the daemon node is running.

    :param args: `DaemonNode` arguments namespace.
    """
    with DaemonNode(args) as node:
        return node.connected


def shutdown_daemon(args, timeout=None):
    """
    Shut down daemon node if it's running.

    :param args: `DaemonNode` arguments namespace.
    :param timeout: optional duration, in seconds, to wait
      until the daemon node is fully shut down. Non-positive
      durations will result in an indefinite wait.
    :return: `True` if the the daemon was shut down,
      `False` if it was already shut down.
    :raises: if it fails to shutdown the daemon.
    """
    with DaemonNode(args) as node:
        if not node.connected:
            return False
        node.system.shutdown()
        if timeout is not None:
            predicate = (lambda: not node.connected)
            if not wait_for(predicate, timeout):
                raise RuntimeError(
                    'Timed out waiting for '
                    'daemon to shutdown'
                )
        return True


def spawn_daemon(args, timeout=None, debug=False):
    """
    Spawn daemon node if it's not running.

    To avoid TOCTOU races, this function instantiates
    the XMLRPC server (binding the socket in the process)
    and transfers it to the daemon process through pipes
    (sending the inheritable socket with it). In a sense,
    the socket functionally behaves as a mutex.

    :param args: `DaemonNode` arguments namespace.
    :param timeout: optional duration, in seconds, to wait
      until the daemon node is ready. Non-positive
      durations will result in an indefinite wait.
    :param debug: if `True`, the daemon process will output
      to the current `stdout` and `stderr` streams.
    :return: `True` if the the daemon was spawned,
      `False` if it was already running.
    :raises: if it fails to spawn the daemon.
    """
    # Acquire socket by instantiating XMLRPC server.
    try:
        server = daemon.make_xmlrpc_server()
        server.socket.set_inheritable(True)
    except socket.error as e:
        if e.errno == errno.EADDRINUSE:
            # Failed to acquire socket
            # Daemon already running
            return False
        raise

    # Transfer XMLRPC server to daemon (and the socket with it).
    try:
        tags = {
            'name': 'ros2-daemon', 'ros_domain_id': get_ros_domain_id(),
            'rmw_implementation': rclpy.get_rmw_implementation_identifier()}

        daemonize(
            functools.partial(daemon.serve, server),
            tags=tags, timeout=timeout, debug=debug)
    finally:
        server.server_close()

    return True


def add_arguments(parser):
    pass
