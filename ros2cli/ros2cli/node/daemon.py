# Copyright 2017 Open Source Robotics Foundation, Inc.
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
import os
import platform
import socket
import subprocess
import sys
import time
from xmlrpc.client import ProtocolError
from xmlrpc.client import ServerProxy

import rclpy
from ros2cli.daemon import get_daemon_port
from ros2cli.node.direct import DirectNode


def is_daemon_running(args):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if platform.system() != 'Windows':
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    addr = ('localhost', get_daemon_port())
    try:
        s.bind(addr)
    except socket.error as e:
        if e.errno == errno.EADDRINUSE:
            return True
    else:
        s.close()
    return False


def spawn_daemon(args, wait_until_spawned=None, debug=False):
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
    timeout_duration = int(args.timeout)
    kwargs = {}
    if platform.system() != 'Windows':
        cmd = ['_ros2_daemon']
    else:
        # allow closing the shell the daemon was spawned from
        # while the daemon is still running
        cmd = [
            sys.executable,
            os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'daemon/__init__.py')]
        # Process Creation Flag documented in the MSDN
        DETACHED_PROCESS = 0x00000008  # noqa: N806
        kwargs.update(creationflags=DETACHED_PROCESS)
        # avoid showing cmd windows for subprocess
        si = subprocess.STARTUPINFO()
        si.dwFlags = subprocess.STARTF_USESHOWWINDOW
        si.wShowWindow = subprocess.SW_HIDE
        kwargs['startupinfo'] = si
        # don't keep handle of current working directory in daemon process
        kwargs.update(cwd=os.environ.get('SYSTEMROOT', None))

    rmw_implementation_identifier = rclpy.get_rmw_implementation_identifier()
    if rmw_implementation_identifier is None:
        raise RuntimeError(
            'Unable to get rmw_implementation_identifier, '
            'try specifying the implementation to use via the '
            "'RMW_IMPLEMENTATION' environment variable")
    cmd.extend([
        # the arguments are only passed for visibility in e.g. the process list
        '--rmw-implementation', rmw_implementation_identifier,
        '--ros-domain-id', str(ros_domain_id),
        '--timeout', str(timeout_duration)])
    if not debug:
        kwargs['stdout'] = subprocess.DEVNULL
        kwargs['stderr'] = subprocess.DEVNULL
    subprocess.Popen(cmd, **kwargs)

    if wait_until_spawned is None:
        return True

    if wait_until_spawned > 0.0:
        timeout = time.time() + wait_until_spawned
    else:
        timeout = None
    while True:
        if is_daemon_running(args):
            return True
        time.sleep(0.1)
        if timeout is not None and time.time() >= timeout:
            return None


class DaemonNode:

    def __init__(self, args):
        self._args = args
        self._proxy = ServerProxy(
            'http://localhost:%d/ros2cli/' % get_daemon_port(),
            allow_none=True)
        self._methods = []

    def __enter__(self):
        self._proxy.__enter__()

        try:
            methods = self._proxy.system.listMethods()
        except ProtocolError as e:
            if e.errcode != 404:
                raise
            # remote daemon returned 404, likely using different rmw impl.
            self._proxy.__exit__()
            # fall back to use direct node
            self._proxy = DirectNode(self._args)
            self._proxy.__enter__()
        else:
            self._methods = [m for m in methods if not m.startswith('system.')]

        return self

    def __getattr__(self, name):
        return getattr(self._proxy, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self._proxy.__exit__(exc_type, exc_value, traceback)


def add_arguments(parser):
    pass
