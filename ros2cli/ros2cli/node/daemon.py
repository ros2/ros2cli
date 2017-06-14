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

from ros2cli.daemon import get_daemon_port

import xmlrpc.client


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


def spawn_daemon(args):
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
    subprocess.Popen(
        ['_ros2_daemon', '--ros-domain-id', str(ros_domain_id)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class DaemonNode(object):

    def __init__(self, args):
        self._proxy = xmlrpc.client.ServerProxy(
            'http://localhost:%d/' % get_daemon_port())
        self._methods = []

    def __enter__(self):
        self._proxy.__enter__()

        methods = self._proxy.system.listMethods()
        self._methods = [m for m in methods if not m.startswith('system.')]

        return self

    def __getattr__(self, name):
        return getattr(self._proxy, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self._proxy.__exit__(exc_type, exc_value, traceback)


def add_arguments(parser):
    pass
