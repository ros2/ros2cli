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

from typing import Optional

from ros2cli.node.daemon import add_arguments as add_daemon_node_arguments
from ros2cli.node.daemon import DaemonNode
from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import spawn_daemon
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode


class NodeStrategy:

    def __init__(self, args, *, node_name: Optional[str] = None):
        use_daemon = not getattr(args, 'no_daemon', False)
        if use_daemon and is_daemon_running(args):
            self._daemon_node = DaemonNode(args)
            self._direct_node = None
            if not self._daemon_node.connected:
                self._direct_node = DirectNode(args, node_name=node_name)
                self._daemon_node = None
        else:
            if use_daemon:
                spawn_daemon(args)
            self._direct_node = DirectNode(args, node_name=node_name)
            self._daemon_node = None
        self._args = args
        self._node_name = node_name
        self._in_scope = False

    @property
    def daemon_node(self):
        return self._daemon_node

    @property
    def direct_node(self):
        if self._direct_node is None:
            self._direct_node = DirectNode(self._args, node_name=self._node_name)
            if self._in_scope:
                self._direct_node.__enter__()
        return self._direct_node

    def __enter__(self):
        if self._daemon_node:
            self._daemon_node.__enter__()
        if self._direct_node:
            self._direct_node.__enter__()
        self._in_scope = True
        return self

    def __getattr__(self, name):
        if self.daemon_node and name in self.daemon_node.methods:
            return getattr(self.daemon_node, name)
        return getattr(self.direct_node, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self._in_scope = False
        if self._direct_node:
            self._direct_node.__exit__(exc_type, exc_value, traceback)
        if self._daemon_node:
            self._daemon_node.__exit__(exc_type, exc_value, traceback)


def add_arguments(parser):
    add_daemon_node_arguments(parser)
    add_direct_node_arguments(parser)
    parser.add_argument(
        '--no-daemon', action='store_true',
        help='Do not spawn nor use an already running daemon'
    )
