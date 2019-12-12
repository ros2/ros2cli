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

from ros2cli.node.daemon import add_arguments as add_daemon_node_arguments
from ros2cli.node.daemon import DaemonNode
from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import spawn_daemon
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode


class NodeStrategy:

    def __init__(self, args):
        use_daemon = not getattr(args, 'no_daemon', False)
        if use_daemon and is_daemon_running(args):
            self.node = DaemonNode(args)
        else:
            if use_daemon:
                spawn_daemon(args)
            self.node = DirectNode(args)

    def __enter__(self):
        self.node.__enter__()
        return self

    def __getattr__(self, name):
        return getattr(self.node, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self.node.__exit__(exc_type, exc_value, traceback)


def add_arguments(parser):
    add_daemon_node_arguments(parser)
    add_direct_node_arguments(parser)
    parser.add_argument(
        '--no-daemon', action='store_true',
        help='Do not spawn nor use an already running daemon'
    )
