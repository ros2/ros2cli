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

import sys

from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import spawn_daemon
from ros2cli.verb.daemon import VerbExtension


class StartVerb(VerbExtension):
    """Start the daemon if it isn't running."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--debug', '-d', action='store_true',
            help='Print debug messages')

    def main(self, *, args):
        running = is_daemon_running(args)
        if running:
            print('The daemon is already running')
            return

        spawned = spawn_daemon(args, wait_until_spawned=10.0)
        if spawned:
            print('The daemon has been started')
        else:
            print(
                'Failed to confirm that the daemon started successfully',
                file=sys.stderr)
            return 1
