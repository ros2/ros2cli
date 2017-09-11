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

from ros2cli.node.daemon import DaemonNode
from ros2cli.node.daemon import is_daemon_running
from ros2cli.verb.daemon import VerbExtension


class StopVerb(VerbExtension):
    """Stop the daemon if it is running."""

    def main(self, *, args):
        running = is_daemon_running(args)
        if not running:
            print('The daemon is not running')
            return

        with DaemonNode(args) as daemon:
            try:
                shutdown = daemon.system.shutdown
            except AttributeError:
                return 'Failed to shutdown daemon, ' \
                    'it might be using a different rmw implementation'
            shutdown()
        print('The daemon has been stopped')
