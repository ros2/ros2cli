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

from ros2interface.verb import VerbExtension
from ros2msg.api import get_all_message_types
from ros2srv.api import get_all_service_types


def printMessages():
    print('Messages: ')
    message_types = get_all_message_types()
    for package_name in sorted(message_types.keys()):
        for message_name in sorted(message_types[package_name]):
            print('{package_name}/msg/{message_name}'.format_map(locals()))


def printServices():
    print('Services: ')
    service_types = get_all_service_types()
    for package_name in sorted(service_types.keys()):
        for service_name in sorted(service_types[package_name]):
            print('{package_name}/srv/{service_name}'.format_map(locals()))


class ListVerb(VerbExtension):
    """List all .msg, .srv, and .action types available."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('-m', '--only-msgs', action='count',
                            help='Print out only the message types')

        parser.add_argument('-s', '--only-srvs', action='count',
                            help='Print out only the srvs types')

    def main(self, *, args):
        if(args.only_msgs == 1):
            printMessages()
            return
        if(args.only_srvs == 1):
            printServices()
            return
        printMessages()
        printServices()
