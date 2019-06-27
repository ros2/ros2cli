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

from ros2interface.api import get_all_action_types
from ros2interface.api import get_all_interface_packages
from ros2interface.api import get_all_message_types
from ros2interface.api import get_all_service_types
from ros2interface.verb import VerbExtension


def print_packages(packs):
    for name in sorted(packs):
        print((name))


class PackagesVerb(VerbExtension):
    """Output a list of packages which contain msgs, srvs, and actions."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('-m', '--only-msgs', action='count',
                            help='Only list packages that generate messages')

        parser.add_argument('-s', '--only-srvs', action='count',
                            help='Only list packages that generate services')

        parser.add_argument('-a', '--only-actions', action='count',
                            help='Only list packages that generate actions')

    def main(self, *, args):
        packages = set()
        if args.only_msgs:
            print_packages(get_all_message_types().keys())
        elif args.only_srvs:
            print_packages(get_all_service_types().keys())
        elif args.only_actions:
            print_packages(get_all_action_types().keys())
        else:
            all_interface_packages = get_all_interface_packages()
            for package in all_interface_packages:
                packages.add(package)
            print_packages(packages)
