# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from ros2debug.api import print_ros2_reqs
from ros2debug.api import print_sys_info
from ros2debug.api import setup_checks
from ros2debug.verb import VerbExtension


class SetupVerb(VerbExtension):
    """Cross check system info and ROS 2 system requirements."""

    def add_arguments(self, parser, cli_name):
        """Determine the args and subcommand taken."""
        parser.add_argument(
            '-r', '--report', action='store_true',
            help='Display system setup information and ROS 2 requirements'
        )

    def main(self, *, args):
        """Donduct setup checks on system against ROS2 distro reqs."""
        if args.report:
            print_sys_info()
            print_ros2_reqs()
            print('To check setup requirements use `ros2 debug setup`.')
        else:
            setup_checks()
            print('Setup check completed!')
            print('\n')
