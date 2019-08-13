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

from ros2doctor.api import check_platform
from ros2doctor.api import print_platform_info
from ros2doctor.api import print_ros2_reqs
from ros2doctor.verb import VerbExtension


class PlatformVerb(VerbExtension):
    """Cross check platform and ROS 2 system requirements."""

    def add_arguments(self, parser, cli_name):
        """Determine the args and subcommand taken."""
        parser.add_argument(
            '-r', '--report', action='store_true',
            help='Display platform information and ROS 2 requirements'
        )

    def main(self, *, args):
        """Conduct platform checks on machine against ROS2 distro reqs."""
        if args.report:
            print_platform_info()
            print_ros2_reqs()
            print('To check platform requirements use `ros2 doctor platform`.')
        else:
            check_platform()
            print('Platform check completed!')
            print('\n')
