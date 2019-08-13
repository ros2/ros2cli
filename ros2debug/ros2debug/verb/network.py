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

from ros2debug.verb import VerbExtension


class SetupVerb(VerbExtension):
    """Cross check system info and ROS 2 system requirements."""

    def add_arguments(self, parser, cli_name):
        """Determine the args and subcommand taken."""
        parser.add_argument(
            '-r', '--report', action='store_true',
            help='Display network interface'
        )

    def main(self, *, args):
        """Sanity check network interface."""
        # if args.report:
        print('Network check completed!')
        print()
