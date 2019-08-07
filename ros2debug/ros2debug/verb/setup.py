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
 
import platform 
import sys
import os 

from ros2cli.node.strategy import add_arguments
from ros2debug.verb import VerbExtension

class SetupVerb(VerbExtension):
    """Cross check system info and ROS 2 system requirements."""

    def add_arguments(self, parser, cli_name):
        # determine the args and subcommand taken 
        add_arguments(parser)
        parser.add_arguments(
            '-r', '--report', help='Generate a report of current system information \
                and the corresponding ROS 2 system requirements.'
        )

    def main(self, *, args):
        # conduct a series of setup checks on sys and version against ROS2 reqs 
        if args.report: 
            print(platform.platform())
        else: 
            print('No report requested')