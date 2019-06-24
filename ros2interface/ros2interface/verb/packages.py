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

from ros2msg.api import get_all_message_types
from ros2srv.api import get_all_service_types
from ros2interface.verb import VerbExtension

def printPackages(packs):
    for name in sorted(packs):
        print((name))

class PackagesVerb(VerbExtension):
    """Output a list of packages which contain msgs, srvs, & actions."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('-m', '--only-msgs', action='count',
               help='Only list packages that generate messages')

        parser.add_argument('-s', '--only-srvs', action='count',
               help='Only list packages that generate services')

    def main(self, *, args):
        packs = set()
        message_types = get_all_message_types()
        if(args.only_msgs):
            for package_name in sorted(message_types.keys()):
                packs.add(package_name)
            printPackages(packs)
            return

        service_types = get_all_service_types()
        if(args.only_srvs):
            for package_name in sorted(service_types.keys()):
                packs.add(package_name)
            printPackages(packs)
            return 

        for package_name in sorted(message_types.keys()):
            packs.add(package_name)
        for package_name in sorted(service_types.keys()):
            packs.add(package_name)
        printPackages(packs)
