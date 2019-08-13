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

from ros2debug.api import print_network_interface
from ros2debug.verb import VerbExtension


class NetworkVerb(VerbExtension):
    """Cross check system info and ROS 2 system requirements."""

    def add_arguments(self, parser, cli_name):
        """Determine the args and subcommand taken."""
        parser.add_argument(
            '-r', '--report', action='store_true',
            help='Display network interface'
        )

    def main(self, *, args):
        """Sanity check network interface."""
        if args.report:
            print_network_interface()
            print('To check network interface, use `ros2 debug network`.')
        else:
            print('Network check completed!')
            print('\n')


def print_network_interface_helper(addrs, layer_type):
    """Format print network interface."""
    layer_iface = addrs.get(layer_type)
    if not layer_iface:
        return
    print('Address famility number: ', layer_type)
    for l in layer_iface:
        for k, v in l.items():
            print('%s: %s' % (k, v))


def print_network_interface():
    """Print out network interface."""
    ids = netifaces.interfaces()
    for i in ids:
        print('Network Interface Identifier: ', i)
        addrs = netifaces.ifaddresses(i)
        print('*************Link Layer**************')
        print_network_interface_helper(addrs, netifaces.AF_LINK)
        print('******Normal Internet Addresses******')
        print_network_interface_helper(addrs, netifaces.AF_INET)
        print('****************IPv6*****************')
        print_network_interface_helper(addrs, netifaces.AF_INET6)
        print('\n')
