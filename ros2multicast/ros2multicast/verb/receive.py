# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from ros2multicast.api import receive
from ros2multicast.verb import add_group_argument
from ros2multicast.verb import add_port_argument
from ros2multicast.verb import VerbExtension


class ReceiveVerb(VerbExtension):
    """Receive a single UDP multicast packet."""

    def add_arguments(self, parser, cli_name):
        add_group_argument(parser)
        add_port_argument(parser)

    def main(self, *, args):
        print('Waiting for UDP multicast datagram...')
        try:
            data, (host, port) = receive(group=args.group, port=args.port)
        except KeyboardInterrupt:
            pass
        else:
            data = data.decode('utf-8')
            print(f"Received from {host}:{port}: '{data}'")
