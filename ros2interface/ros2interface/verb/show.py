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

from ros2action.api import get_action_path
from ros2interface.api import type_completer
from ros2interface.verb import VerbExtension
from ros2msg.api import get_message_path
from ros2srv.api import get_service_path


class ShowVerb(VerbExtension):
    """Output the interface definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
                'type',
                help='Show an interface definition (msgs, srvs, actions)')
        arg.completer = type_completer

    def main(self, *, args):
        try:
            parts = args.type.split('/')
            if len(parts) < 3:
                raise ValueError()
            if not parts[0] or not parts[1] or not parts[2]:
                raise ValueError()
            # now have to check if msg or srv
        except ValueError:
            raise RuntimeError('The passed interface type is invalid')

        try:
            if parts[1] == 'msg':
                path = get_message_path(parts[0], parts[2])
            elif parts[1] == 'srv':
                path = get_service_path(parts[0], parts[2])
            elif parts[1] == 'action':
                path = get_action_path(parts[0], parts[2])
            else:
                raise ValueError()
        except ValueError:
            raise RuntimeError('The passed interface type is invalid')
        except LookupError as e:
            return str(e)
        with open(path, 'r') as h:
            print(h.read(), end='')
