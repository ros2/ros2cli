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

import warnings

from ros2srv.api import get_service_path
from ros2srv.api import service_type_completer
from ros2srv.verb import VerbExtension


class ShowVerb(VerbExtension):
    """Output the service definition."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_type',
            help="Type of the ROS service (e.g. 'std_srvs/srv/Trigger')")
        arg.completer = service_type_completer

    def main(self, *, args):
        # TODO(dirk-thomas) this logic should come from a rosidl related
        # package
        try:
            parts = args.service_type.split('/')
            if len(parts) == 1:
                raise ValueError()
            if len(parts) == 2:
                parts = [parts[0], 'srv', parts[1]]
            package_name = parts[0]
            service_name = parts[-1]
            if not package_name or not service_name:
                raise ValueError()
        except ValueError:
            raise RuntimeError('The passed service type is invalid')

        fqn = '/'.join(parts)
        warnings.warn(
            "'ros2 srv' is deprecated and will be removed in a future ROS release. "
            f"Instead use: 'ros2 interface show {fqn}.srv'"
        )

        try:
            path = get_service_path(package_name, service_name)
        except LookupError as e:
            return str(e)
        with open(path, 'r') as h:
            print(h.read(), end='')
