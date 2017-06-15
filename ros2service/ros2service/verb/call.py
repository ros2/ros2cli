# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import importlib
import json

import rclpy
from ros2service.api import ServiceNameCompleter
from ros2service.verb import VerbExtension


class CallVerb(VerbExtension):
    """Call a service."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to publish to (e.g. '/chatter')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        parser.add_argument(
            'service_type',
            help="Type of the ROS service (e.g. 'std_srvs/Empty')")
        parser.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the service request with in JSON format ' +
                 '(e.g. {"a": 1, "b": 2}), ' +
                 'otherwise the service request will be published with default values')

    def main(self, *, args):
        return main(args)


def main(args):
    requester(args.service_type, args.service_name, args.values)


def requester(service_type, service_name, values):
    # TODO(wjwwood) this logic should come from a rosidl related package
    try:
        package_name, service_name = service_type.split('/', 2)
    except ValueError:
        raise RuntimeError('The passed service type is invalid')
    module = importlib.import_module(package_name + '.srv')
    srv_module = getattr(module, service_name)
    values_dictionary = json.loads(values)

    rclpy.init()

    node = rclpy.create_node('requester_%s_%s' % (package_name, service_name))

    cli = node.create_client(srv_module, service_name)

    request = srv_module.Request()
    for field_name, field_value in values_dictionary.items():
        field_type = type(getattr(request, field_name))
        setattr(request, field_name, field_type(field_value))

    print('requester: making request: %r\n' % request)
    cli.call(request)
    cli.wait_for_future()
    print('response:\n%r\n' % cli.response)

    node.shutdown()
    rclpy.shutdown()
