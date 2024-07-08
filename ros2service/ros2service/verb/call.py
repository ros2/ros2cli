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
import time

import rclpy
from ros2cli.helpers import collect_stdin
from ros2cli.node import NODE_NAME_PREFIX
from ros2service.api import ServiceNameCompleter
from ros2service.api import ServicePrototypeCompleter
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension
from rosidl_runtime_py import set_message_fields
import yaml


class CallVerb(VerbExtension):
    """Call a service."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to call to (e.g. '/add_two_ints')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        arg = parser.add_argument(
            'service_type',
            help="Type of the ROS service (e.g. 'std_srvs/srv/Empty')")
        arg.completer = ServiceTypeCompleter(
            service_name_key='service_name')
        group = parser.add_mutually_exclusive_group()
        arg = group.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the service request with in YAML format ' +
                 "(e.g. '{a: 1, b: 2}'), " +
                 'otherwise the service request will be published with default values')
        arg.completer = ServicePrototypeCompleter(
            service_type_key='service_type')
        group.add_argument(
            '--stdin', action='store_true',
            help='Read values from standard input')
        parser.add_argument(
            '-r', '--rate', metavar='N', type=float,
            help='Repeat the call at a specific rate in Hz')

    def main(self, *, args):
        if args.rate is not None and args.rate <= 0:
            raise RuntimeError('rate must be greater than zero')
        period = 1. / args.rate if args.rate else None

        if args.stdin:
            values = collect_stdin()
        else:
            values = args.values

        return requester(
            args.service_type, args.service_name, values, period)


def requester(service_type, service_name, values, period):
    # TODO(wjwwood) this logic should come from a rosidl related package
    try:
        parts = service_type.split('/')
        if len(parts) == 2:
            parts = [parts[0], 'srv', parts[1]]
        package_name = parts[0]
        module = importlib.import_module('.'.join(parts[:-1]))
        srv_name = parts[-1]
        srv_module = getattr(module, srv_name)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError('The passed service type is invalid')
    try:
        srv_module.Request
        srv_module.Response
    except AttributeError:
        raise RuntimeError('The passed type is not a service')

    values_dictionary = yaml.safe_load(values)

    with rclpy.init():
        node = rclpy.create_node(NODE_NAME_PREFIX + '_requester_%s_%s' % (package_name, srv_name))

        cli = node.create_client(srv_module, service_name)

        request = srv_module.Request()

        try:
            set_message_fields(request, values_dictionary)
        except Exception as e:
            return 'Failed to populate field: {0}'.format(e)

        if not cli.service_is_ready():
            print('waiting for service to become available...')
            cli.wait_for_service()

        while True:
            print('requester: making request: %r\n' % request)
            last_call = time.time()
            future = cli.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None:
                print('response:\n%r\n' % future.result())
            else:
                raise RuntimeError('Exception while calling service: %r' % future.exception())
            if period is None or not rclpy.ok():
                break
            time_until_next_period = (last_call + period) - time.time()
            if time_until_next_period > 0:
                time.sleep(time_until_next_period)
