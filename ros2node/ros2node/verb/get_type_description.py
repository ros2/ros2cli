# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import rclpy
from ros2cli.node import NODE_NAME_PREFIX
from ros2node.api import NodeNameCompleter
from ros2node.verb import VerbExtension
from rosidl_runtime_py import message_to_yaml
from type_description_interfaces.srv import GetTypeDescription


class GetTypeDescriptionVerb(VerbExtension):
    """Call a node's ~/get_type_description service and print the response."""

    def add_arguments(self, parser, cli_name):
        argument = parser.add_argument(
            'node_name',
            help='Node name to request information')
        argument.completer = NodeNameCompleter()
        argument = parser.add_argument(
            'type_name',
            help='ROS interface type name, in PACKAGE/NAMESPACE/TYPENAME format')
        argument = parser.add_argument(
            'type_hash',
            help='REP-2011 RIHS hash string')
        argument = parser.add_argument(
            '--include_type_sources', default=False, action='store_true',
            help='Whether to return the original idl/msg/etc. source file(s) in the response')

    def main(self, *, args):
        rclpy.init()
        node = rclpy.create_node(NODE_NAME_PREFIX + '_requester_%s_%s' %
                                 ('type_description_interfaces', 'GetTypeDescription'))

        service_name = args.node_name + '/get_type_description'
        cli = node.create_client(GetTypeDescription, service_name)

        if not cli.service_is_ready():
            print(f'waiting for service {service_name} to become available...')
            cli.wait_for_service()

        request = GetTypeDescription.Request(type_name=args.type_name,
                                             type_hash=args.type_hash,
                                             include_type_sources=args.include_type_sources)
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print(message_to_yaml(future.result()))
        else:
            raise RuntimeError('Exception while calling service: %r' % future.exception())

        node.destroy_node()
        rclpy.shutdown()
