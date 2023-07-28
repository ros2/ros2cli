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

import os

import rclpy
from ros2cli.node import NODE_NAME_PREFIX
from ros2cli.node.strategy import add_arguments
from ros2node.api import (
    parse_node_name,
    INFO_NONUNIQUE_WARNING_TEMPLATE,
    NodeNameCompleter,
)
from ros2node.verb import VerbExtension
from type_description_interfaces.msg import FieldType
from type_description_interfaces.srv import GetTypeDescription


def print_field_type(ft):
    type_name = None
    type_str = ''
    for x, y in FieldType.__dict__.items():
        if y == ft.type_id:
            type_name = x
            break
    if not type_name:
        raise RuntimeError(f'Could not match FieldType type_id {ft.type_id} to a name')

    type_str = type_name
    if 'NESTED_TYPE' in type_name:
        type_str += f' ({ft.nested_type_name})'
    if ft.string_capacity:
        type_str += f' - string_capacity {ft.string_capacity}'
    if ft.capacity:
        type_str += f' - capacity {ft.capacity}'
    return type_str


def print_individual_type_description(itd, indent=0):
    def p(line):
        print(' ' * indent + line)

    print()
    p(f'{itd.type_name}')
    p('Fields:')
    indent += 2
    for field in itd.fields:
        field_string = f'{field.name}: {print_field_type(field.type)}'
        if field.default_value:
            field_string += f' = {field.default_value}'
        p(field_string)
    indent -= 2


def print_type_description(td):
    print_individual_type_description(td.type_description)
    print()
    print('Referenced Type Descriptions:')
    for rtd in td.referenced_type_descriptions:
        print_individual_type_description(rtd, indent=2)


def discover_hash_for_type(node, remote_node_name, type_name):
    # TODO(emersonknapp) get hashes from names_and_types when that is implemented
    ns = type_name.split('/')[1]
    if ns != 'msg':
        raise RuntimeError(
            f'Currently cannot auto-discover hashes for "{ns}", only "msg". '
            'Please provide type_hash value to command.')
    remote_node = parse_node_name(remote_node_name)
    nt = node.get_publisher_names_and_types_by_node(remote_node.name, remote_node.namespace)
    nt += node.get_subscriber_names_and_types_by_node(remote_node.name, remote_node.namespace)
    discover_topic = None
    for topic, types in nt:
        if type_name in types:
            discover_topic = topic
    if not discover_topic:
        raise RuntimeError(
            f'Could not find type "{type_name}" advertised as a topic type on node '
            f'"{remote_node_name}"')

    endpoint_infos = node.get_publishers_info_by_topic(discover_topic)
    endpoint_infos += node.get_subscriptions_info_by_topic(discover_topic)
    for endpoint_info in endpoint_infos:
        if (
            endpoint_info.node_name == remote_node.name and
            endpoint_info.node_namespace == remote_node.namespace and
            endpoint_info.topic_type == type_name
        ):
            return str(endpoint_info.topic_type_hash)
    raise RuntimeError(
        f'Cound not find endpoint type hash for topic "{discover_topic}"')


class GetTypeDescriptionVerb(VerbExtension):
    """Output information about a node."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        argument = parser.add_argument(
            'node_name',
            help='Node name to request information from')
        argument.completer = NodeNameCompleter()
        parser.add_argument(
            'type_name',
            help='Interface type to get description of')
        parser.add_argument(
            'type_hash',
            default=None,
            nargs='?',
            help='Hash string of the type. If not provided, will try to determine automatically.')
        parser.add_argument(
            '--include-sources', action='store_true',
            help='Fetch and display the raw source text as well')

    def main(self, *, args):
        service_name = f'{args.node_name}/get_type_description'

        rclpy.init()
        node = rclpy.create_node(f'{NODE_NAME_PREFIX}_td_requester_{os.getpid()}')
        cli = node.create_client(GetTypeDescription, '/talker/get_type_description')
        if not cli.service_is_ready():
            print('Waiting for service to become available...')
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f'Service {service_name} not found')

        if args.type_hash:
            type_hash = args.type_hash
        else:
            type_hash = discover_hash_for_type(node, args.node_name, args.type_name)
            print(f'Requesting type hash {type_hash}')

        request = GetTypeDescription.Request()
        request.type_name = args.type_name
        request.type_hash = type_hash
        request.include_type_sources = args.include_sources
        print('Sending request...')
        future = cli.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if not response.successful:
            raise RuntimeError(
                f'Failed to get type description: {response.failure_reason}')

        print('Response successful:')
        print_type_description(response.type_description)
