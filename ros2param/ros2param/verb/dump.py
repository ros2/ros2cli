# Copyright 2019 Canonical Ltd.
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
import sys
import yaml

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import ListParameters
import rclpy
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2node.api import parse_node_name
from ros2param.api import call_get_parameters
from ros2param.verb import VerbExtension


class DumpVerb(VerbExtension):
    """Dump the parameters of a node to a yaml file."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        parser.add_argument(
            '--file-path',
            default='.',
            help='The absolute path were to save the generated file')

    @staticmethod
    def get_parameter_value(node, node_name, param):
        response = call_get_parameters(
            node=node, node_name=node_name,
            parameter_names=[param])

        # requested parameter not set
        if not response.values:
            return '# Parameter not set'

        # extract type specific value
        pvalue = response.values[0]
        if pvalue.type == ParameterType.PARAMETER_BOOL:
            value = pvalue.bool_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER:
            value = pvalue.integer_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
            value = pvalue.double_value
        elif pvalue.type == ParameterType.PARAMETER_STRING:
            value = pvalue.string_value
        elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = pvalue.byte_array_value
        elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = pvalue.bool_array_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = pvalue.integer_array_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = pvalue.double_array_value
        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = pvalue.string_array_value
        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            value = None

        return value

    def insert_dict(self, dict, key, value):
        split = key.split(PARAMETER_SEPARATOR_STRING, 1)
        if len(split) > 1:
            if not split[0] in dict:
                dict[split[0]] = {}
            self.insert_dict(dict[split[0]], split[1], value)
        else:
            dict[key] = value

    def main(self, *, args):  # noqa: D102

        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        absolute_node_name = get_absolute_node_name(args.node_name)
        node_name = parse_node_name(absolute_node_name)
        if absolute_node_name:
            if absolute_node_name not in [n.full_name for n in node_names]:
                return 'Node not found'

        if not os.path.isdir(args.file_path):
            return 'Invalid output directory'

        with DirectNode(args) as node:
            # create client
            service_name = '{absolute_node_name}/list_parameters'.format_map(locals())
            client = node.create_client(ListParameters, service_name)

            client.wait_for_service()

            if not client.service_is_ready():
                return 'Something went wrong'

            request = ListParameters.Request()
            future = client.call_async(request)

            # wait for response
            rclpy.spin_until_future_complete(node, future)

            yaml_output = {node_name.name: {'ros__parameters': {}}}

            # retrieve values
            if future.result() is not None:
                response = future.result()
                for param_name in sorted(response.result.names):
                    pval = self.get_parameter_value(node, absolute_node_name, param_name)
                    self.insert_dict(
                        yaml_output[node_name.name]['ros__parameters'], param_name, pval)
            else:
                e = future.exception()
                print(
                    'Exception while calling service of node '
                    "'{node_name.full_name}': {e}".format_map(locals()),
                    file=sys.stderr)

            print('Saving to: ', os.path.join(args.file_path, node_name.name + ".yaml"))
            with open(os.path.join(args.file_path, node_name.name + ".yaml"), "w") as yaml_file:
                yaml.dump(yaml_output, yaml_file, default_flow_style=False)
