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
from ros2param.api import get_value
from ros2param.verb import VerbExtension

import yaml


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
            '--output-dir',
            default='.',
            help='The absolute path were to save the generated file')
        parser.add_argument(
            '--write', action='store_true',
            help='Save parameters to a file, otherwise print to stdout in terminal.')

    @staticmethod
    def get_parameter_value(node, node_name, param):
        response = call_get_parameters(
            node=node, node_name=node_name,
            parameter_names=[param])

        # requested parameter not set
        if not response.values:
            return '# Parameter not set'

        # extract type specific value
        return get_value(parameter_value=response.values[0])

    def insert_dict(self, dictionary, key, value):
        split = key.split(PARAMETER_SEPARATOR_STRING, 1)
        if len(split) > 1:
            if not split[0] in dictionary:
                dictionary[split[0]] = {}
            self.insert_dict(dictionary[split[0]], split[1], value)
        else:
            dictionary[key] = value

    def main(self, *, args):  # noqa: D102

        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=args.include_hidden_nodes)

        absolute_node_name = get_absolute_node_name(args.node_name)
        node_name = parse_node_name(absolute_node_name)
        if absolute_node_name:
            if absolute_node_name not in [n.full_name for n in node_names]:
                return 'Node not found'

        if not os.path.isdir(args.output_dir):
            raise RuntimeError(
                f"'{args.output_dir}' is not a valid directory.")

        with DirectNode(args) as node:
            # create client
            service_name = f'{absolute_node_name}/list_parameters'
            client = node.create_client(ListParameters, service_name)

            client.wait_for_service()

            if not client.service_is_ready():
                raise RuntimeError(f"Could not reach service '{service_name}'")

            request = ListParameters.Request()
            future = client.call_async(request)

            # wait for response
            rclpy.spin_until_future_complete(node, future)

            yaml_output = {node_name.full_name: {'ros__parameters': {}}}

            # retrieve values
            if future.result() is not None:
                response = future.result()
                for param_name in sorted(response.result.names):
                    pval = self.get_parameter_value(node, absolute_node_name, param_name)
                    self.insert_dict(
                        yaml_output[node_name.full_name]['ros__parameters'], param_name, pval)
            else:
                e = future.exception()
                raise RuntimeError(
                    'Exception while calling service of node '
                    f"'{node_name.full_name}': {e}")

            if not args.write:
                print(yaml.dump(yaml_output, default_flow_style=False))
                return

            if absolute_node_name[0] == '/':
                file_name = absolute_node_name[1:].replace('/', '__')
            else:
                file_name = absolute_node_name.replace('/', '__')

            print('Saving to: ', os.path.join(args.output_dir, file_name + '.yaml'))
            with open(os.path.join(args.output_dir, file_name + '.yaml'), 'w') as yaml_file:
                yaml.dump(yaml_output, yaml_file, default_flow_style=False)
