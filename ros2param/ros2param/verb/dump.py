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

from rclpy.parameter import PARAMETER_SEPARATOR_STRING
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2node.api import parse_node_name

from ros2param.api import call_get_parameters
from ros2param.api import call_list_parameters
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
            help='DEPRECATED: The absolute path where to save the generated file')
        parser.add_argument(
            '--print', action='store_true',
            help='DEPRECATED: Does nothing.')

    @staticmethod
    def get_parameter_values(node, node_name, params):
        response = call_get_parameters(
            node=node, node_name=node_name,
            parameter_names=params)

        # requested parameter not set
        if not response.values:
            return '# Parameter not set'

        # extract type specific value
        return [get_value(parameter_value=i) for i in response.values]

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
            yaml_output = {node_name.full_name: {'ros__parameters': {}}}

            # retrieve values
            response = call_list_parameters(node=node, node_name=absolute_node_name)
            response = sorted(response)
            parameter_values = self.get_parameter_values(node, absolute_node_name, response)

            for param_name, pval in zip(response, parameter_values):
                self.insert_dict(
                    yaml_output[node_name.full_name]['ros__parameters'], param_name, pval)

            if args.print:
                print(
                    "WARNING: '--print' is deprecated; this utility prints to stdout by default",
                    file=sys.stderr)

            if args.output_dir != '.':
                print(
                    "WARNING: '--output-dir' is deprecated; use redirection to save to a file",
                    file=sys.stderr)
            else:
                print(yaml.dump(yaml_output, default_flow_style=False))
                return

            if absolute_node_name[0] == '/':
                file_name = absolute_node_name[1:].replace('/', '__')
            else:
                file_name = absolute_node_name.replace('/', '__')

            print('Saving to: ', os.path.join(args.output_dir, file_name + '.yaml'))
            with open(os.path.join(args.output_dir, file_name + '.yaml'), 'w') as yaml_file:
                yaml.dump(yaml_output, yaml_file, default_flow_style=False)
