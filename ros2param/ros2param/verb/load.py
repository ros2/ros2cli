# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeNameCompleter
from ros2param.api import load_parameter_dict
from ros2param.verb import VerbExtension

import yaml


class LoadVerb(VerbExtension):
    """Load parameter file for a node."""

    def add_arguments(self, parser, cli_name):  # noqa: D102
        add_arguments(parser)
        arg = parser.add_argument(
            'node_name', help='Name of the ROS node')
        arg.completer = NodeNameCompleter(
            include_hidden_nodes_key='include_hidden_nodes')
        parser.add_argument(
            '--include-hidden-nodes', action='store_true',
            help='Consider hidden nodes as well')
        arg = parser.add_argument(
            'parameter_file', help='Parameter file')
        parser.add_argument(
            '--use-wildcard', action='store_true',
            help='Load parameters in the \'/**\' namespace into the node')

    def main(self, *, args):  # noqa: D102
        with NodeStrategy(args) as node:
            node_names = get_node_names(
                node=node, include_hidden_nodes=args.include_hidden_nodes)

        node_name = get_absolute_node_name(args.node_name)
        if node_name not in {n.full_name for n in node_names}:
            return 'Node not found'
        # Remove leading slash
        node_namespace = node_name[1:]

        with DirectNode(args) as node:
            with open(args.parameter_file, "r") as f:
                param_file = yaml.safe_load(f)
                param_namespaces = []
                if args.use_wildcard and "/**" in param_file:
                    param_namespaces.append("/**")
                if node_namespace in param_file:
                    param_namespaces.append(node_namespace)

                if param_namespaces == []:
                    raise RuntimeError("Param file doesn't contain parameters for {}, "
                                       " only for namespaces: {}" .format(node_namespace,
                                                                          param_file.keys()))

                for ns in param_namespaces:
                    value = param_file[ns]
                    if type(value) != dict or "ros__parameters" not in value:
                        raise RuntimeError("Invalid structure of parameter file in namespace {}"
                                           "expected same format as provided by ros2 param dump"
                                           .format(ns))
                    load_parameter_dict(node=node, node_name=node_name,
                                        parameter_dict=value["ros__parameters"])
