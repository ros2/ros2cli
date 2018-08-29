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

from ros2cli.node.strategy import NodeStrategy

# TODO(mikaelarguedas) revisit this once it's specified
HIDDEN_NODE_PREFIX = '_'


def get_node_names(*, node, include_hidden_nodes=False):
    node_names = node.get_node_names()
    if not include_hidden_nodes:
        node_names = [
            n for n in node_names
            if n and not n.startswith(HIDDEN_NODE_PREFIX)]
    return node_names

def get_node_namespaced_names(*, node, include_hidden_nodes=False):
    node_names_ns = node.get_node_names_and_namespaces()
    namespaced_nodes = []
    for (node_name, node_ns) in node_names_ns:
        if not include_hidden_nodes and node_name.startswith(HIDDEN_NODE_PREFIX):
            continue
        if node_ns == '/':
            namespaced_nodes.append('/' + node_name)
        else:
            namespaced_nodes.append(node_ns + '/' + node_name)
    return namespaced_nodes

class NodeNameCompleter:
    """Callable returning a list of node names."""

    def __init__(self, *, include_hidden_nodes_key=None):
        self.include_hidden_nodes_key = include_hidden_nodes_key

    def __call__(self, prefix, parsed_args, **kwargs):
        with NodeStrategy(parsed_args) as node:
            return get_node_namespaced_names(
                node=node,
                include_hidden_nodes=getattr(
                    parsed_args, self.include_hidden_nodes_key))
