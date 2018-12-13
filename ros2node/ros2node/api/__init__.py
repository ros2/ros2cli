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

from collections import namedtuple

from ros2cli.node.strategy import NodeStrategy

# TODO(mikaelarguedas) revisit this once it's specified
HIDDEN_NODE_PREFIX = '_'

NodeName = namedtuple('NodeName', ('name', 'namespace', 'full_name'))
TopicInfo = namedtuple('Topic', ('name', 'types'))


def get_absolute_node_name(node_name):
    if not node_name:
        return None
    if node_name[0] != '/':
        node_name = '/' + node_name
    return node_name


def parse_node_name(full_node_name):
    tokens = full_node_name.split('/')
    if 1 > len(tokens):
        raise RuntimeError('Invalid node name: ' + full_node_name)
    node_name = full_node_name
    namespace = '/'
    if len(tokens) > 1:
        node_name = tokens[-1]
        namespace = '/'.join(tokens[:-1])
    return NodeName(node_name, namespace, full_node_name)


def get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return [
        NodeName(
            name=t[0],
            namespace=t[1],
            full_name=t[1] + ('' if t[1].endswith('/') else '/') + t[0])
        for t in node_names_and_namespaces
        if (
            include_hidden_nodes or
            (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX))
        )
    ]


def get_topics(remote_node_name, func):
    node = parse_node_name(remote_node_name)
    names_and_types = func(node.name, node.namespace)
    return [
        TopicInfo(
            name=t[0],
            types=t[1])
        for t in names_and_types]


def get_subscriber_info(*, node, remote_node_name):
    return get_topics(remote_node_name, node.get_subscriber_names_and_types_by_node)


def get_publisher_info(*, node, remote_node_name):
    return get_topics(remote_node_name, node.get_publisher_names_and_types_by_node)


def get_service_info(*, node, remote_node_name):
    return get_topics(remote_node_name, node.get_service_names_and_types_by_node)


class NodeNameCompleter:
    """Callable returning a list of node names."""

    def __init__(self, *, include_hidden_nodes_key=None):
        self.include_hidden_nodes_key = include_hidden_nodes_key

    def __call__(self, prefix, parsed_args, **kwargs):
        with NodeStrategy(parsed_args) as node:
            return [
                n.full_name for n in get_node_names(
                    node=node,
                    include_hidden_nodes=getattr(
                        parsed_args, self.include_hidden_nodes_key))]
