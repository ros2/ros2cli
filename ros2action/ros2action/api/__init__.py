# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import rclpy.action
from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name
from ros2cli.node.direct import DirectNode
from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_action


def _is_action_status_topic(topic_name, action_name):
    return action_name + '/_action/status' == topic_name


def get_action_clients_and_servers(*, node, action_name):
    action_clients = []
    action_servers = []

    expanded_name = expand_topic_name(action_name, node.get_name(), node.get_namespace())
    validate_full_topic_name(expanded_name)

    node_names_and_ns = node.get_node_names_and_namespaces()
    for node_name, node_ns in node_names_and_ns:
        # Construct fully qualified name
        node_fqn = '/'.join(node_ns) + node_name

        # Get any action clients associated with the node
        client_names_and_types = rclpy.action.get_action_client_names_and_types_by_node(
            node,
            node_name,
            node_ns,
        )
        for client_name, client_types in client_names_and_types:
            if client_name == expanded_name:
                action_clients.append((node_fqn, client_types))

        # Get any action servers associated with the node
        server_names_and_types = rclpy.action.get_action_server_names_and_types_by_node(
            node,
            node_name,
            node_ns,
        )
        for server_name, server_types in server_names_and_types:
            if server_name == expanded_name:
                action_servers.append((node_fqn, server_types))

    return (action_clients, action_servers)


def get_action_names_and_types(*, node):
    return rclpy.action.get_action_names_and_types(node)


def get_action_names(*, node):
    action_names_and_types = get_action_names_and_types(node=node)
    return [n for (n, t) in action_names_and_types]


def action_name_completer(prefix, parsed_args, **kwargs):
    """Callable returning a list of action names."""
    with DirectNode(parsed_args) as node:
        return get_action_names(node=node)


def action_type_completer(**kwargs):
    """Callable returning a list of action types."""
    action_types = []
    for package_name, action_names in get_action_interfaces.items():
        for action_name in action_names:
            action_types.append(f'{package_name}/{action_name}')
    return action_types


class ActionTypeCompleter:
    """Callable returning a list of action types."""

    def __init__(self, *, action_name_key=None):
        self.action_name_key = action_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        if self.action_name_key is None:
            return action_type_completer()

        action_name = getattr(parsed_args, self.action_name_key)
        with DirectNode(parsed_args) as node:
            names_and_types = get_action_names_and_types(node=node)
            for n, t in names_and_types:
                if n == action_name:
                    return t
        return []


class ActionGoalPrototypeCompleter:
    """Callable returning an action goal prototype."""

    def __init__(self, *, action_type_key=None):
        self.action_type_key = action_type_key

    def __call__(self, prefix, parsed_args, **kwargs):
        action = get_action(getattr(parsed_args, self.action_type_key))
        return [message_to_yaml(action.Goal())]
