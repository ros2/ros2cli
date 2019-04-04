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

import os

from ament_index_python import get_resource
from ament_index_python import has_resource

import rclpy.action


def _is_action_status_topic(topic_name, action_name):
    return action_name + '/_action/status' == topic_name


def get_action_clients_and_servers(*, node, action_name):
    action_clients = []
    action_servers = []

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
            if client_name == action_name:
                action_clients.append((node_fqn, client_types))

        # Get any action servers associated with the node
        server_names_and_types = rclpy.action.get_action_server_names_and_types_by_node(
            node,
            node_name,
            node_ns,
        )
        for server_name, server_types in server_names_and_types:
            if server_name == action_name:
                action_servers.append((node_fqn, server_types))

    return (action_clients, action_servers)


def get_action_names_and_types(*, node):
    return rclpy.action.get_action_names_and_types(node)


def get_action_names(*, node):
    action_names_and_types = get_action_names_and_types(node=node)
    return [n for (n, t) in action_names_and_types]


def get_action_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(jacobperron) this logic should come from a rosidl related package
    # Only return actions in action folder
    return list(sorted({
        n[7:-7]
        for n in interface_names
        if n.startswith('action/') and n[-7:] in ('.idl', '.action')}))


def get_action_path(package_name, action_name):
    action_types = get_action_types(package_name)
    if action_name not in action_types:
        raise LookupError('Unknown action type')
    prefix_path = has_resource('packages', package_name)
    # TODO(jacobperron) this logic should come from a rosidl related package
    return os.path.join(prefix_path, 'share', package_name, 'action', action_name + '.action')
