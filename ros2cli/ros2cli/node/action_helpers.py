# Copyright 2025 Open Source Robotics Foundation, Inc.
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


from typing import Union

from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name
from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy


Ros2CliNodes = Union[DirectNode, NodeStrategy]
ActionNameAndTypeInfo = list[tuple[str, list[str]]]


# Originally lived in ros2action but lifted so can be used in other cli tools e.g. ros2doctor
def get_action_clients_and_servers(*, node: Ros2CliNodes,
                                   action_name: str
                                   ) -> tuple[ActionNameAndTypeInfo, ActionNameAndTypeInfo]:
    action_clients: ActionNameAndTypeInfo = []
    action_servers: ActionNameAndTypeInfo = []

    expanded_name = expand_topic_name(action_name, node.get_name(), node.get_namespace())
    validate_full_topic_name(expanded_name)

    node_names_and_ns = node.get_node_names_and_namespaces()
    for node_name, node_ns in node_names_and_ns:
        # Construct fully qualified name
        node_fqn = (node_ns.rstrip('/') + '/' + node_name.lstrip('/')) if node_ns else node_name

        # Get any action clients associated with the node
        client_names_and_types = node.get_action_client_names_and_types_by_node(
            node_name,
            node_ns,
        )
        for client_name, client_types in client_names_and_types:
            if client_name == expanded_name:
                action_clients.append((node_fqn, client_types))

        # Get any action servers associated with the node
        server_names_and_types = node.get_action_server_names_and_types_by_node(
            node_name,
            node_ns,
        )
        for server_name, server_types in server_names_and_types:
            if server_name == expanded_name:
                action_servers.append((node_fqn, server_types))

    return (action_clients, action_servers)

def get_action_names_and_types(*, node: Ros2CliNodes) -> ActionNameAndTypeInfo:
    return node.get_action_names_and_types()


def get_action_names(*, node: Ros2CliNodes) -> list[str]:
    action_names_and_types = get_action_names_and_types(node=node)
    return [n for (n, _) in action_names_and_types]