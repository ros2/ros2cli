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

from collections import namedtuple

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource

import composition_interfaces.srv
import rcl_interfaces.msg

from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import get_service_info
from ros2param.api import get_parameter_value


COMPONENTS_RESOURCE_TYPE = 'rclcpp_components'


def get_package_component_types(*, package_name=None):
    if not has_resource(COMPONENTS_RESOURCE_TYPE, package_name):
        return []
    component_registry, _ = get_resource(COMPONENTS_RESOURCE_TYPE, package_name)
    return [line.split(';')[0] for line in component_registry]


def get_registered_component_types():
    components = {}
    component_registries = get_resources(COMPONENTS_RESOURCE_TYPE)
    for package_name, path_to_component_registry in component_registries.items():
        with open(path_to_component_registry, 'r') as f:
            components[package_name] = [line.split(';')[0] for line in f]
    return components


ComponentInfo = namedtuple('Component', ('uid', 'name'))


def get_container_components_info(*, node, remote_container_node_name):
    list_nodes_client = node.create_client(
        composition_interfaces.srv.ListNodes,
        '{}/_container/list_nodes'.format(remote_container_node_name)
    )
    try:
        response = list_nodes_client.call(
            composition_interfaces.srv.ListNodes.Request()
        )
        return [
            ComponentInfo(uid, name) for uid, name in
            zip(response.unique_ids, response.full_node_names)
        ]
    finally:
        node.destroy_client(list_nodes_client)


def load_component_into_container(
    *,
    node,
    remote_container_node_name,
    package_name,
    plugin_name,
    node_name=None,
    namespace_name=None,
    log_level=None,
    remap_rules=None,
    parameters=None,
    extra_arguments=None
):
    load_node_client = node.create_client(
        composition_interfaces.srv.LoadNode,
        '{}/_container/load_node'.format(remote_container_node_name)
    )
    try:
        request = composition_interfaces.srv.LoadNode.Request()
        request.package_name = package_name
        request.plugin_name = plugin_name
        if node_name is not None:
            request.node_name = node_name
        if namespace_name is not None:
            request.namespace_name = namespace_name
        if log_level is not None:
            request.log_level = log_level
        if remap_rules is not None:
            request.remap_rules = remap_rules
        if parameters is not None:
            for param in parameters:
                name, _, value = param.partition(':=')
                param_msg = rcl_interfaces.msg.Parameter()
                param_msg.value = get_parameter_value(string_value=value)
                param_msg.name = name
                request.parameters.append(param_msg)
        if extra_arguments is not None:
            for arg in extra_arguments:
                name, _, value = arg.partition(':=')
                arg_msg = rcl_interfaces.msg.Parameter()
                arg_msg.value = get_parameter_value(string_value=value)
                arg_msg.name = name
                request.extra_arguments.append(arg_msg)
        response = load_node_client.call(request)
        if not response.status:
            return 'Failed to load component: ' + response.error_message.capitalize()
    finally:
        node.destroy_client(load_node_client)


def unload_component_from_container(*, node, remote_container_node_name, component_uids):
    """."""
    unload_node_client = node.create_client(
        composition_interfaces.srv.UnloadNode,
        '{}/_container/unload_node'.format(remote_container_node_name)
    )
    try:
        for uid in component_uids:
            request = composition_interfaces.srv.UnloadNode.Request()
            request.unique_id = uid
            response = unload_node_client.call(request)
            if not response.success:
                print('Failed to unload component {} from {} container: {}'.format(
                    uid, remote_container_node_name, response.error_message
                ))
            else:
                print('Unloaded component {} from {} container'.format(
                    uid, remote_container_node_name
                ))
    finally:
        node.destroy_client(unload_node_client)


def find_container_node_names(*, node, node_names):
    container_node_names = []
    for n in node_names:
        services = get_service_info(node=node, remote_node_name=n.full_name)
        print(services)
        if not any(s.name.endswith('_container/load_node') and
                   'composition_interfaces/LoadNode' in s.types
                   for s in services):
            continue
        if not any(s.name.endswith('_container/unload_node') and
                   'composition_interfaces/UnloadNode' in s.types
                   for s in services):
            continue
        if not any(s.name.endswith('_container/list_nodes') and
                   'composition_interfaces/ListNodes' in s.types
                   for s in services):
            continue
        container_node_names.append(n)
    return container_node_names


def container_node_name_completer(prefix, parsed_args, **kwargs):
    """Callable returning a list of container node names."""
    with NodeStrategy(parsed_args) as node:
        return [
            n.full_name for n in get_container_node_names(node=node)
        ]


class ComponentTypeNameCompleter:
    """Callable returning a list of component type names."""

    def __init__(self, *, package_name_key=None):
        self.package_name_key = package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        package_name = getattr(parsed_args, self.package_name_key)
        return get_package_component_types(package_name=package_name)
