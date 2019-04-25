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
import subprocess

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource

import composition_interfaces.srv
import rcl_interfaces.msg

import rclpy

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy
from ros2node.api import get_node_names
from ros2node.api import get_service_info
from ros2param.api import get_parameter_value
from ros2pkg.api import get_executable_paths
from ros2pkg.api import PackageNotFound

COMPONENTS_RESOURCE_TYPE = 'rclcpp_components'


def get_package_names_with_component_types():
    """Get the names of all packages that register component types in the ament index."""
    return list(get_resources(COMPONENTS_RESOURCE_TYPE).keys())


def get_package_component_types(*, package_name=None):
    """
    Get all component types registered in the ament index for the given package.

    :param package_name: whose component types are to be retrieved.
    :return: a list of component type names.
    """
    if not has_resource(COMPONENTS_RESOURCE_TYPE, package_name):
        return []
    component_registry, _ = get_resource(COMPONENTS_RESOURCE_TYPE, package_name)
    return [line.split(';')[0] for line in component_registry.splitlines()]


def get_registered_component_types():
    """
    Get all component types registered in the ament index.

    :return: a list of (package name, component type names) tuples.
    """
    return [
        (package_name, get_package_component_types(package_name=package_name))
        for package_name in get_package_names_with_component_types()
    ]


ComponentInfo = namedtuple('Component', ('uid', 'name'))


def get_container_components_info(*, node, remote_container_node_name):
    """
    Get information about the components in a given container.

    :param node: an `rclpy.Node` instance.
    :param remote_container_node_name: of the container node to inspect.
    :return: a list of `ComponentInfo` instances, with the unique ID and
    name for each of the components in the container.
    """
    list_nodes_client = node.create_client(
        composition_interfaces.srv.ListNodes,
        '{}/_container/list_nodes'.format(remote_container_node_name)
    )
    try:
        list_nodes_client.wait_for_service()
        future = list_nodes_client.call_async(
            composition_interfaces.srv.ListNodes.Request()
        )
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
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
    node_namespace=None,
    log_level=None,
    remap_rules=None,
    parameters=None,
    extra_arguments=None
):
    """
    Load component into a running container synchronously.

    :param node: an `rclpy.Node` instance
    :param remote_container_node_name: of the container node to load the component into
    :param package_name: where the component node plugin is to be found
    :param plugin_name: of the component plugin to load
    :param node_name: name for the component node
    :param node_namespace: namespace for the component node
    :param log_level: log level for the component node
    :param remap_rules: remapping rules for the component node, in the 'from:=to' form
    :param parameters: optional parameters for the component node, in the 'name:=value' form
    :param extra_arguments: arguments specific to the container node in the 'name:=value' form
    """
    load_node_client = node.create_client(
        composition_interfaces.srv.LoadNode,
        '{}/_container/load_node'.format(remote_container_node_name)
    )
    try:
        load_node_client.wait_for_service()
        request = composition_interfaces.srv.LoadNode.Request()
        request.package_name = package_name
        request.plugin_name = plugin_name
        if node_name is not None:
            request.node_name = node_name
        if node_namespace is not None:
            request.node_namespace = node_namespace
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
        future = load_node_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if not response.success:
            raise RuntimeError('Failed to load component: ' + response.error_message.capitalize())
        return response.unique_id, response.full_node_name
    finally:
        node.destroy_client(load_node_client)


def unload_component_from_container(*, node, remote_container_node_name, component_uids):
    """
    Unload a component from a running container synchronously.

    :param node: an `rclpy.Node` instance
    :param remote_container_node_name: of the container node to unload the component from
    :param component_uids: list of unique IDs of the components to be unloaded
    """
    unload_node_client = node.create_client(
        composition_interfaces.srv.UnloadNode,
        '{}/_container/unload_node'.format(remote_container_node_name)
    )
    try:
        unload_node_client.wait_for_service()
        for uid in component_uids:
            request = composition_interfaces.srv.UnloadNode.Request()
            request.unique_id = uid
            future = unload_node_client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            response = future.result()
            yield uid, not response.success, response.error_message
    finally:
        node.destroy_client(unload_node_client)


def find_container_node_names(*, node, node_names):
    """
    Identify container nodes from a a list of node names.

    :param node: a `ros2cli.node.DirectNode` instance
    :param node_names: list of `ros2node.api.NodeName` instances, as returned
        by `ros2node.api.get_node_names()`
    :return: list of `ros2node.api.NodeName` instances for nodes that are
        component containers
    """
    container_node_names = []
    for n in node_names:
        services = get_service_info(node=node, remote_node_name=n.full_name)
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


def package_with_components_name_completer(prefix, parsed_args, **kwargs):
    """Callable returning a list of package names with registered components."""
    return get_package_names_with_component_types()


def container_node_name_completer(prefix, parsed_args, **kwargs):
    """Callable returning a list of container node names."""
    with NodeStrategy(parsed_args) as node:
        node_names = get_node_names(node=node)
    with DirectNode(parsed_args) as node:
        return [
            n.full_name for n in find_container_node_names(
                node=node, node_names=node_names
            )
        ]


class ComponentTypeNameCompleter:
    """Callable returning a list of component type names."""

    def __init__(self, *, package_name_key=None):
        self.package_name_key = package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        package_name = getattr(parsed_args, self.package_name_key)
        return get_package_component_types(package_name=package_name)


def add_component_arguments(parser):
    """Add component specific arguments to the CLI parser."""
    argument = parser.add_argument(
        'package_name', help='Name of the package where the component is to be found'
    )
    argument.completer = package_with_components_name_completer
    argument = parser.add_argument(
        'plugin_name', help='Type name of the component to be loaded'
    )
    argument.completer = ComponentTypeNameCompleter(package_name_key='package_name')
    parser.add_argument('-n', '--node-name', default=None, help='Component node name')
    parser.add_argument('--node-namespace', default=None, help='Component node namespace')
    parser.add_argument('--log-level', default=None, help='Component node log level')
    parser.add_argument(
        '-r', '--remap-rule', action='append', default=None, dest='remap_rules',
        help="Component node remapping rules, in the 'from:=to' form"
    )
    parser.add_argument(
        '-p', '--parameter', action='append', default=None, dest='parameters',
        help="Component node parameters, in the 'name:=value' form"
    )
    parser.add_argument(
        '-e', '--extra-argument', action='append', default=None, dest='extra_arguments',
        help="Extra arguments for the container, in the 'name:=value' form"
    )
