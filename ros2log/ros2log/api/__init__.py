# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

from collections.abc import Iterator
from collections.abc import Sequence

from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.msg import SetLoggerLevelsResult
from rcl_interfaces.srv import GetLoggerLevels
from rcl_interfaces.srv import SetLoggerLevels
import rclpy

from ros2node.api import get_absolute_node_name
from ros2node.api import get_node_names
from ros2node.api import NodeName


LOGGER_GET_SERVICE_SUFFIX = '/get_logger_levels'
LOGGER_SET_SERVICE_SUFFIX = '/set_logger_levels'
LOGGER_GET_SERVICE_TYPE = 'rcl_interfaces/srv/GetLoggerLevels'
LOGGER_SET_SERVICE_TYPE = 'rcl_interfaces/srv/SetLoggerLevels'


def _require_absolute_node_name(node_name: str) -> str:
    absolute_node_name = get_absolute_node_name(node_name)
    if absolute_node_name is None:
        raise ValueError('node_name must not be empty')
    return absolute_node_name


def get_get_logger_levels_service_name(node_name: str) -> str:
    """Return the get-logger-levels service name for a node."""
    return f'{_require_absolute_node_name(node_name)}{LOGGER_GET_SERVICE_SUFFIX}'


def get_set_logger_levels_service_name(node_name: str) -> str:
    """Return the set-logger-levels service name for a node."""
    return f'{_require_absolute_node_name(node_name)}{LOGGER_SET_SERVICE_SUFFIX}'


def get_logger_name_for_node(node_name: str) -> str:
    """
    Convert a fully qualified node name into its root logger name.

    ROS node logger names use dot-separated namespaces, e.g. `/demo/talker`
    becomes `demo.talker`.
    """
    absolute_node_name = _require_absolute_node_name(node_name)
    return absolute_node_name.lstrip('/').replace('/', '.')


def format_logger_service_unavailable_error(node_name: str) -> str:
    """Return the user-facing error for nodes without logger services."""
    absolute_node_name = _require_absolute_node_name(node_name)
    return (
        f"Logger service not available for node '{absolute_node_name}'.\n"
        "The 'enable_logger_service' node option must be enabled for this node."
    )


def iter_logger_service_nodes(
    *,
    node,
) -> Iterator[NodeName]:
    """Yield nodes that expose the logger get/set services as they are discovered."""
    for node_name in get_node_names(node=node):
        if node_has_logger_services(node, node_name):
            yield node_name


def get_logger_service_nodes(*, node) -> list[NodeName]:
    """Return all nodes that expose the logger get/set services."""
    return list(iter_logger_service_nodes(node=node))


def get_target_node_names(
    *,
    node,
    node_name: str | None = None,
    all_nodes: bool = False,
) -> tuple[list[str] | None, str | None]:
    """Resolve the node names targeted by a get/set command."""
    logger_service_nodes = get_logger_service_nodes(node=node)
    logger_service_node_names = {
        logger_node.full_name for logger_node in logger_service_nodes
    }

    if all_nodes:
        return sorted(logger_service_node_names), None

    absolute_node_name = get_absolute_node_name(node_name)
    all_node_names = {
        discovered_node.full_name for discovered_node in get_node_names(node=node)
    }

    if absolute_node_name not in all_node_names:
        return None, 'Node not found'

    if absolute_node_name not in logger_service_node_names:
        return None, format_logger_service_unavailable_error(absolute_node_name)

    return [absolute_node_name], None


def node_has_logger_services(node, node_name: NodeName) -> bool:
    """Check if a node provides both get/set logger level services."""
    services = node.get_service_names_and_types_by_node(node_name.name, node_name.namespace)
    service_map = dict(services)

    expected_get = get_get_logger_levels_service_name(node_name.full_name)
    expected_set = get_set_logger_levels_service_name(node_name.full_name)

    return (
        _service_has_type(service_map, expected_get, LOGGER_GET_SERVICE_TYPE) and
        _service_has_type(service_map, expected_set, LOGGER_SET_SERVICE_TYPE)
    )


def call_get_logger_levels(
    *,
    node,
    logger_names_by_node: dict[str, Sequence[str]],
    timeout_sec: float = 5.0,
) -> dict[str, list[LoggerLevel] | Exception]:
    """Call the get-logger-levels service for one or more nodes."""
    clients = {}
    futures = {}
    results = {}

    try:
        for node_name, logger_names in logger_names_by_node.items():
            client = node.create_client(
                GetLoggerLevels,
                get_get_logger_levels_service_name(node_name),
            )
            clients[node_name] = client
            if not client.wait_for_service(timeout_sec=timeout_sec):
                results[node_name] = RuntimeError(
                    'Wait for service timed out waiting for logger services '
                    f'for node {node_name}'
                )
                continue

            request = GetLoggerLevels.Request()
            request.names = list(logger_names)
            futures[node_name] = client.call_async(request)

        while futures and not all(future.done() for future in futures.values()):
            rclpy.spin_once(node, timeout_sec=0.1)

        for node_name, future in futures.items():
            if future.result() is not None:
                results[node_name] = list(future.result().levels)
            else:
                results[node_name] = future.exception()

        return results
    finally:
        for client in clients.values():
            node.destroy_client(client)


def call_set_logger_levels(
    *,
    node,
    levels_by_node: dict[str, Sequence[LoggerLevel]],
    timeout_sec: float = 5.0,
) -> dict[str, list[SetLoggerLevelsResult] | Exception]:
    """Call the set-logger-levels service for one or more nodes."""
    clients = {}
    futures = {}
    results = {}

    try:
        for node_name, levels in levels_by_node.items():
            client = node.create_client(
                SetLoggerLevels,
                get_set_logger_levels_service_name(node_name),
            )
            clients[node_name] = client
            if not client.wait_for_service(timeout_sec=timeout_sec):
                results[node_name] = RuntimeError(
                    'Wait for service timed out waiting for logger services '
                    f'for node {node_name}'
                )
                continue

            request = SetLoggerLevels.Request()
            request.levels = list(levels)
            futures[node_name] = client.call_async(request)

        while futures and not all(future.done() for future in futures.values()):
            rclpy.spin_once(node, timeout_sec=0.1)

        for node_name, future in futures.items():
            if future.result() is not None:
                results[node_name] = list(future.result().results)
            else:
                results[node_name] = future.exception()

        return results
    finally:
        for client in clients.values():
            node.destroy_client(client)


def _service_has_type(service_map, service_name: str, service_type: str) -> bool:
    """Check if a service exists and matches the expected type."""
    types = service_map.get(service_name)
    if not types:
        return False
    return service_type in types
